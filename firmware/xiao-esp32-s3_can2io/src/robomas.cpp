/*====================================================================
<robomas.cpp>
・MODE_ROBOMAS(DJI RoboMasterシリーズ CANドライバ)の実装ファイル

config.hppのROBOMAS_MOTOR_TYPEで選択した機種(M3508/M2006/GM6020)を、
同一バス上に最大NUM_MOTOR(4)台まで速度制御する。他機種との混在は非対応。

スロット割り当て (frame_data.hppのTx_16Data/Rx_16Dataを使用、独立デバイスとして
24スロットをそのまま使う。ノード/スロット分配は行わない):

  Rx_16Data (PC -> 本機, 指令):
    0-3: target_rpm (モータ1-4、生のrpm値、スケール無し)
    4-23: 未使用

  Tx_16Data (本機 -> PC, 帰還):
    0-3: angle  [0.1deg単位] (出力軸換算、M3508/M2006はギア比込み)
    4-7: velocity [rpm]      (出力軸換算)
    8-11: current [mA単位]   (実電流換算値)
    12-23: 未使用

Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "robomas.hpp"
#include "config.hpp"
#include "defs.hpp"
#include "frame_data.hpp"
#include "PID.hpp"
#include <Arduino.h>

namespace {

// -------- 状態量 (CAN受信フィードバック) -------- //
int16_t encoder_count[NUM_MOTOR] = {0};
int16_t rpm[NUM_MOTOR] = {0};
int16_t current_raw[NUM_MOTOR] = {0};
int last_encoder[NUM_MOTOR] = {0};
int rotation_count[NUM_MOTOR] = {0};
long total_encoder[NUM_MOTOR] = {0};
float angle[NUM_MOTOR] = {0};   // 出力軸角度[deg]
float vel[NUM_MOTOR] = {0};     // 出力軸速度[rpm]
float current_a[NUM_MOTOR] = {0}; // 実電流[A]

// -------- 指令値 -------- //
float target_rpm[NUM_MOTOR] = {0};

// -------- 速度PID状態 -------- //
float motor_output_current[NUM_MOTOR] = {0};

unsigned long g_last_pid_time = 0;

PIDController vel_pid[NUM_MOTOR] = {
    PIDController(ROBOMAS_KP_VEL, ROBOMAS_KI_VEL, ROBOMAS_KD_VEL, ROBOMAS_MAX_CURRENT_A / ROBOMAS_OUTPUT_GAIN),
    PIDController(ROBOMAS_KP_VEL, ROBOMAS_KI_VEL, ROBOMAS_KD_VEL, ROBOMAS_MAX_CURRENT_A / ROBOMAS_OUTPUT_GAIN),
    PIDController(ROBOMAS_KP_VEL, ROBOMAS_KI_VEL, ROBOMAS_KD_VEL, ROBOMAS_MAX_CURRENT_A / ROBOMAS_OUTPUT_GAIN),
    PIDController(ROBOMAS_KP_VEL, ROBOMAS_KI_VEL, ROBOMAS_KD_VEL, ROBOMAS_MAX_CURRENT_A / ROBOMAS_OUTPUT_GAIN),
};

float constrainFloat(float val, float min_val, float max_val) {
    if (val < min_val)
        return min_val;
    if (val > max_val)
        return max_val;
    return val;
}

// -------- CAN送信 (指令 -> ESC) -------- //

void sendCurC620(const float cur_array[NUM_MOTOR]) {
    // C620(M3508)/C610(M2006)共通、ID1-4宛の一括電流指令フレーム
    twai_message_t tx{};
    tx.identifier = 0x200;
    tx.extd = 0;
    tx.rtr = 0;
    tx.data_length_code = 8;

    for (int i = 0; i < NUM_MOTOR; i++) {
#if ROBOMAS_MOTOR_TYPE == ROBOMAS_MOTOR_M3508
        constexpr float MAX_CUR = 20.0f;
        constexpr int16_t MAX_CUR_VAL = 16384;
#else // ROBOMAS_MOTOR_M2006
        constexpr float MAX_CUR = 10.0f;
        constexpr int16_t MAX_CUR_VAL = 10000;
#endif
        float amp = constrainFloat(cur_array[i], -MAX_CUR, MAX_CUR);
        int16_t val = static_cast<int16_t>(amp * (MAX_CUR_VAL / MAX_CUR));

        tx.data[i * 2] = (uint8_t)(val >> 8);
        tx.data[i * 2 + 1] = (uint8_t)(val & 0xFF);
    }

    if (twai_transmit(&tx, pdMS_TO_TICKS(20)) != ESP_OK) {
        Serial.println("[ERR] robomas: twai_transmit(0x200) failed");
    }
}

void sendCurGm6020(const float cur_array[NUM_MOTOR]) {
    twai_message_t tx{};
    tx.identifier = 0x1FE;
    tx.extd = 0;
    tx.rtr = 0;
    tx.data_length_code = 8;

    constexpr float MAX_CUR = 3.0f;
    constexpr int16_t MAX_CUR_VAL = 16384;

    for (int i = 0; i < NUM_MOTOR; i++) {
        float amp = constrainFloat(cur_array[i], -MAX_CUR, MAX_CUR);
        int16_t val = static_cast<int16_t>(amp * (MAX_CUR_VAL / MAX_CUR));

        tx.data[i * 2] = (uint8_t)(val >> 8);
        tx.data[i * 2 + 1] = (uint8_t)(val & 0xFF);
    }

    if (twai_transmit(&tx, pdMS_TO_TICKS(20)) != ESP_OK) {
        Serial.println("[ERR] robomas: twai_transmit(0x1FE) failed");
    }
}

void sendCurrentCommand(const float cur_array[NUM_MOTOR]) {
#if ROBOMAS_MOTOR_TYPE == ROBOMAS_MOTOR_GM6020
    sendCurGm6020(cur_array);
#else
    sendCurC620(cur_array);
#endif
}

// -------- CAN受信 (ESC -> 帰還) -------- //

void receiveFeedback() {
    twai_message_t rx_msg;

    while (twai_receive(&rx_msg, 0) == ESP_OK) {
        if (rx_msg.data_length_code != 8)
            continue;

#if ROBOMAS_MOTOR_TYPE == ROBOMAS_MOTOR_GM6020
        // GM6020: ID1-4 の帰還フレームは 0x205-0x208
        if (rx_msg.identifier < 0x205 || rx_msg.identifier > 0x208)
            continue;
        int m = rx_msg.identifier - 0x205;
#else
        // C620/C610: ID1-4 の帰還フレームは 0x201-0x204
        if (rx_msg.identifier < 0x201 || rx_msg.identifier > 0x204)
            continue;
        int m = rx_msg.identifier - 0x201;
#endif
        if (m < 0 || m >= NUM_MOTOR)
            continue;

        encoder_count[m] = (int16_t)(rx_msg.data[0] << 8 | rx_msg.data[1]);
        rpm[m] = (int16_t)(rx_msg.data[2] << 8 | rx_msg.data[3]);
        current_raw[m] = (int16_t)(rx_msg.data[4] << 8 | rx_msg.data[5]);

        // エンコーダ回転数計算 (周回検出)
        int diff = encoder_count[m] - last_encoder[m];
        if (diff > HALF_ENCODER)
            rotation_count[m]--;
        else if (diff < -HALF_ENCODER)
            rotation_count[m]++;
        last_encoder[m] = encoder_count[m];

        total_encoder[m] = rotation_count[m] * (long)ENCODER_MAX + encoder_count[m];

#if ROBOMAS_MOTOR_TYPE == ROBOMAS_MOTOR_M3508
        angle[m] = total_encoder[m] * (360.0f / (ENCODER_MAX * gear_m3508));
        vel[m] = rpm[m] / gear_m3508;
        current_a[m] = current_raw[m] * 20.0f / 16384.0f;
#elif ROBOMAS_MOTOR_TYPE == ROBOMAS_MOTOR_M2006
        angle[m] = total_encoder[m] * (360.0f / (ENCODER_MAX * gear_m2006));
        vel[m] = rpm[m] / gear_m2006;
        current_a[m] = current_raw[m] * 10.0f / 10000.0f;
#elif ROBOMAS_MOTOR_TYPE == ROBOMAS_MOTOR_GM6020
        angle[m] = total_encoder[m] * (360.0f / ENCODER_MAX); // ダイレクトドライブ、ギア無し
        vel[m] = (float)rpm[m];
        current_a[m] = current_raw[m] * 3.0f / 16384.0f;
#endif
    }
}

} // namespace

void robomasInit() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS(); // DJI RoboMasterシリーズは1Mbps固定
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("[ERR] robomas: TWAI install failed");
        while (1)
            ;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("[ERR] robomas: TWAI start failed");
        while (1)
            ;
    }

    g_last_pid_time = micros();

    for (int i = 0; i < NUM_MOTOR; i++) {
        vel_pid[i].reset();
    }
}

void robomasTask(void *pvParameters) {
    while (1) {
        for (int i = 0; i < NUM_MOTOR; i++) {
            target_rpm[i] = Rx_16Data[i];
        }

        // ループ周期は約5ms(200Hz、CubeMars/センサノードと1Mbpsバスを共有するため
        // 指令送信頻度を落としてある)。millis()では量子化誤差がdtに対して無視でき
        // ない比率になるため、引き続きmicros()で測る。
        unsigned long now = micros();
        float dt = (unsigned long)(now - g_last_pid_time) / 1000000.0f;
        if (dt <= 0)
            dt = 0.000001f;
        if (dt > 0.02f)
            dt = 0.02f;
        g_last_pid_time = now;

        receiveFeedback();

        for (int i = 0; i < NUM_MOTOR; i++) {
            vel_pid[i].set_target(target_rpm[i]);
            float vel_out = vel_pid[i].update(vel[i], dt);
            motor_output_current[i] = constrainFloat(vel_out * ROBOMAS_OUTPUT_GAIN, -ROBOMAS_MAX_CURRENT_A, ROBOMAS_MAX_CURRENT_A);
        }

        sendCurrentCommand(motor_output_current);

        for (int i = 0; i < NUM_MOTOR; i++) {
            // 0.1deg単位。int16なので±3276.7deg(約±9回転)で飽和する点に注意。
            Tx_16Data[i] = (int16_t)(angle[i] * 10.0f);
            Tx_16Data[4 + i] = (int16_t)vel[i];
            Tx_16Data[8 + i] = (int16_t)(current_a[i] * 1000.0f);
        }

        // 200Hz。CubeMars/センサノードと1Mbpsバスを共有する構成での帯域見積りは
        // README.md参照(1kHzのままだと合計がバス容量を超える)。
        vTaskDelay(5);
    }
}
