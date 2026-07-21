/*====================================================================
<robomas.cpp>
・ロボマス関連の実装ファイル
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "robomas.hpp"
#include "frame_data.hpp"
// #include "pid_task.hpp"
#include "pin_ctrl_init.hpp"
#include <Arduino.h>
#include <defs.hpp>

// -------- 状態量 / CAN受信関連 -------- //
int encoder_count[NUM_MOTOR] = {0};  // エンコーダ値
int rpm[NUM_MOTOR] = {0};            // 回転速度
int current[NUM_MOTOR] = {0};        // 電流値
bool offset_ok[NUM_MOTOR] = {false}; // オフセット完了フラグ
int encoder_offset[NUM_MOTOR] = {0}; // エンコーダオフセット
int last_encoder[NUM_MOTOR];         // 前回エンコーダ値
int rotation_count[NUM_MOTOR] = {0}; // 回転数
long total_encoder[NUM_MOTOR] = {0}; // 累積エンコーダ値
float angle[NUM_MOTOR] = {0};        // 角度
float vel[NUM_MOTOR] = {0};          // 速度

// -------- PID関連変数 -------- //
float target_angle[NUM_MOTOR] = {0};   // 目標角度
float pos_error_prev[NUM_MOTOR] = {0}; // 前回角度誤差
float cur_error_prev[NUM_MOTOR] = {0}; // 前回角度誤差
float pos_integral[NUM_MOTOR] = {0};   // 角度積分項
float vel_integral[NUM_MOTOR] = {0};
float cur_integral[NUM_MOTOR] = {0};
float pos_output[NUM_MOTOR] = {0};           // PID出力
float cur_output[NUM_MOTOR] = {0};           // PID出力
float motor_output_current[NUM_MOTOR] = {0}; // 出力電流
float output[NUM_MOTOR] = {0};

// 速度PID
float target_rpm[NUM_MOTOR] = {0};     // 目標速度
float vel_error_prev[NUM_MOTOR] = {0}; // 前回速度誤差
float vel_prop_prev[NUM_MOTOR] = {0};  // 速度比例項
float vel_output[NUM_MOTOR] = {0};     // 速度PID出力
float vel_out[NUM_MOTOR] = {0};        // 最終速度出力

// -------- 状態量 / CAN受信関連 -------- //
float angle_m3508[NUM_MOTOR] = {0};  // 角度
float angle_m2006[NUM_MOTOR] = {0};  // 角度
float angle_gm6020[NUM_MOTOR] = {0}; // 角度
float vel_m3508[NUM_MOTOR] = {0};    // 速度
float vel_m2006[NUM_MOTOR] = {0};    // 速度
float vel_gm6020[NUM_MOTOR] = {0};   // 速度
float c_m3508[NUM_MOTOR] = {0};      //
float c_m2006[NUM_MOTOR] = {0};      //
float c_gm6020[NUM_MOTOR] = {0};     //

unsigned long lastPidTime = 0; // PID制御用タイマー

// -------- PIDゲイン -------- //
float kp_pos = 0.8f;  // 角度比例ゲイン
float ki_pos = 0.01f; // 角度積分ゲイン
float kd_pos = 0.02f; // 角度微分ゲイン

float kp_pos_gm6020 = 0.03;
float ki_pos_gm6020 = 0.0;  // 目標付近での収束を早めるため追加（要チューニング）
float kd_pos_gm6020 = 0.00; // 微分は控えめに

// -------- 速度PIDゲイン -------- //
float kp_vel_m3508 = 0.8;
float ki_vel_m3508 = 0.0;
float kd_vel_m3508 = 0.05; // 微分は控えめに

float kp_vel_m2006 = 0.8;
float ki_vel_m2006 = 0.0;
float kd_vel_m2006 = 0.02; // 微分は控えめに

// -------- 電流PIDゲイン -------- //
float kp_cur = 0.01;
float ki_cur = 0.0;
float kd_cur = 0.0; // 微分は控えめに

float constrain_double(float val, float min_val, float max_val) {
    if (val < min_val)
        return min_val;
    if (val > max_val)
        return max_val;
    return val;
}

void send_cur_all(float cur_array[NUM_MOTOR]) {
    twai_message_t tx;       // 送信用メッセージ
    tx.identifier = 0x200;   // CAN ID
    tx.extd = 0;             // 標準フレーム
    tx.rtr = 0;              // データフレーム
    tx.data_length_code = 8; // 8バイト

    // C620 の仕様: -16384 ～ +16384
    for (int i = 0; i < NUM_MOTOR; i++) {
        float amp = constrain_double(cur_array[i], -20, 20);
        int16_t val = amp * (16384.0f / 200.0f);

        tx.data[i * 2] = (val >> 8) & 0xFF;
        tx.data[i * 2 + 1] = val & 0xFF;
    }

    if (twai_transmit(&tx, pdMS_TO_TICKS(20)) != ESP_OK) {
        Serial.println("[ERR] twai_transmit failed");
    }
}

void send_cur_c610(float cur_array[NUM_MOTOR]) {
    twai_message_t tx;       // 送信用メッセージ
    tx.identifier = 0x200;   // CAN ID
    tx.extd = 0;             // 標準フレーム
    tx.rtr = 0;              // データフレーム
    tx.data_length_code = 8; // 8バイト

    // C610 の仕様: 電流指令はおおむね -10000 ～ +10000（±10A相当）
    constexpr float MAX_CUR = 10.0f;
    constexpr int16_t MAX_CUR_VAL = 10000;

    for (int i = 0; i < NUM_MOTOR; i++) {
        float amp = constrain_double(cur_array[i], -MAX_CUR, MAX_CUR);
        int16_t val = static_cast<int16_t>(amp * (MAX_CUR_VAL / MAX_CUR));

        tx.data[i * 2] = (val >> 8) & 0xFF;
        tx.data[i * 2 + 1] = val & 0xFF;
    }

    if (twai_transmit(&tx, pdMS_TO_TICKS(20)) != ESP_OK) {
        Serial.println("[ERR] twai_transmit failed");
    }
}

void send_cur_gm6020(float cur_array[NUM_MOTOR]) {
    twai_message_t tx;       // 送信用メッセージ
    tx.identifier = 0x1FE;   // CAN ID
    tx.extd = 0;             // 標準フレーム
    tx.rtr = 0;              // データフレーム
    tx.data_length_code = 8; // 8バイト

    // GM6020 の仕様: 電流指令はおおむね -16384 ～ +16384（±20A相当）
    constexpr float MAX_CUR = 3.0f;
    constexpr int16_t MAX_CUR_VAL = 16384;

    for (int i = 0; i < NUM_MOTOR; i++) {
        float amp = constrain_double(cur_array[i], -MAX_CUR, MAX_CUR);
        int16_t val = static_cast<int16_t>(amp * (MAX_CUR_VAL / MAX_CUR));

        tx.data[i * 2] = (val >> 8) & 0xFF;
        tx.data[i * 2 + 1] = val & 0xFF;
    }

    if (twai_transmit(&tx, pdMS_TO_TICKS(20)) != ESP_OK) {
        Serial.println("[ERR] twai_transmit failed");
    }
}

float pid(float setpoint, float input, float &error_prev, float &integral,
          float kp, float ki, float kd, float dt) {
    float error = setpoint - input;
    integral += ((error + error_prev) * dt / 2.0f); // 台形積分
    float derivative = (error - error_prev) / dt;
    error_prev = error;
    return kp * error + ki * integral + kd * derivative;
}

float pid_vel(float setpoint, float input, float &error_prev, float &prop_prev, float &output,
              float kp, float ki, float kd, float dt) {
    float error = setpoint - input;
    float prop = error - error_prev;
    float deriv = prop - prop_prev;
    float du = kp * prop + ki * error * dt + kd * deriv;
    output += du;

    prop_prev = prop;
    error_prev = error;

    return output;
}

// M3508制御タスク
void M3508_Task(void *pvParameters) {

    // 初期化
    lastPidTime = millis();

    while (1) {
        for (int i = 0; i < NUM_MOTOR; i++) {
            // 2026/02/14, 7,8,9,10から5,6,7,8に変更
            target_rpm[i] = Rx_16Data[i + 5];
        }
        unsigned long now = millis();
        float dt = (now - lastPidTime) / 1000.0f;
        if (dt <= 0)
            dt = 0.000001f;
        if (dt > 0.02f)
            dt = 0.02f;
        lastPidTime = now;

        // TWAI受信処理
        twai_receive_feedback();

        // // 5秒後にターゲットRPM増加
        for (int i = 0; i < NUM_MOTOR; i++) {

            vel_out[i] = pid_vel(target_rpm[i], vel_m3508[i], vel_error_prev[i], vel_prop_prev[i], vel_output[i], kp_vel_m3508, ki_vel_m3508, kd_vel_m3508, dt);

            motor_output_current[i] = constrain_double(vel_out[i] * 10, -20, 20);
        }

        // 送信
        send_cur_all(motor_output_current);

        // debug

        Tx_16Data[7] = static_cast<int16_t>(angle_m3508[0]);
        Tx_16Data[8] = static_cast<int16_t>(angle_m3508[1]);
        Tx_16Data[9] = static_cast<int16_t>(angle_m3508[2]);
        Tx_16Data[10] = static_cast<int16_t>(angle_m3508[3]);

        Tx_16Data[11] = static_cast<int16_t>(vel_m3508[0]);
        Tx_16Data[12] = static_cast<int16_t>(vel_m3508[1]);
        Tx_16Data[13] = static_cast<int16_t>(vel_m3508[2]);
        Tx_16Data[14] = static_cast<int16_t>(vel_m3508[3]);

        // Serial.print(vel_m3508[0]);
        // Serial.print("\t");
        // Serial.println(current[0]);

        vTaskDelay(1);
    }
}

void M2006_Task(void *pvParameters) {
    // 初期化
    lastPidTime = millis();

    while (1) {

        for (int i = 0; i < NUM_MOTOR; i++) {
            // 2026/02/14, 7,8,9,10から5,6,7,8に変更
            target_rpm[i] = Rx_16Data[i + 5];
        }
        unsigned long now = millis();
        float dt = (now - lastPidTime) / 1000.0f;
        if (dt <= 0)
            dt = 0.000001f;
        if (dt > 0.02f)
            dt = 0.02f;
        lastPidTime = now;
        // TWAI受信処理
        twai_receive_feedback();
        for (int i = 0; i < NUM_MOTOR; i++) {
            c_m2006[i] = pid_vel(target_rpm[i], vel_m2006[i], vel_error_prev[i], vel_prop_prev[i], vel_output[i], kp_vel_m2006, ki_vel_m2006, kd_vel_m2006, dt);

            motor_output_current[i] = constrain_double(c_m2006[i], -1, 1);
        }

        // 送信
        send_cur_c610(motor_output_current);

        // debug

        // 場所がないのでMDかぶらせてる
        Tx_16Data[11] = static_cast<int16_t>(angle_m2006[0]);
        Tx_16Data[12] = static_cast<int16_t>(angle_m2006[1]);
        Tx_16Data[13] = static_cast<int16_t>(angle_m2006[2]);
        Tx_16Data[14] = static_cast<int16_t>(angle_m2006[3]);

        Tx_16Data[15] = static_cast<int16_t>(vel_m2006[0]);
        Tx_16Data[16] = static_cast<int16_t>(vel_m2006[1]);
        Tx_16Data[17] = static_cast<int16_t>(vel_m2006[2]);
        Tx_16Data[18] = static_cast<int16_t>(vel_m2006[3]);

        // Tx_16Data[15] = static_cast<int16_t>(c_m2006[2]);
        // Serial.print(vel_m2006[0]);
        // Serial.print("\t");
        // Serial.println(current[0]);

        vTaskDelay(1);
    }
}

void gm6020_Task(void *pvParameters) {
    // 初期化
    lastPidTime = millis();

    while (1) {

        for (int i = 0; i < NUM_MOTOR; i++) {
            // 2026/02/14, 7,8,9,10から5,6,7,8に変更
            target_angle[i] = Rx_16Data[i];
        }
        unsigned long now = millis();
        float dt = (now - lastPidTime) / 1000.0f;
        if (dt <= 0)
            dt = 0.000001f;
        if (dt > 0.02f)
            dt = 0.02f;
        lastPidTime = now;
        // TWAI受信処理
        twai_receive_feedback_gm6020();
        for (int i = 0; i < NUM_MOTOR; i++) {
            c_gm6020[i] = pid(target_angle[i], angle_gm6020[i], pos_error_prev[i], pos_integral[i], kp_pos_gm6020, ki_pos_gm6020, kd_pos_gm6020, dt);
            motor_output_current[i] = constrain_double(c_gm6020[i], -10, 10);
        }

        // 送信
        send_cur_gm6020(motor_output_current);

        // debug

        // 場所がないのでMDかぶらせてる
        Tx_16Data[11] = static_cast<int16_t>(angle_gm6020[0]);
        Tx_16Data[12] = static_cast<int16_t>(angle_gm6020[1]);
        Tx_16Data[13] = static_cast<int16_t>(angle_gm6020[2]);
        Tx_16Data[14] = static_cast<int16_t>(angle_gm6020[3]);

        Tx_16Data[15] = static_cast<int16_t>(vel_gm6020[0]);
        Tx_16Data[16] = static_cast<int16_t>(vel_gm6020[1]);
        Tx_16Data[17] = static_cast<int16_t>(vel_gm6020[2]);
        Tx_16Data[18] = static_cast<int16_t>(vel_gm6020[3]);

        Tx_16Data[19] = static_cast<int16_t>(c_gm6020[1]);
        Tx_16Data[20] = static_cast<int16_t>(motor_output_current[1] * 100);
        Tx_16Data[21] = static_cast<int16_t>(encoder_count[1]);

        vTaskDelay(1);
    }
}

void twai_receive_feedback() {
    twai_message_t rx_msg;

    while (twai_receive(&rx_msg, 0) == ESP_OK) {
        if (rx_msg.data_length_code != 8)
            continue;
        if (rx_msg.identifier < 0x201 || rx_msg.identifier > 0x204)
            continue;

        int m = rx_msg.identifier - 0x201;

        encoder_count[m] = (int16_t)(rx_msg.data[0] << 8 | rx_msg.data[1]);
        rpm[m] = (int16_t)(rx_msg.data[2] << 8 | rx_msg.data[3]);
        current[m] = (int16_t)(rx_msg.data[4] << 8 | rx_msg.data[5]);

        // エンコーダ回転数計算
        int diff = encoder_count[m] - last_encoder[m];
        if (diff > HALF_ENCODER)
            rotation_count[m]--;
        else if (diff < -HALF_ENCODER)
            rotation_count[m]++;

        last_encoder[m] = encoder_count[m];

        total_encoder[m] =
            rotation_count[m] * ENCODER_MAX + encoder_count[m];

        angle_m3508[m] = total_encoder[m] * (360.0f / (ENCODER_MAX * gear_m3508));
        vel_m3508[m] = rpm[m] / gear_m3508;
        c_m3508[m] = current[m] * 20.0f / 16384.0f;

        angle_m2006[m] = total_encoder[m] * (360.0f / (ENCODER_MAX * gear_m2006));
        vel_m2006[m] = rpm[m] / gear_m2006;
        c_m2006[m] = current[m] * 10.0f / 10000.0f;
    }
}

void twai_receive_feedback_gm6020() {
    twai_message_t rx_msg;

    while (twai_receive(&rx_msg, 0) == ESP_OK) {
        if (rx_msg.data_length_code != 8)
            continue;
        if (rx_msg.identifier < 0x205 || rx_msg.identifier > 0x207)
            continue;

        int m = rx_msg.identifier - 0x205;

        encoder_count[m] = (int16_t)(rx_msg.data[0] << 8 | rx_msg.data[1]);
        rpm[m] = (int16_t)(rx_msg.data[2] << 8 | rx_msg.data[3]);
        current[m] = (int16_t)(rx_msg.data[4] << 8 | rx_msg.data[5]);

        // エンコーダ回転数計算
        int diff = encoder_count[m] - last_encoder[m];
        if (diff > HALF_ENCODER)
            rotation_count[m]--;
        else if (diff < -HALF_ENCODER)
            rotation_count[m]++;

        last_encoder[m] = encoder_count[m];

        total_encoder[m] =
            rotation_count[m] * ENCODER_MAX + encoder_count[m];

        angle_gm6020[m] = total_encoder[m] * (360.0f / 8192.0f); // GM6020のエンコーダ分解能は8192
        vel_gm6020[m] = rpm[m];
        c_gm6020[m] = current[m]; //* 3.0f / 16384.0f;
    }
}

void robomas_init() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        // Serial.println("TWAI install failed");
        while (1)
            ;
    }
    if (twai_start() != ESP_OK) {
        // Serial.println("TWAI start failed");
        while (1)
            ;
    }
}

// M3508からのCAN受信タスク
// void M3508_RX(void *)
// {
//     while (true)
//     {
//         for (int i = 0; i < NUM_MOTOR; i++)
//         {
//             target_rpm[i] = received_data[i + 1];
//         }
//         unsigned long now = millis();
//         float dt = (now - lastPidTime) / 1000.0f;
//         if (dt <= 0)
//             dt = 0.000001f;
//         if (dt > 0.02f)
//             dt = 0.02f;
//         lastPidTime = now;

//         // TWAI受信処理
//         twai_receive_feedback();

//         // // 5秒後にターゲットRPM増加
//         for (int i = 0; i < NUM_MOTOR; i++)
//         {
//             //     static float rpm_step[NUM_MOTOR] = {100, 100, 100, 100};

//             //     if (now < 5000)
//             //     {
//             //         target_rpm[i] = 0;
//             //     }
//             //     else
//             //     {
//             //         if (rpm_step[i] < 300)
//             //             rpm_step[i] += 0.1;
//             //         target_rpm[i] = rpm_step[i];
//             //     }

//             vel_out[i] = pid_vel(target_rpm[i], vel_m3508[i], vel_error_prev[i], vel_prop_prev[i], vel_output[i], kp_vel, ki_vel, kd_vel, dt);

//             motor_output_current[i] = constrain_double(vel_out[i] * 10, -20, 20);
//         }

//         // 送信
//         send_cur_all(motor_output_current);

//         // debug
//         // Serial.print(vel_m3508[0]);
//         // Serial.print("\t");
//         // Serial.println(current[0]);

//         vTaskDelay(1);
//     }
// }

/*====================================================================

                robomas.cpp 処理全体フロー

        ┌─────────────────────────────┐
        │      CAN 受信  (M3508_RX)    　|
        │-----------------------------│
        │  ・CAN ID: 0x201～0x204      │
        │  ・encoder                  │
        │  ・rpm                      │
        │  ・current                  │
        └─────────────┬──────────────┘
                      │
                      ▼
        ┌─────────────────────────────┐
        │    状態量 更新   (M3508_RX)  │
        │-----------------------------│
        │ ・encoder_count[]           │
        │ ・rpm[]                     │
        │ ・current[]                 │
        │ ・rotation_count[]          │
        │ ・total_encoder[]           │
        │ ・angle[]                   │
        │ ・vel[]                     │
        └─────────────┬──────────────┘
                      │
                      ▼
        ┌─────────────────────────────┐
        │  制御計算 (PID) (M3508_Task) │
        │-----------------------------│
        │ ・目標角度 target_angle[]  │
        │ ・角度PID (外側)           │
        │ ・速度PID (中間)           │
        │ ・電流PID (内側 or 省略)    │
        └─────────────┬─────────────┘
                      │
                      ▼
        ┌─────────────────────────────┐
        │     CAN 送信   (send_cur) │
        │-----------------------------│
        │ ・CAN ID: 0x200             │
        │ ・motor_output_current[]   │
        └─────────────────────────────┘

====================================================================*/
