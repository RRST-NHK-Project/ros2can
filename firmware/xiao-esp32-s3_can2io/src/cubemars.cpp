/*====================================================================
<cubemars.cpp>
・MODE_CUBEMARS(CubeMars AKシリーズ Servo(CAN)モード ドライバ)の実装ファイル

CubeMars AK Series Module Product Manual V3.2.0 4.1節の Servo(CAN)モードプロトコル
(拡張CAN ID = 制御モードID(bit28-8) + ドライバID(bit7-0)、1Mbps固定)を直接喋る。
AK40-10等のAKシリーズはアクチュエータ内蔵のクローズドループ(位置/速度)が指令に
そのまま追従するため、ロボマスのGM6020のようなホスト側PID(PID.hpp)は不要。

スロット割り当て (frame_data.hppのTx_16Data/Rx_16Dataを使用、独立デバイスとして
24スロットをそのまま使う。ノード/スロット分配は行わない):

  Rx_16Data (PC -> 本機, 指令):
    0-3: target (モータ1-4、意味はcontrol_modeに依存)
         - control_mode=0(速度): 電気角速度、10ERPM/LSB (レンジ ±327670ERPM)
           マニュアル4.1.4の帰還フレーム速度フィールドと同一スケール
         - control_mode=1(位置): 0.1deg/LSB (レンジ ±3276.7deg)
           マニュアル4.3.1の帰還フレーム位置フィールドと同一スケール
    4-7: control_mode (モータ1-4): 0=速度ループ, 1=位置ループ
         全ゼロ(E-STOP/未接続時のデフォルト)で0=速度・target=0となり、
         安全にゼロ速度指令(その場停止)になる設計。位置ジャンプは発生しない。
    8-23: 未使用

  Tx_16Data (本機 -> PC, 帰還。マニュアル4.3.1 CAN Upload Message Protocol
             (Function ID 0x29) の値をスケール変換無しでそのまま格納):
    0-3: position [0.1deg/LSB]
    4-7: speed    [10ERPM/LSB] (電気角速度。出力軸rpmへの換算はGUIプロファイルの
                                 scale項目で行う想定、極対数・減速比はモータ機種依存のため)
    8-11: current [0.01A/LSB]
    12-15: motor temperature [degC]
    16-19: error code (0=no fault, 1=motor over-temp, 2=over-current, 3=over-voltage,
                        4=under-voltage, 5=encoder fault, 6=MOSFET over-temp, 7=motor stall)
    20-23: 未使用

Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "cubemars.hpp"
#include "config.hpp"
#include "defs.hpp"
#include "frame_data.hpp"
#include <Arduino.h>

namespace {

// Servo(CAN)モードの制御モードID (マニュアル4.1節)
constexpr uint32_t CUBEMARS_CMD_RPM = 3; // Velocity Loop Mode
constexpr uint32_t CUBEMARS_CMD_POS = 4; // Position Loop Mode
constexpr uint32_t CUBEMARS_FEEDBACK_FUNCTION_ID = 0x29; // 定期帰還フレーム

// PC側(Rx_16Data)のcontrol_mode enum値
constexpr int16_t CUBEMARS_MODE_VELOCITY = 0;
constexpr int16_t CUBEMARS_MODE_POSITION = 1;

const uint8_t kMotorCanId[CUBEMARS_MOTOR_COUNT] = {
    CUBEMARS_MOTOR_ID_1,
#if CUBEMARS_MOTOR_COUNT > 1
    CUBEMARS_MOTOR_ID_2,
#endif
#if CUBEMARS_MOTOR_COUNT > 2
    CUBEMARS_MOTOR_ID_3,
#endif
#if CUBEMARS_MOTOR_COUNT > 3
    CUBEMARS_MOTOR_ID_4,
#endif
};

void sendExtended(uint32_t cmd_id, uint8_t motor_can_id, const uint8_t *data, uint8_t len) {
    twai_message_t tx{};
    tx.identifier = (cmd_id << 8) | motor_can_id;
    tx.extd = 1;
    tx.rtr = 0;
    tx.data_length_code = len;
    for (uint8_t i = 0; i < len; i++) {
        tx.data[i] = data[i];
    }
    if (twai_transmit(&tx, pdMS_TO_TICKS(20)) != ESP_OK) {
        Serial.println("[ERR] cubemars: twai_transmit failed");
    }
}

void sendInt32Command(uint32_t cmd_id, uint8_t motor_can_id, int32_t value) {
    uint8_t buffer[4];
    buffer[0] = (uint8_t)(value >> 24);
    buffer[1] = (uint8_t)(value >> 16);
    buffer[2] = (uint8_t)(value >> 8);
    buffer[3] = (uint8_t)value;
    sendExtended(cmd_id, motor_can_id, buffer, 4);
}

// -------- CAN送信 (指令 -> AKシリーズ) -------- //

void sendCommands() {
    for (int m = 0; m < CUBEMARS_MOTOR_COUNT; m++) {
        int16_t target = Rx_16Data[m];
        int16_t mode = Rx_16Data[4 + m];

        if (mode == CUBEMARS_MODE_POSITION) {
            // 0.1deg/LSB(本機スロット) -> 0.0001deg/LSB(CAN指令、マニュアル4.1.5)
            int32_t pos_cmd = (int32_t)target * 1000;
            sendInt32Command(CUBEMARS_CMD_POS, kMotorCanId[m], pos_cmd);
        } else {
            // 10ERPM/LSB(本機スロット) -> 1ERPM/LSB(CAN指令、マニュアル4.1.4)
            int32_t rpm_cmd = (int32_t)target * 10;
            sendInt32Command(CUBEMARS_CMD_RPM, kMotorCanId[m], rpm_cmd);
        }
    }
}

// -------- CAN受信 (AKシリーズ -> 帰還) -------- //

int motorIndexForCanId(uint8_t can_id) {
    for (int m = 0; m < CUBEMARS_MOTOR_COUNT; m++) {
        if (kMotorCanId[m] == can_id)
            return m;
    }
    return -1;
}

void receiveFeedback() {
    twai_message_t rx_msg;

    while (twai_receive(&rx_msg, 0) == ESP_OK) {
        if (!rx_msg.extd || rx_msg.data_length_code != 8)
            continue;

        uint32_t function_id = rx_msg.identifier >> 8;
        uint8_t driver_id = (uint8_t)(rx_msg.identifier & 0xFF);
        if (function_id != CUBEMARS_FEEDBACK_FUNCTION_ID)
            continue;

        int m = motorIndexForCanId(driver_id);
        if (m < 0)
            continue;

        int16_t position = (int16_t)((rx_msg.data[0] << 8) | rx_msg.data[1]);
        int16_t speed = (int16_t)((rx_msg.data[2] << 8) | rx_msg.data[3]);
        int16_t current = (int16_t)((rx_msg.data[4] << 8) | rx_msg.data[5]);
        int8_t temperature = (int8_t)rx_msg.data[6];
        uint8_t error_code = rx_msg.data[7];

        Tx_16Data[m] = position;
        Tx_16Data[4 + m] = speed;
        Tx_16Data[8 + m] = current;
        Tx_16Data[12 + m] = temperature;
        Tx_16Data[16 + m] = error_code;
    }
}

} // namespace

void cubemarsInit() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS(); // AKシリーズは1Mbps固定(マニュアル1.1節)
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("[ERR] cubemars: TWAI install failed");
        while (1)
            ;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("[ERR] cubemars: TWAI start failed");
        while (1)
            ;
    }
}

void cubemarsTask(void *pvParameters) {
    while (1) {
        receiveFeedback();
        sendCommands();
        vTaskDelay(1);
    }
}
