/*====================================================================
<pin_ctrl_task.cpp>
・ピン操作関連の関数とタスクの実装ファイル
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "SDM15.h"
#include "defs.hpp"
#include "driver/pcnt.h"
#include "frame_data.hpp"
#include "pin_ctrl_init.hpp"
#include <Arduino.h>

constexpr uint32_t CTRL_PERIOD_MS = 5; // ピン更新周期（ミリ秒）

void IO_MD_Output();
void IO_Servo_Outout();
void IO_TR_Output();
void IO_ENC_Input();
void IO_SW_Input();

void IO_Task(void *);
void SDM15_Task(void *);

HardwareSerial SerialSDM(1);
SDM15 sdm15(SerialSDM);

// ================= TASK =================

void IO_Task(void *) {
    TickType_t last_wake = xTaskGetTickCount();
    IO_init();

    while (1) {
        IO_MD_Output();
        IO_Servo_Outout();
        IO_TR_Output();
        IO_ENC_Input();
        IO_SW_Input();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CTRL_PERIOD_MS));
    }
}

// ================= 関数 =================
// マイコンや基板の不具合に対応するためにfor文は使っていない

void IO_MD_Output() {

    static int Rx16Data_local[Rx16NUM];

    for (int i = 1; i <= 4; i++) {
        Rx16Data_local[i] = constrain(Rx_16Data[i], -MD_PWM_MAX, MD_PWM_MAX);
    }

    digitalWrite(MD1D, Rx16Data_local[1] > 0 ? HIGH : LOW);
    digitalWrite(MD2D, Rx16Data_local[2] > 0 ? HIGH : LOW);
    digitalWrite(MD3D, Rx16Data_local[3] > 0 ? HIGH : LOW);
    digitalWrite(MD4D, Rx16Data_local[4] > 0 ? HIGH : LOW);

    ledcWrite(0, abs(Rx16Data_local[1]));
    ledcWrite(1, abs(Rx16Data_local[2]));
    ledcWrite(2, abs(Rx16Data_local[3]));
    ledcWrite(3, abs(Rx16Data_local[4]));
}

void IO_Servo_Outout() {

    // サーボ1
    int angle1 = Rx_16Data[9];
    angle1 = constrain(angle1, SERVO1_MIN_DEG, SERVO1_MAX_DEG);
    int us1 = (int)map(angle1, SERVO1_MIN_DEG, SERVO1_MAX_DEG, SERVO1_MIN_US, SERVO1_MAX_US);
    int duty1 = (int)(us1 * SERVO_PWM_SCALE);
    ledcWrite(4, duty1);

    // サーボ2
    int angle2 = Rx_16Data[10];
    angle2 = constrain(angle2, SERVO2_MIN_DEG, SERVO2_MAX_DEG);
    int us2 = (int)map(angle2, SERVO2_MIN_DEG, SERVO2_MAX_DEG, SERVO2_MIN_US, SERVO2_MAX_US);
    int duty2 = (int)(us2 * SERVO_PWM_SCALE);
    ledcWrite(5, duty2);

    // サーボ3
    int angle3 = Rx_16Data[11];
    angle3 = constrain(angle3, SERVO3_MIN_DEG, SERVO3_MAX_DEG);
    int us3 = (int)map(angle3, SERVO3_MIN_DEG, SERVO3_MAX_DEG, SERVO3_MIN_US, SERVO3_MAX_US);
    int duty3 = (int)(us3 * SERVO_PWM_SCALE);
    ledcWrite(6, duty3);

    // サーボ4
    int angle4 = Rx_16Data[12];
    angle4 = constrain(angle4, SERVO4_MIN_DEG, SERVO4_MAX_DEG);
    int us4 = (int)map(angle4, SERVO4_MIN_DEG, SERVO4_MAX_DEG, SERVO4_MIN_US, SERVO4_MAX_US);
    int duty4 = (int)(us4 * SERVO_PWM_SCALE);
    ledcWrite(7, duty4);
}

void IO_TR_Output() {
    digitalWrite(TR1, Rx_16Data[17] ? HIGH : LOW);
    digitalWrite(TR2, Rx_16Data[18] ? HIGH : LOW);
    digitalWrite(TR3, Rx_16Data[19] ? HIGH : LOW);
    digitalWrite(TR4, Rx_16Data[20] ? HIGH : LOW);
    digitalWrite(TR5, Rx_16Data[21] ? HIGH : LOW);
    digitalWrite(TR6, Rx_16Data[22] ? HIGH : LOW);
}

void IO_ENC_Input() {
    // ENC入力処理
    // taskENTER_CRITICAL();
    pcnt_get_counter_value(PCNT_UNIT_0, (int16_t *)&Tx_16Data[1]);
    pcnt_get_counter_value(PCNT_UNIT_1, (int16_t *)&Tx_16Data[2]);
    // taskEXIT_CRITICAL();
}

void IO_SW_Input() {
    // SW入力処理
    Tx_16Data[9] = !digitalRead(SW1);
    Tx_16Data[10] = !digitalRead(SW2);
    Tx_16Data[11] = !digitalRead(SW3);
    Tx_16Data[12] = !digitalRead(SW4);
    Tx_16Data[13] = !digitalRead(SW5);
}

void SDM15_Task(void *pvParameters) {

    (void)pvParameters;

    // HardwareSerial.begin() は (baud, config, rxPin, txPin) の順序
    SerialSDM.begin(460800, SERIAL_8N1, SDM15_RX, SDM15_TX);
    SerialSDM.setTimeout(20);

    // 高速通信対策
    SerialSDM.setRxBufferSize(2048);

    vTaskDelay(pdMS_TO_TICKS(1000)); // 安定待ち

    sdm15.StartScan();

    while (1) {
        // 溜まっているフレームを読み切り、最後の有効値だけを反映する
        bool has_latest = false;
        ScanData latest_data = {};

        while (SerialSDM.available() >= 9) {
            ScanData data = sdm15.GetScanData();
            if (!data.checksum_error) {
                latest_data = data;
                has_latest = true;
            }
        }

        if (has_latest) {
            // 受信データを送信用配列に格納（常に最新値のみ）
            Tx_16Data[0] = int16_t(latest_data.distance);
            Tx_16Data[1] = int16_t(latest_data.intensity);
            Tx_16Data[2] = int16_t(latest_data.disturb);
        }

        vTaskDelay(pdMS_TO_TICKS(CTRL_PERIOD_MS));
    }
}
