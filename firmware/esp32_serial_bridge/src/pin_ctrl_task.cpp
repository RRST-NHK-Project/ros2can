/*====================================================================
<pin_ctrl_task.cpp>
・ピン操作関連の関数とタスクの実装ファイル
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "defs.hpp"
#include "driver/pcnt.h"
#include "frame_data.hpp"
#include "pin_ctrl_init.hpp"
#include <Arduino.h>

constexpr uint32_t CTRL_PERIOD_MS = 5; // ピン更新周期（ミリ秒）

void MD_Output();
void Servo_Output();
void TR_Output();
void ENC_Input();
void SW_Input();
void IO_MD_Output();
void IO_TR_Output();
void IO_ENC_Input();
void IO_SW_Input();
void ROBOMAS_IO_ENC_Input();
void ROBOMAS_IO_SW_Input();

// ================= TASK =================

void Output_Task(void *) {
    TickType_t last_wake = xTaskGetTickCount();
    Output_init();

    Rx_16Data[9] = SERVO1_INIT_DEG;
    Rx_16Data[10] = SERVO2_INIT_DEG;
    Rx_16Data[11] = SERVO3_INIT_DEG;
    Rx_16Data[12] = SERVO4_INIT_DEG;

    while (1) {
        MD_Output();
        Servo_Output();
        TR_Output();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CTRL_PERIOD_MS));
    }
}

void Input_Task(void *) {
    TickType_t last_wake = xTaskGetTickCount();
    Input_init();

    while (1) {
        ENC_Input();
        SW_Input();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CTRL_PERIOD_MS));
    }
}

void IO_Task(void *) {
    TickType_t last_wake = xTaskGetTickCount();
    IO_init();

    while (1) {
        IO_MD_Output();
        IO_TR_Output();
        IO_ENC_Input();
        IO_SW_Input();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CTRL_PERIOD_MS));
    }
}

void ROBOMAS_IO_Task(void *) {
    TickType_t last_wake = xTaskGetTickCount();
    ROBOMAS_IO_init();

    while (1) {
        //Servo_Output();
        IO_TR_Output();
        ROBOMAS_IO_ENC_Input();
        ROBOMAS_IO_SW_Input();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CTRL_PERIOD_MS));
    }
}

void ENC_Input() {
    // ENC入力処理
    // taskENTER_CRITICAL();
    pcnt_get_counter_value(PCNT_UNIT_0, (int16_t *)&Tx_16Data[1]);
    pcnt_get_counter_value(PCNT_UNIT_1, (int16_t *)&Tx_16Data[2]);
    pcnt_get_counter_value(PCNT_UNIT_2, (int16_t *)&Tx_16Data[3]);
    pcnt_get_counter_value(PCNT_UNIT_3, (int16_t *)&Tx_16Data[4]);
    // taskEXIT_CRITICAL();
}

void SW_Input() {
    // SW入力処理
    Tx_16Data[9] = !digitalRead(SW1);
    Tx_16Data[10] = !digitalRead(SW2);
    Tx_16Data[11] = !digitalRead(SW3);
    Tx_16Data[12] = !digitalRead(SW4);
    Tx_16Data[13] = !digitalRead(SW5);
    Tx_16Data[14] = !digitalRead(SW6);
    Tx_16Data[15] = !digitalRead(SW7);
    Tx_16Data[16] = !digitalRead(SW8);
}

// ================= 関数 =================
// マイコンや基板の不具合に対応するためにfor文は使っていない

void MD_Output() {

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

void Servo_Output() {

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

void TR_Output() {
    digitalWrite(TR1, Rx_16Data[17] ? HIGH : LOW);
    digitalWrite(TR2, Rx_16Data[18] ? HIGH : LOW);
    digitalWrite(TR3, Rx_16Data[19] ? HIGH : LOW);
    digitalWrite(TR4, Rx_16Data[20] ? HIGH : LOW);
    digitalWrite(TR5, Rx_16Data[21] ? HIGH : LOW);
    if (ENABLE_EXTRA_TR_PIN) {
        digitalWrite(TR6, Rx_16Data[22] ? HIGH : LOW);
        digitalWrite(TR7, Rx_16Data[23] ? HIGH : LOW);
    }
}

void IO_MD_Output() {

    static int Rx16Data_local[Rx16NUM];

    for (int i = 1; i <= 2; i++) {
        Rx16Data_local[i] = constrain(Rx_16Data[i], -MD_PWM_MAX, MD_PWM_MAX);
    }

    digitalWrite(MD1D, Rx16Data_local[1] > 0 ? HIGH : LOW);
    digitalWrite(MD2D, Rx16Data_local[2] > 0 ? HIGH : LOW);

    ledcWrite(0, abs(Rx16Data_local[1]));
    ledcWrite(1, abs(Rx16Data_local[2]));
}

void IO_TR_Output() {
    digitalWrite(TR1, Rx_16Data[17] ? HIGH : LOW);
    digitalWrite(TR2, Rx_16Data[18] ? HIGH : LOW);
    digitalWrite(TR3, Rx_16Data[19] ? HIGH : LOW);
    digitalWrite(TR4, Rx_16Data[20] ? HIGH : LOW);
    digitalWrite(TR5, Rx_16Data[21] ? HIGH : LOW);
    if (ENABLE_EXTRA_TR_PIN) {
        digitalWrite(TR6, Rx_16Data[22] ? HIGH : LOW);
        digitalWrite(TR7, Rx_16Data[23] ? HIGH : LOW);
    }
}

void IO_ENC_Input() {
    pcnt_get_counter_value(PCNT_UNIT_0, (int16_t *)&Tx_16Data[1]);
    pcnt_get_counter_value(PCNT_UNIT_1, (int16_t *)&Tx_16Data[2]);
}

void ROBOMAS_IO_ENC_Input() {
    pcnt_get_counter_value(PCNT_UNIT_0, (int16_t *)&Tx_16Data[1]);
    pcnt_get_counter_value(PCNT_UNIT_1, (int16_t *)&Tx_16Data[2]);
    pcnt_get_counter_value(PCNT_UNIT_2, (int16_t *)&Tx_16Data[3]);
    pcnt_get_counter_value(PCNT_UNIT_3, (int16_t *)&Tx_16Data[4]);
}

void ROBOMAS_IO_SW_Input() {
    Tx_16Data[9] = !digitalRead(SW1);
    Tx_16Data[10] = !digitalRead(SW2);
    }

// void IO_ENC_Input() {
//     // ENC入力処理
//     // taskENTER_CRITICAL();

//     int16_t cnt0, cnt1;
//     float angle0, angle1;

//     // カウント取得
//     pcnt_get_counter_value(PCNT_UNIT_0, &cnt0);
//     pcnt_get_counter_value(PCNT_UNIT_1, &cnt1);

//     // 度数法変換
//     angle0 = cnt0 * DEG_PER_COUNT;
//     angle1 = cnt1 * DEG_PER_COUNT;

//     Tx_16Data[1] = static_cast<int16_t>(angle0);
//     Tx_16Data[2] = static_cast<int16_t>(angle1);

//     // taskEXIT_CRITICAL();
// }

void IO_SW_Input() {
    // SW入力処理
    Tx_16Data[11] = !digitalRead(SW3);
    Tx_16Data[12] = !digitalRead(SW4);
    Tx_16Data[15] = !digitalRead(SW7);
    Tx_16Data[16] = !digitalRead(SW8);
}
