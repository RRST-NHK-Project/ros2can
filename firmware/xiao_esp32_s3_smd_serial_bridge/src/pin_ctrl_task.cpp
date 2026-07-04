/*====================================================================
<pin_ctrl_task.cpp>
・ピン操作関連の関数とタスクの実装ファイル
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "config.hpp"
#include "defs.hpp"
#include "driver/pcnt.h"
#include "frame_data.hpp"
#include "pin_ctrl_init.hpp"
#include <Arduino.h>

constexpr uint32_t CTRL_PERIOD_MS = 5; // ピン更新周期（ミリ秒）

void IO_Servo_Outout();
void IO_ENC_Input();
void IO_SW_Input();

void IO_Task(void *);

// ================= TASK =================

void IO_Task(void *) {
    TickType_t last_wake = xTaskGetTickCount();
    IO_init();

    while (1) {
        IO_Servo_Outout();
        IO_ENC_Input();
        IO_SW_Input();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CTRL_PERIOD_MS));
    }
}

// ================= 関数 =================
// マイコンや基板の不具合に対応するためにfor文は使っていない
// 実機は DCモータ非搭載 (ENCx2, SWx3, SERVOx3のみ。SW/SERVOはピン共有でconfig.hppのMULTI1-3により切替)

void IO_Servo_Outout() {
#if defined(MODE_CAN) || defined(MODE_CAN_HOST)
    if (MULTI1 == 1) {
        int angle1 = CanIoRxData[0];
        angle1 = constrain(angle1, SERVO1_MIN_DEG, SERVO1_MAX_DEG);
        int us1 = (int)map(angle1, SERVO1_MIN_DEG, SERVO1_MAX_DEG, SERVO1_MIN_US, SERVO1_MAX_US);
        int duty1 = (int)(us1 * SERVO_PWM_SCALE);
        ledcWrite(4, duty1);
    }

    if (MULTI2 == 1) {
        int angle2 = CanIoRxData[1];
        angle2 = constrain(angle2, SERVO2_MIN_DEG, SERVO2_MAX_DEG);
        int us2 = (int)map(angle2, SERVO2_MIN_DEG, SERVO2_MAX_DEG, SERVO2_MIN_US, SERVO2_MAX_US);
        int duty2 = (int)(us2 * SERVO_PWM_SCALE);
        ledcWrite(5, duty2);
    }

    if (MULTI3 == 1) {
        int angle3 = CanIoRxData[2];
        angle3 = constrain(angle3, SERVO3_MIN_DEG, SERVO3_MAX_DEG);
        int us3 = (int)map(angle3, SERVO3_MIN_DEG, SERVO3_MAX_DEG, SERVO3_MIN_US, SERVO3_MAX_US);
        int duty3 = (int)(us3 * SERVO_PWM_SCALE);
        ledcWrite(6, duty3);
    }
#else
    if (MULTI1 == 1) {
        int angle1 = Rx_16Data[9];
        angle1 = constrain(angle1, SERVO1_MIN_DEG, SERVO1_MAX_DEG);
        int us1 = (int)map(angle1, SERVO1_MIN_DEG, SERVO1_MAX_DEG, SERVO1_MIN_US, SERVO1_MAX_US);
        int duty1 = (int)(us1 * SERVO_PWM_SCALE);
        ledcWrite(4, duty1);
    }

    if (MULTI2 == 1) {
        int angle2 = Rx_16Data[10];
        angle2 = constrain(angle2, SERVO2_MIN_DEG, SERVO2_MAX_DEG);
        int us2 = (int)map(angle2, SERVO2_MIN_DEG, SERVO2_MAX_DEG, SERVO2_MIN_US, SERVO2_MAX_US);
        int duty2 = (int)(us2 * SERVO_PWM_SCALE);
        ledcWrite(5, duty2);
    }

    if (MULTI3 == 1) {
        int angle3 = Rx_16Data[11];
        angle3 = constrain(angle3, SERVO3_MIN_DEG, SERVO3_MAX_DEG);
        int us3 = (int)map(angle3, SERVO3_MIN_DEG, SERVO3_MAX_DEG, SERVO3_MIN_US, SERVO3_MAX_US);
        int duty3 = (int)(us3 * SERVO_PWM_SCALE);
        ledcWrite(6, duty3);
    }
#endif
}

void IO_ENC_Input() {
    // ENC入力処理
#if defined(MODE_CAN) || defined(MODE_CAN_HOST)
    int16_t enc1 = 0;
    int16_t enc2 = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &enc1);
    pcnt_get_counter_value(PCNT_UNIT_1, &enc2);
    CanIoTxData[3] = enc1;
    CanIoTxData[4] = enc2;
#elif !defined(MODE_CAN_HOST)
    // ホストモードでは CAN からのフィードバックを優先して、ローカルエンコーダ値で上書きしない
    int16_t enc1 = 0;
    int16_t enc2 = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &enc1);
    pcnt_get_counter_value(PCNT_UNIT_1, &enc2);
    Tx_16Data[1] = enc1;
    Tx_16Data[2] = enc2;
#endif
}

void IO_SW_Input() {
    // SW入力処理
#if defined(MODE_CAN) || defined(MODE_CAN_HOST)
    if (MULTI1 == 0) {
        CanIoTxData[0] = !digitalRead(SW1);
    } else {
        CanIoTxData[0] = 0;
    }

    if (MULTI2 == 0) {
        CanIoTxData[1] = !digitalRead(SW2);
    } else {
        CanIoTxData[1] = 0;
    }

    if (MULTI3 == 0) {
        CanIoTxData[2] = !digitalRead(SW3);
    } else {
        CanIoTxData[2] = 0;
    }
#elif !defined(MODE_CAN_HOST)
    if (MULTI1 == 0) {
        Tx_16Data[9] = !digitalRead(SW1);
    } else {
        Tx_16Data[9] = 0;
    }

    if (MULTI2 == 0) {
        Tx_16Data[10] = !digitalRead(SW2);
    } else {
        Tx_16Data[10] = 0;
    }

    if (MULTI3 == 0) {
        Tx_16Data[11] = !digitalRead(SW3);
    } else {
        Tx_16Data[11] = 0;
    }
#endif
}
