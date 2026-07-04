/*====================================================================
Project: esp32_s3_serial_bridge
Target board: RRST-ESP32-S3 Rev.1

Description:
  ROS 2・マイコン間の通信を行うserial_bridgeパッケージのマイコン側プログラム。
  PCから送られてくるバイナリデータを受信、デコードしマイコンのGPIO出力に反映させる。
  config.hppで各種設定をするだけで使用可能です。このファイル(main.cpp)を直接編集しないこと。

Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "config.hpp"
#include "defs.hpp"
#include "pid_task.hpp"
#include "pin_ctrl_task.hpp"
#include "robomas.hpp"
#include "serial_task.hpp"
#include <Arduino.h>
// ================= SETUP =================

void setup() {

    // ボーレートは実機テストしながら調整する予定
    Serial.begin(115200);

    delay(200);
    delay(100 * DEVICE_ID); // 安定待ち, IDごとに開始タイミングをずらす

    pinMode(LED, OUTPUT);

    // ready
    for (int i = 0; i < 5; i++) {
        digitalWrite(LED, HIGH);
        delay(50);
        digitalWrite(LED, LOW);
        delay(50);
    }

    // ledcSetup(1, 20000, 8);
    // ledcAttachPin(LED, 1);

    xTaskCreate(
        serialTask,   // タスク関数
        "serialTask", // タスク名
        2048,         // スタックサイズ（words）
        NULL,
        10, // 優先度
        NULL);

    // モードに応じた初期化
    // #if defined(MODE_OUTPUT)
    //     // 出力モード初期化
    //     xTaskCreate(
    //         Output_Task,   // タスク関数
    //         "Output_Task", // タスク名
    //         2048,          // スタックサイズ（words）
    //         NULL,
    //         11, // 優先度
    //         NULL);

    // #elif defined(MODE_INPUT)
    //     // 入力モード初期化
    //     xTaskCreate(
    //         Input_Task,   // タスク関数
    //         "Input_Task", // タスク名
    //         1024,         // スタックサイズ（words）
    //         NULL,
    //         4, // 優先度
    //         NULL);

#if defined(MODE_IO)
    // 入出力モード初期化
    xTaskCreate(
        IO_Task,   // タスク関数
        "IO_Task", // タスク名
        2048,      // スタックサイズ（words）
        NULL,
        11, // 優先度
        NULL);

#elif defined(MODE_ROBOMAS)
    // ロボマスモード初期化

    robomas_init();

    // xTaskCreate(
    //     M3508_Task,   // タスク関数
    //     "M3508_Task", // タスク名
    //     2048,         // スタックサイズ（words）
    //     NULL,
    //     9, // 優先度
    //     NULL);

    // xTaskCreate(
    //     PID_Task,   // タスク関数
    //     "PID_Task", // タスク名
    //     2048,       // スタックサイズ（words）
    //     NULL,
    //     11, // 優先度
    //     NULL);

    xTaskCreate(
        M2006_Task,   // タスク関数
        "M2006_Task", // タスク名
        2048,         // スタックサイズ（words）
        NULL,
        9, // 優先度
        NULL);
#elif defined(MODE_ROBOMAS_AD)
    // ロボマスモード初期化

    robomas_init();

    xTaskCreate(
        M3508_Task,   // タスク関数
        "M3508_Task", // タスク名
        2048,         // スタックサイズ（words）
        NULL,
        9, // 優先度
        NULL);

    // xTaskCreate(
    //     PID_Task,   // タスク関数
    //     "PID_Task", // タスク名
    //     2048,       // スタックサイズ（words）
    //     NULL,
    //     11, // 優先度
    //     NULL);

    // 出力モード初期化
    xTaskCreate(
        Output_Task,   // タスク関数
        "Output_Task", // タスク名
        2048,          // スタックサイズ（words）
        NULL,
        8, // 優先度
        NULL);

#elif defined(MODE_SDM15)
    // 出力モード初期化
    xTaskCreate(
        SDM15_Task,   // タスク関数
        "SDM15_Task", // タスク名
        2048,         // スタックサイズ（words）
        NULL,
        11, // 優先度
        NULL);

#elif defined(MODE_DEBUG)
    // デバッグモード初期化

    // xTaskCreate(
    //     LED_PWM_Task,   // タスク関数
    //     "LED_PWM_Task", // タスク名
    //     1024,           // スタックサイズ（words）
    //     NULL,
    //     9, // 優先度
    //     NULL)0;

    // xTaskCreate(
    //     LED_Blink100_Task,   // タスク関数
    //     "LED_Blink100_Task", // タスク名
    //     1024,                // スタックサイズ（words）
    //     NULL,
    //     9, // 優先度
    //     NULL);

    xTaskCreate(
        PID_Task,   // タスク関数
        "PID_Task", // タスク名
        2048,       // スタックサイズ（words）
        NULL,
        11, // 優先度
        NULL);

#else
#error "No mode defined. Please define one mode in config.hpp."
#endif

#if (defined(MODE_OUTPUT) + defined(MODE_INPUT) + defined(MODE_IO) + \
     defined(MODE_ROBOMAS) + defined(MODE_ROBOMAS_AD) + defined(MODE_SDM15) + defined(MODE_DEBUG)) != 1
#error "Invalid mode configuration. Please define exactly *one mode* in config.hpp."
#endif
}

// ================= LOOP =================

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
    // メインループはなにもしない、処理はすべてFreeRTOSタスクで行う
}
