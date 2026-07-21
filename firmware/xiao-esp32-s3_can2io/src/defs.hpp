/*====================================================================
<defs.h>
・定数の定義ファイル

Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once
#include "config.hpp"
#include <Arduino.h>

// ================= ピンの定義 =================

// パルスカウンタの上限・下限の定義
#define COUNTER_H_LIM 32767
#define COUNTER_L_LIM -32768
#define PCNT_FILTER_VALUE 1023 // 0~1023, 1 = 12.5ns

#define ENC_PPR_SPEC 2048      // エンコーダの設定値
#define PPR (ENC_PPR_SPEC * 4) // 実効PPR（x4カウント）

#define DEG_PER_COUNT (360.0f / PPR)
#define HALF_PPR (PPR / 2)

// ピンの定義 //
// 状態表示LED
#define LED 21

// サーボ
#define SERVO1 7
#define SERVO2 8
#define SERVO3 9

// エンコーダ
#define ENC1_A 3
#define ENC1_B 4
#define ENC2_A 5
#define ENC2_B 6

// スイッチ
#define SW1 7
#define SW2 8
#define SW3 9

// CAN (MCP2561 + TWAI)
#define CAN_RX 2
#define CAN_TX 1

// サーボ用
#define SERVO_PWM_PERIOD_US (1000000.0 / SERVO_PWM_FREQ) // 周波数から周期を計算
#define SERVO_PWM_MAX_DUTY ((1 << SERVO_PWM_RESOLUTION) - 1)
#define SERVO_PWM_SCALE (SERVO_PWM_MAX_DUTY / SERVO_PWM_PERIOD_US)

// ロボマス (MODE_ROBOMAS、詳細はrobomas.hpp/cppを参照)
#define NUM_MOTOR 4                    // 1バスあたりのモーター数
#define gear_m3508 19.2f               // M3508ギア比
#define gear_m2006 36.0f               // M2006ギア比
#define ENCODER_MAX 8192               // エンコーダ分解能
#define HALF_ENCODER (ENCODER_MAX / 2) // エンコーダ分解能の半分