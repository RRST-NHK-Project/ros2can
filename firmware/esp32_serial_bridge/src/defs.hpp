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
#define LED 0

// MD PWM
#define MD1P 5
#define MD2P 12
#define MD3P 13
#define MD4P 14

// MD DIR
#define MD1D 15
#define MD2D 16
#define MD3D 17
#define MD4D 18

// サーボ
#define SERVO1 19
#define SERVO2 21
#define SERVO3 22
#define SERVO4 23

// ソレノイドバルブ
#define TR1 25
#define TR2 26
#define TR3 27
#define TR4 32
#define TR5 33
#define TR6 22
#define TR7 23

// エンコーダ
#define ENC1_A 19
#define ENC1_B 21
#define ENC2_A 22
#define ENC2_B 23
#define ENC3_A 15
#define ENC3_B 16
#define ENC4_A 17
#define ENC4_B 18

// スイッチ
#define SW1 5
#define SW2 12
#define SW3 13
#define SW4 14
#define SW5 15
#define SW6 16
#define SW7 17
#define SW8 18

// モータ数
#define motor 2

// ロボマス
#define CAN_RX 2
#define CAN_TX 4

// MD用
#define MD_PWM_MAX ((1 << MD_PWM_RESOLUTION) - 1)

// サーボ用
#define SERVO_PWM_PERIOD_US (1000000.0 / SERVO_PWM_FREQ) // 周波数から周期を計算
#define SERVO_PWM_MAX_DUTY ((1 << SERVO_PWM_RESOLUTION) - 1)
#define SERVO_PWM_SCALE (SERVO_PWM_MAX_DUTY / SERVO_PWM_PERIOD_US)

// ロボマス
#define NUM_MOTOR 4                    // モーター数
#define gear_m3508 19.2f               // M3508ギア比
#define gear_m2006 36.0f               // M2006ギア比
#define ENCODER_MAX 8192               // エンコーダ分解能
#define HALF_ENCODER (ENCODER_MAX / 2) // エンコーダ分解能の半分

// 状態変数の宣言
extern int encoder_count[NUM_MOTOR];
extern int rpm[NUM_MOTOR];
extern int current[NUM_MOTOR];
extern bool offset_ok[NUM_MOTOR];
extern int encoder_offset[NUM_MOTOR];
extern int last_encoder[NUM_MOTOR];
extern int rotation_count[NUM_MOTOR];
extern long total_encoder[NUM_MOTOR];
// extern float angle[NUM_MOTOR];
// extern float vel[NUM_MOTOR];

// PID関連変数
extern float target_angle[NUM_MOTOR];
extern float pos_error_prev[NUM_MOTOR];
extern float cur_error_prev[NUM_MOTOR];
extern float pos_integral[NUM_MOTOR];
extern float vel_integral[NUM_MOTOR];
extern float cur_integral[NUM_MOTOR];
extern float pos_output[NUM_MOTOR];
extern float cur_output[NUM_MOTOR];
extern float motor_output_current[NUM_MOTOR];

extern float angle_m3508[NUM_MOTOR];
extern float vel_m3508[NUM_MOTOR];
extern float c[NUM_MOTOR];
extern float output[NUM_MOTOR];
extern float angle[NUM_MOTOR];

extern unsigned long lastPidTime;