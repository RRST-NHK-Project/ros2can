/*====================================================================
<config.h>
書き込み前にここでIDと動作モードを設定してください．MDやサーボの設定もここで行います．
MDは基本的に変更不要ですが，サーボは型番、機構に応じて適切に設定する必要があります．
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once
#include <Arduino.h>

// ================= 基本設定 =================

// IDの設定，ROS側からマイコンを識別するために使用，すべてのマイコンで異なる値にすること
#define DEVICE_ID 3

// モードの設定，どれか一つをコメントアウト解除する
// #define MODE_OUTPUT
// #define MODE_INPUT
// #define MODE_IO
// #define MODE_ROBOMAS
// #define MODE_ROBOMAS_PLUS_OUTPUT
// #define MODE_ROBOMAS_PLUS_INPUT
#define MODE_ROBOMAS_PLUS_IO
// #define MODE_DEBUG

// ================= MD関連 =================

// MD関連の設定，使用するMDに応じて変更
#define MD_PWM_FREQ 20000   // MDのPWM周波数（Hz）
#define MD_PWM_RESOLUTION 8 // MDのPWM分解能（bit）

// ================= サーボ関連 =================

// サーボ関連の設定、使用するサーボに応じて変更
#define SERVO_PWM_FREQ 50       // サーボPWM周波数（Hz）
#define SERVO_PWM_RESOLUTION 14 // サーボPWM分解能（bit）14ビット推奨

// サーボの最小・最大パルス幅、角度範囲、初期角度の設定
#define SERVO1_MIN_US 500
#define SERVO1_MAX_US 2500
#define SERVO1_MIN_DEG 0
#define SERVO1_MAX_DEG 270
#define SERVO1_INIT_DEG 0

#define SERVO2_MIN_US 500
#define SERVO2_MAX_US 2500
#define SERVO2_MIN_DEG 0
#define SERVO2_MAX_DEG 270
#define SERVO2_INIT_DEG 0

#define SERVO3_MIN_US 500
#define SERVO3_MAX_US 2500
#define SERVO3_MIN_DEG 0
#define SERVO3_MAX_DEG 270
#define SERVO3_INIT_DEG 0

#define SERVO4_MIN_US 500
#define SERVO4_MAX_US 2500
#define SERVO4_MIN_DEG 0
#define SERVO4_MAX_DEG 270
#define SERVO4_INIT_DEG 0

// ================= 高度な設定（通常は変更不要） =================

// 以下の設定は必要に応じて変更
#define ENABLE_LED 1          // 状態表示LEDを有効にする場合1に設定
#define ENABLE_EXTRA_TR_PIN 0 // TR6,TR7を有効にする場合1に設定、サーボとのピン競合に注意
