/*====================================================================
<config.h>
書き込み前にここでIDと動作モードを設定してください．MDやサーボの設定もここで行います．
MDは基本的に変更不要ですが，サーボは型番、機構に応じて適切に設定する必要があります．
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once
#include <Arduino.h>

// ================= 基本設定 =================

// IDの設定，シリアルフレームのDEVICE_IDとして使用します。
#define DEVICE_ID 1

// CAN_IDは3桁形式で指定します。
// 1桁目はバス番号、末尾2桁はノード番号を表します。
// 例: 101, 102, 103, 104
#define CAN_ID 401

// モードの設定，どれか一つをコメントアウト解除すること
// #define MODE_CAN
#define MODE_CAN_HOST
// #define MODE_IO
// #define MODE_DEBUG
// #define MODE_CAN_MONITOR
// #define MODE_ROBOMAS

// ================= サーボ関連 =================

// サーボ関連の設定、使用するサーボに応じて変更
#define SERVO_PWM_FREQ 50       // サーボPWM周波数（Hz）
#define SERVO_PWM_RESOLUTION 14 // サーボPWM分解能（bit）

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
#define ENABLE_LED 1 // 状態表示LEDを有効にする場合1に設定

// 汎用（MULTI）ポートの設定（スイッチ:0, サーボ:1）
#define MULTI1 0
#define MULTI2 0
#define MULTI3 0

// CANのノード割り当て設定
// 1つのCANバス上で最大4ノードまで対応し、1ノードあたり5スロットをCANで送受信する
#define CAN_NODE_COUNT 4
#define CAN_SLOTS_PER_NODE 5
// CAN_ID の下位2桁をノード番号として使用する。
// 例: CAN_ID=101 -> node 1, CAN_ID=102 -> node 2, CAN_ID=103 -> node 3, CAN_ID=104 -> node 4
#define CAN_NODE_INDEX ((CAN_ID % 100U) - 1U)

// ================= ロボマス関連 (MODE_ROBOMASのみ有効) =================
// MODE_ROBOMASはxiao-esp32-s3_can2ioのノード/スロット分配方式(CAN 500kbps)とは
// 別系統で、DJI RoboMasterシリーズのCANプロトコル(1Mbps固定, ID固定)を直接喋る
// 独立デバイスとして動作する。1マイコン(1バス)には同一機種のみ最大4台まで接続可能。
// このバスにxiao-esp32-s3_can2ioの他ノード(MODE_CAN等)を混在させることはできない
// (ビットレートが異なるため)。詳細はREADME.md参照。

#define ROBOMAS_MOTOR_M3508 1  // C620 + M3508 (メカナム/足回り等、ギア比19.2)
#define ROBOMAS_MOTOR_M2006 2  // C610 + M2006 (小型アクチュエータ等、ギア比36.0)
#define ROBOMAS_MOTOR_GM6020 3 // GM6020 (ダイレクトドライブ、ギア無し)

// 使用するモータ機種を1つ選択すること。
#define ROBOMAS_MOTOR_TYPE ROBOMAS_MOTOR_M2006

// 速度PIDゲイン。ros2can(PC)側からは変更できない固定値。チューニングはここで行う。
#if ROBOMAS_MOTOR_TYPE == ROBOMAS_MOTOR_M3508
#define ROBOMAS_KP_VEL 0.8f
#define ROBOMAS_KI_VEL 0.0f
#define ROBOMAS_KD_VEL 0.05f
#define ROBOMAS_OUTPUT_GAIN 10.0f   // PID出力 -> 電流指令[A]への換算係数
#define ROBOMAS_MAX_CURRENT_A 20.0f // 電流指令の飽和値[A] (C620仕様上限)
#elif ROBOMAS_MOTOR_TYPE == ROBOMAS_MOTOR_M2006
#define ROBOMAS_KP_VEL 0.8f
#define ROBOMAS_KI_VEL 0.0f
#define ROBOMAS_KD_VEL 0.02f
#define ROBOMAS_OUTPUT_GAIN 1.0f
#define ROBOMAS_MAX_CURRENT_A 1.0f
#elif ROBOMAS_MOTOR_TYPE == ROBOMAS_MOTOR_GM6020
#define ROBOMAS_KP_VEL 0.03f
#define ROBOMAS_KI_VEL 0.0f
#define ROBOMAS_KD_VEL 0.0f
#define ROBOMAS_OUTPUT_GAIN 1.0f
#define ROBOMAS_MAX_CURRENT_A 10.0f
#else
#error "ROBOMAS_MOTOR_TYPE: unknown motor type"
#endif
