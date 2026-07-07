/*====================================================================
<config.hpp>
書き込み前にここでCAN ID・スロット数・モータ設定を確認してください。

このボードは xiao-esp32-s3_can2io (MODE_CAN_HOST) の配下にぶら下がる
CANノードの1台として動作します。単体ではROSと直接通信しません。

Copyright (c) 2026.
====================================================================*/

#pragma once

#include <stdint.h>

// ================= CAN 基本設定 =================

// CAN_IDは3桁形式(xiao-esp32-s3_can2io側と同じ規則)で指定します。
// 末尾2桁がノード番号(1-origin)。例: 101 -> node1, 102 -> node2, ...
// ホスト側 config.hpp の CAN_NODE_COUNT の範囲内で、他ノードと重複しない値にすること。
#define CAN_ID 101
#define CAN_NODE_INDEX ((CAN_ID % 100U) - 1U)

// ホスト側 (xiao-esp32-s3_can2io/src/config.hpp) の CAN_SLOTS_PER_NODE と
// 必ず一致させること。ここを変更する場合はホスト側も合わせて変更する。
#define CAN_SLOTS_PER_NODE 5

// CANバスのビットレート。ホスト側 (TWAI_TIMING_CONFIG_500KBITS) と一致させる。
#define CAN_BITRATE 500000UL

// この時間 [ms] 以上ホストからのCANフレームを受信できない場合はフェイルセーフ
// (トルク/速度指令を強制的に0にする)。
#define CAN_CMD_TIMEOUT_MS 500

// ================= モータ設定 =================

// C3542-920KV 想定の初期値
// - 一般的な 14 極ロータを想定して pole pairs = 7
#define MOTOR_POLE_PAIRS 7

#define DEFAULT_VOLTAGE_SUPPLY 24.0f
#define DEFAULT_VOLTAGE_LIMIT 6.0f
#define DEFAULT_VELOCITY_LIMIT 1500.0f
// 電流制限 [A] (トルクモードの指令クランプにも使用する)
#define DEFAULT_CURRENT_LIMIT 10.0f

// ================= Sensor (Encoder, TIM4) =================
// AMT102 相当、実配線に合わせて変更すること。
#define ENCODER_CPR 2048

// ================= FOC gains =================
// これらはコンパイル時定数固定。CAN経由でのランタイム変更はできない
// (CAN_SLOTS_PER_NODE=5 では枠が無いため)。

// Current loop
#define CURRENT_PID_P 0.1f
#define CURRENT_PID_I 10.0f
#define CURRENT_PID_D 0.0f

// Velocity loop
#define VELOCITY_PID_P 0.02f
#define VELOCITY_PID_I 0.0f
#define VELOCITY_PID_D 0.0f
#define VELOCITY_PID_OUTPUT_RAMP 1000.0f
#define VELOCITY_LPF_TF 0.02f

// Angle loop
#define ANGLE_P_GAIN 8.0f

// ================= Slot Index (host -> node, 指令) =================
// Rx_16Data[0..4]
#define RX_ENABLE 0
#define RX_MODE 1
#define RX_TARGET_VELOCITY 2
#define RX_TARGET_ANGLE 3
#define RX_TARGET_TORQUE 4

// ================= Slot Index (node -> host, 帰還) =================
// Tx_16Data[0..4]
#define TX_ANGLE 0
#define TX_VELOCITY 1
#define TX_CURRENT_Q 2
#define TX_MODE 3
#define TX_STATUS 4

// TX_STATUS のビット定義
#define STATUS_BIT_CAN_ALIVE (1 << 0)
#define STATUS_BIT_OVERSPEED_GUARD (1 << 1)
#define STATUS_BIT_ENABLED (1 << 2)

// ================= Unit scaling =================
// RX (host -> node): int16 -> float
#define TARGET_VELOCITY_SCALE 0.1f  // rad/s per LSB
#define TARGET_ANGLE_SCALE 0.1f     // deg per LSB
#define TARGET_TORQUE_SCALE 0.001f  // A per LSB

// TX (node -> host): float -> int16
#define ANGLE_TX_SCALE 0.1f      // deg per LSB
#define VELOCITY_TX_SCALE 0.1f   // rad/s per LSB
#define CURRENT_TX_SCALE 0.001f  // A per LSB
