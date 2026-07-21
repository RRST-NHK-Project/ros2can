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
#define CAN_ID 102
#define CAN_NODE_INDEX ((CAN_ID % 100U) - 1U)

// ホスト側 (xiao-esp32-s3_can2io/src/config.hpp) の CAN_SLOTS_PER_NODE と
// 必ず一致させること。ここを変更する場合はホスト側も合わせて変更する。
#define CAN_SLOTS_PER_NODE 5

// CANバスのビットレート。ホスト側 (TWAI_TIMING_CONFIG_500KBITS) と一致させる。
#define CAN_BITRATE 500000UL

// CAN通信デバッグ用のシリアル診断出力(can_task.cpp canTaskPrintDiagnostics)。
// canTaskUpdate()から呼ばれ、loop()と同じスレッドで直列実行されるため、Serial出力が
// 詰まる(TXバッファ満杯等)とFOC制御ループ(motor.loopFOC()/motor.move())が
// その分だけ止まり、CAN_DIAG_PERIOD_MS(500ms)と同じ周期でトルクの「ガクッ」を
// 引き起こすことが実機で確認されている。CAN通信の確認が終わったら0のままにしておくこと。
#define CAN_DIAG_ENABLE 0

// robomas (MODE_ROBOMAS) 互換のため、CAN途絶時のフェイルセーフは持たない。
// CANが途切れても最後に受信したtarget_velocityを保持し続ける。

// 基板搭載のCAN終端抵抗(120Ω)。B-G431B-ESC1はCAN_TERM(PC14)でオンボードの
// アナログSPDTスイッチを介して120Ωの接続/切断をGPIOから制御できる
// (ST UM2516 Table 3参照。デフォルトはソルダーブリッジR26側)。
// このノードがCANバスの終端(末端)に位置する場合のみ1にすること。
// バス中間のノードで有効にすると特性インピーダンスが乱れ通信不安定の原因になる。
#define CAN_TERM_ENABLE 1 // 1=120Ω終端を有効化, 0=無効化(ホスト側または他ノードで既に終端している場合)

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

// ================= Slot Index (host -> node, 指令) =================
// Rx_16Data[0..4] (このボードはCAN_SLOTS_PER_NODE=5のうち index 0 のみ使用、1-4は未使用/予約)
// robomas (xiao-esp32-s3_can2io MODE_ROBOMAS) と互換: 速度制御のみ、生rpm値。
#define RX_TARGET_VELOCITY 0

// ================= Slot Index (node -> host, 帰還) =================
// Tx_16Data[0..4] (index 0-2 使用、3-4は未使用/予約)
// robomas と互換: angle(0.1deg) / velocity(生rpm) / current(0.001A)
#define TX_ANGLE 0
#define TX_VELOCITY 1
#define TX_CURRENT_Q 2

// ================= Unit scaling =================
// RX (host -> node): target_velocityは生rpm値(スケール無し)。SimpleFOC内部はrad/sのため
// CAN境界でのみ変換する。
#define RPM_TO_RAD_S (2.0f * PI / 60.0f)
#define RAD_S_TO_RPM (60.0f / (2.0f * PI))

// TX (node -> host): float -> int16
#define ANGLE_TX_SCALE 0.1f     // deg per LSB
#define CURRENT_TX_SCALE 0.001f // A per LSB
// velocityは生rpm値(スケール無し)。RAD_S_TO_RPMで変換してそのままint16化する。
