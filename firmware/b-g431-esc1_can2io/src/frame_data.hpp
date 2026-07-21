/*====================================================================
<frame_data.hpp>
・CANノードのローカルスロットデータ定義ヘッダーファイル

xiao-esp32-s3_can2io (CANホスト, MODE_CAN_HOST) との関係:
- ホストは 24 スロットを CAN_NODE_COUNT 台のノードへ CAN_SLOTS_PER_NODE ずつ
  分配/集約する。このボードはその1ノード分 (CAN_SLOTS_PER_NODE=5) を担当する。
- host -> node (指令) は Rx_16Data、node -> host (帰還) は Tx_16Data。
  (esp32_serial_bridge 系ファームと同じ命名に合わせている)

Copyright (c) 2026.
====================================================================*/

#pragma once
#include <stdint.h>

#include "config.hpp"

#define Tx16NUM CAN_SLOTS_PER_NODE // node -> host (帰還)
#define Rx16NUM CAN_SLOTS_PER_NODE // host -> node (指令)

extern volatile int16_t Tx_16Data[Tx16NUM];
/*
robomas (xiao-esp32-s3_can2io MODE_ROBOMAS) 互換のスロット構成。
0: angle (0.1 deg, エンコーダ連続角)
1: velocity (生rpm値, FOC推定)
2: current_q (0.001 A, 実測電流)
3-4: 未使用(予約)
*/

extern volatile int16_t Rx_16Data[Rx16NUM];
/*
robomas (xiao-esp32-s3_can2io MODE_ROBOMAS) 互換のスロット構成。速度制御のみ。
0: target_velocity (生rpm値)
1-4: 未使用(予約)
*/
