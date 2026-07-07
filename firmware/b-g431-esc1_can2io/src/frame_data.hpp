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
0: angle (0.1 deg, エンコーダ連続角)
1: velocity (0.1 rad/s, FOC推定)
2: current_q (0.001 A, 実測電流)
3: mode echo (0=velocity, 1=angle, 2=torque)
4: status bits (bit0=CAN受信生存, bit1=overspeed guard作動, bit2=enable状態)
*/

extern volatile int16_t Rx_16Data[Rx16NUM];
/*
0: enable (0=stop, 1=run)
1: mode (0=velocity, 1=angle, 2=torque)
2: target_velocity (0.1 rad/s)
3: target_angle (0.1 deg)
4: target_torque (0.001 A, 電流指令)
*/
