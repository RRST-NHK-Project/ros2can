/*====================================================================
<robstride.hpp>
・MODE_ROBSTRIDE(RobStride EduLite 05 ドライバ)のヘッダーファイル

MODE_DAMIAO/MODE_CUBEMARSと同様、xiao-esp32-s3_can2ioの他モード
(MODE_IO/MODE_CAN/MODE_CAN_HOST)が使うノード/スロット分配プロトコルとは
別系統で、RobStride独自(私有)プロトコル(29bit拡張ID, 1Mbps)を直接喋る
独立デバイスとして動作する。1マイコン(1バス)には最大 ROBSTRIDE_MOTOR_COUNT
台(上限2)まで接続する構成を想定する。

指令(PC -> 本機, Rx_16Data)/帰還(本機 -> PC, Tx_16Data)のスロット割り当ては
robstride.cppの先頭コメントを参照。

Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once

#include "driver/gpio.h"
#include "driver/twai.h"
#include <Arduino.h>

// CANドライバ初期化(1Mbps、他モードと共通)
void robstrideInit();

// CAN送受信タスク本体(モータ内蔵の速度/位置ループに指令を送るのみ、
// ホスト側でのPIDは行わない)
void robstrideTask(void *pvParameters);
