/*====================================================================
<robomas.hpp>
・MODE_ROBOMAS(DJI RoboMasterシリーズ CANドライバ)のヘッダーファイル

MODE_ROBOMASは、xiao-esp32-s3_can2ioの他モード(MODE_IO/MODE_CAN/MODE_CAN_HOST)
が使うノード/スロット分配プロトコル(CAN 500kbps)とは別系統で、DJIのCANプロトコル
(1Mbps固定, ID固定)を直接喋る独立デバイスとして動作する。
1マイコン(1バス)には同一機種のロボマスを最大4台まで接続する構成を想定する。

指令(PC -> 本機, Rx_16Data)/帰還(本機 -> PC, Tx_16Data)のスロット割り当ては
robomas.cppの先頭コメント、およびREADME.mdを参照。

Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once

#include "driver/gpio.h"
#include "driver/twai.h"
#include <Arduino.h>

// CANドライバ初期化(1Mbps)
void robomasInit();

// 速度制御ループ + CAN送受信を行うタスク本体
void robomasTask(void *pvParameters);
