/*====================================================================
<damiao.hpp>
・MODE_DAMIAO(Damiao DM2325+DM3520(ESC) ドライバ)のヘッダーファイル

MODE_CUBEMARS/MODE_ROBOMASと同様、xiao-esp32-s3_can2ioの他モード
(MODE_IO/MODE_CAN/MODE_CAN_HOST)が使うノード/スロット分配プロトコルとは
別系統で、Damiao公式CANプロトコル(VEL/POS_VELモード)を直接喋る独立
デバイスとして動作する。1マイコン(1バス)には最大4台。台数とCAN IDは
config.hppの DAMIAO_MOTORS(表の1行=1台)で決める。

指令(PC -> 本機, Rx_16Data)/帰還(本機 -> PC, Tx_16Data)のスロット割り当ては
config.hppのDamiao関連セクション、およびdamiao.cppの先頭コメントを参照。

Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once

#include "driver/gpio.h"
#include "driver/twai.h"
#include <Arduino.h>

// CANドライバ初期化(1Mbps、他モードと共通)
void damiaoInit();

// CAN送受信タスク本体(アクチュエータ内蔵のクローズドループに指令を送るのみ、
// ホスト側での速度/位置PIDは行わない)
void damiaoTask(void *pvParameters);
