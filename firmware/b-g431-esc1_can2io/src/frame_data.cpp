/*====================================================================
<frame_data.cpp>
・CANノードのローカルスロットデータ定義ファイル

Copyright (c) 2026 RRST-NHK-Project
SPDX-License-Identifier: MIT
====================================================================*/

#include "frame_data.hpp"

volatile int16_t Tx_16Data[Tx16NUM] = {0};
volatile int16_t Rx_16Data[Rx16NUM] = {0};
