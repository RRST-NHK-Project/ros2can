/*====================================================================
<frame_data.hpp>
・シリアル通信のフレームデータ定義ヘッダーファイル
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once
#include <stdint.h>

#define Tx16NUM 24          // 送信するint16データの数
#define Rx16NUM 24          // 受信するint16データの数
#define CAN_IO_SLOT_COUNT 5 // CANモードで扱うIO値の数

extern volatile int16_t Tx_16Data[Tx16NUM];
/*
MODE_CAN_HOST/MODE_CAN時は下記のMD/SERVO/TR直接マッピングは使用しない。
24スロットを CAN バス上の各ノードへ 5スロットずつ分配する
(詳細は README.md の CAN Slot Mapping、CanIoRxData/CanIoTxData を参照)。

MODE_IO (非CAN, スタンドアロン) 時のみ以下のマッピングを使用:
0: デバッグ用
1~8: ENC1~8
9~16: SW1~8
17~23: 予備
*/

extern volatile int16_t Rx_16Data[Rx16NUM];
/*
MODE_CAN_HOST/MODE_CAN時は下記のMD/SERVO/TR直接マッピングは使用しない。
24スロットを CAN バス上の各ノードへ 5スロットずつ分配する
(詳細は README.md の CAN Slot Mapping、CanIoRxData/CanIoTxData を参照)。

MODE_IO (非CAN, スタンドアロン) 時のみ以下のマッピングを使用:
0: デバッグ用
1~8: MD1~8 (実機はDCモータ非搭載のため未使用)
9~16: SERVO1~8
17~23: TR1~7 (実機は非搭載のため未使用)
*/

extern volatile int16_t CanIoRxData[CAN_IO_SLOT_COUNT];
/*
実機はDCモータ非搭載のため MD は無し。
0: SERVO1
1: SERVO2
2: SERVO3
3~4: 予備(未使用)
(SERVOn は SWn とピン共有。config.hpp の MULTIn で入出力を切替)
*/

extern volatile int16_t CanIoTxData[CAN_IO_SLOT_COUNT];
/*
0~2: SW1~3
3~4: ENC1~2
*/
