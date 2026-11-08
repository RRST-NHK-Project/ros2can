/*====================================================================
<serial_task.hpp>
・シリアル通信関連のタスクと関数のヘッダーファイル
Copyright (c) 2025 RRST-NHK-Project
SPDX-License-Identifier: MIT
====================================================================*/

#pragma once

#include <Arduino.h>

// 関数のプロトタイプ宣言
void serialTask(void *); // 送受信を同時に行うタスク
void txTask(void *);     // 送信タスク（削除予定）
void rxTask(void *);     // 受信タスク（削除予定）
void send_frame();
void receive_frame();

// 自機宛ての有効フレームを最後に受信した時刻[ms](0=未受信)
uint32_t serialLastValidRxMs();

// Rx_16Data全体をロックしてoutへコピーする(outはRx16NUM個分)
void serialSnapshotRx(int16_t *out);
// Tx_16Data[start]からcount個をロックして書き込む
void serialWriteTx(int start, const int16_t *values, int count);