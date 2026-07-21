/*====================================================================
<can_task.hpp>
・CANノード通信タスクAPI (STM32G4 FDCAN, xiao-esp32-s3_can2io ホスト互換)

Copyright (c) 2026.
====================================================================*/

#pragma once

#include <stdint.h>

// FDCANペリフェラルの初期化(GPIO/クロック/ビットタイミング/フィルタ)を行う。
void canTaskInit();

// loop()から毎回呼ぶこと。受信フレームをRx_16Dataへ反映し、
// 一定周期でTx_16Dataを送信する(非ブロッキング)。
void canTaskUpdate();

// 最後にホストからのCANフレームを受信した時刻 (millis())。
uint32_t canLastRxMs();

// デバッグ用シリアルモニター。一定周期(CAN_DIAG_PERIOD_MS)でFDCANの状態
// (Bus-Off/エラーカウンタ/最終受信からの経過時間/現在の指令値)をSerialへ出力する。
// canTaskUpdate()から自動的に呼ばれるため、通常はloop()側で明示的に呼ぶ必要はない。
void canTaskPrintDiagnostics();
