/*====================================================================
<pin_ctrl_task.hpp>
・ピン操作関連の関数とタスクのヘッダーファイル
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once

#include <Arduino.h>

// 関数のプロトタイプ宣言
void Input_Task(void *);      // 入力タスク
void Output_Task(void *);     // 出力タスク
void IO_Task(void *);         // 入出力タスク
void ROBOMAS_IO_Task(void *); // ロボマス入出力タスク
void MD_Output();
void Servo_Output();
void TR_Output();
void ENC_Input();
void SW_Input();
void IO_MD_Output();
void IO_TR_Output();
void IO_ENC_Input();
void IO_SW_Input();
