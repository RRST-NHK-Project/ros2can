/*====================================================================
<pid_task.hpp>
・PID制御タスク関連のヘッダーファイル
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once

#include <Arduino.h>

// 関数のプロトタイプ宣言
void PID_Task(void *); // PID制御タスク
void pid_control();
void md_enc_init();