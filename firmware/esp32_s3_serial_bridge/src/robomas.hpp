/*====================================================================
<robomas.hpp>
・ロボマス関連のヘッダーファイル
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once

#include "defs.hpp"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "frame_data.hpp"
#include <Arduino.h>

void send_cur_all(float cur_array[NUM_MOTOR]);
void send_cur_c610(float cur_array[NUM_MOTOR]);

// 関数のプロトタイプ宣言
void M3508_Task(void *pvParameters);
void M2006_Task(void *pvParameters);
void M3508_RX(void *);
void robomas_init();

void twai_receive_feedback();
