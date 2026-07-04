/*====================================================================
<>
・使用しているMDとswが異なるので注意!!!
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/
#include "defs.hpp"
#include "driver/pcnt.h"
#include "frame_data.hpp"
#include "pin_ctrl_init.hpp"
#include <Arduino.h>

constexpr uint32_t CTRL_PERIOD_MS = 5; // ピン更新周期（ミリ秒）

void pid_control();
void pid_calculate();
void md_enc_init();
void pid_vel_control();
// エンコーダのDIPスイッチをすべてoffにすること
//  ================= TASK =================

// PID制御タスク馬渕385
void PID_Task(void *)
{
    TickType_t last_wake = xTaskGetTickCount();
    md_enc_init();
    while (1)
    {
        pid_control();
        // pid_vel_control();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CTRL_PERIOD_MS));
    }
}

float pid_calculate(float setpoint, float input, float &error_prev, float &integral,
                    float kp, float ki, float kd, float dt)
{
    float error = setpoint - input;
    integral += (error + error_prev) * dt;
    float derivative = (error - error_prev) / dt;
    error_prev = error;
    return kp * error + ki * integral + kd * derivative;
}

// 半田ミスったから使うの変更
void md_enc_init()
{
    // MDの方向ピンを出力に設定
    pinMode(MD1D, OUTPUT);
    pinMode(MD2D, OUTPUT);

    // PWMの初期化
    ledcSetup(0, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcSetup(1, MD_PWM_FREQ, MD_PWM_RESOLUTION);

    ledcAttachPin(MD1P, 0);
    ledcAttachPin(MD2P, 1);

    // プルアップを有効化
    gpio_set_pull_mode((gpio_num_t)ENC1_A, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC1_B, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC2_A, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC2_B, GPIO_PULLUP_ONLY);

    // パルスカウンタの設定
    pcnt_config_t pcnt_config1 = {};
    pcnt_config1.pulse_gpio_num = ENC1_A;
    pcnt_config1.ctrl_gpio_num = ENC1_B;
    pcnt_config1.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config1.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config1.pos_mode = PCNT_COUNT_INC;
    pcnt_config1.neg_mode = PCNT_COUNT_DEC;
    pcnt_config1.counter_h_lim = COUNTER_H_LIM;
    pcnt_config1.counter_l_lim = COUNTER_L_LIM;
    pcnt_config1.unit = PCNT_UNIT_0;
    pcnt_config1.channel = PCNT_CHANNEL_0;

    pcnt_config_t pcnt_config2 = {};
    pcnt_config2.pulse_gpio_num = ENC1_B;
    pcnt_config2.ctrl_gpio_num = ENC1_A;
    pcnt_config2.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config2.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config2.pos_mode = PCNT_COUNT_INC;
    pcnt_config2.neg_mode = PCNT_COUNT_DEC;
    pcnt_config2.counter_h_lim = COUNTER_H_LIM;
    pcnt_config2.counter_l_lim = COUNTER_L_LIM;
    pcnt_config2.unit = PCNT_UNIT_0;
    pcnt_config2.channel = PCNT_CHANNEL_1;

    pcnt_config_t pcnt_config3 = {};
    pcnt_config3.pulse_gpio_num = ENC2_A;
    pcnt_config3.ctrl_gpio_num = ENC2_B;
    pcnt_config3.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config3.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config3.pos_mode = PCNT_COUNT_INC;
    pcnt_config3.neg_mode = PCNT_COUNT_DEC;
    pcnt_config3.counter_h_lim = COUNTER_H_LIM;
    pcnt_config3.counter_l_lim = COUNTER_L_LIM;
    pcnt_config3.unit = PCNT_UNIT_1;
    pcnt_config3.channel = PCNT_CHANNEL_0;

    pcnt_config_t pcnt_config4 = {};
    pcnt_config4.pulse_gpio_num = ENC2_B;
    pcnt_config4.ctrl_gpio_num = ENC2_A;
    pcnt_config4.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config4.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config4.pos_mode = PCNT_COUNT_INC;
    pcnt_config4.neg_mode = PCNT_COUNT_DEC;
    pcnt_config4.counter_h_lim = COUNTER_H_LIM;
    pcnt_config4.counter_l_lim = COUNTER_L_LIM;
    pcnt_config4.unit = PCNT_UNIT_1;
    pcnt_config4.channel = PCNT_CHANNEL_1;

    // パルスカウンタの初期化
    pcnt_unit_config(&pcnt_config1);
    pcnt_unit_config(&pcnt_config2);
    pcnt_unit_config(&pcnt_config3);
    pcnt_unit_config(&pcnt_config4);

    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_pause(PCNT_UNIT_1);

    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);

    pcnt_counter_resume(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_1);

    // チャタリング防止のフィルターを有効化
    pcnt_filter_enable(PCNT_UNIT_0);
    pcnt_filter_enable(PCNT_UNIT_1);

    // フィルター値を設定
    pcnt_set_filter_value(PCNT_UNIT_0, PCNT_FILTER_VALUE);
    pcnt_set_filter_value(PCNT_UNIT_1, PCNT_FILTER_VALUE);

    // SW ピン初期化
    pinMode(SW3, INPUT_PULLUP);
    pinMode(SW4, INPUT_PULLUP);
}

// PID制御関数
void pid_control()
{
    //////////////定義
    float kp = 1.0; // 3.0// Rx_16Data[21];
    float ki = 0.0; // Rx_16Data[22];
    float kd = 0.3; // 0.1 Rx_16Data[23];
    float dt = CTRL_PERIOD_MS / 1000.0f;

    int16_t cnt0, cnt1;
    static int32_t total_cnt0 = 0;
    static int32_t total_cnt1 = 0;
    static float target_angle_cur[2] = {0.0f, 0.0f};

    pcnt_get_counter_value(PCNT_UNIT_0, &cnt0);
    pcnt_get_counter_value(PCNT_UNIT_1, &cnt1);

    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);

    total_cnt0 += cnt0;
    total_cnt1 -= cnt1;

    angle[0] = total_cnt0 * DEG_PER_COUNT;
    angle[1] = total_cnt1 * DEG_PER_COUNT;

    ////////////////////
    // 起動時の調整
    static bool first = true;
    if (first)
    {
        target_angle_cur[0] = angle[0];
        target_angle_cur[1] = angle[1];
        first = false;
    }
    // スイッチでゼロリセット

    if (Rx_16Data[5] == 1)
    {
        total_cnt0 = 0;
        angle[0] = 0.0f;
        target_angle_cur[0] = 0.0f;
        pos_integral[0] = 0.0f;
        pos_error_prev[0] = 0.0f;
    }
    if (Rx_16Data[6] == 1)
    {
        total_cnt1 = 0;
        angle[1] = 0.0f;
        target_angle_cur[1] = 0.0f;
        pos_integral[1] = 0.0f;
        pos_error_prev[1] = 0.0f;
    }

    // オーバーフロー対策が甘いがとりあえずそのまま送る
    Tx_16Data[1] = static_cast<int16_t>(angle[0]);
    Tx_16Data[2] = static_cast<int16_t>(angle[1]);
    Tx_16Data[11] = digitalRead(SW3);
    Tx_16Data[12] = digitalRead(SW4);

    // ===== 360度オーバーフロー処理 =====
    // if (Rx_16Data[3] > 300.0f)
    // {
    //     angle[0] -= 360.0f;
    //     target_angle_cur[0] -= 360.0f;
    // }
    // else if (Rx_16Data[4] > 300.0f)
    // {
    //     angle[1] -= 360.0f;
    //     target_angle_cur[1] -= 360.0f;
    // }

    // ===== 目標角ランプ生成 =====
    target_angle[0] = Rx_16Data[1];
    target_angle[1] = Rx_16Data[2];

    // ランプ後の目標角度
    constexpr float MAX_STEP_DEG = 0.2f;

    target_angle_cur[0] += constrain(target_angle[0] - target_angle_cur[0], -MAX_STEP_DEG, +MAX_STEP_DEG);
    target_angle_cur[1] += constrain(target_angle[1] - target_angle_cur[1], -MAX_STEP_DEG, +MAX_STEP_DEG);

    output[0] = pid_calculate(target_angle_cur[0], angle[0], pos_error_prev[0], pos_integral[0],
                              kp, ki, kd, dt);
    output[0] = constrain(output[0], -MD_PWM_MAX, MD_PWM_MAX);

    output[1] = pid_calculate(target_angle_cur[1], angle[1], pos_error_prev[1], pos_integral[1],
                              kp, ki, kd, dt);
    output[1] = constrain(output[1], -MD_PWM_MAX, MD_PWM_MAX);

    digitalWrite(MD1D, output[0] > 0 ? HIGH : LOW);
    digitalWrite(MD2D, output[1] > 0 ? HIGH : LOW);

    ledcWrite(0, abs(output[0]));
    ledcWrite(1, abs(output[1]));
}

// PID制御関数
void pid_vel_control()
{

    float kp_v = 0.8f;
    float kd_v = 0.0f;
    float dt = CTRL_PERIOD_MS / 1000.0f;

    int16_t cnt0, cnt1;
    static int32_t total_cnt0 = 0;
    static int32_t total_cnt1 = 0;
    static float target_angle_cur[2] = {0.0f, 0.0f};

    static float angle_prev[2] = {0.0f, 0.0f};
    static float vel_error_prev[2] = {0.0f, 0.0f};
    static float vel_integral[2] = {0.0f, 0.0f};

    float vel[2];
    // ===== 目標速度 rpm → deg/s =====
    float vel_target0 = Rx_16Data[1] * 6.0f; // rpm → deg/s
    float vel_target1 = Rx_16Data[2] * 6.0f;

    pcnt_get_counter_value(PCNT_UNIT_0, &cnt0);
    pcnt_get_counter_value(PCNT_UNIT_1, &cnt1);

    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);

    total_cnt0 += cnt0;
    total_cnt1 -= cnt1;

    angle[0] = total_cnt0 * DEG_PER_COUNT;
    angle[1] = total_cnt1 * DEG_PER_COUNT;

    // 角速度計算
    vel[0] = (angle[0] - angle_prev[0]) / dt;
    vel[1] = (angle[1] - angle_prev[1]) / dt;

    angle_prev[0] = angle[0];
    angle_prev[1] = angle[1];

    // オーバーフロー対策が甘いがとりあえずそのまま送る
    Tx_16Data[1] = static_cast<int16_t>(angle[0]);
    Tx_16Data[2] = static_cast<int16_t>(angle[1]);
    Tx_16Data[9] = static_cast<int16_t>(vel[0]);
    Tx_16Data[10] = static_cast<int16_t>(vel[1]);

    output[0] = pid_calculate(vel_target0, vel[0], vel_error_prev[0], vel_integral[0], kp_v, 0.0f, kd_v, dt);

    output[1] = pid_calculate(vel_target1, vel[1], vel_error_prev[1], vel_integral[1], kp_v, 0.0f, kd_v, dt);
    output[0] = constrain(output[0], -MD_PWM_MAX, MD_PWM_MAX);

    output[1] = constrain(output[1], -MD_PWM_MAX, MD_PWM_MAX);

    digitalWrite(MD1D, output[0] > 0 ? HIGH : LOW);
    digitalWrite(MD2D, output[1] > 0 ? HIGH : LOW);

    ledcWrite(0, abs(output[0]));
    ledcWrite(1, abs(output[1]));
    
}