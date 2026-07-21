#include "can_task.hpp"
#include "config.hpp"
#include "frame_data.hpp"

#include <Arduino.h>
#include <SimpleFOC.h>
#include <math.h>

#include "stm32g4xx_hal.h"

// ======================================================
// TIM4 HARD ENCODER
// ======================================================
TIM_HandleTypeDef htim4;

// ======================================================
// SimpleFOC Sensor (TIM4 wrapper)
// ======================================================
class TIM4Sensor : public Sensor {
public:
    int32_t cpr;

    TIM4Sensor(int32_t cpr) {
        this->cpr = cpr;
    }

    void init() override {
        prev = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
        full_rotations = 0;
    }

    float getSensorAngle() override {
        int16_t now = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);

        int16_t diff = now - prev;
        prev = now;

        // 16bitオーバーフロー補正
        if (diff > 32767)
            diff -= 65536;
        if (diff < -32768)
            diff += 65536;

        acc += diff;

        // ★ここが重要：連続角（ラップ禁止）
        return (float)acc / (float)cpr * _2PI;
    }

private:
    int16_t prev = 0;
    int32_t acc = 0;
};

TIM4Sensor encoder(ENCODER_CPR);

// ======================================================
// SimpleFOC Objects
// ======================================================
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);

BLDCDriver6PWM driver = BLDCDriver6PWM(
    A_PHASE_UH, A_PHASE_UL,
    A_PHASE_VH, A_PHASE_VL,
    A_PHASE_WH, A_PHASE_WL);

LowsideCurrentSense currentSense =
    LowsideCurrentSense(0.003f, -64.0f / 7.0f,
                        A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// ======================================================
// Control state (CAN bridge)
// ======================================================
static float estimated_velocity = 0;
static int16_t manual_velocity_prev_count = 0;
static uint32_t manual_velocity_prev_us = 0;

static int16_t saturate_to_i16(float value) {
    if (value > 32767.0f) {
        return 32767;
    }
    if (value < -32768.0f) {
        return -32768;
    }
    return (int16_t)lroundf(value);
}

static void reset_manual_velocity_estimate() {
    manual_velocity_prev_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    manual_velocity_prev_us = micros();
    estimated_velocity = 0;
}

static void update_manual_velocity_estimate() {
    const uint32_t now_us = micros();
    const uint32_t dt_us = now_us - manual_velocity_prev_us;
    if (dt_us == 0) {
        return;
    }

    const int16_t now_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    int32_t diff = (int32_t)now_count - (int32_t)manual_velocity_prev_count;
    manual_velocity_prev_count = now_count;
    manual_velocity_prev_us = now_us;

    if (diff > 32767) {
        diff -= 65536;
    }
    if (diff < -32768) {
        diff += 65536;
    }

    const float dt = (float)dt_us * 1.0e-6f;
    estimated_velocity = ((float)diff / (float)encoder.cpr) * _2PI / dt;
}

// ======================================================
// TIM4 init (FINAL STABLE)
// ======================================================
void tim4_init() {
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP; // 安定優先
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &gpio);

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 0xFFFF;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    TIM_Encoder_InitTypeDef enc = {0};
    enc.EncoderMode = TIM_ENCODERMODE_TI12;

    enc.IC1Polarity = TIM_ICPOLARITY_RISING;
    enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC1Prescaler = TIM_ICPSC_DIV1;
    enc.IC1Filter = 3;

    enc.IC2Polarity = TIM_ICPOLARITY_RISING;
    enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC2Prescaler = TIM_ICPSC_DIV1;
    enc.IC2Filter = 3;

    HAL_TIM_Encoder_Init(&htim4, &enc);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
}

// ======================================================
// helpers
// ======================================================
static void apply_velocity(float v) {
    motor.controller = MotionControlType::velocity;
    motor.target = v;
}

// ======================================================
// RX command handling
// ======================================================
// robomas (MODE_ROBOMAS) 互換: 速度制御のみ、生rpm値。CANが途絶えても最後の
// target_velocityを保持し続ける(フェイルセーフ無し)。
static void apply_rx() {
    const float vel_rpm = (float)Rx_16Data[RX_TARGET_VELOCITY];
    apply_velocity(vel_rpm * RPM_TO_RAD_S);
}

// ======================================================
// telemetry
// ======================================================
static void update_tx() {
    update_manual_velocity_estimate();

    const float angle = encoder.getAngle();
    const float foc_velocity_rpm = motor.shaft_velocity * RAD_S_TO_RPM;

    Tx_16Data[TX_ANGLE] = saturate_to_i16(angle * 180.0f / PI / ANGLE_TX_SCALE);
    Tx_16Data[TX_VELOCITY] = saturate_to_i16(foc_velocity_rpm);
    Tx_16Data[TX_CURRENT_Q] = saturate_to_i16(motor.current.q / CURRENT_TX_SCALE);
}

// ======================================================
// SETUP
// ======================================================
void setup() {
    canTaskInit();

    // ===== TIM4 =====
    tim4_init();
    encoder.init();
    reset_manual_velocity_estimate();

    // ===== driver =====
    driver.voltage_power_supply = DEFAULT_VOLTAGE_SUPPLY;
    driver.init();

    motor.linkDriver(&driver);

    // ===== sensor =====
    motor.linkSensor(&encoder);

    // ===== current sense =====
    currentSense.linkDriver(&driver);
    currentSense.init();
    currentSense.skip_align = true;
    motor.linkCurrentSense(&currentSense);

    // ===== config =====
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.torque_controller = TorqueControlType::foc_current;

    motor.voltage_limit = DEFAULT_VOLTAGE_LIMIT;
    motor.velocity_limit = DEFAULT_VELOCITY_LIMIT;
    motor.current_limit = DEFAULT_CURRENT_LIMIT;

    motor.PID_velocity.P = VELOCITY_PID_P;
    motor.PID_velocity.I = VELOCITY_PID_I;
    motor.PID_velocity.D = VELOCITY_PID_D;
    motor.LPF_velocity.Tf = VELOCITY_LPF_TF;
    motor.PID_velocity.output_ramp = VELOCITY_PID_OUTPUT_RAMP;

    // torque_controller=foc_current のため、速度モードでも内側の電流ループに
    // このゲインが使われる(角度モードは廃止したためP_angleは未使用)。
    motor.PID_current_q.P = CURRENT_PID_P;
    motor.PID_current_q.I = CURRENT_PID_I;
    motor.PID_current_q.D = CURRENT_PID_D;
    motor.PID_current_d.P = CURRENT_PID_P;
    motor.PID_current_d.I = CURRENT_PID_I;
    motor.PID_current_d.D = CURRENT_PID_D;

    motor.init();
    motor.initFOC();

    apply_velocity(0);
}

// ======================================================
// LOOP
// ======================================================
void loop() {
    motor.loopFOC();

    canTaskUpdate();
    apply_rx();

    motor.move();

    update_tx();
}
