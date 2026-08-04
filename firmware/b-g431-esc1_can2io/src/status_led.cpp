/*====================================================================
<status_led.cpp>
・通信アクティビティ表示用LEDの共通ヘルパー (xiao-esp32-s3_can2io/src/status_led.cpp 移植)

CANは指令フレームが数msおきに届くため、「受信のたびに点灯して一定時間保持する」
方式では次のフレームが保持時間内に届き続けてしまい、実質的に常時点灯（消灯する
暇がない）になってしまう。そのため「直近に通信があった間は固定レートで点滅し、
一定時間通信が途絶えたら消灯する」方式にしている(xiao側と同じロジック)。
通信量に関わらず点滅が視認でき、通信が止まればすぐ消灯するので状態が分かりやすい。

このボードはFreeRTOSタスクを使わずloop()のみで動作するため、xiao側にあった
volatile(複数タスクからのアクセスを想定)は不要だが、呼び出し元を揃える意味で
インターフェースは合わせてある。

オンボードLED(PC6, LED_BUILTIN)を使用する。
Copyright (c) 2026.
====================================================================*/

#include "status_led.hpp"
#include "config.hpp"
#include <Arduino.h>

namespace {
constexpr uint32_t LED_BLINK_INTERVAL_MS = 100; // 通信あり時の点滅周期
constexpr uint32_t LED_IDLE_TIMEOUT_MS = 300;   // この時間フレームが来なければ消灯

uint32_t g_last_activity_ms = 0;
bool g_has_activity = false;
uint32_t g_last_toggle_ms = 0;
bool g_led_on = false;
} // namespace

void statusLedInit() {
#if ENABLE_LED
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
#endif
}

void statusLedPulse() {
#if !ENABLE_LED
    return;
#else
    g_last_activity_ms = millis();
    g_has_activity = true;
#endif
}

void statusLedUpdate() {
#if !ENABLE_LED
    return;
#else
    const uint32_t now = millis();

    if (!g_has_activity || now - g_last_activity_ms >= LED_IDLE_TIMEOUT_MS) {
        // 通信が途絶えている間は消灯を維持する
        if (g_led_on) {
            digitalWrite(LED_BUILTIN, LOW);
            g_led_on = false;
        }
        return;
    }

    if (now - g_last_toggle_ms >= LED_BLINK_INTERVAL_MS) {
        g_led_on = !g_led_on;
        digitalWrite(LED_BUILTIN, g_led_on ? HIGH : LOW);
        g_last_toggle_ms = now;
    }
#endif
}
