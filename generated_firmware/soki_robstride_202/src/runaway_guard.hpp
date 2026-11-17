/*====================================================================
<runaway_guard.hpp>
・モータの暴走の見張り(1台分)。MODE_DAMIAO(damiao.cpp)が使う。

  2026-11-09と11-17に、DM3519(Damiao)が指令と逆向きに全速(約−40rad/s)で回り続け、
  モータの中から白い煙が出た(電流の向きと回転の向きの読み取りが食い違った時の暴走と
  考えられる。指令に従わないので、速度の上限を下げても止まらない)。そこで、次のどちらかが
  RUNAWAY_TIME_MS 続いたら「暴走」とみなす。見つけたら呼び出し側が脱力させて止めたままにする。

  1. 逆向き: 速度制御で、実際の速度が指令と逆向き(または指令がほぼ0)なのに
     RUNAWAY_MIN_SPEED より速く、しかも減速していない
     (|速度| が RUNAWAY_PROGRESS 以上下がったら、減速中とみなして時計を戻す)。
     正常な減速(前進から後退への切り替えなど)は約100rad/s²で、0.2秒あれば20rad/sも
     下がるので引っかからない(11/9の実機の記録で確認)。
  2. 速すぎる: 実際の速度が上限 + OVERSPEED_MARGIN を超えたまま
     (上限 = 速度制御はモータの上限、角度制御は速さの上限)。

  ESP32に依存しない書き方にしてあるので、PCでもコンパイルして試せる。
  PC側(damiao_ctrl/include/damiao_ctrl/runaway_guard.hpp)にも同じ考え方の見張りがある。
====================================================================*/

#pragma once

#include <cmath>
#include <cstdint>

struct RunawayGuard {
    static constexpr float RUNAWAY_MIN_SPEED = 5.0f;  // [rad/s] これ以下の逆向きは見ない
    static constexpr float RUNAWAY_PROGRESS = 1.0f;   // [rad/s] これ以上下がれば減速中
    static constexpr float OVERSPEED_MARGIN = 5.0f;   // [rad/s] 上限をこれだけ超えたら速すぎ
    static constexpr uint32_t RUNAWAY_TIME_MS = 200;  // これだけ続いたら暴走

    bool wrong_watching = false;
    float wrong_ref_speed = 0.0f; // 見張り始め(または最後に減速した時)の |速度|
    uint32_t wrong_since_ms = 0;
    bool over_watching = false;
    uint32_t over_since_ms = 0;

    void reset() {
        wrong_watching = false;
        over_watching = false;
    }

    // 制御周期ごとに呼ぶ。暴走ならtrue。
    //   vel_mode: 速度制御中ならtrue(角度制御ならfalse。逆向きの見張りは速度制御だけ)
    //   target:   速度制御の指令[rad/s](角度制御では使わない)
    //   limit:    速度の上限[rad/s]
    //   vel:      実際の速度[rad/s](モータの返事)
    bool update(bool vel_mode, float target, float limit, float vel, uint32_t now_ms) {
        const float speed = std::fabs(vel);

        // 2. 速すぎる
        if (speed > limit + OVERSPEED_MARGIN) {
            if (!over_watching) {
                over_watching = true;
                over_since_ms = now_ms;
            } else if (now_ms - over_since_ms >= RUNAWAY_TIME_MS) {
                return true;
            }
        } else {
            over_watching = false;
        }

        // 1. 逆向き(または指令がほぼ0)なのに速く、減速していない
        const bool wrong = vel_mode && speed > RUNAWAY_MIN_SPEED &&
                           (vel * target < 0.0f || std::fabs(target) < 0.5f);
        if (!wrong) {
            wrong_watching = false;
            return false;
        }
        if (!wrong_watching || speed < wrong_ref_speed - RUNAWAY_PROGRESS) {
            // 見張り始め、または減速している -> 時計を戻す
            wrong_watching = true;
            wrong_ref_speed = speed;
            wrong_since_ms = now_ms;
            return false;
        }
        return now_ms - wrong_since_ms >= RUNAWAY_TIME_MS;
    }
};
