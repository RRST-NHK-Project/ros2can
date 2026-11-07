/*====================================================================
<twai_util.cpp>
・独立デバイス型のモータドライバ(MODE_DAMIAO / MODE_ROBSTRIDE)が共通で使う
  CAN(TWAI)の補助関数の実装。説明は twai_util.hpp を参照。
====================================================================*/

#include "twai_util.hpp"
#include "config.hpp"
#include "defs.hpp"

namespace {

uint32_t g_rx_queue_len = 0;      // 入れ直しの時に同じ設定で入れるため覚えておく
uint32_t g_send_fail_count = 0;
uint32_t g_reinstall_count = 0;

} // namespace

bool twaiInstall(uint32_t rx_queue_len) {
    g_rx_queue_len = rx_queue_len;
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
    if (rx_queue_len > 0) {
        g_config.rx_queue_len = rx_queue_len;
    }
    // 1Mbps固定。Damiao(DM3520)・RobStride(EL05)とも既定は1Mbpsで、ros2canの他モードとも
    // 同じ(以前damiao.cppは #if CAN_BITRATE_1MBPS で切り替えていたが、ros2canのconfig.hppに
    // このマクロが無く、未定義=0扱いで500kbpsになっていた不具合があった。2026-10-14)。
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        return false;
    }
    if (twai_start() != ESP_OK) {
        twai_driver_uninstall();
        return false;
    }
    return true;
}

// CAN未接続・相手が無応答のままだとBus-Offになるが、TWAIドライバは自動で復帰しない
// (ros2canのcan_task.cpp/cubemars.cppと同じ対策)。
//
// さらに、ESP-IDF TWAIドライバの既知のバグ(espressif/esp-idf#19068)への対策も兼ねる:
// 送信の流れがレースで止まったままになることがあり、その場合エラーカウンタは0のまま・
// Bus-Offにもならず、twai_transmit()がタイムアウトを返し続けるだけになる(エラー状態の
// 監視だけでは気付けない)。送信待ちの数(msgs_to_tx)が一定時間変わらなければ、止まった
// とみなしてドライバを入れ直す(待っていたフレームは捨てる)。
void twaiRecoverIfNeeded() {
    constexpr uint32_t CHECK_PERIOD_MS = 100;
    constexpr uint32_t STUCK_THRESHOLD_MS = 300; // msgs_to_txが変わらないまま許す時間
    static uint32_t last_check_ms = 0;
    static uint32_t last_msgs_to_tx_change_ms = 0;
    static uint32_t last_msgs_to_tx = 0;

    const uint32_t now_ms = millis();
    if (now_ms - last_check_ms < CHECK_PERIOD_MS) {
        return;
    }
    last_check_ms = now_ms;

    twai_status_info_t status{};
    if (twai_get_status_info(&status) != ESP_OK) {
        return;
    }
    if (status.state == TWAI_STATE_BUS_OFF) {
        twai_initiate_recovery();
        return;
    }
    if (status.state == TWAI_STATE_STOPPED) {
        twai_start();
        return;
    }
    if (status.msgs_to_tx != last_msgs_to_tx) {
        last_msgs_to_tx = status.msgs_to_tx;
        last_msgs_to_tx_change_ms = now_ms;
        return;
    }
    if (status.msgs_to_tx > 0 && (now_ms - last_msgs_to_tx_change_ms) > STUCK_THRESHOLD_MS) {
        twai_stop();
        twai_driver_uninstall();
        twaiInstall(g_rx_queue_len);
        g_reinstall_count++;
        last_msgs_to_tx = 0;
        last_msgs_to_tx_change_ms = now_ms;
    }
}

bool twaiSend(const twai_message_t &msg, uint32_t timeout_ms) {
    if (twai_transmit(&msg, pdMS_TO_TICKS(timeout_ms)) != ESP_OK) {
        g_send_fail_count++;
        return false;
    }
    return true;
}

uint32_t twaiSendFailCount() {
    return g_send_fail_count;
}

uint32_t twaiReinstallCount() {
    return g_reinstall_count;
}
