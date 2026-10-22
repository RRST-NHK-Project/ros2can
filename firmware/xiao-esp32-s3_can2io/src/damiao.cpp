/*====================================================================
<damiao.cpp>
・Damiao DM2325(モータ) + DM3520(ESC) 用CANドライバの実装ファイル

達妙(Damiao)公式Python実装 (github.com/cmjang/DM_Control_Python,
DM_CAN.py) のCAN ID体系・バイトパッキングを一次情報として実装した、
VEL(速度)モード・POS_VEL(位置+速度)モードのドライバ。MITモードは
Kp/Kd調整が必要で初回動作確認に不向きなため未実装(将来必要になれば
cubemars.cppのsendMitCommand()と同じ考え方で追加できる)。

CANプロトコル (標準11bit ID, 1Mbps)。2026-10-01、ユーザーが過去に実機で
動作確認済みのArduinoリファレンス実装(Arduino Mega + MCP2515 + PS4USB)
を一次情報として全面的に見直した:
  Enable/Disable/ClearError/原点設定 (ID = モータのCAN IDそのまま, DLC=8):
    data = [0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, cmd]
    cmd: 0xFC=Enable, 0xFD=Disable, 0xFB=ClearError, 0xFE=原点設定(SetZero)
  モードレジスタ書き込み (ID = 0x7FF固定, DLC=8):
    data = [motor_id下位8bit, motor_id上位3bit, 0x55, RID(=10/CMODE), value(int32 LE)]
    value: 2=POS_VELモード, 3=VELモード(要検証)。
    制御フレームを送る前に、このレジスタ書き込みでモータ側の受付モードを
    切り替えておかないと、モータは制御フレームに一切反応しない
    (今回最大の見落としだった箇所)。
  VELモード (ID = 0x200 + モータCAN ID, DLC=8):
    data[0:4] = 目標速度[rad/s] (float32 little-endian), data[4:8] = 0
  POS_VELモード (ID = 0x100 + モータCAN ID, DLC=8):
    data[0:4] = 目標位置[rad] (float32 LE), data[4:8] = 目標速度[rad/s] (float32 LE)
  フィードバック (ID = Master ID(実機は0x000)。モータIDではない点に注意。
                  どのモータかはbyte0の下位4bitで判別する。DLC=8。MIT/Cheetah系で共通の
                  固定小数点ステータスフレーム。config.hppのDAMIAO_P_MAX/
                  V_MAX/T_MAX(モータのレジスタRID21/22/23と一致させること)
                  でデコードする):
    byte0: [エラーコード(上位4bit)][モータID下位4bit]
    byte1-2: 位置 (uint16, 0~65535 -> -P_MAX~P_MAX)
    byte3, byte4上位4bit: 速度 (uint12, 0~4095 -> -V_MAX~V_MAX)
    byte4下位4bit, byte5: トルク (uint12, 0~4095 -> -T_MAX~T_MAX)
    byte6: MOS温度[degC], byte7: ロータ温度[degC]

スロット割り当て (damiao_ctrl/include/damiao_ctrl/slot_layout.hppと一致
させること。6スロット/モータ、モータnのオフセット = (n-1)*6):
  Rx_16Data (PC -> 本機, 指令):
    +0: mode_word (0=DISABLE, 1=VEL, 2=POS_VEL)。全ゼロ=DISABLEが安全な
        既定値(Damiaoモータは明示的なEnableコマンドを送るまでCAN指令を
        無視するため)。
    +1: target (VELなら0.01 rad/s/LSB、POS_VELなら0.001 rad/LSB)
    +2: vel_limit (0.01 rad/s/LSB。POS_VELモードでのみ使用)
  Tx_16Data (本機 -> PC, 帰還):
    +0: モータ状態 (0=Disable, 1=Enable, 5/6/8-E=故障コード, 13(D)=通信喪失)
    +1: 帰還フレームの受信回数(増え続けていればモータが応答している)
    +3: position [0.001 rad/LSB]
    +4: velocity [0.01 rad/s/LSB]
  速度だけ0.01単位なのは、最高速度(無負荷560rpm=58.6rad/s)がint16の0.001単位の
  上限(32.767)を超えるため(2026-10-15変更)。
    +5: torque   [0.001 N・m/LSB]
====================================================================*/

#include "damiao.hpp"
#include "config.hpp"
#include "defs.hpp"
#include "frame_data.hpp"
#include "serial_task.hpp"
#include <Arduino.h>
#include <algorithm>
#include <cmath>
#include <cstring>

namespace {

constexpr int SLOTS_PER_MOTOR = 6;
// スロット1LSBあたりの物理量。PC側(damiao_ctrl/slot_layout.hpp)と一致させること。
constexpr double POS_SCALE = 0.001;   // rad
constexpr double VEL_SCALE = 0.01;    // rad/s (最高58.6rad/sを表すため0.01単位)
constexpr double TORQUE_SCALE = 0.001; // N・m

// PC側(Rx_16Data)のmode_word enum値。damiao_ctrl/slot_layout.hppと一致させること。
constexpr int16_t MODE_DISABLED = 0;
constexpr int16_t MODE_VEL = 1;
constexpr int16_t MODE_POS_VEL = 2;

// Damiao制御コマンド (公式DM_CAN.pyのenable()/disable()/set_zero_position()と同一)。
// 2026-10-01、ユーザーが過去に実機で動作確認済みのArduinoリファレンス
// 実装(Arduino Mega + MCP2515 + PS4USB)で確認: Enable/Disable/ClearError
// の宛先CAN IDはmotor_can_idそのまま(オフセット無し)。以前「公式STM32
// サンプル(DM3519一拖四)」を参考にmotor_can_id+モードオフセットへ変更
// したが、DM3519は別系統の製品であり誤りだった。
constexpr uint8_t CMD_ENABLE = 0xFC;
constexpr uint8_t CMD_DISABLE = 0xFD;
constexpr uint8_t CMD_CLEAR_ERROR = 0xFB;
constexpr uint8_t CMD_SET_ZERO = 0xFE;

// パラメータ書き込み(レジスタ)コマンド。固定CAN ID 0x7FFへ
// {motor_id_lo, motor_id_hi, 0x55, RID, d0,d1,d2,d3} を送る。
// RID_CMODE(=10)に制御モード値を書き込むことで、モータ側の受付モードを
// 切り替える。この書き込みを行わずに制御フレームだけ送っても、モータは
// 反応しない(今回のバグの本命)。
constexpr uint8_t RID_CMODE = 10;
// 通信タイムアウト(1あたり50us, 2026-10-14実測)。0=無効が工場出荷値。
constexpr uint8_t RID_TIMEOUT = 9;
constexpr uint32_t TIMEOUT_UNIT_US = 50;
constexpr int32_t CMODE_POS_VEL = 2;
constexpr int32_t CMODE_VEL = 3; // 2026-10-14 実機確認済み(0x200+id, float32速度)

const uint16_t kMotorCanId[DAMIAO_MOTOR_COUNT] = {
    DAMIAO_MOTOR_ID_1,
};

bool g_enabled[DAMIAO_MOTOR_COUNT] = {false};
// 現在Enable済みのモード(MODE_VEL/MODE_POS_VEL)。モードが変わったら
// 一旦Disable+再初期化(モードレジスタ書き込み)してからEnableし直す。
int16_t g_enabled_mode[DAMIAO_MOTOR_COUNT] = {MODE_DISABLED};
// 帰還で最後に受け取った状態コード(0=Disable, 1=Enable, それ以外=故障)。
uint8_t g_last_state[DAMIAO_MOTOR_COUNT] = {0};

void logTransmitFailure(const char *message);

bool installTwaiDriver() {
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
    // 1Mbps固定(2026-10-14実機確認)。以前は #if CAN_BITRATE_1MBPS で切り替えて
    // いたが、ros2can版のconfig.hppにはこのマクロが無く、未定義=0扱いで
    // 500kbpsになってDM3520がACKを返さない不具合になっていた。
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

// CAN未接続/ESC無応答が続くとBus-Offへ遷移するが、TWAIドライバは自動復帰
// しない。ros2canのcan_task.cpp/cubemars.cppと同じ対策。
//
// さらに、既知のESP-IDF TWAIドライバのバグ(espressif/esp-idf#19068)への
// 対策も兼ねる: TXパイプラインがレースコンディションで永久にスタックする
// ことがあり、この場合エラーカウンタは0のまま・bus-offにもならず、
// twai_transmit()がESP_ERR_TIMEOUTを返し続けるだけになる(=タスク側の
// エラーカウンタ監視だけでは検知できない)。msgs_to_txが一定時間変化
// しない状態が続いたら、詰まっていると判断してドライバを再インストール
// する。
void recoverBusIfNeeded() {
    constexpr uint32_t CHECK_PERIOD_MS = 100;
    constexpr uint32_t STUCK_THRESHOLD_MS = 300; // msgs_to_txが変化しないまま許容する時間
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
        // 送信キューに滞留したまま進まない = TXパイプラインのスタック。
        // ドライバを再インストールして復旧する(滞留していたフレームは失われる)。
        logTransmitFailure("[ERR] damiao: TWAI TX stuck, reinstalling driver");
        twai_stop();
        twai_driver_uninstall();
        installTwaiDriver();
        last_msgs_to_tx = 0;
        last_msgs_to_tx_change_ms = now_ms;
    }
}

// twai_transmit失敗ログはserialTaskのバイナリフレームと同じUARTに出るため、
// 頻発すると混入してホスト側のフレーム同期を壊す。間隔を絞って出す。
void logTransmitFailure(const char *message) {
    constexpr uint32_t LOG_THROTTLE_MS = 500;
    static uint32_t last_log_ms = 0;
    const uint32_t now_ms = millis();
    if (now_ms - last_log_ms < LOG_THROTTLE_MS) {
        return;
    }
    last_log_ms = now_ms;
    Serial.println(message);
}

void sendStandard(uint32_t can_id, const uint8_t *data, uint8_t len) {
    twai_message_t tx{};
    tx.identifier = can_id;
    tx.extd = 0; // Damiaoは標準11bit ID
    tx.rtr = 0;
    tx.data_length_code = len;
    for (uint8_t i = 0; i < len; i++) {
        tx.data[i] = data[i];
    }
    if (twai_transmit(&tx, pdMS_TO_TICKS(20)) != ESP_OK) {
        logTransmitFailure("[ERR] damiao: twai_transmit failed");
    }
}

void sendControlCmd(uint16_t motor_can_id, uint8_t cmd) {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd};
    sendStandard(motor_can_id, data, 8);
}

// レジスタ書き込みコマンド(固定ID 0x7FF)。
void writeRegister(uint16_t motor_can_id, uint8_t rid, int32_t value) {
    uint8_t can_id_l = motor_can_id & 0xFF;
    uint8_t can_id_h = (motor_can_id >> 8) & 0x07;
    uint8_t data[8] = {can_id_l, can_id_h, 0x55, rid, 0, 0, 0, 0};
    std::memcpy(&data[4], &value, 4);
    sendStandard(0x7FF, data, 8);
}

void floatToBytesLE(float value, uint8_t *out4) {
    std::memcpy(out4, &value, 4);
}

void sendVel(uint16_t motor_can_id, float vel_radps) {
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    floatToBytesLE(vel_radps, data);
    sendStandard(0x200 + motor_can_id, data, 8);
}

void sendPosVel(uint16_t motor_can_id, float pos_rad, float vel_radps) {
    uint8_t data[8];
    floatToBytesLE(pos_rad, data);
    floatToBytesLE(vel_radps, data + 4);
    sendStandard(0x100 + motor_can_id, data, 8);
}

// モード切替+Enableの初期化シーケンス。実機で動作確認済みのArduino
// リファレンス実装(initMotor())と同じ順序: Disable -> ClearError ->
// モードレジスタ書き込み -> Enable。各コマンド間に短い待ち時間を入れる
// (リファレンスは200-500ms待っていたが、ここでは起動時の一度きりの
// 処理なので50msに短縮)。
// 通信タイムアウトもここで毎回書き込む(RAMのみ。電源を切ると0に戻るため)。
// 注: この関数はモータ1台あたり約250ms処理を止める。複数台にする場合、
// 他のモータへの送信がDAMIAO_MOTOR_TIMEOUT_MSを超えて途切れ、そのモータが
// 状態D(通信喪失)で止まるので、その時は待ち時間かタイムアウト値を見直すこと。
void initAndEnable(uint16_t motor_can_id, int32_t cmode_value) {
    sendControlCmd(motor_can_id, CMD_DISABLE);
    delay(50);
    sendControlCmd(motor_can_id, CMD_CLEAR_ERROR);
    delay(50);
    writeRegister(motor_can_id, RID_TIMEOUT,
                  (int32_t)(DAMIAO_MOTOR_TIMEOUT_MS * 1000UL / TIMEOUT_UNIT_US));
    delay(50);
    writeRegister(motor_can_id, RID_CMODE, cmode_value);
    delay(50);
    sendControlCmd(motor_can_id, CMD_ENABLE);
    delay(50);
}

// -------- CAN送信 (指令 -> DM3520) -------- //

// PCからの有効フレームがDAMIAO_CMD_TIMEOUT_MS以上途絶えていればtrue。
// DM3520自身の通信タイムアウト(RID9, initAndEnableで設定)はESP32<->モータ間の
// 途絶用なので、PC<->ESP32間の途絶はここで止める(CubeMars版の
// CUBEMARS_CMD_TIMEOUT_MSと同じ考え方)。
bool commandLinkLost() {
    const uint32_t last_ms = serialLastValidRxMs();
    if (last_ms == 0) {
        return true; // 起動後まだ一度も指令を受け取っていない
    }
    return (millis() - last_ms) > DAMIAO_CMD_TIMEOUT_MS;
}

void sendCommands() {
    const bool link_lost = commandLinkLost();
    for (int m = 0; m < DAMIAO_MOTOR_COUNT; m++) {
        const int off = m * SLOTS_PER_MOTOR;
        // 指令途絶中はPCの最後の指令に関わらずDisable扱いにする。
        const int16_t mode = link_lost ? MODE_DISABLED : Rx_16Data[off + 0];
        const float target =
            (float)(Rx_16Data[off + 1] * (mode == MODE_VEL ? VEL_SCALE : POS_SCALE));
        const float vel_limit = (float)(Rx_16Data[off + 2] * VEL_SCALE);
        const uint16_t motor_can_id = kMotorCanId[m];

        if (mode != MODE_VEL && mode != MODE_POS_VEL) {
            // 未知の値も含め、DISABLE以外の全てを安全側(無効化)として扱う。
            if (g_enabled[m]) {
                sendControlCmd(motor_can_id, CMD_DISABLE);
                g_enabled[m] = false;
                g_enabled_mode[m] = MODE_DISABLED;
            }
            continue;
        }

        if (!g_enabled[m] || g_enabled_mode[m] != mode) {
            // 初回Enable、またはモードが切り替わった場合は、
            // Disable -> ClearError -> モードレジスタ書き込み -> Enable の
            // 初期化シーケンスをやり直す(実機確認済みのArduinoリファレンス
            // 実装と同じ手順)。これを行わないとモータが制御フレームに
            // 一切反応しない。
            const int32_t cmode_value = (mode == MODE_POS_VEL) ? CMODE_POS_VEL : CMODE_VEL;
            initAndEnable(motor_can_id, cmode_value);
            g_enabled[m] = true;
            g_enabled_mode[m] = mode;
        }

        if (mode == MODE_VEL) {
            sendVel(motor_can_id, target);
        } else { // MODE_POS_VEL
            sendPosVel(motor_can_id, target, vel_limit);
        }
    }
}

// Disable中に現在位置を読むため、Disableコマンドを定期的に送ってその返信
// (状態フレーム)を受け取る。モータは元々Disableなので動かない。
// これでROS側はEnable前に現在位置を知ることができ、Enable直後に目標位置へ
// 飛び出す事故を防げる。
// (公式STM32サンプルの帰還要求 0x7FF/0xCC は、このDM3520では返信が
//  無かったため使わない。2026-10-14実機確認)
void requestFeedbackWhileDisabled() {
    constexpr uint32_t PERIOD_MS = 20; // 50Hz
    static uint32_t last_ms = 0;
    const uint32_t now_ms = millis();
    if (now_ms - last_ms < PERIOD_MS) {
        return;
    }
    last_ms = now_ms;
    for (int m = 0; m < DAMIAO_MOTOR_COUNT; m++) {
        if (g_enabled[m]) {
            continue; // Enable中は制御フレームへの返信で帰還が来る
        }
        // 故障(通信喪失Dなど)はClearErrorを送るまでモータ側に残り続けるため、
        // Disable中に限りClearErrorで消す(ClearErrorも状態フレームを返し、
        // Enableはしない)。Enable中に起きた故障は、PCが0(DISABLE)を送って
        // ここに来るまで消さない(勝手に再始動させないため)。
        const bool fault = (g_last_state[m] != 0 && g_last_state[m] != 1);
        sendControlCmd(kMotorCanId[m], fault ? CMD_CLEAR_ERROR : CMD_DISABLE);
    }
}

// -------- CAN受信 (DM3520 -> 帰還) -------- //

// 帰還フレームのCAN IDはモータIDではなくMaster ID(実機で0x000と確認、
// 2026-10-14)。モータの判別はデータ1バイト目の下位4bit(=ESC_IDの下位4bit)
// で行う(公式STM32サンプルdm_motor_fbdata()と同じ)。
int motorIndexForFeedback(const uint8_t *d) {
    const uint8_t id_low4 = d[0] & 0x0F;
    for (int m = 0; m < DAMIAO_MOTOR_COUNT; m++) {
        if ((kMotorCanId[m] & 0x0F) == id_low4)
            return m;
    }
    return -1;
}

// レジスタ読み書きの返信 {id_lo, id_hi, 0x33/0x55, RID, value...} を、
// 位置などの状態帰還と取り違えないための判定。
bool isRegisterReply(const uint8_t *d) {
    if (d[2] != 0x33 && d[2] != 0x55) {
        return false;
    }
    for (int m = 0; m < DAMIAO_MOTOR_COUNT; m++) {
        if (d[0] == (kMotorCanId[m] & 0xFF) && d[1] == ((kMotorCanId[m] >> 8) & 0x07))
            return true;
    }
    return false;
}

// DM_CAN.py uint_to_float() と同じ変換式 (x_min~x_maxの範囲へ線形展開)。
float uintToFloat(uint16_t x, float x_min, float x_max, uint8_t bits) {
    const float span = x_max - x_min;
    const float norm = (float)x / (float)((1UL << bits) - 1UL);
    return norm * span + x_min;
}

// int16スロットへの変換。Arduino.hのmin/maxマクロがstd::clampと衝突する
// ため(cubemars.cppと同じ理由)、std::min/std::maxの組み合わせで手書きする。
int16_t physicalToSlot(double physical_value, double scale) {
    const long raw = lround(physical_value / scale);
    const long clamped = std::min<long>(std::max<long>(raw, INT16_MIN), INT16_MAX);
    return (int16_t)clamped;
}

void receiveFeedback() {
    twai_message_t rx_msg;
    while (twai_receive(&rx_msg, 0) == ESP_OK) {
        if (rx_msg.extd || rx_msg.data_length_code != 8) {
            continue; // Damiaoの標準フィードバックフレームではない
        }
        if (rx_msg.identifier != DAMIAO_MASTER_ID) {
            continue; // 帰還はMaster ID宛てに返ってくる
        }

        const uint8_t *d = rx_msg.data;
        if (isRegisterReply(d)) {
            continue;
        }
        const int m = motorIndexForFeedback(d);
        if (m < 0) {
            continue;
        }
        const uint16_t q_uint = (uint16_t)((d[1] << 8) | d[2]);
        const uint16_t dq_uint = (uint16_t)((d[3] << 4) | (d[4] >> 4));
        const uint16_t tau_uint = (uint16_t)(((d[4] & 0x0F) << 8) | d[5]);

        const float position = uintToFloat(q_uint, -DAMIAO_P_MAX, DAMIAO_P_MAX, 16);
        const float velocity = uintToFloat(dq_uint, -DAMIAO_V_MAX, DAMIAO_V_MAX, 12);
        const float torque = uintToFloat(tau_uint, -DAMIAO_T_MAX, DAMIAO_T_MAX, 12);

        const int off = m * SLOTS_PER_MOTOR;
        // Tx側の+0〜+2は指令(Rx側)用のスロットで帰還では未使用だったため流用する。
        g_last_state[m] = d[0] >> 4;
        Tx_16Data[off + 0] = (int16_t)g_last_state[m]; // 状態: 0=Disable 1=Enable それ以外=故障
        Tx_16Data[off + 1] = (int16_t)(Tx_16Data[off + 1] + 1); // 帰還の受信回数
        Tx_16Data[off + 3] = physicalToSlot(position, POS_SCALE);
        Tx_16Data[off + 4] = physicalToSlot(velocity, VEL_SCALE);
        Tx_16Data[off + 5] = physicalToSlot(torque, TORQUE_SCALE);
    }
}

} // namespace

void damiaoInit() {
    if (!installTwaiDriver()) {
        Serial.println("[ERR] damiao: TWAI install failed");
        while (1)
            ;
    }
}

void damiaoTask(void *pvParameters) {
    while (1) {
        recoverBusIfNeeded();
        receiveFeedback();
        sendCommands();
        requestFeedbackWhileDisabled();
        vTaskDelay(5); // 200Hz (ros2canのcubemars.cpp/robomas.cppと同じ周期)
    }
}
