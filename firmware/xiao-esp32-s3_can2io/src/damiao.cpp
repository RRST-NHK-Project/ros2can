/*====================================================================
<damiao.cpp>  Damiao DM3520(ESC) + DM2325(モータ) をCANで動かすドライバ(MODE_DAMIAO)

■ このファイルがやっていること(damiaoTask が5msごとに繰り返す)
  1. receiveFeedback()   モータからの返事(状態・位置・速度・トルク)を受け取り、
                         PCへ送る枠(Tx_16Data)に入れる
  2. sendCommands()      PCからの指令(Rx_16Data)を読み、各モータへ速度/角度の指令を送る。
                         有効化が必要なら「脱力→エラー解除→設定書き込み→有効化」を
                         1台ずつ・1段ずつ進める(stepInit)
  3. requestFeedbackWhileDisabled()  脱力中のモータにも返事を出させる(位置を知るため)。
                         内部の位置(レジスタ80)もここで読む
  4. recoverBusIfNeeded() CANの送信が止まったら復旧させる

■ よく変える場所(このファイルではなく config.hpp)
  ・モータの台数・ID       → DAMIAO_MOTORS(表の1行=1台)
  ・タイムアウトの時間     → DAMIAO_CMD_TIMEOUT_MS / DAMIAO_MOTOR_TIMEOUT_MS

■ PCとのデータ枠(1台6個。モータnは (n-1)*6 番から。PC側の
  damiao_ctrl/include/damiao_ctrl/slot_layout.hpp と必ず一致させること)
       PC → ESP32(指令)                    ESP32 → PC(返事)
  +0   モード 0=脱力 1=速度 2=角度          状態 0=脱力 1=有効 それ以外=故障(13=通信喪失)
  +1   目標 速度:0.01rad/s 角度:0.001rad   返事を受け取った回数(増えていれば通信OK)
  +2   角度制御の速さ上限 0.01rad/s         モータ内部の位置 0.1rad(不明なら-32768)
  +3   (未使用)                             位置 0.001rad(±12.5radで一周)
  +4   (未使用)                             速度 0.01rad/s
  +5   (未使用)                             トルク 0.001N·m
  ・全部0 = 脱力。PCが何も送っていない時も安全側になる。
  ・速度だけ0.01単位なのは、最高速度(58.6rad/s)が0.001単位の上限(32.767)に入らないため。

■ DamiaoのCAN通信(標準11bit ID、1Mbps)
  一次情報: 達妙公式 github.com/cmjang/DM_Control_Python の DM_CAN.py と、
  以前Arduinoで動いた実装。
  ・有効化/脱力/エラー解除/原点設定  ID=ESC_ID          data=FF×7 + cmd(FC/FD/FB/FE)
  ・設定(レジスタ)の書き込み          ID=0x7FF           data={ID下位,ID上位,0x55,RID,値(4byte)}
    読み出しは0x55の代わりに0x33。返事はMST_IDで届く。使うRID:
      10=制御モード(2=角度POS_VEL, 3=速度VEL)  ←これを書かないと制御の指令に反応しない
       9=通信タイムアウト(1=50us)   80=内部の位置(出力軸rad、何周分も数える)
  ・速度の指令  ID=0x200+ESC_ID  data=速度[rad/s](float32) + 0×4
  ・角度の指令  ID=0x100+ESC_ID  data=位置[rad](float32) + 速さ上限[rad/s](float32)
  ・返事        ID=MST_ID        byte0=状態(上4bit)とESC_IDの下4bit
                                 byte1-2=位置, byte3-4=速度, byte4-5=トルク
                                 (±P_MAX/±V_MAX/±T_MAXの範囲に割り付けた整数)
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
// モータ内部の位置(レジスタ80 = p_m。出力軸のrad、何周分も数え続ける)。
// 2026-11-02実機確認: 帰還の位置は p_m を±P_MAX(12.5rad)の範囲に折り返した値。
// 速度制御で何百rad回すと p_m が大きくなり(例: 1144rad)、モータ内部の小数の精度が
// 足りなくなって、ゆっくり回した時に位置(帰還も p_m も)が変わらなくなる(実際には
// 回っている)。この状態でPOS_VELを使うと、目標(折り返した値)と p_m の差を縮めようと
// して何百radも回ってしまう。そこで脱力中に p_m を読み、|p_m| が
// POS_VEL_MAX_INTERNAL_RAD を超える(または不明な)モータはPOS_VELを有効にしない。
// p_m はPCへTxの+2スロット(0.1rad単位、不明ならINT16_MIN)で送る。
// モータの電源を入れ直すと p_m は0に戻る。
constexpr uint8_t RID_P_M = 80;
constexpr float POS_VEL_MAX_INTERNAL_RAD = 12.0f; // POS_VELを有効にしてよい|p_m|の上限
constexpr double INTERNAL_POS_SCALE = 0.1;        // Tx+2スロットの単位[rad]
constexpr int32_t CMODE_POS_VEL = 2;
constexpr int32_t CMODE_VEL = 3; // 2026-10-14 実機確認済み(0x200+id, float32速度)

// 各モータのCAN ID。config.hppの DAMIAO_MOTORS(表の1行=1台)から作る。
struct MotorIds {
    uint16_t esc_id;    // 指令を受け取るCAN ID(ESC_ID)
    uint16_t master_id; // 返事を送ってくるCAN ID(MST_ID)
};
const MotorIds kMotors[] = DAMIAO_MOTORS;
constexpr int kMotorCount = sizeof(kMotors) / sizeof(kMotors[0]); // 表の行数=台数
static_assert(kMotorCount >= 1 && kMotorCount * SLOTS_PER_MOTOR <= Rx16NUM,
              "DAMIAO_MOTORSは1〜4行(PCとのデータ枠24個 ÷ 1台6個)");

// 有効化シーケンス(Disable -> ClearError -> TIMEOUT書込 -> CMODE書込 -> Enable)の
// 各ステップ。1ステップごとにINIT_STEP_MSだけ間を空ける。以前はdelay()で
// まとめて待っていたが、1台あたり約250ms処理が止まり、その間ほかのモータへの
// 指令が途切れて、そのモータが自身の通信タイムアウト(100ms)で止まってしまうため、
// ステップごとに進める方式にした(2026-11-02、2台対応)。
enum InitStep : int8_t {
    INIT_IDLE = -1, // 有効化シーケンス中ではない
    INIT_DISABLE = 0,
    INIT_CLEAR_ERROR,
    INIT_TIMEOUT,
    INIT_CMODE,
    INIT_ENABLE,
    INIT_WAIT_ENABLED, // Enableを送った後の待ち。終わったら制御フレームを送り始める
};
constexpr uint32_t INIT_STEP_MS = 50;

// Enable済み(制御フレームを送ってよい)か。
bool g_enabled[kMotorCount] = {false};
// 有効化シーケンス中/有効中のモード(MODE_VEL/MODE_POS_VEL)。モードが変わったら
// 一旦Disable+再初期化(モードレジスタ書き込み)してからEnableし直す。
int16_t g_enabled_mode[kMotorCount] = {MODE_DISABLED};
int8_t g_init_step[kMotorCount];
uint32_t g_init_next_ms[kMotorCount] = {0};
// 帰還で最後に受け取った状態コード(0=Disable, 1=Enable, それ以外=故障)。
uint8_t g_last_state[kMotorCount] = {0};
// モータ内部の位置 p_m [rad]。脱力中はレジスタ80の読み出しで更新し、有効中は
// 帰還の位置の変化(折り返しを補正)を足して追いかける。
float g_pm[kMotorCount] = {0};
bool g_pm_valid[kMotorCount] = {false};
float g_last_fb_pos[kMotorCount] = {0};
bool g_have_fb_pos[kMotorCount] = {false};

// POS_VELを有効にしてよいか(内部の位置が分かっていて、小さいこと)。
bool posVelAllowed(int m) {
    return g_pm_valid[m] && std::fabs(g_pm[m]) <= POS_VEL_MAX_INTERNAL_RAD;
}

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

// レジスタ読み出し要求(固定ID 0x7FF, {id_lo, id_hi, 0x33, RID})。返信はMST_IDで届く。
void readRegister(uint16_t motor_can_id, uint8_t rid) {
    uint8_t data[8] = {(uint8_t)(motor_can_id & 0xFF), (uint8_t)((motor_can_id >> 8) & 0x07), 0x33, rid,
                       0, 0, 0, 0};
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

// モード切替+Enableの初期化シーケンスを1ステップ進める。実機で動作確認済みの
// Arduinoリファレンス実装(initMotor())と同じ順序: Disable -> ClearError ->
// モードレジスタ書き込み -> Enable(各ステップの間はINIT_STEP_MS)。
// 通信タイムアウトもここで毎回書き込む(RAMのみ。電源を切ると0に戻るため)。
// Enableを送ってから最初の制御フレームまではINIT_STEP_MS(50ms)で、
// モータの通信タイムアウト(100ms)より短い。
void stepInit(int m, uint32_t now_ms) {
    if ((int32_t)(now_ms - g_init_next_ms[m]) < 0) {
        return;
    }
    const uint16_t id = kMotors[m].esc_id;
    switch (g_init_step[m]) {
    case INIT_DISABLE:
        sendControlCmd(id, CMD_DISABLE);
        break;
    case INIT_CLEAR_ERROR:
        sendControlCmd(id, CMD_CLEAR_ERROR);
        break;
    case INIT_TIMEOUT:
        writeRegister(id, RID_TIMEOUT, (int32_t)(DAMIAO_MOTOR_TIMEOUT_MS * 1000UL / TIMEOUT_UNIT_US));
        break;
    case INIT_CMODE:
        writeRegister(id, RID_CMODE,
                      (g_enabled_mode[m] == MODE_POS_VEL) ? CMODE_POS_VEL : CMODE_VEL);
        break;
    case INIT_ENABLE:
        sendControlCmd(id, CMD_ENABLE);
        break;
    case INIT_WAIT_ENABLED:
        g_enabled[m] = true;
        g_init_step[m] = INIT_IDLE;
        return;
    default:
        g_init_step[m] = INIT_IDLE;
        return;
    }
    g_init_step[m]++;
    g_init_next_ms[m] = now_ms + INIT_STEP_MS;
}

// -------- CAN送信 (指令 -> DM3520) -------- //

// PCからの有効フレームがDAMIAO_CMD_TIMEOUT_MS以上途絶えていればtrue。
// DM3520自身の通信タイムアウト(RID9, 有効化シーケンスで設定)はESP32<->モータ間の
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
    // 1フレーム分の指令をまとめて取り出す(mode・target・vel_limitが別フレームの値と
    // 混ざらないように、serialTaskの書き込みとロックで排他した写しを使う)。
    int16_t rx[Rx16NUM];
    serialSnapshotRx(rx);
    const uint32_t now_ms = millis();
    for (int m = 0; m < kMotorCount; m++) {
        const int off = m * SLOTS_PER_MOTOR;
        // 指令途絶中はPCの最後の指令に関わらずDisable扱いにする。
        int16_t mode = link_lost ? MODE_DISABLED : rx[off + 0];
        // 内部の位置が大きい/不明なモータはPOS_VELにしない(上の RID_P_M の説明参照)。
        // 動作中に±P_MAXを超えた場合も止める。
        if (mode == MODE_POS_VEL) {
            const bool running = g_enabled[m] && g_enabled_mode[m] == MODE_POS_VEL;
            const bool ok = running ? (g_pm_valid[m] && std::fabs(g_pm[m]) <= DAMIAO_P_MAX)
                                    : posVelAllowed(m);
            if (!ok) {
                mode = MODE_DISABLED;
            }
        }
        const float target = (float)(rx[off + 1] * (mode == MODE_VEL ? VEL_SCALE : POS_SCALE));
        const float vel_limit = (float)(rx[off + 2] * VEL_SCALE);
        const uint16_t motor_can_id = kMotors[m].esc_id;

        if (mode != MODE_VEL && mode != MODE_POS_VEL) {
            // 未知の値も含め、DISABLE以外の全てを安全側(無効化)として扱う。
            if (g_enabled[m] || g_init_step[m] != INIT_IDLE) {
                sendControlCmd(motor_can_id, CMD_DISABLE);
                g_enabled[m] = false;
                g_init_step[m] = INIT_IDLE;
                g_enabled_mode[m] = MODE_DISABLED;
            }
            continue;
        }

        if (g_enabled_mode[m] != mode || (!g_enabled[m] && g_init_step[m] == INIT_IDLE)) {
            // 初回Enable、またはモードが切り替わった場合は、
            // Disable -> ClearError -> モードレジスタ書き込み -> Enable の
            // 初期化シーケンスをやり直す(実機確認済みのArduinoリファレンス
            // 実装と同じ手順)。これを行わないとモータが制御フレームに
            // 一切反応しない。
            g_enabled[m] = false;
            g_enabled_mode[m] = mode;
            g_init_step[m] = INIT_DISABLE;
            g_init_next_ms[m] = now_ms;
        }
        if (!g_enabled[m]) {
            stepInit(m, now_ms); // シーケンス中は制御フレームを送らない
            continue;
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
    static uint8_t tick = 0;
    tick++;
    for (int m = 0; m < kMotorCount; m++) {
        if (g_enabled[m] || g_init_step[m] != INIT_IDLE) {
            continue; // Enable中は制御フレームへの返信で帰還が来る。シーケンス中も送らない
        }
        if (tick % 10 == 0) { // 0.2秒ごとに内部の位置(レジスタ80)を読む
            readRegister(kMotors[m].esc_id, RID_P_M);
        }
        // 故障(通信喪失Dなど)はClearErrorを送るまでモータ側に残り続けるため、
        // Disable中に限りClearErrorで消す(ClearErrorも状態フレームを返し、
        // Enableはしない)。Enable中に起きた故障は、PCが0(DISABLE)を送って
        // ここに来るまで消さない(勝手に再始動させないため)。
        const bool fault = (g_last_state[m] != 0 && g_last_state[m] != 1);
        sendControlCmd(kMotors[m].esc_id, fault ? CMD_CLEAR_ERROR : CMD_DISABLE);
    }
}

// -------- CAN受信 (DM3520 -> 帰還) -------- //

// 帰還フレームのCAN IDはモータIDではなくMaster ID(MST_ID。1台目は0x000、
// 2026-10-14確認)。CAN IDが各モータのMST_IDと一致し、かつデータ1バイト目の
// 下位4bit(=ESC_IDの下位4bit、公式STM32サンプルdm_motor_fbdata()と同じ)も
// 一致するモータを返す(MST_IDを共有している構成でも下位4bitで判別できる)。
int motorIndexForFeedback(uint32_t can_id, const uint8_t *d) {
    const uint8_t id_low4 = d[0] & 0x0F;
    for (int m = 0; m < kMotorCount; m++) {
        if (kMotors[m].master_id == can_id && (kMotors[m].esc_id & 0x0F) == id_low4)
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
    for (int m = 0; m < kMotorCount; m++) {
        if (d[0] == (kMotors[m].esc_id & 0xFF) && d[1] == ((kMotors[m].esc_id >> 8) & 0x07))
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
        const uint8_t *d = rx_msg.data;
        if (isRegisterReply(d)) {
            // 内部の位置(レジスタ80)の返信。脱力中に読んだ値で上書きする。
            if (d[2] == 0x33 && d[3] == RID_P_M) {
                for (int m = 0; m < kMotorCount; m++) {
                    if (d[0] == (kMotors[m].esc_id & 0xFF) && !g_enabled[m]) {
                        float v;
                        std::memcpy(&v, &d[4], 4);
                        if (std::isfinite(v)) {
                            g_pm[m] = v;
                            g_pm_valid[m] = true;
                        }
                    }
                }
            }
            continue;
        }
        // 帰還は各モータのMaster ID(MST_ID)で返ってくる。それ以外は無視。
        const int m = motorIndexForFeedback(rx_msg.identifier, d);
        if (m < 0) {
            continue;
        }
        const uint16_t q_uint = (uint16_t)((d[1] << 8) | d[2]);
        const uint16_t dq_uint = (uint16_t)((d[3] << 4) | (d[4] >> 4));
        const uint16_t tau_uint = (uint16_t)(((d[4] & 0x0F) << 8) | d[5]);

        const float position = uintToFloat(q_uint, -DAMIAO_P_MAX, DAMIAO_P_MAX, 16);
        const float velocity = uintToFloat(dq_uint, -DAMIAO_V_MAX, DAMIAO_V_MAX, 12);
        const float torque = uintToFloat(tau_uint, -DAMIAO_T_MAX, DAMIAO_T_MAX, 12);

        // 有効中は、内部の位置を帰還の位置の変化で追いかける(折り返しを補正)
        if (g_have_fb_pos[m] && g_pm_valid[m] && g_enabled[m]) {
            float dp = position - g_last_fb_pos[m];
            if (dp > DAMIAO_P_MAX) {
                dp -= 2.0f * DAMIAO_P_MAX;
            } else if (dp < -DAMIAO_P_MAX) {
                dp += 2.0f * DAMIAO_P_MAX;
            }
            g_pm[m] += dp;
        }
        g_last_fb_pos[m] = position;
        g_have_fb_pos[m] = true;

        const int off = m * SLOTS_PER_MOTOR;
        // Tx側の+0〜+2は指令(Rx側)用のスロットで帰還では未使用だったため流用する。
        // 1台分の6スロットはロックしてまとめて書く(PCへ途中の値が混ざって出ないように)。
        g_last_state[m] = d[0] >> 4;
        static int16_t fb_counter[kMotorCount] = {0};
        fb_counter[m]++;
        const int16_t slots[SLOTS_PER_MOTOR] = {
            (int16_t)g_last_state[m], // 状態: 0=Disable 1=Enable それ以外=故障
            fb_counter[m],            // 帰還の受信回数
            g_pm_valid[m] ? physicalToSlot(g_pm[m], INTERNAL_POS_SCALE) : (int16_t)INT16_MIN, // 内部の位置
            physicalToSlot(position, POS_SCALE),
            physicalToSlot(velocity, VEL_SCALE),
            physicalToSlot(torque, TORQUE_SCALE),
        };
        serialWriteTx(off, slots, SLOTS_PER_MOTOR);
    }
}

} // namespace

void damiaoInit() {
    for (int m = 0; m < kMotorCount; m++) {
        g_init_step[m] = INIT_IDLE;
    }
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
