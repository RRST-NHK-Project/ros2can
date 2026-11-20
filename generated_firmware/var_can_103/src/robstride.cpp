/*====================================================================
<robstride.cpp>
・RobStride EduLite 05 (EL05) 用CANドライバの実装ファイル

RobStride独自(私有)プロトコルの速度モード(run_mode=2)と位置モード
CSP(run_mode=5)を使う。MIT相当の運控モード(run_mode=0)はKp/Kdの調整が必要で
初回動作確認に不向きなため未実装。

一次情報: RobStride EduLite05 説明書(CN v1.1 / EN v1.0)を、実機(EL05)で
確かめながら整理した資料 github.com/mocchan0221/EDULITE05_Controller の
docs/research/el05_protocol.md と hw_bench.md (2026-10-02〜03 実機確認)。

CANプロトコル (29bit拡張ID, 1Mbps, DLC=8):
  ID = 通信タイプ(bit28-24) | データ領域2(bit23-8) | 宛先(bit7-0)
  本機 -> モータ: データ領域2の下位8bit(ID bit15-8) = ホストID(ROBSTRIDE_HOST_ID)、
                  宛先 = モータのCAN ID
  モータ -> 本機: ID bit15-8 = モータのCAN ID、宛先 = ホストID
  使う通信タイプ:
    0  ID確認       応答は宛先0xFE固定の通信タイプ0(データ=UID)。起動時のIDスキャンに使う
    2  帰還(モータ->本機) ID bit23-22 = モード(0=Reset/停止, 1=校正, 2=運転)、
                          ID bit21-16 = 故障(bit16 低電圧, 17 過電流, 18 過温,
                          19 エンコーダ, 20 ストール過負荷, 21 未校正)
                          データ(ビッグエンディアン): 角度u16(±12.57rad), 速度u16(±50rad/s),
                          トルクu16(±6N・m), 温度int16(x10 degC)
                          通信タイプ3/4/18への応答として返ってくる
    3  有効化(運転開始)
    4  停止。data[0]=1で故障クリアも行う
    17 パラメータ読み出し data[0-1]=index(LE)。応答も通信タイプ17で、data[4-7]=値(LE)
    18 パラメータ書き込み data[0-1]=index(LE), data[4-7]=値(LE。uint8はdata[4]のみ)
    24 主動上報(このファームでは使わない。上報と応答が重なると応答が欠けるため)
  使うパラメータ(通信タイプ17/18。どれもRAMのみで、電源を切ると工場値に戻る):
    0x7005 run_mode (uint8) 2=速度, 5=位置CSP。停止中にしか変えられない
    0x700A spd_ref  (float rad/s) 速度モードの目標
    0x7016 loc_ref  (float rad)   位置モードの目標(多回転の位置mechPosと同じ座標)
    0x7017 limit_spd(float rad/s) 位置モードの速度上限(実機は+22%程度まで行き過ぎる)
    0x7018 limit_cur(float A)     速度・位置モードの電流上限
    0x7019 mechPos  (float rad)   出力軸の多回転位置(読み出し専用)
    0x7022 acc_rad  (float rad/s^2) 速度モードの加速度
    0x7028 canTimeout(uint32, 1あたり50us) CANが途絶えたらモータが自分で停止する時間

位置は帰還(通信タイプ2)の角度ではなくmechPosを使う。帰還の角度は±4π(約±2回転)で
折り返すが(実機確認)、位置モードの目標loc_refはmechPos(折り返さない)と同じ座標
なので、帰還の角度を元に目標を作ると8πずれた位置へ向かって走る恐れがあるため。

スロット割り当て (robstride_ctrl/include/robstride_ctrl/slot_layout.hppと一致
させること。8スロット/モータ、モータnのオフセット = (n-1)*8、最大2台):
  Rx_16Data (PC -> 本機, 指令):
    +0: mode_word (0=DISABLE, 1=VEL(速度), 2=POS(位置CSP))。全ゼロ=DISABLEが
        安全な既定値(起動直後・PC未接続時はモータを有効化しない)。
    +1,+2: target int32 (+1が上位16bit, +2が下位16bit)。0.001単位
           (VELならrad/s、POSならrad)。
    +3: vel_limit (0.01 rad/s/LSB。POSモードの速度上限。0以下は有効化しない)
  Tx_16Data (本機 -> PC, 帰還):
    +0: モータのモード (0=Reset(停止), 1=校正, 2=運転)
    +1: 故障ビット。下位6bit=モータの故障(上記)、
        0x100=本機が止めた(起動直後・PCからの指令が途絶えた・有効化中にモータが
              運転モードを外れた・帰還が途絶えた。PCが一度0(DISABLE)を送るまで
              再始動しない。通信が戻っただけでは外れない)、
        0x200=位置(mechPos)をまだ読めていない
    +2: 帰還フレームの受信回数(増え続けていればモータが応答している)
    +3,+4: position int32 (+3が上位16bit) [0.001 rad/LSB] 出力軸の多回転位置
    +5: velocity [0.01 rad/s/LSB]
    +6: torque   [0.001 N・m/LSB]
    +7: 温度     [0.1 degC/LSB]
  Tx_16Data[20-23] (診断): IDスキャンの結果。[20]=応答したモータの数、
                           [21-23]=応答したCAN ID(最大3個、無ければ-1)

CAN送信の補助(ドライバの導入・Bus-Off復帰・送信停止対策)は twai_util.cpp
(MODE_DAMIAOと共通)。送信失敗のログはシリアルに出さない(PCとのバイナリ
フレームに文字が混ざるとPC側が読めなくなるため。2026-11-06 PRレビュー指摘)。
====================================================================*/

#include "robstride.hpp"
#include "config.hpp"
#include "defs.hpp"
#include "frame_data.hpp"
#include "serial_task.hpp"
#include "twai_util.hpp"
#include <Arduino.h>
#include <algorithm>
#include <cmath>
#include <cstring>

namespace {

constexpr int SLOTS_PER_MOTOR = 8;
constexpr int MAX_MOTORS = 2;
static_assert(ROBSTRIDE_MOTOR_COUNT >= 1 && ROBSTRIDE_MOTOR_COUNT <= MAX_MOTORS,
              "ROBSTRIDE_MOTOR_COUNT must be 1 or 2 (8 slots/motor + diagnostic slots 20-23)");
constexpr int DIAG_SLOT = 20; // 20: 応答数, 21-23: 応答したID
constexpr int DIAG_ID_SLOTS = 3;

// スロット1LSBあたりの物理量。PC側(robstride_ctrl/slot_layout.hpp)と一致させること。
constexpr double TARGET_SCALE = 0.001; // rad または rad/s
constexpr double POS_SCALE = 0.001;    // rad
constexpr double VEL_SCALE = 0.01;     // rad/s
constexpr double TORQUE_SCALE = 0.001; // N・m

// PC側(Rx_16Data)のmode_word。robstride_ctrl/slot_layout.hppと一致させること。
constexpr int16_t MODE_DISABLED = 0;
constexpr int16_t MODE_VEL = 1;
constexpr int16_t MODE_POS = 2;

// Tx +1 の故障スロットのうち本機が付けるビット
constexpr uint16_t FLAG_LATCHED = 0x100;
constexpr uint16_t FLAG_POS_UNKNOWN = 0x200;

// 通信タイプ
constexpr uint8_t COMM_GET_ID = 0;
constexpr uint8_t COMM_FEEDBACK = 2;
constexpr uint8_t COMM_ENABLE = 3;
constexpr uint8_t COMM_STOP = 4;
constexpr uint8_t COMM_PARAM_READ = 17;
constexpr uint8_t COMM_PARAM_WRITE = 18;
constexpr uint8_t COMM_ACTIVE_REPORT = 24;
constexpr uint8_t ID_REPLY_DEST = 0xFE; // 通信タイプ0の応答の宛先(固定)

// パラメータindex
constexpr uint16_t PARAM_RUN_MODE = 0x7005;
constexpr uint16_t PARAM_SPD_REF = 0x700A;
constexpr uint16_t PARAM_LOC_REF = 0x7016;
constexpr uint16_t PARAM_LIMIT_SPD = 0x7017;
constexpr uint16_t PARAM_LIMIT_CUR = 0x7018;
constexpr uint16_t PARAM_MECH_POS = 0x7019;
constexpr uint16_t PARAM_ACC_RAD = 0x7022;
constexpr uint16_t PARAM_CAN_TIMEOUT = 0x7028;
constexpr uint32_t CAN_TIMEOUT_UNIT_US = 50;

constexpr uint8_t RUN_MODE_VELOCITY = 2;
constexpr uint8_t RUN_MODE_CSP = 5;

// モータのモード(帰還 ID bit23-22)
constexpr uint8_t MOTOR_MODE_RUN = 2;

// 帰還(通信タイプ2)のデコード範囲。EL05説明書CN版の値(EN版のトルク±5.5はRS05からの誤記)。
constexpr float P_MAX = 12.57f;
constexpr float V_MAX = 50.0f;
constexpr float T_MAX = 6.0f;

// フレームを続けて送るときの間隔[ms]。0.5ms未満で送るとモータの応答が欠けた
// (公式USB-CAN経由の実測。モータ側かアダプタ側かは未切り分けなので安全側で空ける)。
constexpr uint32_t FRAME_GAP_MS = 1;
// 有効化の手順の各フレームの間隔[ms]
constexpr uint32_t INIT_GAP_MS = 2;
// 有効化してから、帰還のモードが「運転」になるのを待つ猶予[ms]
constexpr uint32_t ENABLE_GRACE_MS = 100;
// 有効化中に帰還がこの時間届かなければ、モータが応答していないとみなして止める[ms]
constexpr uint32_t FEEDBACK_LOST_MS = 200;

const uint8_t kMotorCanId[MAX_MOTORS] = {ROBSTRIDE_MOTOR_ID_1, ROBSTRIDE_MOTOR_ID_2};

struct MotorRuntime {
    bool enabled = false;
    int16_t enabled_mode = MODE_DISABLED;
    uint32_t enabled_at_ms = 0;
    // 有効化中にモータが運転モードを外れた(CANタイムアウト・故障・電源断など)。
    // PCが0(DISABLE)を送るまで再始動しない。
    bool latched = false;
    float last_limit_spd = -1.0f; // 位置モードで最後に書いたlimit_spd(負=未送信)

    // 帰還
    uint8_t mode_bits = 0;
    uint8_t fault_bits = 0;
    uint32_t last_fb_ms = 0; // 0=まだ受信していない
    int16_t fb_counter = 0;
    float velocity = 0.0f;
    float torque = 0.0f;
    int16_t temp_x10 = 0;
    bool have_mech_pos = false;
    float mech_pos = 0.0f;
};

MotorRuntime g_motor[ROBSTRIDE_MOTOR_COUNT];

// IDスキャン
int16_t g_found_ids[DIAG_ID_SLOTS] = {-1, -1, -1};
int16_t g_found_count = 0;

// 1フレーム送る。送信キューが空かなければ最大 timeout_ms 待ち、入らなければ捨てる。
// 失敗しても文字は出さない(PCとのシリアルにログが混ざるとフレームが壊れるため。
// 2026-11-06 PRレビュー指摘)。失敗回数は twai_util が数えている。
bool sendExtended(uint8_t comm_type, uint16_t data2, uint8_t dest, const uint8_t *data,
                  uint32_t timeout_ms) {
    twai_message_t tx{};
    tx.identifier = ((uint32_t)(comm_type & 0x1F) << 24) | ((uint32_t)data2 << 8) | dest;
    tx.extd = 1; // RobStride独自プロトコルは29bit拡張ID
    tx.rtr = 0;
    tx.data_length_code = 8;
    std::memcpy(tx.data, data, 8);
    return twaiSend(tx, timeout_ms);
}

void sendToMotor(uint8_t comm_type, uint8_t motor_id, const uint8_t *data) {
    // 以前は20ms待っていたが、モータの電源が切れてACKが返らない時に制御ループが
    // 長く止まるため短くした(1フレームの送信は約0.13ms)
    sendExtended(comm_type, ROBSTRIDE_HOST_ID, motor_id, data, 2);
}

void sendEnable(uint8_t motor_id) {
    const uint8_t data[8] = {0};
    sendToMotor(COMM_ENABLE, motor_id, data);
}

// 停止(Reset)。clear_fault=trueで故障クリアも行う。応答は帰還(通信タイプ2)。
void sendStop(uint8_t motor_id, bool clear_fault) {
    const uint8_t data[8] = {(uint8_t)(clear_fault ? 1 : 0), 0, 0, 0, 0, 0, 0, 0};
    sendToMotor(COMM_STOP, motor_id, data);
}

void writeParamFloat(uint8_t motor_id, uint16_t index, float value) {
    uint8_t data[8] = {(uint8_t)(index & 0xFF), (uint8_t)(index >> 8), 0, 0, 0, 0, 0, 0};
    std::memcpy(&data[4], &value, 4); // ESP32はリトルエンディアン = プロトコルと同じ
    sendToMotor(COMM_PARAM_WRITE, motor_id, data);
}

void writeParamU8(uint8_t motor_id, uint16_t index, uint8_t value) {
    // uint8はdata[4]に整数で入れる(floatで書いてはいけない)
    const uint8_t data[8] = {(uint8_t)(index & 0xFF), (uint8_t)(index >> 8), 0, 0, value, 0, 0, 0};
    sendToMotor(COMM_PARAM_WRITE, motor_id, data);
}

void writeParamU32(uint8_t motor_id, uint16_t index, uint32_t value) {
    uint8_t data[8] = {(uint8_t)(index & 0xFF), (uint8_t)(index >> 8), 0, 0, 0, 0, 0, 0};
    std::memcpy(&data[4], &value, 4);
    sendToMotor(COMM_PARAM_WRITE, motor_id, data);
}

void readParam(uint8_t motor_id, uint16_t index) {
    const uint8_t data[8] = {(uint8_t)(index & 0xFF), (uint8_t)(index >> 8), 0, 0, 0, 0, 0, 0};
    sendToMotor(COMM_PARAM_READ, motor_id, data);
}

int16_t physicalToSlot(double physical_value, double scale) {
    const long raw = lround(physical_value / scale);
    const long clamped = std::min<long>(std::max<long>(raw, INT16_MIN), INT16_MAX);
    return (int16_t)clamped;
}

int32_t physicalToInt32(double physical_value, double scale) {
    const double raw = std::round(physical_value / scale);
    const double clamped = std::min<double>(std::max<double>(raw, (double)INT32_MIN), (double)INT32_MAX);
    return (int32_t)clamped;
}

int32_t slotsToInt32(int16_t hi, int16_t lo) {
    return (int32_t)(((uint32_t)(uint16_t)hi << 16) | (uint32_t)(uint16_t)lo);
}

float clampFloat(float value, float lo, float hi) {
    return std::min(std::max(value, lo), hi);
}

float uintToFloat(uint16_t x, float x_min, float x_max) {
    return (float)x * (x_max - x_min) / 65535.0f + x_min;
}

int motorIndexForId(uint8_t motor_id) {
    for (int m = 0; m < ROBSTRIDE_MOTOR_COUNT; m++) {
        if (kMotorCanId[m] == motor_id) {
            return m;
        }
    }
    return -1;
}

bool feedbackFresh(const MotorRuntime &st, uint32_t now_ms, uint32_t max_age_ms) {
    return st.last_fb_ms != 0 && (now_ms - st.last_fb_ms) <= max_age_ms;
}

// -------- 有効化の手順 -------- //

// 停止(故障クリア) -> CANタイムアウト -> run_mode -> 目標の初期値 -> 上限 -> 有効化。
// 目標の初期値は、有効化した瞬間に前回の目標へ向かって飛び出さないようにするため
// (速度モードは0、位置モードは現在の位置)。上限・目標を有効化より先に書くのは、
// 実機で動作確認済みの手順(EDULITE05_Controllerのenable)と同じ。
// 所要時間は約15ms(INIT_GAP_MS x 7)。
void initAndEnable(int m, int16_t pc_mode, float vel_limit) {
    MotorRuntime &st = g_motor[m];
    const uint8_t id = kMotorCanId[m];

    sendStop(id, true);
    delay(INIT_GAP_MS);
    writeParamU32(id, PARAM_CAN_TIMEOUT,
                  (uint32_t)ROBSTRIDE_MOTOR_TIMEOUT_MS * 1000UL / CAN_TIMEOUT_UNIT_US);
    delay(INIT_GAP_MS);
    if (pc_mode == MODE_VEL) {
        writeParamU8(id, PARAM_RUN_MODE, RUN_MODE_VELOCITY);
        delay(INIT_GAP_MS);
        writeParamFloat(id, PARAM_SPD_REF, 0.0f);
        delay(INIT_GAP_MS);
        writeParamFloat(id, PARAM_LIMIT_CUR, ROBSTRIDE_LIMIT_CUR_A);
        delay(INIT_GAP_MS);
        writeParamFloat(id, PARAM_ACC_RAD, ROBSTRIDE_VEL_ACC_RADPS2);
        delay(INIT_GAP_MS);
    } else { // MODE_POS
        writeParamU8(id, PARAM_RUN_MODE, RUN_MODE_CSP);
        delay(INIT_GAP_MS);
        writeParamFloat(id, PARAM_LOC_REF, st.mech_pos);
        delay(INIT_GAP_MS);
        writeParamFloat(id, PARAM_LIMIT_CUR, ROBSTRIDE_LIMIT_CUR_A);
        delay(INIT_GAP_MS);
        writeParamFloat(id, PARAM_LIMIT_SPD, vel_limit);
        st.last_limit_spd = vel_limit;
        delay(INIT_GAP_MS);
    }
    sendEnable(id);
    delay(INIT_GAP_MS);

    st.enabled = true;
    st.enabled_mode = pc_mode;
    st.enabled_at_ms = millis();
}

void disableMotor(int m) {
    sendStop(kMotorCanId[m], false);
    g_motor[m].enabled = false;
    g_motor[m].enabled_mode = MODE_DISABLED;
}

// -------- CAN送信 (指令 -> EL05) -------- //

// PCからの有効フレームがROBSTRIDE_CMD_TIMEOUT_MS以上途絶えていればtrue。
bool commandLinkLost() {
    const uint32_t last_ms = serialLastValidRxMs();
    if (last_ms == 0) {
        return true; // 起動後まだ一度も指令を受け取っていない
    }
    return (millis() - last_ms) > ROBSTRIDE_CMD_TIMEOUT_MS;
}

void sendCommands() {
    int16_t rx[Rx16NUM];
    serialSnapshotRx(rx);
    const bool link_lost = commandLinkLost();
    const uint32_t now_ms = millis();

    for (int m = 0; m < ROBSTRIDE_MOTOR_COUNT; m++) {
        MotorRuntime &st = g_motor[m];
        const int off = m * SLOTS_PER_MOTOR;
        const int16_t pc_mode = rx[off + 0]; // PCが送ってきたモード(そのままの値)
        const int16_t mode = link_lost ? MODE_DISABLED : pc_mode;
        const float target =
            (float)(slotsToInt32(rx[off + 1], rx[off + 2]) * TARGET_SCALE);
        const float vel_limit = (float)(rx[off + 3] * VEL_SCALE);
        const uint8_t id = kMotorCanId[m];

        // 有効化中の異常: モータが運転モードを外れた(CANタイムアウト・故障・電源の入れ直し)
        // または帰還が途絶えた -> 止めてラッチ。PCが0を送るまで再始動しない
        // (電源が戻った瞬間に勝手に動き出さないようにするため)。
        if (st.enabled && (now_ms - st.enabled_at_ms) > ENABLE_GRACE_MS) {
            const bool fb_ok = feedbackFresh(st, now_ms, FEEDBACK_LOST_MS);
            const bool left_run = fb_ok && (st.mode_bits != MOTOR_MODE_RUN || st.fault_bits != 0);
            if (!fb_ok || left_run) {
                disableMotor(m);
                st.latched = true;
            }
        }

        // PCからの指令が途絶えたらラッチする。以前は途絶を「PCが0を送った」と同じ扱いに
        // していたため、ラッチが勝手に外れ、通信が戻ると最後の指令で動き出していた
        // (2026-11-06 PRレビュー指摘)。
        if (link_lost) {
            st.latched = true;
        }

        const bool want_run = (mode == MODE_VEL) || (mode == MODE_POS && st.have_mech_pos &&
                                                     vel_limit > 0.0f);
        if (st.latched) {
            if (st.enabled) {
                disableMotor(m); // 途絶でラッチした時は、まだ有効のことがある
            }
            if (!link_lost && pc_mode == MODE_DISABLED) {
                st.latched = false; // PCが脱力を送った = 人が止まったことを確認した
            }
            continue;
        }
        if (!want_run) {
            // DISABLE・未知の値・位置未取得のPOS・速度上限0以下のPOSは全て停止扱い
            if (st.enabled) {
                disableMotor(m);
            }
            continue;
        }

        if (!st.enabled || st.enabled_mode != mode) {
            // 初回、またはモード切替。run_modeは停止中にしか変えられないため、
            // 毎回 停止 -> run_mode書き込み -> 有効化 をやり直す。
            initAndEnable(m, mode, std::min(vel_limit, ROBSTRIDE_V_MAX_RADPS));
        }

        if (mode == MODE_VEL) {
            writeParamFloat(id, PARAM_SPD_REF,
                            clampFloat(target, -ROBSTRIDE_V_MAX_RADPS, ROBSTRIDE_V_MAX_RADPS));
        } else { // MODE_POS
            const float limit = std::min(vel_limit, ROBSTRIDE_V_MAX_RADPS);
            if (std::fabs(limit - st.last_limit_spd) > 1e-4f) {
                writeParamFloat(id, PARAM_LIMIT_SPD, limit);
                st.last_limit_spd = limit;
                delay(FRAME_GAP_MS);
            }
            writeParamFloat(id, PARAM_LOC_REF, target);
        }
        delay(FRAME_GAP_MS);
    }
}

// 停止中も状態を読むため、停止コマンド(故障クリアなし)を50Hzで送って応答の帰還を得る。
// モータは元々停止しているので動かない。故障ビットはクリアしないのでPCから見える。
void pollWhileDisabled() {
    constexpr uint32_t PERIOD_MS = 20;
    static uint32_t last_ms = 0;
    const uint32_t now_ms = millis();
    if (now_ms - last_ms < PERIOD_MS) {
        return;
    }
    last_ms = now_ms;
    for (int m = 0; m < ROBSTRIDE_MOTOR_COUNT; m++) {
        if (!g_motor[m].enabled) {
            sendStop(kMotorCanId[m], false);
            delay(FRAME_GAP_MS);
        }
    }
}

// 多回転の位置mechPosを100Hzで読む(有効・停止どちらでも)。
void pollMechPos() {
    constexpr uint32_t PERIOD_MS = 10;
    static uint32_t last_ms = 0;
    const uint32_t now_ms = millis();
    if (now_ms - last_ms < PERIOD_MS) {
        return;
    }
    last_ms = now_ms;
    for (int m = 0; m < ROBSTRIDE_MOTOR_COUNT; m++) {
        readParam(kMotorCanId[m], PARAM_MECH_POS);
        delay(FRAME_GAP_MS);
    }
}

// -------- IDスキャン -------- //
// config.hppのIDのモータから帰還が無い間、全ID(0-127)へ通信タイプ0を送り、
// 応答したIDを診断スロットに出す(工場出荷のIDは説明書では1だが、実機は127だった
// 例があるため)。1サイクルに16個ずつ送り、一巡したら3秒あける。
void scanIfNoFeedback() {
    constexpr uint8_t MAX_ID = 127;
    constexpr int IDS_PER_CYCLE = 16;
    constexpr uint32_t ROUND_INTERVAL_MS = 3000;
    constexpr uint32_t FEEDBACK_OK_MS = 1000;
    static int next_id = -1; // -1=待機中
    static uint32_t round_done_ms = 0;
    static bool first_round = true;

    const uint32_t now_ms = millis();
    if (next_id < 0) {
        bool all_ok = true;
        for (int m = 0; m < ROBSTRIDE_MOTOR_COUNT; m++) {
            all_ok = all_ok && feedbackFresh(g_motor[m], now_ms, FEEDBACK_OK_MS);
        }
        // 起動直後に1回は必ず行う(IDが合っていても、他のモータが居ないか確認できる)
        if ((all_ok && !first_round) || (!first_round && now_ms - round_done_ms < ROUND_INTERVAL_MS)) {
            return;
        }
        if (first_round && now_ms < 1000) {
            return; // 起動直後はモータ側の起動を少し待つ
        }
        next_id = 0;
        g_found_count = 0;
        for (int i = 0; i < DIAG_ID_SLOTS; i++) {
            g_found_ids[i] = -1;
        }
    }
    const uint8_t data[8] = {0};
    for (int i = 0; i < IDS_PER_CYCLE && next_id <= MAX_ID; i++, next_id++) {
        // 相手が居ないとACKが返らず送信が詰まるので、待たずに捨てる
        sendExtended(COMM_GET_ID, ROBSTRIDE_HOST_ID, (uint8_t)next_id, data, 0);
        delay(FRAME_GAP_MS);
    }
    if (next_id > MAX_ID) {
        next_id = -1;
        first_round = false;
        round_done_ms = now_ms;
    }
}

// -------- CAN受信 (EL05 -> 帰還) -------- //

void handleFeedback(const twai_message_t &rx) {
    const uint16_t data2 = (rx.identifier >> 8) & 0xFFFF;
    const int m = motorIndexForId(data2 & 0xFF);
    if (m < 0) {
        return;
    }
    const uint8_t *d = rx.data;
    // バージョン応答(00 C4 56 ..)も通信タイプ2で返るので状態として使わない
    if (d[0] == 0x00 && d[1] == 0xC4 && d[2] == 0x56) {
        return;
    }
    MotorRuntime &st = g_motor[m];
    st.mode_bits = (data2 >> 14) & 0x3;
    st.fault_bits = (data2 >> 8) & 0x3F;
    const uint16_t v_u = (uint16_t)((d[2] << 8) | d[3]);
    const uint16_t t_u = (uint16_t)((d[4] << 8) | d[5]);
    st.velocity = uintToFloat(v_u, -V_MAX, V_MAX);
    st.torque = uintToFloat(t_u, -T_MAX, T_MAX);
    st.temp_x10 = (int16_t)((d[6] << 8) | d[7]);
    st.last_fb_ms = millis();
    st.fb_counter = (int16_t)(st.fb_counter + 1);
    // 角度(d[0-1])は±4πで折り返すので使わない(位置はmechPos)
}

void handleParamReply(const twai_message_t &rx) {
    const uint16_t data2 = (rx.identifier >> 8) & 0xFFFF;
    const int m = motorIndexForId(data2 & 0xFF);
    if (m < 0 || (data2 >> 8) != 0) {
        return; // 対象外 or 読み出し失敗
    }
    const uint16_t index = (uint16_t)(rx.data[0] | (rx.data[1] << 8));
    if (index != PARAM_MECH_POS) {
        return;
    }
    float value;
    std::memcpy(&value, &rx.data[4], 4);
    if (std::isfinite(value)) {
        g_motor[m].mech_pos = value;
        g_motor[m].have_mech_pos = true;
    }
}

void handleIdReply(const twai_message_t &rx) {
    const int16_t id = (int16_t)((rx.identifier >> 8) & 0xFF);
    for (int i = 0; i < g_found_count && i < DIAG_ID_SLOTS; i++) {
        if (g_found_ids[i] == id) {
            return;
        }
    }
    if (g_found_count < DIAG_ID_SLOTS) {
        g_found_ids[g_found_count] = id;
    }
    g_found_count++;
}

void receiveAll() {
    twai_message_t rx;
    while (twai_receive(&rx, 0) == ESP_OK) {
        if (!rx.extd || rx.rtr || rx.data_length_code != 8) {
            continue; // RobStride独自プロトコルのフレームではない
        }
        const uint8_t comm_type = (rx.identifier >> 24) & 0x1F;
        const uint8_t dest = rx.identifier & 0xFF;
        if (dest == ID_REPLY_DEST && comm_type == COMM_GET_ID) {
            handleIdReply(rx);
            continue;
        }
        if (dest != ROBSTRIDE_HOST_ID) {
            continue;
        }
        if (comm_type == COMM_FEEDBACK || comm_type == COMM_ACTIVE_REPORT) {
            handleFeedback(rx);
        } else if (comm_type == COMM_PARAM_READ) {
            handleParamReply(rx);
        }
    }
}

void publishFeedback() {
    for (int m = 0; m < ROBSTRIDE_MOTOR_COUNT; m++) {
        const MotorRuntime &st = g_motor[m];
        uint16_t flags = st.fault_bits;
        if (st.latched) {
            flags |= FLAG_LATCHED;
        }
        if (!st.have_mech_pos) {
            flags |= FLAG_POS_UNKNOWN;
        }
        const int32_t pos = physicalToInt32(st.mech_pos, POS_SCALE);
        const int16_t values[SLOTS_PER_MOTOR] = {
            (int16_t)st.mode_bits,
            (int16_t)flags,
            st.fb_counter,
            (int16_t)((uint32_t)pos >> 16),
            (int16_t)((uint32_t)pos & 0xFFFF),
            physicalToSlot(st.velocity, VEL_SCALE),
            physicalToSlot(st.torque, TORQUE_SCALE),
            st.temp_x10,
        };
        serialWriteTx(m * SLOTS_PER_MOTOR, values, SLOTS_PER_MOTOR);
    }
    const int16_t diag[1 + DIAG_ID_SLOTS] = {g_found_count, g_found_ids[0], g_found_ids[1],
                                            g_found_ids[2]};
    serialWriteTx(DIAG_SLOT, diag, 1 + DIAG_ID_SLOTS);
}

} // namespace

void robstrideInit() {
    for (int m = 0; m < ROBSTRIDE_MOTOR_COUNT; m++) {
        g_motor[m].latched = true; // 起動直後はPCが0を送るまで動かさない
    }
    // 受信キューを広めに取る(IDスキャンの応答・帰還がまとめて届くため)
    if (!twaiInstall(32)) {
        Serial.println("[ERR] robstride: TWAI install failed");
        while (1)
            ;
    }
}

void robstrideTask(void *pvParameters) {
    while (1) {
        twaiRecoverIfNeeded();
        receiveAll();
        sendCommands();
        pollWhileDisabled();
        pollMechPos();
        bool any_enabled = false;
        for (int m = 0; m < ROBSTRIDE_MOTOR_COUNT; m++) {
            any_enabled = any_enabled || g_motor[m].enabled;
        }
        if (!any_enabled) {
            scanIfNoFeedback(); // 運転中はスキャンしない(指令の邪魔をしないため)
        }
        receiveAll();
        publishFeedback();
        vTaskDelay(pdMS_TO_TICKS(4)); // 送信の間隔と合わせて約200Hz
    }
}
