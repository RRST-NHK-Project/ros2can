/*====================================================================
<config.h>
書き込み前にここでIDと動作モードを設定してください．MDやサーボの設定もここで行います．
MDは基本的に変更不要ですが，サーボは型番、機構に応じて適切に設定する必要があります．
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once
#include <Arduino.h>

// ================= 基本設定 =================

// IDの設定，シリアルフレームのDEVICE_IDとして使用します。
#define DEVICE_ID 101

// CAN_IDは3桁形式で指定します。
// 1桁目はバス番号、末尾2桁はノード番号を表します。
// 例: 101, 102, 103, 104
#define CAN_ID 101

// モードの設定，どれか一つをコメントアウト解除すること
// #define MODE_CAN
// define MODE_CAN_HOST
// #define MODE_IO
// #define MODE_DEBUG
// #define MODE_CAN_MONITOR
// #define MODE_ROBOMAS
#define MODE_CUBEMARS

// ================= サーボ関連 =================

// サーボ関連の設定、使用するサーボに応じて変更
#define SERVO_PWM_FREQ 50       // サーボPWM周波数（Hz）
#define SERVO_PWM_RESOLUTION 14 // サーボPWM分解能（bit）

// サーボの最小・最大パルス幅、角度範囲、初期角度の設定
#define SERVO1_MIN_US 500
#define SERVO1_MAX_US 2500
#define SERVO1_MIN_DEG 0
#define SERVO1_MAX_DEG 270
#define SERVO1_INIT_DEG 0

#define SERVO2_MIN_US 500
#define SERVO2_MAX_US 2500
#define SERVO2_MIN_DEG 0
#define SERVO2_MAX_DEG 270
#define SERVO2_INIT_DEG 0

#define SERVO3_MIN_US 500
#define SERVO3_MAX_US 2500
#define SERVO3_MIN_DEG 0
#define SERVO3_MAX_DEG 270
#define SERVO3_INIT_DEG 0

#define SERVO4_MIN_US 500
#define SERVO4_MAX_US 2500
#define SERVO4_MIN_DEG 0
#define SERVO4_MAX_DEG 270
#define SERVO4_INIT_DEG 0

// ================= 高度な設定（通常は変更不要） =================

// 以下の設定は必要に応じて変更
#define ENABLE_LED 1 // 状態表示LEDを有効にする場合1に設定

// 汎用（MULTI）ポートの設定（スイッチ:0, サーボ:1）
#define MULTI1 0
#define MULTI2 0
#define MULTI3 0

// CANのノード割り当て設定
// 1つのCANバス上で最大4ノードまで対応し、1ノードあたり5スロットをCANで送受信する
// 重要: ここは「対応可能な最大数」ではなく「実際にバスへ接続されているノード数
// (ホスト自身を含む)」に必ず合わせること。ホストは毎周期、自分以外の
// [0, CAN_NODE_COUNT) 全ノードへ指令フレームを送信するため、存在しないノード宛の
// 分だけACKエラーが発生し続け、ホスト自身のCANコントローラがBus-Offに陥る
// (実測: 4ノード設定・実接続2台の場合、起動後約40msでBus-Off)。
// 現在の実接続: ホスト(node0)+STM32 b-g431-esc1_can2io(node1)の2台のみ。
#define CAN_NODE_COUNT 2
#define CAN_SLOTS_PER_NODE 5
// CAN_ID の下位2桁をノード番号として使用する。
// 例: CAN_ID=101 -> node 1, CAN_ID=102 -> node 2, CAN_ID=103 -> node 3, CAN_ID=104 -> node 4
#define CAN_NODE_INDEX ((CAN_ID % 100U) - 1U)

// ================= CANモニタ関連 (MODE_CAN_MONITORのみ有効) =================
// バス上の任意のトラフィックを観測するための設定。ホスト用のCAN_NODE_COUNTとは
// 独立しており、ここで設定した範囲外のノードから来たフレームも
// CAN_MONITOR_RAW_ENABLEが1であれば生データとして出力される(汎用スニファ)。

#define CAN_MONITOR_RAW_ENABLE 1     // 1=受信した全フレームをID/DLC/生バイトでそのまま出力
#define CAN_MONITOR_SUMMARY_ENABLE 1 // 1=node/slot単位にデコードした要約も併せて出力
#define CAN_MONITOR_MAX_NODES 16     // 要約デコード対象のノード数上限(CAN_NODE_COUNTと無関係)

// ================= ロボマス関連 (MODE_ROBOMASのみ有効) =================
// MODE_ROBOMASはxiao-esp32-s3_can2ioのノード/スロット分配方式(CAN 500kbps)とは
// 別系統で、DJI RoboMasterシリーズのCANプロトコル(1Mbps固定, ID固定)を直接喋る
// 独立デバイスとして動作する。1マイコン(1バス)には同一機種のみ最大4台まで接続可能。
// このバスにxiao-esp32-s3_can2ioの他ノード(MODE_CAN等)を混在させることはできない
// (ビットレートが異なるため)。詳細はREADME.md参照。

#define ROBOMAS_MOTOR_M3508 1  // C620 + M3508 (メカナム/足回り等、ギア比19.2)
#define ROBOMAS_MOTOR_M2006 2  // C610 + M2006 (小型アクチュエータ等、ギア比36.0)
#define ROBOMAS_MOTOR_GM6020 3 // GM6020 (ダイレクトドライブ、ギア無し)

// 使用するモータ機種を1つ選択すること。
#define ROBOMAS_MOTOR_TYPE ROBOMAS_MOTOR_GM6020

// 速度PIDゲイン。ros2can(PC)側からは変更できない固定値。チューニングはここで行う。
#if ROBOMAS_MOTOR_TYPE == ROBOMAS_MOTOR_M3508
#define ROBOMAS_KP_VEL 0.8f
#define ROBOMAS_KI_VEL 0.0f
#define ROBOMAS_KD_VEL 0.05f
#define ROBOMAS_OUTPUT_GAIN 10.0f   // PID出力 -> 電流指令[A]への換算係数
#define ROBOMAS_MAX_CURRENT_A 20.0f // 電流指令の飽和値[A] (C620仕様上限)
#elif ROBOMAS_MOTOR_TYPE == ROBOMAS_MOTOR_M2006
// Kp=0.8のため誤差1.25rpm(=max_out/Kp)を超えると出力は電流上限1.0Aに張り付く。
// 目標200rpmに対し実測が183rpm付近で頭打ちなのはこの電流上限による飽和であり、
// ゲインでは解消しない(電流上限を上げない前提で追い込む場合はここが天井)。
// Kdは旧増分PID(dt未除算)向けの値0.02のままだった。PID.hppのDifferential_は
// dtで除算する実装のため、dt≈1msでは同じKd値でも実効ゲインが約1000倍になり、
// 速度センサの数rpmのノイズがそのまま数十A相当に増幅されて出力を瞬間的に
// デサチュレートさせ、リップルの原因になっていた。dt換算(Kd_new≈Kd_old*dt)で
// 0とみなせる値まで下げる。
#define ROBOMAS_KP_VEL 0.8f
#define ROBOMAS_KI_VEL 0.0f
#define ROBOMAS_KD_VEL 0.0f
#define ROBOMAS_OUTPUT_GAIN 1.0f
#define ROBOMAS_MAX_CURRENT_A 1.0f
#elif ROBOMAS_MOTOR_TYPE == ROBOMAS_MOTOR_GM6020
// PD制御 (Ki=0)。実測: P単独でKp=0.0080から振動 -> 限界感度 Ku=0.0080。
//
// Kd: PID.hppのDifferential_は -(current - last_current)/dt とdtで除算するので、
//     Kdは連続系の単位[A*s/rpm]そのまま。微分先行時間Tdとは Kd = Kp * Td。
//     ここを増分形PID(dt未除算)の感覚で書くとdt≈1msのぶん1000倍になり、速度の
//     量子化1LSB(1rpm)だけで Kd*(1/0.001)=Kd*1000 [A] のキックが出て出力が全振幅
//     で飽和・反転し、停止中でも激しく振動する(M2006の項と同じ罠)。
//     Tdの上限を決めるのもこの量子化。Td=4msなら1LSBあたり25.6mA(上限3Aの0.9%)で
//     許容範囲、Td=10ms以上ではノイズの増幅が信号を上回る。
//     Td=4ms採用 -> Kd = Kp * 0.004。Kpを動かすときはこの比を保つこと。
// Kp: ZNのPD則 0.8*Ku = 0.0064 から開始。Dの位相進みでKu自体が上がるはずなので、
//     無音なら 0.0080 -> 0.0100 と上げられる(その都度Kdも上式で追随させる)。
//
// Kiは意図的に0。ただし摩擦・コギング抗力を打ち消す電流(実測55mA)をP項だけで
// 作ることになるため、55mA/Kp = 8.6rpm の定常偏差が恒久的に残る。PもDも偏差ゼロ
// では出力ゼロなのでこれは消えない。許容できない場合のみKiを戻すこと。
#define ROBOMAS_KP_VEL 0.02f
#define ROBOMAS_KI_VEL 0.0f
#define ROBOMAS_KD_VEL 0.0000256f
#define ROBOMAS_OUTPUT_GAIN 1.0f
// sendCurGm6020()が±3.0A(±16384)で切っているので外側もそこに合わせる。
// 10.0Aのままだと飽和点が実効上限とずれ、アンチワインドアップが機能しない。
#define ROBOMAS_MAX_CURRENT_A 3.0f
#else
#error "ROBOMAS_MOTOR_TYPE: unknown motor type"
#endif

// ================= CubeMars AK関連 (MODE_CUBEMARSのみ有効) =================
// MODE_CUBEMARSはMODE_ROBOMASと同様、xiao-esp32-s3_can2ioのノード/スロット分配方式
// (CAN 500kbps)とは別系統で、CubeMars AKシリーズのServo(CAN)モードプロトコル
// (1Mbps固定、AK Series Module Product Manual V3.2.0 4.1節)を直接喋る独立デバイス
// として動作する。速度/位置ともアクチュエータ内蔵のクローズドループがそのまま
// 追従するため、ロボマスのGM6020のようなホスト側速度PIDは不要。
// 1マイコン(1バス)には最大 CUBEMARS_MOTOR_COUNT 台まで接続可能。

#define CUBEMARS_MOTOR_COUNT 4 // 1バスあたりの接続台数(最大4)

// 各モータのCAN ID (R-Link CubeMarsTool の Application Configuration -> CAN ID
// 設定と一致させること。1台ずつ重複しない値を書き込んでおく)
#define CUBEMARS_MOTOR_ID_1 101
#define CUBEMARS_MOTOR_ID_2 102
#define CUBEMARS_MOTOR_ID_3 103
#define CUBEMARS_MOTOR_ID_4 104

// ---- MIT(Force Control)モード関連 (AK Series Module Product Manual V3.2.0 4.2節) ----
// MITモードはServo(CAN)モードと同じ拡張ID方式(control_mode_id<<8 | driver_id)の
// 別モード(control_mode_id=8)で、位置・速度・トルクFF・Kp・Kdを同時に指令し
// モータ側で torque = Kp*(p_des-p) + Kd*(v_des-v) + t_ff を計算するインピーダンス制御。
// 有効化はcubemars.cpp側、ホストからの選択はcontrol_modeスロット(=2)で行う。
//
// 下記のP/V/T(位置・速度・トルク)レンジはホスト側のエンコード基準であり、
// モータ側のデコード基準(ファーム内蔵、R-Linkからは変更不可)と一致していないと
// 指令値の意味(rad, rad/s, N・m)がズレて意図しない速度・トルクが出力される。
// マニュアルのパラメータ表にAK40-10の掲載が無いため、他モータ(AK70-9/AK80-9等)の
// 値から類推した暫定値になっている。使用前に必ずCubeMars公式の最新AK40-10
// データシート、またはR-Link(CubeMarsTool)のMIT Controlタブで実機の値を確認し、
// 書き換えること。Kp/Kdレンジ(0-500 / 0-5)は全モータ共通の値としてマニュアルに
// 明記されているためそのまま使用してよい。
#define CUBEMARS_MIT_P_MIN_RAD -12.5f
#define CUBEMARS_MIT_P_MAX_RAD 12.5f
#define CUBEMARS_MIT_V_MIN_RADPS -50.0f
#define CUBEMARS_MIT_V_MAX_RADPS 50.0f
#define CUBEMARS_MIT_T_MIN_NM -18.0f
#define CUBEMARS_MIT_T_MAX_NM 18.0f
#define CUBEMARS_MIT_KP_MIN 0.0f
#define CUBEMARS_MIT_KP_MAX 500.0f
#define CUBEMARS_MIT_KD_MIN 0.0f
#define CUBEMARS_MIT_KD_MAX 5.0f
