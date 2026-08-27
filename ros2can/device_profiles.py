"""xiao_esp32_s3_smd_serial_bridge (MODE_CAN_HOST) のスロット割り当て定義。

この基板は USB-シリアルで ROS 2 (serial_bridge) とつながる「CANホスト」であり、
自身の配下に CAN バス経由で最大 CAN_NODE_COUNT 台の子マイコン(ノード)をぶら下げ、
1ノードあたり CAN_SLOTS_PER_NODE 個のスロットを担当させる。
ホストは USB 側では通常の serial_bridge フレーム (24 x int16, TX/RX) を使い、
その 24 スロットを CAN バス上の各ノードへ分配/集約する。

[xiao_esp32_s3_smd_serial_bridge/src/frame_data.hpp, can_task.cpp, config.hpp より]

  実機はDCモータ非搭載。ENCx2, SWx3, SERVOx3のみで、SERVOn/SWn はピン共有
  (config.hpp の MULTIn で切替、0=スイッチ入力/1=サーボ出力)。

  ノードごとの担当スロット (CAN_SLOTS_PER_NODE = 5):
    指令 (ROS -> ホスト -> CAN -> ノード):
      0: SERVO1, 1: SERVO2, 2: SERVO3, 3: 予備, 4: 予備
    帰還 (ノード -> CAN -> ホスト -> ROS):
      0: SW1, 1: SW2, 2: SW3, 3: ENC1, 4: ENC2

  グローバルスロット index = node_index * CAN_SLOTS_PER_NODE + local_index
  (node_index は 0-origin。ノードの CAN_ID は 101,102,103,104 のように
   下2桁が (node_index+1) になるよう設定する: CAN_NODE_INDEX = (CAN_ID % 100) - 1)

config.hpp で CAN_NODE_COUNT / CAN_SLOTS_PER_NODE を変更した場合は、
GUI の「プロファイル編集」でノード数/スロット数を調整したカスタムプロファイルを
作成すること。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

SLOT_COUNT = 24

# ファームウェア既定値 (config.hpp)
DEFAULT_CAN_NODE_COUNT = 4
DEFAULT_CAN_SLOTS_PER_NODE = 5

# --- 送信 (ROS -> ホスト -> CAN -> ノード, actuator command) ---
MOTOR = "motor"          # 双極 PWM 指令 (DCモータ)
SERVO = "servo"          # サーボ角度指令 [deg]
DIGITAL_OUT = "digital_out"  # ON/OFF 出力
ENUM_OUT = "enum_out"     # 選択式の指令値
RAW_OUT = "raw_out"       # 生の int16 指令値

# --- 受信 (ノード -> CAN -> ホスト -> ROS, sensor feedback) ---
COUNTER = "counter"       # 符号付きカウンタ (エンコーダ等)
DIGITAL_IN = "digital_in"  # ON/OFF 入力 (スイッチ等)
READOUT = "readout"       # スケール付き数値表示
ENUM_IN = "enum_in"       # 選択肢付き数値表示
RAW_IN = "raw_in"         # 生の int16 値表示


@dataclass
class ChannelDef:
    index: int
    label: str
    kind: str
    group: str = ""
    min: int = -32768
    max: int = 32767
    step: int = 1
    unit: str = ""
    scale: float = 1.0
    decimals: int = 0
    options: Optional[List[Tuple[int, str]]] = None
    note: str = ""
    # 「原点セット」ボタン(GUI)/zero_channelサービスの対象にするか。
    # 角度・位置・カウンタ等、蓄積されて機構上の原点とズレうる値にのみTrueにする。
    # 同じREADOUT種別でも速度/電流/温度等の瞬時値には付けない(原点という概念が無い)。
    zeroable: bool = False

    def display_value(self, raw: int) -> float:
        return raw * self.scale

    def raw_from_display(self, value: float) -> int:
        if self.scale == 0:
            return int(value)
        return int(round(value / self.scale))


@dataclass
class DeviceProfile:
    key: str
    name: str
    description: str = ""
    tx: List[ChannelDef] = field(default_factory=list)
    rx: List[ChannelDef] = field(default_factory=list)
    node_count: int = 0
    slots_per_node: int = 0
    editable: bool = False

    def tx_by_index(self):
        return {c.index: c for c in self.tx}

    def rx_by_index(self):
        return {c.index: c for c in self.rx}


def _raw_out(index: int) -> ChannelDef:
    return ChannelDef(index, f"slot{index}", RAW_OUT, group="raw", min=-32768, max=32767)


def _raw_in(index: int) -> ChannelDef:
    return ChannelDef(index, f"slot{index}", RAW_IN, group="raw", min=-32768, max=32767)


def _fill_remaining(defs: List[ChannelDef], factory) -> List[ChannelDef]:
    used = {c.index for c in defs}
    out = list(defs)
    for i in range(SLOT_COUNT):
        if i not in used:
            out.append(factory(i))
    out.sort(key=lambda c: c.index)
    return out


def make_generic_raw_profile() -> DeviceProfile:
    """CANノード分配を意識せず、ホストのUARTフレーム24スロットをそのまま扱う汎用プロファイル。"""
    tx = [_raw_out(i) for i in range(SLOT_COUNT)]
    rx = [_raw_in(i) for i in range(SLOT_COUNT)]
    return DeviceProfile(
        key="generic_raw",
        name="汎用 Raw (24スロット、CAN分配なし)",
        description="ホストが送受信するUARTフレーム24スロットをそのまま直接編集/表示する。",
        tx=tx,
        rx=rx,
    )


def _append_generic_io_node_channels(
    tx: List[ChannelDef], rx: List[ChannelDef],
    node: int, slots_per_node: int,
    servo_min_deg: int, servo_max_deg: int,
) -> None:
    """汎用IOノード (xiao_esp32_s3_smd_serial_bridge 系, ENCx2/SWx3/SERVOx3) の
    1ノード分のチャンネルを tx/rx に追加する。SERVOn と SWn はピン共有
    (ファームウェア config.hpp の MULTIn で入出力を切替、n=1..3、0=スイッチ入力/1=サーボ出力)。
    """
    base = node * slots_per_node
    node_no = node + 1
    can_id = 100 + node_no
    group_cmd = f"ノード{node_no} (CAN_ID={can_id}) 指令"
    group_fb = f"ノード{node_no} (CAN_ID={can_id}) 帰還"

    servo_slots = min(3, slots_per_node)
    for s in range(servo_slots):
        tx.append(ChannelDef(base + s, f"N{node_no} SERVO{s + 1}", SERVO,
                              group=group_cmd, min=servo_min_deg, max=servo_max_deg, unit="deg",
                              note=f"SW{s + 1} とピン共有。MULTI{s + 1}=1(サーボ)のときのみ有効"))

    sw_slots = min(3, slots_per_node)
    for s in range(sw_slots):
        rx.append(ChannelDef(base + s, f"N{node_no} SW{s + 1}", DIGITAL_IN, group=group_fb,
                              note=f"SERVO{s + 1} とピン共有。MULTI{s + 1}=0(スイッチ)のときのみ有効"))
    if slots_per_node > 3:
        rx.append(ChannelDef(base + 3, f"N{node_no} ENC1", COUNTER, group=group_fb, unit="count",
                              zeroable=True))
    if slots_per_node > 4:
        rx.append(ChannelDef(base + 4, f"N{node_no} ENC2", COUNTER, group=group_fb, unit="count",
                              zeroable=True))


def _append_foc_motor_node_channels(
    tx: List[ChannelDef], rx: List[ChannelDef],
    node: int, slots_per_node: int,
) -> None:
    """b-g431-esc1_can2io (SimpleFOC, 速度制御のみ) の1ノード分のチャンネルを追加する。

    firmware/b-g431-esc1_can2io/src/config.hpp のスロット割当 (CAN_SLOTS_PER_NODE=5、
    実使用は指令1/帰還3スロットのみ) と一致させること。xiao-esp32-s3_can2io の
    MODE_ROBOMAS と同じデータモデル(target_velocityは生rpm値、angleは0.1deg、
    currentは0.001A)。ゲイン類はCAN経由では送れず、ファーム側のコンパイル時定数固定。
    """
    base = node * slots_per_node
    node_no = node + 1
    can_id = 100 + node_no
    group_cmd = f"ノード{node_no} (CAN_ID={can_id}, FOCモータ) 指令"
    group_fb = f"ノード{node_no} (CAN_ID={can_id}, FOCモータ) 帰還"

    tx.append(ChannelDef(base + 0, f"N{node_no} target_velocity", MOTOR, group=group_cmd,
                          unit="rpm", note="出力軸rpm、スケール無し(robomas互換)"))

    rx.append(ChannelDef(base + 0, f"N{node_no} angle", READOUT, group=group_fb,
                          scale=0.1, unit="deg", decimals=1, zeroable=True))
    rx.append(ChannelDef(base + 1, f"N{node_no} velocity", READOUT, group=group_fb,
                          unit="rpm", decimals=0))
    rx.append(ChannelDef(base + 2, f"N{node_no} current_q", READOUT, group=group_fb,
                          scale=0.001, unit="A", decimals=3))


def make_can_host_profile(
    key: str = "xiao_smd_can_host",
    name: str = "XIAO ESP32S3 SMD (CAN Host)",
    node_count: int = DEFAULT_CAN_NODE_COUNT,
    slots_per_node: int = DEFAULT_CAN_SLOTS_PER_NODE,
    servo_min_deg: int = 0,
    servo_max_deg: int = 270,
) -> DeviceProfile:
    """xiao_esp32_s3_smd_serial_bridge MODE_CAN_HOST 用プロファイル。

    実機はDCモータ非搭載。ENCx2, SWx3, SERVOx3のみで、SERVOn と SWn は
    ピン共有 (ファームウェア config.hpp の MULTIn で入出力を切替、
    n=1..3, 0=スイッチ入力/1=サーボ出力)。

    24スロットを CAN バス上の最大 node_count ノードへ slots_per_node ずつ分配する。
    各ノード (既定 5スロット):
      指令: SERVO1, SERVO2, SERVO3, (予備, 予備)
      帰還: SW1, SW2, SW3, ENC1, ENC2
    CAN_ID は 101,102,103,104 のようにノード番号 (1-origin) を下2桁に持つ。
    """
    tx: List[ChannelDef] = []
    rx: List[ChannelDef] = []

    for node in range(node_count):
        _append_generic_io_node_channels(tx, rx, node, slots_per_node, servo_min_deg, servo_max_deg)

    tx = _fill_remaining(tx, _raw_out)
    rx = _fill_remaining(rx, _raw_in)

    return DeviceProfile(
        key=key,
        name=name,
        description=(
            f"MODE_CAN_HOST 用。実機はDCモータ非搭載。24スロットを CAN バス上の最大"
            f"{node_count}ノードへ{slots_per_node}スロットずつ分配する。各ノードは "
            "SERVO1-3 (指令) / SW1-3 + ENC1-2 (帰還) を担当 (SERVOn/SWnはピン共有、"
            "config.hppのMULTInで切替)。ノード数・スロット数はプロファイル編集で変更可能。"
        ),
        tx=tx,
        rx=rx,
        node_count=node_count,
        slots_per_node=slots_per_node,
    )


def make_can_host_with_foc_node_profile(
    key: str = "xiao_can2io_with_foc",
    name: str = "xiao-esp32-s3_can2io + b-g431-esc1_can2io (FOCモータ, robomas互換)",
    node_count: int = 2,
    slots_per_node: int = DEFAULT_CAN_SLOTS_PER_NODE,
    foc_node_index: int = 1,
    servo_min_deg: int = 0,
    servo_max_deg: int = 270,
) -> DeviceProfile:
    """xiao-esp32-s3_can2io (MODE_CAN_HOST) 配下に b-g431-esc1_can2io (SimpleFOCの
    CANノード、速度制御のみ) を1台混在させたプロファイル。

    foc_node_index 番目のノードだけFOCモータ用チャンネルにし、それ以外は
    make_can_host_profile と同じ汎用IOノード(SERVO/SW/ENC)のまま扱う。

    デフォルト値は現在の実機構成(ホスト自身がnode0、b-g431がCAN_ID=102→node1の
    計2台のみ接続、firmware/xiao-esp32-s3_can2io/src/config.hppのCAN_NODE_COUNT=2)
    に合わせてある。ノード数や接続位置を変える場合はここも一緒に変更すること。
    """
    tx: List[ChannelDef] = []
    rx: List[ChannelDef] = []

    for node in range(node_count):
        if node == foc_node_index:
            _append_foc_motor_node_channels(tx, rx, node, slots_per_node)
        else:
            _append_generic_io_node_channels(tx, rx, node, slots_per_node, servo_min_deg, servo_max_deg)

    tx = _fill_remaining(tx, _raw_out)
    rx = _fill_remaining(rx, _raw_in)

    return DeviceProfile(
        key=key,
        name=name,
        description=(
            f"MODE_CAN_HOST 用。ノード{foc_node_index + 1}を b-g431-esc1_can2io "
            "(SimpleFOC, 速度制御のみ、xiao-esp32-s3_can2io の MODE_ROBOMAS と互換の"
            "データモデル) に割り当て、残りは汎用IOノード (SERVO1-3指令 / SW1-3+ENC1-2帰還) "
            "として扱う。FOCノードのゲインはCAN経由では変更できず、ファーム側config.hppの"
            "コンパイル時定数固定。"
        ),
        tx=tx,
        rx=rx,
        node_count=node_count,
        slots_per_node=slots_per_node,
    )


def make_robomas_profile(
    key: str = "robomas_driver",
    name: str = "xiao-esp32-s3_can2io (MODE_ROBOMAS, DJIロボマス x4)",
) -> DeviceProfile:
    """MODE_ROBOMAS (xiao-esp32-s3_can2io) 用プロファイル。

    ノード/スロット分配は行わない独立デバイスとして、24スロットをロボマス最大4台分の
    指令/帰還に直接割り当てる(スロット0起点、ノードオフセット無し)。
    [firmware/xiao-esp32-s3_can2io/src/robomas.cpp のスロット割当と一致させること]

    モータ機種(M3508/M2006/GM6020のいずれか)はファーム側config.hppの
    ROBOMAS_MOTOR_TYPEでコンパイル時固定。速度PIDゲインもCAN経由では変更できず
    ファーム側固定(config.hppのROBOMAS_KP_VEL等)。1バスには単一機種のみ、最大4台。
    """
    tx: List[ChannelDef] = []
    rx: List[ChannelDef] = []

    for i in range(4):
        m = i + 1
        tx.append(ChannelDef(i, f"M{m} target_velocity", MOTOR, group="ロボマス 指令",
                              min=-450, max=450,
                              unit="rpm",
                              note="出力軸rpm(ギア比込み)、スケール無し。モータ機種はconfig.hppで固定。"
                                   "GUIのスライダー横の範囲欄で上下限を変更可能(デフォルト±450rpm)"))

    for i in range(4):
        m = i + 1
        rx.append(ChannelDef(i, f"M{m} angle", READOUT, group="ロボマス 帰還",
                              scale=0.1, unit="deg", decimals=1, note="出力軸角度", zeroable=True))
    for i in range(4):
        m = i + 1
        rx.append(ChannelDef(4 + i, f"M{m} velocity", READOUT, group="ロボマス 帰還",
                              unit="rpm", decimals=0, note="出力軸rpm"))
    for i in range(4):
        m = i + 1
        rx.append(ChannelDef(8 + i, f"M{m} current", READOUT, group="ロボマス 帰還",
                              scale=0.001, unit="A", decimals=3))

    tx = _fill_remaining(tx, _raw_out)
    rx = _fill_remaining(rx, _raw_in)

    return DeviceProfile(
        key=key,
        name=name,
        description=(
            "MODE_ROBOMAS用。ノード/スロット分配は行わず、独立デバイスとしてロボマス"
            "(M3508/M2006/GM6020のいずれか、config.hppのROBOMAS_MOTOR_TYPEで固定)を"
            "最大4台まで速度制御する。速度PIDゲインはCAN経由では変更できず、ファーム側"
            "config.hppのコンパイル時固定。ノード/スロット分配側(MODE_CAN等)も1Mbpsに"
            "統一済みのため同一物理バスへの混在は可能(CAN IDは重ならない設計)だが、"
            "帯域を確保するため指令送信は200Hzに固定(ファーム側robomas.cpp)。台数を"
            "増やす場合はバス帯域の余裕を確認すること(firmware/xiao-esp32-s3_can2io/"
            "README.md参照)。"
        ),
        tx=tx,
        rx=rx,
    )


CUBEMARS_ERROR_CODES: List[Tuple[int, str]] = [
    (0, "no fault"),
    (1, "motor over-temp"),
    (2, "over-current"),
    (3, "over-voltage"),
    (4, "under-voltage"),
    (5, "encoder fault"),
    (6, "MOSFET over-temp"),
    (7, "motor stall"),
]


def make_cubemars_profile(
    key: str = "cubemars_ak_driver",
    name: str = "xiao-esp32-s3_can2io (MODE_CUBEMARS, CubeMars AKシリーズ x4)",
) -> DeviceProfile:
    """MODE_CUBEMARS (xiao-esp32-s3_can2io) 用プロファイル。

    ノード/スロット分配は行わない独立デバイスとして、24スロットをCubeMars AKシリーズ
    (AK40-10等、Servo(CAN)モード / MIT(Force Control)モード)最大4台分の指令/帰還に
    直接割り当てる(スロット0起点、ノードオフセット無し)。
    [firmware/xiao-esp32-s3_can2io/src/cubemars.cpp のスロット割当と一致させること]

    速度/位置ともアクチュエータ内蔵のクローズドループがそのまま追従するため、
    ロボマスのGM6020と違いホスト側PIDは無い。各モータのCAN IDはファーム側
    config.hppのCUBEMARS_MOTOR_ID_nでコンパイル時固定(R-Linkの設定と一致させること)。
    1バスには最大4台まで(config.hppのCUBEMARS_MOTOR_COUNT)。

    MITモード (control_mode=2) は位置・速度・Kp・Kd・トルクFFを同時に指令し、
    モータ側で torque = Kp*(pos_des-pos) + Kd*(vel_des-vel) + torque_ff を計算する
    インピーダンス制御 (AK Series Module Product Manual V3.2.0 4.2節)。
    mit_velocity/mit_kp/mit_kd/mit_torque_ff スロットはcontrol_mode=2のときのみ
    参照される。クランプ範囲(スライダーの範囲欄)はファーム側config.hppの
    CUBEMARS_MIT_*と必ず一致させ、実機のR-Link設定と揃えること
    (velocity/torqueはAK Series Module Driver Manual V1.0.18 5.3節のAK40-10行の値。
    ファーム側を書き換えた場合はここも合わせて調整すること)。
    """
    tx: List[ChannelDef] = []
    rx: List[ChannelDef] = []

    for i in range(4):
        m = i + 1
        group_cmd = f"M{m} 指令"
        tx.append(ChannelDef(i, f"M{m} target", MOTOR, group=group_cmd,
                              note="control_modeが速度のとき10ERPM/LSB(電気角速度)、"
                                   "位置/MITのとき0.1deg/LSB。GUI上はスケール無しの生値。"))
    for i in range(4):
        m = i + 1
        group_cmd = f"M{m} 指令"
        tx.append(ChannelDef(4 + i, f"M{m} control_mode", ENUM_OUT, group=group_cmd,
                              options=[(0, "速度ループ"), (1, "位置ループ"), (2, "MIT(Force Control)")],
                              note="全ゼロ(未接続/E-STOP時の既定)で速度ループ・target=0となり、"
                                   "安全にその場停止する(位置ジャンプは発生しない)。MIT選択時は"
                                   "mit_velocity/mit_kp/mit_kd/mit_torque_ffも参照される"))
    for i in range(4):
        m = i + 1
        group_cmd = f"M{m} 指令"
        tx.append(ChannelDef(8 + i, f"M{m} mit_velocity", MOTOR, group=group_cmd,
                              min=-4550, max=4550, scale=0.01, unit="rad/s", decimals=2,
                              note="MITモード用の目標速度(control_mode=2のときのみ参照)。"
                                   "0.01rad/s/LSB、レンジ±45.5rad/s(config.hppのCUBEMARS_MIT_V_MIN/MAX_RADPSと"
                                   "一致させること。ファーム側で変更した場合はここも合わせて調整すること)"))
    for i in range(4):
        m = i + 1
        group_cmd = f"M{m} 指令"
        tx.append(ChannelDef(12 + i, f"M{m} mit_kp", MOTOR, group=group_cmd,
                              min=0, max=5000, scale=0.1, unit="", decimals=1,
                              note="MITモード用のKp(control_mode=2のときのみ参照)。0.1/LSB、"
                                   "レンジ0-500(マニュアル共通値)"))
    for i in range(4):
        m = i + 1
        group_cmd = f"M{m} 指令"
        tx.append(ChannelDef(16 + i, f"M{m} mit_kd", MOTOR, group=group_cmd,
                              min=0, max=500, scale=0.01, unit="", decimals=2,
                              note="MITモード用のKd(control_mode=2のときのみ参照)。0.01/LSB、"
                                   "レンジ0-5(マニュアル共通値)"))
    for i in range(4):
        m = i + 1
        group_cmd = f"M{m} 指令"
        tx.append(ChannelDef(20 + i, f"M{m} mit_torque_ff", MOTOR, group=group_cmd,
                              min=-500, max=500, scale=0.01, unit="N・m", decimals=2,
                              note="MITモード用のトルクFF(control_mode=2のときのみ参照)。"
                                   "0.01N・m/LSB、レンジ±5.0N・m(config.hppのCUBEMARS_MIT_T_MIN/MAX_NMと"
                                   "一致させること。ファーム側で変更した場合はここも合わせて調整すること)"))

    for i in range(4):
        m = i + 1
        group_fb = f"M{m} 帰還"
        rx.append(ChannelDef(i, f"M{m} position", READOUT, group=group_fb,
                              scale=0.1, unit="deg", decimals=1, zeroable=True))
    for i in range(4):
        m = i + 1
        group_fb = f"M{m} 帰還"
        rx.append(ChannelDef(4 + i, f"M{m} speed", READOUT, group=group_fb,
                              scale=10, unit="ERPM", decimals=0,
                              note="電気角速度。出力軸rpmへの換算(極対数・減速比)は"
                                   "モータ機種依存のためGUI側では未実施"))
    for i in range(4):
        m = i + 1
        group_fb = f"M{m} 帰還"
        rx.append(ChannelDef(8 + i, f"M{m} current", READOUT, group=group_fb,
                              scale=0.01, unit="A", decimals=2))
    for i in range(4):
        m = i + 1
        group_fb = f"M{m} 帰還"
        rx.append(ChannelDef(12 + i, f"M{m} temperature", READOUT, group=group_fb,
                              unit="degC", decimals=0))
    for i in range(4):
        m = i + 1
        group_fb = f"M{m} 帰還"
        rx.append(ChannelDef(16 + i, f"M{m} error", ENUM_IN, group=group_fb,
                              options=list(CUBEMARS_ERROR_CODES)))

    tx = _fill_remaining(tx, _raw_out)
    rx = _fill_remaining(rx, _raw_in)

    return DeviceProfile(
        key=key,
        name=name,
        description=(
            "MODE_CUBEMARS用。ノード/スロット分配は行わず、独立デバイスとしてCubeMars "
            "AKシリーズ(Servo(CAN)モードの速度/位置制御、およびMIT(Force Control)モード)"
            "を最大4台まで制御する。アクチュエータ内蔵のクローズドループがそのまま追従する"
            "ためホスト側PIDは無し。MITモードはKp/Kdを使ったインピーダンス制御で、"
            "mit_velocity/mit_kp/mit_kd/mit_torque_ffのクランプ範囲(スライダー範囲欄)は"
            "実機のR-Link設定・ファーム側config.hppのCUBEMARS_MIT_*と必ず一致させること。"
            "CAN IDはファーム側"
            "config.hppのCUBEMARS_MOTOR_ID_nで固定、R-Linkの設定と一致させること。"
            "ノード/スロット分配側(MODE_CAN等)も1Mbpsに統一済みのため同一物理バスへの"
            "混在は可能(CAN IDは重ならない設計)だが、帯域を確保するため指令送信は"
            "200Hzに固定(ファーム側cubemars.cpp)。アクチュエータ自身の帰還フレームは"
            "指令頻度に関係なく自律送信されるため、台数を増やす場合はバス帯域の余裕を"
            "確認すること(firmware/xiao-esp32-s3_can2io/README.md参照)。"
        ),
        tx=tx,
        rx=rx,
    )


def _build_builtin_profiles() -> "dict[str, DeviceProfile]":
    profiles: List[DeviceProfile] = [
        make_can_host_profile(),
        make_can_host_with_foc_node_profile(),
        make_robomas_profile(),
        make_cubemars_profile(),
        make_generic_raw_profile(),
    ]
    return {p.key: p for p in profiles}


BUILTIN_PROFILES = _build_builtin_profiles()

DEFAULT_PROFILE_KEY = "xiao_smd_can_host"


# ================= カスタムプロファイルの保存/読込 =================
#
# GUI の「プロファイル編集」で作成/変更したプロファイルは
# ~/.config/ros2can/profiles/*.json に保存し、次回起動時に読み込む。
# CAN_NODE_COUNT / CAN_SLOTS_PER_NODE をファームウェア側で変更した場合に利用する。

import json
import os


def custom_profiles_dir() -> str:
    base = os.environ.get("XDG_CONFIG_HOME", os.path.expanduser("~/.config"))
    path = os.path.join(base, "ros2can", "profiles")
    os.makedirs(path, exist_ok=True)
    return path


def _channel_to_dict(c: ChannelDef) -> dict:
    return {
        "index": c.index, "label": c.label, "kind": c.kind, "group": c.group,
        "min": c.min, "max": c.max, "step": c.step, "unit": c.unit,
        "scale": c.scale, "decimals": c.decimals, "options": c.options, "note": c.note,
        "zeroable": c.zeroable,
    }


def _channel_from_dict(d: dict) -> ChannelDef:
    options = d.get("options")
    if options is not None:
        options = [tuple(o) for o in options]
    return ChannelDef(
        index=d["index"], label=d.get("label", f"slot{d['index']}"),
        kind=d.get("kind", RAW_OUT), group=d.get("group", ""),
        min=d.get("min", -32768), max=d.get("max", 32767), step=d.get("step", 1),
        unit=d.get("unit", ""), scale=d.get("scale", 1.0), decimals=d.get("decimals", 0),
        options=options, note=d.get("note", ""), zeroable=d.get("zeroable", False),
    )


def profile_to_dict(p: DeviceProfile) -> dict:
    return {
        "key": p.key, "name": p.name, "description": p.description,
        "node_count": p.node_count, "slots_per_node": p.slots_per_node,
        "tx": [_channel_to_dict(c) for c in p.tx],
        "rx": [_channel_to_dict(c) for c in p.rx],
    }


def profile_from_dict(d: dict) -> DeviceProfile:
    return DeviceProfile(
        key=d["key"], name=d.get("name", d["key"]), description=d.get("description", ""),
        tx=[_channel_from_dict(c) for c in d.get("tx", [])],
        rx=[_channel_from_dict(c) for c in d.get("rx", [])],
        node_count=d.get("node_count", 0), slots_per_node=d.get("slots_per_node", 0),
        editable=True,
    )


def save_custom_profile(p: DeviceProfile) -> str:
    path = os.path.join(custom_profiles_dir(), f"{p.key}.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(profile_to_dict(p), f, ensure_ascii=False, indent=2)
    return path


def delete_custom_profile(key: str) -> None:
    path = os.path.join(custom_profiles_dir(), f"{key}.json")
    if os.path.exists(path):
        os.remove(path)


def load_custom_profiles() -> "dict[str, DeviceProfile]":
    result = {}
    directory = custom_profiles_dir()
    for fname in sorted(os.listdir(directory)):
        if not fname.endswith(".json"):
            continue
        try:
            with open(os.path.join(directory, fname), "r", encoding="utf-8") as f:
                d = json.load(f)
            p = profile_from_dict(d)
            result[p.key] = p
        except (OSError, ValueError, KeyError):
            continue
    return result


def all_profiles() -> "dict[str, DeviceProfile]":
    merged = dict(BUILTIN_PROFILES)
    merged.update(load_custom_profiles())
    return merged


def unique_custom_key(base_key: str, existing: "dict[str, DeviceProfile]") -> str:
    candidate = f"{base_key}_custom"
    n = 2
    while candidate in existing:
        candidate = f"{base_key}_custom{n}"
        n += 1
    return candidate
