"""rclpy と PyQt5 をつなぐバックエンド。

GUI は単一スレッドで動作させる。QTimer から `rclpy.spin_once(timeout_sec=0)` を
高頻度に呼び出すことで ROS のコールバック(サブスクライバ受信)を GUI スレッド上で
直接処理し、スレッド間排他を持ち込まずに Qt シグナルを発行できるようにしている。

デバイスは2つのモードのどちらかで管理される。

- "hardware" : ros2can 自身が `HardwareManager` 経由でシリアルポートを直接専有し、
  serial_bridge プロトコルで直接送受信する(スタンドアローン動作)。
  ROS的には自分自身が bridge_node の役割を担うため、他ノードとの互換のために
  `serial_rx_[ID]` を Publish (センサ値の提供)、`serial_tx_[ID]` を Subscribe
  (外部ノードからの指令受け付け) する。
- "topic_client" : 既に起動している serial_bridge (または他の ros2can インスタンス)
  が Publish/Subscribe している既存のトピックにこちらが相乗りするだけのモード。
  `serial_tx_[ID]` を Publish (指令送信)、`serial_rx_[ID]` を Subscribe (センサ受信)
  する、通常のクライアントの役割。
- "simulator" : 実機のマイコン/CANホストが無くても UI の動作確認ができるよう、
  TX に書き込んだ値をその場で RX にループバックして返す仮想デバイス。
  ROSトピックの Publish/Subscribe の役割は "hardware" と同じ(自分が bridge_node の
  代わりを担う)にしてあるため、他のROSノードや rqt からも実機と区別なく確認できる。
  ENC/SW等、実機ではTX指令と無関係な独立したセンサ入力に相当するRXスロットは
  Monitor タブから直接値を設定できる(set_sim_rx_value)。設定したスロットは
  TX->RXループバック対象から外れ、マイコンの実際の挙動(スイッチ操作・エンコーダ回転)を
  GUI操作で再現できる。
"""

from __future__ import annotations

import re
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Int32MultiArray

from PyQt5.QtCore import QObject, pyqtSignal

from ros2can_interfaces.srv import ZeroChannel

from .counter_unwrapper import CounterUnwrapper
from .device_profiles import DEFAULT_PROFILE_KEY, SLOT_COUNT
from .hardware_manager import HardwareConfig, HardwareManager
from .settings_store import load_settings, parse_device_profile_map

TOPIC_RE = re.compile(r"^/?(serial_tx|serial_rx)_(\d+)$")

# この時間 [s] 以上 RX が無ければ「オフライン」とみなす (表示用のヒューリスティック)
STALE_TIMEOUT_SEC = 1.5

# PCNT生カウントのリセット幅。counter_unwrapper.py 冒頭コメント参照
# (実機確認済み: h_lim=32767/l_lim=-32768到達でそれぞれ独立に0へリセットされる)。
# ENC以外のスロット(SW/SERVO/速度指令等)にも一律適用するが、それらは通常
# この半分(16384)を大きく下回る範囲でしか変化しないため無害。
_COUNTS_PER_WRAP = 32768

MODE_HARDWARE = "hardware"
MODE_TOPIC_CLIENT = "topic_client"
MODE_SIMULATOR = "simulator"


def _clamp_int16(v: int) -> int:
    return max(-32768, min(32767, v))


@dataclass
class DeviceChannel:
    device_id: int
    mode: str = MODE_TOPIC_CLIENT
    port: Optional[str] = None
    profile_key: str = DEFAULT_PROFILE_KEY
    tx_data: List[int] = field(default_factory=lambda: [0] * SLOT_COUNT)
    rx_data: List[int] = field(default_factory=lambda: [0] * SLOT_COUNT)
    topic_passthrough: bool = True
    direct_tx: bool = False
    manual: bool = False
    last_rx_time: Optional[float] = None
    rx_frame_count: int = 0
    tx_frame_count: int = 0
    # MODE_SIMULATOR限定: GUIから手動で値を設定したRXスロットのindex集合。
    # ここに含まれるスロットは service_simulators() の TX->RX ループバック対象から外れ、
    # 手動設定値をそのまま保持し続ける(ENC/SW等、実機では独立したセンサ入力の再現用)。
    sim_rx_override: set = field(default_factory=set)
    _rx_times: List[float] = field(default_factory=list)
    publisher = None
    subscription = None
    # serial_rx_[ID]_unwrapped (Int32MultiArray) 用。MODE_HARDWARE/MODE_SIMULATOR
    # (=自分がserial_rx_[ID]の発行元になる場合)のみ使う。MODE_TOPIC_CLIENTは
    # 他プロセスが既に発行元のため、二重発行を避けるためunwrapは行わない。
    unwrapped_publisher = None
    unwrappers: List[CounterUnwrapper] = field(
        default_factory=lambda: [CounterUnwrapper(_COUNTS_PER_WRAP) for _ in range(SLOT_COUNT)])
    # _publish_unwrapped() が更新する連続値のキャッシュ(MODE_HARDWARE/MODE_SIMULATORのみ)。
    # エンコーダ原点セット(zero_channel)がオフセットの基準として参照する。
    rx_unwrapped: List[int] = field(default_factory=lambda: [0] * SLOT_COUNT)
    # エンコーダ原点セット機能: マイコン側の値は変更せず、GUI側だけが保持するオフセット。
    # 「ゼロ点からの相対値」= 現在値(rx_unwrapped、topic_clientはrx_data) - zero_offset。
    # serial_rx_[ID]_zeroed (Int32MultiArray) として配信する(zero_channel/zero_all_channels参照)。
    zero_offset: List[int] = field(default_factory=lambda: [0] * SLOT_COUNT)
    # serial_rx_[ID]_zeroed (Int32MultiArray) 用。unwrapped_publisherと同じ条件でのみ作る。
    zeroed_publisher = None

    @property
    def connected(self) -> bool:
        if self.last_rx_time is None:
            return False
        return (time.monotonic() - self.last_rx_time) < STALE_TIMEOUT_SEC

    @property
    def rx_hz(self) -> float:
        now = time.monotonic()
        self._rx_times = [t for t in self._rx_times if now - t < 2.0]
        if len(self._rx_times) < 2:
            return 0.0
        span = self._rx_times[-1] - self._rx_times[0]
        if span <= 0:
            return 0.0
        return (len(self._rx_times) - 1) / span

    def note_rx(self) -> None:
        now = time.monotonic()
        self.last_rx_time = now
        self.rx_frame_count += 1
        self._rx_times.append(now)


class RosBackend(QObject):
    deviceListChanged = pyqtSignal()
    rxUpdated = pyqtSignal(int)

    def __init__(self, node_name: str = "ros2can_gui",
                 hardware_config: Optional[HardwareConfig] = None):
        super().__init__()
        self.node: Node = rclpy.create_node(node_name)
        self.devices: Dict[int, DeviceChannel] = {}
        self.device_profile_map: Dict[int, str] = self._load_device_profile_map_from_params()

        if hardware_config is None:
            hardware_config = self._load_hardware_config_from_params()

        self.hardware = HardwareManager(hardware_config)
        self.hardware.deviceClaimed.connect(self._on_hardware_claimed)
        self.hardware.frameReceived.connect(self._on_hardware_frame)
        self.hardware.linkStateChanged.connect(self._on_hardware_link_state)

        self._create_zero_channel_service()

    def _load_device_profile_map_from_params(self) -> Dict[int, str]:
        """device_id とプロファイルキーの対応表 (settings_store.py の
        device_profile_map、config/ros2can.yaml や ~/.config/ros2can/settings.yaml、
        あるいは launch から渡されるROS 2パラメータで上書き可能) を読み込む。
        新規デバイス検出時の初期プロファイル選択 (_initial_profile_key) に使う。"""
        defaults = load_settings()
        self.node.declare_parameter(
            "device_profile_map", [str(v) for v in defaults["device_profile_map"]])
        raw = self.node.get_parameter("device_profile_map").value or []
        return parse_device_profile_map(raw)

    def _initial_profile_key(self, device_id: int) -> str:
        return self.device_profile_map.get(device_id, DEFAULT_PROFILE_KEY)

    def _load_hardware_config_from_params(self) -> HardwareConfig:
        """serial_bridge.yaml と同名のパラメータでハードウェア直結の挙動を設定する。

        declare_parameter の default には settings_store.load_settings() (=
        config/ros2can.yaml に GUI の「設定」ダイアログでのローカル上書きを
        重ねたもの) を渡す。--ros-args -p や launch の parameters=[...] で
        明示的に値が渡された場合は、通常のROSパラメータの優先順位どおり
        そちらが勝つ。"""
        defaults = load_settings()
        self.node.declare_parameter("excluded_ports", list(defaults["excluded_ports"]))
        self.node.declare_parameter("rx_timeout_sec", float(defaults["rx_timeout_sec"]))
        self.node.declare_parameter(
            "reconnect_interval_sec", float(defaults["reconnect_interval_sec"]))
        self.node.declare_parameter("scan_interval_ms", int(defaults["scan_interval_ms"]))
        self.node.declare_parameter("probe_timeout_sec", float(defaults["probe_timeout_sec"]))
        self.node.declare_parameter("probe_settle_sec", float(defaults["probe_settle_sec"]))

        excluded = set(self.node.get_parameter("excluded_ports").value or [])
        return HardwareConfig(
            excluded_ports=excluded,
            scan_interval_sec=self.node.get_parameter("scan_interval_ms").value / 1000.0,
            probe_timeout_sec=self.node.get_parameter("probe_timeout_sec").value,
            probe_settle_sec=self.node.get_parameter("probe_settle_sec").value,
            rx_timeout_sec=self.node.get_parameter("rx_timeout_sec").value,
            reconnect_interval_sec=self.node.get_parameter("reconnect_interval_sec").value,
        )

    def start_hardware_scanning(self) -> None:
        self.hardware.start()

    def service_hardware(self) -> None:
        self.hardware.service()

    # ---------------- topic discovery (topic_client) ----------------

    def rescan_topics(self) -> None:
        found_ids = set()
        try:
            topics = self.node.get_topic_names_and_types()
        except Exception:
            return

        for name, _types in topics:
            m = TOPIC_RE.match(name)
            if m:
                found_ids.add(int(m.group(2)))

        changed = False
        for device_id in sorted(found_ids):
            if device_id not in self.devices:
                self.add_device(device_id, manual=False)
                changed = True

        if changed:
            self.deviceListChanged.emit()

    # ---------------- device management: topic_client ----------------

    def add_device(self, device_id: int, manual: bool = True) -> DeviceChannel:
        """既存の serial_tx_[ID]/serial_rx_[ID] トピックに相乗りするクライアントとして追加する。"""
        if device_id in self.devices:
            return self.devices[device_id]

        ch = DeviceChannel(
            device_id=device_id, mode=MODE_TOPIC_CLIENT, manual=manual,
            profile_key=self._initial_profile_key(device_id))
        ch.publisher = self.node.create_publisher(
            Int16MultiArray, f"serial_tx_{device_id}", 10)
        ch.subscription = self.node.create_subscription(
            Int16MultiArray, f"serial_rx_{device_id}",
            lambda msg, did=device_id: self._on_topic_rx(did, msg), 10)
        self.devices[device_id] = ch
        self.deviceListChanged.emit()
        return ch

    def _on_topic_rx(self, device_id: int, msg: Int16MultiArray) -> None:
        ch = self.devices.get(device_id)
        if ch is None or ch.mode != MODE_TOPIC_CLIENT:
            return
        self._apply_rx_data(ch, list(msg.data))
        self.rxUpdated.emit(device_id)

    # ---------------- device management: hardware ----------------

    def _attach_hardware_pubsub(self, ch: DeviceChannel, device_id: int) -> None:
        """自分自身が bridge_node の役割を担うため、他ノードとの互換のため役割は
        topic_client と逆になる: serial_rx を Publish、serial_tx を Subscribe する。"""
        ch.publisher = self.node.create_publisher(
            Int16MultiArray, f"serial_rx_{device_id}", 10)
        ch.unwrapped_publisher = self.node.create_publisher(
            Int32MultiArray, f"serial_rx_{device_id}_unwrapped", 10)
        ch.zeroed_publisher = self.node.create_publisher(
            Int32MultiArray, f"serial_rx_{device_id}_zeroed", 10)
        ch.subscription = self.node.create_subscription(
            Int16MultiArray, f"serial_tx_{device_id}",
            lambda msg, did=device_id: self._on_hardware_tx_command(did, msg), 10)

    def _on_hardware_claimed(self, device_id: int, port: str) -> None:
        ch = self.devices.get(device_id)
        if ch is not None:
            ch.port = port
            if ch.mode != MODE_HARDWARE:
                # 既存デバイス(topic_client 等)がハードウェア直結に切り替わった場合、
                # 役割が逆転する(serial_rx をPublish/serial_txをSubscribeに変わる)ため、
                # 古いモード向けの publisher/subscription を破棄してから作り直す。
                # (これをせず mode だけ書き換えると、RXフレームが古い publisher=
                # serial_tx_[ID] にそのまま publish されてしまう)
                if ch.publisher is not None:
                    self.node.destroy_publisher(ch.publisher)
                if ch.subscription is not None:
                    self.node.destroy_subscription(ch.subscription)
                ch.mode = MODE_HARDWARE
                self._attach_hardware_pubsub(ch, device_id)
                self.deviceListChanged.emit()
            return

        ch = DeviceChannel(
            device_id=device_id, mode=MODE_HARDWARE, port=port, manual=False,
            profile_key=self._initial_profile_key(device_id))
        self._attach_hardware_pubsub(ch, device_id)
        self.devices[device_id] = ch
        self.deviceListChanged.emit()

    def _on_hardware_tx_command(self, device_id: int, msg: Int16MultiArray) -> None:
        """外部ROSノードから serial_tx_[ID] へ送られてきた指令値を反映する。

        トピック通過(topic_passthrough)がOFFの場合、外部ノードからの指令は
        無視され tx_data にも反映されない。ONの場合は tx_data に反映した上で
        即座にマイコンへ書き込む(direct_tx は見ない)。

        ダイレクト送信(direct_tx)は、GUIで手動編集した tx_data を
        publish_all_direct() が周期送信するための別モードであり、GUI
        (device_panel.py)側では topic_passthrough と相互排他にしている
        (「外部ノードの指令を反映する」か「GUIから直接送信する」かの二択)。
        ただしこのフラグの組み合わせ自体に制約は無く、--nogui
        (serial_bridge互換ブリッジ) は direct_tx も常時ONにして、周期送信による
        heartbeat 送信を兼ねる(main.py参照)。
        """
        ch = self.devices.get(device_id)
        if ch is None or ch.mode != MODE_HARDWARE:
            return
        if not ch.topic_passthrough:
            return
        data = list(msg.data[:SLOT_COUNT])
        if len(data) < SLOT_COUNT:
            data += [0] * (SLOT_COUNT - len(data))
        ch.tx_data = data
        self.hardware.write(device_id, ch.tx_data)
        ch.tx_frame_count += 1

    def _on_hardware_frame(self, device_id: int, values: List[int]) -> None:
        ch = self.devices.get(device_id)
        if ch is None or ch.mode != MODE_HARDWARE:
            return
        self._apply_rx_data(ch, values)
        if ch.publisher is not None:
            msg = Int16MultiArray()
            msg.data = [int(v) for v in ch.rx_data]
            ch.publisher.publish(msg)
        # HardwareManager.service() (高頻度なQTimer駆動、サンプルが密な場所) で
        # フレームを受信した直後にunwrapする。詳細は counter_unwrapper.py 冒頭コメント参照。
        self._publish_unwrapped(ch)
        self.rxUpdated.emit(device_id)

    def _publish_unwrapped(self, ch: DeviceChannel) -> None:
        ch.rx_unwrapped = [u.update(v) for u, v in zip(ch.unwrappers, ch.rx_data)]
        if ch.unwrapped_publisher is not None:
            msg = Int32MultiArray()
            msg.data = list(ch.rx_unwrapped)
            ch.unwrapped_publisher.publish(msg)
        if ch.zeroed_publisher is not None:
            msg = Int32MultiArray()
            msg.data = [u - o for u, o in zip(ch.rx_unwrapped, ch.zero_offset)]
            ch.zeroed_publisher.publish(msg)

    def _on_hardware_link_state(self, device_id: int, _connected: bool) -> None:
        # デバイス一覧への反映(ポート情報等)のトリガーとして使う
        if device_id in self.devices:
            self.deviceListChanged.emit()

    def _apply_rx_data(self, ch: DeviceChannel, data: List[int]) -> None:
        if len(data) < SLOT_COUNT:
            data = list(data) + [0] * (SLOT_COUNT - len(data))
        ch.rx_data = data[:SLOT_COUNT]
        ch.note_rx()

    # ---------------- device management: simulator (debug mode, 実機不要) ----------------

    def add_simulated_device(self, device_id: int, profile_key: Optional[str] = None) -> DeviceChannel:
        """実機マイコン無しでUIの動作確認ができる仮想デバイスを追加する。

        TXに書いた値をそのままRXへループバックし続ける。
        ROSの役割は "hardware" と同じ (serial_rx_[ID] を Publish / serial_tx_[ID]
        を Subscribe) にしてあるため、他ノードから見ても実機接続時と同様に扱える。
        """
        if device_id in self.devices:
            return self.devices[device_id]

        ch = DeviceChannel(
            device_id=device_id, mode=MODE_SIMULATOR, manual=True,
            profile_key=profile_key or self._initial_profile_key(device_id))
        ch.publisher = self.node.create_publisher(
            Int16MultiArray, f"serial_rx_{device_id}", 10)
        ch.unwrapped_publisher = self.node.create_publisher(
            Int32MultiArray, f"serial_rx_{device_id}_unwrapped", 10)
        ch.zeroed_publisher = self.node.create_publisher(
            Int32MultiArray, f"serial_rx_{device_id}_zeroed", 10)
        ch.subscription = self.node.create_subscription(
            Int16MultiArray, f"serial_tx_{device_id}",
            lambda msg, did=device_id: self._on_simulator_tx_command(did, msg), 10)
        self.devices[device_id] = ch
        self.deviceListChanged.emit()
        return ch

    def _on_simulator_tx_command(self, device_id: int, msg: Int16MultiArray) -> None:
        """外部ROSノードから serial_tx_[ID] へ送られてきた指令値を反映する。

        仮想デバイスは実機を書き換えないため direct_tx は見ないが、
        トピック通過(topic_passthrough)がOFFの場合は外部ノードからの
        指令を無視する(GUIでの動作確認用途に使う場合など)。
        """
        ch = self.devices.get(device_id)
        if ch is None or ch.mode != MODE_SIMULATOR:
            return
        if not ch.topic_passthrough:
            return
        data = list(msg.data[:SLOT_COUNT])
        if len(data) < SLOT_COUNT:
            data += [0] * (SLOT_COUNT - len(data))
        ch.tx_data = data

    def service_simulators(self) -> None:
        """全シミュレータデバイスの TX->RX ループバックを1ステップ進める。

        トピック通過/ダイレクト送信の有無に関わらず常にRXを更新する: 実機は接続されていれば
        ホストの指令とは無関係にセンサ値を送り続けるため、それを模して Monitor/Raw
        タブの動作確認をいつでもできるようにしている。
        ただし ch.sim_rx_override に含まれるスロット(ENC/SW等、GUIから手動設定した
        センサ入力)はこのループバックの対象から外し、手動設定値をそのまま維持する。
        """
        for device_id, ch in self.devices.items():
            if ch.mode != MODE_SIMULATOR:
                continue
            values = list(ch.rx_data)
            for i, v in enumerate(ch.tx_data):
                if i in ch.sim_rx_override:
                    continue
                values[i] = v
            self._apply_rx_data(ch, values)
            if ch.publisher is not None:
                msg = Int16MultiArray()
                msg.data = [int(v) for v in ch.rx_data]
                ch.publisher.publish(msg)
            self._publish_unwrapped(ch)
            self.rxUpdated.emit(device_id)

    def set_sim_rx_value(self, device_id: int, index: int, raw: int) -> None:
        """デバッグ(仮想)デバイスのRXスロットをGUIから直接設定する。

        ENC/SW等、実機では独立したセンサ入力に相当し TX 指令とは無関係な値を
        GUIから再現するための入口。設定したスロットは ch.sim_rx_override に登録され、
        以後 service_simulators() のTX->RXループバック対象から外れて、次に変更される
        までこの値を保持し続ける。
        """
        ch = self.devices.get(device_id)
        if ch is None or ch.mode != MODE_SIMULATOR:
            return
        ch.sim_rx_override.add(index)
        data = list(ch.rx_data)
        data[index] = _clamp_int16(raw)
        self._apply_rx_data(ch, data)
        if ch.publisher is not None:
            msg = Int16MultiArray()
            msg.data = [int(v) for v in ch.rx_data]
            ch.publisher.publish(msg)
        self._publish_unwrapped(ch)
        self.rxUpdated.emit(device_id)

    # ---------------- エンコーダ原点セット (zero_offset) ----------------
    #
    # ロボマス内蔵/CubeMars内蔵/汎用AMTエンコーダのいずれも、自動化中の脱調や
    # 再起動をまたいだ蓄積誤差でマイコン側の値と機構上の原点がズレることがある。
    # マイコン側の値を直接書き換える手段は無い(ロボマスの多回転カウンタや
    # CubeMars内蔵エンコーダの生値をリセットするコマンドは実装していない)ため、
    # ros2can側がソフトウェアオフセットを持ち、「現在値 - オフセット」を
    # serial_rx_[ID]_zeroed (Int32MultiArray, 24スロット) として別配信する方式にした。
    # GUI(Monitorタブの「原点セット」ボタン)、ROS 2サービス(zero_channel、外部
    # ノードから利用可)のどちらからも同じ zero_channel()/zero_all_channels() を呼ぶ。
    #
    # MODE_HARDWARE/MODE_SIMULATORはPCNTラップアラウンドを解決した rx_unwrapped
    # を基準にする(多回転する関節でズレなく原点セットできる)。MODE_TOPIC_CLIENTは
    # ラップアラウンド解決を行っていない(counter_unwrapper.py先頭コメント参照:
    # ROSトピック経由はサンプルが疎になり得るため誤unwrapのリスクがある)ため、
    # 素のrx_dataを基準にする(1回のラップ幅=32768countを跨ぐ原点セットはズレ得る)。

    def _current_value_for_zero(self, ch: DeviceChannel, index: int) -> int:
        if ch.mode in (MODE_HARDWARE, MODE_SIMULATOR):
            return ch.rx_unwrapped[index]
        return ch.rx_data[index]

    def zero_channel(self, device_id: int, index: int) -> bool:
        """指定チャンネルについて、現在値をゼロ点として記録する(マイコン側は変更しない)。"""
        ch = self.devices.get(device_id)
        if ch is None or not (0 <= index < SLOT_COUNT):
            return False
        ch.zero_offset[index] = self._current_value_for_zero(ch, index)
        return True

    def zero_all_channels(self, device_id: int) -> bool:
        """デバイスの全24チャンネルについて、現在値をまとめてゼロ点として記録する。"""
        ch = self.devices.get(device_id)
        if ch is None:
            return False
        for index in range(SLOT_COUNT):
            ch.zero_offset[index] = self._current_value_for_zero(ch, index)
        return True

    def zeroed_value(self, device_id: int, index: int) -> Optional[int]:
        """ゼロ点からの相対値(GUI表示用)。デバイス/チャンネルが無ければNone。"""
        ch = self.devices.get(device_id)
        if ch is None or not (0 <= index < SLOT_COUNT):
            return None
        return self._current_value_for_zero(ch, index) - ch.zero_offset[index]

    def _create_zero_channel_service(self) -> None:
        self.node.create_service(ZeroChannel, "zero_channel", self._on_zero_channel_request)

    def _on_zero_channel_request(self, request, response):
        device_id = request.device_id
        index = request.channel_index
        if index < 0:
            ok = self.zero_all_channels(device_id)
            target = "全チャンネル"
        else:
            ok = self.zero_channel(device_id, index)
            target = f"チャンネル{index}"
        response.success = ok
        response.message = (
            f"device_id={device_id} の{target}を原点セットしました" if ok
            else f"device_id={device_id} が見つからないか、channel_indexが不正です"
        )
        return response

    # ---------------- device management: common ----------------

    def remove_device(self, device_id: int) -> None:
        ch = self.devices.pop(device_id, None)
        if ch is None:
            return
        if ch.publisher is not None:
            self.node.destroy_publisher(ch.publisher)
        if ch.unwrapped_publisher is not None:
            self.node.destroy_publisher(ch.unwrapped_publisher)
        if ch.zeroed_publisher is not None:
            self.node.destroy_publisher(ch.zeroed_publisher)
        if ch.subscription is not None:
            self.node.destroy_subscription(ch.subscription)
        if ch.mode == MODE_HARDWARE:
            self.hardware.release(device_id)
        self.deviceListChanged.emit()

    # ---------------- tx ----------------

    def publish_tx(self, device_id: int) -> None:
        """現在の tx_data を実際に送信する(モードに応じて経路を切替)。呼び出し側でゲートすること。"""
        ch = self.devices.get(device_id)
        if ch is None:
            return
        if ch.mode == MODE_HARDWARE:
            self.hardware.write(device_id, ch.tx_data)
            ch.tx_frame_count += 1
        elif ch.mode == MODE_SIMULATOR:
            # 実際のRX生成は service_simulators() が常時行うため、ここではカウントのみ。
            # (ch.publisher は serial_rx_[ID] 用なので tx_data を流してはいけない)
            ch.tx_frame_count += 1
        else:
            if ch.publisher is None:
                return
            msg = Int16MultiArray()
            msg.data = [int(v) for v in ch.tx_data]
            ch.publisher.publish(msg)
            ch.tx_frame_count += 1

    def publish_all_direct(self) -> None:
        """ダイレクト送信(direct_tx)が有効なデバイスへ現在の tx_data を周期送信する。

        GUIから直接操作するモード用の経路(トピック経由の指令は
        _on_hardware_tx_command が即時に書き込むため、ここでは扱わない)。
        """
        for device_id, ch in self.devices.items():
            if ch.direct_tx:
                self.publish_tx(device_id)

    def zero_and_send(self, device_id: int) -> None:
        ch = self.devices.get(device_id)
        if ch is None:
            return
        ch.tx_data = [0] * SLOT_COUNT
        self.publish_tx(device_id)

    def emergency_stop_all(self) -> None:
        """全デバイスへゼロ指令を送信し、以後の自動送信経路を両方とも止める。

        direct_tx だけでなく topic_passthrough もOFFにする: OFFにしないと
        外部ノードが指令を送り続けている場合に _on_hardware_tx_command が
        即座に非ゼロ値を書き込み直してしまい、E-STOPの意味がなくなる。
        """
        for device_id, ch in self.devices.items():
            ch.direct_tx = False
            ch.topic_passthrough = False
            ch.tx_data = [0] * SLOT_COUNT
            self.publish_tx(device_id)

    # ---------------- lifecycle ----------------

    def spin_once(self) -> None:
        rclpy.spin_once(self.node, timeout_sec=0)

    def shutdown(self) -> None:
        self.hardware.stop()
        for device_id in list(self.devices.keys()):
            self.remove_device(device_id)
        self.node.destroy_node()
