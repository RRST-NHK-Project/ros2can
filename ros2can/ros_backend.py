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
  TX に書き込んだ値をその場で RX にループバック(+多少の揺らぎ)して返す仮想デバイス。
  ROSトピックの Publish/Subscribe の役割は "hardware" と同じ(自分が bridge_node の
  代わりを担う)にしてあるため、他のROSノードや rqt からも実機と区別なく確認できる。
"""

from __future__ import annotations

import math
import re
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

from PyQt5.QtCore import QObject, pyqtSignal

from .device_profiles import DEFAULT_PROFILE_KEY, SLOT_COUNT
from .hardware_manager import HardwareConfig, HardwareManager

TOPIC_RE = re.compile(r"^/?(serial_tx|serial_rx)_(\d+)$")

# この時間 [s] 以上 RX が無ければ「オフライン」とみなす (表示用のヒューリスティック)
STALE_TIMEOUT_SEC = 1.5

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
    armed: bool = False
    manual: bool = False
    last_rx_time: Optional[float] = None
    rx_frame_count: int = 0
    tx_frame_count: int = 0
    _rx_times: List[float] = field(default_factory=list)
    publisher = None
    subscription = None

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

        if hardware_config is None:
            hardware_config = self._load_hardware_config_from_params()

        self.hardware = HardwareManager(hardware_config)
        self.hardware.deviceClaimed.connect(self._on_hardware_claimed)
        self.hardware.frameReceived.connect(self._on_hardware_frame)
        self.hardware.linkStateChanged.connect(self._on_hardware_link_state)

    def _load_hardware_config_from_params(self) -> HardwareConfig:
        """serial_bridge.yaml と同名のパラメータでハードウェア直結の挙動を設定する。"""
        self.node.declare_parameter("excluded_ports", [])
        self.node.declare_parameter("rx_timeout_sec", 2.0)
        self.node.declare_parameter("reconnect_interval_sec", 3.0)
        self.node.declare_parameter("scan_interval_ms", 5000)
        self.node.declare_parameter("probe_timeout_sec", 2.0)
        self.node.declare_parameter("probe_settle_sec", 0.5)

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

        ch = DeviceChannel(device_id=device_id, mode=MODE_TOPIC_CLIENT, manual=manual)
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

    def _on_hardware_claimed(self, device_id: int, port: str) -> None:
        if device_id in self.devices:
            # 既存デバイス(topic_client 等)がハードウェア直結に切り替わった場合はポートだけ更新
            self.devices[device_id].port = port
            self.devices[device_id].mode = MODE_HARDWARE
            return

        ch = DeviceChannel(device_id=device_id, mode=MODE_HARDWARE, port=port, manual=False)
        # 自分自身が bridge_node の役割を担うため、他ノードとの互換のため役割は
        # topic_client と逆になる: serial_rx を Publish、serial_tx を Subscribe する。
        ch.publisher = self.node.create_publisher(
            Int16MultiArray, f"serial_rx_{device_id}", 10)
        ch.subscription = self.node.create_subscription(
            Int16MultiArray, f"serial_tx_{device_id}",
            lambda msg, did=device_id: self._on_hardware_tx_command(did, msg), 10)
        self.devices[device_id] = ch
        self.deviceListChanged.emit()

    def _on_hardware_tx_command(self, device_id: int, msg: Int16MultiArray) -> None:
        """外部ROSノードから serial_tx_[ID] へ送られてきた指令値を反映する。"""
        ch = self.devices.get(device_id)
        if ch is None or ch.mode != MODE_HARDWARE:
            return
        data = list(msg.data[:SLOT_COUNT])
        if len(data) < SLOT_COUNT:
            data += [0] * (SLOT_COUNT - len(data))
        ch.tx_data = data
        if ch.armed:
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
        self.rxUpdated.emit(device_id)

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

        TXに書いた値をそのまま(多少の揺らぎ付きで)RXへループバックし続ける。
        ROSの役割は "hardware" と同じ (serial_rx_[ID] を Publish / serial_tx_[ID]
        を Subscribe) にしてあるため、他ノードから見ても実機接続時と同様に扱える。
        """
        if device_id in self.devices:
            return self.devices[device_id]

        ch = DeviceChannel(device_id=device_id, mode=MODE_SIMULATOR, manual=True)
        if profile_key:
            ch.profile_key = profile_key
        ch.publisher = self.node.create_publisher(
            Int16MultiArray, f"serial_rx_{device_id}", 10)
        ch.subscription = self.node.create_subscription(
            Int16MultiArray, f"serial_tx_{device_id}",
            lambda msg, did=device_id: self._on_simulator_tx_command(did, msg), 10)
        self.devices[device_id] = ch
        self.deviceListChanged.emit()
        return ch

    def _on_simulator_tx_command(self, device_id: int, msg: Int16MultiArray) -> None:
        """外部ROSノードから serial_tx_[ID] へ送られてきた指令値を反映する。"""
        ch = self.devices.get(device_id)
        if ch is None or ch.mode != MODE_SIMULATOR:
            return
        data = list(msg.data[:SLOT_COUNT])
        if len(data) < SLOT_COUNT:
            data += [0] * (SLOT_COUNT - len(data))
        ch.tx_data = data

    def service_simulators(self) -> None:
        """全シミュレータデバイスの TX->RX ループバックを1ステップ進める。

        TX有効化(armed)の有無に関わらず常にRXを更新する: 実機は接続されていれば
        ホストの指令とは無関係にセンサ値を送り続けるため、それを模して Monitor/Raw
        タブの動作確認をいつでもできるようにしている。
        """
        now = time.monotonic()
        for device_id, ch in self.devices.items():
            if ch.mode != MODE_SIMULATOR:
                continue
            wobble_phase = now * 2.0
            values = [
                _clamp_int16(v + int(round(4 * math.sin(wobble_phase + i * 0.7))))
                for i, v in enumerate(ch.tx_data)
            ]
            self._apply_rx_data(ch, values)
            if ch.publisher is not None:
                msg = Int16MultiArray()
                msg.data = [int(v) for v in ch.rx_data]
                ch.publisher.publish(msg)
            self.rxUpdated.emit(device_id)

    # ---------------- device management: common ----------------

    def remove_device(self, device_id: int) -> None:
        ch = self.devices.pop(device_id, None)
        if ch is None:
            return
        if ch.publisher is not None:
            self.node.destroy_publisher(ch.publisher)
        if ch.subscription is not None:
            self.node.destroy_subscription(ch.subscription)
        if ch.mode == MODE_HARDWARE:
            self.hardware.release(device_id)
        self.deviceListChanged.emit()

    # ---------------- tx ----------------

    def publish_tx(self, device_id: int) -> None:
        """armed なデバイスへ現在の tx_data を実際に送信する(モードに応じて経路を切替)。"""
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

    def publish_all_armed(self) -> None:
        for device_id, ch in self.devices.items():
            if ch.armed:
                self.publish_tx(device_id)

    def zero_and_send(self, device_id: int) -> None:
        ch = self.devices.get(device_id)
        if ch is None:
            return
        ch.tx_data = [0] * SLOT_COUNT
        self.publish_tx(device_id)

    def emergency_stop_all(self) -> None:
        for device_id, ch in self.devices.items():
            ch.armed = False
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
