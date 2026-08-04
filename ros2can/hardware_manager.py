"""ハードウェア(シリアルポート)を直接専有して serial_bridge プロトコルを
喋るための管理クラス群。ポートスキャンは重い(1ポートあたり最大 数秒)ため
GUIスレッドをブロックしないよう別スレッド(QThread)で実行する。

[serial_bridge/src/main.cpp のスキャナスレッド構成を移植]
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Set

from PyQt5.QtCore import QObject, QThread, pyqtSignal

from .serial_link import SerialLink, SerialLinkError, list_serial_ports, probe_port


@dataclass
class HardwareConfig:
    excluded_ports: Set[str] = field(default_factory=set)
    scan_interval_sec: float = 5.0
    probe_timeout_sec: float = 2.0
    probe_settle_sec: float = 0.5
    rx_timeout_sec: float = 2.0
    reconnect_interval_sec: float = 3.0


class _LinkReaderThread(QThread):
    """1つの SerialLink を専有し、GUIスレッドの詰まりに関係なく継続的に読み続けるスレッド。

    以前は HardwareManager.service() (GUIスレッドのQTimerから高頻度に呼ばれる) が
    read_frames() を直接呼んでいたが、GUIスレッドが他の処理(rclpy.spin_once()や
    描画等)で詰まっている間は read_frames() 自体が呼ばれず、その間にOS側のシリアル
    受信バッファが溢れてフレームが本当に失われることが実機検証で確認された。
    受信だけは専用スレッドで常時ドレインし続けることで、GUIスレッドの詰まりに
    影響されないようにする。

    書き込み(write)は従来通りGUIスレッド側(HardwareManager.write())で行う。
    同一 SerialLink に対する読み取り専用スレッドと書き込み元(GUIスレッド)が
    並行動作する形になるが、pyserialはOSファイルディスクリプタへの読み取り/
    書き込みが独立した操作であるため、読み書きを別スレッドに分けるのは一般的な
    利用パターンであり問題ない。
    """

    frameReceived = pyqtSignal(int, list)  # device_id, values(24)
    linkError = pyqtSignal(int)            # device_id

    def __init__(self, link: SerialLink, parent=None):
        super().__init__(parent)
        self._link = link
        self._device_id = link.device_id
        self._running = True

    def stop(self) -> None:
        self._running = False

    def run(self) -> None:
        while self._running:
            try:
                frames = self._link.read_frames()
            except SerialLinkError:
                self.linkError.emit(self._device_id)
                return
            for frame_id, values in frames:
                if frame_id != self._device_id:
                    continue
                self.frameReceived.emit(self._device_id, values)
            # busy-loopにならない程度に短く待つ(ファームウェアの200Hz送信に対し
            # 十分高頻度。実機のIO_Task/serialTaskのvTaskDelay(1)と同程度)
            self.msleep(1)


class _ScannerThread(QThread):
    """未専有のポートを定期的にプローブし続けるバックグラウンドスレッド。"""

    deviceDetected = pyqtSignal(int, str)  # device_id, port

    def __init__(self, config: HardwareConfig, parent=None):
        super().__init__(parent)
        self.config = config
        self._running = True
        self._skip_ports_provider: Callable[[], Set[str]] = lambda: set()

    def set_skip_ports_provider(self, fn: Callable[[], Set[str]]) -> None:
        self._skip_ports_provider = fn

    def stop(self) -> None:
        self._running = False

    def run(self) -> None:
        while self._running:
            skip = self._skip_ports_provider() | set(self.config.excluded_ports)
            for port in list_serial_ports():
                if not self._running:
                    return
                if port in skip:
                    continue
                device_id = probe_port(
                    port,
                    timeout_sec=self.config.probe_timeout_sec,
                    settle_sec=self.config.probe_settle_sec)
                if device_id is not None:
                    self.deviceDetected.emit(device_id, port)

            waited = 0.0
            while waited < self.config.scan_interval_sec and self._running:
                time.sleep(0.1)
                waited += 0.1


class HardwareManager(QObject):
    """専有中デバイスの一覧管理と、定期的な送受信サービスを行う。

    [serial_bridge::SerialBridgeNode::update()/tx_callback() の役割を統合して移植]
    """

    frameReceived = pyqtSignal(int, list)     # device_id, values(24)
    linkStateChanged = pyqtSignal(int, bool)  # device_id, connected
    deviceClaimed = pyqtSignal(int, str)      # device_id, port (新規専有時)

    def __init__(self, config: Optional[HardwareConfig] = None, parent=None):
        super().__init__(parent)
        self.config = config or HardwareConfig()
        self.links: Dict[int, SerialLink] = {}
        self._readers: Dict[int, _LinkReaderThread] = {}
        self._last_reconnect_attempt: Dict[int, float] = {}
        self._last_rx_time: Dict[int, float] = {}

        self._scanner = _ScannerThread(self.config)
        self._scanner.set_skip_ports_provider(self._owned_ports)
        self._scanner.deviceDetected.connect(self._on_device_detected)

    # ---------------- lifecycle ----------------

    def start(self) -> None:
        self._scanner.start()

    def stop(self) -> None:
        self._scanner.stop()
        self._scanner.wait(2000)
        for reader in self._readers.values():
            reader.stop()
        for reader in self._readers.values():
            reader.wait(1000)
        self._readers.clear()
        for link in self.links.values():
            link.close()

    def _start_reader(self, device_id: int, link: SerialLink) -> None:
        reader = _LinkReaderThread(link)
        reader.frameReceived.connect(self._on_reader_frame)
        reader.linkError.connect(self._on_reader_error)
        reader.start()
        self._readers[device_id] = reader

    def _stop_reader(self, device_id: int) -> None:
        reader = self._readers.pop(device_id, None)
        if reader is not None:
            reader.stop()
            reader.wait(1000)

    def _on_reader_frame(self, device_id: int, values: List[int]) -> None:
        # 専用スレッド(_LinkReaderThread)からキューイング配信されるスロット。
        # ここはGUI/メインスレッド上で実行される。
        self._last_rx_time[device_id] = time.monotonic()
        self.frameReceived.emit(device_id, values)

    def _on_reader_error(self, device_id: int) -> None:
        link = self.links.get(device_id)
        if link is not None:
            self._handle_disconnect(device_id, link)

    def _owned_ports(self) -> Set[str]:
        return {link.port for link in self.links.values() if link.is_open}

    # ---------------- detection ----------------

    def _on_device_detected(self, device_id: int, port: str) -> None:
        existing = self.links.get(device_id)

        if existing is not None and existing.is_open:
            return  # 既に接続中: 何もしない

        if existing is not None and not existing.is_open and existing.port == port:
            # 同じポートでの再検出: 自前の再接続ロジック(_maybe_reconnect)に任せる
            return

        link = SerialLink(port, device_id)
        try:
            link.open()
        except Exception:
            return

        if existing is not None:
            existing.close()

        self.links[device_id] = link
        self._last_rx_time[device_id] = time.monotonic()
        self._start_reader(device_id, link)
        self.deviceClaimed.emit(device_id, port)
        self.linkStateChanged.emit(device_id, True)

    # ---------------- periodic IO ----------------

    def service(self) -> None:
        """高頻度に呼び出し、切断検知・再接続を行う。

        実際のフレーム受信は _LinkReaderThread が専用スレッドで常時ドレインして
        いるため、ここでは行わない(GUIスレッドの詰まりでOS側シリアルバッファが
        溢れてフレームが失われるのを防ぐため、受信だけは切り離してある)。
        """
        now = time.monotonic()
        for device_id, link in list(self.links.items()):
            if not link.is_open:
                self._maybe_reconnect(device_id, link, now)
                continue

            if now - self._last_rx_time.get(device_id, now) >= self.config.rx_timeout_sec:
                self._handle_disconnect(device_id, link)

    def _handle_disconnect(self, device_id: int, link: SerialLink) -> None:
        self._stop_reader(device_id)
        link.close()
        self._last_reconnect_attempt[device_id] = time.monotonic()
        self.linkStateChanged.emit(device_id, False)

    def _maybe_reconnect(self, device_id: int, link: SerialLink, now: float) -> None:
        last_attempt = self._last_reconnect_attempt.get(device_id, 0.0)
        if now - last_attempt < self.config.reconnect_interval_sec:
            return
        self._last_reconnect_attempt[device_id] = now
        try:
            link.open()
            self._last_rx_time[device_id] = now
            self._start_reader(device_id, link)
            self.linkStateChanged.emit(device_id, True)
        except Exception:
            pass

    def release(self, device_id: int) -> None:
        """デバイスの管理自体を終了する(GUI側で明示的に削除された場合)。"""
        self._stop_reader(device_id)
        link = self.links.pop(device_id, None)
        if link is not None:
            link.close()
        self._last_reconnect_attempt.pop(device_id, None)
        self._last_rx_time.pop(device_id, None)

    def write(self, device_id: int, data: List[int]) -> None:
        link = self.links.get(device_id)
        if link is None or not link.is_open:
            return
        try:
            link.write_data(data)
        except SerialLinkError:
            self._handle_disconnect(device_id, link)
