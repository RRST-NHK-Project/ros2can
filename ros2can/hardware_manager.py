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
    logMessage = pyqtSignal(str)              # 接続/切断/フレーム異常などの通知ログ

    def __init__(self, config: Optional[HardwareConfig] = None, parent=None):
        super().__init__(parent)
        self.config = config or HardwareConfig()
        self.links: Dict[int, SerialLink] = {}
        self._last_reconnect_attempt: Dict[int, float] = {}
        self._last_rx_time: Dict[int, float] = {}
        # 新規ファームウェア開発時の通信テストでフレーム同期ずれに気付けるよう、
        # FrameParser(frame_codec.py)が数えている checksum_errors/dropped_bytes の
        # 増分をログ化する際の基準値。SerialLink.open()のたびにパーサが作り直され
        # カウントは0に戻るため、再接続時にリセットする。
        self._last_checksum_errors: Dict[int, int] = {}
        self._last_dropped_bytes: Dict[int, int] = {}
        # firmware_dialog.py がpio書き込み中に一時的にスキャン/再接続の対象から
        # 外すポート。書き込み用サブプロセスとros2can自身のオープンが競合しないため。
        self._flash_locked_ports: Set[str] = set()

        self._scanner = _ScannerThread(self.config)
        self._scanner.set_skip_ports_provider(self._owned_ports)
        self._scanner.deviceDetected.connect(self._on_device_detected)

    # ---------------- lifecycle ----------------

    def start(self) -> None:
        self._scanner.start()

    def stop(self) -> None:
        self._scanner.stop()
        self._scanner.wait(2000)
        for link in self.links.values():
            link.close()

    def _owned_ports(self) -> Set[str]:
        return {link.port for link in self.links.values() if link.is_open} \
            | self._flash_locked_ports

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
        self._last_checksum_errors[device_id] = 0
        self._last_dropped_bytes[device_id] = 0
        self.deviceClaimed.emit(device_id, port)
        self.linkStateChanged.emit(device_id, True)
        self.logMessage.emit(f"[HW] id={device_id} port={port} を検出し接続しました")

    # ---------------- periodic IO ----------------

    def service(self) -> None:
        """高頻度に呼び出し、全リンクの受信処理・切断検知・再接続を行う。"""
        now = time.monotonic()
        for device_id, link in list(self.links.items()):
            if not link.is_open:
                self._maybe_reconnect(device_id, link, now)
                continue

            try:
                frames = link.read_frames()
            except SerialLinkError as exc:
                self.logMessage.emit(
                    f"[HW] id={device_id} port={link.port} でI/Oエラー: {exc}")
                self._handle_disconnect(device_id, link)
                continue

            for frame_id, values in frames:
                if frame_id != device_id:
                    continue  # ID不一致フレームは破棄
                self._last_rx_time[device_id] = now
                self.frameReceived.emit(device_id, values)

            self._check_parser_errors(device_id, link)

            if now - self._last_rx_time.get(device_id, now) >= self.config.rx_timeout_sec:
                self.logMessage.emit(
                    f"[HW] id={device_id} port={link.port} からのRXが"
                    f"{self.config.rx_timeout_sec:.1f}秒途絶えたため切断扱いにします")
                self._handle_disconnect(device_id, link)

    def _check_parser_errors(self, device_id: int, link: SerialLink) -> None:
        """フレーム同期ずれ(チェックサム不一致/同期外れバイト破棄)が増えていたら
        通知する。新規ファームウェア開発時の通信テストで、フレーミング仕様の
        バグ(長さ計算ミス、チェックサム計算ミス等)に気付く手掛かりになる。"""
        checksum_errors = link.parser.checksum_errors
        last_checksum = self._last_checksum_errors.get(device_id, 0)
        checksum_delta = checksum_errors - last_checksum
        if checksum_delta > 0:
            self._last_checksum_errors[device_id] = checksum_errors
            self.logMessage.emit(
                f"[HW] id={device_id} port={link.port} チェックサム不一致 x{checksum_delta} "
                f"(フレーム同期ずれの可能性、累計{checksum_errors})")

        # dropped_bytes はチェックサム不一致でも1byteずつ加算されるため、それ以外の
        # 原因(START_BYTE不一致/LENGTH異常による再同期)で増えた分だけを別掲する。
        dropped_bytes = link.parser.dropped_bytes
        last_dropped = self._last_dropped_bytes.get(device_id, 0)
        dropped_delta = dropped_bytes - last_dropped
        if dropped_delta > 0:
            self._last_dropped_bytes[device_id] = dropped_bytes
            extra = dropped_delta - checksum_delta
            if extra > 0:
                self.logMessage.emit(
                    f"[HW] id={device_id} port={link.port} 不正な同期バイトを{extra}byte破棄 "
                    f"(START_BYTE不一致 or LENGTH異常、累計{dropped_bytes})")

    def _handle_disconnect(self, device_id: int, link: SerialLink) -> None:
        link.close()
        self._last_reconnect_attempt[device_id] = time.monotonic()
        self.linkStateChanged.emit(device_id, False)
        self.logMessage.emit(f"[HW] id={device_id} port={link.port} から切断されました")

    def _maybe_reconnect(self, device_id: int, link: SerialLink, now: float) -> None:
        if link.port in self._flash_locked_ports:
            return
        last_attempt = self._last_reconnect_attempt.get(device_id, 0.0)
        if now - last_attempt < self.config.reconnect_interval_sec:
            return
        self._last_reconnect_attempt[device_id] = now
        try:
            link.open()
            self._last_rx_time[device_id] = now
            self._last_checksum_errors[device_id] = 0
            self._last_dropped_bytes[device_id] = 0
            self.linkStateChanged.emit(device_id, True)
            self.logMessage.emit(f"[HW] id={device_id} port={link.port} へ再接続しました")
        except Exception:
            pass

    def release(self, device_id: int) -> None:
        """デバイスの管理自体を終了する(GUI側で明示的に削除された場合)。"""
        link = self.links.pop(device_id, None)
        if link is not None:
            link.close()
        self._last_reconnect_attempt.pop(device_id, None)
        self._last_rx_time.pop(device_id, None)
        self._last_checksum_errors.pop(device_id, None)
        self._last_dropped_bytes.pop(device_id, None)

    def lock_port_for_flash(self, port: str) -> None:
        """firmware_dialog.py がpio書き込みを始める前に呼ぶ。

        既存の接続があれば閉じ、スキャナ(_owned_ports経由)・再接続ロジック
        (_maybe_reconnect)の両方から一時的に除外する。
        """
        self._flash_locked_ports.add(port)
        for device_id, link in list(self.links.items()):
            if link.port == port and link.is_open:
                link.close()
                self.linkStateChanged.emit(device_id, False)
                self.logMessage.emit(f"[HW] port={port} を書き込みのため一時的に解放しました")

    def unlock_port_for_flash(self, port: str) -> None:
        """書き込み完了(成功/失敗/中断いずれも)後に必ず呼ぶ。以後は通常通り

        スキャン/再接続の対象に戻る。
        """
        self._flash_locked_ports.discard(port)
        self.logMessage.emit(f"[HW] port={port} の書き込み用ロックを解除しました")

    def write(self, device_id: int, data: List[int]) -> None:
        link = self.links.get(device_id)
        if link is None or not link.is_open:
            return
        try:
            link.write_data(data)
        except SerialLinkError:
            self._handle_disconnect(device_id, link)
