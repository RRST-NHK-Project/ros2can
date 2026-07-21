"""ros2can のメインウィンドウ。

左側に検出済み CAN ホスト (serial_bridge の DEVICE_ID) の一覧、
右側に選択したホストの DevicePanel (ノード選択タブ付き) を表示する。
"""

from __future__ import annotations

from typing import Dict, Optional

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QMainWindow, QListWidget, QListWidgetItem, QStackedWidget,
    QToolBar, QAction, QLabel, QInputDialog,
    QMessageBox, QSplitter, QMenu,
)

from .ros_backend import RosBackend
from .device_panel import DevicePanel
from .can_monitor import CanMonitorDialog

UI_REFRESH_MS = 200
TOPIC_RESCAN_MS = 1000


class MainWindow(QMainWindow):
    def __init__(self, backend: RosBackend, parent=None):
        super().__init__(parent)
        self.backend = backend
        self.panels: Dict[int, DevicePanel] = {}
        self._can_monitor_dialog: Optional[CanMonitorDialog] = None

        self.setWindowTitle("ros2can - XIAO ESP32S3 SMD CANバス デバッグGUI")
        self.resize(1280, 800)

        self._build_toolbar()

        splitter = QSplitter(Qt.Horizontal)

        self.device_list = QListWidget()
        self.device_list.setMinimumWidth(220)
        self.device_list.setMaximumWidth(320)
        self.device_list.currentItemChanged.connect(self._on_selection_changed)
        self.device_list.setContextMenuPolicy(Qt.CustomContextMenu)
        self.device_list.customContextMenuRequested.connect(self._on_device_list_context_menu)
        splitter.addWidget(self.device_list)

        self.stack = QStackedWidget()
        self.placeholder = QLabel(
            "CANホストがまだ検出されていません。\n\n"
            "・xiao_esp32_s3_smd_serial_bridge (MODE_CAN_HOST) を書き込んだ基板をUSB接続すると、\n"
            "  ros2can が自身でシリアルポートをスキャンして自動検出します(serial_bridgeは不要)。\n"
            "・serial_bridge (別プロセス) が既に握っているトピックに相乗りしたい場合は、\n"
            "  上部の「デバイスを手動追加」を使用してください。\n"
            "・実機が無くてもUIの動作確認をしたい場合は、上部の「デバッグデバイスを追加」\n"
            "  から仮想デバイスを追加してください(TXの値がそのままRXにループバックされます)。")
        self.placeholder.setAlignment(Qt.AlignCenter)
        self.placeholder.setStyleSheet("color: #888; font-size: 11pt;")
        self.stack.addWidget(self.placeholder)
        splitter.addWidget(self.stack)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)

        self.setCentralWidget(splitter)

        self.statusBar().showMessage("起動しました。トピックをスキャンしています…")

        self.backend.deviceListChanged.connect(self._refresh_device_list)
        self.backend.rxUpdated.connect(self._on_rx_updated)
        # backend にコンストラクタ時点で既に登録済みのデバイスがあれば取りこぼさないよう反映する
        self._refresh_device_list()

        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self._periodic_ui_refresh)
        self.ui_timer.start(UI_REFRESH_MS)

        self.rescan_timer = QTimer(self)
        self.rescan_timer.timeout.connect(self.backend.rescan_topics)
        self.rescan_timer.start(TOPIC_RESCAN_MS)
        self.backend.rescan_topics()

    # ---------------- toolbar ----------------

    def _build_toolbar(self) -> None:
        toolbar = QToolBar("main")
        toolbar.setMovable(False)
        self.addToolBar(toolbar)

        rescan_action = QAction("今すぐ再スキャン", self)
        rescan_action.triggered.connect(self.backend.rescan_topics)
        toolbar.addAction(rescan_action)

        add_action = QAction("デバイスを手動追加…", self)
        add_action.triggered.connect(self._on_add_device_manually)
        toolbar.addAction(add_action)

        add_debug_action = QAction("デバッグデバイスを追加(実機不要)…", self)
        add_debug_action.setToolTip(
            "マイコン実機が無くてもUI確認ができる仮想デバイスを追加します。\n"
            "TXに書き込んだ値がそのままRXへループバックされます。")
        add_debug_action.triggered.connect(self._on_add_debug_device)
        toolbar.addAction(add_debug_action)

        can_monitor_action = QAction("CANモニター…", self)
        can_monitor_action.setToolTip(
            "MODE_CAN_MONITORで書き込んだ基板のシリアル出力(生CANフレーム/デコード済み要約)を"
            "直接閲覧します。serial_bridgeフレームは使わないため、ros2canのデバイス一覧には出ません。")
        can_monitor_action.triggered.connect(self._on_open_can_monitor)
        toolbar.addAction(can_monitor_action)

        toolbar.addSeparator()

        estop_action = QAction("■ 全デバイス E-STOP (全ゼロ送信+TX無効化)", self)
        estop_action.setToolTip("全ての接続中デバイスのTXを即座にゼロにして送信を停止します")
        estop_action.triggered.connect(self._on_global_estop)
        toolbar.addAction(estop_action)

    # ---------------- device list ----------------

    def _on_add_device_manually(self) -> None:
        device_id, ok = QInputDialog.getInt(
            self, "デバイスを手動追加", "DEVICE_ID (0-255):", 0, 0, 255, 1)
        if not ok:
            return
        self.backend.add_device(device_id, manual=True)
        self._refresh_device_list()
        self._select_device(device_id)

    def _on_add_debug_device(self) -> None:
        """実機不要のデバッグ(仮想)デバイスを追加する。UIの動作確認・調整用。"""
        device_id, ok = QInputDialog.getInt(
            self, "デバッグデバイスを追加", "DEVICE_ID (0-255):", 1, 0, 255, 1)
        if not ok:
            return
        if device_id in self.backend.devices:
            QMessageBox.warning(
                self, "デバッグデバイスを追加",
                f"DEVICE_ID {device_id} は既に使用されています。")
            return
        self.backend.add_simulated_device(device_id)
        self._refresh_device_list()
        self._select_device(device_id)

    def _on_open_can_monitor(self) -> None:
        """MODE_CAN_MONITOR機のシリアル出力を見るための独立ウィンドウを開く(モードレス)。"""
        if self._can_monitor_dialog is None:
            self._can_monitor_dialog = CanMonitorDialog(self)
        self._can_monitor_dialog.show()
        self._can_monitor_dialog.raise_()
        self._can_monitor_dialog.activateWindow()

    def _on_device_list_context_menu(self, pos) -> None:
        item = self.device_list.itemAt(pos)
        if item is None:
            return
        device_id = item.data(Qt.UserRole)
        menu = QMenu(self)
        remove_action = menu.addAction("このデバイスを削除")
        chosen = menu.exec_(self.device_list.mapToGlobal(pos))
        if chosen is remove_action:
            self.backend.remove_device(device_id)

    def _refresh_device_list(self) -> None:
        existing_ids = set(self.backend.devices.keys())
        listed_ids = set()
        for i in range(self.device_list.count()):
            item = self.device_list.item(i)
            listed_ids.add(item.data(Qt.UserRole))

        for device_id in existing_ids - listed_ids:
            item = QListWidgetItem()
            item.setData(Qt.UserRole, device_id)
            self.device_list.addItem(item)
            panel = DevicePanel(self.backend, device_id)
            self.panels[device_id] = panel
            self.stack.addWidget(panel)

        for device_id in listed_ids - existing_ids:
            self._remove_device_from_ui(device_id)

        self._update_list_labels()

        if self.device_list.currentItem() is None and self.device_list.count() > 0:
            self.device_list.setCurrentRow(0)

    def _remove_device_from_ui(self, device_id: int) -> None:
        panel = self.panels.pop(device_id, None)
        if panel is not None:
            self.stack.removeWidget(panel)
            panel.deleteLater()
        for i in range(self.device_list.count()):
            item = self.device_list.item(i)
            if item.data(Qt.UserRole) == device_id:
                self.device_list.takeItem(i)
                break

    def _update_list_labels(self) -> None:
        for i in range(self.device_list.count()):
            item = self.device_list.item(i)
            device_id = item.data(Qt.UserRole)
            ch = self.backend.devices.get(device_id)
            if ch is None:
                continue
            state = "🟢接続中" if ch.connected else "⚪未接続"
            direct = " [TX ON]" if ch.direct_tx else ""
            passthrough = "" if ch.topic_passthrough else " [PASS OFF]"
            if ch.mode == "hardware":
                mode_label = f"HW:{ch.port}"
            elif ch.mode == "simulator":
                mode_label = "🧪DEBUG(仮想)"
            else:
                mode_label = "topic"
            item.setText(f"ID {device_id}  {state}{direct}{passthrough}\n{mode_label}  {ch.profile_key}")

    def _on_selection_changed(self, current: Optional[QListWidgetItem], _previous) -> None:
        if current is None:
            self.stack.setCurrentWidget(self.placeholder)
            return
        device_id = current.data(Qt.UserRole)
        panel = self.panels.get(device_id)
        if panel is not None:
            self.stack.setCurrentWidget(panel)

    def _select_device(self, device_id: int) -> None:
        for i in range(self.device_list.count()):
            item = self.device_list.item(i)
            if item.data(Qt.UserRole) == device_id:
                self.device_list.setCurrentItem(item)
                break

    # ---------------- rx / refresh ----------------

    def _on_rx_updated(self, device_id: int) -> None:
        panel = self.panels.get(device_id)
        if panel is not None and self.stack.currentWidget() is panel:
            panel.refresh_from_rx()

    def _periodic_ui_refresh(self) -> None:
        self._update_list_labels()
        current = self.stack.currentWidget()
        if isinstance(current, DevicePanel):
            current.refresh_from_rx()
        connected = sum(1 for c in self.backend.devices.values() if c.connected)
        direct = sum(1 for c in self.backend.devices.values() if c.direct_tx)
        self.statusBar().showMessage(
            f"検出デバイス: {len(self.backend.devices)}  接続中: {connected}  ダイレクト送信: {direct}")

    # ---------------- safety ----------------

    def _on_global_estop(self) -> None:
        self.backend.emergency_stop_all()
        for panel in self.panels.values():
            panel.set_direct_tx_external(False)
        self._update_list_labels()
        QMessageBox.information(self, "E-STOP", "全デバイスへゼロ指令を送信し、ダイレクト送信を無効化しました。")

    def closeEvent(self, event) -> None:
        self.backend.emergency_stop_all()
        super().closeEvent(event)
