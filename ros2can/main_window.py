"""ros2can のメインウィンドウ。

左側に検出済み CAN ホスト (serial_bridge の DEVICE_ID) の一覧、
右側に選択したホストの DevicePanel (ノード選択タブ付き) を表示する。
"""

from __future__ import annotations

import time
from typing import Dict, Optional

from PyQt5.QtCore import Qt, QTimer, QSettings, QSize
from PyQt5.QtGui import QIcon
from PyQt5.QtWidgets import (
    QMainWindow, QListWidget, QListWidgetItem,
    QToolBar, QAction, QLabel, QInputDialog,
    QMessageBox, QMenu, QWidget, QSizePolicy,
    QVBoxLayout, QHBoxLayout, QFrame,
)

from .ros_backend import RosBackend
from .device_panel import DevicePanel
from .can_monitor import CanMonitorDialog
from .log_dialog import LogDialog
from .widgets import SizedStackedWidget, DeviceListRow, SpinnerLabel, SPINNER_ACCENT_COLOR
from .app_info import logo_pixmap, sub_logo_pixmap, app_icon_pixmap, package_version
from .settings_dialog import SettingsDialog
from .about_dialog import AboutDialog, SERIAL_BRIDGE_URL
from .encoder_init_panel import EncoderInitPanel
from .firmware_config_dialog import FirmwareConfigDialog

UI_REFRESH_MS = 200
TOPIC_RESCAN_MS = 1000

# xiao_esp32_s3_smd_serial_bridge (MODE_CAN_HOST) は ros2can 自身のリポジトリの
# firmware/ 以下に同梱されている。
FIRMWARE_URL = "https://github.com/RRST-NHK-Project/ros2can/tree/main/firmware/xiao-esp32-s3_can2io"


class MainWindow(QMainWindow):
    def __init__(self, backend: RosBackend, parent=None):
        super().__init__(parent)
        self.backend = backend
        self.panels: Dict[int, DevicePanel] = {}
        self._can_monitor_dialog: Optional[CanMonitorDialog] = None

        # 過去分をまとめて遡って見るための詳細ログ(トグルで開くウィンドウ)。
        # ダイアログを開く前の分も逃さないよう遅延生成せずここで作る。最新1件だけは
        # 別途ステータスバー右側にも常時表示する(下の _build_toolbar / _on_log_message 参照)。
        self.log_dialog = LogDialog(self)
        self.backend.logMessage.connect(self.log_dialog.append_message)
        self.backend.logMessage.connect(self._on_log_message)

        self.setWindowTitle(f"RRST ros2can GUI - v{package_version()}")

        icon_pixmap = app_icon_pixmap()
        if icon_pixmap is not None:
            self.setWindowIcon(QIcon(icon_pixmap))

        # 前回終了時のウィンドウサイズ/位置を記憶し、毎回リサイズし直す手間を無くす。
        self._settings = QSettings("ros2can", "MainWindow")
        geometry = self._settings.value("geometry")
        if geometry is not None:
            self.restoreGeometry(geometry)
        else:
            self.resize(1000, 680)

        self._build_toolbar()

        self.device_list = QListWidget()
        self.device_list.setMinimumWidth(260)
        self.device_list.setMaximumWidth(400)
        self.device_list.currentItemChanged.connect(self._on_selection_changed)
        # currentItemChanged は選択が実際に変わった時しか発火しないため、
        # 「エンコーダ初期化」ページ(device_listの選択とは独立にstackを切り替える)
        # から、既に選択済みだったデバイス行を再クリックして戻ろうとしても
        # 反応しない問題があった。クリックには常に反応するitemClickedも
        # 併用し、同じ行の再クリックでも該当パネルへ確実に戻れるようにする。
        self.device_list.itemClicked.connect(self._on_device_item_clicked)
        self.device_list.setContextMenuPolicy(Qt.CustomContextMenu)
        self.device_list.customContextMenuRequested.connect(self._on_device_list_context_menu)

        self.device_list.setFrameShape(QListWidget.StyledPanel)
        self.device_list.setFrameShadow(QFrame.Sunken)

        self.stack = SizedStackedWidget()
        self.placeholder = QWidget()
        placeholder_layout = QVBoxLayout(self.placeholder)
        placeholder_layout.addStretch(2)

        # 1台も検出されていない間だけ表示するスキャン中インジケータ。
        # 以前は左のマイコン一覧パネル(幅220〜320pxしかない)の上に置いていたが、
        # 文言が見切れてしまっていたため、幅に余裕があるこちらのプレースホルダー
        # 画面側に移した(表示/非表示の切替は変わらず _refresh_device_list で行う)。
        self.device_list_scanning_label = SpinnerLabel("マイコンをスキャンしています…")
        self.device_list_scanning_label.setAlignment(Qt.AlignCenter)
        self.device_list_scanning_label.setStyleSheet(
            f"color: {SPINNER_ACCENT_COLOR}; font-size: 13pt; padding: 6px 0;")
        placeholder_layout.addWidget(self.device_list_scanning_label)

        placeholder_text = QLabel(
            "マイコン(CANホスト)が検出されていません。<br><br>"
            f"・<a href=\"{FIRMWARE_URL}\">xiao_esp32_s3_smd_serial_bridge</a> "
            f"(MODE_CAN_HOST) または <a href=\"{SERIAL_BRIDGE_URL}\">serial_bridge</a> の<br>"
            "&nbsp;&nbsp;ファームウェアを書き込んだマイコンをUSB接続してください。<br>"
            "&nbsp;&nbsp;ros2can が自動検出します。<br>"
            "・別プロセスが既に握っているトピックに相乗りしたい場合は、<br>"
            "&nbsp;&nbsp;上部の「デバイスを手動追加」を使用してください。<br>"
            "・実機なしで動作確認をしたい場合は、上部の「デバッグデバイスを追加」から<br>"
            "&nbsp;&nbsp;仮想デバイスを追加してください(TXの値がそのままRXにループバックされます)。<br><br>"
            "USB接続しているのに検出されない場合:<br>"
            "・lsusb / dmesg で /dev/ttyUSB* や /dev/ttyACM* として認識されているか確認してください。<br>"
            "・認識はされているのに反応がない場合、ユーザーが dialout グループに未所属で<br>"
            "&nbsp;&nbsp;シリアルポートの権限が無い可能性があります。以下を実行し、<br>"
            "&nbsp;&nbsp;一度ログアウト/ログイン(またはPC再起動)してください。<br>"
            "&nbsp;&nbsp;<code>sudo usermod -aG dialout $USER</code>")
        placeholder_text.setTextFormat(Qt.RichText)
        placeholder_text.setOpenExternalLinks(True)
        placeholder_text.setAlignment(Qt.AlignCenter)
        placeholder_text.setStyleSheet("color: #5f6368; font-size: 11pt;")
        placeholder_layout.addWidget(placeholder_text)

        placeholder_layout.addStretch(2)

        placeholder_pixmap = sub_logo_pixmap()
        if placeholder_pixmap is not None:
            placeholder_logo = QLabel()
            placeholder_logo.setPixmap(placeholder_pixmap.scaledToHeight(160, Qt.SmoothTransformation))
            placeholder_logo.setAlignment(Qt.AlignCenter)
            placeholder_layout.addWidget(placeholder_logo)

        placeholder_layout.addStretch(2)

        self.stack.addWidget(self.placeholder)

        self.encoder_panel = EncoderInitPanel(self.backend)
        self.stack.addWidget(self.encoder_panel)

        central = QWidget()
        central_layout = QHBoxLayout(central)
        central_layout.setContentsMargins(0, 0, 0, 0)
        central_layout.setSpacing(4)
        central_layout.addWidget(self.device_list)
        central_layout.addWidget(self.stack, 1)

        self.setCentralWidget(central)

        self.statusBar().showMessage("起動しました。トピックをスキャンしています…")

        # デバイスカウンタ(左下、showMessage)と対になる、最新の通信ログ1件だけを
        # 表示する右下の簡易インジケータ。過去分は log_dialog (ツールバーの
        # 「通信ログ…」) から遡って見られる。
        self.log_status_label = QLabel()
        self.log_status_label.setStyleSheet("color: #5f6368; font-size: 9pt; padding: 0 6px;")
        self.statusBar().addPermanentWidget(self.log_status_label)
        self.log_dialog.append_message("ros2can を起動しました")
        self._on_log_message("ros2can を起動しました")

        self.backend.deviceListChanged.connect(self._refresh_device_list)
        self.backend.deviceListChanged.connect(self.encoder_panel.rebuild)
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
        toolbar.setMinimumHeight(48)
        self.addToolBar(toolbar)

        rescan_action = QAction("再スキャン", self)
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

        encoder_init_action = QAction("エンコーダ初期化…", self)
        encoder_init_action.setToolTip(
            "検出済み全デバイスの原点セット対象チャンネル(角度/位置/カウンタ系)を"
            "1画面にまとめて表示し、行/デバイス単位/全デバイス一括で原点セットできます。")
        encoder_init_action.triggered.connect(self._on_open_encoder_init)
        toolbar.addAction(encoder_init_action)

        can_monitor_action = QAction("CANモニター…", self)
        can_monitor_action.setToolTip(
            "MODE_CAN_MONITORで書き込んだ基板のシリアル出力(生CANフレーム/デコード済み要約)を"
            "直接閲覧します。serial_bridgeフレームは使わないため、ros2canのデバイス一覧には出ません。")
        can_monitor_action.triggered.connect(self._on_open_can_monitor)
        toolbar.addAction(can_monitor_action)

        log_action = QAction("通信ログ…", self)
        log_action.setToolTip(
            "デバイスの接続/切断、チェックサム不一致などのフレーム異常、\n"
            "デバイスの追加/削除、E-STOPをタイムスタンプ付きで記録した履歴を表示します。\n"
            "新規ファームウェアの通信テスト時のデバッグにどうぞ。\n"
            "最新1件はステータスバー右下にも常時表示されます。")
        log_action.triggered.connect(self._on_open_log)
        toolbar.addAction(log_action)

        settings_action = QAction("設定…", self)
        settings_action.setToolTip(
            "除外ポートやタイムアウトなどのハードウェアスキャン設定を編集します。\n"
            "保存内容は ~/.config/ros2can/settings.yaml に保存され、Gitの追跡対象には"
            "なりません。")
        settings_action.triggered.connect(self._on_open_settings)
        toolbar.addAction(settings_action)

        firmware_config_action = QAction("ファームウェア設定を生成…", self)
        firmware_config_action.setToolTip(
            "xiao-esp32-s3_can2ioをテンプレートに、DEVICE_ID・CAN_ID・MODE・"
            "MULTI1-3・ENC1_MD/ENC2_MDを反映したプロジェクト一式を"
            "generated_firmware/<名前>/へ生成します(テンプレート自体は書き換えません)。"
            "書き込み(pio upload)は対象外です。")
        firmware_config_action.triggered.connect(self._on_open_firmware_config)
        toolbar.addAction(firmware_config_action)

        toolbar.addSeparator()

        estop_action = QAction("■ 全デバイス E-STOP (全ゼロ送信+TX無効化)", self)
        estop_action.setToolTip("全ての接続中デバイスのTXを即座にゼロにして送信を停止します")
        estop_action.triggered.connect(self._on_global_estop)
        toolbar.addAction(estop_action)

        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        toolbar.addWidget(spacer)

        pixmap = logo_pixmap()
        if pixmap is not None:
            logo_label = QLabel()
            logo_label.setPixmap(pixmap.scaledToHeight(44, Qt.SmoothTransformation))
            logo_label.setStyleSheet("padding: 0 6px;")
            toolbar.addWidget(logo_label)

        sub_pixmap = sub_logo_pixmap()
        if sub_pixmap is not None:
            sub_logo_label = QLabel()
            sub_logo_label.setPixmap(sub_pixmap.scaledToHeight(36, Qt.SmoothTransformation))
            toolbar.addWidget(sub_logo_label)

        version_label = QLabel(f"v{package_version()}")
        version_label.setStyleSheet("color: #5f6368; font-size: 10pt; padding: 0 10px;")
        toolbar.addWidget(version_label)

        about_action = QAction("Info…", self)
        about_action.setToolTip("バージョン情報とGitHubリンクを表示します。")
        about_action.triggered.connect(self._on_open_about)
        toolbar.addAction(about_action)

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

    def _on_open_encoder_init(self) -> None:
        """全デバイス横断のエンコーダ初期化ページに切り替える。

        device_list の選択とは独立させてある: QListWidget は最初に表示された
        時点で選択が無ければ先頭の項目を自動選択してしまう(Qtの仕様)ため、
        このページを device_list の項目として組み込むと、実機がまだ検出
        されていない起動直後にこのページが勝手に自動選択されてしまい、
        かつ以後は「未選択なら実デバイスを自動選択する」ロジックも働かなく
        なる不具合があった。ツールバーの独立ボタンにすることでその影響を
        受けないようにしている。
        """
        self.encoder_panel.rebuild()
        self.stack.setCurrentWidget(self.encoder_panel)

    def _on_open_can_monitor(self) -> None:
        """MODE_CAN_MONITOR機のシリアル出力を見るための独立ウィンドウを開く(モードレス)。"""
        if self._can_monitor_dialog is None:
            self._can_monitor_dialog = CanMonitorDialog(self)
        self._can_monitor_dialog.show()
        self._can_monitor_dialog.raise_()
        self._can_monitor_dialog.activateWindow()

    def _on_open_log(self) -> None:
        """通信ログの履歴(接続/切断/フレーム異常/デバイス増減等)ウィンドウを開く(モードレス)。"""
        self.log_dialog.show()
        self.log_dialog.raise_()
        self.log_dialog.activateWindow()

    def _on_log_message(self, text: str) -> None:
        """ステータスバー右下(デバイスカウンタの左下表示と対になる位置)に
        最新の通信ログ1件だけを表示する。"""
        self.log_status_label.setText(f"[{time.strftime('%H:%M:%S')}] {text}")

    def _on_open_settings(self) -> None:
        SettingsDialog(self.backend.hardware.config, self.backend.device_profile_map, self).exec_()

    def _on_open_firmware_config(self) -> None:
        FirmwareConfigDialog(self).exec_()

    def _on_open_about(self) -> None:
        AboutDialog(self).exec_()

    def _on_device_list_context_menu(self, pos) -> None:
        item = self.device_list.itemAt(pos)
        if item is None:
            return
        device_id = item.data(Qt.UserRole)
        menu = QMenu(self)
        remove_action = menu.addAction("デバイスを削除")
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
            item.setSizeHint(QSize(0, 46))
            self.device_list.addItem(item)
            self.device_list.setItemWidget(item, DeviceListRow())
            panel = DevicePanel(self.backend, device_id)
            self.panels[device_id] = panel
            self.stack.addWidget(panel)

        for device_id in listed_ids - existing_ids:
            self._remove_device_from_ui(device_id)

        self._update_list_labels()

        if self.device_list.currentItem() is None and self.device_list.count() > 0:
            self.device_list.setCurrentRow(0)

        self.device_list_scanning_label.setVisible(self.device_list.count() == 0)

    def _remove_device_from_ui(self, device_id: int) -> None:
        panel = self.panels.pop(device_id, None)
        if panel is not None:
            self.stack.removeWidget(panel)
            panel.deleteLater()
        for i in range(self.device_list.count()):
            item = self.device_list.item(i)
            if item.data(Qt.UserRole) == device_id:
                row = self.device_list.itemWidget(item)
                self.device_list.takeItem(i)
                if row is not None:
                    row.deleteLater()
                break

    def _update_list_labels(self) -> None:
        for i in range(self.device_list.count()):
            item = self.device_list.item(i)
            device_id = item.data(Qt.UserRole)
            ch = self.backend.devices.get(device_id)
            if ch is None:
                continue
            row = self.device_list.itemWidget(item)
            if not isinstance(row, DeviceListRow):
                continue
            state = "接続中" if ch.connected else "未接続"
            direct = " [TX ON]" if ch.direct_tx else ""
            passthrough = "" if ch.topic_passthrough else " [PASS OFF]"
            if ch.mode == "hardware":
                mode_label = f"HW:{ch.port}"
            elif ch.mode == "simulator":
                mode_label = "🧪DEBUG(仮想)"
            else:
                mode_label = "topic"
            # モード(HW:ポート名やDEBUG(仮想)等)とプロファイル名を同じ行に並べると
            # 一覧パネルの幅(220〜320px)に収まりきらず見切れることがあるため、
            # 行を分けて必ず両方とも見えるようにする。
            text = f"ID {device_id}  {state}{direct}{passthrough}\n{mode_label}\n{ch.profile_key}"
            row.set_state(ch.connected, text)
            # 固定高だとフォント/テーマ変更で2行テキストが見切れることがあるため、
            # 実際のコンテンツに合わせた高さへ都度更新する。
            item.setSizeHint(QSize(0, row.sizeHint().height() + 8))

    def _on_selection_changed(self, current: Optional[QListWidgetItem], _previous) -> None:
        if current is None:
            self.stack.setCurrentWidget(self.placeholder)
            return
        device_id = current.data(Qt.UserRole)
        panel = self.panels.get(device_id)
        if panel is not None:
            self.stack.setCurrentWidget(panel)

    def _on_device_item_clicked(self, item: QListWidgetItem) -> None:
        device_id = item.data(Qt.UserRole)
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
        elif current is self.encoder_panel:
            self.encoder_panel.refresh_values()
        connected = sum(1 for c in self.backend.devices.values() if c.connected)
        direct = sum(1 for c in self.backend.devices.values() if c.direct_tx)
        self.statusBar().showMessage(
            f"検出デバイス: {len(self.backend.devices)}  接続中: {connected}  ダイレクト送信: {direct}")

    # ---------------- safety ----------------

    def _on_global_estop(self) -> None:
        self.backend.emergency_stop_all()
        for panel in self.panels.values():
            panel.sync_estop_state()
        self._update_list_labels()
        QMessageBox.information(
            self, "E-STOP",
            "全デバイスへゼロ指令を送信し、トピック通過/ダイレクト送信を無効化しました。")

    def closeEvent(self, event) -> None:
        self._settings.setValue("geometry", self.saveGeometry())
        self.backend.emergency_stop_all()
        super().closeEvent(event)
