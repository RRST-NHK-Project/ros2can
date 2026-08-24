"""CANホスト1台分のデバッグパネル (Control / Monitor / Raw / Info タブ)。

Control / Monitor タブは、プロファイルが CAN ノード構成 (node_count > 0) を
持つ場合、ノードごとのサブタブとして表示する。これにより「どのマイコン
(CANノード)を操作するか」をタブ切り替えで直感的に選択できる。
"""

from __future__ import annotations

import copy
from typing import Dict

from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout, QGroupBox,
    QScrollArea, QComboBox, QPushButton, QCheckBox, QPlainTextEdit, QSizePolicy,
)

from .device_profiles import (
    all_profiles, DeviceProfile, RAW_OUT, RAW_IN,
    DIGITAL_IN, COUNTER, ENUM_IN,
    save_custom_profile, unique_custom_key,
)
from .ros_backend import RosBackend, DeviceChannel, MODE_SIMULATOR, MODE_TOPIC_CLIENT
from .widgets import ChannelControlRow, ChannelMonitorRow, RawSlotTable, LedIndicator, SizedTabWidget
from .profile_editor import ProfileEditorDialog

# シミュレータ(仮想デバイス)モードでMonitorタブから直接値を設定できるRX種別。
# 実機ではTX指令と無関係な独立したセンサ入力に相当する(ENC/SW等)。
_SIM_EDITABLE_RX_KINDS = (DIGITAL_IN, COUNTER, ENUM_IN)


def _clear_layout(layout) -> None:
    while layout.count():
        item = layout.takeAt(0)
        w = item.widget()
        if w is not None:
            w.setParent(None)
            w.deleteLater()


class DevicePanel(QWidget):
    def __init__(self, backend: RosBackend, device_id: int, parent=None):
        super().__init__(parent)
        self.backend = backend
        self.device_id = device_id
        self.channel: DeviceChannel = backend.devices[device_id]
        self.control_rows: Dict[int, ChannelControlRow] = {}
        # 通常は ChannelMonitorRow(読取専用)だが、シミュレータモードのENC/SW等は
        # ChannelControlRow(編集可能)になることがある(_make_monitor_row 参照)。
        self.monitor_rows: Dict[int, QWidget] = {}

        self._build_header()

        self.tabs = SizedTabWidget()

        self.control_host = QWidget()
        self.control_host_layout = QVBoxLayout(self.control_host)
        self.control_host_layout.setContentsMargins(0, 0, 0, 0)

        self.monitor_host = QWidget()
        self.monitor_host_layout = QVBoxLayout(self.monitor_host)
        self.monitor_host_layout.setContentsMargins(0, 0, 0, 0)

        self.tabs.addTab(self.control_host, "Control (指令送信)")
        self.tabs.addTab(self.monitor_host, "Monitor (センサ受信)")

        self.raw_tx_table = RawSlotTable(editable=True)
        self.raw_rx_table = RawSlotTable(editable=False)
        raw_widget = QWidget()
        raw_layout = QHBoxLayout(raw_widget)
        tx_box = QVBoxLayout()
        tx_box.addWidget(QLabel("TX (ROS -> ホスト -> CAN, 編集可能)"))
        tx_box.addWidget(self.raw_tx_table)
        rx_box = QVBoxLayout()
        rx_box.addWidget(QLabel("RX (CAN -> ホスト -> ROS, 読み取り専用)"))
        rx_box.addWidget(self.raw_rx_table)
        raw_layout.addLayout(tx_box)
        raw_layout.addLayout(rx_box)
        self.tabs.addTab(raw_widget, "Raw (全24スロット)")

        self.info_text = QPlainTextEdit()
        self.info_text.setReadOnly(True)
        self.info_text.setStyleSheet("font-family: monospace;")
        self.tabs.addTab(self.info_text, "Info")

        self.raw_tx_table.valueChanged.connect(self._on_raw_tx_changed)

        outer = QVBoxLayout(self)
        outer.addWidget(self.header)
        outer.addWidget(self.tabs)

        self._rebuild_for_profile()
        self.refresh_from_rx()

    # ---------------- header ----------------

    def _build_header(self) -> None:
        # 1行のQHBoxLayoutに全コントロールを詰め込むと、各ウィジェットの最小幅が
        # 単純合算されてウィンドウの最小幅を大きく押し上げてしまう(タブ切替や
        # デバイス追加では縮まらない)ため、上段(状態表示)/下段(操作)の2行に分ける。
        self.header = QGroupBox()
        outer = QVBoxLayout(self.header)

        top = QHBoxLayout()
        outer.addLayout(top)

        self.title_label = QLabel(f"<b>Device ID {self.device_id}</b>")
        self.title_label.setStyleSheet("font-size:13pt;")
        top.addWidget(self.title_label)

        self.led = LedIndicator()
        top.addWidget(self.led)
        self.status_label = QLabel("未接続")
        # 接続状態の説明文は長くなり得る(相乗り警告等)ため、これがウィンドウの
        # 最小幅を決めてしまわないよう、必要幅が足りない時は縮んでよいと明示する。
        self.status_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        top.addWidget(self.status_label)

        top.addSpacing(20)
        top.addWidget(QLabel("プロファイル:"))
        self.profile_combo = QComboBox()
        # プロファイル名は長いものがある(60文字超)ため、素のQComboBoxだと
        # 最長の項目に合わせてボックス自体が広がりウィンドウを圧迫してしまう。
        # 表示欄は一定の文字数分だけ確保し、全文はツールチップとポップアップで見る。
        self.profile_combo.setSizeAdjustPolicy(QComboBox.AdjustToMinimumContentsLengthWithIcon)
        self.profile_combo.setMinimumContentsLength(22)
        self.profile_combo.setMinimumWidth(260)
        self._reload_profile_list()
        self.profile_combo.currentIndexChanged.connect(self._on_profile_selected)
        top.addWidget(self.profile_combo)

        self.edit_profile_btn = QPushButton("プロファイル編集")
        self.edit_profile_btn.clicked.connect(self._on_edit_profile)
        top.addWidget(self.edit_profile_btn)

        top.addStretch(1)

        bottom = QHBoxLayout()
        outer.addLayout(bottom)

        self.passthrough_check = QCheckBox("トピック通過 (外部ノードの指令を反映)")
        self.passthrough_check.setChecked(True)
        self.passthrough_check.toggled.connect(self._on_passthrough_toggled)
        bottom.addWidget(self.passthrough_check)

        self.direct_check = QCheckBox("ダイレクト送信 (GUIから直接送信)")
        self.direct_check.setStyleSheet("QCheckBox { font-weight: bold; }")
        self.direct_check.toggled.connect(self._on_direct_toggled)
        bottom.addWidget(self.direct_check)

        bottom.addStretch(1)

        self.zero_btn = QPushButton("全スロットを0にして送信")
        self.zero_btn.setStyleSheet("background-color:#c0392b; color:white; font-weight:bold;")
        self.zero_btn.clicked.connect(self._on_zero_clicked)
        bottom.addWidget(self.zero_btn)

    def _reload_profile_list(self) -> None:
        self.profile_combo.blockSignals(True)
        self.profile_combo.clear()
        profiles = all_profiles()
        keys = sorted(profiles.keys(), key=lambda k: profiles[k].name)
        current_index = 0
        for i, key in enumerate(keys):
            self.profile_combo.addItem(profiles[key].name, key)
            self.profile_combo.setItemData(i, profiles[key].description, 3)  # Qt.ToolTipRole
            if key == self.channel.profile_key:
                current_index = i
        self.profile_combo.setCurrentIndex(current_index)
        self.profile_combo.blockSignals(False)

    def current_profile(self) -> DeviceProfile:
        profiles = all_profiles()
        return profiles.get(self.channel.profile_key, profiles["generic_raw"])

    # ---------------- profile change / rebuild ----------------

    def _on_profile_selected(self, _index: int) -> None:
        key = self.profile_combo.currentData()
        if key:
            self.channel.profile_key = key
            self._rebuild_for_profile()
            self.refresh_from_rx()

    def _on_edit_profile(self) -> None:
        base = self.current_profile()
        new_profile = copy.deepcopy(base)
        existing = all_profiles()
        if not base.editable:
            new_profile.key = unique_custom_key(base.key, existing)
            new_profile.name = f"{base.name} (カスタム)"
        dlg = ProfileEditorDialog(new_profile, self)
        if dlg.exec_():
            result = dlg.result_profile()
            save_custom_profile(result)
            self.channel.profile_key = result.key
            self._reload_profile_list()
            self._rebuild_for_profile()
            self.refresh_from_rx()

    def _build_channel_rows(self, defs, row_factory):
        """group名ごとにまとめた縦並びウィジェットを1つ作る。"""
        widget = QWidget()
        v = QVBoxLayout(widget)
        v.setContentsMargins(4, 4, 4, 4)
        rows = {}
        for chdef in sorted(defs, key=lambda c: c.index):
            row = row_factory(chdef)
            v.addWidget(row)
            rows[chdef.index] = row
        v.addStretch(1)
        return widget, rows

    def _make_control_row(self, chdef) -> ChannelControlRow:
        row = ChannelControlRow(chdef)
        row.valueChanged.connect(self._on_control_value_changed)
        return row

    def _make_monitor_row(self, chdef):
        """RX行を1つ作る。シミュレータモードでENC/SW等に該当する場合は、
        実機とは無関係な独立したセンサ入力をGUIから再現できるよう編集可能行にする。"""
        if self.channel.mode == MODE_SIMULATOR and chdef.kind in _SIM_EDITABLE_RX_KINDS:
            self.channel.sim_rx_override.add(chdef.index)
            row = ChannelControlRow(chdef)
            row.valueChanged.connect(self._on_sim_rx_changed)
            return row
        return ChannelMonitorRow(chdef)

    def _rebuild_for_profile(self) -> None:
        profile = self.current_profile()
        self.control_rows.clear()
        self.monitor_rows.clear()
        _clear_layout(self.control_host_layout)
        _clear_layout(self.monitor_host_layout)

        defined_tx = [c for c in profile.tx if c.kind != RAW_OUT]
        defined_rx = [c for c in profile.rx if c.kind != RAW_IN]

        if profile.node_count > 0:
            # ノードごとのサブタブとして「どのマイコンを操作するか」を選択できるようにする
            control_tabs = SizedTabWidget()
            monitor_tabs = SizedTabWidget()
            for node in range(profile.node_count):
                base = node * profile.slots_per_node
                node_no = node + 1
                node_tx = [c for c in defined_tx if base <= c.index < base + profile.slots_per_node]
                node_rx = [c for c in defined_rx if base <= c.index < base + profile.slots_per_node]

                cw, crows = self._build_channel_rows(node_tx, self._make_control_row)
                self.control_rows.update(crows)
                scroll_c = QScrollArea()
                scroll_c.setWidgetResizable(True)
                scroll_c.setWidget(cw)
                control_tabs.addTab(scroll_c, f"ノード{node_no}")

                mw, mrows = self._build_channel_rows(node_rx, self._make_monitor_row)
                self.monitor_rows.update(mrows)
                scroll_m = QScrollArea()
                scroll_m.setWidgetResizable(True)
                scroll_m.setWidget(mw)
                monitor_tabs.addTab(scroll_m, f"ノード{node_no}")

            self.control_host_layout.addWidget(control_tabs)
            self.monitor_host_layout.addWidget(monitor_tabs)
        else:
            # ノード構成を持たないプロファイル (汎用Raw等): groupごとに縦積み表示
            cw, crows = self._grouped_widget(defined_tx, self._make_control_row,
                                              "このプロファイルに個別定義された送信スロットはありません。"
                                              "Raw タブで全24スロットを直接編集してください。")
            self.control_rows.update(crows)
            scroll_c = QScrollArea()
            scroll_c.setWidgetResizable(True)
            scroll_c.setWidget(cw)
            self.control_host_layout.addWidget(scroll_c)

            mw, mrows = self._grouped_widget(defined_rx, self._make_monitor_row,
                                              "このプロファイルに個別定義された受信スロットはありません。"
                                              "Raw タブで全24スロットを直接確認してください。")
            self.monitor_rows.update(mrows)
            scroll_m = QScrollArea()
            scroll_m.setWidgetResizable(True)
            scroll_m.setWidget(mw)
            self.monitor_host_layout.addWidget(scroll_m)

        # --- Raw tab labels ---
        tx_labels = {c.index: c.label for c in profile.tx}
        rx_labels = {c.index: c.label for c in profile.rx}
        self.raw_tx_table.set_labels(tx_labels)
        self.raw_rx_table.set_labels(rx_labels)
        self.raw_tx_table.set_values(self.channel.tx_data)
        self.raw_rx_table.set_values(self.channel.rx_data)

        # 現在の tx_data を新しい Control 行にも反映
        for index, row in self.control_rows.items():
            row.set_raw_value(self.channel.tx_data[index])

    def _grouped_widget(self, defs, row_factory, empty_message: str):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        groups: Dict[str, QVBoxLayout] = {}
        rows = {}
        for chdef in sorted(defs, key=lambda c: c.index):
            group_name = chdef.group or "その他"
            if group_name not in groups:
                box = QGroupBox(group_name)
                v = QVBoxLayout(box)
                groups[group_name] = v
                layout.addWidget(box)
            row = row_factory(chdef)
            groups[group_name].addWidget(row)
            rows[chdef.index] = row
        if not defs:
            layout.addWidget(QLabel(empty_message))
        layout.addStretch(1)
        return widget, rows

    # ---------------- value flow ----------------

    def _on_control_value_changed(self, index: int, raw: int) -> None:
        self.channel.tx_data[index] = raw
        self.raw_tx_table.set_value_silent(index, raw)

    def _on_raw_tx_changed(self, index: int, raw: int) -> None:
        self.channel.tx_data[index] = raw
        row = self.control_rows.get(index)
        if row is not None:
            row.set_raw_value(raw)

    def _on_sim_rx_changed(self, index: int, raw: int) -> None:
        """シミュレータモードでMonitorタブのENC/SW等をGUIから手動設定した際の反映。"""
        self.backend.set_sim_rx_value(self.device_id, index, raw)

    def _on_passthrough_toggled(self, checked: bool) -> None:
        """トピック通過(外部ノード指令の反映)とダイレクト送信(GUIからの直接送信)は
        両立しない: 有効化すると、もう一方のチェックを自動でOFFにする。"""
        self.channel.topic_passthrough = checked
        if checked and self.direct_check.isChecked():
            self.direct_check.setChecked(False)

    def _on_direct_toggled(self, checked: bool) -> None:
        self.channel.direct_tx = checked
        self.direct_check.setStyleSheet(
            "QCheckBox { font-weight: bold; color: #c0392b; }" if checked
            else "QCheckBox { font-weight: bold; }")
        if checked and self.passthrough_check.isChecked():
            self.passthrough_check.setChecked(False)

    def _on_zero_clicked(self) -> None:
        self.backend.zero_and_send(self.device_id)
        for row in self.control_rows.values():
            row.set_raw_value(0)
        self.raw_tx_table.set_values(self.channel.tx_data)

    def sync_estop_state(self) -> None:
        """E-STOP などパネル外からの状態反映。

        トピック通過(外部ノード指令)/ダイレクト送信(GUI指令)の両方の
        自動送信経路を止めた状態(backend.emergency_stop_all() 済み)に
        チェックボックス表示を合わせる。channel側は既にOFFになっている
        ため、ここでは相互排他のトグルハンドラを経由せず直接同期する。
        """
        self.passthrough_check.blockSignals(True)
        self.passthrough_check.setChecked(False)
        self.passthrough_check.blockSignals(False)

        self.direct_check.blockSignals(True)
        self.direct_check.setChecked(False)
        self.direct_check.blockSignals(False)
        self.direct_check.setStyleSheet("QCheckBox { font-weight: bold; }")

        for row in self.control_rows.values():
            row.set_raw_value(0)
        self.raw_tx_table.set_values(self.channel.tx_data)

    # ---------------- rx refresh ----------------

    def refresh_from_rx(self) -> None:
        ch = self.channel
        self.led.set_state(ch.connected)
        if ch.mode == "hardware":
            mode_label = f"HW直結({ch.port})"
            mode_title = "CANホスト直結"
        elif ch.mode == "simulator":
            mode_label = "デバッグ(仮想デバイス・実機不要)"
            mode_title = "デバッグ(仮想)"
        else:
            mode_label = "トピック相乗り"
            mode_title = "トピック相乗り"

        if ch.mode == MODE_TOPIC_CLIENT and not ch.connected:
            # serial_bridge を併用しない運用のため、相乗り先トピックが未接続 = 実機を
            # どのros2canインスタンスも掴めていない異常状態とみなして警告表示する。
            self.status_label.setStyleSheet(
                "QLabel { font-weight: bold; color: #c0392b; }")
            self.status_label.setText(
                f"⚠ ハードウェア未検出(未接続)  RX {ch.rx_hz:.1f}Hz  [{mode_label}]")
        else:
            self.status_label.setStyleSheet("")
            self.status_label.setText(
                f"{'接続中' if ch.connected else '未接続'}  RX {ch.rx_hz:.1f}Hz  [{mode_label}]")
        self.title_label.setText(f"<b>Device ID {self.device_id}</b> ({mode_title})")

        for index, row in self.monitor_rows.items():
            row.set_raw_value(ch.rx_data[index])
        self.raw_rx_table.set_values(ch.rx_data)

        info_lines = [
            f"device_id       : {self.device_id}",
            f"mode            : {ch.mode}",
            f"port            : {ch.port}",
            f"profile         : {self.channel.profile_key}",
            f"manual_add      : {ch.manual}",
            f"connected       : {ch.connected}",
            f"rx_hz           : {ch.rx_hz:.2f}",
            f"rx_frame_count  : {ch.rx_frame_count}",
            f"tx_frame_count  : {ch.tx_frame_count}",
            f"topic_passthrough : {ch.topic_passthrough}",
            f"direct_tx          : {ch.direct_tx}",
            "",
            f"tx_data (24) = {ch.tx_data}",
            f"rx_data (24) = {ch.rx_data}",
        ]
        self.info_text.setPlainText("\n".join(info_lines))
