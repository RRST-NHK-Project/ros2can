"""全デバイス横断でエンコーダ原点セットを行う一覧ページ。

デバイスごとの DevicePanel (Monitor タブ) にもチャンネル単位の「原点セット」
ボタンはあるが、デバイスを1つずつ選び直す必要がある。起動時の一括初期化
(全デバイスのエンコーダ/位置カウンタをまとめてゼロ点セットしたい場面)向けに、
検出済み全デバイスの zeroable チャンネルを1画面に並べたページを提供する。

上記の「原点セット」はGUI側のソフトウェアオフセット(マイコン側の値は変更しない)
だが、これとは別に、実機のエンコーダ自体に原点を書き込むタイプの初期化
(例: CubeMars本体へのSet Origin CANコマンド送信)はros2can自身の機能ではなく、
対象ノード(soki_sim/trajectory_follower_nodeの/set_root_theta_origin等)が
提供するstd_srvs/Triggerサービス経由で行う。ros2canはCAN通信を直接扱う汎用
ツールでプロジェクト固有のノード名を知らないため、任意のTriggerサービス名を
指定して呼び出せる汎用パネルをここに追加した(2026-08-27)。
"""

from __future__ import annotations

from typing import Dict, Tuple

from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout, QGroupBox, QScrollArea,
    QPushButton, QLineEdit,
)
from std_srvs.srv import Trigger

from .device_profiles import all_profiles
from .ros_backend import RosBackend
from .widgets import ChannelMonitorRow


def _clear_layout(layout) -> None:
    while layout.count():
        item = layout.takeAt(0)
        w = item.widget()
        if w is not None:
            w.setParent(None)
            w.deleteLater()


class EncoderInitPanel(QWidget):
    """検出済み全デバイスの原点セット対象チャンネル (角度/位置/カウンタ系) を一覧表示するページ。"""

    def __init__(self, backend: RosBackend, parent=None):
        super().__init__(parent)
        self.backend = backend
        self.rows: Dict[Tuple[int, int], ChannelMonitorRow] = {}
        # std_srvs/Triggerサービス呼び出し用クライアント(サービス名ごとに遅延生成)。
        self._trigger_clients = {}

        outer = QVBoxLayout(self)

        header = QHBoxLayout()
        title = QLabel("<b>エンコーダ初期化 (検出済み全デバイス一覧)</b>")
        title.setStyleSheet("font-size:13pt;")
        header.addWidget(title)
        header.addStretch(1)
        self.zero_all_btn = QPushButton("検出済み全デバイスのエンコーダ原点セット")
        self.zero_all_btn.setToolTip(
            "検出済みの全デバイスに対し、全24チャンネルの現在値をゼロ点として記録する"
            "(マイコン側の値は変更しない)。")
        self.zero_all_btn.clicked.connect(self._on_zero_every_device_clicked)
        header.addWidget(self.zero_all_btn)
        outer.addLayout(header)

        info = QLabel(
            "マイコン側の値は変更せず、GUI側のソフトウェアオフセットとして各チャンネルの現在値を"
            "ゼロ点に記録します。対象は各デバイスの現在のプロファイルで zeroable な"
            "角度/位置/カウンタ系チャンネルです。")
        info.setWordWrap(True)
        info.setStyleSheet("color:#5f6368;")
        outer.addWidget(info)

        trigger_box = QGroupBox("外部ノードのTriggerサービス呼び出し")
        trigger_layout = QHBoxLayout(trigger_box)
        trigger_layout.addWidget(QLabel("サービス名:"))
        self.trigger_service_edit = QLineEdit("/set_root_theta_origin")
        trigger_layout.addWidget(self.trigger_service_edit, 1)
        self.trigger_call_btn = QPushButton("呼び出し")
        self.trigger_call_btn.setToolTip(
            "std_srvs/Trigger型の任意のROS 2サービスを呼び出す(本機自身の機能ではなく、"
            "他ノードが提供するサービスを叩くための汎用ボタン)。\n"
            "例: /set_root_theta_origin (soki_sim/trajectory_follower_node、"
            "root_theta_jointのCubeMars本体へSet Origin CANコマンドを送信)。\n"
            "呼び出し前に対象の実機・ノードが正しい状態か確認すること。")
        self.trigger_call_btn.clicked.connect(self._on_trigger_call_clicked)
        trigger_layout.addWidget(self.trigger_call_btn)
        self.trigger_status_label = QLabel("")
        self.trigger_status_label.setWordWrap(True)
        trigger_layout.addWidget(self.trigger_status_label, 1)
        outer.addWidget(trigger_box)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        self.content = QWidget()
        self.content_layout = QVBoxLayout(self.content)
        scroll.setWidget(self.content)
        outer.addWidget(scroll)

        self.rebuild()

    # ---------------- build / refresh ----------------

    def rebuild(self) -> None:
        """デバイスの増減やプロファイル変更を反映して一覧を作り直す。"""
        _clear_layout(self.content_layout)
        self.rows.clear()

        profiles = all_profiles()
        any_rows = False
        for device_id in sorted(self.backend.devices.keys()):
            ch = self.backend.devices[device_id]
            profile = profiles.get(ch.profile_key)
            if profile is None:
                continue
            zeroable_defs = sorted(
                (c for c in profile.rx if c.zeroable), key=lambda c: c.index)
            if not zeroable_defs:
                continue
            any_rows = True

            box = QGroupBox(f"Device ID {device_id}  ({profile.name})")
            v = QVBoxLayout(box)

            btn_row = QHBoxLayout()
            btn_row.addStretch(1)
            zero_device_btn = QPushButton("このデバイスの全チャンネル原点セット")
            zero_device_btn.clicked.connect(
                lambda _checked=False, did=device_id: self._on_zero_device_clicked(did))
            btn_row.addWidget(zero_device_btn)
            v.addLayout(btn_row)

            for chdef in zeroable_defs:
                row = ChannelMonitorRow(chdef)
                row.zeroRequested.connect(
                    lambda index, did=device_id: self._on_zero_channel_clicked(did, index))
                v.addWidget(row)
                self.rows[(device_id, chdef.index)] = row

            self.content_layout.addWidget(box)

        if not any_rows:
            empty_label = QLabel(
                "検出済みデバイスに、原点セット対象のチャンネル(角度/位置/カウンタ系)が"
                "ありません。")
            empty_label.setStyleSheet("color:#5f6368;")
            self.content_layout.addWidget(empty_label)

        self.content_layout.addStretch(1)
        self.refresh_values()

    def refresh_values(self) -> None:
        for (device_id, index), row in self.rows.items():
            ch = self.backend.devices.get(device_id)
            if ch is None:
                continue
            row.set_raw_value(ch.rx_data[index], self.backend.zeroed_value(device_id, index))

    # ---------------- actions ----------------

    def _on_zero_channel_clicked(self, device_id: int, index: int) -> None:
        self.backend.zero_channel(device_id, index)
        self.refresh_values()

    def _on_zero_device_clicked(self, device_id: int) -> None:
        self.backend.zero_all_channels(device_id)
        self.refresh_values()

    def _on_zero_every_device_clicked(self) -> None:
        for device_id in list(self.backend.devices.keys()):
            self.backend.zero_all_channels(device_id)
        self.refresh_values()

    def _on_trigger_call_clicked(self) -> None:
        service_name = self.trigger_service_edit.text().strip()
        if not service_name:
            return
        client = self._trigger_clients.get(service_name)
        if client is None:
            client = self.backend.node.create_client(Trigger, service_name)
            self._trigger_clients[service_name] = client
        if not client.service_is_ready():
            self.trigger_status_label.setText(f"{service_name}: サービス未起動です")
            self.trigger_status_label.setStyleSheet("color:#c00;")
            return
        self.trigger_call_btn.setEnabled(False)
        self.trigger_status_label.setText(f"{service_name}: 呼び出し中...")
        self.trigger_status_label.setStyleSheet("color:#5f6368;")
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda fut, name=service_name: self._on_trigger_response(name, fut))

    def _on_trigger_response(self, service_name: str, future) -> None:
        self.trigger_call_btn.setEnabled(True)
        try:
            response = future.result()
        except Exception as exc:
            self.trigger_status_label.setText(f"{service_name}: 失敗 ({exc})")
            self.trigger_status_label.setStyleSheet("color:#c00;")
            return
        color = "#080" if response.success else "#c00"
        self.trigger_status_label.setText(f"{service_name}: {response.message}")
        self.trigger_status_label.setStyleSheet(f"color:{color};")
