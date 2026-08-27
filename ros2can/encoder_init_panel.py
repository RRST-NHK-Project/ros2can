"""全デバイス横断でエンコーダ原点セットを行う一覧ページ。

デバイスごとの DevicePanel (Monitor タブ) にもチャンネル単位の「原点セット」
ボタンはあるが、デバイスを1つずつ選び直す必要がある。起動時の一括初期化
(全デバイスのエンコーダ/位置カウンタをまとめてゼロ点セットしたい場面)向けに、
検出済み全デバイスの zeroable チャンネルを1画面に並べたページを提供する。
"""

from __future__ import annotations

from typing import Dict, Tuple

from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout, QGroupBox, QScrollArea,
    QPushButton,
)

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
        info.setStyleSheet("color:#888;")
        outer.addWidget(info)

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
            empty_label.setStyleSheet("color:#888;")
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
