"""CANモニター(ファームウェアのMODE_CAN_MONITOR)のシリアル出力を閲覧するウィンドウ。

MODE_CAN_MONITORはserial_bridgeのバイナリフレーム(START_BYTE/DEVICE_ID/LEN/CHECKSUM)を
一切使わず、プレーンテキストの行(`[CAN RAW] ...` / `[CAN MON] ...`)をSerialへ流すだけの
モードなので、通常のHardwareManager/SerialLink(バイナリフレーム前提のパーサ)では読めない。
このモジュールはそれ専用の、排他制御なしの読み取り専用シリアルビューアを提供する。

[firmware/xiao-esp32-s3_can2io/src/can_task.cpp の MODE_CAN_MONITOR 実装と対になる]
"""

from __future__ import annotations

import re
from typing import Dict, Optional

import serial
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QComboBox, QPushButton, QLabel,
    QPlainTextEdit, QTableWidget, QTableWidgetItem, QCheckBox, QSplitter,
)

from .serial_link import list_serial_ports

BAUD_RATE = 115200
POLL_INTERVAL_MS = 20
MAX_LOG_LINES = 4000
NODE_TABLE_SLOT_COLUMNS = 8  # CAN_SLOTS_PER_NODEより余裕を持たせておく

# ファーム側の出力フォーマット(can_task.cpp printCanMonitorNodeSummary/printCanRawFrame)と一致させること
MON_LINE_RE = re.compile(r"^\[CAN MON\] node=(\d+)((?:\s+slot\d+=-?\d+)*)")
SLOT_RE = re.compile(r"slot(\d+)=(-?\d+)")


class CanMonitorDialog(QDialog):
    """MODE_CAN_MONITOR機の生シリアル出力を表示するモードレスダイアログ。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("CANモニター (MODE_CAN_MONITOR)")
        self.resize(900, 600)

        self._ser: Optional[serial.Serial] = None
        self._rx_buf = b""
        self._node_rows: Dict[int, int] = {}

        layout = QVBoxLayout(self)

        top = QHBoxLayout()
        top.addWidget(QLabel("ポート:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(180)
        top.addWidget(self.port_combo)

        self.refresh_btn = QPushButton("ポート更新")
        self.refresh_btn.clicked.connect(self._refresh_ports)
        top.addWidget(self.refresh_btn)

        self.connect_btn = QPushButton("接続")
        self.connect_btn.clicked.connect(self._on_connect_clicked)
        top.addWidget(self.connect_btn)

        self.status_label = QLabel("未接続")
        top.addWidget(self.status_label)
        top.addStretch(1)

        self.autoscroll_check = QCheckBox("自動スクロール")
        self.autoscroll_check.setChecked(True)
        top.addWidget(self.autoscroll_check)

        self.clear_btn = QPushButton("ログをクリア")
        self.clear_btn.clicked.connect(self._on_clear_clicked)
        top.addWidget(self.clear_btn)

        layout.addLayout(top)

        layout.addWidget(QLabel(
            "115200bpsの生テキストをそのまま表示します(serial_bridgeフレーム解釈は行いません)。"
            "書き込んだボードのconfig.hppでMODE_CAN_MONITORを有効にしてから接続してください。"))

        splitter = QSplitter(Qt.Horizontal)

        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setStyleSheet("font-family: monospace; font-size: 9pt;")
        self.log_view.setMaximumBlockCount(MAX_LOG_LINES)
        splitter.addWidget(self.log_view)

        self.node_table = QTableWidget(0, 1 + NODE_TABLE_SLOT_COLUMNS)
        self.node_table.setHorizontalHeaderLabels(
            ["node"] + [f"slot{i}" for i in range(NODE_TABLE_SLOT_COLUMNS)])
        self.node_table.verticalHeader().setVisible(False)
        splitter.addWidget(self.node_table)

        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        layout.addWidget(splitter)

        self._refresh_ports()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._poll)
        self.timer.start(POLL_INTERVAL_MS)

    # ---------------- ポート接続 ----------------

    def _refresh_ports(self) -> None:
        current = self.port_combo.currentText()
        self.port_combo.clear()
        self.port_combo.addItems(list_serial_ports())
        idx = self.port_combo.findText(current)
        if idx >= 0:
            self.port_combo.setCurrentIndex(idx)

    def _on_connect_clicked(self) -> None:
        if self._ser is not None:
            self._disconnect()
            return
        port = self.port_combo.currentText()
        if not port:
            self.status_label.setText("ポートが選択されていません")
            return
        try:
            self._ser = serial.Serial(port, BAUD_RATE, timeout=0)
        except (OSError, serial.SerialException) as exc:
            self.status_label.setText(f"接続失敗: {exc}")
            self._ser = None
            return
        self._rx_buf = b""
        self.connect_btn.setText("切断")
        self.status_label.setText(f"接続中: {port}")

    def _disconnect(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except OSError:
                pass
            self._ser = None
        self.connect_btn.setText("接続")
        self.status_label.setText("未接続")

    def _on_clear_clicked(self) -> None:
        self.log_view.clear()

    # ---------------- 受信ポーリング ----------------

    def _poll(self) -> None:
        if self._ser is None:
            return
        try:
            waiting = self._ser.in_waiting
            if waiting:
                self._rx_buf += self._ser.read(waiting)
        except (OSError, serial.SerialException) as exc:
            self.status_label.setText(f"切断されました: {exc}")
            self._disconnect()
            return

        while b"\n" in self._rx_buf:
            line, self._rx_buf = self._rx_buf.split(b"\n", 1)
            text = line.decode("utf-8", errors="replace").rstrip("\r")
            if text:
                self._handle_line(text)

    def _handle_line(self, text: str) -> None:
        self.log_view.appendPlainText(text)
        if self.autoscroll_check.isChecked():
            sb = self.log_view.verticalScrollBar()
            sb.setValue(sb.maximum())

        m = MON_LINE_RE.match(text)
        if m:
            node = int(m.group(1))
            slots = {int(idx): int(value) for idx, value in SLOT_RE.findall(m.group(2))}
            self._update_node_row(node, slots)

    def _update_node_row(self, node: int, slots: Dict[int, int]) -> None:
        if node not in self._node_rows:
            row = self.node_table.rowCount()
            self.node_table.insertRow(row)
            self._node_rows[node] = row
            self.node_table.setItem(row, 0, QTableWidgetItem(str(node)))
        row = self._node_rows[node]
        for slot_idx, value in slots.items():
            col = 1 + slot_idx
            if col >= self.node_table.columnCount():
                continue
            item = self.node_table.item(row, col)
            if item is None:
                item = QTableWidgetItem()
                self.node_table.setItem(row, col, item)
            item.setText(str(value))

    # ---------------- lifecycle ----------------

    def closeEvent(self, event) -> None:
        self.timer.stop()
        self._disconnect()
        super().closeEvent(event)
