"""generated_firmware/<名前>/ のプロジェクトを実機へ書き込む(pio run -t upload)ダイアログ。

ros2can自身がシリアルポートを排他専有している(serial_link.py の TIOCEXCL)ため、
書き込み前に対象ポートの接続を閉じ、書き込み中はバックグラウンドの
スキャナ(hardware_manager._ScannerThread)がそのポートへ触れないよう
HardwareManager.lock_port_for_flash() で一時的に除外する。書き込みは数十秒
かかりうるサブプロセス実行のため、GUIスレッドをブロックしないよう
QThread(FlashWorker)で行う(_ScannerThreadと同じパターン)。
"""

from __future__ import annotations

import os
import subprocess
from typing import List, Optional

from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QComboBox, QLabel, QPushButton,
    QPlainTextEdit, QMessageBox,
)

from . import settings_store
from .firmware_config import guess_template_dir, output_root_for, parse_config
from .firmware_flash import build_upload_command, find_pio_executable, list_generated_projects
from .hardware_manager import HardwareManager
from .serial_link import list_serial_ports

MAX_LOG_LINES = 4000


class FlashWorker(QThread):
    outputLine = pyqtSignal(str)
    finished_ = pyqtSignal(bool, str)

    def __init__(self, cmd: List[str], cwd: str, parent=None):
        super().__init__(parent)
        self._cmd = cmd
        self._cwd = cwd
        self._process: Optional[subprocess.Popen] = None
        self._stop_requested = False

    def request_stop(self) -> None:
        self._stop_requested = True
        if self._process is not None and self._process.poll() is None:
            self._process.terminate()

    def run(self) -> None:
        try:
            self._process = subprocess.Popen(
                self._cmd, cwd=self._cwd,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1)
        except OSError as exc:
            self.outputLine.emit(f"起動に失敗しました: {exc}")
            self.finished_.emit(False, str(exc))
            return

        for line in self._process.stdout:
            self.outputLine.emit(line.rstrip("\n"))

        returncode = self._process.wait()

        if self._stop_requested:
            self.finished_.emit(False, "中断されました")
        elif returncode == 0:
            self.finished_.emit(True, "書き込みが完了しました")
        else:
            self.finished_.emit(False, f"pioがエラー終了しました(終了コード {returncode})")


class FlashDialog(QDialog):
    def __init__(self, hardware_manager: HardwareManager, parent=None):
        super().__init__(parent)
        self.setWindowTitle("ファームウェアを書き込む")
        self.resize(700, 520)
        self._hardware_manager = hardware_manager
        self._worker: Optional[FlashWorker] = None
        self._locked_port: Optional[str] = None

        template_dir = settings_store.load_settings().get("firmware_template_dir", "") \
            or guess_template_dir()
        self._output_root = output_root_for(template_dir) if template_dir else ""

        layout = QVBoxLayout(self)

        note = QLabel(
            "generated_firmware/ 配下に生成済みのプロジェクトを、選択したポートへ"
            "pio run -t upload で書き込みます。書き込み中は対象ポートの他の通信は"
            "一時的に停止します。")
        note.setWordWrap(True)
        note.setStyleSheet("color: #5f6368;")
        layout.addWidget(note)

        project_row = QHBoxLayout()
        project_row.addWidget(QLabel("プロジェクト:"))
        self.project_combo = QComboBox()
        project_row.addWidget(self.project_combo, 1)
        refresh_projects_btn = QPushButton("更新")
        refresh_projects_btn.clicked.connect(self._refresh_projects)
        project_row.addWidget(refresh_projects_btn)
        layout.addLayout(project_row)

        port_row = QHBoxLayout()
        port_row.addWidget(QLabel("ポート:"))
        self.port_combo = QComboBox()
        port_row.addWidget(self.port_combo, 1)
        refresh_ports_btn = QPushButton("再スキャン")
        refresh_ports_btn.clicked.connect(self._refresh_ports)
        port_row.addWidget(refresh_ports_btn)
        layout.addLayout(port_row)

        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setStyleSheet("font-family: monospace; font-size: 9pt;")
        self.log_view.setMaximumBlockCount(MAX_LOG_LINES)
        layout.addWidget(self.log_view, 1)

        button_row = QHBoxLayout()
        self.write_btn = QPushButton("書き込み")
        self.write_btn.clicked.connect(self._on_write_clicked)
        button_row.addWidget(self.write_btn)
        self.abort_btn = QPushButton("中断")
        self.abort_btn.setEnabled(False)
        self.abort_btn.clicked.connect(self._on_abort_clicked)
        button_row.addWidget(self.abort_btn)
        button_row.addStretch(1)
        self.close_btn = QPushButton("閉じる")
        self.close_btn.clicked.connect(self.accept)
        button_row.addWidget(self.close_btn)
        layout.addLayout(button_row)

        self._refresh_projects()
        self._refresh_ports()

    # ---------------- 一覧更新 ----------------

    def _refresh_projects(self) -> None:
        self.project_combo.clear()
        if not self._output_root:
            return
        for name in list_generated_projects(self._output_root):
            label = name
            config_path = os.path.join(self._output_root, name, "src", "config.hpp")
            try:
                with open(config_path, "r", encoding="utf-8") as f:
                    cfg = parse_config(f.read())
                label = f"{name}  (DEVICE_ID={cfg.device_id}, CAN_ID={cfg.can_id}, {cfg.mode})"
            except (OSError, ValueError):
                pass
            self.project_combo.addItem(label, name)

    def _refresh_ports(self) -> None:
        current = self.port_combo.currentText()
        self.port_combo.clear()
        ports = list_serial_ports()
        self.port_combo.addItems(ports)
        idx = self.port_combo.findText(current)
        if idx >= 0:
            self.port_combo.setCurrentIndex(idx)

    # ---------------- 書き込み ----------------

    def _append_log(self, text: str) -> None:
        self.log_view.appendPlainText(text)
        sb = self.log_view.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _on_write_clicked(self) -> None:
        name = self.project_combo.currentData()
        if not name:
            QMessageBox.warning(self, "入力エラー", "書き込むプロジェクトを選択してください。")
            return

        port = self.port_combo.currentText().strip()
        if not port:
            QMessageBox.warning(self, "入力エラー", "書き込み先のポートを選択してください。")
            return

        pio_exe = find_pio_executable()
        if pio_exe is None:
            QMessageBox.warning(
                self, "PlatformIOが見つかりません",
                "pio コマンドが見つかりませんでした。PlatformIO Core をインストールして"
                "ください(例: pip install -U platformio)。")
            return

        project_dir = os.path.join(self._output_root, name)
        reply = QMessageBox.question(
            self, "書き込みを開始します",
            f"{port} へ {name} を書き込みます。\n"
            "書き込み中はこのポートの他の通信ができません。よろしいですか?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply != QMessageBox.Yes:
            return

        self._hardware_manager.lock_port_for_flash(port)
        self._locked_port = port

        self._set_busy(True)
        self.log_view.clear()
        self._append_log(f"$ {' '.join(build_upload_command(pio_exe, project_dir, port))}")

        cmd = build_upload_command(pio_exe, project_dir, port)
        self._worker = FlashWorker(cmd, project_dir, self)
        self._worker.outputLine.connect(self._append_log)
        self._worker.finished_.connect(self._on_flash_finished)
        self._worker.start()

    def _on_abort_clicked(self) -> None:
        if self._worker is not None:
            self._worker.request_stop()
            self.abort_btn.setEnabled(False)

    def _on_flash_finished(self, success: bool, message: str) -> None:
        self._append_log(f"-- {message} --")

        if self._locked_port is not None:
            self._hardware_manager.unlock_port_for_flash(self._locked_port)
            self._locked_port = None

        self._set_busy(False)
        self._worker = None

        if success:
            QMessageBox.information(self, "書き込み完了", message)
        else:
            QMessageBox.warning(self, "書き込み失敗", message)

    def _set_busy(self, busy: bool) -> None:
        self.write_btn.setEnabled(not busy)
        self.abort_btn.setEnabled(busy)
        self.project_combo.setEnabled(not busy)
        self.port_combo.setEnabled(not busy)
        self.close_btn.setEnabled(not busy)

    # ---------------- 終了制御 ----------------

    def closeEvent(self, event) -> None:
        if self._worker is not None and self._worker.isRunning():
            QMessageBox.warning(
                self, "書き込み中です",
                "書き込みが完了するまで閉じられません。先に「中断」してください。")
            event.ignore()
            return
        super().closeEvent(event)
