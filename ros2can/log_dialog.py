"""通信ログを表示するモードレスダイアログ。

新規ファームウェア開発・通信テストの際に「いつ・どのデバイスが繋がった/
切れたか」「チェックサム不一致などのフレーム異常が起きていないか」を
追えるようにするための簡易ログビューア。RosBackend.logMessage シグナルを
購読するだけの単純なビューア。

最新1件だけは main_window.py がステータスバー右側にも常時表示する
(左下のデバイスカウンタと対になる、右下の簡易インジケータ)。こちらは
過去分をまとめて遡って見たい時に開く詳細ビュー。
"""

from __future__ import annotations

import time

from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QPushButton, QCheckBox, QPlainTextEdit,
)

MAX_LOG_LINES = 4000


class LogDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("通信ログ")
        self.resize(760, 420)

        layout = QVBoxLayout(self)

        top = QHBoxLayout()
        self.autoscroll_check = QCheckBox("自動スクロール")
        self.autoscroll_check.setChecked(True)
        top.addWidget(self.autoscroll_check)
        top.addStretch(1)
        clear_btn = QPushButton("ログをクリア")
        clear_btn.clicked.connect(self._on_clear_clicked)
        top.addWidget(clear_btn)
        layout.addLayout(top)

        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setStyleSheet("font-family: monospace; font-size: 9pt;")
        self.log_view.setMaximumBlockCount(MAX_LOG_LINES)
        layout.addWidget(self.log_view)

    def append_message(self, text: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.log_view.appendPlainText(f"[{timestamp}] {text}")
        if self.autoscroll_check.isChecked():
            sb = self.log_view.verticalScrollBar()
            sb.setValue(sb.maximum())

    def _on_clear_clicked(self) -> None:
        self.log_view.clear()
