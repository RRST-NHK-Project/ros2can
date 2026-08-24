"""ハードウェア直結スキャンの挙動を編集するダイアログ。

保存すると HardwareConfig (実行中のスキャナが参照しているオブジェクト) を
その場で書き換えて即座に適用しつつ、settings_store 経由で
~/.config/ros2can/settings.yaml にも保存する(次回起動時にも反映されるが、
リポジトリ外のため Git の追跡対象にはならない)。
"""

from __future__ import annotations

from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QFormLayout, QHBoxLayout, QLineEdit,
    QDoubleSpinBox, QSpinBox, QLabel, QDialogButtonBox, QPushButton,
)

from . import settings_store
from .hardware_manager import HardwareConfig


class SettingsDialog(QDialog):
    def __init__(self, config: HardwareConfig, parent=None):
        super().__init__(parent)
        self.setWindowTitle("設定")
        self.resize(480, 320)
        self._config = config

        layout = QVBoxLayout(self)
        note = QLabel(
            "ハードウェア直結スキャンの挙動を設定します。「保存」すると即座に適用され、\n"
            f"{settings_store.user_settings_path()} にローカル保存されます\n"
            "(リポジトリ外のため Git の追跡対象にはなりません)。\n"
            "チーム共通の既定値を変えたい場合は config/ros2can.yaml を編集してください。")
        note.setStyleSheet("color: #888;")
        layout.addWidget(note)

        form = QFormLayout()

        self.excluded_ports_edit = QLineEdit(", ".join(sorted(config.excluded_ports)))
        self.excluded_ports_edit.setPlaceholderText("/dev/ttyUSB0, /dev/ttyACM1")
        form.addRow("除外ポート (カンマ区切り):", self.excluded_ports_edit)

        self.rx_timeout_spin = QDoubleSpinBox()
        self.rx_timeout_spin.setRange(0.1, 60.0)
        self.rx_timeout_spin.setSingleStep(0.1)
        self.rx_timeout_spin.setValue(config.rx_timeout_sec)
        form.addRow("RXタイムアウト [秒]:", self.rx_timeout_spin)

        self.reconnect_spin = QDoubleSpinBox()
        self.reconnect_spin.setRange(0.1, 60.0)
        self.reconnect_spin.setSingleStep(0.1)
        self.reconnect_spin.setValue(config.reconnect_interval_sec)
        form.addRow("再接続インターバル [秒]:", self.reconnect_spin)

        self.scan_interval_spin = QSpinBox()
        self.scan_interval_spin.setRange(100, 60000)
        self.scan_interval_spin.setSingleStep(100)
        self.scan_interval_spin.setValue(int(round(config.scan_interval_sec * 1000)))
        form.addRow("スキャンインターバル [ms]:", self.scan_interval_spin)

        self.probe_timeout_spin = QDoubleSpinBox()
        self.probe_timeout_spin.setRange(0.1, 30.0)
        self.probe_timeout_spin.setSingleStep(0.1)
        self.probe_timeout_spin.setValue(config.probe_timeout_sec)
        form.addRow("プローブタイムアウト [秒]:", self.probe_timeout_spin)

        self.probe_settle_spin = QDoubleSpinBox()
        self.probe_settle_spin.setRange(0.0, 10.0)
        self.probe_settle_spin.setSingleStep(0.1)
        self.probe_settle_spin.setValue(config.probe_settle_sec)
        form.addRow("ポートオープン後の安定待ち [秒]:", self.probe_settle_spin)

        layout.addLayout(form)

        reset_row = QHBoxLayout()
        reset_btn = QPushButton("既定値を読み込む (config/ros2can.yaml)")
        reset_btn.setToolTip("ローカルの上書きは保存するまで反映されません。")
        reset_btn.clicked.connect(self._on_load_defaults)
        reset_row.addWidget(reset_btn)
        reset_row.addStretch(1)
        layout.addLayout(reset_row)

        buttons = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self._on_save)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def _on_load_defaults(self) -> None:
        defaults = settings_store.bundled_defaults()
        self.excluded_ports_edit.setText(", ".join(sorted(defaults.get("excluded_ports", []))))
        self.rx_timeout_spin.setValue(defaults.get("rx_timeout_sec", 2.0))
        self.reconnect_spin.setValue(defaults.get("reconnect_interval_sec", 3.0))
        self.scan_interval_spin.setValue(int(defaults.get("scan_interval_ms", 5000)))
        self.probe_timeout_spin.setValue(defaults.get("probe_timeout_sec", 2.0))
        self.probe_settle_spin.setValue(defaults.get("probe_settle_sec", 0.5))

    def _on_save(self) -> None:
        excluded = sorted({
            p.strip() for p in self.excluded_ports_edit.text().split(",") if p.strip()
        })
        values = {
            "excluded_ports": excluded,
            "rx_timeout_sec": self.rx_timeout_spin.value(),
            "reconnect_interval_sec": self.reconnect_spin.value(),
            "scan_interval_ms": self.scan_interval_spin.value(),
            "probe_timeout_sec": self.probe_timeout_spin.value(),
            "probe_settle_sec": self.probe_settle_spin.value(),
        }

        self._config.excluded_ports = set(excluded)
        self._config.rx_timeout_sec = values["rx_timeout_sec"]
        self._config.reconnect_interval_sec = values["reconnect_interval_sec"]
        self._config.scan_interval_sec = values["scan_interval_ms"] / 1000.0
        self._config.probe_timeout_sec = values["probe_timeout_sec"]
        self._config.probe_settle_sec = values["probe_settle_sec"]

        settings_store.save_user_settings(values)
        self.accept()
