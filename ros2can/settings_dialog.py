"""ハードウェア直結スキャンの挙動を編集するダイアログ。

保存すると HardwareConfig (実行中のスキャナが参照しているオブジェクト) を
その場で書き換えて即座に適用しつつ、settings_store 経由で
~/.config/ros2can/settings.yaml にも保存する(次回起動時にも反映されるが、
リポジトリ外のため Git の追跡対象にはならない)。
"""

from __future__ import annotations

from typing import Dict, Optional

from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QFormLayout, QHBoxLayout, QLineEdit,
    QDoubleSpinBox, QSpinBox, QLabel, QDialogButtonBox, QPushButton,
    QGroupBox, QTableWidget, QComboBox, QMessageBox, QHeaderView,
)

from . import settings_store
from .hardware_manager import HardwareConfig
from .device_profiles import all_profiles


class SettingsDialog(QDialog):
    def __init__(self, config: HardwareConfig, device_profile_map: Dict[int, str], parent=None):
        super().__init__(parent)
        self.setWindowTitle("設定")
        self.resize(560, 620)
        self._config = config
        # 呼び出し元 (RosBackend) が持つ辞書そのものへの参照。保存時はこれを
        # in-place で更新する(config の各属性を直接書き換えているのと同じ方式)。
        self._device_profile_map = device_profile_map

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

        map_group = QGroupBox("device_id ↔ プロファイル対応表")
        map_layout = QVBoxLayout(map_group)
        map_note = QLabel(
            "ここに登録したdevice_idのデバイスが検出されると、対応するプロファイルが\n"
            "自動的に初期選択されます(GUIで手動選択したプロファイルはこの表を上書きしません)。\n"
            "チーム共通の既定値にしたい場合は、保存後に config/ros2can.yaml の\n"
            "device_profile_map へ転記してください。")
        map_note.setStyleSheet("color: #888;")
        map_layout.addWidget(map_note)

        self.profile_map_table = QTableWidget(0, 2)
        self.profile_map_table.setHorizontalHeaderLabels(["Device ID", "プロファイル"])
        self.profile_map_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.profile_map_table.verticalHeader().setVisible(False)
        for device_id, profile_key in sorted(device_profile_map.items()):
            self._add_profile_map_row(device_id, profile_key)
        map_layout.addWidget(self.profile_map_table)

        map_btn_row = QHBoxLayout()
        add_row_btn = QPushButton("行を追加")
        add_row_btn.clicked.connect(lambda: self._add_profile_map_row())
        map_btn_row.addWidget(add_row_btn)
        remove_row_btn = QPushButton("選択行を削除")
        remove_row_btn.clicked.connect(self._on_remove_profile_map_row)
        map_btn_row.addWidget(remove_row_btn)
        map_btn_row.addStretch(1)
        map_layout.addLayout(map_btn_row)

        layout.addWidget(map_group)

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

        self.profile_map_table.setRowCount(0)
        default_map = settings_store.parse_device_profile_map(defaults.get("device_profile_map", []))
        for device_id, profile_key in sorted(default_map.items()):
            self._add_profile_map_row(device_id, profile_key)

    # ---------------- device_id <-> プロファイル対応表 ----------------

    def _add_profile_map_row(self, device_id: int = 0, profile_key: Optional[str] = None) -> None:
        row = self.profile_map_table.rowCount()
        self.profile_map_table.insertRow(row)

        id_spin = QSpinBox()
        id_spin.setRange(0, 255)
        id_spin.setValue(device_id)
        self.profile_map_table.setCellWidget(row, 0, id_spin)

        combo = QComboBox()
        profiles = all_profiles()
        # 未知のプロファイルキー(カスタムプロファイルが削除された等)を選択肢に無い
        # まま保存してしまうと別プロファイルにすり替わるため、先頭にそのまま残す。
        if profile_key and profile_key not in profiles:
            combo.addItem(f"(不明なプロファイル: {profile_key})", profile_key)
        current_index = 0
        for key in sorted(profiles.keys(), key=lambda k: profiles[k].name):
            combo.addItem(profiles[key].name, key)
            combo.setItemData(combo.count() - 1, profiles[key].description, 3)  # Qt.ToolTipRole
            if key == profile_key:
                current_index = combo.count() - 1
        combo.setCurrentIndex(current_index)
        self.profile_map_table.setCellWidget(row, 1, combo)

    def _on_remove_profile_map_row(self) -> None:
        row = self.profile_map_table.currentRow()
        if row >= 0:
            self.profile_map_table.removeRow(row)

    def _collect_device_profile_map(self) -> Optional[Dict[int, str]]:
        mapping: Dict[int, str] = {}
        for row in range(self.profile_map_table.rowCount()):
            id_spin = self.profile_map_table.cellWidget(row, 0)
            combo = self.profile_map_table.cellWidget(row, 1)
            device_id = id_spin.value()
            profile_key = combo.currentData()
            if device_id in mapping:
                QMessageBox.warning(
                    self, "設定",
                    f"Device ID {device_id} が対応表に複数回指定されています。"
                    "重複行を削除してから保存してください。")
                return None
            mapping[device_id] = profile_key
        return mapping

    def _on_save(self) -> None:
        excluded = sorted({
            p.strip() for p in self.excluded_ports_edit.text().split(",") if p.strip()
        })
        device_profile_map = self._collect_device_profile_map()
        if device_profile_map is None:
            return

        values = {
            "excluded_ports": excluded,
            "rx_timeout_sec": self.rx_timeout_spin.value(),
            "reconnect_interval_sec": self.reconnect_spin.value(),
            "scan_interval_ms": self.scan_interval_spin.value(),
            "probe_timeout_sec": self.probe_timeout_spin.value(),
            "probe_settle_sec": self.probe_settle_spin.value(),
            "device_profile_map": [
                f"{device_id}:{profile_key}"
                for device_id, profile_key in sorted(device_profile_map.items())
            ],
        }

        self._config.excluded_ports = set(excluded)
        self._config.rx_timeout_sec = values["rx_timeout_sec"]
        self._config.reconnect_interval_sec = values["reconnect_interval_sec"]
        self._config.scan_interval_sec = values["scan_interval_ms"] / 1000.0
        self._config.probe_timeout_sec = values["probe_timeout_sec"]
        self._config.probe_settle_sec = values["probe_settle_sec"]
        self._device_profile_map.clear()
        self._device_profile_map.update(device_profile_map)

        settings_store.save_user_settings(values)
        self.accept()
