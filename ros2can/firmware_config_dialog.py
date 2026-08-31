"""firmware/xiao-esp32-s3_can2io をテンプレートに、書き込み用ファームウェア一式を
generated_firmware/<名前>/ へ生成するダイアログ。

DEVICE_ID・CAN_ID・MODE_*・MULTI1-3・ENC1_MD/ENC2_MDだけをGUIで編集し、それ以外
(ROBOMASのPIDゲイン等、実測でチューニングされた値を含む)には一切触れない。
テンプレート自体は書き換えず、常にプロジェクト一式を generated_firmware/ 配下の
名前付きフォルダへコピーしてから、そのコピー内の config.hpp だけを書き換える。
generated_firmware/ は .gitignore で追跡対象外(生成のたびに作り直される成果物)。
書き込み(pio upload)はスコープ外で、ここではプロジェクト一式の生成のみ行う。
"""

from __future__ import annotations

import os

from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout, QLineEdit, QSpinBox,
    QComboBox, QLabel, QPushButton, QDialogButtonBox, QMessageBox, QFileDialog,
)

from . import settings_store
from .firmware_config import (
    FirmwareConfig, apply_config, diff_lines, generate_project, parse_config,
    validate_output_name,
)

_MODE_LABELS = {
    "CAN": "CANノード(通常)",
    "CAN_HOST": "CANホスト",
    "IO": "IOのみ(CAN無し)",
    "DEBUG": "デバッグ",
    "CAN_MONITOR": "CANモニター",
    "ROBOMAS": "ロボマス(DJI RoboMaster)",
    "CUBEMARS": "CubeMars AK",
}

_TEMPLATE_RELATIVE_DIR = os.path.join("firmware", "xiao-esp32-s3_can2io")
_CONFIG_HPP_REL_PATH = os.path.join("src", "config.hpp")
_OUTPUT_ROOT_DIRNAME = "generated_firmware"


def _guess_template_dir() -> str:
    """ソースツリーから実行している場合のベストエフォートな初期パス探索。

    見つからなくても「変更…」で必ず選べるため、これは単なるUXの補助。
    """
    here = os.path.dirname(os.path.abspath(__file__))
    # ros2can/ros2can/firmware_config_dialog.py から見て ros2can/firmware/... を探す。
    candidate = os.path.normpath(os.path.join(here, "..", _TEMPLATE_RELATIVE_DIR))
    if os.path.isfile(os.path.join(candidate, _CONFIG_HPP_REL_PATH)):
        return candidate
    return ""


def _output_root_for(template_dir: str) -> str:
    """テンプレートの親ディレクトリ(ros2canリポジトリ直下)に generated_firmware/ を置く。"""
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(template_dir)))
    return os.path.join(repo_root, _OUTPUT_ROOT_DIRNAME)


class FirmwareConfigDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("ファームウェア設定を生成")
        self.resize(600, 460)
        self._cfg: FirmwareConfig = None
        self._template_text: str = ""
        self._template_dir: str = ""
        self._output_root: str = ""

        layout = QVBoxLayout(self)

        note = QLabel(
            "xiao-esp32-s3_can2io をテンプレートに、DEVICE_ID・CAN_ID・モード・\n"
            "MULTI1-3・ENC1_MD/ENC2_MD を反映したプロジェクト一式を\n"
            "generated_firmware/<名前>/ へ生成します(テンプレート自体は書き換えません)。\n"
            "ROBOMASのPIDゲイン等、その他の設定(チューニング値)には一切触れません。\n"
            "書き込み(pio upload)は対象外です。")
        note.setStyleSheet("color: #5f6368;")
        layout.addWidget(note)

        template_row = QHBoxLayout()
        self.template_edit = QLineEdit()
        self.template_edit.setReadOnly(True)
        template_row.addWidget(QLabel("テンプレート:"))
        template_row.addWidget(self.template_edit, 1)
        change_template_btn = QPushButton("変更…")
        change_template_btn.clicked.connect(self._on_change_template)
        template_row.addWidget(change_template_btn)
        layout.addLayout(template_row)

        form = QFormLayout()

        self.name_edit = QLineEdit()
        self.name_edit.setPlaceholderText("例: node1_robomas")
        form.addRow("生成先の名前:", self.name_edit)

        self.output_preview_label = QLabel("-")
        self.output_preview_label.setStyleSheet("color: #5f6368;")
        form.addRow("生成先:", self.output_preview_label)
        self.name_edit.textChanged.connect(self._update_output_preview)

        self.device_id_spin = QSpinBox()
        self.device_id_spin.setRange(0, 255)
        form.addRow("DEVICE_ID:", self.device_id_spin)

        self.can_id_spin = QSpinBox()
        self.can_id_spin.setRange(100, 199)
        form.addRow("CAN_ID (下2桁=ノード番号):", self.can_id_spin)

        self.mode_combo = QComboBox()
        form.addRow("MODE:", self.mode_combo)

        self.multi_combos = []
        for i in range(3):
            combo = QComboBox()
            combo.addItem("スイッチ入力 (0)", 0)
            combo.addItem("サーボ出力 (1)", 1)
            self.multi_combos.append(combo)
            form.addRow(f"MULTI{i + 1}:", combo)

        self.enc_md_combos = []
        for i in range(2):
            combo = QComboBox()
            combo.addItem("エンコーダ (0)", 0)
            combo.addItem("MD (1)", 1)
            self.enc_md_combos.append(combo)
            form.addRow(f"ENC{i + 1}_MD:", combo)

        layout.addLayout(form)
        layout.addStretch(1)

        self.buttons = QDialogButtonBox(QDialogButtonBox.Cancel)
        self.generate_btn = self.buttons.addButton("生成", QDialogButtonBox.AcceptRole)
        self.generate_btn.clicked.connect(self._on_generate)
        self.buttons.rejected.connect(self.reject)
        self.generate_btn.setEnabled(False)
        layout.addWidget(self.buttons)

        initial_template = settings_store.load_settings().get("firmware_template_dir", "") \
            or _guess_template_dir()
        if initial_template:
            self._load_template(initial_template)

    # ---------------- テンプレート選択/ロード ----------------

    def _on_change_template(self) -> None:
        start_dir = self.template_edit.text() or ""
        path = QFileDialog.getExistingDirectory(self, "テンプレートのプロジェクトフォルダを選択", start_dir)
        if path:
            self._load_template(path)

    def _load_template(self, template_dir: str) -> None:
        config_path = os.path.join(template_dir, _CONFIG_HPP_REL_PATH)
        try:
            with open(config_path, "r", encoding="utf-8") as f:
                text = f.read()
            cfg = parse_config(text)
        except (OSError, ValueError) as exc:
            QMessageBox.warning(self, "テンプレート読み込みエラー", f"{template_dir}\n\n{exc}")
            self.generate_btn.setEnabled(False)
            return

        self.template_edit.setText(template_dir)
        self._template_dir = template_dir
        self._template_text = text
        self._output_root = _output_root_for(template_dir)
        self._cfg = cfg

        self.device_id_spin.setValue(cfg.device_id)
        self.can_id_spin.setValue(cfg.can_id)

        self.mode_combo.clear()
        for name in cfg.available_modes:
            self.mode_combo.addItem(_MODE_LABELS.get(name, name), name)
        idx = self.mode_combo.findData(cfg.mode)
        if idx >= 0:
            self.mode_combo.setCurrentIndex(idx)

        for combo, value in zip(self.multi_combos, cfg.multi):
            combo.setCurrentIndex(combo.findData(value))
        for combo, value in zip(self.enc_md_combos, cfg.enc_md):
            combo.setCurrentIndex(combo.findData(value))

        self._update_output_preview()
        self.generate_btn.setEnabled(True)

    def _update_output_preview(self) -> None:
        name = self.name_edit.text().strip()
        if self._output_root and name:
            self.output_preview_label.setText(os.path.join(self._output_root, name))
        elif self._output_root:
            self.output_preview_label.setText(os.path.join(self._output_root, "(名前を入力してください)"))
        else:
            self.output_preview_label.setText("-")

    # ---------------- 生成 ----------------

    def _on_generate(self) -> None:
        if self._cfg is None:
            return

        try:
            name = validate_output_name(self.name_edit.text())
        except ValueError as exc:
            QMessageBox.warning(self, "入力エラー", str(exc))
            return

        cfg = FirmwareConfig(
            device_id=self.device_id_spin.value(),
            can_id=self.can_id_spin.value(),
            mode=self.mode_combo.currentData(),
            multi=[combo.currentData() for combo in self.multi_combos],
            enc_md=[combo.currentData() for combo in self.enc_md_combos],
            available_modes=self._cfg.available_modes,
        )

        try:
            new_config_text = apply_config(self._template_text, cfg)
        except ValueError as exc:
            QMessageBox.warning(self, "生成エラー", str(exc))
            return

        output_dir = os.path.join(self._output_root, name)
        changes = diff_lines(self._template_text, new_config_text)
        change_summary = "\n".join(
            f"L{line}: {old}  ->  {new}" for line, old, new in changes
        ) if changes else "(テンプレートから設定値の変更はありません)"
        exists_note = "既に存在するため、削除してから作り直します。\n\n" \
            if os.path.exists(output_dir) else ""

        reply = QMessageBox.question(
            self, "ファームウェアを生成します",
            f"{output_dir}\n\n{exists_note}"
            f"テンプレートからの config.hpp の変更点:\n\n{change_summary}\n\n"
            "生成しますか?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply != QMessageBox.Yes:
            return

        try:
            generate_project(self._template_dir, self._output_root, name, cfg)
        except (OSError, ValueError) as exc:
            QMessageBox.critical(self, "生成エラー", str(exc))
            return

        values = dict(settings_store.load_user_overrides())
        values["firmware_template_dir"] = self._template_dir
        settings_store.save_user_settings(values)

        QMessageBox.information(self, "生成完了", f"{output_dir} を生成しました。")
        self.accept()
