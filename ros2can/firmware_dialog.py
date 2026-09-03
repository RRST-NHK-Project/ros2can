"""firmware/xiao-esp32-s3_can2io をテンプレートに、書き込み用ファームウェア一式を
generated_firmware/<名前>/ へ生成し、そのまま同じ画面から実機へ書き込む(pio run -t
upload)ダイアログ。

生成: DEVICE_ID・CAN_ID・MODE_*・BOARD_VARIANT・MULTI1-3・ENC1_MD/ENC2_MD・ENC2_SW・
サーボ設定(SERVOn_*、n=1-5)・高度な設定(ADVANCED_MACROS、既定では折りたたみ)だけを
GUIで編集し、それ以外(ROBOMASのPIDゲイン等、実測でチューニングされた値を含む。
BOARD_VARIANTごとに#if分岐するCAN_NODE_COUNT/CAN_SLOTS_PER_NODEも同じ理由で対象外)
には一切触れない。テンプレート
自体は書き換えず、常にプロジェクト一式を generated_firmware/ 配下の名前付きフォルダへ
コピーしてから、そのコピー内の config.hpp だけを書き換える。generated_firmware/ は
Git管理下(生成物として履歴に残す)。不要になった生成物はこのダイアログの「削除」から
取り除ける。

書き込み: ros2can自身がシリアルポートを排他専有している(serial_link.py の TIOCEXCL)ため、
書き込み前に対象ポートの接続を閉じ、書き込み中はバックグラウンドのスキャナ
(hardware_manager._ScannerThread)がそのポートへ触れないよう
HardwareManager.lock_port_for_flash() で一時的に除外する。書き込みは数十秒かかりうる
サブプロセス実行のため、GUIスレッドをブロックしないよう QThread(FlashWorker)で行う
(_ScannerThreadと同じパターン)。
"""

from __future__ import annotations

import os
import subprocess
from typing import List, Optional

from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout, QGridLayout, QLineEdit,
    QSpinBox, QComboBox, QLabel, QPushButton, QToolButton, QGroupBox, QWidget,
    QScrollArea, QFrame, QPlainTextEdit, QDialogButtonBox, QMessageBox,
    QFileDialog,
)

from . import settings_store
from .firmware_config import (
    CONFIG_HPP_REL_PATH, FirmwareConfig, apply_config, delete_project, diff_lines,
    generate_project, guess_template_dir, output_root_for, parse_config,
    validate_output_name,
)
from .firmware_flash import build_upload_command, find_pio_executable, list_generated_projects
from .hardware_manager import HardwareManager
from .serial_link import list_serial_ports

MAX_LOG_LINES = 4000

_MODE_LABELS = {
    "CAN": "CANノード(通常)",
    "CAN_HOST": "CANホスト",
    "IO": "IOのみ(CAN無し)",
    "DEBUG": "デバッグ",
    "CAN_MONITOR": "CANモニター",
    "ROBOMAS": "ロボマス(DJI RoboMaster)",
    "CUBEMARS": "CubeMars AK",
}

_BOARD_LABELS = {
    "BOARD_SOKI": "SOKI (soki本体基板、既存のピン配置)",
    "BOARD_MES": "MES (ENC1/ENC2/MD1/MD2 + SW1)",
    "BOARD_SS": "SS (SERVO1-5 + TR1-4 ソレノイドバルブ)",
}


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


def _section_label(text: str) -> QLabel:
    label = QLabel(text)
    label.setStyleSheet("font-weight: bold;")
    return label


class _CollapsibleBox(QWidget):
    """既定で折りたたまれた見出し付きコンテナ。

    通常は変更不要な「高度な設定」を、基本設定と分離して隠しておくために使う
    (押すと展開/折りたたみするだけの単純なトグル)。
    """

    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)

        self._toggle_btn = QToolButton()
        self._toggle_btn.setText(title)
        self._toggle_btn.setCheckable(True)
        self._toggle_btn.setChecked(False)
        self._toggle_btn.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self._toggle_btn.setArrowType(Qt.RightArrow)
        self._toggle_btn.toggled.connect(self._on_toggled)
        outer.addWidget(self._toggle_btn)

        self.content_area = QWidget()
        self.content_area.setVisible(False)
        outer.addWidget(self.content_area)

    def _on_toggled(self, checked: bool) -> None:
        self._toggle_btn.setArrowType(Qt.DownArrow if checked else Qt.RightArrow)
        self.content_area.setVisible(checked)


class FirmwareDialog(QDialog):
    def __init__(self, hardware_manager: HardwareManager, parent=None):
        super().__init__(parent)
        self.setWindowTitle("ファームウェア生成・書き込み")
        self.resize(1300, 820)
        self._hardware_manager = hardware_manager
        self._cfg: Optional[FirmwareConfig] = None
        self._template_text: str = ""
        self._template_dir: str = ""
        self._output_root: str = ""
        self._worker: Optional[FlashWorker] = None
        self._locked_port: Optional[str] = None

        layout = QVBoxLayout(self)

        panels_row = QHBoxLayout()

        # ==================== 生成 (左) ====================
        gen_panel = QVBoxLayout()
        gen_panel.addWidget(_section_label("生成"))

        gen_note = QLabel(
            "xiao-esp32-s3_can2io をテンプレートに、DEVICE_ID・CAN_ID・モード・基板"
            "(BOARD_VARIANT)・MULTI1-3・ENC1_MD/ENC2_MD/ENC2_SW・サーボ設定・高度な設定を"
            "反映したプロジェクト一式を generated_firmware/<名前>/ へ生成します"
            "(テンプレート自体は書き換えません)。ROBOMASのPIDゲイン等、その他の設定"
            "(チューニング値)には一切触れません。")
        gen_note.setWordWrap(True)
        gen_note.setStyleSheet("color: #5f6368;")
        gen_panel.addWidget(gen_note)

        gen_scroll = QScrollArea()
        gen_scroll.setWidgetResizable(True)
        gen_scroll.setFrameShape(QFrame.NoFrame)
        gen_scroll.setMinimumWidth(420)
        gen_container = QWidget()
        gen_layout = QVBoxLayout(gen_container)
        gen_layout.setContentsMargins(0, 0, 0, 0)
        gen_scroll.setWidget(gen_container)
        gen_panel.addWidget(gen_scroll, 1)

        template_row = QHBoxLayout()
        self.template_edit = QLineEdit()
        self.template_edit.setReadOnly(True)
        template_row.addWidget(QLabel("テンプレート:"))
        template_row.addWidget(self.template_edit, 1)
        change_template_btn = QPushButton("変更…")
        change_template_btn.clicked.connect(self._on_change_template)
        template_row.addWidget(change_template_btn)
        gen_layout.addLayout(template_row)

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

        self.board_combo = QComboBox()
        self.board_combo.setToolTip(
            "書き込み先の物理基板を選択してください。ピン配置(defs.hpp)と入出力"
            "ロジックが基板ごとに切り替わります(BOARD_MES/BOARD_SSはMODE_IO非対応)。")
        form.addRow("BOARD_VARIANT:", self.board_combo)

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

        self.enc2_sw_combo = QComboBox()
        self.enc2_sw_combo.addItem("ENC2 (エンコーダ) (0)", 0)
        self.enc2_sw_combo.addItem("SW2/SW3 (スイッチ) (1)", 1)
        self.enc2_sw_combo.setToolTip(
            "BOARD_MES専用。ENC2とSW2/SW3はピン共有のため、どちらで使うかを選択します。"
            "他のBOARD_VARIANTでは無視されます。")
        form.addRow("ENC2_SW (BOARD_MES専用):", self.enc2_sw_combo)

        gen_layout.addLayout(form)

        # ---- サーボ設定 (SERVO1-5) ----
        gen_layout.addWidget(_section_label("サーボ設定"))
        servo_note = QLabel(
            "SERVOn_MIN_US/MAX_US(パルス幅) と MIN_DEG/MAX_DEG/INIT_DEG(角度) を"
            "サーボごとに設定します。BOARD_SOKIではMULTIn=1(サーボ出力)のチャンネルの"
            "みSERVO1-3が有効、BOARD_SSではSERVO1-5が常時有効です(SERVO4/5は"
            "BOARD_SOKI/BOARD_MESでは未配線・未使用)。")
        servo_note.setWordWrap(True)
        servo_note.setStyleSheet("color: #5f6368;")
        gen_layout.addWidget(servo_note)

        self.servo_min_us_spins: List[QSpinBox] = []
        self.servo_max_us_spins: List[QSpinBox] = []
        self.servo_min_deg_spins: List[QSpinBox] = []
        self.servo_max_deg_spins: List[QSpinBox] = []
        self.servo_init_deg_spins: List[QSpinBox] = []
        for i in range(5):
            box = QGroupBox(f"SERVO{i + 1}")
            servo_form = QFormLayout(box)

            min_us = QSpinBox()
            min_us.setRange(0, 5000)
            min_us.setSuffix(" us")
            servo_form.addRow("MIN_US:", min_us)
            self.servo_min_us_spins.append(min_us)

            max_us = QSpinBox()
            max_us.setRange(0, 5000)
            max_us.setSuffix(" us")
            servo_form.addRow("MAX_US:", max_us)
            self.servo_max_us_spins.append(max_us)

            min_deg = QSpinBox()
            min_deg.setRange(0, 360)
            min_deg.setSuffix(" deg")
            servo_form.addRow("MIN_DEG:", min_deg)
            self.servo_min_deg_spins.append(min_deg)

            max_deg = QSpinBox()
            max_deg.setRange(0, 360)
            max_deg.setSuffix(" deg")
            servo_form.addRow("MAX_DEG:", max_deg)
            self.servo_max_deg_spins.append(max_deg)

            init_deg = QSpinBox()
            init_deg.setRange(0, 360)
            init_deg.setSuffix(" deg")
            servo_form.addRow("INIT_DEG:", init_deg)
            self.servo_init_deg_spins.append(init_deg)

            gen_layout.addWidget(box)

        # ---- 高度な設定 (通常は変更不要) ----
        advanced_box = _CollapsibleBox("高度な設定 (通常は変更不要)")
        advanced_form = QFormLayout(advanced_box.content_area)

        self.servo_pwm_freq_spin = QSpinBox()
        self.servo_pwm_freq_spin.setRange(1, 500)
        self.servo_pwm_freq_spin.setSuffix(" Hz")
        advanced_form.addRow("SERVO_PWM_FREQ:", self.servo_pwm_freq_spin)

        self.servo_pwm_resolution_spin = QSpinBox()
        self.servo_pwm_resolution_spin.setRange(1, 16)
        self.servo_pwm_resolution_spin.setSuffix(" bit")
        advanced_form.addRow("SERVO_PWM_RESOLUTION:", self.servo_pwm_resolution_spin)

        self.md_pwm_freq_spin = QSpinBox()
        self.md_pwm_freq_spin.setRange(1, 100000)
        self.md_pwm_freq_spin.setSuffix(" Hz")
        advanced_form.addRow("MD_PWM_FREQ:", self.md_pwm_freq_spin)

        self.md_pwm_resolution_spin = QSpinBox()
        self.md_pwm_resolution_spin.setRange(1, 16)
        self.md_pwm_resolution_spin.setSuffix(" bit")
        advanced_form.addRow("MD_PWM_RESOLUTION:", self.md_pwm_resolution_spin)

        self.enable_led_combo = QComboBox()
        self.enable_led_combo.addItem("無効 (0)", 0)
        self.enable_led_combo.addItem("有効 (1)", 1)
        advanced_form.addRow("ENABLE_LED:", self.enable_led_combo)

        can_node_count_note = QLabel(
            "CAN_NODE_COUNT / CAN_SLOTS_PER_NODE はBOARD_VARIANTごとにconfig.hpp内で"
            "#if分岐しているため、このGUIからは編集できません(BOARD_SOKI/BOARD_MES: "
            "4ノード×5スロット、BOARD_SS: 2ノード×9スロット)。実際の接続台数に合わせて"
            "変更したい場合は生成後のプロジェクトのsrc/config.hppを直接編集してください。")
        can_node_count_note.setWordWrap(True)
        can_node_count_note.setStyleSheet("color: #5f6368;")
        advanced_form.addRow("CAN_NODE_COUNT:", can_node_count_note)

        self.can_host_diag_enable_combo = QComboBox()
        self.can_host_diag_enable_combo.addItem("無効 (0)", 0)
        self.can_host_diag_enable_combo.addItem("有効 (1)", 1)
        self.can_host_diag_enable_combo.setToolTip(
            "有効にするとCAN診断ログがros2can用USBシリアルに直接出力され、"
            "serial_bridgeフレームと混ざる(切り分け用途以外は無効のままにすること)。")
        advanced_form.addRow("CAN_HOST_DIAG_ENABLE:", self.can_host_diag_enable_combo)

        gen_layout.addWidget(advanced_box)

        gen_button_row = QHBoxLayout()
        gen_button_row.addStretch(1)
        self.generate_btn = QPushButton("生成")
        self.generate_btn.clicked.connect(self._on_generate)
        self.generate_btn.setEnabled(False)
        gen_button_row.addWidget(self.generate_btn)
        gen_layout.addLayout(gen_button_row)

        panels_row.addLayout(gen_panel, 3)

        vdivider = QFrame()
        vdivider.setFrameShape(QFrame.VLine)
        vdivider.setFrameShadow(QFrame.Sunken)
        panels_row.addWidget(vdivider)

        # ==================== 書き込み (右) ====================
        flash_panel = QVBoxLayout()
        flash_panel.addWidget(_section_label("書き込み"))

        flash_note = QLabel(
            "生成済みのプロジェクトを、選択したポートへ pio run -t upload で書き込みます。"
            "書き込み中は対象ポートの他の通信は一時的に停止します。")
        flash_note.setWordWrap(True)
        flash_note.setStyleSheet("color: #5f6368;")
        flash_panel.addWidget(flash_note)

        project_row = QHBoxLayout()
        project_row.addWidget(QLabel("プロジェクト:"))
        self.project_combo = QComboBox()
        project_row.addWidget(self.project_combo, 1)
        refresh_projects_btn = QPushButton("更新")
        refresh_projects_btn.clicked.connect(self._refresh_projects)
        project_row.addWidget(refresh_projects_btn)
        self.delete_project_btn = QPushButton("削除")
        self.delete_project_btn.setToolTip("選択中の生成物(generated_firmware/<名前>/)を削除します。")
        self.delete_project_btn.clicked.connect(self._on_delete_project)
        project_row.addWidget(self.delete_project_btn)
        flash_panel.addLayout(project_row)

        port_row = QHBoxLayout()
        port_row.addWidget(QLabel("ポート:"))
        self.port_combo = QComboBox()
        port_row.addWidget(self.port_combo, 1)
        refresh_ports_btn = QPushButton("再スキャン")
        refresh_ports_btn.clicked.connect(self._refresh_ports)
        port_row.addWidget(refresh_ports_btn)
        flash_panel.addLayout(port_row)

        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setStyleSheet("font-family: monospace; font-size: 9pt;")
        self.log_view.setMaximumBlockCount(MAX_LOG_LINES)
        flash_panel.addWidget(self.log_view, 1)

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
        flash_panel.addLayout(button_row)

        panels_row.addLayout(flash_panel, 2)
        layout.addLayout(panels_row, 1)

        initial_template = settings_store.load_settings().get("firmware_template_dir", "") \
            or guess_template_dir()
        if initial_template:
            self._load_template(initial_template)

        self._refresh_projects()
        self._refresh_ports()

    # ==================== 生成: テンプレート選択/ロード ====================

    def _on_change_template(self) -> None:
        start_dir = self.template_edit.text() or ""
        path = QFileDialog.getExistingDirectory(self, "テンプレートのプロジェクトフォルダを選択", start_dir)
        if path:
            self._load_template(path)

    def _load_template(self, template_dir: str) -> None:
        config_path = os.path.join(template_dir, CONFIG_HPP_REL_PATH)
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
        self._output_root = output_root_for(template_dir)
        self._cfg = cfg

        self.device_id_spin.setValue(cfg.device_id)
        self.can_id_spin.setValue(cfg.can_id)

        self.mode_combo.clear()
        for name in cfg.available_modes:
            self.mode_combo.addItem(_MODE_LABELS.get(name, name), name)
        idx = self.mode_combo.findData(cfg.mode)
        if idx >= 0:
            self.mode_combo.setCurrentIndex(idx)

        self.board_combo.clear()
        for name in cfg.available_boards:
            self.board_combo.addItem(_BOARD_LABELS.get(name, name), name)
        idx = self.board_combo.findData(cfg.board_variant)
        if idx >= 0:
            self.board_combo.setCurrentIndex(idx)

        for combo, value in zip(self.multi_combos, cfg.multi):
            combo.setCurrentIndex(combo.findData(value))
        for combo, value in zip(self.enc_md_combos, cfg.enc_md):
            combo.setCurrentIndex(combo.findData(value))
        self.enc2_sw_combo.setCurrentIndex(self.enc2_sw_combo.findData(cfg.enc2_sw))

        for spin, value in zip(self.servo_min_us_spins, cfg.servo_min_us):
            spin.setValue(value)
        for spin, value in zip(self.servo_max_us_spins, cfg.servo_max_us):
            spin.setValue(value)
        for spin, value in zip(self.servo_min_deg_spins, cfg.servo_min_deg):
            spin.setValue(value)
        for spin, value in zip(self.servo_max_deg_spins, cfg.servo_max_deg):
            spin.setValue(value)
        for spin, value in zip(self.servo_init_deg_spins, cfg.servo_init_deg):
            spin.setValue(value)

        self.servo_pwm_freq_spin.setValue(cfg.servo_pwm_freq)
        self.servo_pwm_resolution_spin.setValue(cfg.servo_pwm_resolution)
        self.md_pwm_freq_spin.setValue(cfg.md_pwm_freq)
        self.md_pwm_resolution_spin.setValue(cfg.md_pwm_resolution)
        self.enable_led_combo.setCurrentIndex(self.enable_led_combo.findData(cfg.enable_led))
        self.can_host_diag_enable_combo.setCurrentIndex(
            self.can_host_diag_enable_combo.findData(cfg.can_host_diag_enable))

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

    # ==================== 生成 ====================

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
            board_variant=self.board_combo.currentData(),
            available_boards=self._cfg.available_boards,
            multi=[combo.currentData() for combo in self.multi_combos],
            enc_md=[combo.currentData() for combo in self.enc_md_combos],
            enc2_sw=self.enc2_sw_combo.currentData(),
            available_modes=self._cfg.available_modes,
            servo_min_us=[spin.value() for spin in self.servo_min_us_spins],
            servo_max_us=[spin.value() for spin in self.servo_max_us_spins],
            servo_min_deg=[spin.value() for spin in self.servo_min_deg_spins],
            servo_max_deg=[spin.value() for spin in self.servo_max_deg_spins],
            servo_init_deg=[spin.value() for spin in self.servo_init_deg_spins],
            servo_pwm_freq=self.servo_pwm_freq_spin.value(),
            servo_pwm_resolution=self.servo_pwm_resolution_spin.value(),
            md_pwm_freq=self.md_pwm_freq_spin.value(),
            md_pwm_resolution=self.md_pwm_resolution_spin.value(),
            enable_led=self.enable_led_combo.currentData(),
            can_host_diag_enable=self.can_host_diag_enable_combo.currentData(),
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

        # 生成した直後にそのまま書き込みへ進めるよう、一覧を更新して選択しておく。
        self._refresh_projects()
        idx = self.project_combo.findData(name)
        if idx >= 0:
            self.project_combo.setCurrentIndex(idx)

    # ==================== 書き込み: 一覧更新 ====================

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
                label = (
                    f"{name}  (DEVICE_ID={cfg.device_id}, CAN_ID={cfg.can_id}, "
                    f"{cfg.mode}, {cfg.board_variant})"
                )
            except (OSError, ValueError):
                pass
            self.project_combo.addItem(label, name)

    def _on_delete_project(self) -> None:
        name = self.project_combo.currentData()
        if not name:
            QMessageBox.warning(self, "入力エラー", "削除するプロジェクトを選択してください。")
            return

        project_dir = os.path.join(self._output_root, name)
        reply = QMessageBox.question(
            self, "生成物を削除します",
            f"{project_dir}\n\nを削除します。元に戻せません。よろしいですか?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply != QMessageBox.Yes:
            return

        try:
            delete_project(self._output_root, name)
        except (OSError, ValueError) as exc:
            QMessageBox.critical(self, "削除エラー", str(exc))
            return

        self._refresh_projects()

    def _refresh_ports(self) -> None:
        current = self.port_combo.currentText()
        self.port_combo.clear()
        ports = list_serial_ports()
        self.port_combo.addItems(ports)
        idx = self.port_combo.findText(current)
        if idx >= 0:
            self.port_combo.setCurrentIndex(idx)

    # ==================== 書き込み ====================

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
        self.delete_project_btn.setEnabled(not busy)
        self.port_combo.setEnabled(not busy)
        self.close_btn.setEnabled(not busy)
        self.generate_btn.setEnabled(not busy and self._cfg is not None)

    # ==================== 終了制御 ====================

    def closeEvent(self, event) -> None:
        if self._worker is not None and self._worker.isRunning():
            QMessageBox.warning(
                self, "書き込み中です",
                "書き込みが完了するまで閉じられません。先に「中断」してください。")
            event.ignore()
            return
        super().closeEvent(event)
