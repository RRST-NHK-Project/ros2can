"""firmware/xiao-esp32-s3_can2io/src/config.hpp の一部マクロを安全に読み書きする。

config.hpp にはROBOMASのPIDゲインやCubeMarsのMITレンジ等、実測でチューニングされた
値も同居しているため、ファイル全体をテンプレートから再生成するのではなく、対象の
マクロ行だけを正規表現で置換し、それ以外の行(コメント・チューニング値)には一切
触れない方式にする。対象は以下の3グループ:

- _SIMPLE_MACROS: DEVICE_ID/CAN_ID/MODE_*/MULTIn/ENCn_MD (基本設定)
- SERVO_MACROS:   SERVOn_MIN_US/MAX_US/MIN_DEG/MAX_DEG/INIT_DEG (サーボ設定)
- ADVANCED_MACROS: SERVO/MD PWM周波数・分解能、ENABLE_LED、CAN_NODE_COUNT等 (高度な設定)

ROBOMASのPIDゲイン等、#if分岐(モータ機種選択)内に同名マクロが複数回登場する定数は、
この「マクロ名で1行置換」方式では全分岐が同じ値に書き換わってしまい安全に扱えないため
対象外とし、テンプレートの値のまま変更しない。
"""

from __future__ import annotations

import os
import re
import shutil
from dataclasses import dataclass, field
from typing import List, Tuple

_SIMPLE_MACROS = ["DEVICE_ID", "CAN_ID", "MULTI1", "MULTI2", "MULTI3", "ENC1_MD", "ENC2_MD"]

_SERVO_FIELDS = ["MIN_US", "MAX_US", "MIN_DEG", "MAX_DEG", "INIT_DEG"]
SERVO_MACROS = [f"SERVO{i}_{field}" for i in range(1, 5) for field in _SERVO_FIELDS]

# 「高度な設定」としてGUIに分離するマクロ。通常は変更不要だが、PWM周波数や
# CAN_NODE_COUNT等、機種・実接続構成によっては変更が必要になるもの。
# ROBOMASのPIDゲイン等(#if ROBOMAS_MOTOR_TYPE == ... で分岐しており、同名マクロが
# ファイル内に複数回登場する)は、この単純な「マクロ名で1行置換」方式では正しく
# 扱えない(全分岐が同じ値に書き換わってしまう)ため、意図的にここへは含めない。
ADVANCED_MACROS = [
    "SERVO_PWM_FREQ", "SERVO_PWM_RESOLUTION",
    "MD_PWM_FREQ", "MD_PWM_RESOLUTION",
    "ENABLE_LED", "CAN_NODE_COUNT", "CAN_HOST_DIAG_ENABLE",
]

_ALL_MACROS = _SIMPLE_MACROS + SERVO_MACROS + ADVANCED_MACROS

CONFIG_HPP_REL_PATH = os.path.join("src", "config.hpp")
_COPY_IGNORE = shutil.ignore_patterns(".pio", ".vscode", "__pycache__", "*.pyc")

TEMPLATE_RELATIVE_DIR = os.path.join("firmware", "xiao-esp32-s3_can2io")
OUTPUT_ROOT_DIRNAME = "generated_firmware"

_INVALID_NAME_CHARS = re.compile(r'[\\/\0]')

_MODE_RE = re.compile(r"^(?P<indent>\s*)(?P<comment>//\s*)?#define\s+MODE_(?P<name>\w+)\s*$")


def _simple_macro_re(name: str) -> "re.Pattern[str]":
    return re.compile(rf"^(?P<prefix>\s*#define\s+{re.escape(name)}\s+)(?P<value>\S+)")


@dataclass
class FirmwareConfig:
    device_id: int
    can_id: int
    mode: str
    multi: List[int] = field(default_factory=lambda: [0, 0, 0])
    enc_md: List[int] = field(default_factory=lambda: [0, 0])
    available_modes: List[str] = field(default_factory=list)

    # ---- サーボ設定 (SERVO1-4) ----
    servo_min_us: List[int] = field(default_factory=lambda: [500, 500, 500, 500])
    servo_max_us: List[int] = field(default_factory=lambda: [2500, 2500, 2500, 2500])
    servo_min_deg: List[int] = field(default_factory=lambda: [0, 0, 0, 0])
    servo_max_deg: List[int] = field(default_factory=lambda: [270, 270, 270, 270])
    servo_init_deg: List[int] = field(default_factory=lambda: [0, 0, 0, 0])

    # ---- 高度な設定 (ADVANCED_MACROS) ----
    servo_pwm_freq: int = 50
    servo_pwm_resolution: int = 14
    md_pwm_freq: int = 20000
    md_pwm_resolution: int = 8
    enable_led: int = 1
    can_node_count: int = 4
    can_host_diag_enable: int = 0


def parse_config(text: str) -> FirmwareConfig:
    lines = text.splitlines()

    values = {}
    for name in _ALL_MACROS:
        pattern = _simple_macro_re(name)
        for line in lines:
            m = pattern.match(line)
            if m:
                values[name] = int(m.group("value"), 0)
                break
        else:
            raise ValueError(f"config.hpp内に #define {name} が見つかりません。")

    mode = None
    available_modes: List[str] = []
    for line in lines:
        m = _MODE_RE.match(line)
        if not m:
            continue
        name = m.group("name")
        available_modes.append(name)
        if m.group("comment") is None:
            if mode is not None:
                raise ValueError(
                    f"config.hpp内に有効な(コメントアウトされていない)MODE_*定義が"
                    f"複数見つかりました({mode}, {name})。")
            mode = name
    if mode is None:
        raise ValueError("config.hpp内に有効な(コメントアウトされていない)MODE_*定義が見つかりません。")

    return FirmwareConfig(
        device_id=values["DEVICE_ID"],
        can_id=values["CAN_ID"],
        mode=mode,
        multi=[values["MULTI1"], values["MULTI2"], values["MULTI3"]],
        enc_md=[values["ENC1_MD"], values["ENC2_MD"]],
        available_modes=available_modes,
        servo_min_us=[values[f"SERVO{i}_MIN_US"] for i in range(1, 5)],
        servo_max_us=[values[f"SERVO{i}_MAX_US"] for i in range(1, 5)],
        servo_min_deg=[values[f"SERVO{i}_MIN_DEG"] for i in range(1, 5)],
        servo_max_deg=[values[f"SERVO{i}_MAX_DEG"] for i in range(1, 5)],
        servo_init_deg=[values[f"SERVO{i}_INIT_DEG"] for i in range(1, 5)],
        servo_pwm_freq=values["SERVO_PWM_FREQ"],
        servo_pwm_resolution=values["SERVO_PWM_RESOLUTION"],
        md_pwm_freq=values["MD_PWM_FREQ"],
        md_pwm_resolution=values["MD_PWM_RESOLUTION"],
        enable_led=values["ENABLE_LED"],
        can_node_count=values["CAN_NODE_COUNT"],
        can_host_diag_enable=values["CAN_HOST_DIAG_ENABLE"],
    )


def apply_config(text: str, cfg: FirmwareConfig) -> str:
    if cfg.mode not in cfg.available_modes:
        raise ValueError(
            f"未知のモード '{cfg.mode}' です。config.hppに定義されているのは "
            f"{cfg.available_modes} のみです。")

    replacements = {
        "DEVICE_ID": cfg.device_id,
        "CAN_ID": cfg.can_id,
        "MULTI1": cfg.multi[0],
        "MULTI2": cfg.multi[1],
        "MULTI3": cfg.multi[2],
        "ENC1_MD": cfg.enc_md[0],
        "ENC2_MD": cfg.enc_md[1],
        "SERVO_PWM_FREQ": cfg.servo_pwm_freq,
        "SERVO_PWM_RESOLUTION": cfg.servo_pwm_resolution,
        "MD_PWM_FREQ": cfg.md_pwm_freq,
        "MD_PWM_RESOLUTION": cfg.md_pwm_resolution,
        "ENABLE_LED": cfg.enable_led,
        "CAN_NODE_COUNT": cfg.can_node_count,
        "CAN_HOST_DIAG_ENABLE": cfg.can_host_diag_enable,
    }
    for i in range(1, 5):
        replacements[f"SERVO{i}_MIN_US"] = cfg.servo_min_us[i - 1]
        replacements[f"SERVO{i}_MAX_US"] = cfg.servo_max_us[i - 1]
        replacements[f"SERVO{i}_MIN_DEG"] = cfg.servo_min_deg[i - 1]
        replacements[f"SERVO{i}_MAX_DEG"] = cfg.servo_max_deg[i - 1]
        replacements[f"SERVO{i}_INIT_DEG"] = cfg.servo_init_deg[i - 1]

    lines = text.splitlines(keepends=True)
    macro_patterns = {name: _simple_macro_re(name) for name in _ALL_MACROS}

    new_lines = []
    for line in lines:
        stripped_end = ""
        body = line
        for ending in ("\r\n", "\n", "\r"):
            if line.endswith(ending):
                stripped_end = ending
                body = line[: -len(ending)]
                break

        mode_m = _MODE_RE.match(body)
        if mode_m is not None:
            name = mode_m.group("name")
            indent = mode_m.group("indent")
            if name == cfg.mode:
                new_body = f"{indent}#define MODE_{name}"
            else:
                new_body = f"{indent}// #define MODE_{name}"
            new_lines.append(new_body + stripped_end)
            continue

        replaced = False
        for macro_name, pattern in macro_patterns.items():
            m = pattern.match(body)
            if m:
                new_body = f"{m.group('prefix')}{replacements[macro_name]}" + body[m.end():]
                new_lines.append(new_body + stripped_end)
                replaced = True
                break
        if not replaced:
            new_lines.append(line)

    return "".join(new_lines)


def diff_lines(old: str, new: str) -> List[Tuple[int, str, str]]:
    """(1-origin行番号, 旧内容, 新内容) のリスト。変更された行のみ。"""
    old_lines = old.splitlines()
    new_lines = new.splitlines()
    result = []
    for i, (o, n) in enumerate(zip(old_lines, new_lines), start=1):
        if o != n:
            result.append((i, o, n))
    return result


def guess_template_dir() -> str:
    """ros2canパッケージのソースツリーから実行している場合のベストエフォートな

    初期パス探索(ros2can/ros2can/firmware_config.py から見て
    ros2can/firmware/xiao-esp32-s3_can2io を探す)。見つからなくてもUI側で
    手動選択できるため必須ではない。
    """
    here = os.path.dirname(os.path.abspath(__file__))
    candidate = os.path.normpath(os.path.join(here, "..", TEMPLATE_RELATIVE_DIR))
    if os.path.isfile(os.path.join(candidate, CONFIG_HPP_REL_PATH)):
        return candidate
    return ""


def output_root_for(template_dir: str) -> str:
    """テンプレートの親ディレクトリ(ros2canリポジトリ直下)に

    generated_firmware/ を置く。
    """
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(template_dir)))
    return os.path.join(repo_root, OUTPUT_ROOT_DIRNAME)


def validate_output_name(name: str) -> str:
    """生成先フォルダ名として妥当か検証し、前後空白を除いた名前を返す。

    不正なら ValueError。パストラバーサル("..")やパス区切り文字を含む名前は
    output_root の外へ書き込んでしまうため拒否する。
    """
    name = name.strip()
    if not name:
        raise ValueError("生成先の名前を入力してください。")
    if name in (".", ".."):
        raise ValueError(f"'{name}' は名前として使用できません。")
    if _INVALID_NAME_CHARS.search(name):
        raise ValueError("名前にパス区切り文字は使用できません。")
    return name


def _resolve_project_dir(output_root: str, name: str) -> str:
    """output_root/<name>を解決し、output_rootの外を指していないか検証する。

    パストラバーサル対策。generate_project/delete_projectの両方から使う。
    """
    name = validate_output_name(name)
    output_root_abs = os.path.abspath(output_root)
    project_dir = os.path.abspath(os.path.join(output_root_abs, name))
    if os.path.commonpath([output_root_abs, project_dir]) != output_root_abs:
        raise ValueError("生成先の解決に失敗しました(output_rootの外を指しています)。")
    return project_dir


def generate_project(template_dir: str, output_root: str, name: str, cfg: FirmwareConfig) -> str:
    """template_dir配下のPlatformIOプロジェクト一式を output_root/<name>/ へコピーし、
    その中の src/config.hpp だけへ cfg を適用する。生成先パスを返す。

    生成先が既に存在する場合は一度削除してから作り直す(前回生成時の残骸
    (.pioビルドキャッシュ等)を残さないため)。output_root の外(テンプレート自身を
    含む)を書き換えないよう、解決後のパスが output_root 配下にあることを検証する。
    """
    template_config_path = os.path.join(template_dir, CONFIG_HPP_REL_PATH)
    if not os.path.isfile(template_config_path):
        raise ValueError(
            f"テンプレートに {CONFIG_HPP_REL_PATH} が見つかりません: {template_dir}")

    output_dir = _resolve_project_dir(output_root, name)
    if os.path.abspath(template_dir) == output_dir:
        raise ValueError("生成先がテンプレート自身と同じです。")

    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(os.path.abspath(output_root), exist_ok=True)
    shutil.copytree(template_dir, output_dir, ignore=_COPY_IGNORE)

    config_path = os.path.join(output_dir, CONFIG_HPP_REL_PATH)
    with open(config_path, "r", encoding="utf-8") as f:
        text = f.read()
    new_text = apply_config(text, cfg)
    with open(config_path, "w", encoding="utf-8") as f:
        f.write(new_text)

    return output_dir


def delete_project(output_root: str, name: str) -> None:
    """generate_project()で生成した output_root/<name>/ を削除する。

    generate_projectと同じパストラバーサル検証を行う。存在しない場合や
    output_rootの外を指す場合はValueError。
    """
    project_dir = _resolve_project_dir(output_root, name)
    if not os.path.isdir(project_dir):
        raise ValueError(f"生成物が見つかりません: {project_dir}")
    shutil.rmtree(project_dir)
