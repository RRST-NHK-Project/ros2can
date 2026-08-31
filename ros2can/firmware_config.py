"""firmware/xiao-esp32-s3_can2io/src/config.hpp の一部マクロを安全に読み書きする。

config.hpp にはROBOMASのPIDゲインやCubeMarsのMITレンジ等、実測でチューニングされた
値も同居しているため、ファイル全体をテンプレートから再生成するのではなく、
DEVICE_ID/CAN_ID/MODE_*/MULTIn/ENCn_MD の対象行だけを正規表現で置換し、それ以外の
行(コメント・チューニング値)には一切触れない方式にする。
"""

from __future__ import annotations

import os
import re
import shutil
from dataclasses import dataclass, field
from typing import List, Tuple

_SIMPLE_MACROS = ["DEVICE_ID", "CAN_ID", "MULTI1", "MULTI2", "MULTI3", "ENC1_MD", "ENC2_MD"]
_CONFIG_HPP_REL_PATH = os.path.join("src", "config.hpp")
_COPY_IGNORE = shutil.ignore_patterns(".pio", ".vscode", "__pycache__", "*.pyc")

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


def parse_config(text: str) -> FirmwareConfig:
    lines = text.splitlines()

    values = {}
    for name in _SIMPLE_MACROS:
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
    }

    lines = text.splitlines(keepends=True)
    macro_patterns = {name: _simple_macro_re(name) for name in _SIMPLE_MACROS}

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


def generate_project(template_dir: str, output_root: str, name: str, cfg: FirmwareConfig) -> str:
    """template_dir配下のPlatformIOプロジェクト一式を output_root/<name>/ へコピーし、
    その中の src/config.hpp だけへ cfg を適用する。生成先パスを返す。

    生成先が既に存在する場合は一度削除してから作り直す(前回生成時の残骸
    (.pioビルドキャッシュ等)を残さないため)。output_root の外(テンプレート自身を
    含む)を書き換えないよう、解決後のパスが output_root 配下にあることを検証する。
    """
    name = validate_output_name(name)

    template_config_path = os.path.join(template_dir, _CONFIG_HPP_REL_PATH)
    if not os.path.isfile(template_config_path):
        raise ValueError(
            f"テンプレートに {_CONFIG_HPP_REL_PATH} が見つかりません: {template_dir}")

    output_root_abs = os.path.abspath(output_root)
    output_dir = os.path.abspath(os.path.join(output_root_abs, name))
    if os.path.commonpath([output_root_abs, output_dir]) != output_root_abs:
        raise ValueError("生成先の解決に失敗しました(output_rootの外を指しています)。")
    if os.path.abspath(template_dir) == output_dir:
        raise ValueError("生成先がテンプレート自身と同じです。")

    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(output_root_abs, exist_ok=True)
    shutil.copytree(template_dir, output_dir, ignore=_COPY_IGNORE)

    config_path = os.path.join(output_dir, _CONFIG_HPP_REL_PATH)
    with open(config_path, "r", encoding="utf-8") as f:
        text = f.read()
    new_text = apply_config(text, cfg)
    with open(config_path, "w", encoding="utf-8") as f:
        f.write(new_text)

    return output_dir
