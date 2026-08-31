"""ハードウェアスキャン関連の「各種設定」のロード/保存。

優先順位 (後勝ち):
  1. DEFAULT_SETTINGS (コード上のフォールバック)
  2. config/ros2can.yaml (パッケージ同梱、Git管理される「一括設定ファイル」。
     チーム全体で共有したい既定値はここを編集する)
  3. ~/.config/ros2can/settings.yaml (GUIの「設定」ダイアログで保存する
     ユーザーローカルの上書き。リポジトリの外にあるため Git の追跡対象には
     ならない。device_profiles.py のカスタムプロファイル保存と同じ方式)
  4. ROS 2 パラメータ (--ros-args -p や launch の parameters=[...] で明示的に
     渡された値。RosBackend._load_hardware_config_from_params が
     declare_parameter の default に load_settings() の結果を渡すことで、
     通常のROSパラメータ機構の優先順位がそのまま最終的な上書きとして働く)
"""

from __future__ import annotations

import os
from typing import Any, Dict

import yaml
from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = 'ros2can'
NODE_NAME = 'ros2can_gui'

DEFAULT_SETTINGS: Dict[str, Any] = {
    "excluded_ports": [],
    "rx_timeout_sec": 2.0,
    "reconnect_interval_sec": 3.0,
    "scan_interval_ms": 5000,
    "probe_timeout_sec": 2.0,
    "probe_settle_sec": 0.5,
    # device_id とプロファイルキーの対応表。"device_id:profile_key" 形式の文字列の
    # リスト (ROS 2 パラメータは辞書型を直接扱えないため文字列配列で表現する)。
    # 新規デバイス検出時、ここに載っているdevice_idはそのプロファイルを初期選択する。
    # 例: ["101:cubemars_ak_driver", "102:robomas_driver"]
    "device_profile_map": [],
    # firmware_config_dialog.py が最後に選択したテンプレートプロジェクトのパス
    # (実行環境ごとに異なるため config/ros2can.yaml には持たせない)。
    "firmware_template_dir": "",
}


def user_settings_path() -> str:
    base = os.environ.get("XDG_CONFIG_HOME", os.path.expanduser("~/.config"))
    directory = os.path.join(base, "ros2can")
    os.makedirs(directory, exist_ok=True)
    return os.path.join(directory, "settings.yaml")


def _load_bundled_yaml() -> Dict[str, Any]:
    try:
        share_dir = get_package_share_directory(PACKAGE_NAME)
        path = os.path.join(share_dir, 'config', 'ros2can.yaml')
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        return data.get(NODE_NAME, {}).get('ros__parameters', {}) or {}
    except Exception:
        return {}


def bundled_defaults() -> Dict[str, Any]:
    """config/ros2can.yaml (Git管理の一括設定ファイル) の内容。"""
    merged = dict(DEFAULT_SETTINGS)
    merged.update(_load_bundled_yaml())
    return merged


def load_user_overrides() -> Dict[str, Any]:
    """~/.config/ros2can/settings.yaml の内容をそのまま返す(未知のキーも含む)。

    save_user_settings() で一部のキーだけ更新して書き戻したい場合は、まずこれで
    現在の内容を読み込んでから対象キーだけ差し替えること(そうしないと、この
    呼び出し元が知らない他のキーを上書き時に消してしまう)。
    """
    path = user_settings_path()
    if not os.path.exists(path):
        return {}
    try:
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def load_settings() -> Dict[str, Any]:
    """bundled_defaults() にユーザーローカルの上書きを重ねた値。"""
    merged = bundled_defaults()
    merged.update(load_user_overrides())
    return merged


def save_user_settings(values: Dict[str, Any]) -> str:
    """GUIの「設定」ダイアログから呼ぶ。~/.config/ros2can/settings.yaml に書き込む
    (リポジトリ外のため Git の追跡対象にはならない)。"""
    path = user_settings_path()
    with open(path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(values, f, allow_unicode=True, sort_keys=False)
    return path


def reset_user_settings() -> None:
    path = user_settings_path()
    if os.path.exists(path):
        os.remove(path)


def parse_device_profile_map(raw: Any) -> Dict[int, str]:
    """device_profile_map設定 (["101:cubemars_ak_driver", ...] 形式) を
    {device_id: profile_key} の辞書に変換する。"id:key" の形になっていない要素は無視する。"""
    result: Dict[int, str] = {}
    for entry in raw or []:
        if not isinstance(entry, str) or ":" not in entry:
            continue
        id_part, _, key_part = entry.partition(":")
        id_part = id_part.strip()
        key_part = key_part.strip()
        if not id_part.isdigit() or not key_part:
            continue
        result[int(id_part)] = key_part
    return result
