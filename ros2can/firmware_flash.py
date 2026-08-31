"""generated_firmware/ 配下のPlatformIOプロジェクトを実機へ書き込むためのロジック層。

サブプロセス起動そのもの(実行・出力ストリーミング)は firmware_dialog.py の
QThreadワーカーで行う。ここには Qt非依存で単体テスト可能な部分だけを置く。
"""

from __future__ import annotations

import os
import shutil
from typing import List, Optional

_PIO_FALLBACK_PATH = os.path.expanduser(os.path.join("~", ".platformio", "penv", "bin", "pio"))


def find_pio_executable() -> Optional[str]:
    """PlatformIO CLI (`pio`) の実行ファイルパスを探す。

    通常のPATH検索に加え、PlatformIO公式インストーラの既定インストール先
    (~/.platformio/penv/bin/pio) もフォールバックとして確認する。GUIが
    デスクトップエントリ等、フルのシェルPATHを引き継がない環境から起動された
    場合の対策。見つからなければNone。
    """
    found = shutil.which("pio")
    if found:
        return found
    if os.path.isfile(_PIO_FALLBACK_PATH) and os.access(_PIO_FALLBACK_PATH, os.X_OK):
        return _PIO_FALLBACK_PATH
    return None


def build_upload_command(pio_exe: str, project_dir: str, port: str) -> List[str]:
    return [pio_exe, "run", "-d", project_dir, "-t", "upload", "--upload-port", port]


def list_generated_projects(output_root: str) -> List[str]:
    """output_root直下の、platformio.iniを含むディレクトリ名だけを名前順で返す。"""
    if not os.path.isdir(output_root):
        return []
    names = []
    for entry in os.listdir(output_root):
        project_dir = os.path.join(output_root, entry)
        if os.path.isfile(os.path.join(project_dir, "platformio.ini")):
            names.append(entry)
    return sorted(names)
