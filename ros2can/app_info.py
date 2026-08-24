"""ロゴ/バージョン/GitHubリンクなど、アプリ全体の静的情報。

main_window.py のツールバー表示と about_dialog.py の両方から参照する。
"""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from typing import Optional

from ament_index_python.packages import get_package_share_directory
from PyQt5.QtGui import QPixmap

PACKAGE_NAME = 'ros2can'


def _package_xml_root() -> ET.Element:
    share_dir = get_package_share_directory(PACKAGE_NAME)
    return ET.parse(os.path.join(share_dir, 'package.xml')).getroot()


def package_version() -> str:
    """git の short hash を優先する(setup.py がビルド時に resources/git_version.txt
    へ焼き込む。colcon build のたびに最新コミットへ自動で追従する)。取得できなければ
    package.xml の <version> にフォールバック。"""
    try:
        share_dir = get_package_share_directory(PACKAGE_NAME)
        with open(os.path.join(share_dir, 'resources', 'git_version.txt')) as f:
            git_hash = f.read().strip()
        if git_hash:
            return git_hash
    except Exception:
        pass
    try:
        version_el = _package_xml_root().find('version')
        if version_el is not None and version_el.text:
            return version_el.text.strip()
    except Exception:
        pass
    return '?'


def repo_url() -> Optional[str]:
    """package.xml の <url type="repository"> を返す(無ければ他のurlタグで代用)。"""
    try:
        urls = _package_xml_root().findall('url')
        for u in urls:
            if u.get('type') == 'repository' and u.text:
                return u.text.strip()
        for u in urls:
            if u.text:
                return u.text.strip()
    except Exception:
        pass
    return None


def logo_pixmap() -> Optional[QPixmap]:
    try:
        share_dir = get_package_share_directory(PACKAGE_NAME)
        pixmap = QPixmap(os.path.join(share_dir, 'resources', 'logo.png'))
        if not pixmap.isNull():
            return pixmap
    except Exception:
        pass
    return None
