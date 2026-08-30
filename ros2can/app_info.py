"""ロゴ/バージョン/GitHubリンクなど、アプリ全体の静的情報。

main_window.py のツールバー表示と about_dialog.py の両方から参照する。
"""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from typing import Optional

from ament_index_python.packages import get_package_share_directory
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap

PACKAGE_NAME = 'ros2can'
DESKTOP_ID = 'ros2can'


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
    """RRSTロゴ"""
    return _load_pixmap('logo.png')


def sub_logo_pixmap() -> Optional[QPixmap]:
    """創機立動ロゴ"""
    return _load_pixmap('soki_logo.png')


def app_icon_pixmap() -> Optional[QPixmap]:
    """アプリアイコン用: RRSTロゴの左端を正方形に切り出したもの。"""
    pixmap = logo_pixmap()
    if pixmap is None or pixmap.height() <= 0:
        return None
    size = pixmap.height()
    return pixmap.copy(0, 0, size, size)


def ensure_desktop_entry_installed() -> str:
    """GNOME(Wayland)のバー/Alt-Tab/ドックにアイコンを表示させるための下準備。

    Waylandではウィンドウ自身がアイコンを持てず、コンポジタは
    QGuiApplication.setDesktopFileName() で名乗る app_id と同名の
    .desktop ファイルの Icon= を見て表示アイコンを決める。実ファイルとして
    ~/.local/share/applications/ に用意しておかないと、素の setWindowIcon()
    だけでは汎用アイコン(歯車)にフォールバックしてしまう。
    戻り値は main.py の QApplication.setDesktopFileName() に渡す desktop file id。
    """
    icon_pixmap = app_icon_pixmap()
    if icon_pixmap is None:
        return DESKTOP_ID

    try:
        data_home = os.environ.get('XDG_DATA_HOME') or os.path.expanduser('~/.local/share')
        app_data_dir = os.path.join(data_home, DESKTOP_ID)
        apps_dir = os.path.join(data_home, 'applications')
        os.makedirs(app_data_dir, exist_ok=True)
        os.makedirs(apps_dir, exist_ok=True)

        icon_path = os.path.join(app_data_dir, 'app_icon.png')
        icon_pixmap.scaled(
            256, 256, Qt.KeepAspectRatio, Qt.SmoothTransformation,
        ).save(icon_path, 'PNG')

        desktop_path = os.path.join(apps_dir, f'{DESKTOP_ID}.desktop')
        with open(desktop_path, 'w') as f:
            f.write(
                "[Desktop Entry]\n"
                "Type=Application\n"
                "Name=ros2can\n"
                "Comment=RRST ros2can GUI\n"
                "Exec=ros2 run ros2can ros2can\n"
                f"Icon={icon_path}\n"
                f"StartupWMClass={DESKTOP_ID}\n"
                "Terminal=false\n"
                "Categories=Development;\n"
            )
    except Exception:
        pass
    return DESKTOP_ID


def _load_pixmap(filename: str) -> Optional[QPixmap]:
    try:
        share_dir = get_package_share_directory(PACKAGE_NAME)
        pixmap = QPixmap(os.path.join(share_dir, 'resources', filename))
        if not pixmap.isNull():
            return pixmap
    except Exception:
        pass
    return None


def stylesheet_text() -> str:
    """resources/style.qss (モダンダークテーマ) を読み込む。読めない場合は
    空文字を返す(Fusionスタイルのみで動作継続)。"""
    try:
        share_dir = get_package_share_directory(PACKAGE_NAME)
        path = os.path.join(share_dir, 'resources', 'style.qss')
        with open(path, 'r', encoding='utf-8') as f:
            return f.read()
    except Exception:
        return ''
