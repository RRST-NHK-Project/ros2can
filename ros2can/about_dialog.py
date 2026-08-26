"""ロゴ・バージョン・GitHubリンクを表示するAboutダイアログ。"""

from __future__ import annotations

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QDialog, QVBoxLayout, QLabel, QDialogButtonBox

from .app_info import logo_pixmap, package_version, repo_url

SERIAL_BRIDGE_URL = "https://github.com/RRST-NHK-Project/serial_bridge"


class AboutDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("ros2can について")

        layout = QVBoxLayout(self)

        pixmap = logo_pixmap()
        if pixmap is not None:
            logo_label = QLabel()
            logo_label.setPixmap(pixmap.scaledToHeight(64, Qt.SmoothTransformation))
            logo_label.setAlignment(Qt.AlignCenter)
            layout.addWidget(logo_label)

        version_label = QLabel(f"ros2can v{package_version()}")
        version_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(version_label)

        powered_by_label = QLabel(
            f'Powered By <a href="{SERIAL_BRIDGE_URL}">serial_bridge</a>')
        powered_by_label.setOpenExternalLinks(True)
        powered_by_label.setAlignment(Qt.AlignCenter)
        powered_by_label.setStyleSheet("color: #888;")
        powered_by_label.setToolTip("ros2can は serial_bridge をベースに開発しています。")
        layout.addWidget(powered_by_label)

        url = repo_url()
        if url:
            link_label = QLabel(f'<a href="{url}">{url}</a>')
            link_label.setOpenExternalLinks(True)
            link_label.setAlignment(Qt.AlignCenter)
            layout.addWidget(link_label)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok)
        buttons.accepted.connect(self.accept)
        layout.addWidget(buttons)
