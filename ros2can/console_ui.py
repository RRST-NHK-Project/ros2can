"""--nogui 用のターミナルダッシュボード。

serial_bridge (graphical_ui.hpp) と同じ見た目のパネル表示を、PyQt5 GUIの代わりに
ros2can のデバイス一覧に対して行う。ANSIエスケープで画面を都度クリアして再描画する。
"""

from __future__ import annotations

import sys
import unicodedata

from .ros_backend import RosBackend, DeviceChannel, MODE_HARDWARE, MODE_SIMULATOR, MODE_TOPIC_CLIENT

_RESET = "\033[0m"
_FG_MUTED = "\033[38;5;245m"
_FG_TITLE = "\033[38;5;45m"
_FG_ACCENT = "\033[38;5;81m"
_FG_GOOD = "\033[38;5;48m"
_FG_WARN = "\033[38;5;214m"
_FG_BAD = "\033[38;5;203m"
_FG_TEXT = "\033[38;5;252m"

_PANEL_WIDTH = 100

_SPINNER_FRAMES = ("⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧", "⠇", "⠏")

_MODE_LABEL = {
    MODE_HARDWARE: "HW",
    MODE_SIMULATOR: "SIM",
    MODE_TOPIC_CLIENT: "TOPIC",
}


def _spinner(tick: int) -> str:
    return _SPINNER_FRAMES[tick % len(_SPINNER_FRAMES)]


def _display_width(text: str) -> int:
    return sum(2 if unicodedata.east_asian_width(c) in ("F", "W") else 1 for c in text)


def _fit(text: str, width: int) -> str:
    """表示幅(全角=2)基準で切り詰め/右パディングする。"""
    if width <= 0:
        return ""
    out = []
    total = 0
    for c in text:
        w = 2 if unicodedata.east_asian_width(c) in ("F", "W") else 1
        if total + w > width:
            break
        out.append(c)
        total += w
    pad = width - total
    return "".join(out) + (" " * pad if pad > 0 else "")


def _panel_border(color: str = _FG_MUTED) -> str:
    return f"{color}+{'-' * (_PANEL_WIDTH - 2)}+{_RESET}"


def _panel_line(content: str, content_color: str = _FG_TEXT, border_color: str = _FG_MUTED) -> str:
    body = _fit(content, _PANEL_WIDTH - 4)
    return f"{border_color}| {content_color}{body}{border_color} |{_RESET}"


def _device_color(ch: DeviceChannel) -> str:
    if not ch.connected:
        return _FG_BAD
    if ch.mode == MODE_TOPIC_CLIENT:
        return _FG_ACCENT
    if ch.rx_hz < 1.0:
        return _FG_WARN
    return _FG_GOOD


def _device_line(ch: DeviceChannel) -> str:
    status = "ON " if ch.connected else "OFF"
    mode = _MODE_LABEL.get(ch.mode, ch.mode)
    port = ch.port or "-"
    return (f"[{status}] id={ch.device_id:<3} mode={mode:<5} port={port:<14} "
            f"RX {ch.rx_hz:5.1f}Hz  rxN={ch.rx_frame_count:<8} txN={ch.tx_frame_count}")


class ConsoleUi:
    """serial_bridgeのgraphical_uiを模した、画面クリア再描画型のダッシュボード。"""

    def __init__(self, backend: RosBackend):
        self.backend = backend
        self._tick = 0
        self._started = False

    def start(self) -> None:
        sys.stdout.write("\033[?25l")  # カーソル非表示
        self._started = True

    def stop(self) -> None:
        if not self._started:
            return
        sys.stdout.write("\033[?25h")  # カーソル復帰
        sys.stdout.flush()
        self._started = False

    def render(self) -> None:
        self._tick += 1
        lines = [
            _panel_border(_FG_TITLE),
            _panel_line("ros2can --nogui live dashboard  ◉", _FG_TITLE, _FG_TITLE),
            _panel_line(
                f"devices: {len(self.backend.devices)}  |  direct_tx: forced ON for HW  |  "
                f"{_FG_GOOD}{_spinner(self._tick)} running{_RESET}",
                _FG_MUTED, _FG_TITLE),
            _panel_border(_FG_TITLE),
            _panel_line("DEVICES", _FG_ACCENT),
        ]

        if not self.backend.devices:
            lines.append(_panel_line("(no active devices)", _FG_MUTED))
        else:
            for device_id in sorted(self.backend.devices):
                ch = self.backend.devices[device_id]
                lines.append(_panel_line(_device_line(ch), _device_color(ch)))

        lines.append(_panel_border())
        lines.append(_panel_line("Ctrl+C to quit (sends zero command to all devices first)", _FG_MUTED))

        sys.stdout.write("\033[H\033[2J")
        sys.stdout.write("\n".join(lines) + "\n")
        sys.stdout.flush()
