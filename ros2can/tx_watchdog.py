"""serial_tx_[ID] の途絶を見張るウォッチドッグ(Qt/ROS非依存、test/test_tx_watchdog.py)。

--nogui は direct_tx が常時ONで、最後の tx_data を20Hzで送り直す。そのため指令を
出していた外部ノードだけが落ちると、最後の指令が送られ続けてモータが止まらない。
ファーム側の途絶停止(MODE_DAMIAO等)も、PCからのフレームが届き続ける限り働かない。
これをPC側で塞ぐのが目的(2026-11-07、BRの自律走行向け依頼で追加)。

一度も指令を受けていないデバイスは見張らない。指令トピックを使わないデバイス
(入力専用のCANホスト等)や、GUIで手動操作中の値を0へ上書きしないため。
"""

from __future__ import annotations

import math
from typing import Optional


def validate_timeout_sec(value: float) -> float:
    """tx_watchdog_sec を検証する。0で無効。

    不正値を黙って「無効」に倒すと、安全装置が切れたまま走り出してしまうので
    起動時に例外で止める。
    """
    timeout = float(value)
    if not math.isfinite(timeout) or timeout < 0.0:
        raise ValueError(f"tx_watchdog_sec は0以上の有限値にしてください: {value!r}")
    return timeout


class TxWatchdog:
    def __init__(self) -> None:
        self._last_command_time: Optional[float] = None
        self.tripped = False

    def note_command(self, now: float) -> bool:
        """指令を受けたときに呼ぶ。途絶からの復帰ならTrueを返す。"""
        recovered = self.tripped
        self.tripped = False
        self._last_command_time = now
        return recovered

    def disarm(self) -> None:
        """見張りをやめる(次の指令まで)。トピック通過をOFFにした場合などに使う。"""
        self._last_command_time = None
        self.tripped = False

    def poll(self, now: float, timeout_sec: float) -> bool:
        """途絶を検出した瞬間だけTrueを返す(次の指令が来るまで再度は返さない)。"""
        if timeout_sec <= 0.0 or self._last_command_time is None:
            return False
        if now - self._last_command_time < timeout_sec:
            return False
        self._last_command_time = None
        self.tripped = True
        return True
