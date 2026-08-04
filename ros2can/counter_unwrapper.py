"""ラップアラウンドする整数カウンタを連続値へ復元するユーティリティ。

serial_bridge/ros2can の各 int16 スロット(エンコーダカウンタ等)は±32768で
折り返すため、折り返し前後の生値の差分を監視し、半周期を超える跳躍をラップと
みなして補正することで連続値に復元する。

[esc_ctrl/include/esc_ctrl/angle_unwrapper.hpp と同一アルゴリズム。ROSトピック
経由(GUIスレッドの詰まり等でサンプルが疎になりうる)で下流にunwrapを任せると
半周期に近い欠落でラップ誤判定が起きるため、フレームを受信した直後(このモジュール
の呼び出し元である HardwareManager.service() のループ内、サンプルが密な場所)で
unwrapし、既に連続値になった状態で配信する。]

1回の update() 呼び出しの間に実際に変化するカウント数が counts_per_wrap の
半分未満であることを前提とする。
"""

from __future__ import annotations

from typing import Optional


class CounterUnwrapper:
    def __init__(self, counts_per_wrap: int = 65536):
        self._counts_per_wrap = counts_per_wrap
        self._half_wrap = counts_per_wrap // 2
        self._prev_raw: Optional[int] = None
        self._unwrapped = 0

    def reset(self) -> None:
        self._prev_raw = None
        self._unwrapped = 0

    def update(self, raw: int) -> int:
        if self._prev_raw is None:
            self._unwrapped = raw
        else:
            delta = raw - self._prev_raw
            if delta > self._half_wrap:
                delta -= self._counts_per_wrap
            elif delta < -self._half_wrap:
                delta += self._counts_per_wrap
            self._unwrapped += delta
        self._prev_raw = raw
        return self._unwrapped
