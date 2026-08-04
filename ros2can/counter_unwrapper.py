"""ラップアラウンドする整数カウンタを連続値へ復元するユーティリティ。

serial_bridge/ros2can の各 int16 スロット(エンコーダカウンタ等)は±32768で
折り返すため、折り返し前後の生値の差分を監視し、半周期を超える跳躍をラップと
みなして補正することで連続値に復元する。

[esc_ctrl/include/esc_ctrl/angle_unwrapper.hpp と同一アルゴリズムがベース。
ROSトピック経由(GUIスレッドの詰まり等でサンプルが疎になりうる)で下流にunwrapを
任せると半周期に近い欠落でラップ誤判定が起きるため、フレームを受信した直後
(このモジュールの呼び出し元である HardwareManager.service() のループ内、
サンプルが密な場所)でunwrapし、既に連続値になった状態で配信する。]

欠落量がちょうど半周期に近い場合、「絶対値最小」の判定だけでは原理的にどちらが
正しいか区別できない(実機検証で確認済み: ラップ境界付近で稀に大量のサンプルが
欠落する瞬間があり、絶対値最小ヒューリスティックが誤った側を選ぶことがある)。

これを緩和するため、差分の絶対値が半周期に近い「本当に曖昧な場合」に限り、
直近の移動方向(符号)と一致する候補を優先する。差分が明らかに小さい通常時
(静止中のノイズ程度の微小変動を含む)は、この判定を一切使わず素直に絶対値最小を
採用する(曖昧でない場合にまでトレンド判定を適用すると、静止中の符号反転する
微小ノイズを誤って1周期分の大ジャンプとして扱ってしまうため)。
"""

from __future__ import annotations

from typing import Optional


class CounterUnwrapper:
    def __init__(
        self,
        counts_per_wrap: int = 65536,
        ambiguous_margin_ratio: float = 0.1,
        trend_noise_floor: int = 4,
    ):
        self._counts_per_wrap = counts_per_wrap
        self._half_wrap = counts_per_wrap / 2.0
        # 差分の絶対値がこの値以上の場合のみ「曖昧」とみなしトレンド判定を使う
        self._ambiguous_threshold = self._half_wrap * (1.0 - ambiguous_margin_ratio)
        # トレンド(直近の移動方向)を更新するのに必要な最小変化量。これ未満の
        # 変化(静止中のノイズ等)ではトレンドを更新せず、直近の「本物の」移動方向を
        # 保持し続ける。
        self._trend_noise_floor = trend_noise_floor
        self._prev_raw: Optional[int] = None
        self._unwrapped = 0
        self._trend_sign = 0  # 直近の移動方向: -1, 0(未確定), +1

    def reset(self) -> None:
        self._prev_raw = None
        self._unwrapped = 0
        self._trend_sign = 0

    def update(self, raw: int) -> int:
        if self._prev_raw is None:
            self._unwrapped = raw
            self._prev_raw = raw
            return self._unwrapped

        naive_delta = raw - self._prev_raw
        # 法 counts_per_wrap で数学的に等価な3候補
        candidates = sorted(
            (
                naive_delta,
                naive_delta - self._counts_per_wrap,
                naive_delta + self._counts_per_wrap,
            ),
            key=abs,
        )
        best, second = candidates[0], candidates[1]

        if abs(best) >= self._ambiguous_threshold and self._trend_sign != 0:
            same_sign = [d for d in (best, second) if (d > 0) == (self._trend_sign > 0)]
            delta = same_sign[0] if same_sign else best
        else:
            delta = best

        self._unwrapped += delta
        self._prev_raw = raw
        if abs(delta) >= self._trend_noise_floor:
            self._trend_sign = 1 if delta > 0 else -1
        return self._unwrapped
