"""ラップアラウンドする int16 カウンタを連続値へ復元するユーティリティ。

[encoder_angle/include/encoder_angle/counter_unwrapper.hpp と同一アルゴリズム]

ROSトピック経由(GUIスレッドの詰まり等でサンプルが疎になりうる)で下流に
unwrapを任せると半周期に近い欠落でラップ誤判定が起きるため、フレームを
受信した直後(このモジュールの呼び出し元である RosBackend._on_hardware_frame /
service_simulators のループ内、サンプルが密な場所)でunwrapし、既に連続値に
なった状態で serial_rx_[ID]_unwrapped として配信する。

[ESP32 PCNT の仕様確認結果 (実機で確認済み)]
xiao-esp32-s3_can2io の PCNT は counter_h_lim=32767 / counter_l_lim=-32768 を
設定しているが pcnt_event_enable() を呼んでいない。ESP-IDF の
pcnt_unit_config() の doxygenコメントには「h_lim/l_lim/zeroの3イベントを
無効化する」と書かれているため、当初は「イベント無効 = 上限/下限到達時の
自動0クリアも無効になり、生値は素の2の補数(全域65536幅)でオーバーフロー
する」と推測したが、これは誤りだった。実機でエンコーダーを負方向へ回転させ
serial_rx_[ID] を直接観測したところ、生値は -32767 に達すると(2の補数の
-32768を経由せず)そのまま0へ戻り、0から再び負方向へカウントが続くことを
確認した。つまり「イベント(割り込み通知)の無効化」と「h_lim/l_lim到達時の
0クリア動作」は別物で、後者はイベントの有効/無効に関わらず常時作動している。
h_lim=32767, l_lim=-32768 から、このリセット幅は正負どちらの方向も32768
(= h_lim+1 = -l_lim) で対称になる(単純な2の補数オーバーフロー=全域65536幅の
1本のリング、とは異なる)。counts_per_wrap の既定値はこれに合わせて32768に
している。

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
        counts_per_wrap: int = 32768,
        ambiguous_margin_ratio: float = 0.1,
        trend_noise_floor: int = 4,
    ) -> None:
        self._counts_per_wrap = counts_per_wrap
        self._half_wrap = counts_per_wrap / 2.0
        self._ambiguous_threshold = self._half_wrap * (1.0 - ambiguous_margin_ratio)
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
        candidates = sorted(
            (
                naive_delta,
                naive_delta - self._counts_per_wrap,
                naive_delta + self._counts_per_wrap,
            ),
            key=abs,
        )
        best, second = candidates[0], candidates[1]

        delta = best
        if abs(best) >= self._ambiguous_threshold and self._trend_sign != 0:
            best_matches = (best > 0) == (self._trend_sign > 0)
            second_matches = (second > 0) == (self._trend_sign > 0)
            if not best_matches and second_matches:
                delta = second

        self._unwrapped += delta
        self._prev_raw = raw
        if abs(delta) >= self._trend_noise_floor:
            self._trend_sign = 1 if delta > 0 else -1
        return self._unwrapped
