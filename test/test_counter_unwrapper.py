"""encoder_angle/test/test_counter_unwrapper.cpp と同一シナリオのPython版テスト。"""

from ros2can.counter_unwrapper import CounterUnwrapper


def _to_pcnt_reset(true_val: int) -> int:
    """xiao-esp32-s3_can2io の実機PCNTを模したヘルパー。

    生カウントは +32767到達で0へ、-32768到達で0へ、それぞれ独立にリセットされる
    (実機観測で確認済み。counter_unwrapper.py 冒頭コメント参照)。
    """
    if true_val >= 0:
        return true_val % 32768
    return -((-true_val) % 32768)


def test_pcnt_reset_to_zero_positive_direction_dense_stream():
    u = CounterUnwrapper(32768)
    true_val = 0
    for _ in range(2000):
        true_val += 37
        raw = _to_pcnt_reset(true_val)
        assert u.update(raw) == true_val


def test_pcnt_reset_to_zero_negative_direction_dense_stream():
    # ユーザーが実機で確認した "-32767 -> 0" のシナリオに対応。
    u = CounterUnwrapper(32768)
    true_val = 0
    for _ in range(2000):
        true_val -= 41
        raw = _to_pcnt_reset(true_val)
        assert u.update(raw) == true_val


def test_pcnt_reset_to_zero_multiple_wraps_both_directions():
    u = CounterUnwrapper(32768)
    true_val = 0
    steps = (500, 500, 500, -300, -300, -300, -300, 900, 900, -700, -700, -700)
    for step in steps:
        for _ in range(40):
            true_val += step
            raw = _to_pcnt_reset(true_val)
            assert u.update(raw) == true_val


def test_real_world_large_gap_at_wrap_boundary_resolves_in_trend_direction():
    # 実機で観測された「半周期にわずかに満たない欠落」のケース(counts_per_wrap=65536時)。
    # 直近のトレンドは負方向で確立されている。
    u = CounterUnwrapper(65536)
    seq = (-32052, -32139, -32224, -32308, -32392, -32475, -32561, -32646, -32716, -33)
    last = 0
    prev = 0
    for raw in seq:
        prev = last
        last = u.update(raw)
    assert last - prev < 0, "半周期未満の大欠落はトレンド方向(負)に解決されるべき"


def test_idle_noise_does_not_cause_large_jump():
    u = CounterUnwrapper(65536)
    for raw in (-100, -110, -120, -130):
        u.update(raw)
    noisy = (-131, -130, -132, -131, -129, -131, -130, -131)
    prev = -130
    max_jump = 0
    for raw in noisy:
        result = u.update(raw)
        max_jump = max(max_jump, abs(result - prev))
        prev = result
    assert max_jump < 10, "静止中のノイズで1周期分の大ジャンプが発生してはいけない"


def test_reset_clears_state():
    u = CounterUnwrapper(65536)
    u.update(32000)
    u.update(-32000)
    u.reset()
    assert u.update(5) == 5
