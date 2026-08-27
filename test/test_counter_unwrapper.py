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


def _to_twos_complement_int16(true_val: int) -> int:
    """CubeMars M{n} positionのような、CAN受信バイト列を直接int16化しただけの値を模す
    (標準的な2の補数、全域65536幅で単純にロールオーバーする)。"""
    v = true_val % 65536
    return v - 65536 if v >= 32768 else v


def test_set_counts_per_wrap_unwraps_standard_twos_complement_int16():
    # 既定(PCNT向け32768)のままだとCubeMarsのような標準int16ラップ(65536幅)を
    # 誤って復元することの確認、およびset_counts_per_wrap(65536)でこれが直ることの確認。
    # root_theta_jointの外部減速(96/7)相当で、AK40-10出力軸が継続回転するケースを模す。
    u_wrong = CounterUnwrapper(32768)
    u_fixed = CounterUnwrapper(65536)
    u_fixed.set_counts_per_wrap(65536)
    true_val = 0
    mismatched = False
    for _ in range(4000):
        true_val += 97  # 継続回転(片方向)
        raw = _to_twos_complement_int16(true_val)
        if u_wrong.update(raw) != true_val:
            mismatched = True
        assert u_fixed.update(raw) == true_val
    assert mismatched, "32768固定のままでは標準2の補数ラップで復元を誤るはず"


def test_set_counts_per_wrap_is_noop_when_unchanged():
    u = CounterUnwrapper(65536)
    u.update(100)
    u.set_counts_per_wrap(65536)
    assert u.update(200) == 200


def test_set_counts_per_wrap_preserves_accumulated_value():
    u = CounterUnwrapper(32768)
    u.update(1000)
    u.update(2000)
    before = u.update(3000)
    u.set_counts_per_wrap(65536)
    assert u.update(3100) == before + 100
