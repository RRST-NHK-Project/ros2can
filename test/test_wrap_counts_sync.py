"""RosBackend._sync_wrap_counts (device_profiles.ChannelDef.wrap_counts の適用)の
ロジックテスト。rclpy Nodeの生成無しに検証できるよう、_sync_wrap_counts本体は
selfの属性を参照しない実装になっているため、RosBackendを生成せず未束縛メソッド
として直接呼び出す。"""

from types import SimpleNamespace

from ros2can.counter_unwrapper import CounterUnwrapper
from ros2can.device_profiles import SLOT_COUNT, make_cubemars_profile, make_can_host_profile
from ros2can.ros_backend import RosBackend, _COUNTS_PER_WRAP


def _make_channel(profile_key: str) -> SimpleNamespace:
    return SimpleNamespace(
        profile_key=profile_key,
        _wrap_counts_profile_key=None,
        unwrappers=[CounterUnwrapper(_COUNTS_PER_WRAP) for _ in range(SLOT_COUNT)],
    )


def test_cubemars_position_slots_get_65536_others_stay_default():
    profile = make_cubemars_profile()
    ch = _make_channel(profile.key)
    RosBackend._sync_wrap_counts(None, ch)

    for i in range(4):  # M1-M4 position
        assert ch.unwrappers[i]._counts_per_wrap == 65536
    for i in range(4, 8):  # M1-M4 speed (READOUTだがwrap_counts未指定)
        assert ch.unwrappers[i]._counts_per_wrap == _COUNTS_PER_WRAP
    assert ch._wrap_counts_profile_key == profile.key


def test_non_cubemars_profile_keeps_default_wrap_counts():
    profile = make_can_host_profile()
    ch = _make_channel(profile.key)
    RosBackend._sync_wrap_counts(None, ch)

    assert all(u._counts_per_wrap == _COUNTS_PER_WRAP for u in ch.unwrappers)


def test_sync_is_skipped_when_profile_key_unchanged():
    profile = make_cubemars_profile()
    ch = _make_channel(profile.key)
    RosBackend._sync_wrap_counts(None, ch)
    ch.unwrappers[0].set_counts_per_wrap(32768)  # 呼び出し元(GUI等)以外の変更を模す

    RosBackend._sync_wrap_counts(None, ch)  # profile_key不変なので再適用されないはず

    assert ch.unwrappers[0]._counts_per_wrap == 32768


def test_sync_reapplies_after_profile_switch():
    ch = _make_channel(make_can_host_profile().key)
    RosBackend._sync_wrap_counts(None, ch)
    assert ch.unwrappers[0]._counts_per_wrap == _COUNTS_PER_WRAP

    ch.profile_key = make_cubemars_profile().key
    RosBackend._sync_wrap_counts(None, ch)
    assert ch.unwrappers[0]._counts_per_wrap == 65536


def test_unknown_profile_key_falls_back_to_default():
    ch = _make_channel("__no_such_profile__")
    RosBackend._sync_wrap_counts(None, ch)
    assert all(u._counts_per_wrap == _COUNTS_PER_WRAP for u in ch.unwrappers)
