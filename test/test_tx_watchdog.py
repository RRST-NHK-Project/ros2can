"""serial_tx_[ID] 途絶監視のテスト(tx_watchdog.py)。"""

import math

import pytest

from ros2can.tx_watchdog import TxWatchdog, validate_timeout_sec


def test_does_not_trip_before_first_command():
    # 指令トピックを使わないデバイス・GUI手動操作中の値を0で上書きしない
    assert TxWatchdog().poll(1000.0, 0.2) is False


def test_trips_once_after_timeout():
    wd = TxWatchdog()
    wd.note_command(10.0)
    assert wd.poll(10.19, 0.2) is False
    assert wd.tripped is False
    assert wd.poll(10.21, 0.2) is True
    assert wd.tripped is True
    assert wd.poll(11.0, 0.2) is False  # 1回だけ(次の指令まで再度は返さない)


def test_command_extends_deadline():
    wd = TxWatchdog()
    wd.note_command(10.0)
    wd.note_command(10.15)
    assert wd.poll(10.3, 0.2) is False


def test_recovery_is_reported_and_watch_resumes():
    wd = TxWatchdog()
    assert wd.note_command(10.0) is False
    assert wd.poll(11.0, 0.2) is True
    assert wd.note_command(12.0) is True
    assert wd.tripped is False
    assert wd.poll(12.3, 0.2) is True


def test_zero_timeout_disables():
    wd = TxWatchdog()
    wd.note_command(10.0)
    assert wd.poll(1000.0, 0.0) is False


def test_disarm_stops_watching():
    wd = TxWatchdog()
    wd.note_command(10.0)
    wd.disarm()
    assert wd.poll(1000.0, 0.2) is False


@pytest.mark.parametrize("value", [-0.1, math.nan, math.inf])
def test_invalid_timeout_is_rejected(value):
    with pytest.raises(ValueError):
        validate_timeout_sec(value)


@pytest.mark.parametrize("value", [0.0, 0.2])
def test_valid_timeout_is_accepted(value):
    assert validate_timeout_sec(value) == value
