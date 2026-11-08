"""RosBackend.spin_once() のテスト(rclpy/PyQt5が必要。無い環境ではスキップ)。

実際にトピックを流して、1回の呼び出しで受信済みの指令をすべて処理できること、
途絶時に0を送ることを確かめる。仮想デバイス(MODE_SIMULATOR)を使い、実機は不要。
"""

import os
import time

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("PyQt5")
pytest.importorskip("ament_index_python")

from std_msgs.msg import Int16MultiArray  # noqa: E402

from ros2can.hardware_manager import HardwareConfig  # noqa: E402
from ros2can.ros_backend import RosBackend  # noqa: E402

DEVICE_IDS = [11, 12, 13]


@pytest.fixture
def rig(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    # 動いている実機系のトピックと混ざらないよう別ドメインにする
    rclpy.init(domain_id=100 + os.getpid() % 100)
    yield
    rclpy.shutdown()


def _make_backend(watchdog_sec: float = 0.0) -> RosBackend:
    backend = RosBackend(hardware_config=HardwareConfig())
    backend.tx_watchdog_sec = watchdog_sec
    for device_id in DEVICE_IDS:
        backend.add_simulated_device(device_id)
    return backend


def _publish_until(backend, publishers, payload, condition, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for pub in publishers:
            pub.publish(payload)
        time.sleep(0.02)
        backend.spin_once()
        if condition():
            return True
    return False


def test_one_spin_processes_all_pending_devices(rig):
    backend = _make_backend()
    node = rclpy.create_node("spin_test_pub")
    publishers = [node.create_publisher(Int16MultiArray, f"serial_tx_{i}", 10) for i in DEVICE_IDS]
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and any(p.get_subscription_count() == 0 for p in publishers):
            time.sleep(0.01)

        # 全デバイスへ1通ずつ送り、1回の spin_once() だけで全部に反映されること。
        # (以前は1回で1コールバックしか処理せず、デバイスが増えるほど遅れていた)
        for attempt in range(1, 21):
            msg = Int16MultiArray()
            msg.data = [attempt] * 24
            for pub in publishers:
                pub.publish(msg)
            time.sleep(0.2)
            backend.spin_once()
            if all(backend.devices[i].tx_data[0] == attempt for i in DEVICE_IDS):
                return
        pytest.fail("1回の spin_once() で全デバイスの指令を処理できなかった")
    finally:
        node.destroy_node()
        backend.shutdown()


def test_watchdog_zeroes_after_commands_stop_and_resumes(rig):
    backend = _make_backend(watchdog_sec=0.1)
    node = rclpy.create_node("spin_test_pub")
    target = DEVICE_IDS[0]
    pub = node.create_publisher(Int16MultiArray, f"serial_tx_{target}", 10)
    msg = Int16MultiArray()
    msg.data = [100] * 24
    ch = backend.devices[target]
    try:
        assert _publish_until(backend, [pub], msg, lambda: ch.tx_data[0] == 100)

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not ch.tx_watchdog.tripped:
            time.sleep(0.01)
            backend.spin_once()
        assert ch.tx_watchdog.tripped
        assert ch.tx_data == [0] * 24
        # 指令を受けていないデバイスには作用しない
        assert not backend.devices[DEVICE_IDS[1]].tx_watchdog.tripped

        assert _publish_until(backend, [pub], msg, lambda: ch.tx_data[0] == 100)
        assert not ch.tx_watchdog.tripped
    finally:
        node.destroy_node()
        backend.shutdown()


def test_watchdog_leaves_values_alone_when_disabled(rig):
    backend = _make_backend(watchdog_sec=0.0)
    node = rclpy.create_node("spin_test_pub")
    target = DEVICE_IDS[0]
    pub = node.create_publisher(Int16MultiArray, f"serial_tx_{target}", 10)
    msg = Int16MultiArray()
    msg.data = [100] * 24
    ch = backend.devices[target]
    try:
        assert _publish_until(backend, [pub], msg, lambda: ch.tx_data[0] == 100)
        end = time.monotonic() + 0.3
        while time.monotonic() < end:
            time.sleep(0.01)
            backend.spin_once()
        assert ch.tx_data[0] == 100
    finally:
        node.destroy_node()
        backend.shutdown()
