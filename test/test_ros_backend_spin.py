"""RosBackend.spin_once() のテスト(rclpy/PyQt5が必要。無い環境ではスキップ)。

実際にトピックを流して、1回の呼び出しで受信済みの指令をすべて処理できることを
確かめる。仮想デバイス(MODE_SIMULATOR)を使い、実機は不要。
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


def _make_backend() -> RosBackend:
    backend = RosBackend(hardware_config=HardwareConfig())
    for device_id in DEVICE_IDS:
        backend.add_simulated_device(device_id)
    return backend


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
