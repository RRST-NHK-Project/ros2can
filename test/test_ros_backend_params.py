"""RosBackend の文字列配列パラメータのテスト(rclpy/PyQt5が必要。無い環境ではスキップ)。

launch は config/ros2can.yaml を --params-file で渡す。yaml の `[]` や、既定値が空の
パラメータへ文字列の配列を渡すと起動時に例外で落ちていた。
"""

import os

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("PyQt5")
pytest.importorskip("ament_index_python")

from ros2can.ros_backend import RosBackend  # noqa: E402
from ros2can.settings_store import NODE_NAME  # noqa: E402

BUNDLED_YAML = os.path.join(os.path.dirname(__file__), "..", "config", "ros2can.yaml")


@pytest.fixture
def make_backend(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    created = []

    def _make(*ros_args):
        args = ["ros2can", "--ros-args", "-r", f"__node:={NODE_NAME}", *ros_args]
        rclpy.init(args=args, domain_id=100 + os.getpid() % 100)
        backend = RosBackend()
        created.append(backend)
        return backend

    yield _make
    for backend in created:
        backend.shutdown()
    if rclpy.ok():
        rclpy.shutdown()


def test_no_parameters_uses_defaults(make_backend):
    backend = make_backend()
    assert backend.hardware.config.excluded_ports == set()
    assert backend.device_profile_map == {}


def test_bundled_yaml_with_empty_lists_starts(make_backend):
    backend = make_backend("--params-file", BUNDLED_YAML)
    assert backend.hardware.config.excluded_ports == set()
    assert backend.device_profile_map == {}


def test_values_from_params_file_are_applied(make_backend, tmp_path):
    params = tmp_path / "params.yaml"
    params.write_text(
        f"{NODE_NAME}:\n"
        "  ros__parameters:\n"
        '    excluded_ports: ["/dev/ttyUSB9"]\n'
        '    device_profile_map: ["201:damiao_driver"]\n',
        encoding="utf-8")
    backend = make_backend("--params-file", str(params))
    assert backend.hardware.config.excluded_ports == {"/dev/ttyUSB9"}
    assert backend.device_profile_map == {201: "damiao_driver"}


def test_empty_list_in_params_file_keeps_user_setting(make_backend, tmp_path):
    settings_dir = tmp_path / "ros2can"
    settings_dir.mkdir()
    (settings_dir / "settings.yaml").write_text(
        'excluded_ports: ["/dev/ttyUSB8"]\n', encoding="utf-8")
    backend = make_backend("--params-file", BUNDLED_YAML)
    assert backend.hardware.config.excluded_ports == {"/dev/ttyUSB8"}
