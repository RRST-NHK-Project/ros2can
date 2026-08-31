import os

from ros2can.firmware_flash import (
    build_upload_command, find_pio_executable, list_generated_projects,
)


def test_build_upload_command():
    cmd = build_upload_command("/usr/bin/pio", "/tmp/proj", "/dev/ttyUSB0")
    assert cmd == [
        "/usr/bin/pio", "run", "-d", "/tmp/proj", "-t", "upload",
        "--upload-port", "/dev/ttyUSB0",
    ]


def test_list_generated_projects_filters_and_sorts(tmp_path):
    root = tmp_path / "generated_firmware"
    root.mkdir()

    for name in ["node_b", "node_a"]:
        d = root / name
        d.mkdir()
        (d / "platformio.ini").write_text("[env:x]\n")

    # platformio.ini が無いディレクトリは無視される
    (root / "not_a_project").mkdir()
    # ファイルはディレクトリではないので無視される
    (root / "stray_file.txt").write_text("x")

    assert list_generated_projects(str(root)) == ["node_a", "node_b"]


def test_list_generated_projects_missing_root_returns_empty(tmp_path):
    assert list_generated_projects(str(tmp_path / "does_not_exist")) == []


def test_find_pio_executable_uses_which(monkeypatch):
    monkeypatch.setattr("ros2can.firmware_flash.shutil.which", lambda name: "/usr/bin/pio")
    assert find_pio_executable() == "/usr/bin/pio"


def test_find_pio_executable_falls_back_to_platformio_penv(monkeypatch, tmp_path):
    fallback = tmp_path / ".platformio" / "penv" / "bin" / "pio"
    fallback.parent.mkdir(parents=True)
    fallback.write_text("#!/bin/sh\n")
    fallback.chmod(0o755)

    monkeypatch.setattr("ros2can.firmware_flash.shutil.which", lambda name: None)
    monkeypatch.setattr("ros2can.firmware_flash._PIO_FALLBACK_PATH", str(fallback))

    assert find_pio_executable() == str(fallback)


def test_find_pio_executable_returns_none_when_not_found(monkeypatch, tmp_path):
    monkeypatch.setattr("ros2can.firmware_flash.shutil.which", lambda name: None)
    monkeypatch.setattr(
        "ros2can.firmware_flash._PIO_FALLBACK_PATH", str(tmp_path / "nonexistent_pio"))

    assert find_pio_executable() is None
