"""MODE_DAMIAO / MODE_ROBSTRIDE まわりの確認(2026-11-06 PRレビュー指摘で追加)。

・テンプレートのconfig.hppは既定(CAN_HOST / DEVICE_ID 101)のままであること
  (実機ごとの設定は generated_firmware/ 側に置く)
・GUIのモード一覧・プロファイルに2つのモードが入っていること
・プロファイルのスロット番号が24個ちょうど・重複なしであること
"""
import os

from ros2can import device_profiles as dp
from ros2can.firmware_config import CONFIG_HPP_REL_PATH, parse_config

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_TEMPLATE_DIR = os.path.join(_ROOT, "firmware", "xiao-esp32-s3_can2io")


def _read_config(project_dir):
    with open(os.path.join(project_dir, CONFIG_HPP_REL_PATH), encoding="utf-8") as f:
        return parse_config(f.read())


def test_template_config_is_default_can_host():
    cfg = _read_config(_TEMPLATE_DIR)
    assert cfg.mode == "CAN_HOST"
    assert cfg.device_id == 101
    assert "DAMIAO" in cfg.available_modes
    assert "ROBSTRIDE" in cfg.available_modes


def test_generated_projects_for_damiao_and_robstride():
    root = os.path.join(_ROOT, "generated_firmware")
    for name, mode, device_id in [("soki_damiao_201", "DAMIAO", 201),
                                  ("soki_robstride_202", "ROBSTRIDE", 202)]:
        cfg = _read_config(os.path.join(root, name))
        assert cfg.mode == mode
        assert cfg.device_id == device_id


def test_mode_labels_cover_new_modes():
    # firmware_dialog はPyQt5を読み込むため、ここではソースの文字列で確認する
    with open(os.path.join(_ROOT, "ros2can", "firmware_dialog.py"), encoding="utf-8") as f:
        text = f.read()
    assert '"DAMIAO":' in text
    assert '"ROBSTRIDE":' in text


def test_new_profiles_have_24_unique_slots():
    for key in ("damiao_driver", "robstride_driver"):
        profile = dp.BUILTIN_PROFILES[key]
        for channels in (profile.tx, profile.rx):
            assert [c.index for c in channels] == list(range(dp.SLOT_COUNT))


def test_damiao_profile_knows_latched_state():
    profile = dp.BUILTIN_PROFILES["damiao_driver"]
    state = profile.rx_by_index()[0]
    assert dict(state.options)[15].startswith("ESP32が停止中")
