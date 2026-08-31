import os

import pytest

from ros2can.firmware_config import (
    FirmwareConfig, apply_config, diff_lines, generate_project, parse_config,
    validate_output_name,
)

SAMPLE = """\
#pragma once
#include <Arduino.h>

// IDの設定
#define DEVICE_ID 21

// CAN_IDは3桁形式
#define CAN_ID 101

// モードの設定，どれか一つをコメントアウト解除すること
// #define MODE_CAN
// #define MODE_CAN_HOST
// #define MODE_IO
// #define MODE_DEBUG
// #define MODE_CAN_MONITOR
#define MODE_ROBOMAS
// #define MODE_CUBEMARS

#define MULTI1 0
#define MULTI2 1
#define MULTI3 0

#define ENC1_MD 1
#define ENC2_MD 0

#define ROBOMAS_KP_VEL 0.8f
"""

FIRMWARE_CONFIG_PATH = os.path.join(
    os.path.dirname(__file__), "..", "firmware", "xiao-esp32-s3_can2io", "src", "config.hpp")


def test_parse_config_reads_expected_values():
    cfg = parse_config(SAMPLE)
    assert cfg.device_id == 21
    assert cfg.can_id == 101
    assert cfg.mode == "ROBOMAS"
    assert cfg.multi == [0, 1, 0]
    assert cfg.enc_md == [1, 0]
    assert cfg.available_modes == [
        "CAN", "CAN_HOST", "IO", "DEBUG", "CAN_MONITOR", "ROBOMAS", "CUBEMARS"]


def test_apply_config_changes_only_targeted_lines():
    cfg = parse_config(SAMPLE)
    cfg.device_id = 22
    cfg.can_id = 102
    cfg.multi = [1, 1, 1]
    cfg.enc_md = [0, 0]
    cfg.mode = "CAN_HOST"

    new_text = apply_config(SAMPLE, cfg)
    changes = diff_lines(SAMPLE, new_text)

    changed_line_contents = {new for _, _old, new in changes}
    assert "#define DEVICE_ID 22" in changed_line_contents
    assert "#define CAN_ID 102" in changed_line_contents
    assert "#define MULTI1 1" in changed_line_contents
    assert "// #define MODE_ROBOMAS" in changed_line_contents
    assert "#define MODE_CAN_HOST" in changed_line_contents

    # チューニング値の行は変化しない
    assert "#define ROBOMAS_KP_VEL 0.8f" in new_text
    # 変更した行以外は完全一致
    old_lines = SAMPLE.splitlines()
    new_lines = new_text.splitlines()
    changed_indices = {i - 1 for i, _o, _n in changes}
    for i, (o, n) in enumerate(zip(old_lines, new_lines)):
        if i not in changed_indices:
            assert o == n


def test_apply_config_reparses_back_to_same_values():
    cfg = parse_config(SAMPLE)
    cfg.mode = "IO"
    new_text = apply_config(SAMPLE, cfg)
    reparsed = parse_config(new_text)
    assert reparsed.mode == "IO"
    assert reparsed.available_modes == cfg.available_modes


def test_apply_config_rejects_unknown_mode():
    cfg = parse_config(SAMPLE)
    cfg.mode = "NOT_A_REAL_MODE"
    with pytest.raises(ValueError):
        apply_config(SAMPLE, cfg)


def test_parse_config_rejects_missing_macro():
    text = SAMPLE.replace("#define DEVICE_ID 21\n", "")
    with pytest.raises(ValueError):
        parse_config(text)


def test_no_op_apply_is_idempotent_on_real_firmware_file():
    if not os.path.exists(FIRMWARE_CONFIG_PATH):
        pytest.skip("firmware/xiao-esp32-s3_can2io (submodule) not checked out")
    with open(FIRMWARE_CONFIG_PATH, "r", encoding="utf-8") as f:
        original = f.read()
    cfg = parse_config(original)
    regenerated = apply_config(original, cfg)
    assert regenerated == original


# ---------------- validate_output_name / generate_project ----------------

def test_validate_output_name_strips_and_accepts_plain_name():
    assert validate_output_name("  node1_robomas  ") == "node1_robomas"


@pytest.mark.parametrize("bad_name", ["", "   ", ".", "..", "a/b", "a\\b"])
def test_validate_output_name_rejects_bad_names(bad_name):
    with pytest.raises(ValueError):
        validate_output_name(bad_name)


def _make_template_project(root) -> str:
    template_dir = os.path.join(str(root), "template")
    os.makedirs(os.path.join(template_dir, "src"))
    os.makedirs(os.path.join(template_dir, ".pio", "build"))
    with open(os.path.join(template_dir, "src", "config.hpp"), "w", encoding="utf-8") as f:
        f.write(SAMPLE)
    with open(os.path.join(template_dir, "platformio.ini"), "w", encoding="utf-8") as f:
        f.write("[env:esp32-s3-devkitc-1]\n")
    with open(os.path.join(template_dir, ".pio", "build", "stale.bin"), "w") as f:
        f.write("stale build artifact")
    return template_dir


def test_generate_project_copies_tree_and_patches_only_config(tmp_path):
    template_dir = _make_template_project(tmp_path)
    output_root = os.path.join(str(tmp_path), "generated_firmware")

    cfg = parse_config(SAMPLE)
    cfg.device_id = 22
    cfg.mode = "CAN_HOST"

    output_dir = generate_project(template_dir, output_root, "my_node", cfg)

    assert output_dir == os.path.join(os.path.abspath(output_root), "my_node")
    # platformio.ini はそのままコピーされる
    with open(os.path.join(output_dir, "platformio.ini"), encoding="utf-8") as f:
        assert f.read() == "[env:esp32-s3-devkitc-1]\n"
    # .pio ビルドキャッシュはコピーされない
    assert not os.path.exists(os.path.join(output_dir, ".pio"))
    # config.hpp だけがパラメータ通りに書き換わっている
    with open(os.path.join(output_dir, "src", "config.hpp"), encoding="utf-8") as f:
        new_config = f.read()
    assert "#define DEVICE_ID 22" in new_config
    assert "#define MODE_CAN_HOST" in new_config
    # テンプレート自体は変更されない
    with open(os.path.join(template_dir, "src", "config.hpp"), encoding="utf-8") as f:
        assert f.read() == SAMPLE


def test_generate_project_overwrites_previous_output_cleanly(tmp_path):
    template_dir = _make_template_project(tmp_path)
    output_root = os.path.join(str(tmp_path), "generated_firmware")
    cfg = parse_config(SAMPLE)

    output_dir = generate_project(template_dir, output_root, "my_node", cfg)
    # 前回生成物に手動でファイルを置いても、再生成で消える(残骸を残さない)
    stray_path = os.path.join(output_dir, "stray_leftover.txt")
    with open(stray_path, "w") as f:
        f.write("leftover")

    generate_project(template_dir, output_root, "my_node", cfg)
    assert not os.path.exists(stray_path)


def test_generate_project_rejects_path_escaping_name(tmp_path):
    template_dir = _make_template_project(tmp_path)
    output_root = os.path.join(str(tmp_path), "generated_firmware")
    cfg = parse_config(SAMPLE)
    with pytest.raises(ValueError):
        generate_project(template_dir, output_root, "../escape", cfg)


def test_generate_project_rejects_missing_config_hpp(tmp_path):
    template_dir = os.path.join(str(tmp_path), "empty_template")
    os.makedirs(template_dir)
    output_root = os.path.join(str(tmp_path), "generated_firmware")
    cfg = parse_config(SAMPLE)
    with pytest.raises(ValueError):
        generate_project(template_dir, output_root, "my_node", cfg)
