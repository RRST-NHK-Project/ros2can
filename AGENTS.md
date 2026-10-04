# AGENTS.md

AIコーディングエージェント向けの作業ガイド。人間向けの説明は README.md と docs/ を参照。

## プロジェクト概要

PC(ROS 2 + PyQt5 GUI) ⇔ USBシリアル ⇔ CANホストマイコン ⇔ CANバス ⇔ 各ノードマイコン、という構成で
アクチュエータ/センサを扱うパッケージ。`serial_bridge` の後継。

**マイコンは基本的にXIAO ESP32-S3(`xiao-esp32-s3_can2io`)を想定する。** STM32(`b-g431-esc1_can2io`)は
ほとんど使っていないため、設計・修正・検証はESP32側を基準に行い、STM32側への追従は依頼された場合のみでよい
(STM32側のドキュメント・実装は古い可能性がある。例: ビットレートの記述が500kbpsのまま)。

| パス | 内容 | 言語/ビルド |
|:---|:---|:---|
| `ros2can/` | GUI本体 + ROS 2ノード | Python, ament_python |
| `firmware/xiao-esp32-s3_can2io/` | ホスト/ノード兼用ファーム(XIAO ESP32-S3)。**GUIのファーム生成のテンプレート** | C++, PlatformIO |
| `firmware/b-g431-esc1_can2io/` | FOCモータ用CANノード(STM32 B-G431B-ESC1, SimpleFOC)。**ほぼ使っていない** | C++, PlatformIO |
| `generated_firmware/<名前>/` | GUIの「ファーム生成」で作られた実機ごとのコピー(実際に書き込んだもの) | C++, PlatformIO |
| `docs/` | GitHub Pagesマニュアル(`index.md`=利用者向け, `quickstart.md`, `internals.md`=内部実装) | Markdown |
| `test/` | pytest ユニットテスト(Qt/ROS非依存部分のみ) | Python |
| `pptx/` | 講習資料(.pptx/.pdf)。編集しない | - |

## コマンド

```bash
# テスト(Qt/ROS不要、どこでも実行可)
python -m pytest test/

# ROS 2 ビルド/実行 (Ubuntu 24.04 + ROS 2 Jazzy 前提、ワークスペースルートで)
colcon build --packages-select ros2can && source install/setup.bash
ros2 launch ros2can ros2can.launch.py

# ファームのビルド(各PlatformIOプロジェクトのディレクトリで)
pio run
```

開発機がWindowsの場合、ROS/colcon/実機書き込みは実行できないことが多い。その場合は pytest と
`pio run`(ビルドのみ)で確認できる範囲を確認し、実機未検証であることを明示する。

## 重要な設計上の前提(壊しやすいポイント)

- **プロトコルはPython側とファーム側で独立実装されている(共通ライブラリなし)。** 片方を変えたら必ずもう片方も合わせる。
  - シリアル: `[0xAA][DEVICE_ID][LENGTH][int16 x 24, BE][XOR checksum]`、52バイト固定。
    PC側 `ros2can/frame_codec.py` ⇔ ファーム側 `src/serial_task.cpp` / `src/frame_data.hpp`。
  - CAN(ノード分配): 指令 `0x100 + node*16 + chunk`、帰還 `0x180 + node*16 + chunk`、DLC=8、int16 x 4/フレーム(BE)。
    `src/can_task.cpp` が基準実装(`b-g431-esc1_can2io` にも同方式の実装があるが、上記の通り追従は任意)。
  - スロットの意味(どのスロットが何か)は ファームの `frame_data.hpp`/`pin_ctrl_task.cpp` と
    GUIの `ros2can/device_profiles.py` で二重に定義されている。
- **`config.hpp` はGUIが正規表現で1行ずつ書き換える**(`ros2can/firmware_config.py`)。
  `#define NAME value` の行形式を崩したり、GUI対象マクロ(`_SIMPLE_MACROS`, `SERVO_MACROS`,
  `ADVANCED_MACROS`)を `#if` 分岐内で複数定義したりすると、生成機能が壊れる/全分岐が同値に書き換わる。
  config.hpp を変更したら `test/test_firmware_config.py` を実行する。
- **`BOARD_VARIANT`(SOKI/MES/SS)でピン配置・1ノードあたりスロット数がコンパイル時に変わる。**
  `CAN_SLOTS_PER_NODE`/`CAN_NODE_COUNT` はバリアント依存。異なるバリアントを同一CANバスに混在させる構成は現状未対応。
- **`CAN_NODE_COUNT` は実接続ノード数に合わせる。** 存在しないノード宛に送るとACKエラーでホストがBus-Offになる。
- **テンプレートの修正は `generated_firmware/` に自動では反映されない。** 生成物は書き込み時点のコピーで、
  `config.hpp` 以外にも差分がありうる。テンプレートのバグを直したときは、どの生成物に反映すべきかユーザーに確認する。
- MODE_ROBOMAS/MODE_CUBEMARS と同一バスに同居させる都合でCANは1Mbps。ロボマス側ID帯(`0x1FE`, `0x200-0x208`)と衝突させないこと。
- `resources/git_version.txt` は `setup.py` がビルド毎に生成する(gitignore済み)。手で編集・コミットしない。

## 安全

このコードは実機のモータ・サーボ・ソレノイドを動かす。スロット割り当て・スケール・クランプ
(`MD_PWM_MAX`、電流上限、サーボ角度範囲等)・フェイルセーフ(タイムアウト時の停止)に関わる変更は、
意図しない動作につながりうる点をユーザーに明示する。チューニング済みの値(ROBOMASのPIDゲイン、
CubeMarsのMITレンジ、電流制限等)は依頼なしに変更しない。

## コーディング規約

- コメント・docstring・ドキュメントは**日本語**。既存コードは「なぜそうしたか」(過去の不具合・実測値・日付)を
  コメントに残す文化があるので、仕様上の理由がある変更は同様に理由を書く。
- Python: 型ヒントあり、`from __future__ import annotations`。Qt非依存のロジックは別モジュールに分けてテスト可能にする
  (例: `firmware_flash.py` ⇔ `firmware_dialog.py`)。
- ファーム: Arduino + FreeRTOS。タスク間共有データ(`Tx_16Data`/`Rx_16Data` 等)はクリティカルセクションでスナップショットしてから使う。
- 挙動やプロトコルを変えたら `docs/index.md`(利用者向け)・`docs/internals.md`(内部実装)・該当ファームの `README.md` も更新する。
- コミットメッセージは日本語の短い1行(例: `ノード側でサーボが動かない不具合を修正`)。
