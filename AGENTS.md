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

## 追加作業のチェックリスト

PC側とファーム側で二重定義している箇所が多く、片方だけ直すとコンパイルは通るのに実機で
スロットがずれる。下記の該当項目をすべて確認してから完了とする。

### 新しい基板(BOARD_VARIANT)を追加する

1. `config.hpp`: `#define BOARD_XXX <数値>` を1行で追加する(GUIは `_BOARD_TOKEN_RE` でこの行を
   拾って選択肢にする)。`CAN_NODE_COUNT`/`CAN_SLOTS_PER_NODE` の `#if BOARD_VARIANT` 分岐に節を足し、
   `CAN_NODE_COUNT * CAN_SLOTS_PER_NODE <= 24` を守る。
2. `defs.hpp`: ピン定義の `#elif BOARD_VARIANT == BOARD_XXX` 節(無いと `#error "unknown board"`)。
3. `pin_ctrl_init.cpp`/`pin_ctrl_task.cpp`: 初期化とスロット⇔ピンの対応。**出力ピンは起動時に安全側
   (OFF/LOW、モータ停止)へ明示的に初期化する**(BOARD_SSのTR1-4と同様)。
4. `frame_data.hpp`: `CanIoRxData`/`CanIoTxData` のスロット表コメント。`main.cpp`: MODE_IO非対応なら
   既存の `#error` ガードに含める。
5. `device_profiles.py`: `_append_xxx_node_channels()` と `make_xxx_host_profile()` を作り、
   `_build_builtin_profiles()` に登録する(スロット順・スケールはファームと一致させる)。
6. `firmware_dialog.py`: `_BOARD_LABELS` と、基板名を列挙しているヘルプ文。基板専用マクロを足すなら
   `firmware_config.py` の `_SIMPLE_MACROS`・`FirmwareConfig`・ダイアログの入力欄・テストも追加する。
7. 全バリアントで `pio run` が通ること、`python -m pytest test/` が通ることを確認する。
8. `docs/index.md`(BOARD_VARIANT表)、`docs/internals.md`(3.2/3.5)、ファームREADME §3a を更新する。

### 新しいモード(MODE_*)・機能を追加する

- `config.hpp` に `// #define MODE_XXX` 形式で追加する(GUIは `_MODE_RE` で拾う)。
  `main.cpp` のモード分岐と、「MODEがちょうど1つ」の `#error` チェック(`defined(...)` の合計)を両方更新する。
- `firmware_dialog.py` の `_MODE_LABELS` と、必要なら `device_profiles.py` に専用プロファイルを追加する。
- 新しいCAN IDを使うなら、ノード分配(`0x100`/`0x180`帯)とロボマス(`0x1FE`, `0x200-0x208`)の両方と
  衝突しないことを確認する。

### 新しいファーム(別MCU・別PlatformIOプロジェクト)を追加する

- PCは「115200baudで開き、正しいフレームを受信できたらそのDEVICE_IDで識別する」
  (`serial_link.probe_port()`)。**ファームは指令が無くても定期的に帰還フレームを送り続ける**必要がある。
- フレーム形式・CAN ID・1Mbps・「全スロット0 = 安全な既定値」の規約を守る。
- GUIのファーム生成は `TEMPLATE_RELATIVE_DIR`(xiaoテンプレート固定)前提。新ファームを生成対象にするなら
  `firmware_config.py` の拡張が必要になる。対象外にするなら README にその旨を書く。

## 過去の不具合から得た実装上の注意

- **ROSコールバックの処理能力に上限がある**: GUIモードでは10msのQTimerから
  `rclpy.spin_once(timeout_sec=0)` を呼んでおり、1回で処理するコールバックは1つだけ。2026-09-12の実測では
  1デバイスあたり約31指令/s、指令の遅れは最大約190ms(serial_tx合計 約310msg/s)だった。
  デバイス・高頻度トピックを増やす変更や「遅延っぽい」挙動の調査では、まずこれを疑う
  (改善案: spin_onceのドレインループ化、QoS depth 1、不要な再発行の削減)。
- **同じ指令トピックに複数の送信元があると、後から書いた方が勝つ**: 送信元が `topic_passthrough`/`direct_tx`
  のように複数ある場合、コード上の排他はGUIのチェックボックスしかない。高頻度でゼロを送り続ける
  送信元(ジョイスティックの無操作時など)が、別の送信元の指令を上書きして実質無効化した事例がある。
  「送信しているのに効かない」ときは、より高頻度の上書きを疑う。所有権を明示的に決めること。
- **GUIの数値入力は実機へ直結する**: 負のゲインが入力されて機構を破壊した事例がある(2026-09-10)。
  ゲイン・速度/加速度上限・サーボ範囲などは (1)ウィジェット (2)ノード/パラメータ受付
  (3)指令を組み立てる箇所 の3か所で検証する(yaml/launchの初期値は(2)を通らない)。
  QValidator にテキストを書き換えさせない(日本語IMEと競合して `1`→`11` のように入力が重複した)。
- **ドキュメント・ノートより実機設定が新しいことがある**: `generated_firmware/` やyamlの値がドキュメントと
  食い違っても、ドキュメントに合わせて黙って直さない。食い違いを報告して確認する。
- **回転方向の符号・原点はエンコーダの種類ごとに違う**: エンコーダの取得元を切り替えたら、
  以前の符号設定を流用せず実機で再確認する。

## 安全

このコードは実機のモータ・サーボ・ソレノイドを動かす。スロット割り当て・スケール・クランプ
(`MD_PWM_MAX`、電流上限、サーボ角度範囲等)・起動時の初期出力に関わる変更は、
意図しない動作につながりうる点をユーザーに明示する。チューニング済みの値(ROBOMASのPIDゲイン、
CubeMarsのMITレンジ、電流制限等)は依頼なしに変更しない。
**ファームには通信タイムアウトによる自動停止は無く、最後の指令を保持し続ける**(`docs/internals.md` 6章)。
停止はPC側のE-STOPか、ゼロ指令の明示的な送信で行う。タイムアウト停止を追加する場合は全モードで
挙動が変わるため、ユーザーに確認する。

## コーディング規約

- コメント・docstring・ドキュメントは**日本語**。既存コードは「なぜそうしたか」(過去の不具合・実測値・日付)を
  コメントに残す文化があるので、仕様上の理由がある変更は同様に理由を書く。
- Python: 型ヒントあり、`from __future__ import annotations`。Qt非依存のロジックは別モジュールに分けてテスト可能にする
  (例: `firmware_flash.py` ⇔ `firmware_dialog.py`)。
- ファーム: Arduino + FreeRTOS。タスク間共有データ(`Tx_16Data`/`Rx_16Data` 等)はクリティカルセクションでスナップショットしてから使う。
- 挙動やプロトコルを変えたら `docs/index.md`(利用者向け)・`docs/internals.md`(内部実装)・該当ファームの `README.md` も更新する。
- コミットメッセージは日本語の短い1行(例: `ノード側でサーボが動かない不具合を修正`)。
