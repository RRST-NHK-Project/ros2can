---
title: ros2can 技術マニュアル（内部実装編）
---

> 本ページは [マニュアル](index.md) の補足として、`ros2can` の**内部処理の流れ**（スレッド/タスク構成、シリアル・CANのプロトコル詳細、制御ループの計算式、既知の実装上の注意点）を実装レベルでまとめたものです。
> 「使い方」は [マニュアル](index.md) / [クイックスタート](quickstart.md) を参照してください。本ページは PC 側 (`ros2can/*.py`) とファームウェア側 (`firmware/xiao-esp32-s3_can2io/`, `firmware/b-g431-esc1_can2io/`) のソースコードを読解して書かれています。ソース自体が一次情報であり、本ページとの食い違いに気付いた場合はソース側を信用してください。

## 目次

- [1. 全体アーキテクチャ](#1-全体アーキテクチャ)
- [2. PC側 (`ros2can` Python/ROS 2 GUI) の内部実装](#2-pc側-ros2can-pythonros-2-gui-の内部実装)
  - [2.1 プロセス構成とイベントループ統合](#21-プロセス構成とイベントループ統合)
  - [2.2 シリアルフレームプロトコル (`frame_codec.py`)](#22-シリアルフレームプロトコル-frame_codecpy)
  - [2.3 シリアルリンク層と排他制御 (`serial_link.py`)](#23-シリアルリンク層と排他制御-serial_linkpy)
  - [2.4 ハードウェア検出・管理 (`hardware_manager.py`)](#24-ハードウェア検出管理-hardware_managerpy)
  - [2.5 ROSバックエンド (`ros_backend.py`)](#25-rosバックエンド-ros_backendpy)
  - [2.6 デバイスプロファイル (`device_profiles.py`)](#26-デバイスプロファイル-device_profilespy)
  - [2.7 エンコーダunwrapアルゴリズム (`counter_unwrapper.py`)](#27-エンコーダunwrapアルゴリズム-counter_unwrapperpy)
  - [2.8 GUI構造 (`main_window.py` / `device_panel.py` / `widgets.py`)](#28-gui構造-main_windowpy--device_panelpy--widgetspy)
  - [2.9 ファームウェア生成・書き込み](#29-ファームウェア生成書き込み)
  - [2.10 設定マージロジック (`settings_store.py`)](#210-設定マージロジック-settings_storepy)
- [3. ファームウェア: `xiao-esp32-s3_can2io`](#3-ファームウェア-xiao-esp32-s3_can2io)
  - [3.1 FreeRTOSタスク構成とモード分岐](#31-freertosタスク構成とモード分岐)
  - [3.2 `config.hpp` のコンパイル時設定とピン共有](#32-confighpp-のコンパイル時設定とピン共有)
  - [3.3 UARTフレームプロトコル（実機側）](#33-uartフレームプロトコル実機側)
  - [3.4 CANフレームプロトコルとノード分配](#34-canフレームプロトコルとノード分配)
  - [3.5 GPIO/PWM/エンコーダ処理 (`pin_ctrl_*`)](#35-gpiopwmエンコーダ処理-pin_ctrl_)
  - [3.6 CubeMars AKシリーズの実装 (`cubemars.cpp`)](#36-cubemars-akシリーズの実装-cubemarscpp)
  - [3.7 DJI RoboMasterの実装 (`robomas.cpp`)](#37-dji-robomasterの実装-robomascpp)
  - [3.8 状態LED (`status_led.cpp`)](#38-状態led-status_ledcpp)
- [4. ファームウェア: `b-g431-esc1_can2io`（FOC）](#4-ファームウェア-b-g431-esc1_can2iofoc)
- [5. エンドツーエンドのデータフロー](#5-エンドツーエンドのデータフロー)
- [6. 安全機構・フェイルセーフの全体まとめ](#6-安全機構フェイルセーフの全体まとめ)
- [7. 既知の注意点・実装上のTODO](#7-既知の注意点実装上のtodo)
- [8. 主要ファイル索引](#8-主要ファイル索引)
- [9. 付録: `serial_bridge`（前身パッケージ）の内部実装](#9-付録-serial_bridge前身パッケージの内部実装)
  - [9.1 開発背景・設計思想](#91-開発背景設計思想)
  - [9.2 ポート番号問題とID方式・仲介ノードという設計判断](#92-ポート番号問題とid方式仲介ノードという設計判断)
  - [9.3 通信フレームの設計](#93-通信フレームの設計)
  - [9.4 ROS 2側: 常駐スキャンスレッドと動的ノード生成 (`main.cpp`)](#94-ros-2側-常駐スキャンスレッドと動的ノード生成-maincpp)
  - [9.5 ポートスキャン (`port_scanner.cpp`)](#95-ポートスキャン-port_scannercpp)
  - [9.6 SerialBridgeNode: 接続管理・RX・TX (`bridge_node.cpp`)](#96-serialbridgenode-接続管理rxtx-bridge_nodecpp)
  - [9.7 グラフィカルUI (`graphical_ui.hpp`)](#97-グラフィカルui-graphical_uihpp)
  - [9.8 マイコン側: FreeRTOSタスク構成 (`firmware/esp32_serial_bridge`)](#98-マイコン側-freertosタスク構成-firmwareesp32_serial_bridge)
  - [9.9 `ros2can` との主要な差分](#99-ros2can-との主要な差分)
  - [9.10 `serial_bridge` 側 主要ファイル索引](#910-serial_bridge-側-主要ファイル索引)

---

## 1. 全体アーキテクチャ

`ros2can` は大きく3つの独立したコードベースで構成されます。

```
┌─────────────────────────┐   USB Serial    ┌───────────────────────────┐   CAN bus   ┌─────────────────────┐
│  PC: ros2can (Python)   │◄───115200bps───►│ xiao-esp32-s3_can2io      │◄──1Mbps────►│ ノード / 独立デバイス │
│  ・rclpy + PyQt5 GUI     │  serial_bridge   │ (ESP32-S3, FreeRTOS)     │             │ (MODE_CAN / ROBOMAS /│
│  ・24 x int16 スロット   │  互換バイナリ    │ ・MODE_CAN_HOST/CAN/IO/  │             │  CUBEMARS 独立機)     │
│    フレーム             │  フレーム        │   ROBOMAS/CUBEMARS/...  │             │  b-g431-esc1_can2io  │
└─────────────────────────┘                  └───────────────────────────┘             │ (STM32, SimpleFOC)   │
                                                                                          └─────────────────────┘
```

- **PC側**: `rclpy` の ROS 2 ノードと PyQt5 GUI が**同一スレッド**内で協調動作（別スレッドは存在せず、`QTimer` で ROS 2 の `spin_once` を回す）。シリアルポートを直接掴み、24要素 `int16` 配列（serial_bridge互換フォーマット）をバイナリフレームとして送受信する。
- **ホストファームウェア (`xiao-esp32-s3_can2io`)**: 1枚のESP32-S3が「CANホスト」として、USBシリアル側は常に24スロットのフレームで通信しつつ、CANバス側では最大4ノード（またはCubeMars/RoboMasterでは独立デバイス4台）にスロットを分配・集約する。FreeRTOSの複数タスクで構成される。
- **ノード/独立デバイス**: 同じ `xiao-esp32-s3_can2io` ファームウェアを別モード（`MODE_CAN`）で書き込んだ子基板、または `MODE_ROBOMAS`/`MODE_CUBEMARS` で書き込まれ独立CANバスに直結する基板、あるいは STM32 B-G431B-ESC1 上で SimpleFOC を使う `b-g431-esc1_can2io`（FOCモータ用、DJI RoboMaster互換のデータモデルで通信）。

3系統ともプロトコルの型（24スロット、`int16`、ビッグエンディアン、XORチェックサム、CAN ID帯の分離など）を共有しており、**PC側とファームウェア側は独立して実装されているが、フレームフォーマットの取り決めのみで結合している**（共通ライブラリはない）。

この「24スロットint16・XORチェックサム・バイト単位再同期」というプロトコルの原型は前身パッケージ `serial_bridge` に由来します。開発背景・設計判断の経緯や `serial_bridge` 自体の内部実装は[9. 付録: `serial_bridge`（前身パッケージ）の内部実装](#9-付録-serial_bridge前身パッケージの内部実装)を参照してください。

---

## 2. PC側 (`ros2can` Python/ROS 2 GUI) の内部実装

対象ディレクトリ: `ros2can/` （パッケージ本体）

### 2.1 プロセス構成とイベントループ統合

`main.py` がエントリポイントで、**専用スレッドを持たず、Qtのイベントループ (`app.exec_()`) を唯一のメインループとして使う**設計です。ROS 2 の `spin` はQTimer経由でGUIスレッド上に統合されています。

| タイマー | 周期 | 呼び出し先 | 役割 |
|:---|---:|:---|:---|
| spin_timer | 10ms | `rclpy.spin_once(node, timeout_sec=0)` | ROSコールバック（サブスクライバ等）をGUIスレッド上で実行 |
| hardware_timer | 10ms | `HardwareManager.service()` | シリアルリンクの読み書きサービス |
| simulator_timer | 50ms (20Hz) | `RosBackend.service_simulators()` | 仮想デバイスのTX→RXループバック |
| publish_timer | 50ms (20Hz) | `RosBackend.publish_all_direct()` | ダイレクト送信ONのデバイスへの周期送信 |
| UI_refresh_timer (`main_window.py`) | 200ms | `_periodic_ui_refresh()` | ラベル・現在表示パネルの再描画 |
| topic_rescan_timer | 1000ms | `RosBackend.rescan_topics()` | 未登録の `serial_tx_*`/`serial_rx_*` トピックを自動検出 |

このため**ロック（mutex）が一切登場しません**。`RosBackend`/`HardwareManager` の共有状態はすべてGUIスレッド上のタイマーコールバックとQtシグナルスロットからのみ触られるため、単一スレッドの逐次実行として安全性が保たれています。唯一の例外は次節で述べる `_ScannerThread`（バックグラウンドポートスキャン）で、これは共有データを直接触らず「まだ誰も掴んでいないポート名の探索」だけを行う設計にすることでデータ競合を避けています。

`--nogui` モードでは `QCoreApplication`（ウィジェット無し）+ `ConsoleUi`（ANSIエスケープでのテキストダッシュボード、`CONSOLE_RENDER_MS=100`で再描画）を使い、`HardwareManager.deviceClaimed` を検出すると自動的に `direct_tx=True` を強制することで、旧 `serial_bridge` と同じ「常時ブリッジ」動作を再現します。SIGINT処理がQtのイベントループにブロックされないよう、200ms周期の空タイマー（`sigint_pump`）でPython側のシグナルハンドラ実行機会を確保しています。

### 2.2 シリアルフレームプロトコル (`frame_codec.py`)

serial_bridge互換のバイナリフレームです。全長 **52バイト固定**。

```
オフセット   サイズ   内容
0            1        START_BYTE = 0xAA
1            1        DEVICE_ID (uint8)
2            1        LENGTH = 48 (= 24スロット × 2バイト、固定)
3..50        48       DATA: int16 × 24個、ビッグエンディアン（各値は 0..0xFFFF の符号なし表現で送出）
51           1        CHECKSUM = DEVICE_ID ^ LENGTH ^ DATA[0]の上位バイト ^ … ^ DATA[23]の下位バイト
                       （XOR、START_BYTEは含まない）
```

- `encode_frame(device_id, data)`: 24要素へパディング/切り詰め、`& 0xFFFF` して2の補数→符号なし変換し、ビッグエンディアンで詰めてXORチェックサムを付与。
- `FrameParser.pop_frame()`: ストリーミング受信バッファに対する**1バイト単位で再同期する状態機械**。`START_BYTE` 以外を見つけたら1バイト破棄して再走査、`LENGTH` が異常（奇数、または想定を大幅に超える）なら同様に破棄、チェックサム不一致でも1バイト破棄して継続する。これにより、ノイズや起動直後のゴミバイトが混入しても自動的に同期を取り戻します。
- `_to_signed16(v)`: `v - 0x10000 if v >= 0x8000 else v` という単純な2の補数変換。

**この設計はファームウェア側 `serial_task.cpp` の状態機械と完全に対称**です（後述 3.3節）。

### 2.3 シリアルリンク層と排他制御 (`serial_link.py`)

ポートの専有には**二重の排他制御**が使われています。

1. **pyserialの `exclusive=True`**: flockベースの排他（同一マシン上でのros2can自身の多重起動対策）。
2. **`ioctl(fd, TIOCEXCL)`**（ioctl番号 `0x540C`）: Linuxのtty層での強制排他。これが立っているttyに対する他プロセスからの新規 `open()` は `EBUSY` で失敗する。旧`serial_bridge`（C++版）はこのフラグを見ないため、**「ros2canが先にポートを掴んでいれば」serial_bridge側の`open()`が静かに失敗する**という片方向の保護になります（逆方向の保護はできない。[3.1節 serial_bridgeとの併用について](index.md#31-serial_bridge-との併用について)参照）。クローズ時は `TIOCNXCL`（`0x540D`）で明示的にフラグを解放します。

主な処理:
- `probe_port()`: ポートを一時的に開き、`settle_sec`(既定0.5秒、USB CDCの安定待ち)後に一定時間バイトを読み、有効フレームが取れれば `device_id` を返す。
- `SerialLink.open()`: クローズ→raw open→TIOCEXCL取得→バッファリセット→新しい `FrameParser` を生成（再接続のたびにエラーカウンタがリセットされる）。
- ボーレートは **115200bps固定**。

### 2.4 ハードウェア検出・管理 (`hardware_manager.py`)

**このモジュール唯一のQThreadが `_ScannerThread`** です。`list_serial_ports()`（`/dev/ttyUSB*`・`/dev/ttyACM*`）から、既にros2can自身が専有中のポートと `excluded_ports` を除いた候補を順に `probe_port()` します。見つかれば `deviceDetected` シグナルを発行し、GUIスレッド側のスロットへ自動的にキューイングされます（Qtの `AutoConnection` が別スレッド発火時に `QueuedConnection` として振る舞うため、明示的なロックは不要）。全ポートを一巡したら `scan_interval_sec`（既定5秒）を0.1秒刻みでポーリング待機しつつ即時停止可能にしています。

GUIスレッド側の `HardwareManager.service()`（10ms周期）が中心ループで、各リンクについて:
- 閉じていれば再接続を試行（`reconnect_interval_sec` 既定3秒間隔）。
- `read_frames()` でフレームを取得し、`frame_id != device_id` なら破棄。
- 各フレームで `_last_rx_time` を更新し `frameReceived` シグナルを発行。
- `rx_timeout_sec`（既定2.0秒）以上RXが無ければ切断扱い。

ファームウェア書き込み（`firmware_dialog.py`）中はポートを `lock_port_for_flash()` で一時的にスキャン対象・再接続対象から除外し、書き込みプロセスとの競合を避けます。

### 2.5 ROSバックエンド (`ros_backend.py`)

**デバイスの3モード**。トピックの発行/購読方向がモードにより**逆転**する点が重要です。

| モード | 意味 | Publish | Subscribe |
|:---|:---|:---|:---|
| `MODE_HARDWARE` | 実機検出済み | `serial_rx_[ID]` | `serial_tx_[ID]` |
| `MODE_SIMULATOR` | デバッグ用仮想デバイス | `serial_rx_[ID]` | `serial_tx_[ID]` |
| `MODE_TOPIC_CLIENT` | 既存トピックへの相乗り | `serial_tx_[ID]` | `serial_rx_[ID]` |

`DeviceChannel` が1台分の状態を保持します（`tx_data`/`rx_data`: 24要素int16配列、`topic_passthrough`/`direct_tx`: 送信経路フラグ、`unwrappers`: 24個の `CounterUnwrapper`、`zero_offset`: 原点セットのオフセット）。`connected` は `last_rx_time` から `STALE_TIMEOUT_SEC=1.5`秒以内かで判定します。

**トピック通過 (`topic_passthrough`) とダイレクト送信 (`direct_tx`) の実装**（本節が全体の中でも特に「動作の要」）:

- 外部ノードから `serial_tx_[ID]` に指令が来ると `_on_hardware_tx_command()` が呼ばれます。**`topic_passthrough` が `False` なら即 `return`（無視）**。`True` なら `tx_data` を更新し、その場で（周期を待たず）`hardware.write()` します。
- `direct_tx` は別経路です。GUI操作で `tx_data` を書き換えても即座には送信されず、`publish_timer`（20Hz）から呼ばれる `publish_all_direct()` が `direct_tx=True` のデバイスにだけ周期送信します。
- **この2フラグの排他はROSバックエンド側にはコード上の制約が無く**、GUI（`device_panel.py` のチェックボックス相互OFF）のみで排他されています。`--nogui` モードは `direct_tx` を常時ONにするため、この排他はGUIモード限定です。

**ハードウェア検出→デバイス登録**: `_on_hardware_claimed()` は、`MODE_TOPIC_CLIENT` などから昇格する場合、既存のpublisher/subscriptionを一度破棄してから作り直します（Publish/Subscribeの向きが逆転するため、これを怠るとRXフレームが誤って `serial_tx_[ID]` に発行されてしまう）。

**受信フレーム処理**: `_on_hardware_frame()` が `rx_data` を更新→`serial_rx_[ID]` をPublish→`_publish_unwrapped()`（unwrap値・zeroed値の配信、2.7節）→`rxUpdated` シグナル発行、という順で処理します。unwrap計算は `HardwareManager.service()` の10ms周期の中で行われるため、サンプル密度が高く保たれます。

**`zero_channel_request` トピック**: `Int32MultiArray`、`data=[device_id, channel_index]`。`channel_index<0` で全24チャンネル一括原点セット。専用サービス型ではなくトピックにしている理由は、カスタムサービス型を定義するには `rosidl` 生成が必要で、`ament_python` 単体パッケージの構成では定義できないためです。応答は同期的には返らず、ログ（通信ログダイアログ）にのみ記録されます。

**E-STOP (`emergency_stop_all()`)**: 全デバイスの `direct_tx` と `topic_passthrough` を**両方**OFFにしてから `tx_data` をゼロ埋めして送信します。`topic_passthrough` もOFFにしないと、外部ノードが指令を送り続けている場合に `_on_hardware_tx_command()` が即座に非ゼロ値を書き戻してしまい、E-STOPが無効化されてしまうためです。

**シミュレータのループバック** (`service_simulators()`, 20Hz): `MODE_SIMULATOR` デバイスについて、`sim_rx_override` に含まれないスロットは `tx_data` をそのまま `rx_data` にコピーします（TX→RXの自動ループバック）。スイッチ・カウンタ・故障コードなど「実機では指令と無関係な独立したセンサ入力」に相当するスロットは、GUIから手動設定するとこの自動ループバックの対象から外れ、値が保持され続けます。

### 2.6 デバイスプロファイル (`device_profiles.py`)

`DeviceProfile`（`key/name/tx/rx/node_count/slots_per_node/editable`）と `ChannelDef`（`index/label/kind/min/max/step/unit/scale/decimals/options/zeroable`）で、24スロットの意味づけを抽象化します。

ビルトインプロファイル:

| キー | 概要 |
|:---|:---|
| `xiao_smd_can_host`（既定） | 4ノード x 5スロット。SERVO1-3+MD1-2指令 / SW1-3+ENC1-2帰還 |
| `xiao_can2io_with_foc` | 上記のうち1ノードをFOCモータ（b-g431-esc1）チャンネルに置換 |
| `xiao_mes_can_host` | BOARD_MES用。MD1-2固定 + ENC1/SW1固定 + ENC2⇔SW2/SW3切替 |
| `xiao_ss_can_host` | BOARD_SS用。9スロット/ノード（SERVO1-5+TR1-4）、ノード数既定2 |
| `robomas_driver` | MODE_ROBOMAS、独立デバイス、DJI RoboMaster x4 |
| `cubemars_ak_driver` | MODE_CUBEMARS、独立デバイス、CubeMars AKシリーズ x4 |
| `generic_raw` | 24スロットの生編集のみ |

カスタムプロファイルは `~/.config/ros2can/profiles/*.json` に保存され（Git管理外）、`all_profiles()` はビルトインにカスタムを `dict.update()` でマージします（同キーはカスタムが優先）。

### 2.7 エンコーダunwrapアルゴリズム (`counter_unwrapper.py`)

**背景**: ESP32のPCNT（パルスカウンタ）は `counter_h_lim=32767`/`counter_l_lim=-32768` に到達するとそれぞれ独立に0へリセットされます（実機確認済み、単純な2の補数オーバーフローとは異なる挙動）。このため `counts_per_wrap` の既定値は **32768**（`h_lim + 1 = -l_lim`）です。

`CounterUnwrapper.update(raw)` のアルゴリズム:

1. 初回は `raw` をそのまま `_unwrapped` として採用。
2. 2回目以降: `naive_delta = raw - prev_raw` を計算。
3. 候補を3つ作る: `naive_delta`, `naive_delta - counts_per_wrap`, `naive_delta + counts_per_wrap`。`abs()` でソートし最小（`best`）と次点（`second`）を得る。
4. **曖昧判定**: `abs(best) >= half_wrap * (1 - ambiguous_margin_ratio)`（既定 `16384 * 0.9 = 14745.6`）かつ直近の移動方向（`trend_sign`）が非ゼロの場合のみ、`best` の符号が `trend_sign` と食い違っていないか確認する。食い違い、かつ `second` が一致するなら `delta = second` に切り替える。
5. それ以外（通常時）は素直に `best` を採用する（静止中の微小ノイズを誤ってラップ判定しないため）。
6. `_unwrapped += delta`。`abs(delta) >= trend_noise_floor`（既定4）なら `trend_sign` を更新。

`serial_rx_[ID]_unwrapped`（`Int32MultiArray`）として配信され、さらに `zeroed = unwrapped - zero_offset` が `serial_rx_[ID]_zeroed` として配信されます。**この処理は `MODE_HARDWARE`/`MODE_SIMULATOR` のみで行われ、`MODE_TOPIC_CLIENT` では行われません**（生の `rx_data` をそのまま原点セットの基準にする設計）。

### 2.8 GUI構造 (`main_window.py` / `device_panel.py` / `widgets.py`)

- `main_window.py`: 左に検出デバイス一覧、右に `DevicePanel` を積んだスタック。`UI_REFRESH_MS=200` で現在表示中パネルの再描画、`TOPIC_RESCAN_MS=1000` でトピック自動検出。`closeEvent` は `emergency_stop_all()` を呼んでから閉じる（安全側）。
- `device_panel.py`: Control/Monitor/Raw/Info の4タブ。ノード構成を持つプロファイルはさらにノード別サブタブに分割。**トピック通過⇔ダイレクト送信の相互排他はチェックボックスの `toggled` シグナルハンドラでの相手強制OFF**として実装（`RosBackend` にはこの制約は無い、2.5節参照）。**20Hz周期送信自体はdevice_panel.py側にはタイマーが無く**、main.pyの `publish_timer` → `RosBackend.publish_all_direct()` が実際の送信を担う。
- `widgets.py`: `SizedTabWidget`/`SizedStackedWidget`（過去の最大サイズを引きずらないようcurrentWidgetのみでサイズヒント計算）、`ChannelControlRow`/`ChannelMonitorRow`（TX編集/RX表示、`zeroable`時のみ原点セットボタン）、`RawSlotTable`。

### 2.9 ファームウェア生成・書き込み

`config.hpp` の書き換え方式は **「正規表現による1行単位の置換」**（パーサでもファイル全体再生成でもない）:

- `#define NAME VALUE` 形式の行を `r"^(?P<prefix>\s*#define\s+{name}\s+)(?P<value>\S+)"` で検出し、`VALUE` 部分だけを置換する。
- `MODE_*` の選択は専用ロジックで、対象モード行のコメントを外し、他の `MODE_*` 行はコメントアウトする（複数有効化されているとパース時に例外）。
- `BOARD_VARIANT` の選択も同様に専用処理（シンボル値のため数値パースとは別扱い）。
- **意図的に対象外**: `#if` 分岐（モータ機種選択など）内に**同名マクロが複数回登場する**定数（例: RoboMasterの速度PIDゲイン）は、単純な「マクロ名で1行置換」では全分岐が同じ値に書き換わってしまうため対象外。同様の理由で `CAN_NODE_COUNT`/`CAN_SLOTS_PER_NODE` もBOARD_VARIANTごとの`#if`分岐に依存するため対象外（テンプレートの値がそのまま引き継がれる）。

生成フロー: `shutil.copytree()` でテンプレート一式をコピー（`.pio`/`.vscode`/`__pycache__` 等は除外）→コピー先の `src/config.hpp` だけを書き換え→`generated_firmware/<名前>/` に保存。パストラバーサル対策として、生成先が `output_root` 配下であることを `os.path.commonpath()` で検証します。

書き込みは `FlashWorker(QThread)` が `pio run -d <project> -t upload --upload-port <port>` をサブプロセス実行し、標準出力を1行ずつシグナルでGUIへ流します。書き込み前に対象ポートを `lock_port_for_flash()` で一時除外し、完了/失敗/中断いずれの場合も `unlock_port_for_flash()` で解除します。

### 2.10 設定マージロジック (`settings_store.py`)

優先順位（後勝ち）:

1. コード内 `DEFAULT_SETTINGS`
2. `config/ros2can.yaml`（パッケージ同梱）
3. `~/.config/ros2can/settings.yaml`（GUIの「設定」ダイアログ保存、Git管理外）
4. ROS 2パラメータ（`--ros-args -p` / launchの `parameters=[...]`）

`load_settings()` は 1.+2. の結果に 3. を `dict.update()` で重ねたものです。この結果は `declare_parameter()` の `default` 値として渡されるため、**4番目が最優先という順序は ROS 2 パラメータ機構自体の標準動作をそのまま利用しているだけ**で、`settings_store.py` 自身には優先順位を制御するロジックはありません。設定ダイアログでの保存は、既存内容を土台にして対象キーだけ `update()` するため、他のダイアログが書いた未知キーを消しません。

---

## 3. ファームウェア: `xiao-esp32-s3_can2io`

対象: `firmware/xiao-esp32-s3_can2io/src/`, `include/`

### 3.1 FreeRTOSタスク構成とモード分岐

`setup()` は起動直後に `delay(200 + 1*DEVICE_ID)` を挟みます（複数マイコンの起動タイミングをDEVICE_ID分だけずらし、電源投入時のバス輻輳を避ける狙い）。その後、コンパイル時に選択された1つのモードに応じて `xTaskCreate()`（コアピン留めなし）でタスクを生成します。

| タスク | スタック | 優先度 | 生成条件（モード） | 実装ファイル |
|:---|---:|---:|:---|:---|
| `serialTask` | 2048 | 10 | CAN_HOST / IO / DEBUG / ROBOMAS / CUBEMARS | serial_task.cpp |
| `canTask` | 4096 | 10 | CAN / CAN_HOST / CAN_MONITOR | can_task.cpp |
| `IO_Task` | 2048 | 11 | IO / CAN / CAN_HOST | pin_ctrl_task.cpp |
| `robomasTask` | 4096 | 11 | ROBOMAS | robomas.cpp |
| `cubemarsTask` | 4096 | 11 | CUBEMARS | cubemars.cpp |

**重要な設計上のポイント**: `MODE_ROBOMAS`/`MODE_CUBEMARS` は `canTask`（ノード分配ロジック）を使いません。代わりに `serialTask`（PCとの24スロット送受信）と `robomasTask`/`cubemarsTask`（独自のCAN送受信）が **`Tx_16Data`/`Rx_16Data` をロック無しで直接共有**することで、PC⇔モータの橋渡しを行います。`MODE_CAN_HOST`/`MODE_CAN` のような「ノード/スロット分配」の概念はこの2モードには存在せず、24スロットのうち先頭から詰めてモータ4台分の指令・帰還が並びます。

`loop()` 自体は `vTaskDelay(1000ms)` するだけで、全処理はFreeRTOSタスクに委譲されています。7モードのうち定義数の合計が1でなければコンパイルエラー(`#error`)になるチェックが入っています。

### 3.2 `config.hpp` のコンパイル時設定とピン共有

- `DEVICE_ID`: シリアルフレームのIDバイト。
- `CAN_ID`（3桁）: `CAN_NODE_INDEX = (CAN_ID % 100) - 1` で自動導出（101→0, 102→1, 103→2, 104→3）。
- `BOARD_VARIANT`: `BOARD_SOKI` / `BOARD_MES` / `BOARD_SS` の3種。ピン配置は `defs.hpp` 側で `#if BOARD_VARIANT == ...` により**コンパイル時**に切り替わります（実行時分岐ではありません）。
- ピン共有マクロもすべてコンパイル時の `#if`:
  - `MULTI1〜3`（BOARD_SOKI専用）: 0=スイッチ入力 / 1=サーボ出力。
  - `ENC1_MD`/`ENC2_MD`（BOARD_SOKI専用）: 0=エンコーダ入力 / 1=MD(PWM+DIR)出力。
  - `ENC2_SW`（BOARD_MES専用）: 0=ENC2 / 1=SW2+SW3。
- `CAN_NODE_COUNT`/`CAN_SLOTS_PER_NODE`: `BOARD_SS` は `2ノード x 9スロット`、それ以外は `4ノード x 5スロット`。`CAN_NODE_COUNT * CAN_SLOTS_PER_NODE > 24` は `#error` で強制的に弾かれます。
  - **実機で確認された既知の障害モード**（config.hppコメントに明記）: ホストは `[0, CAN_NODE_COUNT)` の全ノードへ毎周期送信するため、**`CAN_NODE_COUNT` を実接続数より大きく設定すると、存在しないノード宛のACKエラーが蓄積し、実測で起動後約40msでBus-Offに陥る**。実接続数と必ず一致させる必要があります。
- `CAN_HOST_DIAG_ENABLE`: 診断ログを `Serial.println` で直接出す設定。バイナリフレームと混ざり、PC側で「不正な同期バイトの破棄」が発生するため通常は無効(0)にします。

### 3.3 UARTフレームプロトコル（実機側）

PC側（2.2節）と対称的な**バイト単位ステートマシン**で実装されています。

```
START_BYTE = 0xAA
LENGTH     = Tx16NUM * 2 = 48（固定）
CHECKSUM   = ID ^ LEN ^ DATA[0]の上位バイト ^ … ^ DATA[23]の下位バイト（START_BYTEは含まない）
```

送信 (`send_frame()`, 5ms周期): `Tx_16Data` をクリティカルセクション（`portENTER_CRITICAL`/`portEXIT_CRITICAL`、専用の `portMUX_TYPE g_frame_lock`）でスナップショットしてからフレーム化し `Serial.write()` で一括送信。

受信 (`receive_frame()`): 1バイトずつ `WAIT_START → WAIT_ID → WAIT_LEN → WAIT_DATA → WAIT_CHECKSUM` のステートマシンで処理。`LENGTH` が想定を超えていれば即座に `WAIT_START` へ戻ります。**チェックサムが一致し、かつ受信IDが自機の `DEVICE_ID` と一致する場合のみ** `Rx_16Data` に反映します。条件を満たさない場合は明示的なエラー通知（NAK等）は無く、単に無視して次のフレームを待ちます。

### 3.4 CANフレームプロトコルとノード分配

CANビットレートは **1Mbps** 固定（`TWAI_TIMING_CONFIG_1MBITS()`）。コメントによれば、以前は500kbpsだったものを、`MODE_ROBOMAS`/`MODE_CUBEMARS` の独立バスと物理的に共有できるよう統一した経緯があります。

**ID体系**（指令と帰還を完全に別ID帯に分離）:

```
CAN_FRAME_ID_CMD_BASE = 0x100   // host -> node（指令）
CAN_FRAME_ID_FB_BASE  = 0x180   // node -> host（帰還）
フレームID = base + node_index*16 + chunk
1フレーム: DLC=8, int16 x 4個（ビッグエンディアン）
chunk数 = ceil(CAN_SLOTS_PER_NODE / 4)
```

コメントには「以前は指令と帰還が同一ID帯を共有しており、ホストが送信を始めた瞬間にbus_errorが秒間数千件規模で急増した」という実機不具合の記録が残っています。現行はこの分離により解消されています。

**グローバルスロットindexの変換**（ノード分配の核心）:

```cpp
slot_offset = node_index * CAN_SLOTS_PER_NODE;
// applyNodeSlotBlockToLocalControl(): slot_buffer[slot_offset + i] -> CanIoRxData[i]
// buildNodeSlotBlockFromLocalFeedback(): CanIoTxData[i] -> slot_buffer[slot_offset + i]
```

- **MODE_CAN_HOST**: 5ms周期(`CAN_TX_PERIOD_MS`)で `Rx_16Data` をスナップショットし、自ノード（`CAN_NODE_INDEX`）分は**CANを介さず直接**ローカルの `CanIoRxData` へ反映、他ノード分は `canSendNodeSlotBlock()` でCAN送信します（＝ホスト自身が「ノード0」を兼務）。帰還側は `canRecvAllNodeSlotBlocks()` が全ノードからのフレームを持続バッファへマージします（**未受信スロットは前回値を保持**——ノード送信周期(5ms)とホストのポーリング周期のズレを吸収するため）。
- **MODE_CAN（子ノード）**: 自ノード宛の指令のみ受信・反映し、5ms周期でローカルのセンサ値を帰還フレームとして送信します。
- CAN送信は `twai_transmit(&message, 0)`（**タイムアウト0＝非ブロッキング**）で行われます。ブロッキングするとホスト自身のローカルエンコーダ値publishまで遅延するバグがあった経緯があり、1周期分の送信ドロップは次の5ms周期での再送に任せる設計です。
- **Bus-Off自動復帰**: TWAIドライバは`BUS_OFF`から自動復帰しないため、100msごとに状態を確認し `twai_initiate_recovery()` → `twai_start()` を行うロジックが `canTask`・`cubemarsTask`・`robomasTask` それぞれに重複実装されています。

`MODE_CAN_MONITOR` は送信を一切行わず、帰還ID帯のフレームを要約デコードしてSerial出力するだけの診断専用モードです。

### 3.5 GPIO/PWM/エンコーダ処理 (`pin_ctrl_*`)

`IO_Task`（5ms周期、`vTaskDelayUntil` による固定周期実行）が、`CanIoRxData`/`CanIoTxData`（`MODE_CAN`/`MODE_CAN_HOST` 時）または `Rx_16Data`/`Tx_16Data` の固定インデックス（`MODE_IO` 時）を対象に、サーボ・MD(モータドライバ)・TR(ソレノイド)出力とエンコーダ・スイッチ入力を処理します。この入力元の切り替えも実行時分岐ではなく `#if defined(MODE_CAN)||defined(MODE_CAN_HOST)` によるコンパイル時分岐です。

- サーボ: `angle → clamp(MIN_DEG,MAX_DEG) → map(MIN_US,MAX_US) → duty` の順で変換し `ledcWrite()`。
- MD: 符号でDIRピンをHIGH/LOW、絶対値を `ledcWrite()` のデューティとして出力。
- エンコーダ: ESP32のPCNT（ハードウェアパルスカウンタ）を使用し、**raw pulse countをそのまま送信**（角度換算は行わない。deg換算はPC側 `device_profiles.py` の `scale` の責務）。
- BOARD_SOKIでは `MULTIn`/`ENCn_MD` の値に応じて、該当しない側のチャンネルは 0 固定で送出されます。

### 3.6 CubeMars AKシリーズの実装 (`cubemars.cpp`)

CAN 拡張ID（29bit）方式: `identifier = (cmd_id << 8) | motor_can_id`。制御コマンドID: 速度=3、位置=4、原点設定=5、MIT(Force Control)=8。帰還は `function_id=0x29` の周期送信フレーム。

**Cheetah方式の固定小数点エンコード** (`floatToUint()`、AK Series Module Product Manual 4.2節と同一の変換式):

```cpp
uint16_t floatToUint(float x, float x_min, float x_max, uint8_t bits) {
    x = clamp(x, x_min, x_max);
    return round((x - x_min) * ((1 << bits) - 1) / (x_max - x_min));
}
```

**MITモードの8バイトフレーム**は Kp(12bit)+Kd(12bit)+Position(16bit)+Speed(12bit)+Torque(12bit) をビットパックしたもので、Cheetahドライバ由来の標準的なMIT modeフォーマットと一致します。位置指令の物理限界（±12.5rad、Cheetah方式の固定小数点エンコーディング自体の限界）は[マニュアル 9.2節](index.md#92-mitモードの活用)で解説済みです。

`sendCommands()`（200Hz送信ループ）は各モータの `control_mode` を見て、速度(既定)/位置/MIT/原点設定のいずれかのコマンドを送信します。原点設定はエッジ検出フラグで1回だけ送信（フラッシュ摩耗回避）。

`receiveFeedback()` は8バイトを `position(int16,0.1deg)/speed(int16,10ERPM)/current(int16,0.01A)/temperature(int8,℃)/error_code(uint8)` にデコードします。故障コードは `0=無故障, 1=モータ過熱, 2=過電流, 3=過電圧, 4=低電圧, 5=エンコーダ故障, 6=MOSFET過熱, 7=モータストール`。

送信レートは**200Hz固定**（`vTaskDelay(5)`）。1kHzでは合計CANバス帯域を超えるため意図的に落とされています。

### 3.7 DJI RoboMasterの実装 (`robomas.cpp`)

- 電流指令: C610/C620は一括フレーム `0x200`（4モータ分の `int16`をビッグエンディアンで格納）。GM6020は別ID `0x1FE`。モータ種別ごとに最大電流と正規化スケールが異なる(`M3508: ±20A/16384`, `M2006: ±10A/10000`, `GM6020: ±3A/16384`)。
- 帰還フレーム: C610/C620は `0x201-0x204`、GM6020は `0x205-0x208`。8バイトのうち `encoder_count/rpm/current_raw`(各int16)を使用。
- **エンコーダの多回転展開**は `HALF_ENCODER=4096`（`ENCODER_MAX=8192`の半分）を跨ぐジャンプを検出する典型的なラップアラウンド処理で、ギア比で除して出力軸角度・rpmに変換します。
- **速度モード（既定）**: `PIDController`（`PID.hpp`）による速度PID。台形積分・積分クランプ・**測定値の微分（D on measurement）**を実装。出力は電流換算ゲインを掛けて指令電流にします。
- **MIT(位置PD)モード**: CubeMarsのMITと対称的な機能ですが実装方式は異なります。RoboMaster側のESC/GM6020はCANで生の電流指令しか受け付けないため、**位置PD制御ループそのものをマイコン側で計算**します：

  ```
  pos_error = target_pos_deg - angle
  vel_error = target_vel_ff_rpm - vel
  current = clamp(kp*pos_error + kd*vel_error + current_ff, ±ROBOMAS_MAX_CURRENT_A)
  ```

  独立の積分項はありません（PD＋フィードフォワードのみ）。位置フィードバックはロボマス内蔵ロータエンコーダ由来です。
- 制御ループは200Hz（`vTaskDelay(5)`）。`dt` は `micros()` 基準で計算し（`millis()`だと量子化誤差がdtに対して無視できないため）、異常なdtは飽和処理されます。

### 3.8 状態LED (`status_led.cpp`)

CAN活動とシリアル活動を別々に記録し、優先順位付きで表示します: ①CAN活動があれば100ms周期のトグル点滅、②CANは途絶えたがシリアルは活動中なら500ms周期の短パルス、③両方途絶えなら消灯。「受信の都度点灯して一定時間保持」ではなく「一定時間活動があれば点滅し続ける」方式にすることで、数msおきに指令が届く通常運用時に常時点灯へ張り付いてしまう問題を回避しています。

---

## 4. ファームウェア: `b-g431-esc1_can2io`（FOC）

対象: `firmware/b-g431-esc1_can2io/src/`。STM32 B-G431B-ESC1 + SimpleFOC 2.3.1（`platformio.ini` でバージョン固定）を用いた速度制御専用のCANノードです。FreeRTOSは使わず、Arduinoの `setup()`/`loop()` のみで構成されます（`loop()` 1回=電流ループ1回+速度ループ1回+CAN処理1回）。

**独自エンコーダセンサ `TIM4Sensor`**: SimpleFOCの `Sensor` を継承し、STM32G4のTIM4をハードウェアクアドラチャエンコーダ（`TIM_ENCODERMODE_TI12`、入力フィルタ`IC1Filter=IC2Filter=3`）として使用。16bitカウンタのオーバーフローを自前補正しながら `int32_t` に積算することで、SimpleFOC標準の `full_rotations` 管理に頼らない**連続角**を実現しています。

**SimpleFOCの構成**: `BLDCMotor`(pole pairs=7) + `BLDCDriver6PWM`(6PWM独立ハイ/ローサイド) + `LowsideCurrentSense`(shunt 0.003Ω, gain -64/7, **`skip_align=true`** で電流センスアライメントをスキップ)。モジュレーションは `SpaceVectorPWM`、`torque_controller=foc_current`（内部電流ループを常時使用）、制御モードは **`MotionControlType::velocity` のみ**（`motor.P_angle` は未設定のため角度/トルクモードは実装されていない）。

**電流・速度ゲイン**（すべてコンパイル時固定、CAN経由での変更不可。CAN_SLOTS_PER_NODE=5では枠が無いため）: 速度PID `P=0.02,I=0,D=0`（output_ramp=1000, LPF Tf=0.02）、電流PID(q/d共通) `P=0.1,I=10,D=0`。

**CANプロトコル**: `CAN_ID=102`（`CAN_NODE_INDEX=1`）、`CAN_SLOTS_PER_NODE=5`、**500kbps**（Prescaler=2, Seg1=148, Seg2=21、PCLK1=170MHz実測値ベース、サンプルポイント87.6%）。ID体系・チャンク分割・ビッグエンディアンパッキングは `xiao-esp32-s3_can2io` の `MODE_CAN_HOST` ノード分配プロトコル（3.4節）と完全に同一の方式（`0x100+node*16+chunk` / `0x180+node*16+chunk`）を使います。つまり**「RoboMaster互換」を謳っているのはスロットのデータ意味論・スケールのみ**で、CAN ID帯やフレーム分割方式自体は `MODE_ROBOMAS` の専用ID帯（`0x200`系等）ではなく `MODE_CAN_HOST` の汎用ノード分配方式です。

> ⚠ **要確認の不整合**: `xiao-esp32-s3_can2io` 側の `MODE_CAN_HOST`/`MODE_CAN` は現在CANビットレートを **1Mbps** に統一済み（3.4節）ですが、`b-g431-esc1_can2io` 側の `config.hpp` は **500kbps** のままです。同一物理バスに両者を接続する構成（`xiao_can2io_with_foc` プロファイル）ではビットレート不一致によりCAN通信が成立しない可能性があるため、実機接続前に両ファームウェアの `CAN_BITRATE`/ビットタイミング設定を必ず突き合わせてください。

**スロット割当**: RX(host→node)は index0=`target_velocity`(生rpm)のみ使用。TX(node→host)は index0=`angle`(0.1deg/LSB, 連続角)、index1=`velocity`(生rpm、`motor.shaft_velocity`由来のSimpleFOC内部推定値)、index2=`current_q`(0.001A/LSB、実測q軸電流)。CAN帰還送信は200Hz固定(`CAN_TX_PERIOD_MS=5`)、CAN受信はポーリングで`loop()`毎に即時反映（周期ゲート無し）。

**フェイルセーフ**: 起動時のFDCAN初期化失敗時のみ、モータを一切動かさず無限ループで停止し続けます（唯一のフェイルセーフ）。**ランタイムのCAN途絶に対するフェイルセーフ・enable信号・オーバースピードガードは実装されておらず**、最後に受信した `target_velocity` を保持し続けます（README.mdに明記）。

**実機で確認された注意点**: 診断ログ出力(`CAN_DIAG_ENABLE=1`)を有効にすると、Serial出力が`loop()`と同一スレッドで実行される都合上FOC制御ループをブロックし、500ms周期でトルクが「ガクッ」となる不具合が確認されています（`config.hpp`コメント）。通常運用では無効(0)のままにしてください。

制御フロー: `loop()` 内で `motor.loopFOC()`(電流ループ) → `canTaskUpdate()`(CAN送受信) → `apply_rx()`(target_velocity反映) → `motor.move()`(速度ループ) → `update_tx()`(テレメトリ生成) → `statusLedUpdate()` の順に毎回実行されます。

---

## 5. エンドツーエンドのデータフロー

### 5.1 指令送信（ROS/GUI → 実機動作）

```
外部ROSノード publish(serial_tx_[ID]) または GUI操作(device_panel)
  │
  ├─ 外部ノード経路: spin_timer(10ms) → RosBackend._on_hardware_tx_command()
  │     if not topic_passthrough: return          ← ゲート1
  │     tx_data更新 → HardwareManager.write() を即時実行
  │
  └─ GUI操作経路: tx_data更新（即座には送信しない）
        → publish_timer(20Hz) → RosBackend.publish_all_direct()
              for ch where direct_tx: publish_tx()  ← ゲート2
        │
        ▼
    SerialLink.write_data() → frame_codec.encode_frame()
      [0xAA][DEVICE_ID][LEN=48][24×int16 big-endian][XOR checksum] (52byte)
        │  (USBシリアル 115200bps, TIOCEXCL専有)
        ▼
  [xiao-esp32-s3_can2io] serialTask.receive_frame()
      1バイトステートマシンで同期・チェックサム照合 → Rx_16Data 更新（クリティカルセクション）
        │
        ▼ (MODE_CAN_HOST の場合)
    canTask: 5ms周期で Rx_16Data をスナップショット
      自ノード分 → CanIoRxData へ直接反映（CAN不要）
      他ノード分 → canSendNodeSlotBlock() で CAN 0x100+node*16+chunk 送信（1Mbps, 非ブロッキング）
        │  (CANバス)
        ▼
  [ノードマイコン(MODE_CAN)] canTask.canRecvNodeSlotBlock() → CanIoRxData反映
        │
        ▼
    IO_Task(5ms周期): サーボ/MD/TR へ実GPIO・PWM出力
        │  ※ MODE_ROBOMAS/MODE_CUBEMARS の場合は canTask を介さず
        │    robomasTask/cubemarsTask が Rx_16Data を直接読み、
        │    RoboMaster/CubeMars固有のCANプロトコルでモータへ直接送信(200Hz)
        ▼
  アクチュエータ動作
```

### 5.2 センサ受信（実機 → ROS/GUI）

```
実機センサ(スイッチ/エンコーダ/モータ帰還)
  │
  ▼ IO_Task(5ms) が CanIoTxData へ格納、または robomasTask/cubemarsTask が受信CANをデコード
[ノードマイコン] canTask: 5ms周期で CanIoTxData → canSendNodeSlotBlock(0x180+node*16+chunk) 送信
  │  (CANバス、1Mbps)
  ▼
[xiao-esp32-s3_can2io ホスト] canTask.canRecvAllNodeSlotBlocks()
  全ノード帰還を持続バッファへマージ（未受信スロットは前回値保持）
  → publishCanFeedbackToTxBuffer() で Tx_16Data 更新（クリティカルセクション）
  │
  ▼
serialTask.send_frame()(5ms周期): Tx_16Data スナップショット
  → [0xAA][DEVICE_ID][LEN=48][24×int16][XOR checksum] を Serial.write()
  │  (USBシリアル)
  ▼
[PC] SerialLink.read_frames() → FrameParser.pop_frame()（同期・チェックサム検証）
  → HardwareManager.frameReceived(device_id, values)
  │
  ▼
RosBackend._on_hardware_frame()
  ├─ rx_data更新 → publish(serial_rx_[ID])
  └─ CounterUnwrapper.update() ×24 → publish(serial_rx_[ID]_unwrapped)
        zeroed = unwrapped - zero_offset → publish(serial_rx_[ID]_zeroed)
  │
  ▼
rxUpdated シグナル → DevicePanel.refresh_from_rx()（Monitor/Rawタブ更新）
```

---

## 6. 安全機構・フェイルセーフの全体まとめ

`ros2can` システム全体を通して、**能動的なウォッチドッグ／タイムアウトベースのフェイルセーフはファームウェア側には実装されていません**。安全性は次の受動的な設計原則とPC側GUIの操作に委ねられています。

1. **「全ゼロ = 安全なデフォルト値」という規約**: `Rx_16Data` は初期値0で保持され、CubeMars/RoboMasterの `control_mode` は全ゼロ時に「速度ループ・target=0」を意味するようスロット番号が設計されている（電源投入直後や通信未確立時に位置ジャンプが起きない）。
2. **通信途絶時、ファームウェアは最後の指令値を保持し続ける**（UART・CANいずれも、タイムアウトによる自動ゼロ化は無い）。これは `b-g431-esc1_can2io`・`MODE_ROBOMAS`・`MODE_CUBEMARS` いずれにも共通する仕様であり、外部ノードを書く際は終了時に明示的にゼロ指令を送る責務がある（[7.4節 サンプルパッケージ](index.md#74-サンプルパッケージ-ros2can_examplec-ビルド実行可能)の `send_zero_and_stop()` 参照）。
3. **PC側 (`RosBackend.emergency_stop_all()`) がE-STOPの実体**: 全デバイスへゼロ指令を送りつつ `direct_tx`/`topic_passthrough` を両方OFFにして自動送信経路自体を止める。ファームウェア側に対応する専用処理は無い。
4. **CAN Bus-Off自動復帰**は通信健全性の維持機構であり、モータ出力を安全側に倒すものではない。
5. `CAN_NODE_COUNT` の過大設定によるBus-Off（3.2節）は設定ミスへの安全機構ではなく単なる既知の障害モード。
6. BOARD_SS（ソレノイドバルブ）の `TR1-4` は起動時に明示的に `LOW`（OFF）へ初期化される。

---

## 7. 既知の注意点・実装上のTODO

コード読解の過程で見つかった、ドキュメント化されていない/食い違いのある実装上の注意点です。今後の改修時の参考にしてください。

| 箇所 | 内容 |
|:---|:---|
| `firmware/b-g431-esc1_can2io/src/config.hpp` の `CAN_BITRATE` | `500000`固定だが、接続先の `xiao-esp32-s3_can2io` 側`MODE_CAN_HOST`は既に1Mbpsへ統一済み。**ビットレート不一致の可能性**（4節参照、実機接続前に要確認）。 |
| `firmware/b-g431-esc1_can2io/src/main.cpp` の `estimated_velocity`/`update_manual_velocity_estimate()` | 計算はされるがTXペイロードには使われていない（実際の速度帰還は`motor.shaft_velocity`由来）。デバッグ/将来拡張用の残存コードと推測される。 |
| `firmware/b-g431-esc1_can2io/src/main.cpp` の `currentSense.skip_align = true` | FOC初期化時の電流センスアライメント手順をスキップしているため、電流センスの符号・オフセットが実機ごとに変動する可能性がある。 |
| `firmware/xiao-esp32-s3_can2io/src/serial_task.cpp` 冒頭コメント | 例示に`DEVICE_ID: 0x02`とあるが、実際の`config.hpp`の値(101等)とは異なる古い記述が残存。実装自体には影響なし。 |
| `firmware/xiao-esp32-s3_can2io/src/defs.hpp` の `DEG_PER_COUNT`/`HALF_PPR` | 定義されているが`pin_ctrl_task.cpp`では未使用（raw pulse countをそのまま送信）。角度換算はPC側`device_profiles.py`の`scale`が担う設計。 |
| `firmware/xiao-esp32-s3_can2io` の `CanIoRxData`/`CanIoTxData` | `volatile`のみで明示的な排他制御なし（`Tx_16Data`/`Rx_16Data`にはクリティカルセクションがあるのに対し非対称）。int16単位の読み書きなので実害は限定的と推測されるが保証はない。 |
| `MODE_ROBOMAS`/`MODE_CUBEMARS` での `Tx_16Data`/`Rx_16Data` アクセス | `serialTask`と`robomasTask`/`cubemarsTask`がロック無しで直接共有（`can_task.cpp`側の`g_frame_lock`/`g_can_frame_lock`のような保護機構が使われていない）。 |
| `ros2can/ros_backend.py` の `topic_passthrough`/`direct_tx` 排他 | ROSバックエンド側にはコード上の相互排他が無く、GUIのチェックボックスレベルの実装のみ。`--nogui`モードは意図的に両方独立にON可能。 |

---

## 8. 主要ファイル索引

**PC側 (`ros2can/`)**

| ファイル | 役割 |
|:---|:---|
| `main.py` | エントリポイント、Qt/ROS 2イベントループ統合 |
| `ros_backend.py` | トピック管理、デバイスモード、E-STOP、原点セット |
| `hardware_manager.py` | シリアルポートのスキャン・専有・接続監視 |
| `serial_link.py` | シリアルI/O、TIOCEXCL排他制御 |
| `frame_codec.py` | 24スロットバイナリフレームのエンコード/デコード |
| `device_profiles.py` | プロファイル定義・スロットマッピング |
| `counter_unwrapper.py` | エンコーダのラップアラウンド展開 |
| `main_window.py` / `device_panel.py` / `widgets.py` | GUI本体 |
| `firmware_config.py` / `firmware_flash.py` / `firmware_dialog.py` | ファームウェア生成・書き込み |
| `settings_store.py` / `settings_dialog.py` | 設定の読み込み・保存 |

**ファームウェア (`firmware/xiao-esp32-s3_can2io/src/`)**

| ファイル | 役割 |
|:---|:---|
| `main.cpp` | setup/loop、タスク生成、モード分岐 |
| `config.hpp` / `defs.hpp` | コンパイル時設定・ピン定義 |
| `frame_data.cpp/hpp` | 24スロットデータ実体 |
| `serial_task.cpp/hpp` | UARTフレームプロトコル |
| `can_task.cpp/hpp` | CANフレームプロトコル、ノード分配 |
| `pin_ctrl_init.cpp/hpp` / `pin_ctrl_task.cpp/hpp` | GPIO/PWM/エンコーダ処理 |
| `cubemars.cpp/hpp` | CubeMars AKシリーズ MIT/位置/速度制御 |
| `robomas.cpp/hpp` | DJI RoboMaster 電流指令・位置PD制御 |
| `status_led.cpp/hpp` | 状態LED |

**ファームウェア (`firmware/b-g431-esc1_can2io/src/`)**

| ファイル | 役割 |
|:---|:---|
| `main.cpp` | SimpleFOC初期化、TIM4エンコーダ、制御ループ |
| `config.hpp` | モータ/CAN/PIDパラメータ |
| `can_task.cpp/hpp` | CANノード分配プロトコル（xiaoホストと共通方式） |
| `frame_data.cpp/hpp` | 5スロットデータ実体 |
| `status_led.cpp/hpp` | 状態LED |

---

## 9. 付録: `serial_bridge`（前身パッケージ）の内部実装

> `serial_bridge` は `ros2can` の前身にあたる ROS 2 パッケージで、CANによるノード分配を持たず、マイコン1台につきUSBシリアル1本を直接ブリッジする「星型配線」構成です（[1.1 serial_bridgeとの比較](index.md#11-serial_bridgeとの比較)参照）。`ros2can` の `frame_codec.py`/`serial_task.cpp` が踏襲している「24スロットint16フレーム・START_BYTE(0xAA)によるバイト単位再同期・XORチェックサム」という設計は、すべて `serial_bridge` に由来するものであり、`ros2can` の基盤となった技術です。本節は開発当時の設計文書（社内解説記事）と `serial_bridge` 側のソース（`src/`, `include/serial_bridge/`, 代表ファームウェア `firmware/esp32_serial_bridge/`）を突き合わせてまとめたものです。

### 9.1 開発背景・設計思想

学ロボ25では無線（UDP）通信を採用していましたが、電波干渉により会場での通信が不安定になるという問題があり、学ロボ26では有線通信への切り替えが目標になりました。ロボコンでの有線通信はEtherCAT・CANが主流ですが、EtherCATは導入コストが大きく、CANは導入コストは低いものの開発コスト（CAN対応MD、CAN⇔各アクチュエータの変換など）が高いと判断され、いずれも当時の開発体制（開発者1名・低予算）には見合わないとして採用が見送られました。

代わりに選ばれたのが**シリアル通信 + ESP32（Arduino言語）**という構成です。決定にあたっては以下の要件が明示的に定められていました。

- 現在のリソース（開発者1名）で開発できること
- 低予算で実装可能なもの
- 追加のハードウェアが少ないこと（既存のハードをそのまま活用できること）
- 高速・軽量であること
- 学習コストが低いこと（できればArduino言語で）
- ROS依存でないこと
- 有線通信であること
- 汎用性が高いこと（別の大会・機体でも流用できる）

マイコンにSTM32ではなくESP32を選んだのも、「他大学ではSTM32が主流だが、CubeIDEの学習コストが高すぎて断念」という、同じ「1回生が大多数」という体制上の制約に基づく判断です。通信部分（フレームプロトコル）自体は特定マイコンに依存しないシンプルな設計にすることが最初から意図されており、後年 `stm32_serial_bridge`・`b-g431-esc1_simplefoc_serial_bridge` など他マイコン向けファームウェアへの移植や、`ros2can` 側での再設計・拡張が可能になった土台はここにあります。

### 9.2 ポート番号問題とID方式・仲介ノードという設計判断

Linux上のシリアルポートはUSBに挿した順番で `/dev/ttyUSBn` の番号が割り当てられるため、「機体側の各制御ノードが個別に決め打ちのポート番号を扱う」設計では、マイコンを挿す順序が変わるたびに対応関係が崩れます（起動のたびに順番通り挿し直すのは非現実的、という課題として明記されています）。

この問題への対応として、次の2段階の設計判断がなされました。

1. **マイコン側にIDを持たせ、常時PCへ送信し続ける**: ポート番号ではなくフレームに埋め込まれたIDでマイコンを識別することで、挿す順序に依存しなくなる。
2. **ポート操作を単一の仲介ノードに集約する**: 各ノードが個別に「ポートを開いてIDを確認し、違えば閉じて次のポートを試す」という探索を行うと、複数ノードのポートオープン/クローズのタイミングが互いに干渉し、正常に動作しない。そのため、ポートの走査・オープン・ID確認を一手に引き受ける専用ノード（`serial_bridge` 本体）を用意し、他の制御ノードは `serial_tx_[ID]`/`serial_rx_[ID]` というトピックのみを介してやり取りする。

この「ポート専有主体を1つに集約する」という設計判断は `ros2can` にもそのまま引き継がれており、`hardware_manager.py` が単一の `HardwareManager`（GUIプロセス内で唯一シリアルポートを直接握る主体）としてスキャン・オープン・専有を一元管理する構造の原型になっています（[2.4節](#24-ハードウェア検出管理-hardware_managerpy)）。

### 9.3 通信フレームの設計

フレーム構造は「スタートバイト（フレーム破損時の復帰点）」「ID（マイコンの識別）」「ペイロード長（受信データの処理に利用）」「ペイロード（アクチュエータ指令値・センサ値）」「チェックサム（受信フレームの検証、破損がないか確認）」という役割分担で設計されています。ペイロードの各値はノード上では16bitとして扱われますが、シリアル通信で送信する際には8bit（1バイト）×2に分割されます（ビッグエンディアン）。

代表ファームウェア `esp32_serial_bridge` における標準スロット割当は以下の通りです。

| 方向 | スロット | 用途 |
|:---|---:|:---|
| ROS 2 → マイコン | 1–8 | DCモーターの指令値（-255〜255） |
| ROS 2 → マイコン | 9–16 | PWMサーボの指令値（0〜270） |
| ROS 2 → マイコン | 17–23 | トランジスタアレイへの指令値（0 or 1） |
| マイコン → ROS 2 | 1–8 | エンコーダーのパルス |
| マイコン → ROS 2 | 9–16 | マイクロスイッチ（デジタル入力） |
| マイコン → ROS 2 | 17–23 | その他センサのために予約 |

サーボ指令値のレンジ「0〜270」は、`ros2can` の `device_profiles.py` におけるサーボ既定レンジ（0〜270deg、[9.2節](index.md#92-mitモードの活用)の角度クランプ議論でも参照される値）と一致しており、単なる偶然ではなく `serial_bridge` 時代からの値がそのまま踏襲されていることが伺えます。

このスロット割当は `ros2can` の `device_profiles.py` のような可変プロファイルではなく、ファームウェアターゲットごとに固定された単一の取り決めです。どのスロットが何を意味するかを解釈する仕組み（GUI等）は `serial_bridge` 自体には無く、この規約を踏まえて各制御ノードを自作する前提になっています。

### 9.4 ROS 2側: 常駐スキャンスレッドと動的ノード生成（`main.cpp`）

`main.cpp` は次の要素で構成されます。

- `rclcpp::init()` 後、`excluded_ports`/`rx_timeout_sec`/`reconnect_interval_sec`/`scan_interval_ms` をパラメータとして読み込む `debug_node`（ステータス確認用の空ノード）を生成し、`MultiThreadedExecutor` に登録する。
- `kLogOutputMode` が `kNone`/`kGraphical` の場合、`rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_FATAL)` でデフォルトロガーの重大度をFATALまで引き上げる。**この設定はノード個別のロガーにも及ぶ**ため、`kGraphical` モードでは `bridge_node.cpp` 内の通常の `RCLCPP_INFO`/`RCLCPP_WARN`（`verbose_packet_log` によるフレームダンプ含む）も実質的に画面へ出力されなくなる点に注意（見えるのは `graphical_ui` が直接 `std::cout` へ書き込むASCIIダッシュボードのみ）。
- バックグラウンド `std::thread`（スキャナスレッド）を1本起動し、`node_map`（ID→`SerialBridgeNode`）・`known_ports` を `std::mutex`（`map_mutex`）で保護しながら共有する。ループごとに:
  1. `known_ports` を「既に使用中のポート」としてコピーし、`detect_serial_devices(skip, excluded_ports)` を呼ぶ。
  2. 検出結果を走査し、未知のIDなら `SerialBridgeNode` を新規生成して `node_map`/`known_ports`/`executor` に登録。既知だが切断中のIDが別ポートで再検出された場合は、旧ノードを `executor.remove_node()` してから新ノードを生成・登録（差し替え）。接続中の既知IDは何もしない。
  3. `scan_interval_ms`（既定5000ms）を100ms刻みで待機し、シャットダウン要求（`running`/`rclcpp::ok()`）に迅速に反応できるようにする。
- コード中のコメントには「`is_connected()` はatomicだが `map_mutex` とは同期しないため、接続直後のノードを稀に置き換えてしまう可能性があるが、新ノードが即座に再接続するため実害はない」という既知の（許容された）レースコンディションが明記されています。

これは `ros2can` の `HardwareManager`/`_ScannerThread`（[2.4節](#24-ハードウェア検出管理-hardware_managerpy)）の直接の前身ですが、アーキテクチャは異なります。`ros2can` はQtの単一イベントループ+タイマーでロックを一切使わない設計に移行済みですが、`serial_bridge` は古典的な「ROS 2 `MultiThreadedExecutor` + 素の `std::thread` + `std::mutex`」というマルチスレッドモデルのままです。

### 9.5 ポートスキャン（`port_scanner.cpp`）

- `list_serial_ports()`: `/dev/ttyUSB*`・`/dev/ttyACM*` を `glob()` で列挙する。
- `detect_serial_devices(skip_ports, excluded_ports)`: 列挙したポートのうち `skip_ports`（他ノードが既に専有中）・`excluded_ports`（設定ファイルで除外指定）に含まれるものを飛ばし、残りを1つずつ:
  1. `open_serial_port()`: `open()`(raw) → `cfmakeraw` → 115200bps → `VMIN=0`/`VTIME=1`(0.1秒) → `usleep(500000)`（USB CDCの安定待ち、0.5秒）。
  2. `read_frame(fd, frame, timeout_ms=2000)`: 1バイトずつ読み、`START_BYTE` を待ってからLENGTHを取得し（フレーム全長 `3+LEN+1` が `MAX_FRAME_SIZE=256` を超えれば失敗）、チェックサム（ID〜DATAのXOR）を検証してから `Frame{id, data}` を返す。最大2秒でタイムアウト。
  3. 成功すれば `result[frame.id] = port`。
- **排他制御（`TIOCEXCL`等）は一切使われていません**。`ros2can` 側で既に述べられている通り（[2.3節](#23-シリアルリンク層と排他制御-serial_linkpy)）、これが「`ros2can` が先にポートを掴んでいれば `serial_bridge` の `open()` は失敗するだけで済むが、逆方向は防げない」という非対称な保護の根本原因です。
- 走査1周分のコストは、未接続・非対応ポートが多いと「ポート数 × 最大約2.5秒（0.5秒安定待ち+最大2秒タイムアウト）」に達し得ます。この走査はバックグラウンドスレッドから `scan_interval_ms`（既定5秒）ごとに直列実行されます。

### 9.6 SerialBridgeNode: 接続管理・RX・TX（`bridge_node.cpp`）

1デバイス=1 `rclcpp::Node`（`serial_bridge_<id>`）です。`serial_rx_<id>`（Publish）/`serial_tx_<id>`（Subscribe）を持ち、`kUpdatePeriodMs=5ms` のwall timerが `update()`（再接続判定+RX）を駆動します。TXは**タイマーではなくsubscriptionコールバック駆動**（`tx_callback()`）で、メッセージが来た瞬間にその場で書き込みます。

- **`try_open_port()`**: `close_port()` → `open()`(`O_RDWR|O_NOCTTY|O_SYNC`) → `cfmakeraw`+115200bps+`VMIN=0`/`VTIME=0`（非ブロッキング）→ `tcflush(fd_, TCIOFLUSH)`。このflushは「スキャナが同じポートを一時的に開いて読み込んだ後に残ったバイト列や、切断前の途中フレームがバッファに残っていると START_BYTE 同期が取れなくなる」ことを明示的に防ぐための処理です（コードコメントに明記）。
- **`update()`**: 未接続なら `reconnect_interval_sec`（既定3.0秒）間隔で再接続を試行。接続中なら `read()` で最大 `kReadBufferSize=512` バイトを読み込み、`rx_buffer_`（`std::deque<uint8_t>`）へ追加します。**このバッファは以前は関数内の `static` 変数でしたが、「セグフォ防止のため」メンバ変数へ移動された経緯がコメントに残っています**（過去に踏んだ既知の不具合）。読み取り0バイトが `rx_timeout_sec`（既定2.0秒）続くと切断扱い、`EIO`/`ENODEV`/`ENXIO` の読み取りエラーは即切断扱いです。
- フレーム抽出は1回の `update()` あたり最大 `MAX_FRAMES=64` フレームまでという上限付きで、バッファが尽きるかフレーム未充足になるまでループします（1デバイスの大量データがexecutor全体を長時間占有しないための上限）。
  - 先頭が `START_BYTE` でなければ1バイト破棄。
  - `LENGTH` からフレーム全長を計算し、バッファがまだ足りなければ `return`（次回受信を待つ）。
  - チェックサム不一致は1バイト破棄して再走査。
  - **ID不一致の場合は1バイトではなく検証済みフレーム全体（`frame_size`バイト）を破棄**します（チェックサムで整合性が確認済みの「別デバイス宛の正常なフレーム」なので、1バイトずつ捨てるより安全かつ効率的にスキップできます）。
  - 全て一致すれば24個の `int16`（ビッグエンディアン）にデコードし `Int16MultiArray` として `serial_rx_<id>` へPublish、`verbose_packet_log` が有効なら内容をログ出力。
- **`tx_callback()`**: `fd_<0` または受信データ長が24未満なら黙って破棄。それ以外はフレーム（`[0xAA][ID][LEN=48][24×int16 big-endian][XOR checksum]`）を組み立てて即座に `write()`。**周期的な再送や直近値の保持・自動再送機構は無く**、外部ノードが `serial_tx_<id>` へのPublishを止めれば（クラッシュ等）、マイコン側は最後に受信した指令を保持し続けます（`ros2can` の `direct_tx` 20Hzの周期送信のような仕組みは存在しません）。
- **`maybe_log_status()`**: `status_log_period_ms`（既定100ms、下限100ms）ごとにRXレート・帯域・バス利用率（115200bpsに対する `(rx+tx)Bps×10bit/byte` の割合、`kUartBitsPerByte=10.0`）を集計します。`kGraphical` モードでは1デバイス1行のASCIIバー付き固定行として `graphical_ui` へ、それ以外は `RCLCPP_INFO` 2行として出力します。

### 9.7 グラフィカルUI（`graphical_ui.hpp`）

`kGraphical` モード専用の、ANSIエスケープシーケンスによる簡易ターミナルダッシュボードです。`kGraphicalUiFrameMs`（既定100ms）ごとに動作する専用アニメーションスレッド（`std::thread` + `std::mutex` で状態を保護）が画面全体を `\033[H\033[2J` で再描画し、スキャン状況・検出済みID一覧・デバイスごとの状態行（接続状態`[ON]`/`[OFF]`、RXレートのASCIIバー、送受信カウンタ、チェックサム/IDミスマッチ/ドロップ数、バッファ長、累積バイト数など）を1画面にまとめて表示します。行の色は利用率・エラー数から動的に判定されます（緑=低負荷無エラー→黄→オレンジ→赤=高負荷またはエラー多数）。`ros2can` にはこれに相当するターミナルダッシュボードは無く、代わりにPyQt5のフルGUI（Monitor/Rawタブ等）が同等の役割を担います。

### 9.8 マイコン側: FreeRTOSタスク構成（`firmware/esp32_serial_bridge`）

代表ターゲット `esp32_serial_bridge` の `main.cpp`/`serial_task.cpp` は次の構成です。

- 起動時: `Serial.begin(115200)` → `delay(200)` → `delay(100 * DEVICE_ID)`（IDごとに起動タイミングをずらす、複数マイコン同時起動時のバス輻輳回避）→ LEDを `DEVICE_ID` 回点滅（自身のIDを目視確認できるようにする自己診断）→ `serialTask` を生成（スタック2048語・優先度10）。
- `serialTask` は1つのFreeRTOSタスク内でRX（`receive_frame()`、毎ループ呼び出し）とTX（`send_frame()`、`TX_PERIOD_MS` 経過判定つき）の両方を処理し、ループ末尾で `vTaskDelay(1ms)` します。`TX_PERIOD_MS` は既定100msですが、`MODE_INPUT`/`MODE_ROBOMAS_PLUS_INPUT` では20msに短縮されます（入力系はセンサ値の更新をより高頻度に送りたいため）。
  - RXは `WAIT_START → WAIT_ID → WAIT_LEN → WAIT_DATA → WAIT_CHECKSUM` のバイト単位ステートマシンで、`LENGTH` が `Rx16NUM×2` を超える等の異常があれば即座に `WAIT_START` へ戻ります。チェックサム一致かつID一致の場合のみ `Rx_16Data[]` へ反映されます。
- `config.hpp` で選択する8種類の `MODE_*`（`MODE_OUTPUT`/`MODE_INPUT`/`MODE_IO`/`MODE_ROBOMAS`/`MODE_ROBOMAS_PLUS_OUTPUT`/`MODE_ROBOMAS_PLUS_INPUT`/`MODE_ROBOMAS_PLUS_IO`/`MODE_DEBUG`）はちょうど1つだけ定義する必要があり、0個または2個以上の定義は `#error` でコンパイルエラーになります（`ros2can` の `xiao-esp32-s3_can2io` ファームウェアにおける同様の排他チェック、[3.1節](#31-freertosタスク構成とモード分岐)と同じ設計パターン）。
- `Output_Task`/`Input_Task`（`MODE_IO`では統合された `IO_Task`）は5ms周期（`vTaskDelayUntil`）で、`Rx_16Data[]`/`Tx_16Data[]` を介してGPIO（サーボ角度→パルス幅→duty変換、モータのPWM+DIR出力、トランジスタON/OFF、エンコーダのPCNT読み取り、スイッチのdigitalRead）を処理します。
- `stm32_serial_bridge`・`xiao_esp32_s3_(smd_)serial_bridge`・`esp32_s3_serial_bridge`・`arduino_uno_serial_bridge`（WIP）・`b-g431-esc1_simplefoc_serial_bridge` など他ターゲットも同一のフレームプロトコル・FreeRTOSタスク構成パターン（`serial_bridge/docs/flow_chart.md` 参照）を踏襲しています。

### 9.9 `ros2can` との主要な差分

| 観点 | `serial_bridge` | `ros2can` |
|:---|:---|:---|
| 配線 | マイコン1台＝USB1本の星型 | CANホスト1台（USB1本）＋CANバスでのデイジーチェーン |
| ポート専有 | 排他フラグなし | `TIOCEXCL`（片方向のみ保護、[2.3節](#23-シリアルリンク層と排他制御-serial_linkpy)） |
| プロセス/スレッドモデル | `MultiThreadedExecutor`＋独立`std::thread`（`mutex`で共有状態を保護） | Qt単一イベントループ＋`QTimer`（ロック無し）、スキャンのみ`QThread` |
| UI | ターミナルのみ（テキスト/ASCIIグラフィカル/サイレントの3モード） | PyQt5フルGUI（Control/Monitor/Raw/Info） |
| TXの周期送信 | 無し（Subscribeの都度、即時write） | あり（`direct_tx`ON時、20Hzで`publish_all_direct()`が周期送信） |
| スロットの意味付け | ファームウェアターゲットごとに固定（TX1-8=DCモータ等） | `device_profiles.py`による可変プロファイル、CANノードへの動的分配 |
| エンコーダのラップアラウンド展開 | 無し（生パルス値をそのままPublish） | あり（`counter_unwrapper.py`、[2.7節](#27-エンコーダunwrapアルゴリズム-counter_unwrapperpy)） |
| 実機不要の動作確認 | 無し | あり（仮想デバイス機能） |
| CubeMars/RoboMaster専用ドライバ | 無し | あり（MITモード等、[9章](index.md#9-cubemars-akシリーズmode_cubemars)/[10章](index.md#10-djiロボマスmode_robomas)） |

いずれも `ros2can` が、`serial_bridge` の実運用（学ロボ25の無線通信不安定化を受けた有線化、ポート番号問題とID方式、単一仲介ノードによる排他）から得た知見を土台に、CAN化とGUI化によって拡張したものです。

### 9.10 `serial_bridge` 側 主要ファイル索引

| ファイル | 役割 |
|:---|:---|
| `src/main.cpp` | エントリポイント、`MultiThreadedExecutor`、バックグラウンドスキャンスレッド |
| `src/port_scanner.cpp` | ポート走査・プローブ受信によるID検出 |
| `src/bridge_node.cpp` | 1デバイス1ノード、フレームの送受信・再接続・ステータスログ |
| `include/serial_bridge/config.hpp` | フレーム定数・ログ出力モード・グラフィカル表示項目の設定 |
| `include/serial_bridge/graphical_ui.hpp` | ASCIIダッシュボード（`kGraphical`モード専用） |
| `firmware/esp32_serial_bridge/src/main.cpp` | setup/loop、`MODE_*`分岐、タスク生成 |
| `firmware/esp32_serial_bridge/src/serial_task.cpp` | UARTフレームの送受信ステートマシン |
| `firmware/esp32_serial_bridge/src/pin_ctrl_task.cpp` | GPIO（サーボ/モータ/TR/エンコーダ/スイッチ）処理 |

---

[← マニュアルに戻る](index.md) ｜ [クイックスタート](quickstart.md)
