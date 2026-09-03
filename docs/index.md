---
title: ros2can マニュアル
---

> `ros2can` のマニュアルです。`xiao_esp32_s3_smd_serial_bridge`（MODE_CAN_HOST）
> 専用のスタンドアローンGUIで、シリアルポートのスキャン・専有・serial_bridge
> 互換フレームの送受信を自前で行います。

## 目次

- [1. 概要](#1-概要)
- [2. システム要件](#2-システム要件)
- [3. インストールと起動](#3-インストールと起動)
- [4. 起動直後の画面](#4-起動直後の画面)
- [5. デバッグモード（実機不要でのUI確認）](#5-デバッグモード実機不要でのui確認)
- [6. 画面構成](#6-画面構成)
  - [6.1 Control タブ（指令送信）](#61-control-タブ指令送信)
  - [6.2 Monitor タブ（センサ受信）](#62-monitor-タブセンサ受信)
  - [6.3 Raw タブ（全24スロット）](#63-raw-タブ全24スロット)
  - [6.4 Info タブ](#64-info-タブ)
- [7. プロファイル](#7-プロファイル)
- [8. CubeMars AKシリーズ（MODE_CUBEMARS）](#8-cubemars-akシリーズmode_cubemars)
- [9. DJIロボマス（MODE_ROBOMAS）](#9-djiロボマスmode_robomas)
- [10. ファームウェア生成・書き込み](#10-ファームウェア生成書き込み)
- [11. 設定](#11-設定)
- [12. エンコーダ初期化](#12-エンコーダ初期化)
- [13. 安全機能について](#13-安全機能について)
- [14. About](#14-about)

---

## 1. 概要

`ros2can` は、CANバス経由で複数マイコンをホストする `xiao_esp32_s3_smd_serial_bridge`
（MODE_CAN_HOST）専用の ROS 2 GUI パッケージです。

この基板は USB シリアルでつながる「CANホスト」で、自身の配下に CAN バス経由で
最大4台の子マイコン（ノード）をデイジーチェーン接続可能です。`ros2can` はホストの
シリアルポートを直接掴み、バス上の各ノードへアクチュエータ指令を直接送信したり、
センサ値をリアルタイムに表示することができます。また、`serial_bridge` と同様に
トピックを用いた外部ノードとの接続も可能です。

```
  [ros2can (GUI, スタンドアローン)]
       │
       ├─ port_scanner ─── /dev/ttyUSB*, /dev/ttyACM* を探索
       │
       └─ USBシリアル ──► CANホスト (MODE_CAN_HOST)
                             │
                             └─ CANバス
                                  ├─ ノード1 (CAN_ID=101)
                                  ├─ ノード2 (CAN_ID=102)
                                  ├─ ノード3 (CAN_ID=103)
                                  └─ ノード4 (CAN_ID=104)
```

ホストのUSBシリアル側は常に `serial_bridge` 互換の24 x int16スロット
（TX: ROS→ホスト、RX: ホスト→ROS）を1フレームとしてやり取りします。この24スロットを
CANバス上の各ノードへどう割り当てるかを決めるのが後述の「[7. プロファイル](#7-プロファイル)」です。

`xiao-esp32-s3_can2io` ファームウェア自体は、ノードを束ねる「CANホスト」以外にも、
CANバスに直接ぶら下がる独立デバイスとしていくつかのモードを持っています。

| ファームウェア MODE | 役割 | 対応するros2canプロファイル |
|:---|:---|:---|
| `MODE_CAN_HOST` | 複数ノードを束ねるCANホスト本体（本マニュアルの主対象） | XIAO ESP32S3 SMD/MES/SS (CAN Host) |
| `MODE_CAN` | ホスト配下の汎用IOノード（通常の子マイコン） | （CAN Host側のプロファイルに内包） |
| `MODE_ROBOMAS` | DJIロボマス最大4台を直接駆動する独立デバイス | [xiao-esp32-s3_can2io (MODE_ROBOMAS, DJIロボマス x4)](#9-djiロボマスmode_robomas) |
| `MODE_CUBEMARS` | CubeMars AKシリーズ最大4台を直接駆動する独立デバイス | [xiao-esp32-s3_can2io (MODE_CUBEMARS, CubeMars AKシリーズ x4)](#8-cubemars-akシリーズmode_cubemars) |
| `MODE_IO` | CAN無し、IOのみのスタンドアロン動作 | 汎用 Raw 等 |
| `MODE_CAN_MONITOR` | 生CANフレームをシリアル出力するだけの診断用モード | （ros2canのデバイス一覧には出ない。「CANモニター…」参照） |
| `MODE_DEBUG` | デバッグ用 | - |

`MODE_ROBOMAS` / `MODE_CUBEMARS` は「CANホストにぶら下がるノード」ではなく、
それ自体が独立したCANデバイスとして動作します（ホストを介さずCANバスへ
直接接続）。ros2can上ではそれぞれ専用のプロファイルを選ぶことで、通常の
CANホスト機と同じ画面（Control/Monitor/Raw/Info）から操作できます。

## 2. システム要件

| 項目 | 内容 |
|:---|:---|
| OS | Ubuntu 24.04 LTS |
| ROS | ROS 2 Jazzy |
| GUI | PyQt5 |
| ボーレート | 115200 bps |
| ハードウェア | `xiao_esp32_s3_smd_serial_bridge`（MODE_CAN_HOST）を書き込んだ基板を USB 接続 |
| 追加パッケージ | `python3-pyqt5`, `python3-serial` |

> `/dev/ttyUSB*` や `/dev/ttyACM*` を使用するには `dialout` グループへの追加が必要です。
> `sudo usermod -aG dialout $USER`（反映には再ログインが必要）

## 3. インストールと起動

```bash
cd ~/ros2_ws
colcon build --packages-select ros2can
source install/setup.bash
```

起動（`config/ros2can.yaml` のパラメータを読み込む場合は launch を使用）:

```bash
ros2 run ros2can ros2can
# または
ros2 launch ros2can ros2can.launch.py
```

起動すると `ros2can` 自身がバックグラウンドスレッドで `/dev/ttyUSB*` /
`/dev/ttyACM*` を定期的にスキャンし、CANホストを検出すると自動的に
シリアルポートを専有してデバイス一覧に表示します。

## 4. 起動直後の画面

マイコン未接続の状態では、下図のようなプレースホルダー画面が表示され、
接続手順やトラブルシューティングの案内が表示されます。

![起動直後（マイコン未検出）](images/01_startup_placeholder.png)

ツールバーの各ボタンから、以下の操作ができます。

| ボタン | 用途 |
|:---|:---|
| 再スキャン | ポートを即座に再スキャンする |
| デバイスを手動追加… | 既存トピックへ相乗りする形でデバイスを事前登録する |
| デバッグデバイスを追加（実機不要）… | 実機なしで動作確認できる仮想デバイスを追加する |
| エンコーダ初期化… | 全デバイス横断でエンコーダ/位置カウンタの原点セットを行う |
| CANモニター… | `MODE_CAN_MONITOR` 機の生CANフレームを直接閲覧する |
| 通信ログ… | 接続/切断やフレーム異常の履歴を確認する |
| 設定… | ハードウェアスキャンの挙動（除外ポート等）を編集する |
| ファームウェア生成・書き込み… | パラメータを反映したファームウェアを生成し、実機に書き込む |
| ■ 全デバイス E-STOP | 全接続デバイスへゼロ指令を送信し、送信を無効化する（緊急時に押す） |

## 5. デバッグモード（実機不要でのUI確認）

マイコン実機が手元に無くても、UIのレイアウト調整やウィジェットの動作確認が
できるよう、ツールバーの「デバッグデバイスを追加（実機不要）…」から**仮想デバイス**
を追加できます。DEVICE_ID を入力するだけで、その場で仮想デバイスが一覧に追加されます。

![デバッグデバイスを追加した直後](images/02_device_added.png)

- Control タブでスライダー等を動かすと、書き込んだ値が仮想デバイスのRXに反映され、
  Monitor / Raw / Info タブにリアルタイムに表示されます（ただしスイッチ・エンコーダ・
  故障コード等、実機では指令と無関係な独立したセンサ入力に相当するRXは、この
  自動ループバックの対象外になります。詳細は [6.2 Monitor タブ](#62-monitor-タブセンサ受信) 参照）。
- 実機接続時と同じく `serial_rx_[ID]` を Publish / `serial_tx_[ID]` を Subscribe
  するため、rqt や他の ROS ノードからのテストにもそのまま使えます。
- デバイス一覧でデバイスを右クリックすると「このデバイスを削除」で取り除けます。
- デバッグデバイス追加時にプロファイルを指定すれば、CubeMars / ロボマス構成の
  UIもハードウェアなしでそのまま確認できます（本マニュアルの画面キャプチャも
  すべてこのデバッグモードで取得しています）。

## 6. 画面構成

デバイスを選択すると、右側に **Control / Monitor / Raw / Info** の4タブから
なるデバイスパネルが表示されます。CANホスト系プロファイル（ノード構成を持つもの）
ではさらにノード1〜4のサブタブに分かれますが、CubeMars/ロボマスのように
「ノード分配を行わない独立デバイス」のプロファイルでは、モータ(M1〜M4)ごとに
グループ化された1画面になります。

### 6.1 Control タブ（指令送信）

アクチュエータ（サーボ・モータ等）へ直接送信するタブです。送信には
「ダイレクト送信」チェック（既定OFF、誤操作防止）が必要です。「トピック通過」
（既定ON）をOFFにすると外部ノードからの `serial_tx_[ID]` を無視し、この
パネルの値のみが有効になります。

![Controlタブ: スライダーで指令値を編集し、ダイレクト送信ON](images/03_control_tab.png)

### 6.2 Monitor タブ（センサ受信）

センサ（スイッチ・エンコーダ等）の値をリアルタイム表示するタブです。

デバッグ（仮想）デバイスでは、RXチャンネルの種類によって挙動が異なります。

- **スイッチ・カウンタ・選択式入力**（ON/OFFスイッチ、エンコーダのカウント値、
  CubeMarsのエラーコード等）は、実機では指令(TX)と無関係な独立したセンサ入力に
  相当するため、この画面から**直接編集可能な行**として表示されます。ここで
  設定した値はTX→RXの自動ループバック対象から外れ、次に変更するまで保持されます。
- **数値表示のみのRX**（角度・速度・電流などのREADOUT系)は、Control タブで
  書き込んだTXの生値がそのままそのスロット番号のRXにループバックされ、
  読み取り専用で表示されます（後述のCubeMars/ロボマスのMonitorタブで見える
  数値は、対応するTXスロットの生値がそのまま流れ込んだものです）。

各チャンネルの「原点セット」ボタンで、その場の値をソフトウェア上のゼロ点として
記録できます（[12. エンコーダ初期化](#12-エンコーダ初期化) 参照）。

![Monitorタブ: スイッチON・エンコーダカウントを手動設定した状態](images/04_monitor_tab.png)

### 6.3 Raw タブ（全24スロット）

CAN分配を介さず、生の24 x int16スロットを直接編集/確認できます。
左がTX（ROS → ホスト → CAN、編集可能）、右がRX（CAN → ホスト → ROS、読み取り専用）です。
プロファイルの定義が想定通りか怪しいときや、未定義のプロファイル外スロットを
直接触りたいときに使用します。

![Rawタブ: 24スロットのTX/RXを一覧表示](images/05_raw_tab.png)

### 6.4 Info タブ

接続状態・RX周波数・送受信フレーム数・生データ配列を表示します。
通信の問題を切り分けたいときに参照してください。

![Infoタブ: 接続状態やフレームカウンタの詳細](images/06_info_tab.png)

## 7. プロファイル

「プロファイル」は、ホストのUARTフレーム24スロットをどう解釈するかを定義する
設定セットです。デバイスパネル上部の「プロファイル:」ドロップダウンから
デバイスごとに切り替えられます。組み込み(builtin)プロファイルは以下の通りです。

| プロファイルキー | 名前 | 概要 |
|:---|:---|:---|
| `xiao_smd_can_host` | XIAO ESP32S3 SMD (CAN Host) | 既定プロファイル。24スロットを4ノード x 5スロットに分配（SERVO1-3+MD1-2指令 / SW1-3+ENC1-2帰還）。SERVOn/SWn、ENCn/MDnはファームウェア側でピン共有 |
| `xiao_can2io_with_foc` | xiao-esp32-s3_can2io + b-g431-esc1_can2io (FOCモータ, robomas互換) | 2ノード構成のうち1ノードをFOCモータ(SimpleFOC、速度制御のみ)用チャンネルに置き換え |
| `xiao_mes_can_host` | XIAO ESP32S3 MES (CAN Host) | BOARD_MES用。各ノードMD1-2(指令)/SW1+ENC1-2+SW3(帰還)。ENC2とSW2/SW3はピン共有(ファーム側config.hppで排他固定) |
| `xiao_ss_can_host` | XIAO ESP32S3 SS (CAN Host) | BOARD_SS用。SERVO1-5+TR1-4(ソレノイドバルブ)の9指令チャンネルのため1ノード9スロットに拡張、24スロットの制約でノード数は既定2まで |
| `robomas_driver` | xiao-esp32-s3_can2io (MODE_ROBOMAS, DJIロボマス x4) | ノード分配なし。独立デバイスとしてDJIロボマス最大4台を直接制御。詳細は[9章](#9-djiロボマスmode_robomas) |
| `cubemars_ak_driver` | xiao-esp32-s3_can2io (MODE_CUBEMARS, CubeMars AKシリーズ x4) | ノード分配なし。独立デバイスとしてCubeMars AKシリーズ最大4台を直接制御。詳細は[8章](#8-cubemars-akシリーズmode_cubemars) |
| `generic_raw` | 汎用 Raw (24スロット、CAN分配なし) | ホストのUARTフレーム24スロットをそのまま直接編集/表示する |

ファームウェア側の `config.hpp` で `CAN_NODE_COUNT` / `CAN_SLOTS_PER_NODE`
（BOARD_VARIANTごとに値が異なる）を変更した場合や、独自のノード構成を
使う場合は、デバイスパネル右上の「プロファイル編集」からノード数・スロット数を
指定して雛形を自動生成し、ラベルやレンジを調整して保存してください。
カスタムプロファイルは `~/.config/ros2can/profiles/` にJSONとして保存され、
次回起動時にも読み込まれます（リポジトリ外のためGit管理対象にはなりません）。

### 対応スロットマッピング（既定プロファイル `xiao_smd_can_host`）

```
実機はDCモータ非搭載 (ENCx2, SWx3, SERVOx3のみ)。
1ノードあたり5スロット (CAN_SLOTS_PER_NODE=5):
  指令 (ROS -> ホスト -> CAN -> ノード): SERVO1, SERVO2, SERVO3, (予備, 予備)
  帰還 (ノード -> CAN -> ホスト -> ROS): SW1, SW2, SW3, ENC1, ENC2

SERVOn と SWn はピン共有 (ファームウェア config.hpp の MULTIn で切替、
0=スイッチ入力/1=サーボ出力)。ENCn と MDn もピン共有 (config.hpp の ENCn_MD で
切替、0=エンコーダ入力/1=MD(PWM+DIR)出力)。

グローバルスロット index = node_index(0-origin) * 5 + local_index
ノードの CAN_ID は 101,102,103,104 (下2桁 = ノード番号)
```

## 8. CubeMars AKシリーズ（MODE_CUBEMARS）

`cubemars_ak_driver` プロファイルは、CubeMars AKシリーズ（AK40-10等）の
アクチュエータ最大4台を、CANホストのノード分配を介さず直接制御するための
プロファイルです。ファームウェア側は `MODE_CUBEMARS` でビルドされた独立デバイスで、
各モータのCAN IDは `config.hpp` の `CUBEMARS_MOTOR_ID_n` でコンパイル時に固定します
（実機のR-Link設定と必ず一致させてください）。ノード分配を行わないため、スロット
0番起点でモータ1〜4の指令・帰還がそのまま並びます（`firmware/xiao-esp32-s3_can2io/src/cubemars.cpp`
のスロット割当と一致させること）。

アクチュエータ自身が速度・位置のクローズドループを内蔵しているため、
ロボマス(GM6020)と異なりホスト側でのPID制御は行いません。

![CubeMars Controlタブ: M1をMITモードに設定した状態](images/10_cubemars_control.png)

### 8.1 指令チャンネル（Control タブ）

モータごとに以下の6チャンネルがあります（`M{n}` の `n` は1〜4）。

| チャンネル | 種別 | 説明 |
|:---|:---|:---|
| `M{n} target` | 数値 | `control_mode` が速度ループのとき 10ERPM/LSB（電気角速度）、位置ループ/MITのとき 0.1deg/LSB。GUI上はスケール無しの生値として編集する |
| `M{n} control_mode` | 選択式 | `0`=速度ループ / `1`=位置ループ / `2`=MIT(Force Control)。全ゼロ（未接続・E-STOP時の既定）で速度ループ・target=0となり、位置ジャンプなく安全にその場停止する |
| `M{n} mit_velocity` | 数値 | MITモード用の目標速度（`control_mode=2`のときのみ参照）。0.01rad/s/LSB、レンジ±45.5rad/s |
| `M{n} mit_kp` | 数値 | MITモード用のKp（`control_mode=2`のときのみ参照）。0.1/LSB、レンジ0〜500 |
| `M{n} mit_kd` | 数値 | MITモード用のKd（`control_mode=2`のときのみ参照）。0.01/LSB、レンジ0〜5 |
| `M{n} mit_torque_ff` | 数値 | MITモード用のトルクフィードフォワード（`control_mode=2`のときのみ参照）。0.01N・m/LSB、レンジ±5.0N・m |

**MITモード**は、位置・速度・Kp・Kd・トルクFFを同時に指令し、モータ側で

```
torque = Kp * (pos_des - pos) + Kd * (vel_des - vel) + torque_ff
```

を計算するインピーダンス制御です（AK Series Module Product Manual V3.2.0
4.2節）。`mit_velocity`/`mit_kp`/`mit_kd`/`mit_torque_ff` の各スロットは
`control_mode=2` のときのみ参照され、他のモードでは無視されます。

> スライダーの範囲欄（クランプ範囲）は、ファームウェア側 `config.hpp` の
> `CUBEMARS_MIT_*` および実機のR-Link設定と必ず一致させてください。
> ファーム側を書き換えた場合は、GUI側の「プロファイル編集」でも
> レンジを合わせて調整する必要があります。

### 8.2 帰還チャンネル（Monitor タブ）

![CubeMars Monitorタブ: 各モータの位置・速度・電流・温度・故障コード](images/11_cubemars_monitor.png)

| チャンネル | 種別 | 説明 |
|:---|:---|:---|
| `M{n} position` | 数値表示 | 出力軸角度。0.1deg/LSB。「原点セット」対象（zeroable） |
| `M{n} speed` | 数値表示 | 電気角速度。10ERPM/LSB。出力軸rpmへの換算（極対数・減速比）はモータ機種依存のためGUI側では未実施 |
| `M{n} current` | 数値表示 | 相電流。0.01A/LSB |
| `M{n} temperature` | 数値表示 | モータ温度 [degC] |
| `M{n} error` | 選択式表示 | 故障コード（下表）。デバッグデバイスではGUIから手動設定可能 |

故障コード一覧:

| コード | 意味 |
|:---:|:---|
| 0 | no fault |
| 1 | motor over-temp |
| 2 | over-current |
| 3 | over-voltage |
| 4 | under-voltage |
| 5 | encoder fault |
| 6 | MOSFET over-temp |
| 7 | motor stall |

### 8.3 運用上の注意

- ノード分配側（`MODE_CAN` 等）とCAN速度を1Mbpsに統一済みのため、同一物理バスへの
  混在は可能です（CAN IDが重ならないよう設計されていれば問題ありません）。
- バス帯域を確保するため、ホストからの指令送信は200Hz固定です
  （ファームウェア側 `cubemars.cpp`）。アクチュエータ自身の帰還フレームは
  指令頻度に関係なく自律送信されるため、台数を増やす場合はバス帯域の余裕を
  `firmware/xiao-esp32-s3_can2io/README.md` で確認してください。
- 1バスに接続できるのは最大4台（`config.hpp` の `CUBEMARS_MOTOR_COUNT`）です。

## 9. DJIロボマス（MODE_ROBOMAS）

`robomas_driver` プロファイルは、DJIロボマス（M3508 / M2006 / GM6020 のいずれか）
最大4台を、CANホストのノード分配を介さず直接制御するためのプロファイルです。
モータ機種はファームウェア側 `config.hpp` の `ROBOMAS_MOTOR_TYPE` でコンパイル時に
固定し、1バスには単一機種のみ最大4台接続できます
（スロット割当は `firmware/xiao-esp32-s3_can2io/src/robomas.cpp` を参照）。

![ロボマス Controlタブ: M1をMIT(位置PD)モードに設定した状態](images/12_robomas_control.png)

### 9.1 指令チャンネル（Control タブ）

| チャンネル | 種別 | 説明 |
|:---|:---|:---|
| `M{n} target` | 数値 | `control_mode` が速度ループのとき出力軸rpm（ギア比込み、生値スケール無し）。MITのとき目標位置、1deg/LSB、出力軸角度（レンジ±32767=約±91回転） |
| `M{n} control_mode` | 選択式 | `0`=速度ループ（既定） / `1`=MIT(位置PD)。全ゼロ（未接続・E-STOP時の既定）で速度ループ・target=0となり、安全にその場停止する |
| `M{n} mit_velocity_ff` | 数値 | MITモード用の目標速度フィードフォワード（`control_mode=1`のときのみ参照）。1rpm/LSB |
| `M{n} mit_kp` | 数値 | MITモード用の比例ゲイン（`control_mode=1`のときのみ参照）。0.001(A/deg)/LSB |
| `M{n} mit_kd` | 数値 | MITモード用の微分ゲイン（`control_mode=1`のときのみ参照）。0.0001(A/rpm)/LSB |
| `M{n} mit_current_ff` | 数値 | MITモード用の電流フィードフォワード（`control_mode=1`のときのみ参照）。0.001A/LSB |

速度ループの速度PIDゲインはCAN経由では変更できず、ファームウェア側
`config.hpp`（`ROBOMAS_KP_VEL` 等）のコンパイル時定数で固定されています。
一方、MITモードの `mit_kp`/`mit_kd`/`mit_current_ff` はCAN経由で毎周期
変更可能な可変値です。

CubeMarsのMITと対称的な機能ですが、実装の考え方は異なります。ロボマス側の
ESC/GM6020はCANで生の電流指令しか受け付けないため、位置PD制御ループ自体を
マイコン側（`robomas.cpp`）で計算します。位置フィードバックにはロボマス
内蔵のロータエンコーダを使用します（外付けエンコーダのバックラッシュ補正等は
行いません）。

> スケール値はファームウェア側 `config.hpp` の `ROBOMAS_MIT_*` と必ず
> 一致させてください。

### 9.2 帰還チャンネル（Monitor タブ）

![ロボマス Monitorタブ: 各モータの角度・速度・電流](images/13_robomas_monitor.png)

| チャンネル | 種別 | 説明 |
|:---|:---|:---|
| `M{n} angle` | 数値表示 | 出力軸角度。0.1deg/LSB。「原点セット」対象（zeroable） |
| `M{n} velocity` | 数値表示 | 出力軸rpm |
| `M{n} current` | 数値表示 | 相電流。0.001A/LSB |

### 9.3 運用上の注意

- ノード分配側（`MODE_CAN` 等）もCAN速度を1Mbpsに統一済みのため、同一物理バスへの
  混在は可能です（CAN IDが重ならない設計であること）。
- バス帯域を確保するため、指令送信は200Hz固定です（ファームウェア側
  `robomas.cpp`）。台数を増やす場合はバス帯域の余裕を
  `firmware/xiao-esp32-s3_can2io/README.md` で確認してください。

## 10. ファームウェア生成・書き込み

ツールバーの「ファームウェア生成・書き込み…」から、`xiao-esp32-s3_can2io` を
テンプレートに、実機ごとに異なる設定だけを反映したプロジェクト一式を
`generated_firmware/<名前>/` へ生成し、そのまま同じ画面から実機へ書き込め
（`pio run -t upload`）ます。テンプレート自体は書き換えません。

![ファームウェア生成・書き込みダイアログ](images/14_firmware_dialog.png)

### 10.1 生成できる設定項目

| 項目 | 内容 |
|:---|:---|
| 生成先の名前 | `generated_firmware/<名前>/` としてプロジェクト一式をコピーする際のフォルダ名 |
| DEVICE_ID | serial_bridge互換フレームの `serial_tx_[ID]`/`serial_rx_[ID]` に対応するID |
| CAN_ID（下2桁=ノード番号） | ノードのCAN ID。101〜104のように下2桁がノード番号になるよう設定 |
| MODE | `CAN`(通常のCANノード) / `CAN_HOST`(CANホスト) / `IO`(CAN無し) / `DEBUG` / `CAN_MONITOR` / `ROBOMAS`(DJI RoboMaster) / `CUBEMARS`(CubeMars AK) |
| BOARD_VARIANT | 書き込み先の物理基板。`BOARD_SOKI`(既存のピン配置) / `BOARD_MES`(ENC1/ENC2/MD1/MD2+SW1) / `BOARD_SS`(SERVO1-5+TR1-4) |
| MULTI1〜3 | ピン共有の入出力切替（0=スイッチ入力 / 1=サーボ出力）。BOARD_SOKI系のみ意味を持つ |
| ENC1_MD / ENC2_MD | ピン共有の入出力切替（0=エンコーダ / 1=MD(PWM+DIR)出力） |
| ENC2_SW | BOARD_MES専用。ENC2とSW2/SW3はピン共有のため、どちらで使うかを選択 |
| サーボ設定（SERVO1〜5） | `MIN_US`/`MAX_US`(パルス幅)、`MIN_DEG`/`MAX_DEG`/`INIT_DEG`(角度)をサーボごとに設定。BOARD_SOKIはMULTIn=1(サーボ出力)のチャンネルのみSERVO1-3が有効、BOARD_SSはSERVO1-5が常時有効（SERVO4/5はBOARD_SOKI/BOARD_MESでは未配線） |
| 高度な設定（既定で折りたたみ） | `SERVO_PWM_FREQ`/`SERVO_PWM_RESOLUTION`、`MD_PWM_FREQ`/`MD_PWM_RESOLUTION`、`ENABLE_LED`、`CAN_HOST_DIAG_ENABLE` |

`CAN_NODE_COUNT`/`CAN_SLOTS_PER_NODE` はBOARD_VARIANTごとに `config.hpp` 内で
コンパイル時分岐しているため、このGUIからは編集できません（BOARD_SOKI/BOARD_MES:
4ノード x 5スロット、BOARD_SS: 2ノード x 9スロット）。実際の接続台数に合わせて
変更したい場合は、生成後のプロジェクトの `src/config.hpp` を直接編集してください。
同様に、ROBOMASの速度PIDゲイン等、実測でチューニングされた値もこのGUIの対象外です
（テンプレートの値がそのまま引き継がれます）。

### 10.2 生成の流れ

1. 「テンプレート:」欄に `firmware/xiao-esp32-s3_can2io` が自動検出されます
   （見つからない場合は「変更…」で手動選択）。
2. 各設定項目を入力し、「生成先の名前」を決めます。
3. 「生成」を押すと、テンプレートから `config.hpp` へ反映される変更点の一覧が
   確認ダイアログに表示されます。内容を確認して「はい」を押すと、
   `generated_firmware/<名前>/` へプロジェクト一式がコピーされ、その
   コピー内の `config.hpp` だけが書き換えられます。
4. `generated_firmware/` はGit管理下に置かれ、生成物として履歴に残ります。
   不要になった生成物は、下の「書き込み」欄でプロジェクトを選択して
   「削除」から取り除けます。

### 10.3 書き込みの流れと安全性

1. 下の「書き込み」欄で、生成済みのプロジェクトと書き込み先のポートを選択します。
2. 「書き込み」を押すと確認ダイアログが出ます。承諾すると
   `pio run -t upload` がサブプロセスとして実行され、ログがそのまま
   画面下部に流れます。
3. `ros2can` 自身がシリアルポートを排他専有している（`serial_link.py` の
   `TIOCEXCL`）ため、書き込み開始時に対象ポートの接続を一旦閉じ、書き込み中は
   バックグラウンドのポートスキャナがそのポートに触れないよう一時的に除外します。
   書き込みが完了/中断されると、除外は自動的に解除されます。
4. 書き込みは数十秒かかりうるサブプロセス実行のため、GUIスレッドをブロックしない
   よう別スレッド（QThread）で行われます。「中断」でいつでも打ち切れます。
5. 書き込み中は対象ポートの他の通信（実機からのセンサ受信等）が一時的に
   停止する点に注意してください。

PlatformIO CLI（`pio` コマンド）が必要です。見つからない場合は
`pip install -U platformio` 等でインストールしてください。

## 11. 設定

ツールバーの「設定…」から、ハードウェア直結スキャンの挙動（除外ポート、
タイムアウト、スキャン間隔、device_id ↔ プロファイル対応など）を編集できます。

![設定ダイアログ](images/07_settings_dialog.png)

保存すると実行中のスキャナにもその場で反映されます（再起動不要）。設定の優先順位は

1. `config/ros2can.yaml`（このリポジトリのGit管理下、共通の既定値）
2. `~/.config/ros2can/settings.yaml`（GUIの「設定」で保存されるユーザーローカルの上書き。リポジトリの外にあるためGitの追跡対象にはなりません）
3. `--ros-args -p` / launch の `parameters=[...]` で明示的に渡した値

の順です。

## 12. エンコーダ初期化

デバイスごとの Monitor タブにもチャンネル単位の「原点セット」ボタンがありますが、
起動時に複数デバイスをまとめて初期化したい場合は、ツールバーの「エンコーダ初期化…」
から、検出済み全デバイスの zeroable なチャンネル（角度/位置/カウンタ系）を1画面に
まとめて表示し、行単位・デバイス単位・全デバイス一括で原点セットできます。

![エンコーダ初期化ページ](images/08_encoder_init.png)

ここでの原点セットはGUI側のソフトウェアオフセットで、マイコン側の値は変更しません
（`serial_rx_[ID]_zeroed` として相対値が配信されます）。実機のエンコーダ自体に
原点を書き込みたい場合は、上部の「外部ノードのTriggerサービス呼び出し」から
対象ノードが提供する `std_srvs/Trigger` サービスを呼び出してください。

## 13. 安全機能について

- 起動直後は全デバイスの「ダイレクト送信」が OFF になっており、実際の指令は
  送信されません。意図した値を設定してから ON にしてください。
- 「トピック通過」は既定 ON で、外部ROSノードからの指令がパネルに反映される
  状態になっています（ダイレクト送信がOFFなら実機へは送られません）。
- ダイレクト送信中は 20Hz で現在の指令値を周期送信し続けます（ウィンドウを
  閉じる、または「全ゼロ送信」/E-STOP を押すと即座にゼロ指令が送信されます）。
- CubeMars/ロボマスの `control_mode` は全ゼロ（未接続・E-STOP時の既定）で
  速度ループ・target=0となるよう設計されているため、位置ジャンプなく安全に
  その場停止します。
- ツールバーの「全デバイス E-STOP」は、接続中の全デバイスへゼロ指令を送信し
  ダイレクト送信を無効化します。緊急時はこれを押してください。

## 14. About

ツールバー右端の「Info…」から、バージョン情報とGitHubリンクを確認できます。

![Aboutダイアログ](images/09_about_dialog.png)

---

<img src="https://www.rrst.jp/img/logo.png" alt="Logo" height="50"><br>
立命館大学 ロボット技術研究会, RRST, NHKプロジェクト（2024–2026）
