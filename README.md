# **ros2can**

> 📖 マニュアル(GitHub Pages): https://rrst-nhk-project.github.io/ros2can/

> 本パッケージは [serial_bridge](https://github.com/RRST-NHK-Project/serial_bridge) の後継です。
> `xiao_esp32_s3_smd_serial_bridge`（MODE_CAN_HOST）専用の**スタンドアローン**GUIとして、
> シリアルポートのスキャン・専有・serial_bridge 互換フレームの送受信を自前で行います。
> 外部の `serial_bridge` ノードは不要です。

## 1. 概要

`ros2can` は、CANバス経由で複数マイコンをホストする `xiao_esp32_s3_smd_serial_bridge`
（MODE_CAN_HOST）専用の ROS 2 GUI パッケージです。

この基板は USB シリアルでつながる「CANホスト」で、自身の配下に CAN バス経由で
最大4台の子マイコン（ノード）をデイジーチェーン接続可能です。`ros2can` はホストのシリアルポートを
直接掴み、バス上の各ノードへアクチュエータ指令を直接送信したり、センサ値を
リアルタイムに表示することができます。また、`serial_bridge`と同様にトピックを用いた外部ノードとの接続も可能です。

---

## 2. システム要件

| 項目 | 内容 |
|:---|:---|
| OS | Ubuntu 24.04 LTS |
| ROS | ROS 2 Jazzy |
| GUI | PyQt5 |
| ボーレート | 115200 bps |
| ハードウェア | `xiao_esp32_s3_smd_serial_bridge`（MODE_CAN_HOST）を書き込んだ基板を USB 接続 |
| 追加パッケージ | `python3-pyqt5`, `python3-serial`（`sudo apt install python3-pyqt5 python3-serial`） |

> **注意**:
> `/dev/ttyUSB*` や `/dev/ttyACM*` を使用するには `dialout` グループへの追加が必要です。
> `sudo usermod -aG dialout $USER`（反映には再ログインが必要）

---

## 3. 機能一覧

- シリアルポートの自動スキャン・専有 - `serial_bridge` ノードの起動は不要です。
- CANホスト経由で最大4台の子マイコン（ノード）へアクチュエータ指令を直接送信できます。
- 起動直後は「ダイレクト送信」OFF／「トピック通過」ONで、既存の ROS 2 トピックとの併用も可能です。
- マイコンが接続されていなくてものデバッグ用**仮想デバイス**を追加でき、デバッグが容易です。
- ノード数・スロット数が異なる複数のプロファイルを切り替え可能。カスタムプロファイルも保存できます。
- 「全デバイス E-STOP」による緊急停止機能を搭載しています。
- `serial_bridge` との併用を想定した排他制御・ポート除外設定を備えています。

---

## 4. システム構成

```
  [ros2can (GUI, スタンドアローン)]
       │
       ├─ port_scanner ─── /dev/ttyUSB*, /dev/ttyACM* を探索
       │                    CANホストを検出したポートを TIOCEXCL で排他専有
       │
       └─ USBシリアル ──► CANホスト (xiao_esp32_s3_smd_serial_bridge, MODE_CAN_HOST)
                             │
                             └─ CANバス
                                  ├─ ノード1 (CAN_ID=101)  SERVO/SW/ENC 等
                                  ├─ ノード2 (CAN_ID=102)
                                  ├─ ノード3 (CAN_ID=103)
                                  └─ ノード4 (CAN_ID=104)
```

CANホストの `DEVICE_ID` は他の ROS 2 ノードから見た `serial_tx_[ID]` /
`serial_rx_[ID]` トピックに対応しており、既存のトピックへ相乗りする「トピック
クライアント」としても動作します（[8. デバッグモード](#8-デバッグモード実機不要でのui確認)参照）。

### serial_bridge との併用について

同一マシンで `serial_bridge` を併用することも可能です
（移行期間中、他のマイコンは serial_bridge、CANホストは ros2can、といった構成）。
`ros2can` はポートを開いた直後に `ioctl(fd, TIOCEXCL)` を発行してポートを
排他専有するため、**ros2can が先にポートを掴んでいれば** serial_bridge 側の
`open()` が失敗して静かにリトライされるだけで済み、フレームの競合は起きません。
ただし逆方向（serial_bridge が先にポートを掴んだ場合）は serial_bridge 側にも
同様の排他制御が無いと完全には防げません。既知の対策:

- `config/ros2can.yaml` の `excluded_ports` に serial_bridge 管理下のポートを
  列挙し、ros2can 側のスキャン対象から外す。
- 同様に `serial_bridge.yaml` の `excluded_ports` に ros2can 管理下のポートを
  列挙する。

---

## 5. ROS 2 インターフェース

### Subscribe トピック（ROS → CANホスト）

| トピック | 型 | 説明 |
|:---|:---|:---|
| `serial_tx_[DEVICE_ID]` | `std_msgs/msg/Int16MultiArray` | CANホストへの制御指令（生の24スロット） |

### Publish トピック（CANホスト → ROS）

| トピック | 型 | 説明 |
|:---|:---|:---|
| `serial_rx_[DEVICE_ID]` | `std_msgs/msg/Int16MultiArray` | センサ値（生の24スロット、`serial_bridge` 互換） |
| `serial_rx_[DEVICE_ID]_unwrapped` | `std_msgs/msg/Int32MultiArray` | エンコーダのオーバーフローを展開した積算値（MODE_HARDWARE/MODE_SIMULATOR 時のみ） |
| `serial_rx_[DEVICE_ID]_zeroed` | `std_msgs/msg/Int32MultiArray` | 上記から「原点セット」時点のオフセットを差し引いた相対値（MODE_HARDWARE/MODE_SIMULATOR 時のみ） |

### サービス

| サービス | 型 | 説明 |
|:---|:---|:---|
| `zero_channel` | `ros2can_interfaces/srv/ZeroChannel` | 指定デバイス・チャンネルの現在値を原点（0）としてソフトウェアオフセットを設定する。`channel_index=-1` で全24チャンネル一括原点セット |

`serial_tx_[ID]` / `serial_rx_[ID]` は `serial_bridge` と同一の型・命名規則
のため、他ノードからの Publish/Subscribe による相乗りや、`serial_bridge` から
の移行がそのまま行えます。

---

## 6. パラメータ（`config/ros2can.yaml`, ノード名 `ros2can_gui`）

| パラメータ | デフォルト | 説明 |
|:---|:---|:---|
| `excluded_ports` | `[]` | スキャン対象から除外するポート（serial_bridge 管理下のポート等） |
| `rx_timeout_sec` | `2.0` | この秒数 RX が無ければポートを閉じる |
| `reconnect_interval_sec` | `3.0` | 切断後、同じポートへ再接続を試みるまでの最小待機時間 |
| `scan_interval_ms` | `5000` | 未専有ポートを再スキャンする間隔 |
| `probe_timeout_sec` | `2.0` | ポートプローブ時、有効なフレームを待つ最大時間 |
| `probe_settle_sec` | `0.5` | ポートを開いた直後の USB CDC 安定待ち時間 |
| `device_profile_map` | `[]` | `"device_id:profile_key"` 形式で、検出時に初期選択するプロファイルを指定 |


上記パラメータはツールバーの「設定…」からもGUI上で編集できます。優先順位は
`config/ros2can.yaml`（このリポジトリのGit管理下、共通の既定値）→
`~/.config/ros2can/settings.yaml`（GUIの「設定」で保存されるユーザーローカルの
上書き。**リポジトリの外にあるためGitの追跡対象にはなりません**）→
`--ros-args -p` / launch の `parameters=[...]` で明示的に渡した値、の順です。
GUIから保存すると実行中のスキャナにもその場で反映されます（再起動不要）。

---

## 7. 使い方

### 7.1 ビルド

```bash
cd ~/ros2_ws
colcon build --packages-select ros2can
source install/setup.bash
```

### 7.2 起動

```bash
ros2 run ros2can ros2can
```

または（`config/ros2can.yaml` のパラメータを読み込む場合）

```bash
ros2 launch ros2can ros2can.launch.py
```

起動すると `ros2can` 自身がバックグラウンドスレッドで `/dev/ttyUSB*` /
`/dev/ttyACM*` を定期的にスキャンし、CANホストを検出すると自動的に
シリアルポートを専有してデバイス一覧に表示します。

まだ接続されていない `DEVICE_ID` を、既存のトピックへ相乗りする形で先に登録
しておきたい場合は、ツールバーの「デバイスを手動追加」から追加できます
（この場合はハードウェアを専有せず、`serial_tx_[ID]` を Publish /
`serial_rx_[ID]` を Subscribe するだけのクライアントとして動作します）。

---

## 8. デバッグモード（実機不要でのUI確認）

マイコン実機が手元に無くても、UIのレイアウト調整やウィジェットの動作確認が
できるよう、ツールバーの「デバッグデバイスを追加（実機不要）…」から**仮想デバイス**
を追加できます。

- DEVICE_ID を入力すると、その場で仮想デバイスが追加されます（実機のスキャン
  や接続は一切不要）。
- Control タブでスライダー等を動かすと、書き込んだ値がそのまま（多少の揺らぎを
  付けて）RX側にループバックされ、Monitor / Raw / Info タブにリアルタイムに
  反映されます。トピック通過/ダイレクト送信のON/OFFに関わらずRXは更新され
  続けるので、いつでもMonitor側の見た目を確認できます。
- 実機接続時と同じく `serial_rx_[ID]` を Publish / `serial_tx_[ID]` を
  Subscribe するため、rqt や他のROSノードからのテストにもそのまま使えます。
- デバイス一覧でデバイスを右クリックすると「このデバイスを削除」で取り除けます
  （手動追加・デバッグデバイス共通）。

---

## 9. 画面構成

- **デバイス一覧（左）**: 検出済みの CAN ホスト（DEVICE_ID ごと）。接続状態・
  ダイレクト送信中（TX ON）・トピック通過OFF（PASS OFF）・モード
  （`HW` = 直接専有 / `topic` = 相乗り / `🧪DEBUG` = 仮想デバイス）を表示します。
- **Control タブ**: ノード1〜4のサブタブでアクチュエータ（サーボ等）へ直接送信します。
  送信には「ダイレクト送信」チェック（既定OFF、誤操作防止）が必要です。
  「トピック通過」（既定ON）をOFFにすると外部ノードからの `serial_tx_[ID]` を
  無視し、このパネルの値のみが有効になります。
- **Monitor タブ**: ノードごとにセンサ（スイッチ・エンコーダ等）の値をリアルタイム表示します。
- **Raw タブ**: CAN分配を介さず、生の24 x int16スロットを直接編集/確認できます。
- **Info タブ**: 接続状態・RX周波数・送受信フレーム数・生データ配列を表示します。

---

## 10. プロファイル

既定では下記のプロファイルを切り替えられます。

| プロファイル | 内容 |
|---|---|
| XIAO ESP32S3 SMD (CAN Host) | 24スロットを4ノード x 5スロットに分配（既定の `CAN_NODE_COUNT=4`, `CAN_SLOTS_PER_NODE=5` に対応） |
| xiao-esp32-s3_can2io + b-g431-esc1_can2io (FOCモータ, robomas互換) | 上記のノード1台をFOCモータ（SimpleFOC、速度制御のみ）用チャンネルに置き換え |
| xiao-esp32-s3_can2io (MODE_ROBOMAS, DJIロボマス x4) | `MODE_ROBOMAS`（独立デバイス、CAN 1Mbps固定）用。ノード/スロット分配は行わず、ロボマス最大4台の速度指令/帰還を24スロットに直接割り当てる |
| xiao-esp32-s3_can2io (MODE_CUBEMARS, CubeMars AKシリーズ x4) | `MODE_CUBEMARS`（独立デバイス、CAN 1Mbps固定）用。ノード/スロット分配は行わず、CubeMars AKシリーズ（AK40-10等、Servo(CAN)モードの速度/位置制御、およびMIT(Force Control)モード）最大4台の指令・帰還を24スロットに直接割り当てる |
| 汎用 Raw | CAN分配を意識しない生の24スロット |

ファームウェア側の `config.hpp` で `CAN_NODE_COUNT` / `CAN_SLOTS_PER_NODE` を
変更した場合は、デバイスパネル右上の「プロファイル編集」からノード数・
スロット数を指定して「自動生成」し、必要に応じてラベルやレンジを調整して
保存してください。カスタムプロファイルは `~/.config/ros2can/profiles/` に
JSON として保存され、次回起動時にも読み込まれます。

### 対応スロットマッピング（既定プロファイル）

```
実機はDCモータ非搭載 (ENCx2, SWx3, SERVOx3のみ)。
1ノードあたり5スロット (CAN_SLOTS_PER_NODE=5):
  指令 (ROS -> ホスト -> CAN -> ノード): SERVO1, SERVO2, SERVO3, (予備, 予備)
  帰還 (ノード -> CAN -> ホスト -> ROS): SW1, SW2, SW3, ENC1, ENC2

SERVOn と SWn はピン共有 (ファームウェア config.hpp の MULTIn で切替、
0=スイッチ入力/1=サーボ出力)。

グローバルスロット index = node_index(0-origin) * 5 + local_index
ノードの CAN_ID は 101,102,103,104 (下2桁 = ノード番号)
```

---

## 11. 安全機能について

- 起動直後は全デバイスの「ダイレクト送信」が OFF になっており、実際の指令は
  送信されません。意図した値を設定してから ON にしてください。
  「トピック通過」は既定 ON で、外部ROSノードからの指令がパネルに反映される
  状態になっています（ダイレクト送信がOFFなら実機へは送られません）。
- ダイレクト送信中は 20Hz で現在の指令値を周期送信し続けます（ウィンドウを
  閉じる、または「全ゼロ送信」/E-STOP を押すと即座にゼロ指令が送信されます）。
- ツールバーの「全デバイス E-STOP」は、接続中の全デバイスへゼロ指令を送信し
  ダイレクト送信を無効化します（トピック通過の設定はそのまま）。緊急時は
  これを押してください。

---

## 12. ディレクトリ構成

| パス | 説明 |
|:---|:---|
| `ros2can/` | GUI本体のソース（`main`, `main_window`, `device_panel`, `hardware_manager`, `ros_backend`, `frame_codec` 等） |
| `config/` | ROS 2 パラメータファイル（`ros2can.yaml`） |
| `launch/` | ランチファイル（`ros2can.launch.py`） |
| `resources/` | ロゴ・スタイルシート・バージョン情報（`logo.png`, `soki_logo.png`, `style.qss`, `git_version.txt`） |
| `firmware/xiao-esp32-s3_can2io/` | CANノード側マイコンファームウェア（PlatformIO） |
| `firmware/b-g431-esc1_can2io/` | FOCモータ用CANノードファームウェア（PlatformIO） |
| `test/` | ユニットテスト（`test_counter_unwrapper.py` 等） |

---

## 13. About
<img src="https://www.rrst.jp/img/logo.png" alt="Logo" height="60"><br>
立命館大学 ロボット技術研究会, RRST, NHKプロジェクト（2024–2026）<br><br>

<img src="resources/soki_logo.png" alt="soki Logo" height="60"><br>
キャチロボ2026, 創機立動<br>


