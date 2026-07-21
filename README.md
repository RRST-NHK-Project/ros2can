# ros2can

`xiao_esp32_s3_smd_serial_bridge` (MODE_CAN_HOST) 専用の**スタンドアローン**GUI
です。`serial_bridge` (ROS 2 / C++) の後継として、シリアルポートのスキャン・
専有・serial_bridge 互換フレームの送受信を自前で行います。外部の
`serial_bridge` ノードは不要です。

この基板は USB シリアルでつながる「CANホスト」で、自身の配下に CAN バス経由で
最大4台の子マイコン(ノード)をぶら下げます。ros2can はホストのシリアルポートを
直接掴み、バス上の各ノードへアクチュエータ指令を直接送信したり、センサ値を
リアルタイムに表示します。

## 前提

- `xiao_esp32_s3_smd_serial_bridge` (MODE_CAN_HOST) を書き込んだ基板が USB
  接続されていること (serial_bridge ノードの起動は不要)。
- `python3-pyqt5` / `python3-serial` がインストールされていること
  (`sudo apt install python3-pyqt5 python3-serial`)。
- ユーザーが `dialout` グループに属していること (`/dev/ttyUSB*`/`ttyACM*` アクセス用)。

## ビルド

```bash
cd ~/ros2_ws
colcon build --packages-select ros2can
source install/setup.bash
```

## 起動

```bash
ros2 run ros2can ros2can
```

または (config/ros2can.yaml のパラメータを読み込む場合)

```bash
ros2 launch ros2can ros2can.launch.py
```

起動すると ros2can 自身がバックグラウンドスレッドで `/dev/ttyUSB*` /
`/dev/ttyACM*` を定期的にスキャンし、CANホストを検出すると自動的に
シリアルポートを専有してデバイス一覧に表示します。

### serial_bridge との併用について

同一マシンで旧来の `serial_bridge` ノードを併用することも可能です
(移行期間中、他のマイコンは serial_bridge、CANホストは ros2can、といった構成)。
ros2can はポートを開いた直後に `ioctl(fd, TIOCEXCL)` を発行してポートを
排他専有するため、**ros2can が先にポートを掴んでいれば** serial_bridge 側の
`open()` が失敗して静かにリトライされるだけで済み、フレームの競合は起きません。
ただし逆方向 (serial_bridge が先にポートを掴んだ場合) は serial_bridge 側にも
同様の排他制御が無いと完全には防げません。既知の対策:

- `config/ros2can.yaml` の `excluded_ports` に serial_bridge 管理下のポートを
  列挙し、ros2can 側のスキャン対象から外す。
- 同様に `serial_bridge.yaml` の `excluded_ports` に ros2can 管理下のポートを
  列挙する。
- (将来的な改善案) serial_bridge 側の `port_scanner.cpp` / `bridge_node.cpp`
  の `open()` 直後にも `TIOCEXCL` を追加すれば双方向で安全になる。

まだ接続されていない DEVICE_ID を、既存のトピックへ相乗りする形で先に登録
しておきたい場合は、ツールバーの「デバイスを手動追加」から追加できます
(この場合はハードウェアを専有せず、`serial_tx_[ID]` を Publish /
`serial_rx_[ID]` を Subscribe するだけのクライアントとして動作します)。

## デバッグモード (実機不要でのUI確認)

マイコン実機が手元に無くても、UIのレイアウト調整やウィジェットの動作確認が
できるよう、ツールバーの「デバッグデバイスを追加(実機不要)…」から**仮想デバイス**
を追加できます。

- DEVICE_ID を入力すると、その場で仮想デバイスが追加されます(実機のスキャン
  や接続は一切不要)。
- Control タブでスライダー等を動かすと、書き込んだ値がそのまま(多少の揺らぎを
  付けて)RX側にループバックされ、Monitor / Raw / Info タブにリアルタイムに
  反映されます。トピック通過/ダイレクト送信のON/OFFに関わらずRXは更新され
  続けるので、いつでもMonitor側の見た目を確認できます。
- 実機接続時と同じく `serial_rx_[ID]` を Publish / `serial_tx_[ID]` を
  Subscribe するため、rqt や他のROSノードからのテストにもそのまま使えます。
- デバイス一覧でデバイスを右クリックすると「このデバイスを削除」で取り除けます
  (手動追加・デバッグデバイス共通)。

## 画面構成

- **デバイス一覧 (左)**: 検出済みの CAN ホスト (DEVICE_ID ごと)。
  接続中/未接続、ダイレクト送信中(TX ON)状態、トピック通過OFF時の警告
  (PASS OFF)、モード(`HW:/dev/ttyUSBx` = ros2can が直接専有 / `topic` =
  既存トピックへの相乗り / `🧪DEBUG(仮想)` = 実機不要のデバッグデバイス)
  を表示します。
- **Control タブ**: ノード1〜4のサブタブで「どのマイコン(CANノード)を
  操作するか」を選び、各ノードの `SERVO1-3`(サーボ角度、実機はDCモータ非搭載)
  を直接送信できます。実際に送信するには右上の「ダイレクト送信」チェックを
  ONにする必要があります (誤操作防止のため既定はOFF)。もう一つの「トピック通過」
  チェック(既定ON)は、外部ROSノードから `serial_tx_[ID]` トピック経由で
  届く指令をこのパネルに反映するかどうかを別途制御します。OFFにすると
  外部ノードからの指令は無視され、このパネル(GUI操作またはRawタブ編集)から
  の値だけが有効になります。`SERVOn` は `SWn` と
  ピン共有のため、ファームウェア側で `MULTIn=1`(サーボ)に設定したポートのみ有効です。
- **Monitor タブ**: 同じくノードごとのサブタブで `SW1-3`(スイッチ入力、`MULTIn=0`
  のポートのみ有効) と `ENC1-2`(エンコーダカウンタ) をリアルタイム表示します。
- **Raw タブ**: CAN分配を意識せず、ホストが送受信する生の24 x int16スロットを
  そのまま編集/確認できます。未対応スロットのデバッグや、プロファイルの
  想定と実機がズレている場合の確認に使用します。
- **Info タブ**: 接続状態、RX周波数、送受信フレーム数、現在の生データ配列を
  表示します。不具合報告時にそのままコピーできます。

## プロファイル

既定では下記2種類のプロファイルを切り替えられます。

| プロファイル | 内容 |
|---|---|
| XIAO ESP32S3 SMD (CAN Host) | 24スロットを4ノード x 5スロットに分配 (既定の `CAN_NODE_COUNT=4`, `CAN_SLOTS_PER_NODE=5` に対応) |
| xiao-esp32-s3_can2io + b-g431-esc1_can2io (FOCモータ, robomas互換) | 上記のノード1台をFOCモータ(SimpleFOC、速度制御のみ)用チャンネルに置き換え |
| xiao-esp32-s3_can2io (MODE_ROBOMAS, DJIロボマス x4) | `MODE_ROBOMAS`(独立デバイス、CAN 1Mbps固定)用。ノード/スロット分配は行わず、ロボマス最大4台の速度指令/帰還を24スロットに直接割り当てる |
| 汎用 Raw | CAN分配を意識しない生の24スロット |

ファームウェア側の `config.hpp` で `CAN_NODE_COUNT` / `CAN_SLOTS_PER_NODE` を
変更した場合は、デバイスパネル右上の「プロファイル編集」からノード数・
スロット数を指定して「自動生成」し、必要に応じてラベルやレンジを調整して
保存してください。カスタムプロファイルは `~/.config/ros2can/profiles/` に
JSON として保存され、次回起動時にも読み込まれます。

## 安全に関する注意

- 起動直後は全デバイスの「ダイレクト送信」が OFF になっており、実際の指令は
  送信されません。意図した値を設定してから ON にしてください。
  「トピック通過」は既定 ON で、外部ROSノードからの指令がパネルに反映される
  状態になっています(ダイレクト送信がOFFなら実機へは送られません)。
- ダイレクト送信中は 20Hz で現在の指令値を周期送信し続けます (ウィンドウを
  閉じる、または「全ゼロ送信」/E-STOP を押すと即座にゼロ指令が送信されます)。
- ツールバーの「全デバイス E-STOP」は、接続中の全デバイスへゼロ指令を送信し
  ダイレクト送信を無効化します(トピック通過の設定はそのまま)。緊急時は
  これを押してください。

## 対応スロットマッピング (既定プロファイル)

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

## 設定パラメータ (`config/ros2can.yaml`, ノード名 `ros2can_gui`)

| パラメータ | 既定値 | 説明 |
|:---|:---|:---|
| `excluded_ports` | `[]` | スキャン対象から除外するポート (serial_bridge 管理下のポート等) |
| `rx_timeout_sec` | `2.0` | この秒数 RX が無ければポートを閉じる |
| `reconnect_interval_sec` | `3.0` | 切断後、同じポートへ再接続を試みるまでの最小待機時間 |
| `scan_interval_ms` | `5000` | 未専有ポートを再スキャンする間隔 |
| `probe_timeout_sec` | `2.0` | ポートプローブ時、有効なフレームを待つ最大時間 |
| `probe_settle_sec` | `0.5` | ポートを開いた直後の USB CDC 安定待ち時間 |

`serial_bridge.yaml` と同じパラメータ名にしてあるので、移行時の設定の使い回しが
容易です。
