# b-g431-esc1_can2io

B-G431B-ESC1 用の SimpleFOC ベース PIO プロジェクトです。

`xiao-esp32-s3_can2io` (MODE_CAN_HOST) の配下にぶら下がる **CANノード** として動作し、
速度制御のみの簡易サーボを構成します。指令/帰還のデータモデル(項目・スケール)は
`xiao-esp32-s3_can2io` の `MODE_ROBOMAS` (DJI RoboMasterドライバ) と互換です
(target_velocityは生rpm値、angleは0.1deg、currentは0.001A)。
このボード単体はROSと直接通信しません(USBシリアルは使いません)。

## ビルド

```bash
pio run
```

## 書き込み

ボードは `disco_b_g431b_esc1` を使います。必要に応じて PlatformIO の upload 方法を選んでください。

## ハードウェア

- FDCAN1: `A_CAN_RX` = PA11, `A_CAN_TX` = PB9 (基板シルク印字と同じ、ボード変品定義で予約済み)。
- ビットレートは 500kbit/s (ホスト `xiao-esp32-s3_can2io` の `TWAI_TIMING_CONFIG_500KBITS` と一致させる)。
- 終端抵抗(120Ω): 本ボードはCAN_TERM(PC14)のオンボードアナログスイッチでGPIOから
  切替可能(ST UM2516 Table 3参照。デフォルトはソルダーブリッジR26側)。
  `src/config.hpp` の `CAN_TERM_ENABLE` で有効/無効を選択する。バスの終端(末端)に
  位置するノードでのみ1にすること。中間ノードで有効にすると通信不安定の原因になる。

## CANプロトコル (xiao-esp32-s3_can2io 互換)

指令(host->node): `0x100 + CAN_NODE_INDEX*16 + chunk`
帰還(node->host): `0x180 + CAN_NODE_INDEX*16 + chunk`
1フレーム8バイト、int16 x 4個 (ビッグエンディアン)。
`CAN_SLOTS_PER_NODE=5` のため2チャンク(chunk0: 4値, chunk1: 1値)で1ノード分を送受信する。

指令と帰還で別ID帯にしているのは、以前ホストとノードの帰還が同一IDを共有しており、
ホストが送信を始めた途端にバスエラーが急増する不具合があったため。
`CAN_FRAME_ID_CMD_BASE`/`CAN_FRAME_ID_FB_BASE`(`src/can_task.cpp`)は必ず
ホスト側(`xiao-esp32-s3_can2io/src/can_task.cpp`)と同じ値にすること。

`CAN_ID` (`src/config.hpp`) はホスト側 `CAN_NODE_COUNT` の範囲内で他ノードと重複しない値にすること。
`CAN_SLOTS_PER_NODE` はホスト側 (`xiao-esp32-s3_can2io/src/config.hpp`) と必ず一致させること。

`CAN_SLOTS_PER_NODE` はホスト都合で5固定のため、実際に使うのは以下の一部スロットのみで
残りは未使用/予約です。

### host -> node (指令, Rx_16Data) — robomas互換

| index | 内容 | スケール |
|---|---|---|
| 0 (`RX_TARGET_VELOCITY`) | target_velocity | 生rpm値(スケール無し) |
| 1-4 | 未使用(予約) | - |

### node -> host (帰還, Tx_16Data) — robomas互換

| index | 内容 | スケール |
|---|---|---|
| 0 (`TX_ANGLE`) | angle (エンコーダ連続角) | 0.1 deg |
| 1 (`TX_VELOCITY`) | velocity (FOC推定) | 生rpm値(スケール無し) |
| 2 (`TX_CURRENT_Q`) | current_q (実測電流) | 0.001 A |
| 3-4 | 未使用(予約) | - |

ゲイン(`VELOCITY_PID_*` / `CURRENT_PID_*`)や `VOLTAGE_LIMIT` / `VELOCITY_LIMIT` /
`CURRENT_LIMIT` は `src/config.hpp` のコンパイル時定数固定です。CAN経由でのランタイム変更は
サポートしていません(robomasと同じ方針)。

## シリアルモニター (診断用)

このボード単体はROSと通信しませんが、デバッグ用にUSART2(`SERIAL_UART_INSTANCE=2`、
PB3=TX/PB4=RX)へ115200bpsで診断ログを出力します。多くの場合ST-Linkの仮想COMポート
経由でそのまま確認できます(ボードのバリアント定義で「Connected to ST-Link」)。

起動時: `HAL_FDCAN_Init`成功/失敗と、フィルタ範囲(自ノードが受理するCAN ID)を出力。
初期化に失敗した場合は無言で停止せず、1秒おきにエラーを出し続けながら停止します
(モータは安全のため動かしません)。

稼働中: 500msごとに以下を出力します。

- `last_rx`: 最後にホストからのCANフレームを受信してからの経過時間[ms]
- `BusOff` / `ErrPassive` / `Warning`: FDCANプロトコルステータス(バスオフ/エラーパッシブ/警告)
- `LastErrorCode`: 直近のプロトコルエラー種別(`stm32g4xx_hal_fdcan.h`の`FDCAN_protocol_error_code`参照)
- `TxErrCnt` / `RxErrCnt`: 送受信エラーカウンタ
- `target_velocity`: 現在ホストから受信している指令値(生rpm)

通信が来ない場合、まずこのログで「FDCANの初期化自体は成功しているか」「BusOff/エラー
カウンタが上昇していないか(配線・終端抵抗・トランシーバ電源の問題を示唆)」
「target_velocityが更新されているか(ホスト側の問題を示唆)」を切り分けられます。

## 動作・安全機構

制御は速度モードのみで、`xiao-esp32-s3_can2io` の `MODE_ROBOMAS` と同様に **CAN途絶時の
フェイルセーフ・enable指令・オーバースピードガードは持ちません**。最後に受信した
`target_velocity` を保持し続けるため、ホスト側(ros2can)のTX有効化/E-STOPで確実に
ゼロ指令を送ること。

## 補足

- 実機はTIM4のクアドラチャエンコーダ入力(ハードウェアカウンタ)を使用。`ENCODER_CPR` を実配線に合わせて変更すること。
- 角度制御・トルク制御には対応していません(速度制御のみ)。
