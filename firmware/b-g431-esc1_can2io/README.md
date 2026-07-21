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
- 外付けCANトランシーバの配線と、バス終端抵抗(120Ω)の設定を確認してから接続すること。
- ビットレートは 500kbit/s (ホスト `xiao-esp32-s3_can2io` の `TWAI_TIMING_CONFIG_500KBITS` と一致させる)。

## CANプロトコル (xiao-esp32-s3_can2io 互換)

CANフレームID = `0x100 + CAN_NODE_INDEX*16 + chunk`。1フレーム8バイト、int16 x 4個 (ビッグエンディアン)。
`CAN_SLOTS_PER_NODE=5` のため2チャンク(chunk0: 4値, chunk1: 1値)で1ノード分を送受信する。

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

## 動作・安全機構

制御は速度モードのみで、`xiao-esp32-s3_can2io` の `MODE_ROBOMAS` と同様に **CAN途絶時の
フェイルセーフ・enable指令・オーバースピードガードは持ちません**。最後に受信した
`target_velocity` を保持し続けるため、ホスト側(ros2can)のTX有効化/E-STOPで確実に
ゼロ指令を送ること。

## 補足

- 実機はTIM4のクアドラチャエンコーダ入力(ハードウェアカウンタ)を使用。`ENCODER_CPR` を実配線に合わせて変更すること。
- 角度制御・トルク制御には対応していません(速度制御のみ)。
