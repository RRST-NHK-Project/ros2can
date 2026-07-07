# b-g431-esc1_can2io

B-G431B-ESC1 用の SimpleFOC ベース PIO プロジェクトです。

`xiao-esp32-s3_can2io` (MODE_CAN_HOST) の配下にぶら下がる **CANノード** として動作し、
速度/角度/トルクの3モードを指令で切り替えられる簡易サーボを構成します。
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

### host -> node (指令, Rx_16Data)

| index | 内容 | スケール |
|---|---|---|
| 0 (`RX_ENABLE`) | enable (0=stop, 1=run) | - |
| 1 (`RX_MODE`) | mode (0=velocity, 1=angle, 2=torque) | - |
| 2 (`RX_TARGET_VELOCITY`) | target_velocity | 0.1 rad/s |
| 3 (`RX_TARGET_ANGLE`) | target_angle | 0.1 deg |
| 4 (`RX_TARGET_TORQUE`) | target_torque (電流指令) | 0.001 A |

### node -> host (帰還, Tx_16Data)

| index | 内容 | スケール |
|---|---|---|
| 0 (`TX_ANGLE`) | angle (エンコーダ連続角) | 0.1 deg |
| 1 (`TX_VELOCITY`) | velocity (FOC推定) | 0.1 rad/s |
| 2 (`TX_CURRENT_Q`) | current_q (実測電流) | 0.001 A |
| 3 (`TX_MODE`) | mode echo | - |
| 4 (`TX_STATUS`) | status bits (bit0=CAN受信生存, bit1=overspeed guard作動, bit2=enable状態) | - |

ゲイン(`VELOCITY_PID_*` / `ANGLE_P_GAIN` / `CURRENT_PID_*`)や `VOLTAGE_LIMIT` / `VELOCITY_LIMIT` /
`CURRENT_LIMIT` は `src/config.hpp` のコンパイル時定数固定です。`CAN_SLOTS_PER_NODE=5` の枠には
収まらないため、CAN経由でのランタイム変更はサポートしていません。

## 動作・安全機構

- `enable=0`、またはホストからのCANフレームが `CAN_CMD_TIMEOUT_MS` (既定500ms) 以上途切れた場合、
  トルク/速度指令を強制的に0にします(起動直後、まだ一度も受信していない場合も同様に停止側)。
- トルクモードはSimpleFOC本体の仕様上 `velocity_limit`/`current_limit` による自動クランプが効かないため、
  本ファーム側で電流指令を `CURRENT_LIMIT` にソフトクランプし、`|shaft_velocity| > VELOCITY_LIMIT` を
  検知した場合はトルク指令を0にするオーバースピードガードを実装しています(`TX_STATUS` bit1で通知)。

## 補足

- 実機はTIM4のクアドラチャエンコーダ入力(ハードウェアカウンタ)を使用。`ENCODER_CPR` を実配線に合わせて変更すること。
- 速度/角度/トルクは実行中に切り替え可能です。
