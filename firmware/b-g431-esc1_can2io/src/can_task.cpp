/*====================================================================
<can_task.cpp>
・CANノード通信タスク実装 (STM32G4 FDCAN)

xiao-esp32-s3_can2io (MODE_CAN_HOST) と同じチャンク分割プロトコルで通信する。

  指令(host->node)  ID = CAN_FRAME_ID_CMD_BASE(0x100) + CAN_NODE_INDEX*16 + chunk
  帰還(node->host)  ID = CAN_FRAME_ID_FB_BASE (0x180) + CAN_NODE_INDEX*16 + chunk
  1フレーム8バイト(DLC=8) = int16 x CAN_VALUES_PER_FRAME(4個)、ビッグエンディアン
  CAN_CHUNK_COUNT = ceil(CAN_SLOTS_PER_NODE / CAN_VALUES_PER_FRAME)

  指令と帰還を別ID帯にしているのは、xiao-esp32-s3_can2io側で発生した不具合
  (両方向が同一IDを共有していたため、ホストの指令送信とノードの帰還送信が衝突し、
  ホストが送信を始めた途端にbus_errorが秒間数千件規模で急増していた)を踏まえた修正。
  必ずホスト側(xiao-esp32-s3_can2io/src/can_task.cpp)と同じ値を使うこと。

  [xiao-esp32-s3_can2io/src/can_task.cpp のノード側ロジックをSTM32 FDCAN用に移植]

ビットレート計算 (PCLK1=170MHz, B-G431B-ESC1のSystemClock_Config実測値):
  Prescaler=2, NominalTimeSeg1=148, NominalTimeSeg2=21, Sync=1
  bitrate = 170MHz / (2 * (1+148+21)) = 170MHz / 340 = 500000 bps
  サンプルポイント = (1+148)/170 = 87.6%

Copyright (c) 2026.
====================================================================*/

#include "can_task.hpp"
#include "config.hpp"
#include "frame_data.hpp"

#include <Arduino.h>
#include "stm32g4xx_hal.h"

namespace {

constexpr uint32_t CAN_FRAME_ID_CMD_BASE = 0x100; // host -> node (指令、フィルタ/受信に使用)
constexpr uint32_t CAN_FRAME_ID_FB_BASE = 0x180;  // node -> host (帰還、送信に使用)
constexpr uint8_t CAN_VALUES_PER_FRAME = 4;
constexpr uint8_t CAN_CHUNK_COUNT = (CAN_SLOTS_PER_NODE + CAN_VALUES_PER_FRAME - 1) / CAN_VALUES_PER_FRAME;
constexpr uint32_t CAN_TX_PERIOD_MS = 5;
constexpr uint32_t CAN_DIAG_PERIOD_MS = 500; // シリアル診断出力の周期

FDCAN_HandleTypeDef g_hfdcan1;
uint32_t g_last_rx_ms = 0;
uint32_t g_last_tx_ms = 0;
uint32_t g_last_diag_ms = 0;

uint8_t canChunkValueCount(uint8_t chunk) {
    const uint8_t remaining = (uint8_t)(CAN_SLOTS_PER_NODE - chunk * CAN_VALUES_PER_FRAME);
    return (remaining < CAN_VALUES_PER_FRAME) ? remaining : CAN_VALUES_PER_FRAME;
}

void fdcanGpioInit() {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF9_FDCAN1;

    gpio.Pin = GPIO_PIN_11;
    HAL_GPIO_Init(GPIOA, &gpio); // FDCAN1_RX (基板シルク: A_CAN_RX)

    gpio.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &gpio); // FDCAN1_TX (基板シルク: A_CAN_TX)

    // CAN_TERM (PC14): オンボード120Ω終端抵抗のGPIO切替(ST UM2516 Table 3)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, CAN_TERM_ENABLE ? GPIO_PIN_SET : GPIO_PIN_RESET);
    GPIO_InitTypeDef term_gpio = {0};
    term_gpio.Pin = GPIO_PIN_14;
    term_gpio.Mode = GPIO_MODE_OUTPUT_PP;
    term_gpio.Pull = GPIO_NOPULL;
    term_gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &term_gpio);
}

void fdcanClockInit() {
    RCC_PeriphCLKInitTypeDef periph = {0};
    periph.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    periph.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    HAL_RCCEx_PeriphCLKConfig(&periph);

    __HAL_RCC_FDCAN_CLK_ENABLE();
}

} // namespace

void canTaskInit() {
    fdcanGpioInit();
    fdcanClockInit();

    g_hfdcan1.Instance = FDCAN1;
    g_hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    g_hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    g_hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
    g_hfdcan1.Init.AutoRetransmission = ENABLE;
    g_hfdcan1.Init.TransmitPause = DISABLE;
    g_hfdcan1.Init.ProtocolException = DISABLE;
    // 500kbit/s (ホストのTWAI_TIMING_CONFIG_500KBITSと合わせる)。上のコメント参照。
    g_hfdcan1.Init.NominalPrescaler = 2;
    g_hfdcan1.Init.NominalSyncJumpWidth = 16;
    g_hfdcan1.Init.NominalTimeSeg1 = 148;
    g_hfdcan1.Init.NominalTimeSeg2 = 21;
    // Classic CANのみ使用するためデータフェーズ設定は未使用。有効範囲内の値を形式的に設定する。
    g_hfdcan1.Init.DataPrescaler = 1;
    g_hfdcan1.Init.DataSyncJumpWidth = 1;
    g_hfdcan1.Init.DataTimeSeg1 = 2;
    g_hfdcan1.Init.DataTimeSeg2 = 2;
    g_hfdcan1.Init.StdFiltersNbr = 1;
    g_hfdcan1.Init.ExtFiltersNbr = 0;
    g_hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

    if (HAL_FDCAN_Init(&g_hfdcan1) != HAL_OK) {
        // 初期化失敗時はモータを絶対に動かさない(フェイルセーフで停止したまま)。
        // Serialは動くはずなので、無言で固まる前に必ず理由を出力する。
        Serial.println("[CAN] HAL_FDCAN_Init FAILED. Halting (motor will never move).");
        while (1) {
            delay(1000);
            Serial.println("[CAN] halted: HAL_FDCAN_Init failed at startup.");
        }
    }

    // 自ノード宛て指令(CAN_NODE_INDEX*16 .. +15)のIDだけを受信する。
    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_RANGE;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = CAN_FRAME_ID_CMD_BASE + CAN_NODE_INDEX * 16U;
    filter.FilterID2 = CAN_FRAME_ID_CMD_BASE + CAN_NODE_INDEX * 16U + 15U;
    HAL_FDCAN_ConfigFilter(&g_hfdcan1, &filter);
    HAL_FDCAN_ConfigGlobalFilter(&g_hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

    HAL_FDCAN_Start(&g_hfdcan1);

    g_last_tx_ms = millis();
    g_last_diag_ms = millis();

    Serial.print("[CAN] FDCAN1 started. CAN_ID=");
    Serial.print(CAN_ID);
    Serial.print(" CAN_NODE_INDEX=");
    Serial.print((unsigned)CAN_NODE_INDEX);
    Serial.print(" rx_filter=0x");
    Serial.print(filter.FilterID1, HEX);
    Serial.print("-0x");
    Serial.println(filter.FilterID2, HEX);
}

void canTaskPrintDiagnostics() {
    FDCAN_ProtocolStatusTypeDef proto = {};
    FDCAN_ErrorCountersTypeDef err = {};
    HAL_FDCAN_GetProtocolStatus(&g_hfdcan1, &proto);
    HAL_FDCAN_GetErrorCounters(&g_hfdcan1, &err);

    const uint32_t now = millis();
    Serial.print("[CAN] t=");
    Serial.print(now);
    Serial.print("ms last_rx=");
    Serial.print(now - g_last_rx_ms);
    Serial.print("ms_ago BusOff=");
    Serial.print(proto.BusOff);
    Serial.print(" ErrPassive=");
    Serial.print(proto.ErrorPassive);
    Serial.print(" Warning=");
    Serial.print(proto.Warning);
    Serial.print(" LastErrorCode=");
    Serial.print(proto.LastErrorCode);
    Serial.print(" TxErrCnt=");
    Serial.print(err.TxErrorCnt);
    Serial.print(" RxErrCnt=");
    Serial.print(err.RxErrorCnt);
    Serial.print(" target_velocity=");
    Serial.println(Rx_16Data[RX_TARGET_VELOCITY]);
}

void canTaskUpdate() {
    // ---- RX (host -> node) ----
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    while (HAL_FDCAN_GetRxFifoFillLevel(&g_hfdcan1, FDCAN_RX_FIFO0) > 0) {
        if (HAL_FDCAN_GetRxMessage(&g_hfdcan1, FDCAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
            break;
        }

        const uint32_t base = CAN_FRAME_ID_CMD_BASE + CAN_NODE_INDEX * 16U;
        if (rxHeader.Identifier < base) {
            continue;
        }
        const uint32_t chunk = rxHeader.Identifier - base;
        if (chunk >= CAN_CHUNK_COUNT) {
            continue;
        }

        const uint8_t values_to_unpack = canChunkValueCount((uint8_t)chunk);
        const uint8_t slot_offset = (uint8_t)(chunk * CAN_VALUES_PER_FRAME);

        for (uint8_t i = 0; i < values_to_unpack; i++) {
            const uint8_t byte_index = (uint8_t)(i * 2);
            const int16_t hi = (int16_t)((uint8_t)rxData[byte_index] << 8);
            const int16_t lo = (int16_t)((uint8_t)rxData[byte_index + 1]);
            Rx_16Data[slot_offset + i] = (int16_t)(hi | lo);
        }

        g_last_rx_ms = millis();
    }

    // ---- 診断出力 (TXの周期ゲートより前に評価し、独立した周期で必ず動かす) ----
    if (millis() - g_last_diag_ms >= CAN_DIAG_PERIOD_MS) {
        g_last_diag_ms = millis();
        canTaskPrintDiagnostics();
    }

    // ---- TX (node -> host) ----
    const uint32_t now = millis();
    if (now - g_last_tx_ms < CAN_TX_PERIOD_MS) {
        return;
    }
    g_last_tx_ms = now;

    for (uint8_t chunk = 0; chunk < CAN_CHUNK_COUNT; chunk++) {
        if (HAL_FDCAN_GetTxFifoFreeLevel(&g_hfdcan1) == 0) {
            break;
        }

        FDCAN_TxHeaderTypeDef txHeader = {0};
        txHeader.Identifier = CAN_FRAME_ID_FB_BASE + CAN_NODE_INDEX * 16U + chunk;
        txHeader.IdType = FDCAN_STANDARD_ID;
        txHeader.TxFrameType = FDCAN_DATA_FRAME;
        txHeader.DataLength = FDCAN_DLC_BYTES_8;
        txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        txHeader.BitRateSwitch = FDCAN_BRS_OFF;
        txHeader.FDFormat = FDCAN_CLASSIC_CAN;
        txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        txHeader.MessageMarker = 0;

        uint8_t data[8] = {0};
        const uint8_t values_to_pack = canChunkValueCount(chunk);
        const uint8_t slot_offset = (uint8_t)(chunk * CAN_VALUES_PER_FRAME);
        for (uint8_t i = 0; i < values_to_pack; i++) {
            const int16_t value = Tx_16Data[slot_offset + i];
            data[i * 2] = (uint8_t)(value >> 8);
            data[i * 2 + 1] = (uint8_t)(value & 0xFF);
        }

        HAL_FDCAN_AddMessageToTxFifoQ(&g_hfdcan1, &txHeader, data);
    }
}

uint32_t canLastRxMs() {
    return g_last_rx_ms;
}
