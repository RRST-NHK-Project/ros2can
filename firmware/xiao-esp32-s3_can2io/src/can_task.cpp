/*====================================================================
<can_task.cpp>
・CAN通信（TWAI + MCP2561）を用いた制御データ転送

シリアル通信の 24 スロットを CAN バス上の最大 4 ノードへ分配し、各ノードが自分の担当スロットだけを受信・送信する。
ノードモードでは、自分の担当スロットを GPIO / サーボへ反映し、エンコーダ / スイッチの状態をそのスロットへ戻す。
ホストモードでは、シリアルで受け取った 24 スロットを各ノードへ配り、各ノードから返ってきたデータをまとめてシリアルへ返す。
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "config.hpp"
#include "defs.hpp"
#include "frame_data.hpp"
#include "status_led.hpp"
#include <Arduino.h>
#include <cstring>
#include <driver/twai.h>

constexpr uint32_t CAN_FRAME_ID_BASE = 0x100;
constexpr uint8_t CAN_FRAME_DLC = 8;
constexpr uint8_t CAN_VALUES_PER_FRAME = 4;
constexpr uint8_t CAN_CHUNK_COUNT_PER_NODE = (CAN_SLOTS_PER_NODE + CAN_VALUES_PER_FRAME - 1) / CAN_VALUES_PER_FRAME;
constexpr uint32_t CAN_TX_PERIOD_MS = 5;

static twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
static twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static portMUX_TYPE g_can_frame_lock = portMUX_INITIALIZER_UNLOCKED;

static int16_t g_monitor_node_slots[CAN_NODE_COUNT * CAN_SLOTS_PER_NODE] = {0};
static uint8_t g_monitor_node_chunk_mask[CAN_NODE_COUNT] = {0};

static uint8_t canFrameNodeIndex(uint32_t identifier);
static uint8_t canFrameChunkIndex(uint32_t identifier);
static uint8_t canChunkValueCount(uint8_t chunk);

static void copyRx16DataSnapshot(int16_t *buffer) {
    portENTER_CRITICAL(&g_can_frame_lock);
    std::memcpy(buffer, (const void *)Rx_16Data, sizeof(int16_t) * Rx16NUM);
    portEXIT_CRITICAL(&g_can_frame_lock);
}

static void publishCanFeedbackToTxBuffer(const int16_t *buffer) {
    portENTER_CRITICAL(&g_can_frame_lock);
    std::memcpy((void *)Tx_16Data, buffer, sizeof(int16_t) * Tx16NUM);
    portEXIT_CRITICAL(&g_can_frame_lock);
}

static void printCanMonitorNodeSummary(uint8_t node_index, const int16_t *slot_buffer) {
    char line[220] = {0};
    const uint8_t slot_offset = node_index * CAN_SLOTS_PER_NODE;
    snprintf(line, sizeof(line), "[CAN MON] node=%u SW1=%d SW2=%d SW3=%d ENC1=%d ENC2=%d",
             (unsigned)node_index,
             (int)slot_buffer[slot_offset + 0],
             (int)slot_buffer[slot_offset + 1],
             (int)slot_buffer[slot_offset + 2],
             (int)slot_buffer[slot_offset + 3],
             (int)slot_buffer[slot_offset + 4]);
    Serial.println(line);
}

static void updateCanMonitorNodeFromFrame(const twai_message_t &message) {
    const uint8_t frame_node = canFrameNodeIndex(message.identifier);
    if (frame_node >= CAN_NODE_COUNT) {
        return;
    }

    const uint8_t chunk = canFrameChunkIndex(message.identifier);
    const uint8_t values_to_unpack = canChunkValueCount(chunk);
    const uint8_t slot_offset = frame_node * CAN_SLOTS_PER_NODE + chunk * CAN_VALUES_PER_FRAME;

    for (uint8_t i = 0; i < values_to_unpack; ++i) {
        const uint8_t slot_index = slot_offset + i;
        if (slot_index >= CAN_NODE_COUNT * CAN_SLOTS_PER_NODE) {
            break;
        }

        const uint8_t byte_index = i * 2;
        const int16_t hi = (int16_t)((uint8_t)message.data[byte_index] << 8);
        const int16_t lo = (int16_t)((uint8_t)message.data[byte_index + 1]);
        g_monitor_node_slots[slot_index] = (int16_t)(hi | lo);
    }

    g_monitor_node_chunk_mask[frame_node] |= (uint8_t)(1u << chunk);
    if (g_monitor_node_chunk_mask[frame_node] == ((1u << CAN_CHUNK_COUNT_PER_NODE) - 1u)) {
        printCanMonitorNodeSummary(frame_node, g_monitor_node_slots);
    }
}

static uint8_t canFrameNodeIndex(uint32_t identifier) {
    // CANフレームIDからノード番号を取り出す
    return (uint8_t)((identifier - CAN_FRAME_ID_BASE) / 16U);
}

static uint8_t canFrameChunkIndex(uint32_t identifier) {
    // CANフレームIDからチャンク番号を取り出す
    return (uint8_t)((identifier - CAN_FRAME_ID_BASE) % 16U);
}

static uint8_t canChunkValueCount(uint8_t chunk) {
    const uint8_t remaining_slots = (uint8_t)(CAN_SLOTS_PER_NODE - chunk * CAN_VALUES_PER_FRAME);
    return (remaining_slots < CAN_VALUES_PER_FRAME) ? remaining_slots : CAN_VALUES_PER_FRAME;
}

static void canSendNodeSlotBlock(const int16_t *data, uint8_t node_index) {
    // 指定ノード向けにスロットデータを複数フレームに分けて送信する
    for (uint8_t chunk = 0; chunk < CAN_CHUNK_COUNT_PER_NODE; chunk++) {
        twai_message_t message{};
        message.identifier = CAN_FRAME_ID_BASE + (node_index * 16U) + chunk;
        message.data_length_code = CAN_FRAME_DLC;
        message.flags = 0;
        std::memset(message.data, 0, sizeof(message.data));

        const uint8_t values_to_pack = canChunkValueCount(chunk);
        for (uint8_t i = 0; i < values_to_pack; i++) {
            const uint8_t slot_index = node_index * CAN_SLOTS_PER_NODE + chunk * CAN_VALUES_PER_FRAME + i;
            if (slot_index >= Tx16NUM) {
                break;
            }

            const int16_t value = data[slot_index];
            message.data[i * 2] = (uint8_t)(value >> 8);
            message.data[i * 2 + 1] = (uint8_t)(value & 0xFF);
        }

        (void)twai_transmit(&message, pdMS_TO_TICKS(100));
    }
}

static void canRecvNodeSlotBlock(int16_t *buffer, uint8_t node_index) {
    // 自ノード宛てのCANフレームだけを受信してローカルバッファへ反映する
    twai_message_t message{};

    while (twai_receive(&message, pdMS_TO_TICKS(0)) == ESP_OK) {
        if (message.data_length_code != CAN_FRAME_DLC) {
            continue;
        }

        const uint8_t frame_node = canFrameNodeIndex(message.identifier);
        if (frame_node != node_index) {
            continue;
        }

        const uint8_t chunk = canFrameChunkIndex(message.identifier);
        if (chunk >= CAN_CHUNK_COUNT_PER_NODE) {
            continue;
        }

        statusLedPulse();

        const uint8_t slot_offset = node_index * CAN_SLOTS_PER_NODE + chunk * CAN_VALUES_PER_FRAME;
        const uint8_t values_to_unpack = canChunkValueCount(chunk);
        for (uint8_t i = 0; i < values_to_unpack; i++) {
            const uint8_t slot_index = slot_offset + i;
            if (slot_index >= Rx16NUM) {
                break;
            }

            const uint8_t byte_index = i * 2;
            const int16_t hi = (int16_t)((uint8_t)message.data[byte_index] << 8);
            const int16_t lo = (int16_t)((uint8_t)message.data[byte_index + 1]);
            buffer[slot_index] = (int16_t)(hi | lo);
        }
    }
}

static void canRecvAllNodeSlotBlocks(int16_t *buffer) {
    // ホスト側で全ノードのスロットデータをまとめて受信する
    twai_message_t message{};

    while (twai_receive(&message, pdMS_TO_TICKS(0)) == ESP_OK) {
        if (message.data_length_code != CAN_FRAME_DLC) {
            continue;
        }

        const uint8_t frame_node = canFrameNodeIndex(message.identifier);
        if (frame_node >= CAN_NODE_COUNT) {
            continue;
        }

        const uint8_t chunk = canFrameChunkIndex(message.identifier);
        if (chunk >= CAN_CHUNK_COUNT_PER_NODE) {
            continue;
        }

        statusLedPulse();

        const uint8_t slot_offset = frame_node * CAN_SLOTS_PER_NODE + chunk * CAN_VALUES_PER_FRAME;
        const uint8_t values_to_unpack = canChunkValueCount(chunk);
        for (uint8_t i = 0; i < values_to_unpack; i++) {
            const uint8_t slot_index = slot_offset + i;
            if (slot_index >= Rx16NUM) {
                break;
            }

            const uint8_t byte_index = i * 2;
            const int16_t hi = (int16_t)((uint8_t)message.data[byte_index] << 8);
            const int16_t lo = (int16_t)((uint8_t)message.data[byte_index + 1]);
            buffer[slot_index] = (int16_t)(hi | lo);
        }
    }
}

static void applyNodeSlotBlockToLocalControl(const int16_t *slot_buffer, uint8_t node_index) {
    // 受信したノードデータをローカル制御用配列へ反映する
    // slot_indexはslot_buffer（全ノード分, CAN_NODE_COUNT*CAN_SLOTS_PER_NODE要素）内の絶対位置なので、
    // 書き込み先のCanIoRxData（5要素）ではなく全体サイズと比較すること
    const uint8_t slot_offset = node_index * CAN_SLOTS_PER_NODE;
    constexpr uint8_t kSlotBufferSize = CAN_NODE_COUNT * CAN_SLOTS_PER_NODE;

    for (uint8_t i = 0; i < 4; i++) {
        const uint8_t slot_index = slot_offset + i;
        if (slot_index < kSlotBufferSize) {
            CanIoRxData[i] = slot_buffer[slot_index];
        }
    }

    if (slot_offset + 4 < kSlotBufferSize) {
        CanIoRxData[4] = slot_buffer[slot_offset + 4];
    }
}

static void buildNodeSlotBlockFromLocalFeedback(int16_t *slot_buffer, uint8_t node_index) {
    // ローカルのフィードバック情報をCAN送信用バッファへ組み立てる
    // 5スロットの順序は SW1, SW2, SW3, ENC1, ENC2 とする
    const uint8_t slot_offset = node_index * CAN_SLOTS_PER_NODE;

    if (slot_offset + 4 < Tx16NUM) {
        slot_buffer[slot_offset + 0] = CanIoTxData[0];
        slot_buffer[slot_offset + 1] = CanIoTxData[1];
        slot_buffer[slot_offset + 2] = CanIoTxData[2];
        slot_buffer[slot_offset + 3] = CanIoTxData[3];
        slot_buffer[slot_offset + 4] = CanIoTxData[4];
    }
}

void canInit() {
    twai_stop();
    twai_driver_uninstall();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
}

void canTask(void *) {
    TickType_t last_tx = xTaskGetTickCount();
    // 1周期ごとに送受信を行う
    static int16_t node_slot_buffer[Tx16NUM] = {0};
    static int16_t node_feedback_buffer[Tx16NUM] = {0};
    static int16_t host_tx_payload[Rx16NUM] = {0};

    static uint32_t last_monitor_print_ms = 0;

    while (1) {
#if defined(MODE_CAN_MONITOR)
        twai_message_t message{};
        if (twai_receive(&message, pdMS_TO_TICKS(5)) == ESP_OK) {
            statusLedPulse();
            const uint32_t now_ms = millis();
            if (now_ms - last_monitor_print_ms >= 50U) {
                updateCanMonitorNodeFromFrame(message);
                last_monitor_print_ms = now_ms;
            }
        }
#elif defined(MODE_CAN_HOST)
        // node_feedback_bufferは持続的なバッファ。受信したスロットだけを更新し、
        // まだ受信していないスロットは前回値を保持する（毎ループ全消去すると
        // ノード送信周期(5ms)とホストループ周期(1ms)のズレでほぼ常にゼロを送ってしまう）
        canRecvAllNodeSlotBlocks(node_feedback_buffer);
        // ホスト自身もバス上の1ノード（CAN_NODE_INDEX）として振る舞う。
        // CANを介さずローカルのスイッチ/エンコーダ値を直接自分の担当スロットへ書き込む
        buildNodeSlotBlockFromLocalFeedback(node_feedback_buffer, CAN_NODE_INDEX);
        publishCanFeedbackToTxBuffer(node_feedback_buffer);

        if (xTaskGetTickCount() - last_tx >= pdMS_TO_TICKS(CAN_TX_PERIOD_MS)) {
            copyRx16DataSnapshot(host_tx_payload);
            // ホスト自身の担当スロットはCANへ送らず、ローカルのMD/サーボ制御へ直接反映する
            applyNodeSlotBlockToLocalControl(host_tx_payload, CAN_NODE_INDEX);
            for (uint8_t node_index = 0; node_index < CAN_NODE_COUNT; node_index++) {
                if (node_index == CAN_NODE_INDEX) {
                    continue;
                }
                canSendNodeSlotBlock(host_tx_payload, node_index);
            }
            last_tx = xTaskGetTickCount();
        }
#else
        canRecvNodeSlotBlock(node_slot_buffer, CAN_NODE_INDEX);
        applyNodeSlotBlockToLocalControl(node_slot_buffer, CAN_NODE_INDEX);

        if (xTaskGetTickCount() - last_tx >= pdMS_TO_TICKS(CAN_TX_PERIOD_MS)) {
            buildNodeSlotBlockFromLocalFeedback(node_slot_buffer, CAN_NODE_INDEX);
            canSendNodeSlotBlock(node_slot_buffer, CAN_NODE_INDEX);
            last_tx = xTaskGetTickCount();
        }
#endif

        statusLedUpdate();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
