#include "CanCommTask.h"
#include "driver/twai.h"
#include "esp_err.h"
#include "esp_ldo_regulator.h"

#include <string.h>

static_assert(
    CAN_ERROR_DETAIL_FRAME_COUNT == (CAN_ID_ERROR_DETAIL_LAST - CAN_ID_ERROR_DETAIL_BASE + 1),
    "Error-detail ID range must match frame count.");
static_assert(
    CAN_ERROR_DETAIL_FRAME_COUNT * CAN_ERROR_DETAIL_CODES_PER_FRAME == ENCODER_TOTAL_NUM,
    "Error-detail payload must cover all encoder channels.");
static_assert(
    CAN_ERROR_DETAIL_DLC == (1 + CAN_ERROR_DETAIL_CODES_PER_FRAME),
    "Error-detail DLC must be seq(1) + channel codes.");
static_assert(ENCODER_TOTAL_NUM <= 32, "errorBitmap only supports up to 32 channels.");

static_assert(
    CAN_TAC_SUMMARY_FRAME_COUNT == (CAN_ID_TAC_SUMMARY_LAST - CAN_ID_TAC_SUMMARY_BASE + 1),
    "Tactile frame ID range must match frame count.");
static_assert(
    CAN_TAC_SUMMARY_DLC == (1 + CAN_TAC_SUMMARY_PAYLOAD_PER_FRAME),
    "Tactile DLC must be seq(1) + payload bytes.");
static_assert(
    TACTILE_SUMMARY_BYTES <= (CAN_TAC_SUMMARY_FRAME_COUNT * CAN_TAC_SUMMARY_PAYLOAD_PER_FRAME),
    "Tactile summary payload budget too small.");

// 错误详情帧重组状态：
// 三帧使用相同 seq 才会组成一批完整的 21 通道错误码。
typedef struct {
    uint8_t active;
    uint8_t seq;
    uint8_t frameMask;
    uint8_t codes[ENCODER_TOTAL_NUM];
    uint32_t firstFrameTimeMs;
} ErrorDetailReassemblyState;

static esp_ldo_channel_handle_t s_canVo4Ldo = NULL;

static bool twaiUsesVo4Domain(void) {
    // ESP32-P4 的 GPIO47/48 位于 VO4 电源域，使用前需要打开 LDO。
    return TWAI_TX_PIN == 47 || TWAI_TX_PIN == 48 ||
           TWAI_RX_PIN == 47 || TWAI_RX_PIN == 48;
}

static void ensureTwaiGpioPowerReady(void) {
    // 若 CAN 引脚使用 VO4 电源域，则显式拉起 3.3V，避免 TWAI 无法收发。
    static bool powered = false;
    if (powered || !twaiUsesVo4Domain()) {
        return;
    }

    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = 4,
        .voltage_mv = 3300,
        .flags = {
            .adjustable = 1,
        },
    };

    esp_err_t ret = esp_ldo_acquire_channel(&ldo_cfg, &s_canVo4Ldo);
    if (ret != ESP_OK) {
        Serial.printf("[CAN] VO4 acquire failed: %s\n", esp_err_to_name(ret));
        return;
    }

    ret = esp_ldo_channel_adjust_voltage(s_canVo4Ldo, 3300);
    if (ret != ESP_OK) {
        Serial.printf("[CAN] VO4 set 3.3V failed: %s\n", esp_err_to_name(ret));
        return;
    }

    powered = true;
    Serial.println("[CAN] VO4 set to 3.3V for GPIO47/48");
    vTaskDelay(pdMS_TO_TICKS(20));
}

typedef struct {
    // 触觉汇总帧重组状态，7 帧组成一个完整 RemoteTactileData_t。
    uint8_t active;
    uint8_t seq;
    uint8_t frameMask;
    uint8_t bytes[TACTILE_SUMMARY_BYTES];
    uint32_t firstFrameTimeMs;
} TactileReassemblyState;

static inline void resetErrorDetailReassembly(ErrorDetailReassemblyState* state) {
    if (!state) {
        return;
    }
    state->active = 0;
    state->seq = 0;
    state->frameMask = 0;
    state->firstFrameTimeMs = 0;
    memset(state->codes, 0, sizeof(state->codes));
}

static inline void resetTactileReassembly(TactileReassemblyState* state) {
    if (!state) {
        return;
    }
    state->active = 0;
    state->seq = 0;
    state->frameMask = 0;
    state->firstFrameTimeMs = 0;
    memset(state->bytes, 0, sizeof(state->bytes));
}

static uint32_t buildErrorBitmap(const uint8_t* codes) {
    // 将 21 通道错误码压缩成 bitmap，便于状态机快速判断是否存在错误。
    if (!codes) {
        return 0;
    }
    uint32_t bitmap = 0;
    for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
        if (codes[i] != 0) {
            bitmap |= (1UL << i);
        }
    }
    return bitmap;
}

static void setupTwai() {
    // TWAI 驱动只安装一次；重复调用可安全返回。
    static bool installed = false;
    if (installed) {
        return;
    }

    ensureTwaiGpioPowerReady();

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)TWAI_TX_PIN,
        (gpio_num_t)TWAI_RX_PIN,
        TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    g_config.rx_queue_len = 64;
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        twai_start();
        installed = true;
        Serial.println("[CAN] Driver Installed OK");
    }
}

void canCommunicationTask(void* parameter) {
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    setupTwai();

    RemoteSensorData_t rxBuffer;
    memset(&rxBuffer, 0, sizeof(rxBuffer));

    ErrorDetailReassemblyState errorState;
    resetErrorDetailReassembly(&errorState);

    TactileReassemblyState tactileState;
    resetTactileReassembly(&tactileState);

    twai_message_t rxMsg;
    uint32_t lastRxTime = 0;
    uint32_t lastCompleteErrorBatchTime = millis();
    RemoteCommand_t txCmd;

    const uint8_t kAllErrorFramesMask = (uint8_t)((1U << CAN_ERROR_DETAIL_FRAME_COUNT) - 1U);
    const uint8_t kAllTactileFramesMask = (uint8_t)((1U << CAN_TAC_SUMMARY_FRAME_COUNT) - 1U);

    for (;;) {
        while (twai_receive(&rxMsg, 0) == ESP_OK) {
            const uint32_t nowMs = millis();
            lastRxTime = nowMs;

            if (rxMsg.identifier >= CAN_ID_ENC_BASE && rxMsg.identifier <= CAN_ID_ENC_LAST) {
                // 编码器帧按 4 通道一帧连续排列，最后一帧到达时发布完整快照。
                const int frameIdx = (int)(rxMsg.identifier - CAN_ID_ENC_BASE);
                const int baseIdx = frameIdx * 4;
                const int payloadLen = (int)rxMsg.data_length_code;

                for (int i = 0; i < 4; i++) {
                    const int realIdx = baseIdx + i;
                    const int dataOffset = i * 2;
                    if (realIdx >= ENCODER_TOTAL_NUM) {
                        continue;
                    }
                    if ((dataOffset + 1) >= payloadLen) {
                        break;
                    }
                    const uint16_t val = ((uint16_t)rxMsg.data[dataOffset] << 8) |
                                         rxMsg.data[dataOffset + 1];
                    rxBuffer.encoderValues[realIdx] = val;
                }

                if (rxMsg.identifier == CAN_ID_ENC_LAST) {
                    rxBuffer.timestamp = nowMs;
                    rxBuffer.isValid = true;
                    xQueueOverwrite(sharedData->canRxQueue, &rxBuffer);
                }
            } else if (rxMsg.identifier >= CAN_ID_ERROR_DETAIL_BASE &&
                       rxMsg.identifier <= CAN_ID_ERROR_DETAIL_LAST) {
                // 错误详情为分片协议，seq 改变或超时都表示上一批丢弃。
                if (rxMsg.data_length_code != CAN_ERROR_DETAIL_DLC) {
                    continue;
                }

                const uint8_t seq = rxMsg.data[0];
                const uint8_t frameIdx = (uint8_t)(rxMsg.identifier - CAN_ID_ERROR_DETAIL_BASE);

                if (errorState.active) {
                    const bool seqChanged = (seq != errorState.seq);
                    const bool timedOut =
                        ((uint32_t)(nowMs - errorState.firstFrameTimeMs) > CAN_ERROR_REASSEMBLY_TIMEOUT_MS);
                    if (seqChanged || timedOut) {
                        resetErrorDetailReassembly(&errorState);
                    }
                }

                if (!errorState.active) {
                    errorState.active = 1;
                    errorState.seq = seq;
                    errorState.frameMask = 0;
                    errorState.firstFrameTimeMs = nowMs;
                    memset(errorState.codes, 0, sizeof(errorState.codes));
                }

                const int baseIdx = (int)frameIdx * CAN_ERROR_DETAIL_CODES_PER_FRAME;
                for (int i = 0; i < CAN_ERROR_DETAIL_CODES_PER_FRAME; i++) {
                    const int channelIdx = baseIdx + i;
                    if (channelIdx < ENCODER_TOTAL_NUM) {
                        errorState.codes[channelIdx] = rxMsg.data[1 + i];
                    }
                }

                errorState.frameMask |= (uint8_t)(1U << frameIdx);
                if (errorState.frameMask == kAllErrorFramesMask) {
                    memcpy(rxBuffer.errorFlags, errorState.codes, sizeof(rxBuffer.errorFlags));
                    rxBuffer.errorBitmap = buildErrorBitmap(rxBuffer.errorFlags);
                    lastCompleteErrorBatchTime = nowMs;
                    resetErrorDetailReassembly(&errorState);
                }
            } else if (rxMsg.identifier >= CAN_ID_TAC_SUMMARY_BASE &&
                       rxMsg.identifier <= CAN_ID_TAC_SUMMARY_LAST) {
                // 触觉数据同样使用 seq + frameMask 重组，完整后覆盖 tactileQueue。
                if (rxMsg.data_length_code != CAN_TAC_SUMMARY_DLC) {
                    continue;
                }

                const uint8_t seq = rxMsg.data[0];
                const uint8_t frameIdx = (uint8_t)(rxMsg.identifier - CAN_ID_TAC_SUMMARY_BASE);

                if (tactileState.active) {
                    const bool seqChanged = (seq != tactileState.seq);
                    const bool timedOut =
                        ((uint32_t)(nowMs - tactileState.firstFrameTimeMs) > CAN_TAC_REASSEMBLY_TIMEOUT_MS);
                    if (seqChanged || timedOut) {
                        resetTactileReassembly(&tactileState);
                    }
                }

                if (!tactileState.active) {
                    tactileState.active = 1;
                    tactileState.seq = seq;
                    tactileState.frameMask = 0;
                    tactileState.firstFrameTimeMs = nowMs;
                    memset(tactileState.bytes, 0, sizeof(tactileState.bytes));
                }

                const int baseIdx = (int)frameIdx * CAN_TAC_SUMMARY_PAYLOAD_PER_FRAME;
                for (int i = 0; i < CAN_TAC_SUMMARY_PAYLOAD_PER_FRAME; i++) {
                    const int tactileIdx = baseIdx + i;
                    if (tactileIdx < TACTILE_SUMMARY_BYTES) {
                        tactileState.bytes[tactileIdx] = rxMsg.data[1 + i];
                    }
                }

                tactileState.frameMask |= (uint8_t)(1U << frameIdx);
                if (tactileState.frameMask == kAllTactileFramesMask) {
                    if (sharedData->tactileQueue) {
                        RemoteTactileData_t tactileData;
                        memset(&tactileData, 0, sizeof(tactileData));
                        memcpy(tactileData.values, tactileState.bytes, sizeof(tactileData.values));
                        tactileData.seq = tactileState.seq;
                        tactileData.timestamp = nowMs;
                        tactileData.isValid = true;
                        xQueueOverwrite(sharedData->tactileQueue, &tactileData);
                    }
                    resetTactileReassembly(&tactileState);
                }
            }
        }

        const uint32_t nowMs = millis();
        if (errorState.active &&
            (uint32_t)(nowMs - errorState.firstFrameTimeMs) > CAN_ERROR_REASSEMBLY_TIMEOUT_MS) {
            // 分片长时间不完整时丢弃，避免后续 seq 混入旧数据。
            resetErrorDetailReassembly(&errorState);
        }

        if (tactileState.active &&
            (uint32_t)(nowMs - tactileState.firstFrameTimeMs) > CAN_TAC_REASSEMBLY_TIMEOUT_MS) {
            resetTactileReassembly(&tactileState);
        }

        if ((uint32_t)(nowMs - lastCompleteErrorBatchTime) > CAN_ERROR_TABLE_CLEAR_TIMEOUT_MS &&
            rxBuffer.errorBitmap != 0) {
            // 错误详情表超时未刷新时清空，防止历史错误永久停留。
            memset(rxBuffer.errorFlags, 0, sizeof(rxBuffer.errorFlags));
            rxBuffer.errorBitmap = 0;
        }

        if (millis() - lastRxTime > 500 && rxBuffer.isValid) {
            // Keep compatibility with existing behavior: no immediate invalidation.
        }

        if (xQueueReceive(sharedData->canTxQueue, &txCmd, 0) == pdTRUE) {
            // 保留上层 CAN 下发通道，目前主要用于后续设备命令扩展。
            twai_message_t txMsg;
            txMsg.identifier = txCmd.cmdID;
            txMsg.extd = 0;
            txMsg.data_length_code = txCmd.len;
            memcpy(txMsg.data, txCmd.payload, txCmd.len);
            twai_transmit(&txMsg, pdMS_TO_TICKS(10));
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
