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
    CAN_ENC_FRAME_COUNT == (CAN_ID_ENC_LAST - CAN_ID_ENC_BASE + 1),
    "Encoder frame ID range must match frame count.");
static_assert(CAN_ID_ENC_CRC <= 0x7FF, "Encoder CRC CAN ID must fit standard CAN.");

static_assert(
    CAN_TAC_SUMMARY_FRAME_COUNT == (CAN_ID_TAC_SUMMARY_LAST - CAN_ID_TAC_SUMMARY_BASE + 1),
    "Tactile frame ID range must match frame count.");
static_assert(
    CAN_TAC_SUMMARY_DLC == (1 + CAN_TAC_SUMMARY_PAYLOAD_PER_FRAME),
    "Tactile DLC must be seq(1) + payload bytes.");
static_assert(
    CAN_TAC_SUMMARY_PACKET_BYTES <= (CAN_TAC_SUMMARY_FRAME_COUNT * CAN_TAC_SUMMARY_PAYLOAD_PER_FRAME),
    "Tactile summary payload plus CRC budget too small.");
static_assert(
    CAN_TAC_FULL_DLC == (1 + CAN_TAC_FULL_PAYLOAD_PER_FRAME),
    "Tactile full DLC must be seq(1) + payload bytes.");
static_assert(
    TACTILE_FULL_BYTES <= (CAN_TAC_FULL_FRAME_COUNT * CAN_TAC_FULL_PAYLOAD_PER_FRAME),
    "Tactile full payload budget too small.");
static_assert(CAN_ID_TAC_FULL_DUMP_LAST <= 0x7FF, "Tactile full CAN IDs must fit standard CAN.");

typedef struct {
    uint8_t active;
    uint8_t frameMask;
    uint8_t bytes[CAN_ENC_PAYLOAD_BYTES];
    uint32_t firstFrameTimeMs;
} EncoderReassemblyState;

typedef struct {
    uint8_t active;
    uint8_t seq;
    uint8_t frameMask;
    uint8_t codes[ENCODER_TOTAL_NUM];
    uint32_t firstFrameTimeMs;
} ErrorDetailReassemblyState;

typedef struct {
    uint8_t active;
    uint8_t seq;
    uint8_t frameMask;
    uint8_t packet[CAN_TAC_SUMMARY_PACKET_BYTES];
    uint32_t firstFrameTimeMs;
} TactileReassemblyState;

typedef struct {
    uint8_t active;
    uint8_t seq;
    uint16_t receivedFrames;
    uint8_t frameMask[CAN_TAC_FULL_FRAME_MASK_BYTES];
    uint8_t bytes[TACTILE_FULL_BYTES];
    uint32_t firstFrameTimeMs;
} TactileFullReassemblyState;

static esp_ldo_channel_handle_t s_canVo4Ldo = NULL;

static bool twaiUsesVo4Domain(void) {
    return TWAI_TX_PIN == 47 || TWAI_TX_PIN == 48 ||
           TWAI_RX_PIN == 47 || TWAI_RX_PIN == 48;
}

static void ensureTwaiGpioPowerReady(void) {
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

static inline void resetEncoderReassembly(EncoderReassemblyState* state) {
    if (!state) {
        return;
    }
    state->active = 0;
    state->frameMask = 0;
    state->firstFrameTimeMs = 0;
    memset(state->bytes, 0, sizeof(state->bytes));
}

static inline void resetTactileReassembly(TactileReassemblyState* state) {
    if (!state) {
        return;
    }
    state->active = 0;
    state->seq = 0;
    state->frameMask = 0;
    state->firstFrameTimeMs = 0;
    memset(state->packet, 0, sizeof(state->packet));
}

static inline void resetTactileFullReassembly(TactileFullReassemblyState* state) {
    if (!state) {
        return;
    }
    state->active = 0;
    state->seq = 0;
    state->receivedFrames = 0;
    state->firstFrameTimeMs = 0;
    memset(state->frameMask, 0, sizeof(state->frameMask));
    memset(state->bytes, 0, sizeof(state->bytes));
}

static inline bool tactileFullFrameSeen(const TactileFullReassemblyState* state, uint16_t frameIdx) {
    return (state->frameMask[frameIdx >> 3] & (uint8_t)(1U << (frameIdx & 0x07))) != 0;
}

static inline void markTactileFullFrameSeen(TactileFullReassemblyState* state, uint16_t frameIdx) {
    state->frameMask[frameIdx >> 3] |= (uint8_t)(1U << (frameIdx & 0x07));
}

static uint16_t crc16Ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

static void queueLatestTactile(QueueHandle_t queue, const RemoteTactileData_t* data) {
    if (!queue || !data) {
        return;
    }
    if (xQueueSend(queue, data, 0) != pdTRUE) {
        RemoteTactileData_t dropped;
        xQueueReceive(queue, &dropped, 0);
        xQueueSend(queue, data, 0);
    }
}

static uint32_t buildErrorBitmap(const uint8_t* codes) {
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

    g_config.rx_queue_len = 512;
    esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
    if (ret != ESP_OK) {
        Serial.printf("[CAN] Driver install failed: %s\n", esp_err_to_name(ret));
        return;
    }

    ret = twai_start();
    if (ret != ESP_OK) {
        Serial.printf("[CAN] Driver start failed: %s\n", esp_err_to_name(ret));
        return;
    }

    installed = true;
    Serial.println("[CAN] Driver installed and started OK");
}

void canCommunicationTask(void* parameter) {
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    setupTwai();

    RemoteSensorData_t rxBuffer;
    memset(&rxBuffer, 0, sizeof(rxBuffer));

    EncoderReassemblyState encoderState;
    resetEncoderReassembly(&encoderState);

    ErrorDetailReassemblyState errorState;
    resetErrorDetailReassembly(&errorState);

    TactileReassemblyState tactileState;
    resetTactileReassembly(&tactileState);

    static TactileFullReassemblyState tactileFullState;
    resetTactileFullReassembly(&tactileFullState);

    static RemoteTactileData_t latestTactileData;
    memset(&latestTactileData, 0, sizeof(latestTactileData));
    static uint8_t pendingFullBytes[TACTILE_FULL_BYTES];
    static bool pendingFullValid = false;
    static uint8_t pendingFullSeq = 0;
    static uint32_t pendingFullTimestamp = 0;
    memset(pendingFullBytes, 0, sizeof(pendingFullBytes));

    twai_message_t rxMsg;
    uint32_t lastRxTime = 0;
    uint32_t lastCompleteErrorBatchTime = millis();
    uint32_t lastCanDiagPrintMs = millis();
    uint32_t encFrameCount = 0;
    uint32_t encSnapshotCount = 0;
    uint32_t encoderCrcErrorCount = 0;
    uint32_t tactileFrameCount = 0;
    uint32_t tactileSnapshotCount = 0;
    uint32_t tactileCrcErrorCount = 0;
    uint32_t otherFrameCount = 0;
    uint32_t lastCanId = 0;
    RemoteCommand_t txCmd;

    const uint8_t kAllEncoderFramesMask = (uint8_t)((1U << CAN_ENC_FRAME_COUNT) - 1U);
    const uint8_t kAllErrorFramesMask = (uint8_t)((1U << CAN_ERROR_DETAIL_FRAME_COUNT) - 1U);
    const uint8_t kAllTactileFramesMask = (uint8_t)((1U << CAN_TAC_SUMMARY_FRAME_COUNT) - 1U);

    for (;;) {
        while (twai_receive(&rxMsg, 0) == ESP_OK) {
            const uint32_t nowMs = millis();
            lastRxTime = nowMs;
            lastCanId = rxMsg.identifier;

            if (rxMsg.identifier >= CAN_ID_ENC_BASE && rxMsg.identifier <= CAN_ID_ENC_LAST) {
                encFrameCount++;
                if (rxMsg.data_length_code != 8) {
                    encoderCrcErrorCount++;
                    continue;
                }

                const uint8_t frameIdx = (uint8_t)(rxMsg.identifier - CAN_ID_ENC_BASE);
                if (!encoderState.active) {
                    encoderState.active = 1;
                    encoderState.frameMask = 0;
                    encoderState.firstFrameTimeMs = nowMs;
                    memset(encoderState.bytes, 0, sizeof(encoderState.bytes));
                }

                const int baseIdx = (int)frameIdx * 4;
                for (int i = 0; i < 4; i++) {
                    const int realIdx = baseIdx + i;
                    const int dataOffset = i * 2;
                    if (realIdx >= ENCODER_TOTAL_NUM) {
                        break;
                    }
                    encoderState.bytes[realIdx * 2] = rxMsg.data[dataOffset];
                    encoderState.bytes[realIdx * 2 + 1] = rxMsg.data[dataOffset + 1];
                }

                encoderState.frameMask |= (uint8_t)(1U << frameIdx);
            } else if (rxMsg.identifier == CAN_ID_ENC_CRC) {
                if (rxMsg.data_length_code != CAN_ENC_CRC_DLC) {
                    encoderCrcErrorCount++;
                    resetEncoderReassembly(&encoderState);
                    continue;
                }
                if (!encoderState.active || encoderState.frameMask != kAllEncoderFramesMask) {
                    encoderCrcErrorCount++;
                    resetEncoderReassembly(&encoderState);
                    continue;
                }

                const uint16_t expectedCrc =
                    ((uint16_t)rxMsg.data[1] << 8) | rxMsg.data[2];
                const uint16_t actualCrc =
                    crc16Ccitt(encoderState.bytes, sizeof(encoderState.bytes));
                const bool metadataOk =
                    rxMsg.data[3] == CAN_ENC_FRAME_COUNT &&
                    rxMsg.data[4] == CAN_ENC_PAYLOAD_BYTES;
                if (metadataOk && actualCrc == expectedCrc) {
                    for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
                        rxBuffer.encoderValues[i] =
                            ((uint16_t)encoderState.bytes[i * 2] << 8) |
                            encoderState.bytes[i * 2 + 1];
                    }
                    rxBuffer.timestamp = nowMs;
                    rxBuffer.isValid = true;
                    xQueueOverwrite(sharedData->canRxQueue, &rxBuffer);
                    encSnapshotCount++;
                } else {
                    encoderCrcErrorCount++;
                }
                resetEncoderReassembly(&encoderState);
            } else if (rxMsg.identifier >= CAN_ID_ERROR_DETAIL_BASE &&
                       rxMsg.identifier <= CAN_ID_ERROR_DETAIL_LAST) {
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
                tactileFrameCount++;
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
                    memset(tactileState.packet, 0, sizeof(tactileState.packet));
                }

                const int baseIdx = (int)frameIdx * CAN_TAC_SUMMARY_PAYLOAD_PER_FRAME;
                for (int i = 0; i < CAN_TAC_SUMMARY_PAYLOAD_PER_FRAME; i++) {
                    const int tactileIdx = baseIdx + i;
                    if (tactileIdx < CAN_TAC_SUMMARY_PACKET_BYTES) {
                        tactileState.packet[tactileIdx] = rxMsg.data[1 + i];
                    }
                }

                tactileState.frameMask |= (uint8_t)(1U << frameIdx);
                if (tactileState.frameMask == kAllTactileFramesMask) {
                    const uint16_t expectedCrc =
                        ((uint16_t)tactileState.packet[TACTILE_SUMMARY_BYTES] << 8) |
                        tactileState.packet[TACTILE_SUMMARY_BYTES + 1];
                    const uint16_t actualCrc =
                        crc16Ccitt(tactileState.packet, TACTILE_SUMMARY_BYTES);
                    if (actualCrc != expectedCrc) {
                        tactileCrcErrorCount++;
                        resetTactileReassembly(&tactileState);
                        continue;
                    }

                    if (sharedData->tactileQueue) {
                        static RemoteTactileData_t tactileData;
                        memcpy(latestTactileData.values, tactileState.packet, sizeof(latestTactileData.values));
                        latestTactileData.seq = tactileState.seq;
                        latestTactileData.timestamp = nowMs;
                        latestTactileData.isValid = true;
                        latestTactileData.fullValid = false;

                        tactileData = latestTactileData;
                        tactileData.fullValid = false;
                        queueLatestTactile(sharedData->tactileQueue, &tactileData);
                        tactileSnapshotCount++;

                        if (pendingFullValid && pendingFullSeq == latestTactileData.seq) {
                            memcpy(latestTactileData.fullValues,
                                   pendingFullBytes,
                                   sizeof(latestTactileData.fullValues));
                            latestTactileData.fullSeq = pendingFullSeq;
                            latestTactileData.fullTimestamp = pendingFullTimestamp;
                            latestTactileData.fullValid = true;
                            latestTactileData.timestamp = nowMs;
                            queueLatestTactile(sharedData->tactileQueue, &latestTactileData);
                            pendingFullValid = false;
                        }
                    }
                    resetTactileReassembly(&tactileState);
                }
            } else if (rxMsg.identifier == CAN_ID_TAC_RAW_DIAG) {
                if (rxMsg.data_length_code != 8) {
                    continue;
                }
                const uint8_t seq = rxMsg.data[0];
                const uint8_t group = rxMsg.data[1];
                const uint8_t sensor = rxMsg.data[2];
                const int8_t minZ = (int8_t)rxMsg.data[3];
                const int8_t maxZ = (int8_t)rxMsg.data[4];
                const uint16_t absSum = (uint16_t)rxMsg.data[5] |
                                        ((uint16_t)rxMsg.data[6] << 8);
                const uint8_t nonZero = rxMsg.data[7];
                Serial.printf("[S3TactileRawZ] seq=%u G%uS%u min=%d max=%d sum=%u nz=%u\r\n",
                              seq,
                              group,
                              sensor,
                              minZ,
                              maxZ,
                              absSum,
                              nonZero);
            } else if (rxMsg.identifier >= CAN_ID_TAC_FULL_DUMP_BASE &&
                       rxMsg.identifier <= CAN_ID_TAC_FULL_DUMP_LAST) {
                if (rxMsg.data_length_code != CAN_TAC_FULL_DLC) {
                    continue;
                }

                const uint8_t seq = rxMsg.data[0];
                const uint16_t frameIdx = (uint16_t)(rxMsg.identifier - CAN_ID_TAC_FULL_DUMP_BASE);
                if (frameIdx >= CAN_TAC_FULL_FRAME_COUNT) {
                    continue;
                }

                if (tactileFullState.active) {
                    const bool seqChanged = (seq != tactileFullState.seq);
                    const bool timedOut =
                        ((uint32_t)(nowMs - tactileFullState.firstFrameTimeMs) > CAN_TAC_FULL_REASSEMBLY_TIMEOUT_MS);
                    if (seqChanged || timedOut) {
                        resetTactileFullReassembly(&tactileFullState);
                    }
                }

                if (!tactileFullState.active) {
                    tactileFullState.active = 1;
                    tactileFullState.seq = seq;
                    tactileFullState.receivedFrames = 0;
                    tactileFullState.firstFrameTimeMs = nowMs;
                    memset(tactileFullState.frameMask, 0, sizeof(tactileFullState.frameMask));
                    memset(tactileFullState.bytes, 0, sizeof(tactileFullState.bytes));
                }

                const int baseIdx = (int)frameIdx * CAN_TAC_FULL_PAYLOAD_PER_FRAME;
                for (int i = 0; i < CAN_TAC_FULL_PAYLOAD_PER_FRAME; i++) {
                    const int tactileIdx = baseIdx + i;
                    if (tactileIdx < TACTILE_FULL_BYTES) {
                        tactileFullState.bytes[tactileIdx] = rxMsg.data[1 + i];
                    }
                }

                if (!tactileFullFrameSeen(&tactileFullState, frameIdx)) {
                    markTactileFullFrameSeen(&tactileFullState, frameIdx);
                    tactileFullState.receivedFrames++;
                }

                if (tactileFullState.receivedFrames >= CAN_TAC_FULL_FRAME_COUNT) {
                    if (sharedData->tactileQueue) {
                        if (latestTactileData.isValid &&
                            latestTactileData.seq == tactileFullState.seq) {
                            memcpy(latestTactileData.fullValues,
                                   tactileFullState.bytes,
                                   sizeof(latestTactileData.fullValues));
                            latestTactileData.fullSeq = tactileFullState.seq;
                            latestTactileData.fullTimestamp = nowMs;
                            latestTactileData.timestamp = nowMs;
                            latestTactileData.fullValid = true;
                            queueLatestTactile(sharedData->tactileQueue, &latestTactileData);
                        } else {
                            memcpy(pendingFullBytes,
                                   tactileFullState.bytes,
                                   sizeof(pendingFullBytes));
                            pendingFullSeq = tactileFullState.seq;
                            pendingFullTimestamp = nowMs;
                            pendingFullValid = true;
                        }
                    }
                    resetTactileFullReassembly(&tactileFullState);
                }
            } else {
                otherFrameCount++;
            }
        }

        const uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - lastCanDiagPrintMs) >= 10000) {
            twai_status_info_t statusInfo;
            const bool hasStatus = (twai_get_status_info(&statusInfo) == ESP_OK);
            Serial.printf("[CAN RX DBG] encFrames=%lu encSnap=%lu encCrcErr=%lu tacFrames=%lu tacSnap=%lu tacCrcErr=%lu other=%lu lastId=0x%03lX valid=%u tsAge=%lu firstRaw=%u err0=%u twai=%d rxQ=%lu rxMiss=%lu busErr=%lu rxErr=%lu txErr=%lu\r\n",
                          (unsigned long)encFrameCount,
                          (unsigned long)encSnapshotCount,
                          (unsigned long)encoderCrcErrorCount,
                          (unsigned long)tactileFrameCount,
                          (unsigned long)tactileSnapshotCount,
                          (unsigned long)tactileCrcErrorCount,
                          (unsigned long)otherFrameCount,
                          (unsigned long)lastCanId,
                          rxBuffer.isValid ? 1U : 0U,
                          rxBuffer.isValid ? (unsigned long)(nowMs - rxBuffer.timestamp) : 0UL,
                          (unsigned)rxBuffer.encoderValues[0],
                          (unsigned)rxBuffer.errorFlags[0],
                          hasStatus ? (int)statusInfo.state : -1,
                          hasStatus ? (unsigned long)statusInfo.msgs_to_rx : 0UL,
                          hasStatus ? (unsigned long)statusInfo.rx_missed_count : 0UL,
                          hasStatus ? (unsigned long)statusInfo.bus_error_count : 0UL,
                          hasStatus ? (unsigned long)statusInfo.rx_error_counter : 0UL,
                          hasStatus ? (unsigned long)statusInfo.tx_error_counter : 0UL);
            encFrameCount = 0;
            encSnapshotCount = 0;
            tactileFrameCount = 0;
            tactileSnapshotCount = 0;
            otherFrameCount = 0;
            lastCanDiagPrintMs = nowMs;
        }

        if (errorState.active &&
            (uint32_t)(nowMs - errorState.firstFrameTimeMs) > CAN_ERROR_REASSEMBLY_TIMEOUT_MS) {
            resetErrorDetailReassembly(&errorState);
        }

        if (encoderState.active &&
            (uint32_t)(nowMs - encoderState.firstFrameTimeMs) > CAN_ENC_REASSEMBLY_TIMEOUT_MS) {
            resetEncoderReassembly(&encoderState);
        }

        if (tactileState.active &&
            (uint32_t)(nowMs - tactileState.firstFrameTimeMs) > CAN_TAC_REASSEMBLY_TIMEOUT_MS) {
            resetTactileReassembly(&tactileState);
        }

        if (tactileFullState.active &&
            (uint32_t)(nowMs - tactileFullState.firstFrameTimeMs) > CAN_TAC_FULL_REASSEMBLY_TIMEOUT_MS) {
            resetTactileFullReassembly(&tactileFullState);
        }

        if ((uint32_t)(nowMs - lastCompleteErrorBatchTime) > CAN_ERROR_TABLE_CLEAR_TIMEOUT_MS &&
            rxBuffer.errorBitmap != 0) {
            memset(rxBuffer.errorFlags, 0, sizeof(rxBuffer.errorFlags));
            rxBuffer.errorBitmap = 0;
        }

        if (millis() - lastRxTime > 500 && rxBuffer.isValid) {
            // Keep compatibility with existing behavior: no immediate invalidation.
        }

        if (xQueueReceive(sharedData->canTxQueue, &txCmd, 0) == pdTRUE) {
            twai_message_t txMsg;
            txMsg.identifier = txCmd.cmdID;
            txMsg.extd = 0;
            txMsg.data_length_code = txCmd.len;
            memcpy(txMsg.data, txCmd.payload, txCmd.len);
            twai_transmit(&txMsg, pdMS_TO_TICKS(10));
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
