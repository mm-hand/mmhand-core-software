/**
 * @file S3CanRxService.cpp
 * @brief 接收 S3 CAN 触觉与磁编数据并发布一致性快照。
 */
#include "S3CanRxService.h"
#include "EncoderHealthMonitor.h"
#include "../../config/protocol/S3SensorCanProtocol.h"
#include "../../config/protocol/Crc16Ccitt.h"
#include "../../hal/can/TwaiCanDriver.h"
#include "../../system/rtos/SharedSafetyState.h"
#include "../../system/log/FirmwareLog.h"
#include "../../system/TaskRuntimeMetrics.h"
#include "../../system/rtos/AtomicValue.h"
#include "../../safety/SafetyEventReporter.h"
#include "../../safety/policies/CommunicationSafetyPolicy.h"

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
    CAN_TAC_SUMMARY_PACKET_BYTES == (CAN_TAC_SUMMARY_FRAME_COUNT * CAN_TAC_SUMMARY_PAYLOAD_PER_FRAME),
    "Tactile summary payload, valid mask and CRC must fill seven frames exactly.");
static_assert(
    CAN_TAC_FULL_DLC == (1 + CAN_TAC_FULL_PAYLOAD_PER_FRAME),
    "Tactile full DLC must be seq(1) + payload bytes.");
static_assert(
    TACTILE_FULL_BYTES <= (CAN_TAC_FULL_FRAME_COUNT * CAN_TAC_FULL_PAYLOAD_PER_FRAME),
    "Tactile full payload budget too small.");
static_assert(CAN_ID_TAC_FULL_DUMP_LAST <= 0x7FF, "Tactile full CAN IDs must fit standard CAN.");

typedef struct {
    uint8_t active;                       ///< 1 表示正在等待同一批编码器分帧。
    uint8_t frameMask;                    ///< 已收到的 0x100 起始数据帧位图。
    uint8_t bytes[CAN_ENC_PAYLOAD_BYTES]; ///< 21 路 uint16 raw 的连续字节。
    uint32_t firstFrameTimeMs;            ///< 首帧 P4 单调毫秒时间，用于超时丢包。
} EncoderReassemblyState;

typedef struct {
    uint8_t active;                    ///< 是否正在重组 error detail。
    uint8_t seq;                       ///< S3 批次序号，跨批帧不得混合。
    uint8_t frameMask;                 ///< 三个错误详情帧位图。
    uint8_t codes[ENCODER_TOTAL_NUM];  ///< 每路 S3 原始错误码。
    uint32_t firstFrameTimeMs;         ///< 首帧时间。
} ErrorDetailReassemblyState;

typedef struct {
    uint8_t active;                              ///< 是否正在重组触觉摘要。
    uint8_t seq;                                 ///< S3 摘要批次序号。
    uint8_t frameMask;                           ///< 七个摘要帧位图。
    uint8_t packet[CAN_TAC_SUMMARY_PACKET_BYTES];///< 45字节摘要+2字节有效位图+CRC16。
    uint32_t firstFrameTimeMs;                   ///< 首帧时间。
} TactileReassemblyState;

typedef struct {
    uint8_t active;                                  ///< 是否正在重组全量触觉。
    uint8_t seq;                                     ///< S3 全量批次序号。
    uint16_t receivedFrames;                         ///< 已收到的不重复帧数。
    uint8_t frameMask[CAN_TAC_FULL_FRAME_MASK_BYTES];///< 每帧一位的去重表。
    uint8_t bytes[TACTILE_FULL_BYTES];               ///< 完整全量触觉字节。
    uint32_t firstFrameTimeMs;                       ///< 首帧时间。
} TactileFullReassemblyState;

/** 清空错误详情分帧状态，防止超时批次污染下一批。 */
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

/** 清空编码器数据分帧状态。 */
static inline void resetEncoderReassembly(EncoderReassemblyState* state) {
    if (!state) {
        return;
    }
    state->active = 0;
    state->frameMask = 0;
    state->firstFrameTimeMs = 0;
    memset(state->bytes, 0, sizeof(state->bytes));
}

/** 清空触觉摘要分帧状态。 */
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

/** 清空触觉全量分帧状态和逐帧去重位图。 */
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

/** 检查全量触觉 frameIdx 是否已经写入，避免重复帧重复计数。 */
static inline bool tactileFullFrameSeen(const TactileFullReassemblyState* state, uint16_t frameIdx) {
    return (state->frameMask[frameIdx >> 3] & (uint8_t)(1U << (frameIdx & 0x07))) != 0;
}

/** 标记全量触觉 frameIdx 已写入。 */
static inline void markTactileFullFrameSeen(TactileFullReassemblyState* state, uint16_t frameIdx) {
    state->frameMask[frameIdx >> 3] |= (uint8_t)(1U << (frameIdx & 0x07));
}

/** 触觉队列满时丢弃最旧帧，始终保留最新完整快照。 */
static void queueLatestTactile(
    MessageChannel<RemoteTactileData_t>& queue,
    const RemoteTactileData_t* data) {
    if (!data) {
        return;
    }
    if (!queue.send(*data)) {
        RemoteTactileData_t dropped;
        queue.receive(&dropped);
        queue.send(*data);
    }
}

/** 将 21 路非零错误码压成 Jxx 位图。 */
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

/** CAN 接入服务跨周期状态；由 S3CanRxTask 独占，避免分帧状态散落在任务栈。 */
struct SensorIngressRuntimeContext {
    bool initialized;
    TwaiCanDriver canDriver;
    RemoteSensorData_t rxBuffer;
    EncoderHealthState encoderHealth[ENCODER_TOTAL_NUM];
    EncoderReassemblyState encoderState;
    ErrorDetailReassemblyState errorState;
    TactileReassemblyState tactileState;
    TactileFullReassemblyState tactileFullState;
    RemoteTactileData_t latestTactileData;
    uint8_t pendingFullBytes[TACTILE_FULL_BYTES];
    bool pendingFullValid;
    uint8_t pendingFullSeq;
    uint32_t pendingFullTimestamp;
    uint32_t lastCompleteErrorBatchTime;
    uint32_t lastCanDiagPrintMs;
    uint32_t encFrameCount;
    uint32_t encSnapshotCount;
    uint32_t encoderCrcErrorCount;
    uint32_t tactileFrameCount;
    uint32_t tactileSnapshotCount;
    uint32_t tactileCrcErrorCount;
    uint32_t otherFrameCount;
    uint32_t lastCanId;
    uint32_t lastTwaiHealthCheckMs;
    bool twaiRecoveryRequested;
    uint32_t lastCanFrameRuleMs;       ///< COM-6002 最近上报时间。
    uint32_t lastEncoderRuleMs;        ///< COM-6003 最近上报时间。
    uint32_t lastTactileRuleMs;        ///< COM-6004 最近上报时间。
};

static SensorIngressRuntimeContext g_sensorIngressRuntime;

/** CAN 噪声可能连续产生坏帧；同类规则每秒最多记录一次，保留证据但避免事件风暴。 */
static void reportCommunicationRule(SystemRuntimeContext* sharedData,
                                    SafetyRuleId ruleId,
                                    uint32_t nowMs,
                                    uint32_t* lastReportMs,
                                    int32_t requestedValue,
                                    int32_t observedValue)
{
    if (!sharedData || !lastReportMs) return;
    if (*lastReportMs != 0 && (uint32_t)(nowMs - *lastReportMs) < 1000U) {
        return;
    }
    *lastReportMs = nowMs;
    p4::safety::reportSafetyRuleEvent(sharedData,
                                      ruleId,
                                      requestedValue,
                                      observedValue);
}

bool initS3CanRxService(SystemRuntimeContext* sharedData, uint32_t nowMs)
{
    if (!sharedData) {
        return false;
    }
    memset(&g_sensorIngressRuntime, 0, sizeof(g_sensorIngressRuntime));
    const bool canInitialized = g_sensorIngressRuntime.canDriver.begin();
    const SafetyRuleId canInitializationRule =
        p4::safety::communicationCanInitializationRule(canInitialized);
    if (canInitializationRule != SafetyRuleId::None) {
        FW_LOG_INFO_PRINTLN("[CAN] TWAI HAL initialization failed");
        p4::safety::reportSafetyRuleEvent(
            sharedData,
            canInitializationRule);
    }
    clearEncoderHealth(sharedData, g_sensorIngressRuntime.encoderHealth);
    resetEncoderReassembly(&g_sensorIngressRuntime.encoderState);
    resetErrorDetailReassembly(&g_sensorIngressRuntime.errorState);
    resetTactileReassembly(&g_sensorIngressRuntime.tactileState);
    resetTactileFullReassembly(&g_sensorIngressRuntime.tactileFullState);
    g_sensorIngressRuntime.lastCompleteErrorBatchTime = nowMs;
    g_sensorIngressRuntime.lastCanDiagPrintMs = nowMs;
    g_sensorIngressRuntime.initialized = true;
    return true;
}

bool stepS3CanRxService(SystemRuntimeContext* sharedData, uint32_t nowMs)
{
    if (!sharedData || !g_sensorIngressRuntime.initialized) {
        return false;
    }
    SensorIngressRuntimeContext& runtime = g_sensorIngressRuntime;
    auto& canDriver = runtime.canDriver;
    auto& rxBuffer = runtime.rxBuffer;
    auto& encoderHealth = runtime.encoderHealth;
    auto& encoderState = runtime.encoderState;
    auto& errorState = runtime.errorState;
    auto& tactileState = runtime.tactileState;
    auto& tactileFullState = runtime.tactileFullState;
    auto& latestTactileData = runtime.latestTactileData;
    auto& pendingFullBytes = runtime.pendingFullBytes;
    auto& pendingFullValid = runtime.pendingFullValid;
    auto& pendingFullSeq = runtime.pendingFullSeq;
    auto& pendingFullTimestamp = runtime.pendingFullTimestamp;
    auto& lastCompleteErrorBatchTime = runtime.lastCompleteErrorBatchTime;
    auto& lastCanDiagPrintMs = runtime.lastCanDiagPrintMs;
    auto& encFrameCount = runtime.encFrameCount;
    auto& encSnapshotCount = runtime.encSnapshotCount;
    auto& encoderCrcErrorCount = runtime.encoderCrcErrorCount;
    auto& tactileFrameCount = runtime.tactileFrameCount;
    auto& tactileSnapshotCount = runtime.tactileSnapshotCount;
    auto& tactileCrcErrorCount = runtime.tactileCrcErrorCount;
    auto& otherFrameCount = runtime.otherFrameCount;
    auto& lastCanId = runtime.lastCanId;
    auto& lastTwaiHealthCheckMs = runtime.lastTwaiHealthCheckMs;
    auto& twaiRecoveryRequested = runtime.twaiRecoveryRequested;
    CanFrame rxMsg;

    const uint8_t kAllEncoderFramesMask = (uint8_t)((1U << CAN_ENC_FRAME_COUNT) - 1U);
    const uint8_t kAllErrorFramesMask = (uint8_t)((1U << CAN_ERROR_DETAIL_FRAME_COUNT) - 1U);
    const uint8_t kAllTactileFramesMask = (uint8_t)((1U << CAN_TAC_SUMMARY_FRAME_COUNT) - 1U);

        while (canDriver.tryReceive(&rxMsg)) {
            lastCanId = rxMsg.identifier;

            if (rxMsg.identifier >= CAN_ID_ENC_BASE && rxMsg.identifier <= CAN_ID_ENC_LAST) {
                encFrameCount++;
                if (!p4::safety::communicationLengthValid(
                        rxMsg.dataLength, 8)) {
                    reportCommunicationRule(
                        sharedData,
                        SafetyRuleId::CommunicationCanFrameInvalid,
                        nowMs,
                        &runtime.lastCanFrameRuleMs,
                        8,
                        rxMsg.dataLength);
                    encoderCrcErrorCount++;
                    markEncoderSnapshotBad(sharedData, encoderHealth, nowMs);
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
                if (!p4::safety::communicationLengthValid(
                        rxMsg.dataLength, CAN_ENC_CRC_DLC)) {
                    reportCommunicationRule(
                        sharedData,
                        SafetyRuleId::CommunicationCanFrameInvalid,
                        nowMs,
                        &runtime.lastCanFrameRuleMs,
                        CAN_ENC_CRC_DLC,
                        rxMsg.dataLength);
                    encoderCrcErrorCount++;
                    markEncoderSnapshotBad(sharedData, encoderHealth, nowMs);
                    resetEncoderReassembly(&encoderState);
                    continue;
                }
                if (!p4::safety::communicationReassemblyComplete(
                        encoderState.active != 0,
                        encoderState.frameMask,
                        kAllEncoderFramesMask)) {
                    reportCommunicationRule(
                        sharedData,
                        SafetyRuleId::CommunicationEncoderSnapshotInvalid,
                        nowMs,
                        &runtime.lastEncoderRuleMs,
                        kAllEncoderFramesMask,
                        encoderState.frameMask);
                    encoderCrcErrorCount++;
                    markEncoderSnapshotBad(sharedData, encoderHealth, nowMs);
                    resetEncoderReassembly(&encoderState);
                    continue;
                }

                const uint16_t expectedCrc =
                    ((uint16_t)rxMsg.data[1] << 8) | rxMsg.data[2];
                const uint16_t actualCrc =
                    p4::integrity::crc16CcittFalse(
                        encoderState.bytes,
                        sizeof(encoderState.bytes));
                const bool metadataOk =
                    rxMsg.data[3] == CAN_ENC_FRAME_COUNT &&
                    rxMsg.data[4] == CAN_ENC_PAYLOAD_BYTES;
                if (p4::safety::communicationCrcValid(
                        metadataOk, actualCrc, expectedCrc)) {
                    uint32_t encoderValidMask = 0;
                    for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
                        const uint16_t rawValue =
                            ((uint16_t)encoderState.bytes[i * 2] << 8) |
                            encoderState.bytes[i * 2 + 1];
                        if (rawValue != 0xFFFFU && rxBuffer.errorFlags[i] == 0) {
                            rxBuffer.encoderValues[i] = rawValue;
                            encoderValidMask |= (1UL << i);
                        } else {
                            rxBuffer.encoderValues[i] = 0xFFFFU;
                        }
                    }
                    rxBuffer.timestamp = nowMs;
                    rxBuffer.isValid = true;
                    updateEncoderHealthFromSnapshot(sharedData,
                                                    encoderHealth,
                                                    encoderValidMask,
                                                    nowMs);
                    sharedData->canRxQueue.overwrite(rxBuffer);
                    encSnapshotCount++;
                } else {
                    reportCommunicationRule(
                        sharedData,
                        SafetyRuleId::CommunicationEncoderSnapshotInvalid,
                        nowMs,
                        &runtime.lastEncoderRuleMs,
                        expectedCrc,
                        actualCrc);
                    encoderCrcErrorCount++;
                    markEncoderSnapshotBad(sharedData, encoderHealth, nowMs);
                }
                resetEncoderReassembly(&encoderState);
            } else if (rxMsg.identifier >= CAN_ID_ERROR_DETAIL_BASE &&
                       rxMsg.identifier <= CAN_ID_ERROR_DETAIL_LAST) {
                if (!p4::safety::communicationLengthValid(
                        rxMsg.dataLength, CAN_ERROR_DETAIL_DLC)) {
                    continue;
                }

                const uint8_t seq = rxMsg.data[0];
                const uint8_t frameIdx = (uint8_t)(rxMsg.identifier - CAN_ID_ERROR_DETAIL_BASE);

                if (errorState.active) {
                    const bool seqChanged = (seq != errorState.seq);
                    const bool timedOut =
                        p4::safety::communicationAssemblyExpired(
                            nowMs,
                            errorState.firstFrameTimeMs,
                            CAN_ERROR_REASSEMBLY_TIMEOUT_MS);
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
                if (p4::safety::communicationReassemblyComplete(
                        errorState.active != 0,
                        errorState.frameMask,
                        kAllErrorFramesMask)) {
                    memcpy(rxBuffer.errorFlags, errorState.codes, sizeof(rxBuffer.errorFlags));
                    rxBuffer.errorBitmap = buildErrorBitmap(rxBuffer.errorFlags);
                    lastCompleteErrorBatchTime = nowMs;
                    resetErrorDetailReassembly(&errorState);
                }
            } else if (rxMsg.identifier >= CAN_ID_TAC_SUMMARY_BASE &&
                       rxMsg.identifier <= CAN_ID_TAC_SUMMARY_LAST) {
                tactileFrameCount++;
                if (!p4::safety::communicationLengthValid(
                        rxMsg.dataLength, CAN_TAC_SUMMARY_DLC)) {
                    reportCommunicationRule(
                        sharedData,
                        SafetyRuleId::CommunicationCanFrameInvalid,
                        nowMs,
                        &runtime.lastCanFrameRuleMs,
                        CAN_TAC_SUMMARY_DLC,
                        rxMsg.dataLength);
                    continue;
                }

                const uint8_t seq = rxMsg.data[0];
                const uint8_t frameIdx = (uint8_t)(rxMsg.identifier - CAN_ID_TAC_SUMMARY_BASE);

                if (tactileState.active) {
                    const bool seqChanged = (seq != tactileState.seq);
                    const bool timedOut =
                        p4::safety::communicationAssemblyExpired(
                            nowMs,
                            tactileState.firstFrameTimeMs,
                            CAN_TAC_REASSEMBLY_TIMEOUT_MS);
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
                if (p4::safety::communicationReassemblyComplete(
                        tactileState.active != 0,
                        tactileState.frameMask,
                        kAllTactileFramesMask)) {
                    const uint16_t expectedCrc =
                        ((uint16_t)tactileState.packet[CAN_TAC_SUMMARY_CRC_INPUT_BYTES] << 8) |
                        tactileState.packet[CAN_TAC_SUMMARY_CRC_INPUT_BYTES + 1];
                    const uint16_t actualCrc =
                        p4::integrity::crc16CcittFalse(
                            tactileState.packet,
                            CAN_TAC_SUMMARY_CRC_INPUT_BYTES);
                    if (!p4::safety::communicationCrcValid(
                            true, actualCrc, expectedCrc)) {
                        reportCommunicationRule(
                            sharedData,
                            SafetyRuleId::CommunicationTactileSnapshotInvalid,
                            nowMs,
                            &runtime.lastTactileRuleMs,
                            expectedCrc,
                            actualCrc);
                        tactileCrcErrorCount++;
                        resetTactileReassembly(&tactileState);
                        continue;
                    }

                    static RemoteTactileData_t tactileData;
                    memcpy(latestTactileData.values, tactileState.packet, sizeof(latestTactileData.values));
                    // 摘要值为零不能代表模块掉线，在线判断只使用S3 SPI读取有效位图。
                    latestTactileData.moduleValidMask =
                        ((uint16_t)tactileState.packet[TACTILE_SUMMARY_BYTES] << 8) |
                        tactileState.packet[TACTILE_SUMMARY_BYTES + 1];
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
                        // 全量帧作为独立事件入队；isValid=false 明确禁止上行层
                        // 再次发送同一 seq 的摘要。
                        tactileData = latestTactileData;
                        tactileData.isValid = false;
                        queueLatestTactile(sharedData->tactileQueue, &tactileData);
                        pendingFullValid = false;
                    }
                    resetTactileReassembly(&tactileState);
                }
            } else if (rxMsg.identifier == CAN_ID_TAC_RAW_DIAG) {
                if (!p4::safety::communicationLengthValid(
                        rxMsg.dataLength, 8)) {
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
                FW_LOG_TRACE_PRINTF("[S3TactileRawZ] seq=%u G%uS%u min=%d max=%d sum=%u nz=%u\r\n",
                                    seq,
                                    group,
                                    sensor,
                                    minZ,
                                    maxZ,
                                    absSum,
                                    nonZero);
            } else if (rxMsg.identifier >= CAN_ID_TAC_FULL_DUMP_BASE &&
                       rxMsg.identifier <= CAN_ID_TAC_FULL_DUMP_LAST) {
                if (!p4::safety::communicationLengthValid(
                        rxMsg.dataLength, CAN_TAC_FULL_DLC)) {
                    continue;
                }

                const uint8_t seq = rxMsg.data[0];
                const uint16_t frameIdx = (uint16_t)(rxMsg.identifier - CAN_ID_TAC_FULL_DUMP_BASE);
                if (!p4::safety::communicationIndexValid(
                        frameIdx, CAN_TAC_FULL_FRAME_COUNT)) {
                    continue;
                }

                if (tactileFullState.active) {
                    const bool seqChanged = (seq != tactileFullState.seq);
                    const bool timedOut =
                        p4::safety::communicationAssemblyExpired(
                            nowMs,
                            tactileFullState.firstFrameTimeMs,
                            CAN_TAC_FULL_REASSEMBLY_TIMEOUT_MS);
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

                if (p4::safety::communicationFrameCountComplete(
                        tactileFullState.receivedFrames,
                        CAN_TAC_FULL_FRAME_COUNT)) {
                    if (latestTactileData.isValid &&
                        latestTactileData.seq == tactileFullState.seq) {
                        static RemoteTactileData_t fullEvent;
                        memcpy(latestTactileData.fullValues,
                               tactileFullState.bytes,
                               sizeof(latestTactileData.fullValues));
                        latestTactileData.fullSeq = tactileFullState.seq;
                        latestTactileData.fullTimestamp = nowMs;
                        latestTactileData.timestamp = nowMs;
                        latestTactileData.fullValid = true;
                        fullEvent = latestTactileData;
                        fullEvent.isValid = false;
                        queueLatestTactile(sharedData->tactileQueue, &fullEvent);
                    } else {
                        memcpy(pendingFullBytes,
                               tactileFullState.bytes,
                               sizeof(pendingFullBytes));
                        pendingFullSeq = tactileFullState.seq;
                        pendingFullTimestamp = nowMs;
                        pendingFullValid = true;
                    }
                    resetTactileFullReassembly(&tactileFullState);
                }
            } else {
                otherFrameCount++;
            }
        }

        if ((uint32_t)(nowMs - lastCanDiagPrintMs) >= 10000) {
            const CanControllerStatus statusInfo = canDriver.status();
            const bool hasStatus = statusInfo.valid != 0;
            FW_LOG_DEBUG_PRINTF("[CAN RX DBG] encFrames=%lu encSnap=%lu encCrcErr=%lu tacFrames=%lu tacSnap=%lu tacCrcErr=%lu other=%lu lastId=0x%03lX valid=%u tsAge=%lu firstRaw=%u err0=%u twai=%d rxQ=%lu rxMiss=%lu rxOverrun=%lu busErr=%lu rxErr=%lu txErr=%lu\r\n",
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
                                hasStatus ? (unsigned long)statusInfo.pendingRxCount : 0UL,
                                hasStatus ? (unsigned long)statusInfo.rxMissedCount : 0UL,
                                hasStatus ? (unsigned long)statusInfo.rxOverrunCount : 0UL,
                                hasStatus ? (unsigned long)statusInfo.busErrorCount : 0UL,
                                hasStatus ? (unsigned long)statusInfo.rxErrorCounter : 0UL,
                                hasStatus ? (unsigned long)statusInfo.txErrorCounter : 0UL);
            encFrameCount = 0;
            encSnapshotCount = 0;
            tactileFrameCount = 0;
            tactileSnapshotCount = 0;
            otherFrameCount = 0;
            lastCanDiagPrintMs = nowMs;
        }

        if (errorState.active &&
            p4::safety::communicationAssemblyExpired(
                nowMs, errorState.firstFrameTimeMs,
                CAN_ERROR_REASSEMBLY_TIMEOUT_MS)) {
            resetErrorDetailReassembly(&errorState);
        }

        if (encoderState.active &&
            p4::safety::communicationAssemblyExpired(
                nowMs, encoderState.firstFrameTimeMs,
                CAN_ENC_REASSEMBLY_TIMEOUT_MS)) {
            reportCommunicationRule(
                sharedData,
                SafetyRuleId::CommunicationEncoderSnapshotInvalid,
                nowMs,
                &runtime.lastEncoderRuleMs,
                kAllEncoderFramesMask,
                encoderState.frameMask);
            resetEncoderReassembly(&encoderState);
        }

        if (tactileState.active &&
            p4::safety::communicationAssemblyExpired(
                nowMs, tactileState.firstFrameTimeMs,
                CAN_TAC_REASSEMBLY_TIMEOUT_MS)) {
            reportCommunicationRule(
                sharedData,
                SafetyRuleId::CommunicationTactileSnapshotInvalid,
                nowMs,
                &runtime.lastTactileRuleMs,
                kAllTactileFramesMask,
                tactileState.frameMask);
            resetTactileReassembly(&tactileState);
        }

        if (tactileFullState.active &&
            p4::safety::communicationAssemblyExpired(
                nowMs, tactileFullState.firstFrameTimeMs,
                CAN_TAC_FULL_REASSEMBLY_TIMEOUT_MS)) {
            resetTactileFullReassembly(&tactileFullState);
        }

        if (p4::safety::communicationAssemblyExpired(
                nowMs, lastCompleteErrorBatchTime,
                CAN_ERROR_TABLE_CLEAR_TIMEOUT_MS) &&
            rxBuffer.errorBitmap != 0) {
            memset(rxBuffer.errorFlags, 0, sizeof(rxBuffer.errorFlags));
            rxBuffer.errorBitmap = 0;
        }

        applyEncoderHealthTimeout(sharedData, encoderHealth, nowMs);

        if (p4::safety::communicationStreamStale(
                rxBuffer.isValid, nowMs, rxBuffer.timestamp, 500)) {
            rxBuffer.isValid = false;
            clearEncoderHealth(sharedData, encoderHealth);
            sharedData->canRxQueue.overwrite(rxBuffer);
            resetEncoderReassembly(&encoderState);
            FW_LOG_INFO_PRINTLN("[CAN HOTPLUG] encoder stream offline");
        }

        if ((uint32_t)(nowMs - lastTwaiHealthCheckMs) >= 100) {
            lastTwaiHealthCheckMs = nowMs;
            const CanControllerStatus statusInfo = canDriver.status();
            if (statusInfo.valid) {
                if (statusInfo.busOff && !twaiRecoveryRequested) {
                    clearEncoderHealth(sharedData, encoderHealth);
                    rxBuffer.isValid = false;
                    sharedData->canRxQueue.overwrite(rxBuffer);
                    if (canDriver.requestRecovery()) {
                        twaiRecoveryRequested = true;
                        FW_LOG_INFO_PRINTLN("[CAN HOTPLUG] bus-off recovery started");
                    }
                } else if (twaiRecoveryRequested && statusInfo.stopped) {
                    if (canDriver.restart()) {
                        twaiRecoveryRequested = false;
                        resetEncoderReassembly(&encoderState);
                        FW_LOG_INFO_PRINTLN("[CAN HOTPLUG] controller restarted after recovery");
                    }
                }
            }
        }

    return true;
}
