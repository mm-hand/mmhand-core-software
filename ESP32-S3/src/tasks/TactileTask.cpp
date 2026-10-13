#include "TactileTask.h"
#include "../config/Config.h"
#include "../hal/Crc16.h"
#include "../hal/HalTactile.h"
#include "../hal/HalTWAI.h"
#include "../system/FirmwareLog.h"

#include <cstring>

TaskHandle_t xTacTask = NULL;

namespace {
// 约30Hz采样；实际周期受15个模块共30次寄存器读取耗时影响。
constexpr uint32_t kTactileTaskPeriodMs = 33;
// [待确认是否调大] 当前每一轮都发送全量1530字节触觉数据。
constexpr uint8_t kFullDumpDecimation = 1;
// 编译期关闭串口/CAN原始统计打印。
constexpr bool kEnableTactileDebugPrint = false;
// S3本地Serial二进制协议常量；与P4上位机协议编号保持一致。
constexpr uint8_t kProtocolHeader = 0xFE;
constexpr uint8_t kProtocolTail = 0xFF;
constexpr uint8_t kPacketTypeTactile = 0x0A;
constexpr uint8_t kPacketTypeTactileFull = 0x0E;
constexpr uint8_t kPacketTypeTactileDiag = 0x10;
constexpr uint8_t kTactileDiagVersion = 1;
constexpr uint8_t kTactileDiagDecimation = 30;
constexpr uint8_t kTactileFullMagic0 = 'T';
constexpr uint8_t kTactileFullMagic1 = '2';
// 256字节串口缓冲中为帧头、元数据和尾字节预留空间。
constexpr size_t kTactileFullSerialChunkBytes = 240;

/** @brief 取三轴绝对幅值最大值；X/Y按有符号值解释，Z按无符号压力解释。 */
uint8_t axisMagnitude(const uint8_t* axes) {
    const int x = abs((int8_t)axes[0]);
    const int y = abs((int8_t)axes[1]);
    const int z = axes[2];
    int value = x;
    if (y > value) value = y;
    if (z > value) value = z;
    return (uint8_t)value;
}

/** @brief 调试时统计一块触觉模块Z轴范围、绝对和及显著非零触元数。 */
void printForceZStats(const char* label, const uint8_t* forces, uint16_t forceLen) {
    int minZ = 127;
    int maxZ = -128;
    uint32_t absSum = 0;
    uint16_t nonZero = 0;
    for (uint16_t idx = 2; idx < forceLen; idx += TACTILE_AXIS_COUNT) {
        const int z = (int8_t)forces[idx];
        if (z < minZ) minZ = z;
        if (z > maxZ) maxZ = z;
        const int mag = abs(z);
        absSum += (uint32_t)mag;
        if (mag > 3) nonZero++;
    }
    if (absSum > 50 || nonZero > 4) {
        Serial.printf(" %s:min=%d max=%d sum=%lu nz=%u",
                      label,
                      minZ,
                      maxZ,
                      (unsigned long)absSum,
                      nonZero);
    }
}

/** @brief 应用摘要噪声门限；低于门限时三轴统一清零。 */
void writeFilteredSummaryAxes(uint8_t* dst, const uint8_t* src) {
    if (!dst || !src) return;
    // [待确认是否删除滤波分支] 当前门限为0，除全零外不会过滤任何数据。
    if (axisMagnitude(src) <= TACTILE_SUMMARY_NOISE_FLOOR) {
        dst[0] = 0;
        dst[1] = 0;
        dst[2] = 0;
        return;
    }
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
}

/** @brief 按“5组×3模块×3轴”顺序展平45字节触觉摘要。 */
void flattenTactileSummary(const TactileData& data, uint8_t* out, size_t outLen) {
    if (!out || outLen < TACTILE_SUMMARY_BYTES) return;
    uint8_t cursor = 0;
    for (uint8_t g = 0; g < TACTILE_GROUP_NUM; g++) {
        const TacGroup& grp = data.groups[g];
        const uint8_t* ptrs[TACTILE_SENSOR_PER_GROUP] = {
            grp.sensor_A.global,
            grp.sensor_B.global,
            grp.sensor_C.global,
        };
        for (uint8_t s = 0; s < TACTILE_SENSOR_PER_GROUP; s++) {
            writeFilteredSummaryAxes(out + cursor, ptrs[s]);
            cursor += TACTILE_AXIS_COUNT;
        }
    }
}

/** @brief 按组和模块顺序展平1530字节全触元数据。 */
void flattenTactileFull(const TactileData& data, uint8_t* out, size_t outLen) {
    if (!out || outLen < TACTILE_FULL_BYTES) return;
    size_t cursor = 0;
    // 局部lambda统一处理边界，防止未来结构尺寸变化造成越界。
    auto pushBlock = [&](const uint8_t* bytes, size_t len) {
        if (!bytes || cursor >= outLen) return;
        const size_t n = (cursor + len <= outLen) ? len : (outLen - cursor);
        memcpy(out + cursor, bytes, n);
        cursor += n;
    };

    for (uint8_t g = 0; g < TACTILE_GROUP_NUM; g++) {
        const TacGroup& grp = data.groups[g];
        pushBlock(&grp.sensor_A.forces[0][0], TACTILE_SENSOR0_TAXELS * TACTILE_AXIS_COUNT);
        pushBlock(&grp.sensor_B.forces[0][0], TACTILE_SENSOR1_TAXELS * TACTILE_AXIS_COUNT);
        pushBlock(&grp.sensor_C.forces[0][0], TACTILE_SENSOR2_TAXELS * TACTILE_AXIS_COUNT);
    }
}

/** @brief 通过S3调试串口发送摘要、序号和15位模块有效位图。 */
void sendSerialTactileSummary(const TactileData& data, uint8_t seq, uint16_t moduleValidMask) {
    uint8_t payload[TACTILE_SUMMARY_BYTES] = {0};
    flattenTactileSummary(data, payload, sizeof(payload));

    uint8_t buffer[2 + 1 + 1 + TACTILE_SUMMARY_BYTES + 2 + 1] = {0};
    size_t idx = 0;
    buffer[idx++] = kProtocolHeader;
    buffer[idx++] = 0;
    buffer[idx++] = kPacketTypeTactile;
    buffer[idx++] = seq;
    memcpy(buffer + idx, payload, sizeof(payload));
    idx += sizeof(payload);
    // 与CAN/P4摘要保持同一布局，避免直连S3和P4中继出现两套解析规则。
    buffer[idx++] = (uint8_t)(moduleValidMask >> 8);
    buffer[idx++] = (uint8_t)(moduleValidMask & 0xFF);
    buffer[idx++] = kProtocolTail;
    buffer[1] = (uint8_t)(idx - 2);
    Serial.write(buffer, idx);
}

/**
 * @brief 将1530字节全量触觉数据按240字节分片，通过S3调试串口发送。
 * @note [待确认是否保留] 当前无独立开关且每33ms发送一次，会与文本Serial日志混在同一串口。
 */
void sendSerialTactileFullDump(const TactileData& data, uint8_t seq) {
    uint8_t raw[TACTILE_FULL_BYTES] = {0};
    flattenTactileFull(data, raw, sizeof(raw));

    const uint8_t fragCount = (uint8_t)((TACTILE_FULL_BYTES + kTactileFullSerialChunkBytes - 1) /
                                        kTactileFullSerialChunkBytes);
    const uint16_t fullCrc = crc16Ccitt(raw, sizeof(raw));
    for (uint8_t fragIdx = 0; fragIdx < fragCount; fragIdx++) {
        const size_t offset = (size_t)fragIdx * kTactileFullSerialChunkBytes;
        const size_t remaining = TACTILE_FULL_BYTES - offset;
        const uint8_t chunkLen = (uint8_t)((remaining > kTactileFullSerialChunkBytes)
            ? kTactileFullSerialChunkBytes
            : remaining);

        uint8_t buffer[256] = {0};
        size_t idx = 0;
        buffer[idx++] = kProtocolHeader;
        buffer[idx++] = 0;
        buffer[idx++] = kPacketTypeTactileFull;
        buffer[idx++] = kTactileFullMagic0;
        buffer[idx++] = kTactileFullMagic1;
        // 协议预留“全量序号”和“摘要序号”两个字段；当前二者共用同一seq。
        buffer[idx++] = seq;
        buffer[idx++] = seq;
        buffer[idx++] = fragIdx;
        buffer[idx++] = fragCount;
        buffer[idx++] = chunkLen;
        buffer[idx++] = (uint8_t)(fullCrc & 0xFF);
        buffer[idx++] = (uint8_t)((fullCrc >> 8) & 0xFF);
        memcpy(buffer + idx, raw + offset, chunkLen);
        idx += chunkLen;
        buffer[idx++] = kProtocolTail;
        buffer[1] = (uint8_t)(idx - 2);
        Serial.write(buffer, idx);
    }
}

/** @brief 低频发送分模块SPI诊断；新包号保证旧上位机可直接忽略。 */
void sendSerialTactileDiagnostics(const TactileReadDiag& diag) {
    constexpr size_t kMaskCount = 8;
    constexpr size_t kCounterArrayCount = 4;
    constexpr size_t kReasonArrayCount = 3;
    constexpr size_t kPayloadBytes = 1 + 4 + kMaskCount * 2 +
        kCounterArrayCount * TACTILE_MODULE_NUM * 2 +
        kReasonArrayCount * TACTILE_MODULE_NUM;
    uint8_t buffer[2 + 1 + kPayloadBytes + 1] = {0};
    size_t idx = 0;
    buffer[idx++] = kProtocolHeader;
    buffer[idx++] = 0;
    buffer[idx++] = kPacketTypeTactileDiag;
    buffer[idx++] = kTactileDiagVersion;

    auto pushU16 = [&](uint16_t value) {
        buffer[idx++] = (uint8_t)(value & 0xFF);
        buffer[idx++] = (uint8_t)(value >> 8);
    };
    auto pushU32 = [&](uint32_t value) {
        pushU16((uint16_t)(value & 0xFFFF));
        pushU16((uint16_t)(value >> 16));
    };
    auto pushCounters = [&](const uint16_t* values) {
        for (uint8_t module = 0; module < TACTILE_MODULE_NUM; module++) {
            pushU16(values[module]);
        }
    };

    pushU32(diag.frameCount);
    pushU16(diag.enabledMask);
    pushU16(diag.summaryOkMask);
    pushU16(diag.fullOkMask);
    pushU16(diag.commandFailMask);
    pushU16(diag.responseFailMask);
    pushU16(diag.crcFailMask);
    pushU16(diag.statusNonzeroMask);
    pushU16(diag.summaryRetryRecoveredMask);
    pushCounters(diag.summarySuccessCount);
    pushCounters(diag.summaryFailureCount);
    pushCounters(diag.fullSuccessCount);
    pushCounters(diag.fullFailureCount);
    memcpy(buffer + idx, diag.summaryLastFailReason, TACTILE_MODULE_NUM);
    idx += TACTILE_MODULE_NUM;
    memcpy(buffer + idx, diag.fullLastFailReason, TACTILE_MODULE_NUM);
    idx += TACTILE_MODULE_NUM;
    memcpy(buffer + idx, diag.moduleLastStatus, TACTILE_MODULE_NUM);
    idx += TACTILE_MODULE_NUM;
    buffer[idx++] = kProtocolTail;
    buffer[1] = (uint8_t)(idx - 2);
    Serial.write(buffer, idx);
}
} // namespace

/**
 * @brief 周期采集触觉数据，并复用同一批次序号发送CAN摘要和S3串口数据。
 * @param pvParameters FreeRTOS预留任务参数，当前未使用。
 */
void Task_Tactile(void* pvParameters) {
    (void)pvParameters;
    const TickType_t xFrequency = pdMS_TO_TICKS(kTactileTaskPeriodMs);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    Serial.println("[Task_Tactile] Started!");

    for (;;) {
        // HAL先完成整轮采样，再取出一致快照供多种上报链路复用。
        tactile.update();
        const TactileData data = tactile.getData();
        const TactileReadDiag diag = tactile.getDiagnostics();

#if ENABLE_TACTILE_CAN_RELAY
        // 只把摘要读取成功位图送往P4；全量读取失败不能误判摘要模块离线。
        const uint8_t tactileSeq = twaiBus.sendTactileSummary(data, diag.summaryOkMask);
#else
        static uint8_t tactileSeqCounter = 0;
        const uint8_t tactileSeq = tactileSeqCounter++;
#endif

        // [待确认是否保留] 即使P4通过CAN转发，S3仍会额外从Serial直接发送一份摘要。
        sendSerialTactileSummary(data, tactileSeq, diag.summaryOkMask);

        static uint8_t fullDumpDecimator = 0;
        fullDumpDecimator++;
        if (fullDumpDecimator >= kFullDumpDecimation) {
            fullDumpDecimator = 0;
#if ENABLE_TACTILE_CAN_RELAY && ENABLE_TACTILE_CAN_FULL_DUMP
            // 默认关闭：完整触觉数据需要219个标准CAN帧，可能长期占满1Mbit/s总线。
            twaiBus.sendTactileFullDump(data, tactileSeq);
#endif
            // S3串口全量上报不受ENABLE_TACTILE_CAN_FULL_DUMP控制。
            sendSerialTactileFullDump(data, tactileSeq);
        }

        static uint8_t diagDecimator = 0;
        if (++diagDecimator >= kTactileDiagDecimation) {
            diagDecimator = 0;
            sendSerialTactileDiagnostics(diag);
        }

        static uint32_t lastPrint = 0;
        // TRACE调试同时打印本地统计并可经CAN发送稀疏原始诊断。
        if (kEnableTactileDebugPrint &&
            FW_LOG_ENABLED(FW_LOG_LEVEL_TRACE) &&
            millis() - lastPrint > 1000) {
            lastPrint = millis();
            FW_LOG_DEBUG_PRINTF(
                "[Tactile] frame=%lu enabled=%04X ok=%u fail=%u summary=%04X full=%04X both=%04X miss=%04X st=%02X crc=%02X/%02X\n",
                (unsigned long)diag.frameCount,
                diag.enabledMask,
                diag.okReads,
                diag.failedReads,
                diag.summaryOkMask,
                diag.fullOkMask,
                diag.okMask,
                (uint16_t)(diag.enabledMask & ~diag.okMask),
                diag.lastStatus,
                diag.lastRecvCrc,
                diag.lastCalcCrc
            );

            Serial.print("[TactileRawZ]");
            for (uint8_t g = 0; g < TACTILE_GROUP_NUM; g++) {
                const TacGroup& grp = data.groups[g];
                char label[8];
                snprintf(label, sizeof(label), "G%uS0", g);
                printForceZStats(label, &grp.sensor_A.forces[0][0], TACTILE_SENSOR0_TAXELS * TACTILE_AXIS_COUNT);
                snprintf(label, sizeof(label), "G%uS1", g);
                printForceZStats(label, &grp.sensor_B.forces[0][0], TACTILE_SENSOR1_TAXELS * TACTILE_AXIS_COUNT);
                snprintf(label, sizeof(label), "G%uS2", g);
                printForceZStats(label, &grp.sensor_C.forces[0][0], TACTILE_SENSOR2_TAXELS * TACTILE_AXIS_COUNT);
            }
            Serial.println();
#if ENABLE_TACTILE_CAN_RELAY
            twaiBus.sendTactileRawDiag(data);
#endif
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
