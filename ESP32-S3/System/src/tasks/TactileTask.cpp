#include "TactileTask.h"
#include "../config/Config.h"
#include "../hal/HalTactile.h"
#include "../hal/HalTWAI.h"

#include <cstring>

// 任务句柄
TaskHandle_t xTacTask = NULL;

namespace {
constexpr uint32_t kTactileTaskPeriodMs = 33;
constexpr uint8_t kFullDumpDecimation = 1; // Send the full matrix every tactile cycle.
constexpr bool kEnableTactileDebugPrint = false;
constexpr uint8_t kProtocolHeader = 0xFE;
constexpr uint8_t kProtocolTail = 0xFF;
constexpr uint8_t kPacketTypeTactile = 0x0A;
constexpr uint8_t kPacketTypeTactileFull = 0x0E;
constexpr uint8_t kTactileFullMagic0 = 'T';
constexpr uint8_t kTactileFullMagic1 = '2';
constexpr size_t kTactileFullSerialChunkBytes = 240;

uint8_t axisMagnitude(const uint8_t* axes) {
    const int x = abs((int8_t)axes[0]);
    const int y = abs((int8_t)axes[1]);
    const int z = axes[2];
    int value = x;
    if (y > value) {
        value = y;
    }
    if (z > value) {
        value = z;
    }
    return (uint8_t)value;
}

void printVec(const uint8_t* axes) {
    Serial.printf("%+04d,%+04d,%03u", (int8_t)axes[0], (int8_t)axes[1], axes[2]);
}

void printForceZStats(const char* label, const uint8_t* forces, uint16_t forceLen) {
    int minZ = 127;
    int maxZ = -128;
    uint32_t absSum = 0;
    uint16_t nonZero = 0;
    for (uint16_t idx = 2; idx < forceLen; idx += TACTILE_AXIS_COUNT) {
        const int z = (int8_t)forces[idx];
        if (z < minZ) {
            minZ = z;
        }
        if (z > maxZ) {
            maxZ = z;
        }
        const int mag = abs(z);
        absSum += (uint32_t)mag;
        if (mag > 3) {
            nonZero++;
        }
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

void writeFilteredSummaryAxes(uint8_t* dst, const uint8_t* src) {
    if (!dst || !src) {
        return;
    }
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

uint16_t crc16Ccitt(const uint8_t* data, size_t len) {
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

void flattenTactileSummary(const TactileData& data, uint8_t* out, size_t outLen) {
    if (!out || outLen < TACTILE_SUMMARY_BYTES) {
        return;
    }
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

void flattenTactileFull(const TactileData& data, uint8_t* out, size_t outLen) {
    if (!out || outLen < TACTILE_FULL_BYTES) {
        return;
    }
    size_t cursor = 0;
    auto pushBlock = [&](const uint8_t* bytes, size_t len) {
        if (!bytes || cursor >= outLen) {
            return;
        }
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

void sendSerialTactileSummary(const TactileData& data, uint8_t seq) {
    uint8_t payload[TACTILE_SUMMARY_BYTES] = {0};
    flattenTactileSummary(data, payload, sizeof(payload));

    uint8_t buffer[2 + 1 + 1 + TACTILE_SUMMARY_BYTES + 1] = {0};
    size_t idx = 0;
    buffer[idx++] = kProtocolHeader;
    buffer[idx++] = 0;
    buffer[idx++] = kPacketTypeTactile;
    buffer[idx++] = seq;
    memcpy(buffer + idx, payload, sizeof(payload));
    idx += sizeof(payload);
    buffer[idx++] = kProtocolTail;
    buffer[1] = (uint8_t)(idx - 2);
    Serial.write(buffer, idx);
}

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
} // namespace

/**
 * 触觉任务 (10Hz)
 * 负责处理触觉传感器数据
 */
void Task_Tactile(void* pvParameters) {
    (void)pvParameters;
    const TickType_t xFrequency = pdMS_TO_TICKS(kTactileTaskPeriodMs);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    Serial.println("[Task_Tactile] Started!");

    for (;;) {
        tactile.update();
        const TactileData data = tactile.getData();
#if ENABLE_TACTILE_CAN_RELAY
        const uint8_t tactileSeq = twaiBus.sendTactileSummary(data);
#else
        static uint8_t tactileSeqCounter = 0;
        const uint8_t tactileSeq = tactileSeqCounter++;
#endif
        sendSerialTactileSummary(data, tactileSeq);

        static uint8_t fullDumpDecimator = 0;
        fullDumpDecimator++;
        if (fullDumpDecimator >= kFullDumpDecimation) {
            fullDumpDecimator = 0;
#if ENABLE_TACTILE_CAN_RELAY && ENABLE_TACTILE_CAN_FULL_DUMP
            twaiBus.sendTactileFullDump(data, tactileSeq);
#endif
            sendSerialTactileFullDump(data, tactileSeq);
        }

        static uint32_t lastPrint = 0;
        if (kEnableTactileDebugPrint && millis() - lastPrint > 1000) {
            lastPrint = millis();
            const TactileReadDiag diag = tactile.getDiagnostics();
            Serial.printf(
                "[Tactile] frame=%lu enabled=%04X ok=%u fail=%u mask=%04X miss=%04X st=%02X crc=%02X/%02X\n",
                (unsigned long)diag.frameCount,
                diag.enabledMask,
                diag.okReads,
                diag.failedReads,
                diag.okMask,
                (uint16_t)(diag.enabledMask & ~diag.okMask),
                diag.lastStatus,
                diag.lastRecvCrc,
                diag.lastCalcCrc
            );
            Serial.print("[TactileFz]");
            for (uint8_t g = 0; g < TACTILE_GROUP_NUM; g++) {
                const TacGroup& grp = data.groups[g];
                Serial.printf(
                    " G%u:%03u,%03u,%03u",
                    g,
                    grp.sensor_A.global[2],
                    grp.sensor_B.global[2],
                    grp.sensor_C.global[2]
                );
            }
            Serial.println();

            Serial.print("[TactileMag]");
            for (uint8_t g = 0; g < TACTILE_GROUP_NUM; g++) {
                const TacGroup& grp = data.groups[g];
                Serial.printf(
                    " G%u:%03u,%03u,%03u",
                    g,
                    axisMagnitude(grp.sensor_A.global),
                    axisMagnitude(grp.sensor_B.global),
                    axisMagnitude(grp.sensor_C.global)
                );
            }
            Serial.println();

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
