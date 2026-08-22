/**
 * @file ManualEncoderReferenceStore.cpp
 * @brief 实现 Manual Min/Max 的 RAM 校验、CRC 和活动范围门禁。
 * @details 本文件故意不包含 Flash、Preferences 或任何持久化 API。
 */
#include "ManualEncoderReferenceStore.h"

#include <string.h>

#include "CalibrationMath.h"
#include "../../platform/rtos/SharedSafetyState.h"

bool manualEncoderReferencesValidate(
    const ManualEncoderReference* references,
    uint32_t* outValidMask)
{
    uint32_t validMask = 0;
    if (references) {
        for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; ++joint) {
            const ManualEncoderReference& reference = references[joint];
            if (reference.valid && reference.minRaw < kCalibrationEncoderModulo &&
                reference.maxRaw < kCalibrationEncoderModulo) {
                validMask |= 1UL << joint;
            }
        }
    }
    if (outValidMask) {
        *outValidMask = validMask;
    }
    return validMask == kManualEncoderReferenceFullMask;
}

bool manualEncoderReferencesCoverMask(
    const ManualEncoderReference* references,
    uint32_t requiredMask,
    uint32_t* outValidMask)
{
    uint32_t validMask = 0;
    manualEncoderReferencesValidate(references, &validMask);
    requiredMask &= kManualEncoderReferenceFullMask;
    if (outValidMask) {
        *outValidMask = validMask;
    }
    return requiredMask != 0 && (validMask & requiredMask) == requiredMask;
}

uint32_t manualEncoderReferencesCrc(const ManualEncoderReference* references)
{
    if (!references) {
        return 0;
    }
    uint32_t crc = 0;
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; ++joint) {
        const uint8_t values[5] = {
            joint,
            static_cast<uint8_t>(references[joint].minRaw >> 8),
            static_cast<uint8_t>(references[joint].minRaw & 0xFF),
            static_cast<uint8_t>(references[joint].maxRaw >> 8),
            static_cast<uint8_t>(references[joint].maxRaw & 0xFF),
        };
        for (uint8_t value : values) {
            crc = (crc << 5) - crc + value;
        }
    }
    return crc;
}

uint32_t manualEncoderReferenceEntryCrc(
    uint8_t jointIndex,
    const ManualEncoderReference& reference)
{
    const ManualEncoderReference entries[1] = {reference};
    const uint8_t values[5] = {
        jointIndex,
        static_cast<uint8_t>(entries[0].minRaw >> 8),
        static_cast<uint8_t>(entries[0].minRaw & 0xFF),
        static_cast<uint8_t>(entries[0].maxRaw >> 8),
        static_cast<uint8_t>(entries[0].maxRaw & 0xFF),
    };
    uint32_t crc = 0;
    for (uint8_t value : values) {
        crc = (crc << 5) - crc + value;
    }
    return crc;
}

void manualEncoderReferencesClearRam()
{
    SharedSafetyGuard guard;
    memset(g_manualEncoderReference, 0, sizeof(g_manualEncoderReference));
}

void manualEncoderReferencesApplyRam(const ManualEncoderReference* references)
{
    if (!references) {
        return;
    }
    SharedSafetyGuard guard;
    memcpy(g_manualEncoderReference, references, sizeof(g_manualEncoderReference));
}

uint32_t manualEncoderReferencesValidMask()
{
    uint32_t validMask = 0;
    manualEncoderReferencesValidate(g_manualEncoderReference, &validMask);
    return validMask;
}

bool manualEncoderReferencesCoverActiveMask(uint32_t requiredMask)
{
    requiredMask &= kManualEncoderReferenceFullMask;
    if (requiredMask == 0) {
        return false;
    }
    uint32_t usableMask = 0;
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; ++joint) {
        const ManualEncoderReference& reference = g_manualEncoderReference[joint];
        if (!reference.valid || reference.minRaw >= kCalibrationEncoderModulo ||
            reference.maxRaw >= kCalibrationEncoderModulo) {
            continue;
        }
        const int32_t span = calibrationDirectedEncoderDelta(
            reference.minRaw, reference.maxRaw, g_encoderDirection[joint]);
        if (span > 0 && span < kCalibrationEncoderModulo) {
            usableMask |= 1UL << joint;
        }
    }
    return (usableMask & requiredMask) == requiredMask;
}
