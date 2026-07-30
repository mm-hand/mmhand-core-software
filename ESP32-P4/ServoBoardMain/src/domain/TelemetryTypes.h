/**
 * @file TelemetryTypes.h
 * @brief 定义经 CAN 接收并向上位机转发的触觉数据快照。
 */
#ifndef MMHAND_TELEMETRY_TYPES_H
#define MMHAND_TELEMETRY_TYPES_H

#include "../config/DimensionsConfig.h"
#include <stdint.h>

struct RemoteTactileData_t {
    uint8_t values[TACTILE_SUMMARY_BYTES];
    uint8_t fullValues[TACTILE_FULL_BYTES];
    uint8_t seq;
    uint8_t fullSeq;
    uint32_t timestamp;
    uint32_t fullTimestamp;
    bool isValid;
    bool fullValid;
};

#endif // MMHAND_TELEMETRY_TYPES_H
