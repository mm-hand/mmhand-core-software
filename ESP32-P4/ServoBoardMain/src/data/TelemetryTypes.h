/**
 * @file TelemetryTypes.h
 * @brief 定义经 CAN 接收并向上位机转发的触觉数据快照。
 */
#ifndef MMHAND_TELEMETRY_TYPES_H
#define MMHAND_TELEMETRY_TYPES_H

#include "../config/DimensionsConfig.h"
#include <stdint.h>

struct RemoteTactileData_t {
    uint8_t values[TACTILE_SUMMARY_BYTES];   ///< 五指三模组三轴触觉汇总。
    uint8_t fullValues[TACTILE_FULL_BYTES];  ///< 全部 taxel 的三轴原始数据。
    uint8_t seq;                             ///< 汇总帧序号，用于检测丢帧。
    uint8_t fullSeq;                         ///< 全量帧序号。
    uint16_t moduleValidMask;                ///< bit0~bit14表示15块模块摘要读取有效。
    uint32_t timestamp;                      ///< 汇总帧完成接收时间。
    uint32_t fullTimestamp;                  ///< 全量帧完成接收时间。
    bool isValid;                            ///< 汇总帧至少完整接收一次。
    bool fullValid;                          ///< 全量帧至少完整接收一次。
};

#endif // MMHAND_TELEMETRY_TYPES_H
