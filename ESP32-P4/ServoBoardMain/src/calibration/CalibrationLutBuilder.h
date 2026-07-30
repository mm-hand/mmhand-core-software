/**
 * @file CalibrationLutBuilder.h
 * @brief 定义标定原始样本缓存和 Tendon LUT 构建接口。
 *
 * 自动标定状态机只负责采集磁编与舵机 ABS。本模块负责方向归一化、
 * 固定角步长重采样、Manual 比例映射及 V3 标定结果构造。
 */
#ifndef MMHAND_CALIBRATION_LUT_BUILDER_H
#define MMHAND_CALIBRATION_LUT_BUILDER_H

#include <stdint.h>

#include "CalibrationTask.h"

namespace mmhand::calibration {

inline constexpr uint8_t kTendonRawSampleCapacity =
    TENDON_LUT_MAX_SAMPLES * 3;

/** 一帧标定原始样本；encoder 已转换为机械正方向坐标。 */
struct TendonRawFitSample {
    int32_t encoder;
    int32_t primaryAbs;
    int32_t secondaryAbs;
};

/** 固定容量原始样本缓存，避免实时任务进行堆内存分配。 */
struct TendonRawFitBuffer {
    uint8_t count;
    float nextTravelDeg;
    TendonRawFitSample samples[kTendonRawSampleCapacity];
};

int32_t roundToInt32(float value);
int32_t signedEncoderDelta(int32_t fromEncoder, int32_t toEncoder);
int32_t encoderForPositiveTravel(int32_t encoder, int8_t travelSign);

void resetTendonRawFitBuffer(TendonRawFitBuffer* buffer);
bool appendTendonRawFitSample(TendonRawFitBuffer* buffer,
                              int32_t encoder,
                              int32_t primaryAbs,
                              int32_t secondaryAbs);
void maybeAppendTendonTravelFitSample(TendonRawFitBuffer* buffer,
                                     float travelDeg,
                                     int32_t encoder,
                                     int32_t primaryAbs,
                                     int32_t secondaryAbs);

bool buildTendonLutFromRawSamples(JointCalibrationResult* result,
                                  uint8_t primaryMotor,
                                  uint8_t secondaryMotor,
                                  uint8_t actuatorCount,
                                  const TendonRawFitBuffer& raw);

bool buildAutoZeroTightCalibrationResult(uint8_t jointIndex,
                                         int32_t tautEncoder,
                                         int32_t zeroEncoder,
                                         int32_t tightEncoder,
                                         int32_t tautServoAbs,
                                         int32_t zeroServoAbs,
                                         int32_t tightServoAbs,
                                         JointCalibrationResult* out);

bool buildDualZeroTightCalibrationResult(uint8_t jointIndex,
                                         int32_t startEncoder,
                                         int32_t zeroEncoder,
                                         int32_t tightEncoder,
                                         int32_t startServoAbs,
                                         int32_t zeroServoAbs,
                                         int32_t tightServoAbs,
                                         JointCalibrationResult* out);

} // namespace mmhand::calibration

#endif // MMHAND_CALIBRATION_LUT_BUILDER_H
