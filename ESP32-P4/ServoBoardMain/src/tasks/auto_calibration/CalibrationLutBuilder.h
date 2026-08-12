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

#include "../../modules/calibration/CalibrationRuntimeStore.h"

namespace mmhand::calibration {

inline constexpr uint8_t kTendonRawSampleCapacity =
    TENDON_LUT_MAX_SAMPLES * 3; ///< 原始采样密度高于最终固定角步长 LUT。

/** 一帧标定原始样本；encoder 已转换为机械正方向坐标。 */
struct TendonRawFitSample {
    int32_t encoder;      ///< 机械正方向的连续磁编坐标。
    int32_t primaryAbs;   ///< 主电机 P4 logical ABS。
    int32_t secondaryAbs; ///< 副电机 P4 logical ABS；单腱关节填 0。
};

/** 固定容量原始样本缓存，避免实时任务进行堆内存分配。 */
struct TendonRawFitBuffer {
    uint8_t count;          ///< 当前有效原始样本数。
    float nextTravelDeg;    ///< 下一次按角行程采样的门槛。
    TendonRawFitSample samples[kTendonRawSampleCapacity]; ///< 固定容量，实时任务不分配堆内存。
};

/** 四舍五入到 int32，统一标定拟合和协议字段的取整规则。 */
int32_t roundToInt32(float value);
/** 返回两个连续磁编坐标的有符号差，不执行 14 位回绕。 */
int32_t signedEncoderDelta(int32_t fromEncoder, int32_t toEncoder);
/** 按本轮实测方向把 encoder 转成“行程增加为正”的拟合坐标。 */
int32_t encoderForPositiveTravel(int32_t encoder, int8_t travelSign);

/** 清空固定容量原始样本及下一采样门槛。 */
void resetTendonRawFitBuffer(TendonRawFitBuffer* buffer);
/** 追加一帧原始样本；容量满或输入无效时返回 false。 */
bool appendTendonRawFitSample(TendonRawFitBuffer* buffer,
                              int32_t encoder,
                              int32_t primaryAbs,
                              int32_t secondaryAbs);
/** 仅在 travelDeg 跨过下一固定门槛时追加样本，限制数据量和采样偏置。 */
void maybeAppendTendonTravelFitSample(TendonRawFitBuffer* buffer,
                                     float travelDeg,
                                     int32_t encoder,
                                     int32_t primaryAbs,
                                     int32_t secondaryAbs);

/** 将原始轨迹重采样为固定角步长 Tendon LUT，并写入 result。 */
bool buildTendonLutFromRawSamples(JointCalibrationResult* result,
                                  uint8_t primaryMotor,
                                  uint8_t secondaryMotor,
                                  uint8_t actuatorCount,
                                  const TendonRawFitBuffer& raw);

/** 构建单腱关节 V3 Auto 端点、动态角范围和单路 Tendon LUT。 */
bool buildAutoZeroTightCalibrationResult(uint8_t jointIndex,
                                         int32_t tautEncoder,
                                         int32_t zeroEncoder,
                                         int32_t tightEncoder,
                                         int32_t tautServoAbs,
                                         int32_t zeroServoAbs,
                                         int32_t tightServoAbs,
                                         JointCalibrationResult* out);

/** 构建双腱关节 V3 Auto 端点；双路 LUT 由标定状态机随后写入。 */
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
