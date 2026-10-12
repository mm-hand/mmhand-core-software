/**
 * @file TendonCalibrationLut.h
 * @brief 定义 Tendon LUT 访问、插值和端部线性外推接口。
 */
#ifndef TENDON_CALIBRATION_LUT_H
#define TENDON_CALIBRATION_LUT_H

#include <Arduino.h>
#include "CalibrationRuntimeStore.h"

// 将 URDF 关节角转换到标定 LUT 的内部角度坐标，再对指定执行器的
// raw ABS counts 做线性插值；超出 LUT 角度端点时沿首末段斜率外推。
bool interpolateTendonCalibrationMotorAbs(const JointCalibrationResult& calibration,
                                          uint8_t actuatorIndex,
                                          float urdfDeg,
                                          float* outMotorAbs);

/** 返回指定角度是否位于 LUT 的实际采样横轴内，不允许端部外推。 */
bool tendonCalibrationContainsAngle(const JointCalibrationResult& calibration,
                                    float urdfDeg);

// 返回某一路执行器在 LUT 实测样本中的 raw ABS 最小值和最大值。
// 该范围用于生成单腱关节的标定后舵机工作区。
bool tendonCalibrationMotorAbsRange(const JointCalibrationResult& calibration,
                                    uint8_t actuatorIndex,
                                    int32_t* outMinAbs,
                                    int32_t* outMaxAbs);

/**
 * 按实际腱绳收放方向扩展 LUT 工作区。release/tight 裕量不会被 ABS 正负方向混淆。
 */
bool tendonCalibrationMotorAbsRangeWithMargins(
    const JointCalibrationResult& calibration,
    uint8_t actuatorIndex,
    uint8_t motorChannel,
    int32_t releaseMarginCounts,
    int32_t tightMarginCounts,
    int32_t* outMinAbs,
    int32_t* outMaxAbs);

/**
 * 计算单腱运行边界：松端由 LUT 包络向外保留裕量，紧端以自动标定
 * 实际找到的机械紧端为基准再保留裕量。单腱 LUT 采自松绳回程，不能
 * 把 URDF Max 处的回程样本误当成机械紧端，否则会提前钳住反馈控制。
 */
bool singleTendonCalibrationMotorAbsRange(
    const JointCalibrationResult& calibration,
    uint8_t actuatorIndex,
    uint8_t motorChannel,
    int32_t releaseMarginCounts,
    int32_t tightMarginCounts,
    int32_t* outMinAbs,
    int32_t* outMaxAbs);

#endif // TENDON_CALIBRATION_LUT_H
