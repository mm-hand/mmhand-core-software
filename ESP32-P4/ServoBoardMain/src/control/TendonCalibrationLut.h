/**
 * @file TendonCalibrationLut.h
 * @brief 定义 Tendon LUT 访问接口和边界钳位结果。
 */
#ifndef TENDON_CALIBRATION_LUT_H
#define TENDON_CALIBRATION_LUT_H

#include <Arduino.h>
#include "../calibration/CalibrationTask.h"

// 将 URDF 关节角转换到标定 LUT 的内部角度坐标，再对指定执行器的
// raw ABS counts 做线性插值。超出 LUT 角度端点时钳到最近端点，不外推。
bool interpolateTendonCalibrationMotorAbs(const JointCalibrationResult& calibration,
                                          uint8_t actuatorIndex,
                                          float urdfDeg,
                                          float* outMotorAbs);

// 返回某一路执行器在 LUT 实测样本中的 raw ABS 最小值和最大值。
// 该范围用于生成单腱关节的标定后舵机工作区。
bool tendonCalibrationMotorAbsRange(const JointCalibrationResult& calibration,
                                    uint8_t actuatorIndex,
                                    int32_t* outMinAbs,
                                    int32_t* outMaxAbs);

#endif // TENDON_CALIBRATION_LUT_H
