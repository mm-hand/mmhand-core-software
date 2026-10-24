/**
 * @file CalibrationMath.h
 * @brief 声明标定坐标转换和 Manual/URDF 线性角度映射纯函数。
 */
#ifndef CALIBRATION_MATH_H
#define CALIBRATION_MATH_H

#include "CalibrationRuntimeStore.h"

// 14 位磁编码器一圈的计数和半圈阈值。
static const int32_t kCalibrationEncoderModulo = 16384;
static const int32_t kCalibrationEncoderHalfTurn = kCalibrationEncoderModulo / 2;
// Rope Init/READY 允许落在 Manual Min 外侧的最大容差。
static const int32_t kFeedbackReadyEnterOutsideToleranceCounts = 24;

// S3 只发映射后的 raw（正方向递增）。P4 直接使用 raw，不做方向/取反。
// 将任意整数折回 0..16383。
int32_t calibrationWrapEncoderCount(int32_t value);
// 计算环形坐标中 from 到 to 的正向距离，结果为 0..16383。
int32_t calibrationForwardEncoderDelta(int32_t from, int32_t to);
// 计算 target-actual 的最短有符号环形差，结果位于 [-8192,8192]。
int32_t calibrationWrapEncoderDelta(int32_t target, int32_t actual);
// 按 16384 counts/360° 在 counts 与角度之间换算，仅用于区间外诊断。
float calibrationEncoderCountToDeg(int32_t count);
// 角度换算为磁编 counts，使用同一诊断比例并四舍五入。
int32_t calibrationDegToEncoderCount(float deg);

// 单次磁编到 URDF 角度映射结果。
// outsideCounts 必须保留符号：负数表示低于 Manual Min，正数表示高于 Manual Max。
struct EncoderUrdfMapResult {
    bool valid;                    // 输入和标定契约是否有效。
    bool feedbackReady;            // 是否满足 READY 下边界要求。
    int32_t outsideCounts;         // 区间外有符号距离。
    int32_t belowMinCounts;        // 低于 Manual Min 的距离，否则为 0。
    int32_t aboveMaxCounts;        // 高于 Manual Max 的距离，否则为 0。
    float urdfDeg;                 // 区间内线性映射、区间外诊断外推角度。
};

// 生成当前 V4 映射契约：Manual Min/Max 对应 URDF Min/Max，Auto 仅供诊断。
bool applyManualScaledUrdfMappingToCalibration(
    uint8_t jointIndex,
    const ManualEncoderReference& manual,
    uint16_t autoMinRaw,
    uint16_t autoMaxRaw,
    uint32_t manualEntryCrc,
    JointCalibrationResult* result);

// 使用 V4 标定把实时 raw 映射到角度。超过 Manual Max 仍可显示并告警；
// 只有低于 Manual Min 才会令 feedbackReady=false。
bool mapManualScaledEncoderToUrdf(uint8_t jointIndex,
                                  const JointCalibrationResult& calibration,
                                  const ManualEncoderReference& manual,
                                  uint16_t currentRaw,
                                  EncoderUrdfMapResult* out);

#endif // CALIBRATION_MATH_H
