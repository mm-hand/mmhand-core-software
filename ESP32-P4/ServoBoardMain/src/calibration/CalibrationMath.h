/**
 * @file CalibrationMath.h
 * @brief 声明标定坐标转换和动态角度映射纯函数。
 */
#ifndef CALIBRATION_MATH_H
#define CALIBRATION_MATH_H

#include "CalibrationTask.h"

// 14 位磁编码器一圈的计数和半圈阈值。
static const int32_t kCalibrationEncoderModulo = 16384;
static const int32_t kCalibrationEncoderHalfTurn = kCalibrationEncoderModulo / 2;
// Rope Init/READY 允许落在 Auto Min 外侧的最大容差。
static const int32_t kFeedbackReadyEnterOutsideToleranceCounts = 24;

// 按安装方向把 0..16383 原始值变成“正方向递增”的环形坐标。
int32_t calibrationOrientEncoderRaw(uint16_t rawValue, int8_t direction);
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
// outsideCounts 必须保留符号：负数表示低于 Auto Min，正数表示高于 Auto Max。
struct EncoderUrdfMapResult {
    bool valid;                    // 输入和标定契约是否有效。
    bool insideCalibratedWindow;   // 当前 raw 是否在 Auto Min/Max 内。
    bool feedbackReady;            // 是否满足 READY 下边界要求。
    int32_t signedTravelCounts;    // 相对 Auto Min 的有符号正向坐标。
    int32_t outsideCounts;         // 区间外有符号距离。
    int32_t belowMinCounts;        // 低于 Auto Min 的距离，否则为 0。
    int32_t aboveMaxCounts;        // 高于 Auto Max 的距离，否则为 0。
    float urdfDeg;                 // 区间内线性映射、区间外诊断外推角度。
};

// 沿安装正方向计算两个原始端点的距离；端点允许跨 0/16383。
int32_t calibrationDirectedEncoderDelta(uint16_t fromRaw,
                                        uint16_t toRaw,
                                        int8_t direction);

// 沿安装正方向计算最短有符号位移，正数为同向，负数为反向。
int32_t calibrationSignedDirectedEncoderDelta(uint16_t fromRaw,
                                              uint16_t toRaw,
                                              int8_t direction);

// 由 Manual/Auto 端点生成当前 V3 动态角度映射，并绑定单关节 Manual CRC。
bool applyManualScaledUrdfMappingToCalibration(
    uint8_t jointIndex,
    const ManualEncoderReference& manual,
    uint16_t autoMinRaw,
    uint16_t autoMaxRaw,
    int8_t direction,
    uint32_t manualEntryCrc,
    JointCalibrationResult* result);

// 使用 V3 标定把实时 raw 映射到角度。超过 Auto Max 仍可显示并告警；
// 只有低于 Auto Min 才会令 feedbackReady=false。
bool mapManualScaledEncoderToUrdf(uint8_t jointIndex,
                                  const JointCalibrationResult& calibration,
                                  const ManualEncoderReference& manual,
                                  uint16_t currentRaw,
                                  int8_t direction,
                                  EncoderUrdfMapResult* out);

// V2 快照兼容映射：把方向坐标中的实测端点线性映射到完整 URDF 区间。
// 区间外角度只用于诊断，当前 V3 标定流程不会生成这种结果。
bool mapEncoderWindowToUrdf(int32_t orientedRaw,
                            int32_t encoderAtUrdfMin,
                            int32_t encoderAtUrdfMax,
                            float urdfMinDeg,
                            float urdfMaxDeg,
                            EncoderUrdfMapResult* out);

// 校验 V2 模型 CRC 后执行兼容映射；ControlTask 仍用它读取历史快照。
bool mapJointCalibrationEncoderToUrdf(uint8_t jointIndex,
                                      const JointCalibrationResult& calibration,
                                      int32_t orientedRaw,
                                      EncoderUrdfMapResult* out);

#endif // CALIBRATION_MATH_H
