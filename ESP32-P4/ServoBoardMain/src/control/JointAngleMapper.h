/**
 * @file JointAngleMapper.h
 * @brief 提供磁编码器 raw、自动标定坐标与 URDF 角度之间的纯数学映射。
 *
 * 该模块不读取任务状态、不发送舵机命令。调用方必须显式传入标定结果，
 * 因而可以单独测试跨圈、方向翻转和工作区外角度外推。
 */
#ifndef P4_JOINT_ANGLE_MAPPER_H
#define P4_JOINT_ANGLE_MAPPER_H

#include <stdint.h>

#include "../calibration/CalibrationTask.h"

namespace p4::control {

/** 按硬件方向表把 0..16383 raw 统一到“关节正方向增加”的坐标。 */
int32_t orientEncoderRaw(uint16_t rawValue, int8_t direction);

/** 将任意整数折回 14 bit 磁编码器的 0..16383 区间。 */
int32_t wrapEncoderCount(int32_t value);

/** 计算环形编码器从 from 沿正方向到 to 的距离。 */
int32_t forwardEncoderDelta(int32_t from, int32_t to);

/** 按 16384 counts = 360 degree 把带符号编码器增量换算为角度。 */
float encoderCountToDeg(int32_t encoderCount);
/** 将角度换算为最近的 14 bit 编码器 counts，结果允许带符号。 */
int32_t degToEncoderCount(float deg);

/**
 * 根据本次自动标定结果计算实时关节角。
 *
 * 标定区间内使用实测 encoderSpan 与 totalDeg；区间外仍按
 * 16384 counts = 360 度连续外推，便于 UI 显示 READY 之外的真实位置。
 */
float mapCalibratedEncoderToDeg(const JointCalibrationResult& calibration,
                                int32_t orientedRaw);

/** 将目标角限制到该关节当前有效动态区间；无 Auto 时退回 URDF 静态范围。 */
float clampJointTargetDeg(float targetDeg, uint8_t jointIndex);

} // namespace p4::control

#endif // P4_JOINT_ANGLE_MAPPER_H
