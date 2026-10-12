/**
 * @file JointAngleMapper.h
 * @brief 提供磁编码器 raw、Manual 机械端点与 URDF 角度之间的纯数学映射。
 *
 * 该模块不读取任务状态、不发送舵机命令。调用方必须显式传入标定结果，
 * 因而可以单独测试跨圈、方向翻转和工作区外角度外推。
 */
#ifndef P4_JOINT_ANGLE_MAPPER_H
#define P4_JOINT_ANGLE_MAPPER_H

#include <stdint.h>


namespace p4::control {

/** 将角度换算为最近的 14 bit 编码器 counts，结果允许带符号。 */
int32_t degToEncoderCount(float deg);

/** 将目标角限制到该关节的静态 URDF Min/Max。 */
float clampJointTargetDeg(float targetDeg, uint8_t jointIndex);

} // namespace p4::control

#endif // P4_JOINT_ANGLE_MAPPER_H
