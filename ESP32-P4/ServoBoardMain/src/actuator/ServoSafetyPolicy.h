/**
 * @file ServoSafetyPolicy.h
 * @brief 集中定义所有舵机目标在进入执行邮箱前必须满足的结构安全规则。
 */
#ifndef P4_SERVO_SAFETY_POLICY_H
#define P4_SERVO_SAFETY_POLICY_H

#include "../domain/ActuatorTypes.h"

namespace p4::actuator {

/**
 * 校验批次数量、总线编号和舵机 ID。
 *
 * logical ABS/LUT/硬件行程属于依赖实时反馈的发送前安全规则，仍由执行层进行
 * 最后裁决；这里先阻止结构损坏的数据进入邮箱。
 */
bool batchStructureIsSafe(const ServoTargetBatch_t& batch);

} // namespace p4::actuator

#endif // P4_SERVO_SAFETY_POLICY_H
