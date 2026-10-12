/**
 * @file ControlInputSafetyPolicy.h
 * @brief 定义负反馈控制输入的结构化安全裁决。
 * @details 本模块只判断 CAN、磁编与映射角是否足以进入 PID；不访问 RTOS、舵机总线或上位机协议。
 */
#ifndef P4_CONTROL_INPUT_SAFETY_POLICY_H
#define P4_CONTROL_INPUT_SAFETY_POLICY_H

#include <Arduino.h>

#include "../../data/SafetyRuleTypes.h"
#include "../../system/SystemRuntimeContext.h"

namespace p4::safety {

/**
 * @brief 一次关节反馈安全检查的完整结果。
 * @details `allowed=false` 时调用方必须保持该关节当前舵机 ABS，并把 ruleId 与证据写入安全事件缓存。
 */
struct ControlSafetyDecision {
    bool allowed;                 ///< true 表示反馈可用于本周期 PID。
    SafetyRuleId ruleId;          ///< 拒绝原因；允许时为 None。
    int32_t requestedValue;       ///< 规则期望值，用于上位机诊断。
    int32_t observedValue;        ///< 实际观测值，用于还原触发现场。
};

/**
 * @brief 检查一个关节的闭环反馈是否有效。
 * @return 结构化裁决；检查顺序固定为 CAN、索引、磁编故障、映射有效位。
 */
ControlSafetyDecision evaluateControlSafety(
    bool canBusOnline,
    const RemoteSensorData_t& sensorData,
    const MappedAngleData_t& mappedData,
    uint8_t jointIndex,
    bool encoderFeedbackFault);

} // namespace p4::safety

#endif // P4_CONTROL_INPUT_SAFETY_POLICY_H
