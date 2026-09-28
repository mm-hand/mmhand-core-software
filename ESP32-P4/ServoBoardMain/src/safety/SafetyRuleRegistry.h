/**
 * @file SafetyRuleRegistry.h
 * @brief 声明业务枚举、拒绝原因和故障位到稳定安全规则 ID 的映射接口。
 * @details 本文件不是完整规则目录。只有需要“运行时枚举转规则 ID”的规则经过这里；
 *          Policy 直接返回 SafetyRuleId 的规则不应重复登记。全部规则调用关系位于
 *          generated/SafetyRuleIndex.generated.json，仅供审计工具读取，不进入固件。
 */
#ifndef P4_SAFETY_SAFETY_RULE_REGISTRY_H
#define P4_SAFETY_SAFETY_RULE_REGISTRY_H

#include "../data/SafetyRuleTypes.h"
#include "../data/SafetyTypes.h"
#include "../data/CalibrationTypes.h"
#include "../data/SystemTypes.h"

namespace p4::safety {

/** 将舵机拒绝枚举映射到稳定规则 ID。 */
SafetyRuleId ruleForServoRejectReason(ServoRejectReason reason);

/** 将状态机直控拒绝原因映射到稳定规则 ID。 */
SafetyRuleId ruleForDirectRejectReason(uint8_t reason);

/** 将状态机负反馈拒绝原因映射到稳定规则 ID。 */
SafetyRuleId ruleForFeedbackRejectReason(uint8_t reason);

/** @brief 将 READY 评估拒绝原因映射为 RDY-33xx 稳定规则。 */
SafetyRuleId ruleForFeedbackReadyRejectReason(uint8_t reason);

/** 将 Rope Init 失败原因映射到稳定规则 ID。 */
SafetyRuleId ruleForRopeInitReason(uint8_t reason);

/** 将自动标定诊断原因映射到稳定规则 ID。 */
SafetyRuleId ruleForCalibrationDiagReason(uint8_t reason);

/** 将 CALIB_WARN_* 单一告警位映射到稳定规则 ID。 */
SafetyRuleId ruleForCalibrationWarningBit(uint8_t warningBit);

/** 将系统故障位映射到稳定规则 ID；组合位图应逐位调用。 */
SafetyRuleId ruleForSystemFaultBit(uint32_t faultBit);

} // namespace p4::safety

#endif // P4_SAFETY_SAFETY_RULE_REGISTRY_H
