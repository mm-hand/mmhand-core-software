/**
 * @file SafetyEventReporter.h
 * @brief 提供安全规则事件的统一上报和恢复清除接口。
 *
 * 本模块只负责诊断事件去重与缓存，不判断业务条件，不执行 Hold、Clamp、Stop
 * 或状态迁移。各业务模块必须调用对应 Policy 完成判断，并自行执行 Policy 结果。
 */
#ifndef P4_SAFETY_EVENT_REPORTER_H
#define P4_SAFETY_EVENT_REPORTER_H

#include "../data/SafetyRuleTypes.h"
#include "../data/SafetyTypes.h"

#include <stdint.h>

struct SystemRuntimeContext;

namespace p4::safety {

/** 一条规则事件携带的对象和数值证据。 */
struct SafetyEventEvidence {
    uint8_t jointIndex = 0xFF;
    uint8_t motorChannel = 0xFF;
    int32_t requestedValue = 0;
    int32_t observedValue = 0;
};

/** 便捷构造规则证据，避免业务调用点依赖 C++ 指定初始化语法。 */
inline SafetyEventEvidence makeSafetyEventEvidence(
    uint8_t jointIndex = 0xFF,
    uint8_t motorChannel = 0xFF,
    int32_t requestedValue = 0,
    int32_t observedValue = 0)
{
    SafetyEventEvidence evidence;
    evidence.jointIndex = jointIndex;
    evidence.motorChannel = motorChannel;
    evidence.requestedValue = requestedValue;
    evidence.observedValue = observedValue;
    return evidence;
}

/**
 * @brief 上报一条已经由业务 Policy 判定成立的规则。
 * @details 同一 ruleId 与同一 J/M 对象持续成立时只记录一次；条件恢复后应调用
 * clearSafetyRuleEvent()，使下一次独立触发能够重新记录。
 */
void reportSafetyRuleEvent(SystemRuntimeContext* runtime,
                           SafetyRuleId ruleId,
                           const SafetyEventEvidence& evidence = {},
                           uint8_t fallbackSystemState = 0xFF,
                           uint8_t fallbackOwner = 0xFF);

/** 适合业务现场直接传入 expected/observed 与可选 J/M 对象的上报接口。 */
void reportSafetyRuleEvent(SystemRuntimeContext* runtime,
                           SafetyRuleId ruleId,
                           int32_t requestedValue,
                           int32_t observedValue,
                           uint8_t jointIndex = 0xFF,
                           uint8_t motorChannel = 0xFF,
                           uint8_t fallbackSystemState = 0xFF,
                           uint8_t fallbackOwner = 0xFF);

/** 无运行时上下文时使用显式 state/owner 上报，例如系统装配失败。 */
void reportSafetyRuleEvent(SafetyRuleId ruleId,
                           uint8_t systemState,
                           uint8_t owner,
                           const SafetyEventEvidence& evidence = {});

/**
 * 把舵机 Policy 的拒绝结果转换为规则事件。
 * @return 本次确实写入了一条新事件时返回 true；持续重复的同一拒绝返回 false。
 */
bool reportServoSafetyDecision(const ServoSafetyDecision& decision,
                               uint8_t systemState,
                               uint8_t owner);

/** 标记指定规则和对象已经恢复。 */
void clearSafetyRuleEvent(SafetyRuleId ruleId,
                          uint8_t jointIndex = 0xFF,
                          uint8_t motorChannel = 0xFF);

} // namespace p4::safety

#endif // P4_SAFETY_EVENT_REPORTER_H
