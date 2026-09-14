/**
 * @file SafetyRuleTypes.h
 * @brief 定义跨 Safety 模块共享的稳定规则编号和事件记录格式。
 * @details 规则 ID 会进入日志、上位机解释和回归报告，发布后不得复用或重编号。
 */
#ifndef MMHAND_SAFETY_RULE_TYPES_H
#define MMHAND_SAFETY_RULE_TYPES_H

#include <stdint.h>

enum class SafetyRuleId : uint16_t {
    None = 0,
#define P4_SAFETY_RULE(name, value, code, title, severity, layer, action, recovery) name = value,
#include "../generated/SafetyRuleCatalog.generated.h"
#undef P4_SAFETY_RULE
};

struct SafetyEventRecord {
    uint32_t sequence; ///< 单调递增事件序号。
    uint32_t timestampMs; ///< P4 millis() 时间。
    SafetyRuleId ruleId; ///< 对应的稳定安全规则 ID。
    uint8_t motorChannel; ///< Mxx；无单电机对象时为 0xFF。
    uint8_t jointIndex; ///< Jxx；无单关节对象时为 0xFF。
    uint8_t systemState; ///< 触发时顶层状态。
    uint8_t owner; ///< 触发时舵机输出 owner。
    int32_t requestedValue; ///< 请求值，含义由规则决定。
    int32_t observedValue; ///< 实测/换算值，含义由规则决定。
};

#endif // MMHAND_SAFETY_RULE_TYPES_H
