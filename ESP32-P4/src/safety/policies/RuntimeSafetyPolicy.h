/**
 * @file RuntimeSafetyPolicy.h
 * @brief 集中定义任务低栈和连续周期超限的触发、锁存与恢复判断。
 */
#ifndef P4_SAFETY_POLICIES_RUNTIME_SAFETY_POLICY_H
#define P4_SAFETY_POLICIES_RUNTIME_SAFETY_POLICY_H

#include <stdint.h>

#include "../../data/SafetyRuleTypes.h"

namespace p4::safety {

struct RuntimeLatchDecision {
    bool active;
    bool entered;
    bool recovered;
};

/** 启动阶段创建的 RTOS 资源类型；用于把失败统一映射为 RUN 规则。 */
enum class RuntimeResourceKind : uint8_t {
    Queue = 0,
    Mutex,
    Task,
};

/** @brief 资源创建成功返回 None，失败返回对应 RUN-8002~8004。 */
SafetyRuleId runtimeResourceCreationRule(RuntimeResourceKind kind,
                                         bool creationSucceeded);

/** @brief 环形缓存下一次写入是否会覆盖最旧安全事件。 */
bool runtimeEventCacheWillOverflow(uint32_t eventCount,
                                   uint32_t capacity);

uint32_t runtimeStackWarningThreshold(uint32_t allocatedBytes,
                                      uint32_t minimumThresholdBytes);
RuntimeLatchDecision evaluateRuntimeStack(uint32_t freeBytes,
                                          uint32_t thresholdBytes,
                                          bool previouslyActive);
RuntimeLatchDecision evaluateRuntimeOverrun(uint16_t consecutiveOverruns,
                                            uint16_t triggerCount,
                                            bool previouslyActive);

} // namespace p4::safety

#endif // P4_SAFETY_POLICIES_RUNTIME_SAFETY_POLICY_H
