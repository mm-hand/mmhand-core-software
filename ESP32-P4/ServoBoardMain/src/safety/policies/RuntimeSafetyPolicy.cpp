/**
 * @file RuntimeSafetyPolicy.cpp
 * @brief 实现运行时资源阈值和事件边沿判断。
 */
#include "RuntimeSafetyPolicy.h"

namespace p4::safety {

SafetyRuleId runtimeResourceCreationRule(RuntimeResourceKind kind,
                                         bool creationSucceeded)
{
    if (creationSucceeded) return SafetyRuleId::None;
    switch (kind) {
        case RuntimeResourceKind::Queue:
            return SafetyRuleId::RuntimeQueueCreateFailed;
        case RuntimeResourceKind::Mutex:
            return SafetyRuleId::RuntimeMutexCreateFailed;
        case RuntimeResourceKind::Task:
            return SafetyRuleId::RuntimeTaskCreateFailed;
        default:
            return SafetyRuleId::RuntimeTaskCreateFailed;
    }
}

bool runtimeEventCacheWillOverflow(uint32_t eventCount,
                                   uint32_t capacity)
{
    return capacity > 0 && eventCount >= capacity;
}

uint32_t runtimeStackWarningThreshold(uint32_t allocatedBytes,
                                      uint32_t minimumThresholdBytes)
{
    const uint32_t ratioThreshold = allocatedBytes / 10U;
    return ratioThreshold < minimumThresholdBytes
        ? minimumThresholdBytes
        : ratioThreshold;
}

RuntimeLatchDecision evaluateRuntimeStack(uint32_t freeBytes,
                                          uint32_t thresholdBytes,
                                          bool previouslyActive)
{
    const bool active = freeBytes > 0 && freeBytes <= thresholdBytes;
    return {active, active && !previouslyActive,
            !active && previouslyActive};
}

RuntimeLatchDecision evaluateRuntimeOverrun(uint16_t consecutiveOverruns,
                                            uint16_t triggerCount,
                                            bool previouslyActive)
{
    const bool active = consecutiveOverruns >= triggerCount;
    return {active, active && !previouslyActive,
            !active && previouslyActive};
}

} // namespace p4::safety
