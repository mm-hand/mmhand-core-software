/**
 * @file CommunicationSafetyPolicy.cpp
 * @brief 实现通信输入的纯校验规则，不读串口/CAN，也不修改重组缓存。
 */
#include "CommunicationSafetyPolicy.h"

namespace p4::safety {

SafetyRuleId communicationCanInitializationRule(bool initialized)
{
    return initialized
        ? SafetyRuleId::None
        : SafetyRuleId::CommunicationCanInitFailed;
}

bool communicationLengthValid(size_t actual, size_t expected)
{
    return actual == expected;
}

bool communicationReassemblyComplete(bool active,
                                     uint32_t actualFrameMask,
                                     uint32_t expectedFrameMask)
{
    return active && actualFrameMask == expectedFrameMask;
}

bool communicationFrameCountComplete(uint32_t actualFrameCount,
                                     uint32_t expectedFrameCount)
{
    return expectedFrameCount > 0 && actualFrameCount >= expectedFrameCount;
}

bool communicationIndexValid(uint32_t index, uint32_t itemCount)
{
    return itemCount > 0 && index < itemCount;
}

bool communicationCrcValid(bool metadataValid,
                           uint16_t actualCrc,
                           uint16_t expectedCrc)
{
    return metadataValid && actualCrc == expectedCrc;
}

bool communicationAssemblyExpired(uint32_t nowMs,
                                  uint32_t firstFrameMs,
                                  uint32_t timeoutMs)
{
    return static_cast<uint32_t>(nowMs - firstFrameMs) > timeoutMs;
}

bool communicationStreamStale(bool streamValid,
                              uint32_t nowMs,
                              uint32_t lastUpdateMs,
                              uint32_t staleAfterMs)
{
    return streamValid &&
           static_cast<uint32_t>(nowMs - lastUpdateMs) > staleAfterMs;
}

bool communicationEventMayReport(uint32_t nowMs,
                                 uint32_t lastReportMs,
                                 uint32_t minimumIntervalMs)
{
    return lastReportMs == 0 ||
           static_cast<uint32_t>(nowMs - lastReportMs) >= minimumIntervalMs;
}

} // namespace p4::safety
