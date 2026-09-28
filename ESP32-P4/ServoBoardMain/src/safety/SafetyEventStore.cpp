/**
 * @file SafetyEventStore.cpp
 * @brief 实现跨核安全事件环形缓存，供日志和上位机诊断读取。
 */
#include "SafetyEventStore.h"
#include "policies/RuntimeSafetyPolicy.h"

#include "../platform/rtos/FreeRtosKernel.h"
#include "../platform/time/Clock.h"

namespace p4::safety {
namespace {

SafetyEventRecord s_events[kSafetyEventCacheCapacity] = {};
size_t s_writeIndex = 0;
size_t s_eventCount = 0;
uint32_t s_sequence = 0;
uint32_t s_overflowCount = 0;
bool s_overflowEvidenceRecorded = false;
portMUX_TYPE s_eventMux = portMUX_INITIALIZER_UNLOCKED;

/** 在已持有临界区时写入一条事件；调用方负责处理满缓存语义。 */
void appendEventLocked(SafetyEventRecord event)
{
    event.sequence = ++s_sequence;
    s_events[s_writeIndex] = event;
    s_writeIndex = (s_writeIndex + 1U) % kSafetyEventCacheCapacity;
    if (s_eventCount < kSafetyEventCacheCapacity) {
        ++s_eventCount;
    }
}

} // namespace

void recordSafetyEvent(const SafetyEventRecord& input)
{
    // 单调时钟放在临界区外读取，避免跨核安全缓存持锁期间调用平台服务。
    SafetyEventRecord event = input;
    event.timestampMs = p4::platform::time::monotonicMillis();

    // 安全事件可能由状态机核和舵机核同时写入，临界区只做固定长度结构复制。
    portENTER_CRITICAL(&s_eventMux);
    if (runtimeEventCacheWillOverflow(
            static_cast<uint32_t>(s_eventCount),
            static_cast<uint32_t>(kSafetyEventCacheCapacity))) {
        ++s_overflowCount;
        if (!s_overflowEvidenceRecorded) {
            // 第一次覆盖额外插入 RUN-8001。后续只累计计数，避免事件风暴。
            SafetyEventRecord overflow = {};
            overflow.timestampMs = event.timestampMs;
            overflow.ruleId = SafetyRuleId::RuntimeSafetyEventCacheOverflow;
            overflow.motorChannel = 0xFF;
            overflow.jointIndex = 0xFF;
            overflow.requestedValue = static_cast<int32_t>(s_overflowCount);
            overflow.observedValue = static_cast<int32_t>(kSafetyEventCacheCapacity);
            appendEventLocked(overflow);
            s_overflowEvidenceRecorded = true;
        }
    }
    appendEventLocked(event);
    portEXIT_CRITICAL(&s_eventMux);
}

void recordSafetyRuleEvent(SafetyRuleId ruleId,
                           uint8_t systemState,
                           uint8_t owner,
                           uint8_t jointIndex,
                           uint8_t motorChannel,
                           int32_t requestedValue,
                           int32_t observedValue)
{
    if (ruleId == SafetyRuleId::None) return;
    SafetyEventRecord event = {};
    event.ruleId = ruleId;
    event.systemState = systemState;
    event.owner = owner;
    event.jointIndex = jointIndex;
    event.motorChannel = motorChannel;
    event.requestedValue = requestedValue;
    event.observedValue = observedValue;
    recordSafetyEvent(event);
}

size_t copySafetyEventsAfter(uint32_t afterSequence,
                             SafetyEventRecord* output,
                             size_t capacity)
{
    if (!output || capacity == 0) {
        return 0;
    }

    size_t copied = 0;
    portENTER_CRITICAL(&s_eventMux);
    const size_t oldest =
        (s_writeIndex + kSafetyEventCacheCapacity - s_eventCount) %
        kSafetyEventCacheCapacity;
    for (size_t i = 0; i < s_eventCount && copied < capacity; ++i) {
        const SafetyEventRecord& event =
            s_events[(oldest + i) % kSafetyEventCacheCapacity];
        // 有符号差值可在 uint32 sequence 回绕后继续维持“更新”语义。
        if (static_cast<int32_t>(event.sequence - afterSequence) > 0) {
            output[copied++] = event;
        }
    }
    portEXIT_CRITICAL(&s_eventMux);
    return copied;
}

uint32_t safetyEventOverflowCount()
{
    portENTER_CRITICAL(&s_eventMux);
    const uint32_t value = s_overflowCount;
    portEXIT_CRITICAL(&s_eventMux);
    return value;
}

} // namespace p4::safety
