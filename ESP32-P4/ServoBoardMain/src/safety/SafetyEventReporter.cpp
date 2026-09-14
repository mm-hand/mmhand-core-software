/**
 * @file SafetyEventReporter.cpp
 * @brief 实现固定容量、非阻塞的安全事件去重上报。
 */
#include "SafetyEventReporter.h"

#include "SafetyEventStore.h"
#include "../platform/rtos/AtomicValue.h"
#include "../platform/rtos/FreeRtosKernel.h"
#include "../platform/time/Clock.h"
#include "../runtime/SystemRuntimeContext.h"

namespace p4::safety {
namespace {

constexpr uint8_t kReportedRuleSlotCount = 64;
constexpr uint32_t kRuleRetriggerGapMs = 1000U;

/** 仅保存事件边沿，不参与任何业务裁决。 */
struct ReportedRuleSlot {
    SafetyRuleId ruleId = SafetyRuleId::None;
    uint32_t scopeKey = 0;
    uint32_t lastObservedMs = 0;
    bool active = false;
};

ReportedRuleSlot s_slots[kReportedRuleSlotCount] = {};
portMUX_TYPE s_reportMux = portMUX_INITIALIZER_UNLOCKED;

uint32_t eventScopeKey(uint8_t jointIndex, uint8_t motorChannel)
{
    if (jointIndex < ENCODER_TOTAL_NUM) {
        return 0x80000000UL | (1UL << jointIndex);
    }
    if (motorChannel < SERVO_TOTAL_NUM) {
        return 1UL << motorChannel;
    }
    return 0;
}

ReportedRuleSlot* findOrAllocateSlot(SafetyRuleId ruleId, uint32_t scopeKey)
{
    ReportedRuleSlot* freeSlot = nullptr;
    ReportedRuleSlot* oldestInactive = nullptr;
    for (ReportedRuleSlot& slot : s_slots) {
        if (slot.ruleId == ruleId && slot.scopeKey == scopeKey) return &slot;
        if (!slot.active && !freeSlot) freeSlot = &slot;
        if (!slot.active &&
            (!oldestInactive || slot.lastObservedMs < oldestInactive->lastObservedMs)) {
            oldestInactive = &slot;
        }
    }
    ReportedRuleSlot* selected = freeSlot ? freeSlot : oldestInactive;
    if (!selected) return nullptr;
    *selected = {};
    selected->ruleId = ruleId;
    selected->scopeKey = scopeKey;
    return selected;
}

bool reportWithContext(SafetyRuleId ruleId,
                       uint8_t systemState,
                       uint8_t owner,
                       const SafetyEventEvidence& evidence)
{
    if (ruleId == SafetyRuleId::None) return false;

    const uint32_t nowMs = p4::platform::time::monotonicMillis();
    const uint32_t scopeKey = eventScopeKey(evidence.jointIndex,
                                            evidence.motorChannel);
    bool shouldRecord = false;

    portENTER_CRITICAL(&s_reportMux);
    ReportedRuleSlot* slot = findOrAllocateSlot(ruleId, scopeKey);
    if (slot) {
        const bool retriggeredAfterGap = slot->active &&
            static_cast<uint32_t>(nowMs - slot->lastObservedMs) >=
                kRuleRetriggerGapMs;
        shouldRecord = !slot->active || retriggeredAfterGap;
        slot->active = true;
        slot->lastObservedMs = nowMs;
    }
    portEXIT_CRITICAL(&s_reportMux);

    if (shouldRecord) {
        recordSafetyRuleEvent(ruleId,
                              systemState,
                              owner,
                              evidence.jointIndex,
                              evidence.motorChannel,
                              evidence.requestedValue,
                              evidence.observedValue);
    }
    return shouldRecord;
}

} // namespace

void reportSafetyRuleEvent(SystemRuntimeContext* runtime,
                           SafetyRuleId ruleId,
                           const SafetyEventEvidence& evidence,
                           uint8_t fallbackSystemState,
                           uint8_t fallbackOwner)
{
    const uint8_t systemState = runtime
        ? p4::platform::rtos::atomicLoad(&runtime->system_state)
        : fallbackSystemState;
    const uint8_t owner = runtime
        ? p4::platform::rtos::atomicLoad(&runtime->servo_target_owner)
        : fallbackOwner;
    reportWithContext(ruleId, systemState, owner, evidence);
}

void reportSafetyRuleEvent(SystemRuntimeContext* runtime,
                           SafetyRuleId ruleId,
                           int32_t requestedValue,
                           int32_t observedValue,
                           uint8_t jointIndex,
                           uint8_t motorChannel,
                           uint8_t fallbackSystemState,
                           uint8_t fallbackOwner)
{
    reportSafetyRuleEvent(runtime,
                          ruleId,
                          makeSafetyEventEvidence(jointIndex,
                                                  motorChannel,
                                                  requestedValue,
                                                  observedValue),
                          fallbackSystemState,
                          fallbackOwner);
}

void reportSafetyRuleEvent(SafetyRuleId ruleId,
                           uint8_t systemState,
                           uint8_t owner,
                           const SafetyEventEvidence& evidence)
{
    reportWithContext(ruleId, systemState, owner, evidence);
}

bool reportServoSafetyDecision(const ServoSafetyDecision& decision,
                               uint8_t systemState,
                               uint8_t owner)
{
    if (decision.allowed || decision.ruleId == SafetyRuleId::None) return false;
    SafetyEventEvidence evidence;
    evidence.motorChannel = decision.motorChannel;
    evidence.requestedValue = decision.requestedAbs;
    evidence.observedValue = decision.hardwareAbs;
    return reportWithContext(decision.ruleId, systemState, owner, evidence);
}

void clearSafetyRuleEvent(SafetyRuleId ruleId,
                          uint8_t jointIndex,
                          uint8_t motorChannel)
{
    const uint32_t scopeKey = eventScopeKey(jointIndex, motorChannel);
    portENTER_CRITICAL(&s_reportMux);
    for (ReportedRuleSlot& slot : s_slots) {
        if (slot.ruleId == ruleId && slot.scopeKey == scopeKey) {
            slot.active = false;
            break;
        }
    }
    portEXIT_CRITICAL(&s_reportMux);
}

} // namespace p4::safety
