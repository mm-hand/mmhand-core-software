/**
 * @file EncoderHealthMonitor.cpp
 * @brief 实现磁编码器健康去抖、超时升级与共享位图发布。
 */
#include "EncoderHealthMonitor.h"

#include <string.h>

#include "../../platform/rtos/SharedSafetyState.h"
#include "../../runtime/SystemRuntimeContext.h"

namespace {
uint32_t encoderChannelMask()
{
    return ENCODER_TOTAL_NUM >= 32
        ? 0xFFFFFFFFUL
        : ((1UL << ENCODER_TOTAL_NUM) - 1UL);
}

void publishEncoderHealth(SystemRuntimeContext* sharedData,
                          const EncoderHealthState* health,
                          uint32_t nowMs)
{
    if (!sharedData || !health) return;
    uint32_t onlineMask = 0;
    uint32_t warnMask = 0;
    uint32_t faultMask = 0;
    for (uint8_t i = 0; i < ENCODER_TOTAL_NUM; ++i) {
        const uint32_t bit = 1UL << i;
        if (health[i].online) onlineMask |= bit;
        if (health[i].fault) {
            faultMask |= bit;
        } else if (health[i].warnUntilMs != 0 &&
                   static_cast<int32_t>(health[i].warnUntilMs - nowMs) > 0) {
            warnMask |= bit;
        }
    }
    SharedSafetyGuard guard;
    sharedAtomicStore(&sharedData->encoder_online_bitmap, onlineMask);
    sharedAtomicStore(&sharedData->encoder_warn_bitmap, warnMask);
    sharedAtomicStore(&sharedData->encoder_fault_bitmap, faultMask);
    sharedAtomicStore(&sharedData->can_link_online,
                      static_cast<uint8_t>((onlineMask | warnMask) != 0));
}

} // namespace

void updateEncoderHealthFromSnapshot(SystemRuntimeContext* sharedData,
                                     EncoderHealthState* health,
                                     uint32_t validMask,
                                     uint32_t nowMs)
{
    if (!health) return;
    for (uint8_t i = 0; i < ENCODER_TOTAL_NUM; ++i) {
        if ((validMask & (1UL << i)) != 0) {
            p4::safety::acceptEncoderSample(&health[i], nowMs);
        } else {
            p4::safety::rejectEncoderSample(&health[i], nowMs, true);
        }
    }
    publishEncoderHealth(sharedData, health, nowMs);
}

void markEncoderSnapshotBad(SystemRuntimeContext* sharedData,
                            EncoderHealthState* health,
                            uint32_t nowMs)
{
    if (!health) return;
    for (uint8_t i = 0; i < ENCODER_TOTAL_NUM; ++i) {
        p4::safety::rejectEncoderSample(&health[i], nowMs, true);
    }
    publishEncoderHealth(sharedData, health, nowMs);
}

bool applyEncoderHealthTimeout(SystemRuntimeContext* sharedData,
                               EncoderHealthState* health,
                               uint32_t nowMs)
{
    if (!health) return false;
    bool changed = false;
    for (uint8_t i = 0; i < ENCODER_TOTAL_NUM; ++i) {
        changed = p4::safety::applyEncoderFeedbackTimeout(&health[i], nowMs) ||
                  changed;
    }
    if (changed) publishEncoderHealth(sharedData, health, nowMs);
    return changed;
}

void clearEncoderHealth(SystemRuntimeContext* sharedData,
                        EncoderHealthState* health)
{
    if (health) {
        for (uint8_t i = 0; i < ENCODER_TOTAL_NUM; ++i) {
            p4::safety::resetEncoderHealthState(&health[i]);
        }
    }
    if (!sharedData) return;
    const uint32_t allEncoderMask = encoderChannelMask();
    const uint32_t activeMask =
        sharedAtomicLoad(&sharedData->active_joint_mask) & allEncoderMask;
    SharedSafetyGuard guard;
    sharedAtomicStore(&sharedData->encoder_online_bitmap, (uint32_t)0);
    sharedAtomicStore(&sharedData->encoder_warn_bitmap, (uint32_t)0);
    sharedAtomicStore(&sharedData->encoder_fault_bitmap,
                      activeMask != 0 ? activeMask : allEncoderMask);
    sharedAtomicStore(&sharedData->can_link_online, (uint8_t)0);
}
