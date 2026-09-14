/**
 * @file RopeInitSafetyPolicy.cpp
 * @brief 实现 Rope Init 前置数据、会话所有权和反馈新鲜度判定。
 */
#include "RopeInitSafetyPolicy.h"

#include "../../generated/UrdfJointLimits.generated.h"
#include "../../modules/calibration/CalibrationMath.h"
#include "../../modules/calibration/CalibrationRuntimeStore.h"
#include "../../modules/calibration/ManualEncoderReferenceStore.h"
#include "../../platform/rtos/SharedSafetyState.h"

namespace p4::safety {
namespace {
constexpr uint32_t kRopeInitTimeoutMs = 8000;
constexpr uint32_t kRopeInitFeedbackMaxAgeMs = 500;
} // namespace

bool ropeInitCalibrationMatchesManual(uint8_t jointIndex)
{
    if (jointIndex >= ENCODER_TOTAL_NUM) return false;
    JointCalibrationRuntimeSnapshot snapshot;
    if (!readJointCalibrationRuntimeSnapshot(jointIndex, &snapshot)) {
        return false;
    }
    const JointCalibrationResult& result = snapshot.result;
    const uint32_t manualCrc = manualEncoderReferenceEntryCrc(
        jointIndex, snapshot.manual);
    return result.success &&
           result.mappingVersion == kManualScaledUrdfFeedbackMappingVersion &&
           result.urdfConfigCrc == kManualScaledUrdfConfigCrc &&
           result.manualReferenceCrc == manualCrc;
}

uint8_t evaluateRopeInitActiveScope(SystemRuntimeContext* runtime)
{
    if (!runtime || !runtime->manual_reference_committed) {
        return ROPE_INIT_REASON_MANUAL_REQUIRED;
    }
    const uint32_t activeMask =
        sharedAtomicLoad(&runtime->active_joint_mask) &
        kManualEncoderReferenceFullMask;
    if (activeMask == 0 ||
        !manualEncoderReferencesCoverActiveMask(activeMask)) {
        return ROPE_INIT_REASON_MANUAL_REQUIRED;
    }
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; ++joint) {
        if ((activeMask & (1UL << joint)) != 0 &&
            !ropeInitCalibrationMatchesManual(joint)) {
            return ROPE_INIT_REASON_AUTO_REQUIRED;
        }
    }
    return ROPE_INIT_REASON_NONE;
}

RopeInitSessionDecision evaluateRopeInitSession(SystemRuntimeContext* runtime,
                                                uint32_t nowMs,
                                                uint32_t beginMs)
{
    RopeInitSessionDecision decision = {};
    if (!runtime ||
        sharedAtomicLoad(&runtime->system_state) != SYSTEM_STATE_ROPE_INIT_RUNNING ||
        sharedAtomicLoad(&runtime->servo_target_owner) !=
            SERVO_TARGET_OWNER_CALIBRATION ||
        sharedAtomicLoad(&runtime->control_enabled) != 0) {
        decision.reason = ROPE_INIT_REASON_CANCELLED;
        return decision;
    }
    decision.active = true;
    const uint32_t faults =
        sharedAtomicLoad(&runtime->system_fault_bitmap) |
        sharedAtomicLoad(&runtime->system_fault_latched_bitmap);
    if (faults != 0) {
        decision.reason = ROPE_INIT_REASON_SYSTEM_FAULT;
    } else if ((uint32_t)(nowMs - beginMs) > kRopeInitTimeoutMs) {
        decision.reason = ROPE_INIT_REASON_TIMEOUT;
    }
    return decision;
}

uint8_t readRopeInitFeedback(SystemRuntimeContext* runtime,
                             uint32_t nowMs,
                             MappedAngleData_t* mapped,
                             ServoAngleData_t* servo)
{
    if (!runtime || !mapped || !servo ||
        !runtime->mappedAngleQueue.peek(mapped) ||
        !runtime->servoFeedbackQueue.peek(servo) || !mapped->isValid ||
        (uint32_t)(nowMs - mapped->timestamp) > kRopeInitFeedbackMaxAgeMs ||
        (uint32_t)(nowMs - servo->timestamp) > kRopeInitFeedbackMaxAgeMs) {
        return ROPE_INIT_REASON_FEEDBACK_STALE;
    }
    return ROPE_INIT_REASON_NONE;
}

} // namespace p4::safety
