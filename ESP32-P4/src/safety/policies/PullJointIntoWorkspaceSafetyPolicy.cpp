/**
 * @file PullJointIntoWorkspaceSafetyPolicy.cpp
 * @brief 实现 Pull Joint into Workspace 前置数据、会话所有权和反馈新鲜度判定。
 */
#include "PullJointIntoWorkspaceSafetyPolicy.h"

#include "../../generated/UrdfJointLimits.generated.h"
#include "../../system/calibration/CalibrationMath.h"
#include "../../system/calibration/CalibrationValidity.h"
#include "../../system/calibration/CalibrationRuntimeStore.h"
#include "../../system/calibration/ManualEncoderReferenceStore.h"
#include "../../system/rtos/SharedSafetyState.h"

namespace p4::safety {
namespace {
constexpr uint32_t kPullJointIntoWorkspaceTimeoutMs = 10000;
constexpr uint32_t kPullJointIntoWorkspaceFeedbackMaxAgeMs = 500;
} // namespace

bool pullJointIntoWorkspaceCalibrationMatchesManual(uint8_t jointIndex)
{
    if (jointIndex >= ENCODER_TOTAL_NUM) return false;
    JointCalibrationRuntimeSnapshot snapshot;
    if (!readJointCalibrationRuntimeSnapshot(jointIndex, &snapshot)) {
        return false;
    }
    return calibrationMatchesManual(jointIndex, snapshot);
}

uint8_t evaluatePullJointIntoWorkspaceActiveScope(SystemRuntimeContext* runtime)
{
    if (!runtime || !runtime->manual_reference_committed) {
        return PULL_JOINT_INTO_WORKSPACE_REASON_MANUAL_REQUIRED;
    }
    const uint32_t activeMask =
        sharedAtomicLoad(&runtime->active_joint_mask) &
        kManualEncoderReferenceFullMask;
    if (activeMask == 0 ||
        !manualEncoderReferencesCoverActiveMask(activeMask)) {
        return PULL_JOINT_INTO_WORKSPACE_REASON_MANUAL_REQUIRED;
    }
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; ++joint) {
        if ((activeMask & (1UL << joint)) != 0 &&
            !pullJointIntoWorkspaceCalibrationMatchesManual(joint)) {
            return PULL_JOINT_INTO_WORKSPACE_REASON_AUTO_REQUIRED;
        }
    }
    return PULL_JOINT_INTO_WORKSPACE_REASON_NONE;
}

PullJointIntoWorkspaceSessionDecision evaluatePullJointIntoWorkspaceSession(SystemRuntimeContext* runtime,
                                                uint32_t nowMs,
                                                uint32_t beginMs)
{
    PullJointIntoWorkspaceSessionDecision decision = {};
    if (!runtime ||
        sharedAtomicLoad(&runtime->system_state) != SYSTEM_STATE_PULL_JOINT_INTO_WORKSPACE_RUNNING ||
        sharedAtomicLoad(&runtime->servo_target_owner) !=
            SERVO_TARGET_OWNER_CALIBRATION ||
        sharedAtomicLoad(&runtime->control_enabled) != 0) {
        decision.reason = PULL_JOINT_INTO_WORKSPACE_REASON_CANCELLED;
        return decision;
    }
    decision.active = true;
    const uint32_t faults =
        sharedAtomicLoad(&runtime->system_fault_bitmap) |
        sharedAtomicLoad(&runtime->system_fault_latched_bitmap);
    if (faults != 0) {
        decision.reason = PULL_JOINT_INTO_WORKSPACE_REASON_SYSTEM_FAULT;
    } else if ((uint32_t)(nowMs - beginMs) > kPullJointIntoWorkspaceTimeoutMs) {
        decision.reason = PULL_JOINT_INTO_WORKSPACE_REASON_TIMEOUT;
    }
    return decision;
}

uint8_t readPullJointIntoWorkspaceFeedback(SystemRuntimeContext* runtime,
                             uint32_t nowMs,
                             MappedAngleData_t* mapped,
                             ServoAngleData_t* servo)
{
    if (!runtime || !mapped || !servo ||
        !runtime->mappedAngleQueue.peek(mapped) ||
        !runtime->servoFeedbackQueue.peek(servo) || !mapped->isValid ||
        (uint32_t)(nowMs - mapped->timestamp) > kPullJointIntoWorkspaceFeedbackMaxAgeMs ||
        (uint32_t)(nowMs - servo->timestamp) > kPullJointIntoWorkspaceFeedbackMaxAgeMs) {
        return PULL_JOINT_INTO_WORKSPACE_REASON_FEEDBACK_STALE;
    }
    return PULL_JOINT_INTO_WORKSPACE_REASON_NONE;
}

} // namespace p4::safety
