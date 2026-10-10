/**
 * @file SystemReadinessPolicy.cpp
 * @brief 实现 active scope、反馈新鲜度、标定一致性与工作空间迟滞判定。
 */
#include "SystemReadinessPolicy.h"

#include "SystemFaultPolicy.h"
#include "../../config/SafetyConfig.h"
#include "../../config/topology/HardwareMap.h"
#include "../../data/SafetyTypes.h"
#include "../../generated/UrdfJointLimits.generated.h"
#include "../../modules/calibration/CalibrationMath.h"
#include "../../modules/calibration/CalibrationRuntimeStore.h"
#include "../../modules/calibration/ManualEncoderReferenceStore.h"
#include "../../platform/rtos/SharedSafetyState.h"
#include "../../platform/time/Clock.h"

using namespace mmhand::config::safety;

namespace p4::state {
namespace {

constexpr int32_t kReadyExitOutsideToleranceCounts = 48;
constexpr uint8_t kWorkspaceStableSampleCount = 3;

uint32_t s_workspaceReadyLatchMask = 0;
uint32_t s_workspaceLastMappedTimestamp = 0;
uint8_t s_workspaceEnterStable[ENCODER_TOTAL_NUM] = {0};
uint8_t s_workspaceExitStable[ENCODER_TOTAL_NUM] = {0};
uint16_t s_workspaceAutoMinRaw[ENCODER_TOTAL_NUM] = {0};
uint16_t s_workspaceAutoMaxRaw[ENCODER_TOTAL_NUM] = {0};
uint32_t s_workspaceManualCrc[ENCODER_TOTAL_NUM] = {0};
uint8_t s_workspaceMappingVersion[ENCODER_TOTAL_NUM] = {0};

bool routesControllable(SystemRuntimeContext* runtime, uint32_t requiredMask)
{
    if (!runtime || requiredMask == 0) {
        return false;
    }
    const uint8_t scanState =
        sharedAtomicLoad(&runtime->servo_route_scan_state);
    return (scanState == 0 || scanState == 2) &&
           (requiredMask &
            ~sharedAtomicLoad(&runtime->servo_route_controllable_bitmap)) == 0;
}

FeedbackReadinessDecision reject(uint8_t reason,
                                 uint32_t a = 0,
                                 uint32_t b = 0,
                                 int32_t c = 0,
                                 int32_t d = 0,
                                 uint32_t recoverableMask = 0)
{
    FeedbackReadinessDecision decision = {};
    decision.ready = false;
    decision.recoverableBelowMask = recoverableMask;
    decision.reason = reason;
    decision.a = a;
    decision.b = b;
    decision.c = c;
    decision.d = d;
    return decision;
}

} // namespace

uint32_t requiredMotorMask(SystemRuntimeContext* runtime)
{
    if (!runtime) {
        return 0;
    }
    uint32_t mask = sharedAtomicLoad(&runtime->active_motor_mask);
    if (sharedAtomicLoad(&runtime->system_state) == SYSTEM_STATE_RUN_DIRECT) {
        mask = sharedAtomicLoad(&runtime->applied_direct_motor_mask);
    }
    return mask & validMotorChannelMask();
}

bool requiresCan(SystemRuntimeContext* runtime)
{
    if (!runtime || sharedAtomicLoad(&runtime->control_mode) ==
                        CONTROL_MODE_DIRECT_MOTOR) {
        return false;
    }
    return sharedAtomicLoad(&runtime->require_can_feedback) != 0;
}

bool requiredServosOnline(SystemRuntimeContext* runtime,
                          const ServoAngleData_t& servoSnapshot)
{
    const uint32_t mask = requiredMotorMask(runtime);
    if (mask == 0 || !routesControllable(runtime, mask)) {
        return false;
    }
    for (uint8_t motor = 0; motor < SERVO_TOTAL_NUM; ++motor) {
        if ((mask & (1UL << motor)) != 0 &&
            servoSnapshot.onlineStatus[motor] == 0) {
            return false;
        }
    }
    return true;
}

bool directMotorMaskReady(SystemRuntimeContext* runtime,
                          uint32_t requiredMask)
{
    if (!runtime) {
        return false;
    }
    requiredMask &= validMotorChannelMask();
    if (requiredMask == 0) {
        return false;
    }

    ServoAngleData_t snapshot = {};
    const uint32_t nowMs = p4::platform::time::monotonicMillis();
    if (!runtime->servoFeedbackQueue.peek(&snapshot) ||
        (uint32_t)(nowMs - snapshot.timestamp) >
            kStateServoFeedbackTimeoutMs) {
        return false;
    }
    for (uint8_t motor = 0; motor < SERVO_TOTAL_NUM; ++motor) {
        if ((requiredMask & (1UL << motor)) != 0 &&
            snapshot.onlineStatus[motor] == 0) {
            return false;
        }
    }
    if ((sharedAtomicLoad(&runtime->multi_turn_unknown_bitmap) & requiredMask) != 0 ||
        (sharedAtomicLoad(&runtime->servo_turn_overflow_bitmap) & requiredMask) != 0 ||
        !routesControllable(runtime, requiredMask)) {
        return false;
    }

    const uint32_t scopeFaults = SYSTEM_FAULT_CAN_OFFLINE |
                                 SYSTEM_FAULT_SERVO_OFFLINE |
                                 SYSTEM_FAULT_MULTITURN_UNKNOWN;
    const uint32_t blocking =
        (sharedAtomicLoad(&runtime->system_fault_bitmap) |
         sharedAtomicLoad(&runtime->system_fault_latched_bitmap)) &
        ~scopeFaults;
    return blocking == 0;
}

bool controllableEntryReady(SystemRuntimeContext* runtime)
{
    return runtime && directMotorMaskReady(runtime, requiredMotorMask(runtime));
}

bool manualReferenceGateReady(SystemRuntimeContext* runtime)
{
    if (!runtime) {
        return false;
    }
    const uint32_t activeMask =
        sharedAtomicLoad(&runtime->active_joint_mask) &
        kManualEncoderReferenceFullMask;
    const uint32_t validMask =
        sharedAtomicLoad(&runtime->manual_reference_valid_bitmap) &
        kManualEncoderReferenceFullMask;
    return activeMask != 0 && runtime->manual_reference_committed != 0 &&
           (validMask & activeMask) == activeMask &&
           manualEncoderReferencesCoverActiveMask(activeMask);
}

bool activeMotorsMultiTurnKnown(SystemRuntimeContext* runtime)
{
    const uint32_t activeMask = requiredMotorMask(runtime);
    return runtime && activeMask != 0 &&
           (sharedAtomicLoad(&runtime->multi_turn_unknown_bitmap) & activeMask) == 0 &&
           (sharedAtomicLoad(&runtime->servo_turn_overflow_bitmap) & activeMask) == 0;
}

bool activeMotorsAbsLimitsCalibrated(SystemRuntimeContext* runtime)
{
    const uint32_t activeMask = requiredMotorMask(runtime);
    return runtime && activeMask != 0 &&
           (activeMask &
            ~sharedAtomicLoad(&runtime->motor_abs_limit_valid_bitmap)) == 0;
}

FeedbackReadinessDecision evaluateFeedbackReadiness(
    SystemRuntimeContext* runtime,
    uint32_t nowMs)
{
    if (!runtime || !manualReferenceGateReady(runtime)) {
        return reject(FEEDBACK_READY_REJECT_CALIBRATION_MISSING);
    }
    if (!requiresCan(runtime)) {
        return reject(FEEDBACK_READY_REJECT_CAN_GATE,
                      sharedAtomicLoad(&runtime->active_motor_mask),
                      sharedAtomicLoad(&runtime->active_joint_mask),
                      runtime->control_mode,
                      sharedAtomicLoad(&runtime->require_can_feedback));
    }
    if (!activeMotorsMultiTurnKnown(runtime)) {
        const uint32_t motorMask = requiredMotorMask(runtime);
        const uint32_t overflow =
            sharedAtomicLoad(&runtime->servo_turn_overflow_bitmap) & motorMask;
        return overflow != 0
            ? reject(FEEDBACK_READY_REJECT_SERVO_TURN_OVERFLOW,
                     motorMask, overflow)
            : reject(FEEDBACK_READY_REJECT_MULTITURN,
                     motorMask,
                     sharedAtomicLoad(&runtime->multi_turn_unknown_bitmap));
    }
    if (!activeMotorsAbsLimitsCalibrated(runtime)) {
        return reject(FEEDBACK_READY_REJECT_CALIBRATION_MISSING);
    }

    const uint32_t activeMask =
        sharedAtomicLoad(&runtime->active_joint_mask) &
        kManualEncoderReferenceFullMask;
    if (activeMask == 0) {
        return reject(FEEDBACK_READY_REJECT_EMPTY_SCOPE,
                      sharedAtomicLoad(&runtime->active_motor_mask));
    }

    RemoteSensorData_t sensor = {};
    const uint32_t onlineMask =
        sharedAtomicLoad(&runtime->encoder_online_bitmap);
    const uint32_t faultMask =
        sharedAtomicLoad(&runtime->encoder_fault_bitmap);
    const uint8_t canOnline = sharedAtomicLoad(&runtime->can_link_online);
    const bool sensorFresh = runtime->canRxQueue.peek(&sensor) && sensor.isValid &&
        (uint32_t)(nowMs - sensor.timestamp) <= kStateCanOfflineTimeoutMs;
    if (!sensorFresh || canOnline == 0 ||
        (activeMask & ~onlineMask) != 0 ||
        (activeMask & faultMask) != 0) {
        return reject(FEEDBACK_READY_REJECT_ENCODER_CAN,
                      activeMask & ~onlineMask,
                      activeMask & faultMask,
                      sensorFresh ? 1 : 0,
                      canOnline);
    }
    const uint32_t faults = p4::safety::collectSystemFaults(runtime);
    if (faults != 0) {
        return reject(FEEDBACK_READY_REJECT_SYSTEM_FAULT, faults);
    }

    MappedAngleData_t mapped = {};
    if (!runtime->mappedAngleQueue.valid()) {
        return reject(FEEDBACK_READY_REJECT_MAPPED_UNAVAILABLE, 0);
    }
    if (!runtime->mappedAngleQueue.peek(&mapped)) {
        return reject(FEEDBACK_READY_REJECT_MAPPED_UNAVAILABLE, 1);
    }
    const uint32_t mappedAgeMs = nowMs - mapped.timestamp;
    if (!mapped.isValid) {
        return reject(FEEDBACK_READY_REJECT_MAPPED_UNAVAILABLE,
                      2, 0, static_cast<int32_t>(mappedAgeMs));
    }
    if (mappedAgeMs > kStateCanOfflineTimeoutMs) {
        return reject(FEEDBACK_READY_REJECT_MAPPED_UNAVAILABLE,
                      3, 0, static_cast<int32_t>(mappedAgeMs));
    }

    const bool newSample = mapped.timestamp != s_workspaceLastMappedTimestamp;
    if (newSample) {
        s_workspaceLastMappedTimestamp = mapped.timestamp;
    }
    uint32_t recoverableMask = 0;
    FeedbackReadinessDecision firstReject = {};
    bool hasReject = false;

    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; ++joint) {
        const uint32_t bit = 1UL << joint;
        if ((activeMask & bit) == 0) {
            continue;
        }
        JointCalibrationRuntimeSnapshot snapshot;
        if (!readJointCalibrationRuntimeSnapshot(joint, &snapshot)) {
            return reject(FEEDBACK_READY_REJECT_CALIBRATION_MISSING,
                          joint, activeMask);
        }
        const JointCalibrationResult& calib = snapshot.result;
        if (!calib.success) {
            return reject(FEEDBACK_READY_REJECT_CALIBRATION_MISSING,
                          joint, activeMask);
        }
        const uint32_t manualCrc = manualEncoderReferenceEntryCrc(
            joint, snapshot.manual);
        if (calib.mappingVersion != kManualScaledUrdfFeedbackMappingVersion ||
            calib.urdfConfigCrc != kManualScaledUrdfConfigCrc ||
            calib.manualReferenceCrc != manualCrc) {
            return reject(FEEDBACK_READY_REJECT_CALIBRATION_MISMATCH,
                          joint,
                          calib.mappingVersion,
                          static_cast<int32_t>(calib.urdfConfigCrc),
                          static_cast<int32_t>(manualCrc));
        }
        if (mapped.validFlags[joint] == 0) {
            uint32_t validMask = 0;
            for (uint8_t i = 0; i < ENCODER_TOTAL_NUM; ++i) {
                if (mapped.validFlags[i] != 0) {
                    validMask |= 1UL << i;
                }
            }
            return reject(FEEDBACK_READY_REJECT_MAPPED_INVALID,
                          joint, validMask);
        }

        const bool calibrationChanged =
            s_workspaceAutoMinRaw[joint] != calib.autoMinRaw ||
            s_workspaceAutoMaxRaw[joint] != calib.autoMaxRaw ||
            s_workspaceManualCrc[joint] != calib.manualReferenceCrc ||
            s_workspaceMappingVersion[joint] != calib.mappingVersion;
        if (calibrationChanged) {
            s_workspaceReadyLatchMask &= ~bit;
            s_workspaceEnterStable[joint] = 0;
            s_workspaceExitStable[joint] = 0;
            s_workspaceAutoMinRaw[joint] = calib.autoMinRaw;
            s_workspaceAutoMaxRaw[joint] = calib.autoMaxRaw;
            s_workspaceManualCrc[joint] = calib.manualReferenceCrc;
            s_workspaceMappingVersion[joint] = calib.mappingVersion;
        }

        const int32_t outside = mapped.outsideCounts[joint];
        if (newSample) {
            if ((s_workspaceReadyLatchMask & bit) != 0) {
                s_workspaceEnterStable[joint] = 0;
                if (outside < -kReadyExitOutsideToleranceCounts) {
                    if (s_workspaceExitStable[joint] < 255) {
                        ++s_workspaceExitStable[joint];
                    }
                    if (s_workspaceExitStable[joint] >=
                        kWorkspaceStableSampleCount) {
                        s_workspaceReadyLatchMask &= ~bit;
                    }
                } else {
                    s_workspaceExitStable[joint] = 0;
                }
            } else {
                s_workspaceExitStable[joint] = 0;
                if (outside >= -kFeedbackReadyEnterOutsideToleranceCounts) {
                    if (s_workspaceEnterStable[joint] < 255) {
                        ++s_workspaceEnterStable[joint];
                    }
                    if (s_workspaceEnterStable[joint] >=
                        kWorkspaceStableSampleCount) {
                        s_workspaceReadyLatchMask |= bit;
                    }
                } else {
                    s_workspaceEnterStable[joint] = 0;
                }
            }
        }

        if ((s_workspaceReadyLatchMask & bit) == 0) {
            if (outside < -kFeedbackReadyEnterOutsideToleranceCounts) {
                recoverableMask |= bit;
                if (!hasReject) {
                    firstReject = reject(FEEDBACK_READY_REJECT_BELOW_WORKSPACE,
                                         joint,
                                         0,
                                         outside,
                                         kFeedbackReadyEnterOutsideToleranceCounts);
                    hasReject = true;
                }
            }
        }
    }

    if (hasReject || (s_workspaceReadyLatchMask & activeMask) != activeMask) {
        if (!hasReject) {
            firstReject = reject(FEEDBACK_READY_REJECT_BELOW_WORKSPACE);
        }
        firstReject.recoverableBelowMask = recoverableMask;
        return firstReject;
    }

    FeedbackReadinessDecision ready = {};
    ready.ready = true;
    return ready;
}

void markActiveJointsWorkspaceReady(SystemRuntimeContext* runtime)
{
    if (!runtime) {
        return;
    }
    const uint32_t activeMask =
        sharedAtomicLoad(&runtime->active_joint_mask) &
        kManualEncoderReferenceFullMask;
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; ++joint) {
        const uint32_t bit = 1UL << joint;
        if ((activeMask & bit) == 0) {
            continue;
        }
        JointCalibrationRuntimeSnapshot snapshot;
        if (!readJointCalibrationRuntimeSnapshot(joint, &snapshot)) {
            s_workspaceReadyLatchMask &= ~bit;
            continue;
        }
        const JointCalibrationResult& calib = snapshot.result;
        s_workspaceAutoMinRaw[joint] = calib.autoMinRaw;
        s_workspaceAutoMaxRaw[joint] = calib.autoMaxRaw;
        s_workspaceManualCrc[joint] = calib.manualReferenceCrc;
        s_workspaceMappingVersion[joint] = calib.mappingVersion;
        s_workspaceEnterStable[joint] = kWorkspaceStableSampleCount;
        s_workspaceExitStable[joint] = 0;
        s_workspaceReadyLatchMask |= bit;
    }
}

} // namespace p4::state
