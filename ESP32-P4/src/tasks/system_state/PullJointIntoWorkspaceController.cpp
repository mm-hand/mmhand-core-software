/**
 * @file PullJointIntoWorkspaceController.cpp
 * @brief 以非阻塞状态机实现 active scope 从 IDLE 正向进入 READY 工作区。
 * @details 本模块复用统一舵机目标邮箱，不再占用独立 FreeRTOS Task 和 8K 栈。
 */
#include "PullJointIntoWorkspaceController.h"

#include <string.h>

#include "../../system/ActuatorCommandGateway.h"
#include "../../system/calibration/CalibrationMath.h"
#include "../../system/calibration/ManualEncoderReferenceStore.h"
#include "../../system/calibration/TendonCalibrationLut.h"
#include "../../control/ControlParameters.h"
#include "../../hal/servo/ServoValue.h"
#include "../../config/topology/HardwareMap.h"
#include "../../system/rtos/SharedSafetyState.h"
#include "../../safety/SafetyEventReporter.h"
#include "../../safety/SafetyRuleRegistry.h"
#include "../../safety/policies/PullJointIntoWorkspaceSafetyPolicy.h"
#include "../../safety/policies/SystemReadinessPolicy.h"
#include "../../system/log/FirmwareLog.h"
#include "../../system/ports/SystemEventPort.h"

namespace p4::recovery {
namespace {

inline constexpr uint32_t kStepPeriodMs = 50;       ///< 保持原 Pull Joint into Workspace 控制频率。
inline constexpr int32_t kTightenStepCounts = 60;  ///< 主执行器每步收绳量。
inline constexpr int32_t kReleaseStepCounts = 30;  ///< 双腱副执行器每步放绳量。
inline constexpr int32_t kDualTightenStepCounts = 30; ///< 越界阶段双腱同步收/放各 30 counts。
inline constexpr int32_t kDualTensionStepCounts = 30; ///< 入区后每步收绳 30 counts（600 counts/s）。
inline constexpr int16_t kDualTargetLoad = 250;
inline constexpr uint32_t kDualWorkspaceTimeoutMs = 6000;
inline constexpr uint32_t kDualTensionTimeoutMs = 4000;
inline constexpr int32_t kDualWorkspaceEnterMarginCounts = 24; ///< 进入工作区内部的深度，与 READY 入口容差一致。
inline constexpr uint8_t kInsideStableSamples = 3; ///< READY 连续确认帧数。

bool appendMotorTarget(ServoTargetBatch_t* batch,
                       uint8_t motorChannel,
                       int32_t targetAbs)
{
    if (!batch || batch->count >= SERVO_TARGET_BATCH_MAX ||
        !isMotorChannelValid(motorChannel)) {
        return false;
    }
    ServoTargetCommand_t& command = batch->commands[batch->count++];
    command.motorChannel = motorChannel;
    command.position = targetAbs;
    command.speed = SERVO_TARGET_SPEED_DEFAULT;
    command.acc = SERVO_TARGET_ACC_DEFAULT;
    return true;
}

/** 统一提交最终状态、安全事件和内部完成事件，确保一个会话只结束一次。 */
void finish(SystemRuntimeContext* sharedData,
            bool success,
            uint8_t reason,
            uint32_t nowMs)
{
    if (!sharedData || sharedData->pullJointIntoWorkspaceRuntime.active == 0) return;
    sharedData->pullJointIntoWorkspaceRuntime.active = 0;
    {
        SharedSafetyGuard guard;
        sharedAtomicStore(&sharedData->pull_joint_into_workspace_status,
                          success ? PULL_JOINT_INTO_WORKSPACE_STATUS_SUCCESS
                                  : PULL_JOINT_INTO_WORKSPACE_STATUS_FAILED);
        sharedAtomicStore(&sharedData->pull_joint_into_workspace_reason,
                          success ? PULL_JOINT_INTO_WORKSPACE_REASON_NONE : reason);
        sharedAtomicStore(&sharedData->pull_joint_into_workspace_timestamp, nowMs);
    }

    if (!success) {
        const SafetyRuleId ruleId = p4::safety::ruleForPullJointIntoWorkspaceReason(reason);
        if (ruleId != SafetyRuleId::None) {
            p4::safety::reportSafetyRuleEvent(sharedData,
                                              ruleId,
                                              reason,
                                              0);
        }
    }
    postSystemEvent(sharedData,
                    success ? SYSTEM_EVENT_PULL_JOINT_INTO_WORKSPACE_DONE
                            : SYSTEM_EVENT_PULL_JOINT_INTO_WORKSPACE_FAILED);
}

} // namespace

bool startPullJointIntoWorkspaceController(SystemRuntimeContext* sharedData, uint32_t nowMs)
{
    if (!sharedData || sharedData->pullJointIntoWorkspaceRuntime.active != 0) return false;
    PullJointIntoWorkspaceRuntimeState& runtime = sharedData->pullJointIntoWorkspaceRuntime;
    runtime.active = 1;
    runtime.insideStableSamples = 0;
    runtime.beginMs = nowMs;
    runtime.nextStepMs = nowMs;
    memset(runtime.dualPhase, 0, sizeof(runtime.dualPhase));
    memset(runtime.dualPhaseBeginMs, 0, sizeof(runtime.dualPhaseBeginMs));
    return true;
}

void cancelPullJointIntoWorkspaceController(SystemRuntimeContext* sharedData)
{
    if (!sharedData) return;
    sharedData->pullJointIntoWorkspaceRuntime.active = 0;
    sharedData->pullJointIntoWorkspaceRuntime.insideStableSamples = 0;
}

void stepPullJointIntoWorkspaceController(SystemRuntimeContext* sharedData, uint32_t nowMs)
{
    if (!sharedData || sharedData->pullJointIntoWorkspaceRuntime.active == 0) return;
    PullJointIntoWorkspaceRuntimeState& runtime = sharedData->pullJointIntoWorkspaceRuntime;
    if ((int32_t)(nowMs - runtime.nextStepMs) < 0) return;
    runtime.nextStepMs = nowMs + kStepPeriodMs;

    // Policy 只判断会话是否仍持有控制权；Controller 负责取消或结束。
    const p4::safety::PullJointIntoWorkspaceSessionDecision session =
        p4::safety::evaluatePullJointIntoWorkspaceSession(sharedData,
                                            nowMs,
                                            runtime.beginMs);
    if (!session.active) {
        cancelPullJointIntoWorkspaceController(sharedData);
        return;
    }
    if (session.reason != PULL_JOINT_INTO_WORKSPACE_REASON_NONE) {
        FW_LOG_INFO_PRINTF("[PULL JOINT INTO WORKSPACE] session rejected reason=%u\r\n",
                           (unsigned)session.reason);
        finish(sharedData, false, session.reason, nowMs);
        return;
    }

    const uint32_t activeJointMask =
        sharedAtomicLoad(&sharedData->active_joint_mask) &
        kManualEncoderReferenceFullMask;
    MappedAngleData_t mapped = {};
    ServoAngleData_t servo = {};
    const uint8_t feedbackReason =
        p4::safety::readPullJointIntoWorkspaceFeedback(sharedData,
                                         nowMs,
                                         &mapped,
                                         &servo);
    if (activeJointMask == 0 || feedbackReason != PULL_JOINT_INTO_WORKSPACE_REASON_NONE) {
        FW_LOG_INFO_PRINTLN("[PULL JOINT INTO WORKSPACE] feedback unavailable");
        finish(sharedData,
               false,
               feedbackReason != PULL_JOINT_INTO_WORKSPACE_REASON_NONE
                   ? feedbackReason
                   : PULL_JOINT_INTO_WORKSPACE_REASON_MANUAL_REQUIRED,
               nowMs);
        return;
    }


    ServoTargetBatch_t batch = {};
    batch.source = SERVO_TARGET_OWNER_CALIBRATION;
    batch.timestamp = nowMs;
    bool allInside = true;
    uint32_t movedMotorMask = 0;

    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; joint++) {
        if ((activeJointMask & (1UL << joint)) == 0) continue;
        if (!p4::safety::pullJointIntoWorkspaceCalibrationMatchesManual(joint) ||
            mapped.validFlags[joint] == 0) {
            finish(sharedData, false, PULL_JOINT_INTO_WORKSPACE_REASON_AUTO_REQUIRED, nowMs);
            return;
        }
        const int32_t outside = mapped.outsideCounts[joint];
        JointActuatorGroup group = {};
        if (!getJointActuatorGroup(joint, &group) || group.count == 0) {
            finish(sharedData, false, PULL_JOINT_INTO_WORKSPACE_REASON_AUTO_REQUIRED, nowMs);
            return;
        }
        if (group.count != 2) {
            if (outside == 0) continue;
            allInside = false;
            const uint8_t motor = group.actuators[0].motorChannel;
            if (!servo.onlineStatus[motor] || (movedMotorMask & (1UL << motor))) {
                finish(sharedData, false, PULL_JOINT_INTO_WORKSPACE_REASON_SERVO_OFFLINE, nowMs);
                return;
            }
            const int32_t target = motorApplyTendonDelta(motor, servo.servoAngles[motor],
                outside < 0 ? kTightenStepCounts : -kReleaseStepCounts);
            if (!appendMotorTarget(&batch, motor, target)) {
                finish(sharedData, false, PULL_JOINT_INTO_WORKSPACE_REASON_TARGET_PUBLISH_FAILED, nowMs);
                return;
            }
            movedMotorMask |= 1UL << motor;
            continue;
        }
        auto& phase = runtime.dualPhase[joint];
        if (phase == 3 && outside != 0) {
            phase = 1;
            runtime.dualPhaseBeginMs[joint] = nowMs;
        }
        if (phase == 0 && outside != 0) {
            runtime.dualTightActuator[joint] = outside < 0 ? 0 : 1;
            phase = 1;
            runtime.dualPhaseBeginMs[joint] = nowMs;
        }
        if (phase == 0) continue;
        if (phase != 3) allInside = false;
        const uint8_t ti = runtime.dualTightActuator[joint];
        const uint8_t ri = 1 - ti;
        const uint8_t tight = group.actuators[ti].motorChannel;
        const uint8_t release = group.actuators[ri].motorChannel;
        if (!servo.onlineStatus[tight] || !servo.onlineStatus[release] ||
            (movedMotorMask & ((1UL << tight) | (1UL << release)))) {
            finish(sharedData, false, PULL_JOINT_INTO_WORKSPACE_REASON_SERVO_OFFLINE, nowMs);
            return;
        }
        ServoTelemetryData_t telemetry = {};
        const uint8_t bus = sharedData->servo_route_bus[tight];
        if (!sharedData->servoTelemetryQueue.peek(&telemetry) ||
            nowMs - telemetry.timestamp > 150 || !telemetry.onlineStatus[tight] ||
            bus >= NUM_BUSES || !(telemetry.cycle.freshBusMask & (1U << bus))) {
            finish(sharedData, false, PULL_JOINT_INTO_WORKSPACE_REASON_FEEDBACK_STALE, nowMs);
            return;
        }
        const int16_t load = mmhand::servo::loadMagnitude(telemetry.load[tight]);
        if (phase == 3) continue;
        const auto parameters = mmhand::control::readParameters();
        JointCalibrationRuntimeSnapshot calibration;
        if (!parameters.configured || !readJointCalibrationRuntimeSnapshot(joint, &calibration)) {
            finish(sharedData, false, PULL_JOINT_INTO_WORKSPACE_REASON_AUTO_REQUIRED, nowMs);
            return;
        }
        // 第一阶段把关节送进工作区内部后再切张紧，而不是贴着下界切换；
        // 0.53° 的进入深度与 P4 READY 的 24-count 入口容差一致。
        const float mappedDeg = (float)mapped.angleValues[joint] * (360.0f / 16384.0f);
        const float enterMarginDeg =
            (float)kDualWorkspaceEnterMarginCounts * (360.0f / 16384.0f);
        const bool belowWorkspace = runtime.dualTightActuator[joint] == 0;
        const bool enteredWorkspace = belowWorkspace
            ? mappedDeg >= calibration.result.angleMin + enterMarginDeg
            : mappedDeg <= calibration.result.angleMax - enterMarginDeg;
        if (phase == 1 && enteredWorkspace) {
            phase = 2;
            runtime.dualPhaseBeginMs[joint] = nowMs;
            FW_LOG_INFO_PRINTF(
                "[PULL JOINT INTO WORKSPACE] workspace entered J%02u M%02u mapped=%.2f pos=%ld\r\n",
                (unsigned)joint, (unsigned)tight, (double)mappedDeg,
                (long)servo.servoAngles[tight]);
        }
        if (phase == 1 && nowMs - runtime.dualPhaseBeginMs[joint] >= kDualWorkspaceTimeoutMs) {
            FW_LOG_INFO_PRINTF(
                "[PULL JOINT INTO WORKSPACE] workspace timeout J%02u M%02u pos=%ld outside=%ld\r\n",
                (unsigned)joint, (unsigned)tight,
                (long)servo.servoAngles[tight], (long)outside);
            finish(sharedData, false, PULL_JOINT_INTO_WORKSPACE_REASON_TIMEOUT, nowMs);
            return;
        }
        if (phase == 2 && nowMs - runtime.dualPhaseBeginMs[joint] >= kDualTensionTimeoutMs) {
            FW_LOG_INFO_PRINTF(
                "[PULL JOINT INTO WORKSPACE] tension timeout J%02u M%02u load=%d pos=%ld elapsed=%lu\r\n",
                (unsigned)joint, (unsigned)tight, (int)load,
                (long)servo.servoAngles[tight],
                (unsigned long)(nowMs - runtime.dualPhaseBeginMs[joint]));
            finish(sharedData, false, PULL_JOINT_INTO_WORKSPACE_REASON_TIMEOUT, nowMs);
            return;
        }
        if (phase == 2 && load >= kDualTargetLoad) {
            FW_LOG_INFO_PRINTF(
                "[PULL JOINT INTO WORKSPACE] tension reached J%02u M%02u load=%d pos=%ld elapsed=%lu\r\n",
                (unsigned)joint, (unsigned)tight, (int)load,
                (long)servo.servoAngles[tight],
                (unsigned long)(nowMs - runtime.dualPhaseBeginMs[joint]));
            if (!appendMotorTarget(&batch, tight, servo.servoAngles[tight])) {
                finish(sharedData, false, PULL_JOINT_INTO_WORKSPACE_REASON_TARGET_PUBLISH_FAILED, nowMs);
                return;
            }
            phase = 3;
            movedMotorMask |= 1UL << tight;
            continue;
        }
        const uint8_t actuator = ti;
        const uint8_t motor = group.actuators[actuator].motorChannel;
        const int32_t delta = phase == 1 ? kDualTightenStepCounts : kDualTensionStepCounts;
        const int32_t target = motorApplyTendonDelta(motor, servo.servoAngles[motor], delta);
        int32_t low = 0, high = 0;
        const bool hasRange = tendonCalibrationMotorAbsRangeWithMargins(
            calibration.result, actuator, motor,
            parameters[mmhand::control::OutputLutReleaseMarginCounts],
            parameters[mmhand::control::OutputLutTightMarginCounts], &low, &high);
        const int32_t current = servo.servoAngles[motor];
        // Recovering a slack tendon can start just outside the calibrated LUT
        // range; tightening toward the range is legal, so clamp the first
        // target onto the boundary instead of rejecting the whole session.
        const bool tighteningInward = (phase == 1 || phase == 2) &&
            ((current < low && target > current) ||
             (current > high && target < current));
        int32_t appliedTarget = target;
        if (tighteningInward) {
            // Only pull the target back onto the boundary when the step itself
            // is still outside; a normal +delta step must keep advancing.
            if (current < low && appliedTarget < low) appliedTarget = low;
            if (current > high && appliedTarget > high) appliedTarget = high;
        }
        if (!hasRange || ((appliedTarget < low || appliedTarget > high) && !tighteningInward)) {
            FW_LOG_INFO_PRINTF(
                "[PULL JOINT INTO WORKSPACE] LUT reject J%02u M%02u phase=%u current=%ld target=%ld range=[%ld,%ld]\r\n",
                (unsigned)joint, (unsigned)motor, (unsigned)phase,
                (long)current, (long)target, (long)low, (long)high);
            finish(sharedData, false, PULL_JOINT_INTO_WORKSPACE_REASON_TARGET_PUBLISH_FAILED, nowMs);
            return;
        }
        if (phase == 2) {
            FW_LOG_INFO_PRINTF(
                "[PULL JOINT INTO WORKSPACE] tension J%02u M%02u load=%d pos=%ld target=%ld elapsed=%lu\r\n",
                (unsigned)joint, (unsigned)motor, (int)load, (long)current,
                (long)appliedTarget,
                (unsigned long)(nowMs - runtime.dualPhaseBeginMs[joint]));
        }
        if (!appendMotorTarget(&batch, motor, appliedTarget)) {
            finish(sharedData, false, PULL_JOINT_INTO_WORKSPACE_REASON_TARGET_PUBLISH_FAILED, nowMs);
            return;
        }
        movedMotorMask |= 1UL << motor;
        if (phase == 1) {
            const int32_t releaseTarget = motorApplyTendonDelta(release,
                servo.servoAngles[release], -kReleaseStepCounts);
            int32_t releaseLow = 0, releaseHigh = 0;
            const bool releaseRange = tendonCalibrationMotorAbsRangeWithMargins(
                calibration.result, ri, release,
                parameters[mmhand::control::OutputLutReleaseMarginCounts],
                parameters[mmhand::control::OutputLutTightMarginCounts],
                &releaseLow, &releaseHigh);
            if (!releaseRange || releaseTarget < releaseLow || releaseTarget > releaseHigh) {
                FW_LOG_INFO_PRINTF(
                    "[PULL JOINT INTO WORKSPACE] LUT reject J%02u M%02u release target=%ld range=[%ld,%ld]\r\n",
                    (unsigned)joint, (unsigned)release, (long)releaseTarget,
                    (long)releaseLow, (long)releaseHigh);
                finish(sharedData, false, PULL_JOINT_INTO_WORKSPACE_REASON_TARGET_PUBLISH_FAILED, nowMs);
                return;
            }
            if (!appendMotorTarget(&batch, release, releaseTarget)) {
                finish(sharedData, false, PULL_JOINT_INTO_WORKSPACE_REASON_TARGET_PUBLISH_FAILED, nowMs);
                return;
            }
            movedMotorMask |= 1UL << release;
        }
    }

    if (allInside) {
        if (runtime.insideStableSamples < 255) runtime.insideStableSamples++;
        if (runtime.insideStableSamples >= kInsideStableSamples) {
            FW_LOG_INFO_PRINTF("[PULL JOINT INTO WORKSPACE] complete joints=0x%08lX\r\n",
                               (unsigned long)activeJointMask);
            finish(sharedData, true, PULL_JOINT_INTO_WORKSPACE_REASON_NONE, nowMs);
        }
        return;
    }

    runtime.insideStableSamples = 0;
    if (batch.count == 0 || !p4::actuator::publishLatest(sharedData, batch)) {
        finish(sharedData,
               false,
               PULL_JOINT_INTO_WORKSPACE_REASON_TARGET_PUBLISH_FAILED,
               nowMs);
    }
}

uint8_t pullJointIntoWorkspaceActiveScopeRejectReason(SystemRuntimeContext* sharedData)
{
    return p4::safety::evaluatePullJointIntoWorkspaceActiveScope(sharedData);
}

} // namespace p4::recovery
