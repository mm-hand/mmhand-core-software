/**
 * @file RopeInitController.cpp
 * @brief 以非阻塞状态机实现 active scope 从 IDLE 正向进入 READY 工作区。
 * @details 本模块复用统一舵机目标邮箱，不再占用独立 FreeRTOS Task 和 8K 栈。
 */
#include "RopeInitController.h"

#include <string.h>

#include "../../system/ActuatorCommandGateway.h"
#include "../../system/calibration/CalibrationMath.h"
#include "../../system/calibration/ManualEncoderReferenceStore.h"
#include "../../config/topology/HardwareMap.h"
#include "../../system/rtos/SharedSafetyState.h"
#include "../../safety/SafetyEventReporter.h"
#include "../../safety/SafetyRuleRegistry.h"
#include "../../safety/policies/RopeInitSafetyPolicy.h"
#include "../../safety/policies/SystemReadinessPolicy.h"
#include "../../system/log/FirmwareLog.h"
#include "../../system/ports/SystemEventPort.h"

namespace p4::recovery {
namespace {

inline constexpr uint32_t kStepPeriodMs = 50;       ///< 保持原 Rope Init 控制频率。
inline constexpr int32_t kTightenStepCounts = 60;  ///< 主执行器每步收绳量。
inline constexpr int32_t kReleaseStepCounts = 30;  ///< 双腱副执行器每步放绳量。
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
    if (!sharedData || sharedData->ropeInitRuntime.active == 0) return;
    sharedData->ropeInitRuntime.active = 0;
    sharedAtomicStore(&sharedData->rope_init_status,
                      success ? ROPE_INIT_STATUS_SUCCESS
                              : ROPE_INIT_STATUS_FAILED);
    sharedAtomicStore(&sharedData->rope_init_reason,
                      success ? ROPE_INIT_REASON_NONE : reason);
    sharedAtomicStore(&sharedData->rope_init_timestamp, nowMs);

    if (!success) {
        const SafetyRuleId ruleId = p4::safety::ruleForRopeInitReason(reason);
        if (ruleId != SafetyRuleId::None) {
            p4::safety::reportSafetyRuleEvent(sharedData,
                                              ruleId,
                                              reason,
                                              0);
        }
    }
    postSystemEvent(sharedData,
                    success ? SYSTEM_EVENT_ROPE_INIT_DONE
                            : SYSTEM_EVENT_ROPE_INIT_FAILED);
}

} // namespace

bool startRopeInitController(SystemRuntimeContext* sharedData, uint32_t nowMs)
{
    if (!sharedData || sharedData->ropeInitRuntime.active != 0) return false;
    RopeInitRuntimeState& runtime = sharedData->ropeInitRuntime;
    runtime.active = 1;
    runtime.insideStableSamples = 0;
    runtime.beginMs = nowMs;
    runtime.nextStepMs = nowMs;
    return true;
}

void cancelRopeInitController(SystemRuntimeContext* sharedData)
{
    if (!sharedData) return;
    sharedData->ropeInitRuntime.active = 0;
    sharedData->ropeInitRuntime.insideStableSamples = 0;
}

void stepRopeInitController(SystemRuntimeContext* sharedData, uint32_t nowMs)
{
    if (!sharedData || sharedData->ropeInitRuntime.active == 0) return;
    RopeInitRuntimeState& runtime = sharedData->ropeInitRuntime;
    if ((int32_t)(nowMs - runtime.nextStepMs) < 0) return;
    runtime.nextStepMs = nowMs + kStepPeriodMs;

    // Policy 只判断会话是否仍持有控制权；Controller 负责取消或结束。
    const p4::safety::RopeInitSessionDecision session =
        p4::safety::evaluateRopeInitSession(sharedData,
                                            nowMs,
                                            runtime.beginMs);
    if (!session.active) {
        cancelRopeInitController(sharedData);
        return;
    }
    if (session.reason != ROPE_INIT_REASON_NONE) {
        FW_LOG_INFO_PRINTF("[ROPE INIT] session rejected reason=%u\r\n",
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
        p4::safety::readRopeInitFeedback(sharedData,
                                         nowMs,
                                         &mapped,
                                         &servo);
    if (activeJointMask == 0 || feedbackReason != ROPE_INIT_REASON_NONE) {
        FW_LOG_INFO_PRINTLN("[ROPE INIT] feedback unavailable");
        finish(sharedData,
               false,
               feedbackReason != ROPE_INIT_REASON_NONE
                   ? feedbackReason
                   : ROPE_INIT_REASON_MANUAL_REQUIRED,
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
        if (!p4::safety::ropeInitCalibrationMatchesManual(joint) ||
            mapped.validFlags[joint] == 0) {
            finish(sharedData, false, ROPE_INIT_REASON_AUTO_REQUIRED, nowMs);
            return;
        }
        // Enable 把“当前姿态”当保持目标下发，上位机要求角度 >= URDF 下限（0 容差）。
        // 所以回正必须收进下限里侧（outsideCounts 归零），不能像 READY 那样停在
        // −24 counts 的容差带里，否则回正完主机仍然会夹回。
        if (mapped.outsideCounts[joint] >= 0) {
            continue;
        }

        allInside = false;
        JointActuatorGroup group = {};
        if (!getJointActuatorGroup(joint, &group) || group.count == 0) {
            finish(sharedData, false, ROPE_INIT_REASON_AUTO_REQUIRED, nowMs);
            return;
        }
        for (uint8_t actuator = 0; actuator < group.count; actuator++) {
            const uint8_t motor = group.actuators[actuator].motorChannel;
            if ((movedMotorMask & (1UL << motor)) != 0 ||
                servo.onlineStatus[motor] == 0) {
                finish(sharedData, false, ROPE_INIT_REASON_SERVO_OFFLINE, nowMs);
                return;
            }
            const int32_t tendonDelta =
                actuator == 0 ? kTightenStepCounts : -kReleaseStepCounts;
            const int32_t target = motorApplyTendonDelta(
                motor, servo.servoAngles[motor], tendonDelta);
            if (!appendMotorTarget(&batch, motor, target)) {
                finish(sharedData,
                       false,
                       ROPE_INIT_REASON_TARGET_PUBLISH_FAILED,
                       nowMs);
                return;
            }
            movedMotorMask |= 1UL << motor;
        }
    }

    if (allInside) {
        if (runtime.insideStableSamples < 255) runtime.insideStableSamples++;
        if (runtime.insideStableSamples >= kInsideStableSamples) {
            FW_LOG_INFO_PRINTF("[ROPE INIT] complete joints=0x%08lX\r\n",
                               (unsigned long)activeJointMask);
            finish(sharedData, true, ROPE_INIT_REASON_NONE, nowMs);
        }
        return;
    }

    runtime.insideStableSamples = 0;
    if (batch.count == 0 || !p4::actuator::publishLatest(sharedData, batch)) {
        finish(sharedData,
               false,
               ROPE_INIT_REASON_TARGET_PUBLISH_FAILED,
               nowMs);
    }
}

uint8_t ropeInitActiveScopeRejectReason(SystemRuntimeContext* sharedData)
{
    return p4::safety::evaluateRopeInitActiveScope(sharedData);
}

} // namespace p4::recovery
