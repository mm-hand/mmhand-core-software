#include "RopeInitTask.h"

#include <string.h>

#include "CalibrationMath.h"
#include "CalibrationTask.h"
#include "../generated/UrdfJointLimits.generated.h"
#include "../hardware/HardwareMap.h"
#include "../persistence/ManualCalibrationNvsStore.h"
#include "../shared/SharedSafetyState.h"
#include "../system/FirmwareLog.h"
#include "../system/StateMachineTask.h"

namespace {

static TaskHandle_t g_ropeInitTaskHandle = NULL;
static const uint32_t kLoopMs = 50;
static const uint32_t kTimeoutMs = 8000;
static const uint32_t kFeedbackMaxAgeMs = 500;
static const int32_t kTightenStepCounts = 60;
static const int32_t kReleaseStepCounts = 30;
static const uint8_t kInsideStableSamples = 3;
static const int32_t kServoAbsMin = -30719;
static const int32_t kServoAbsMax = 30719;

static int32_t clampServoAbs(int32_t value)
{
    if (value < kServoAbsMin) return kServoAbsMin;
    if (value > kServoAbsMax) return kServoAbsMax;
    return value;
}

static bool calibrationMatchesManual(uint8_t jointIndex)
{
    if (jointIndex >= ENCODER_TOTAL_NUM) return false;
    const JointCalibrationResult& result = g_jointCalibResult[jointIndex];
    const uint32_t manualEntryCrc = manualEncoderReferenceEntryCrc(
        jointIndex, g_manualEncoderReference[jointIndex]);
    return result.success &&
           result.mappingVersion == kManualScaledUrdfFeedbackMappingVersion &&
           result.urdfConfigCrc == kManualScaledUrdfConfigCrc &&
           result.manualReferenceCrc == manualEntryCrc;
}

static bool appendMotorTarget(ServoTargetBatch_t* batch,
                              uint8_t motorChannel,
                              int32_t targetAbs)
{
    if (!batch || batch->count >= SERVO_TARGET_BATCH_MAX ||
        !isMotorChannelValid(motorChannel)) {
        return false;
    }
    const MotorMapItem& map = motorMap[motorChannel];
    ServoTargetCommand_t& command = batch->commands[batch->count++];
    command.busIndex = map.busIndex;
    command.servoId = map.servoID;
    command.position = (int16_t)clampServoAbs(targetAbs);
    command.speed = SERVO_TARGET_SPEED_DEFAULT;
    command.acc = SERVO_TARGET_ACC_DEFAULT;
    return true;
}

static bool taskStillOwnsControl(TaskSharedData_t* sharedData)
{
    return sharedData &&
           sharedData->system_state == SYSTEM_STATE_ROPE_INIT_RUNNING &&
           sharedData->servo_target_owner == SERVO_TARGET_OWNER_CALIBRATION &&
           sharedData->control_enabled == 0;
}

static void ropeInitTask(void* parameter)
{
    TaskSharedData_t* sharedData = static_cast<TaskSharedData_t*>(parameter);
    const uint32_t beginMs = millis();
    bool ok = sharedData != NULL;
    bool completed = false;
    uint8_t failureReason = ROPE_INIT_REASON_NONE;

    uint8_t insideStableSamples = 0;
    while (ok && taskStillOwnsControl(sharedData)) {
        const uint32_t nowMs = millis();
        const uint32_t systemFaults =
            sharedAtomicLoad(&sharedData->system_fault_bitmap) |
            sharedAtomicLoad(&sharedData->system_fault_latched_bitmap);
        if (systemFaults != 0) {
            FW_LOG_INFO_PRINTF("[ROPE INIT] system fault=0x%08lX\r\n",
                               (unsigned long)systemFaults);
            failureReason = ROPE_INIT_REASON_SYSTEM_FAULT;
            ok = false;
            break;
        }
        if ((uint32_t)(nowMs - beginMs) > kTimeoutMs) {
            FW_LOG_INFO_PRINTLN("[ROPE INIT] timeout");
            failureReason = ROPE_INIT_REASON_TIMEOUT;
            ok = false;
            break;
        }

        const uint32_t activeJointMask =
            sharedAtomicLoad(&sharedData->active_joint_mask) &
            kManualEncoderReferenceFullMask;
        MappedAngleData_t mapped;
        ServoAngleData_t servo;
        if (activeJointMask == 0 || !sharedData->mappedAngleQueue ||
            !sharedData->servoFeedbackQueue ||
            xQueuePeek(sharedData->mappedAngleQueue, &mapped, 0) != pdTRUE ||
            xQueuePeek(sharedData->servoFeedbackQueue, &servo, 0) != pdTRUE ||
            !mapped.isValid ||
            (uint32_t)(nowMs - mapped.timestamp) > kFeedbackMaxAgeMs ||
            (uint32_t)(nowMs - servo.timestamp) > kFeedbackMaxAgeMs) {
            FW_LOG_INFO_PRINTLN("[ROPE INIT] feedback unavailable");
            failureReason = ROPE_INIT_REASON_FEEDBACK_STALE;
            ok = false;
            break;
        }

        ServoTargetBatch_t batch;
        memset(&batch, 0, sizeof(batch));
        batch.source = SERVO_TARGET_OWNER_CALIBRATION;
        batch.timestamp = nowMs;
        bool allInside = true;
        uint32_t movedMotorMask = 0;

        for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; joint++) {
            if ((activeJointMask & (1UL << joint)) == 0) continue;
            if (!calibrationMatchesManual(joint) ||
                mapped.validFlags[joint] == 0) {
                FW_LOG_INFO_PRINTF("[ROPE INIT] J%02u missing v3 calibration\r\n",
                                   (unsigned)joint);
                failureReason = ROPE_INIT_REASON_AUTO_REQUIRED;
                ok = false;
                break;
            }
            if (mapped.outsideCounts[joint] >=
                -kFeedbackReadyEnterOutsideToleranceCounts) {
                continue;
            }

            allInside = false;
            JointActuatorGroup group;
            if (!getJointActuatorGroup(joint, &group) || group.count == 0) {
                failureReason = ROPE_INIT_REASON_AUTO_REQUIRED;
                ok = false;
                break;
            }
            for (uint8_t actuator = 0; actuator < group.count; actuator++) {
                const uint8_t motor = group.actuators[actuator].motorChannel;
                if ((movedMotorMask & (1UL << motor)) != 0 ||
                    servo.onlineStatus[motor] == 0) {
                    failureReason = ROPE_INIT_REASON_SERVO_OFFLINE;
                    ok = false;
                    break;
                }
                const int32_t tendonDelta = actuator == 0
                    ? kTightenStepCounts : -kReleaseStepCounts;
                const int32_t target = motorApplyTendonDelta(
                    motor, servo.servoAngles[motor], tendonDelta);
                if (!appendMotorTarget(&batch, motor, target)) {
                    failureReason = ROPE_INIT_REASON_TARGET_PUBLISH_FAILED;
                    ok = false;
                    break;
                }
                movedMotorMask |= (1UL << motor);
            }
            if (!ok) break;
        }

        if (!ok) break;
        if (allInside) {
            if (insideStableSamples < 255) insideStableSamples++;
            FW_LOG_INFO_PRINTF(
                "[ROPE INIT] inside stable=%u/%u joints=0x%08lX\r\n",
                (unsigned)insideStableSamples,
                (unsigned)kInsideStableSamples,
                (unsigned long)activeJointMask);
            if (insideStableSamples >= kInsideStableSamples) {
                FW_LOG_INFO_PRINTF(
                    "[ROPE INIT] complete joints=0x%08lX\r\n",
                    (unsigned long)activeJointMask);
                completed = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(kLoopMs));
            continue;
        }
        insideStableSamples = 0;
        if (batch.count == 0 ||
            xQueueOverwrite(sharedData->servoTargetQueue, &batch) != pdTRUE) {
            FW_LOG_INFO_PRINTLN("[ROPE INIT] target publish failed");
            failureReason = ROPE_INIT_REASON_TARGET_PUBLISH_FAILED;
            ok = false;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(kLoopMs));
    }

    if (ok && !completed) {
        ok = false;
        failureReason = ROPE_INIT_REASON_CANCELLED;
    }
    if (sharedData) {
        if (sharedData->rope_init_status == ROPE_INIT_STATUS_FAILED &&
            sharedData->rope_init_reason == ROPE_INIT_REASON_SYSTEM_FAULT) {
            ok = false;
            failureReason = ROPE_INIT_REASON_SYSTEM_FAULT;
        }
        sharedData->rope_init_status = ok
            ? ROPE_INIT_STATUS_SUCCESS
            : ROPE_INIT_STATUS_FAILED;
        sharedData->rope_init_reason = ok
            ? ROPE_INIT_REASON_NONE
            : failureReason;
        sharedData->rope_init_timestamp = millis();
    }
    postSystemEvent(sharedData,
                    ok ? SYSTEM_EVENT_ROPE_INIT_DONE
                       : SYSTEM_EVENT_ROPE_INIT_FAILED);
    g_ropeInitTaskHandle = NULL;
    vTaskDelete(NULL);
}

} // namespace

bool ropeInitActiveScopeReady(TaskSharedData_t* sharedData)
{
    return ropeInitActiveScopeRejectReason(sharedData) == ROPE_INIT_REASON_NONE;
}

uint8_t ropeInitActiveScopeRejectReason(TaskSharedData_t* sharedData)
{
    if (!sharedData || !sharedData->manual_reference_nvs_valid) {
        return ROPE_INIT_REASON_MANUAL_REQUIRED;
    }
    const uint32_t activeMask =
        sharedAtomicLoad(&sharedData->active_joint_mask) &
        kManualEncoderReferenceFullMask;
    if (activeMask == 0 || !manualEncoderReferencesCoverActiveMask(activeMask)) {
        return ROPE_INIT_REASON_MANUAL_REQUIRED;
    }
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; joint++) {
        if ((activeMask & (1UL << joint)) != 0 &&
            !calibrationMatchesManual(joint)) {
            return ROPE_INIT_REASON_AUTO_REQUIRED;
        }
    }
    return ROPE_INIT_REASON_NONE;
}

bool startRopeInitTask(TaskSharedData_t* sharedData)
{
    if (!sharedData || g_ropeInitTaskHandle != NULL) return false;
    return xTaskCreate(ropeInitTask,
                       "ropeInitTask",
                       AUTO_CALIB_TASK_STACK_SIZE,
                       sharedData,
                       TASK_AUTO_CALIB_PRIORITY,
                       &g_ropeInitTaskHandle) == pdPASS;
}
