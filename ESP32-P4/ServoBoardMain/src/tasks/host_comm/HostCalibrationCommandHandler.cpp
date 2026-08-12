/**
 * @file HostCalibrationCommandHandler.cpp
 * @brief 实现 Manual 标定维护、局部 Auto 失效和 Reset Zero 结果联动。
 */
#include "HostCalibrationCommandHandler.h"

#include <string.h>

#include "../../modules/calibration/CalibrationRuntimeStore.h"
#include "../../modules/persistence/ManualCalibrationNvsStore.h"
#include "../../modules/protocol/HostProtocol.h"
#include "../../platform/time/Clock.h"
#include "../../config/topology/HardwareMap.h"
#include "../../data/SafetyTypes.h"
#include "../../generated/UrdfJointLimits.generated.h"
#include "../../platform/rtos/AtomicValue.h"
#include "../../platform/rtos/SharedSafetyState.h"
#include "../../app/AppBootstrap.h"
#include "../../platform/log/FirmwareLog.h"
#include "../../runtime/SystemRuntimeContext.h"
#include "../../runtime/ports/SystemEventPort.h"

uint32_t autoCalibrationValidMask(SystemRuntimeContext* sharedData)
{
    if (!sharedData) {
        return 0;
    }
    uint32_t mask = 0;
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; joint++) {
        const JointCalibrationResult& result = g_jointCalibResult[joint];
        const uint32_t manualEntryCrc = manualEncoderReferenceEntryCrc(
            joint, g_manualEncoderReference[joint]);
        if (g_manualEncoderReference[joint].valid &&
            result.success &&
            result.mappingVersion == kManualScaledUrdfFeedbackMappingVersion &&
            result.urdfConfigCrc == kManualScaledUrdfConfigCrc &&
            result.manualReferenceCrc == manualEntryCrc) {
            mask |= (1UL << joint);
        }
    }
    return mask;
}

bool commandRequiresManualReference(uint8_t command)
{
    switch (command) {
        case CMD_CALIBRATE:
        case CMD_RUN_FEEDBACK:
        case CMD_ANGLE_CTRL:
        case CMD_ANGLE_CTRL_MASKED:
            return true;
        default:
            return false;
    }
}

bool manualUpdateMotionActive(SystemRuntimeContext* sharedData)
{
    return !sharedData ||
           sharedData->control_enabled != 0 ||
           sharedData->servo_target_owner != SERVO_TARGET_OWNER_NONE ||
           sharedData->system_state == SYSTEM_STATE_RUN_DIRECT ||
           sharedData->system_state == SYSTEM_STATE_RUN_FEEDBACK ||
           sharedData->system_state == SYSTEM_STATE_CALIBRATION_RUNNING ||
           sharedData->system_state == SYSTEM_STATE_ROPE_INIT_RUNNING ||
           sharedData->system_state == SYSTEM_STATE_PAUSED;
}

static uint32_t changedManualReferenceMask(
    const ManualEncoderReference before[ENCODER_TOTAL_NUM],
    const ManualEncoderReference after[ENCODER_TOTAL_NUM])
{
    uint32_t changedMask = 0;
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; joint++) {
        if (before[joint].valid != after[joint].valid ||
            before[joint].minRaw != after[joint].minRaw ||
            before[joint].maxRaw != after[joint].maxRaw) {
            changedMask |= (1UL << joint);
        }
    }
    return changedMask & kManualEncoderReferenceFullMask;
}

static uint32_t motorMaskForJoint(uint8_t jointIndex)
{
    if (jointIndex >= ENCODER_TOTAL_NUM) {
        return 0;
    }
    uint32_t motorMask = 0;
    JointActuatorGroup group;
    if (getJointActuatorGroup(jointIndex, &group)) {
        for (uint8_t actuator = 0; actuator < group.count; actuator++) {
            const uint8_t motor = group.actuators[actuator].motorChannel;
            if (motor < SERVO_TOTAL_NUM && isMotorChannelValid(motor)) {
                motorMask |= (1UL << motor);
            }
        }
    } else {
        const int primaryMotor = findPrimaryMotorChannelForJoint(jointIndex);
        if (primaryMotor >= 0 && primaryMotor < SERVO_TOTAL_NUM) {
            motorMask |= (1UL << (uint8_t)primaryMotor);
        }
    }
    return motorMask;
}

uint32_t jointMaskForMotorMask(uint32_t motorMask)
{
    motorMask &= validMotorChannelMask();
    uint32_t jointMask = 0;
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; joint++) {
        if ((motorMaskForJoint(joint) & motorMask) != 0) {
            jointMask |= (1UL << joint);
        }
    }
    return jointMask & kManualEncoderReferenceFullMask;
}

void invalidateAutomaticCalibration(SystemRuntimeContext* sharedData,
                                    uint32_t jointMask)
{
    if (!sharedData) {
        return;
    }
    jointMask &= kManualEncoderReferenceFullMask;
    uint32_t motorMask = 0;
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; joint++) {
        if ((jointMask & (1UL << joint)) == 0) {
            continue;
        }
        memset(&g_jointCalibResult[joint], 0,
               sizeof(g_jointCalibResult[joint]));
        motorMask |= motorMaskForJoint(joint);
    }

    if (motorMask != 0) {
        SharedSafetyGuard guard;
        sharedAtomicFetchAnd(&sharedData->motor_abs_limit_valid_bitmap,
                             ~motorMask);
        sharedAtomicFetchAnd(&sharedData->motor_abs_limit_locked_bitmap,
                             ~motorMask);
        for (uint8_t motor = 0; motor < SERVO_TOTAL_NUM; motor++) {
            if ((motorMask & (1UL << motor)) == 0) {
                continue;
            }
            sharedData->motor_abs_limit_min[motor] = 0;
            sharedData->motor_abs_limit_max[motor] = 0;
        }
    }

    clearCalibrationAssemblyWarnings(sharedData, jointMask);
    const bool anyAutoValid = autoCalibrationValidMask(sharedData) != 0;
    g_calibrationStatus = anyAutoValid
        ? (sharedAtomicLoad(&sharedData->calibration_warning_bitmap) != 0
               ? CALIB_STATUS_WARN
               : CALIB_STATUS_SUCCESS)
        : CALIB_STATUS_IDLE;
    sharedAtomicFetchAnd(&sharedData->system_fault_bitmap,
                         (uint32_t)~SYSTEM_FAULT_CALIBRATION);
    FW_LOG_INFO_PRINTF(
        "[NVS MANUAL] AUTO_INVALIDATE joints=0x%08lX motors=0x%08lX remaining=0x%08lX\r\n",
        (unsigned long)jointMask,
        (unsigned long)motorMask,
        (unsigned long)autoCalibrationValidMask(sharedData));
}

uint8_t commitManualReferences(
    SystemRuntimeContext* sharedData,
    const ManualEncoderReference references[ENCODER_TOTAL_NUM])
{
    uint32_t validMask = 0;
    manualEncoderReferencesValidate(references, &validMask);
    if (!sharedData || validMask == 0) {
        return PROTO_ACK_STATUS_INVALID_DATA;
    }

    if (!sharedData->manualNvsSaveQueue.valid()) {
        sharedData->manual_reference_save_status = MANUAL_SAVE_STATUS_FAILED;
        return PROTO_ACK_STATUS_NVS_FAILURE;
    }

    ManualNvsSaveJob job = {};
    job.requestToken = p4::platform::rtos::atomicAddFetch(
        &sharedData->manual_reference_save_request_token,
        static_cast<uint32_t>(1));
    job.validMask = validMask;
    job.queuedAtMs = p4::platform::time::monotonicMillis();
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; ++joint) {
        job.minRaw[joint] = references[joint].minRaw;
        job.maxRaw[joint] = references[joint].maxRaw;
    }
    // 维护命令必须逐项完成，禁止 overwrite 覆盖已经返回 QUEUED 的前一事务。
    if (!sharedData->manualNvsSaveQueue.send(job)) {
        sharedData->manual_reference_save_status = MANUAL_SAVE_STATUS_FAILED;
        return PROTO_ACK_STATUS_NVS_FAILURE;
    }
    // NONE 表示事务已受理但尚未收到 NVS 任务的读回校验结果。
    sharedData->manual_reference_save_status = MANUAL_SAVE_STATUS_NONE;
    return PROTO_ACK_STATUS_QUEUED;
}

bool processManualNvsSaveResult(SystemRuntimeContext* sharedData)
{
    if (!sharedData || !sharedData->manualNvsResultQueue.valid()) {
        return false;
    }
    ManualNvsSaveResult result = {};
    bool received = false;
    while (sharedData->manualNvsResultQueue.receive(&result)) {
        received = true;
    }
    if (!received ||
        result.requestToken != sharedAtomicLoad(
            &sharedData->manual_reference_save_request_token)) {
        return false;
    }
    if (!result.success) {
        // 写失败时保留上一份已验证 RAM/NVS 镜像，只更新本次事务状态。
        sharedData->manual_reference_save_status = MANUAL_SAVE_STATUS_FAILED;
        return true;
    }

    ManualEncoderReference references[ENCODER_TOTAL_NUM] = {};
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; ++joint) {
        references[joint].valid =
            (result.validMask & (1UL << joint)) != 0U ? 1U : 0U;
        references[joint].minRaw = result.minRaw[joint];
        references[joint].maxRaw = result.maxRaw[joint];
    }
    ManualEncoderReference previous[ENCODER_TOTAL_NUM] = {};
    {
        SharedSafetyGuard guard;
        memcpy(previous, g_manualEncoderReference, sizeof(previous));
    }
    const uint32_t changedMask =
        changedManualReferenceMask(previous, references);

    // valid 最后置位，作为其他任务读取整组 Manual RAM 镜像的提交点。
    sharedData->manual_reference_nvs_valid = 0;
    manualEncoderReferencesApplyRam(references);
    sharedAtomicStore(&sharedData->manual_reference_valid_bitmap,
                      result.validMask);
    sharedAtomicStore(&sharedData->manual_reference_crc,
                      result.referenceCrc);
    sharedAtomicStore(&sharedData->manual_reference_nvs_sequence,
                      result.sequence);
    invalidateAutomaticCalibration(sharedData, changedMask);
    sharedData->manual_reference_save_status = MANUAL_SAVE_STATUS_OK;
    sharedData->manual_reference_nvs_valid = 1;
    postSystemEvent(sharedData, SYSTEM_EVENT_REEVALUATE_READY);
    return true;
}

void processServoResetZeroResult(SystemRuntimeContext* sharedData,
                                 uint32_t* lastResultToken)
{
    if (!sharedData || !lastResultToken) {
        return;
    }
    const uint32_t resultToken =
        sharedAtomicLoad(&sharedData->servo_reset_zero_result_token);
    if (resultToken == 0 || resultToken == *lastResultToken) {
        return;
    }
    *lastResultToken = resultToken;

    // success mask 表示硬件 turn0 已改变，旧 LUT 即使 NVS 清除失败也必须失效。
    const uint32_t resetMotorMask =
        sharedAtomicLoad(&sharedData->servo_reset_zero_success_mask) &
        validMotorChannelMask();
    const uint32_t jointMask = jointMaskForMotorMask(resetMotorMask);
    sharedAtomicStore(&sharedData->servo_reset_zero_invalidated_joint_mask,
                      jointMask);
    if (jointMask != 0) {
        invalidateAutomaticCalibration(sharedData, jointMask);
        postSystemEvent(sharedData, SYSTEM_EVENT_REEVALUATE_READY);
    }
    FW_LOG_WARN_PRINTF(
        "[SERVO RESET ZERO] result token=%lu motors=0x%08lX failed=0x%08lX invalidatedJoints=0x%08lX status=%u\r\n",
        (unsigned long)resultToken,
        (unsigned long)resetMotorMask,
        (unsigned long)sharedAtomicLoad(
            &sharedData->servo_reset_zero_failed_mask),
        (unsigned long)jointMask,
        (unsigned)sharedAtomicLoad(&sharedData->servo_reset_zero_status));
}
