/**
 * @file HostCalibrationCommandHandler.cpp
 * @brief 实现 Manual 标定维护、局部 Auto 失效。
 */
#include "HostCalibrationCommandHandler.h"

#include <string.h>

#include "../../system/calibration/CalibrationRuntimeStore.h"
#include "../../system/calibration/ManualEncoderReferenceStore.h"
#include "../../config/protocol/HostProtocol.h"
#include "../../system/time/Clock.h"
#include "../../config/topology/HardwareMap.h"
#include "../../config/ServoConfig.h"
#include "../../data/SafetyTypes.h"
#include "../../generated/UrdfJointLimits.generated.h"
#include "../../system/rtos/AtomicValue.h"
#include "../../system/rtos/SharedSafetyState.h"
#include "../../system/log/FirmwareLog.h"
#include "../../system/SystemRuntimeContext.h"
#include "../../system/ports/SystemEventPort.h"

uint32_t autoCalibrationValidMask(SystemRuntimeContext* sharedData)
{
    if (!sharedData) {
        return 0;
    }
    uint32_t mask = 0;
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; joint++) {
        JointCalibrationRuntimeSnapshot snapshot;
        if (!readJointCalibrationRuntimeSnapshot(joint, &snapshot)) {
            continue;
        }
        const JointCalibrationResult& result = snapshot.result;
        const uint32_t manualEntryCrc = manualEncoderReferenceEntryCrc(
            joint, snapshot.manual);
        if (snapshot.manual.valid &&
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
        case CMD_RUN_FEEDBACK:
        case CMD_ANGLE_CTRL:
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
           sharedData->system_state == SYSTEM_STATE_PULL_JOINT_INTO_WORKSPACE_RUNNING;
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
        clearJointCalibrationResult(joint);
        motorMask |= motorMaskForJoint(joint);
    }

    if (motorMask != 0) {
        SharedSafetyGuard guard;
        sharedAtomicFetchAnd(&sharedData->motor_abs_limit_valid_bitmap,
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
    FW_LOG_INFO_PRINTF(
        "[MANUAL RAM] AUTO_INVALIDATE joints=0x%08lX motors=0x%08lX remaining=0x%08lX\r\n",
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

    ManualEncoderReference previous[ENCODER_TOTAL_NUM] = {};
    {
        SharedSafetyGuard guard;
        memcpy(previous, g_manualEncoderReference, sizeof(previous));
    }
    const uint32_t changedMask =
        changedManualReferenceMask(previous, references);

    // P4 只维护本次会话 RAM 镜像；上位机读回校验成功后负责写入便携 JSON。
    // committed 最后置位，是其他任务读取整组 Manual 数据的提交点。
    sharedData->manual_reference_committed = 0;
    manualEncoderReferencesApplyRam(references);
    sharedAtomicStore(&sharedData->manual_reference_valid_bitmap, validMask);
    sharedAtomicStore(
        &sharedData->manual_reference_crc,
        manualEncoderReferencesCrc(references));
    sharedAtomicAddFetch(
        &sharedData->manual_reference_revision,
        static_cast<uint32_t>(1));
    invalidateAutomaticCalibration(sharedData, changedMask);
    sharedData->manual_reference_commit_status = MANUAL_SAVE_STATUS_OK;
    sharedData->manual_reference_committed = 1;
    postSystemEvent(sharedData, SYSTEM_EVENT_REEVALUATE_READY);
    FW_LOG_INFO_PRINTF(
        "[MANUAL RAM] committed mask=0x%08lX changed=0x%08lX crc=0x%08lX\r\n",
        (unsigned long)validMask,
        (unsigned long)changedMask,
        (unsigned long)sharedAtomicLoad(&sharedData->manual_reference_crc));
    return PROTO_ACK_STATUS_OK;
}
