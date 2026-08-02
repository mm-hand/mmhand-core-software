/**
 * @file UpperCommTask.cpp
 * @brief 负责上位机串口收包、命令分发和状态遥测发包。
 */
#include "UpperCommTask.h"
#include "UpperCommCodec.h"
#include "../can/CanCommTask.h"
#include "../../shared/TaskSharedData.h"
#include "../../shared/SharedSafetyState.h"
#include "../../calibration/CalibrationMath.h"
#include "../../calibration/CalibrationTask.h"
#include "../../generated/UrdfJointLimits.generated.h"
#include "../../generated/ProductFingerprint.generated.h"
#include "../../hardware/HardwareMap.h"
#include "../../platform/AtomicValue.h"
#include "../../system/FirmwareLog.h"
#include "../../system/StateMachineTask.h"
#include "../../system/SystemTask.h"
#include "../../system/TaskRuntimeMonitor.h"
#include "../../persistence/ManualCalibrationNvsStore.h"
#if MULTITURN_NVS_PERSISTENCE_ENABLE
#include "../../persistence/MultiTurnNvsStore.h"
#endif
#include "UpperCommCommandRouter.h"
#include "UpperCommProtocol.h"

#include <math.h>
#include <string.h>

using namespace mmhand::upper;

// 职责：解析下行命令并更新共享命令状态；发送传感器/舵机/故障遥测；
// 维护协议 ACK、周期心跳和长触觉包的发送节奏。
static const uint16_t kEncoderDisconnectRaw = 0xFFFF; ///< CAN 磁编断开原始哨兵。
static const uint8_t kTactileFullPacketMagic0 = 0x54; ///< 全量触觉分包魔数 'T'。
static const uint8_t kTactileFullPacketMagic1 = 0x32; ///< 全量触觉分包版本 '2'。

/** 校验快照 LUT 行数、采样数和 Mxx 归属是否与当前 HardwareMap 一致。 */
static bool calibrationTendonLutMatchesJoint(
    uint8_t jointIndex,
    const JointCalibrationResult& result)
{
    if (!result.success) {
        return true;
    }
    if (jointIndex >= ENCODER_TOTAL_NUM ||
        result.tendonLutValid == 0 ||
        result.tendonLutCount < 3 ||
        result.tendonLutCount > TENDON_LUT_MAX_SAMPLES ||
        !isfinite(result.tendonLutAngleStartDeg) ||
        !isfinite(result.tendonLutAngleStepDeg) ||
        result.tendonLutAngleStepDeg <= 0.0f) {
        return false;
    }

    const uint8_t actuatorCount = getJointActuatorCount(jointIndex);
    const uint8_t expectedCount = actuatorCount >= 2 ? 2 : 1;
    for (uint8_t actuatorIndex = 0; actuatorIndex < expectedCount;
         actuatorIndex++) {
        JointActuatorItem actuator;
        if (!getJointActuator(jointIndex, actuatorIndex, &actuator) ||
            actuator.motorChannel >= SERVO_TOTAL_NUM ||
            result.tendonLutMotor[actuatorIndex] != actuator.motorChannel) {
            return false;
        }
    }
    // 单电机快照必须明确把未使用的第 1 行标成 0xFF，避免错误地把
    // 另一台电机的表加载到当前关节。
    if (expectedCount == 1 && result.tendonLutMotor[1] != 0xFF) {
        return false;
    }
    return true;
}

/** 饱和压入 int16 上行字段，避免调试值溢出后改变符号。 */
static int16_t clampI16Local(int32_t value)
{
    if (value > 32767) return 32767;
    if (value < -32768) return -32768;
    return (int16_t)value;
}
/** 返回 Manual、V3 Auto、URDF CRC 和单关节 Manual CRC 全部匹配的 Jxx。 */
static uint32_t autoCalibrationValidMask(TaskSharedData_t* sharedData)
{
    if (!sharedData) return 0;
    uint32_t mask = 0;
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; joint++) {
        const JointCalibrationResult& result = g_jointCalibResult[joint];
        const uint32_t manualEntryCrc =
            manualEncoderReferenceEntryCrc(
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

/** 检查当前 active scope 是否已有 NVS Manual 且每路跨度非零。 */
static bool activeManualReferencesReady(TaskSharedData_t* sharedData)
{
    if (!sharedData || !sharedData->manual_reference_nvs_valid) return false;
    const uint32_t activeMask =
        sharedAtomicLoad(&sharedData->active_joint_mask) &
        kManualEncoderReferenceFullMask;
    return manualEncoderReferencesCoverActiveMask(activeMask);
}

// 将 CAN 磁编 raw 转为线上值；无效通道统一发送断开哨兵。
static uint16_t encodeRawEncoderForWire(uint16_t rawValue, bool channelValid)
{
    if (!channelValid) {
        return (uint16_t)PROTOCOL_DISCONNECT_SENTINEL;
    }

    if (rawValue == kEncoderDisconnectRaw || rawValue == (uint16_t)PROTOCOL_DISCONNECT_SENTINEL) {
        return (uint16_t)PROTOCOL_DISCONNECT_SENTINEL;
    }

    return rawValue;
}

/** 标定 ACK 是否仍需重复发送，防止上位机漏掉瞬时状态。 */
static bool calibrationAckPending()
{
    return g_calibrationAckStatus != CALIB_STATUS_IDLE &&
           g_calibrationAckRepeatRemaining > 0;
}

/** 消费一次待发标定 ACK；最后一次发送后自动恢复 IDLE。 */
static uint8_t takeCalibrationAckStatusForPacket()
{
    if (!calibrationAckPending()) {
        return CALIB_STATUS_IDLE;
    }
    const uint8_t status = g_calibrationAckStatus;
    const uint8_t remaining =
        p4::platform::atomicSubFetch(&g_calibrationAckRepeatRemaining,
                                     (uint8_t)1);
    if (remaining == 0) {
        g_calibrationAckStatus = CALIB_STATUS_IDLE;
    }
    return status;
}

// 协议 ACK 在包尾携带 P4 处理命令时的毫秒时间，供上位机建立操作闭环。
static void sendProtoAckPacket(uint8_t cmd, uint8_t appliedMode, uint8_t status)
{
    uint8_t buffer[12];
    size_t idx = 0;

    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_PROTO_ACK;
    buffer[idx++] = cmd;
    buffer[idx++] = appliedMode;
    buffer[idx++] = status;
    appendU32BigEndian(buffer, &idx, millis());
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    FW_SERIAL_WRITE(buffer, idx);
}

// 上报状态机、owner、active scope、在线/故障和事务 token。
static bool sendSystemStatusPacket(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return false;
    }

    uint8_t buffer[224];
    size_t idx = 0;
    uint32_t faultBitmap = 0;
    uint32_t unknownBitmap = 0;
    uint32_t activeMotorMask = 0;
    uint32_t activeJointMask = 0;
    uint32_t servoOnlineMask = 0;
    uint32_t encoderOnlineMask = 0;
    uint32_t encoderWarnMask = 0;
    uint32_t encoderFaultMask = 0;
    uint8_t canLinkOnline = 0;
    uint8_t requireCanFeedback = 0;
    {
        SharedSafetyGuard guard;
        faultBitmap = sharedAtomicLoad(&sharedData->system_fault_bitmap);
        unknownBitmap = sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap);
        activeMotorMask = sharedAtomicLoad(&sharedData->active_motor_mask);
        activeJointMask = sharedAtomicLoad(&sharedData->active_joint_mask);
        servoOnlineMask = sharedAtomicLoad(&sharedData->servo_online_bitmap);
        encoderOnlineMask = sharedAtomicLoad(&sharedData->encoder_online_bitmap);
        encoderWarnMask = sharedAtomicLoad(&sharedData->encoder_warn_bitmap);
        encoderFaultMask = sharedAtomicLoad(&sharedData->encoder_fault_bitmap);
        canLinkOnline = sharedAtomicLoad(&sharedData->can_link_online);
        requireCanFeedback = sharedAtomicLoad(&sharedData->require_can_feedback);
    }
    const uint32_t transitionCount = sharedData->state_transition_count;

    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_SYSTEM_STATUS;
    buffer[idx++] = sharedData->system_state;
    buffer[idx++] = sharedData->control_enabled;
    buffer[idx++] = sharedData->servo_target_owner;
    buffer[idx++] = sharedData->control_mode;
    buffer[idx++] = g_calibrationStatus;
    buffer[idx++] = (uint8_t)((faultBitmap >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((faultBitmap >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((faultBitmap >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(faultBitmap & 0xFF);
    buffer[idx++] = (uint8_t)((transitionCount >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((transitionCount >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((transitionCount >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(transitionCount & 0xFF);
    buffer[idx++] = (uint8_t)((unknownBitmap >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((unknownBitmap >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((unknownBitmap >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(unknownBitmap & 0xFF);
    buffer[idx++] = sharedData->multiturn_nvs_enabled ? 1 : 0;
    buffer[idx++] = sharedData->multiturn_nvs_snapshot_valid ? 1 : 0;
    buffer[idx++] = (uint8_t)((activeMotorMask >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((activeMotorMask >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((activeMotorMask >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(activeMotorMask & 0xFF);
    buffer[idx++] = (uint8_t)((activeJointMask >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((activeJointMask >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((activeJointMask >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(activeJointMask & 0xFF);
    buffer[idx++] = requireCanFeedback ? 1 : 0;
    buffer[idx++] = (uint8_t)((servoOnlineMask >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((servoOnlineMask >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((servoOnlineMask >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(servoOnlineMask & 0xFF);
    buffer[idx++] = (uint8_t)((encoderOnlineMask >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((encoderOnlineMask >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((encoderOnlineMask >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(encoderOnlineMask & 0xFF);
    buffer[idx++] = canLinkOnline ? 1 : 0;
    buffer[idx++] = (uint8_t)((encoderWarnMask >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((encoderWarnMask >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((encoderWarnMask >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(encoderWarnMask & 0xFF);
    buffer[idx++] = (uint8_t)((encoderFaultMask >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((encoderFaultMask >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((encoderFaultMask >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(encoderFaultMask & 0xFF);
    appendU32BigEndian(buffer, &idx, sharedAtomicLoad(&sharedData->multiturn_nvs_save_request_token));
    appendU32BigEndian(buffer, &idx, sharedAtomicLoad(&sharedData->multiturn_nvs_save_result_token));
    appendU32BigEndian(buffer, &idx, sharedAtomicLoad(&sharedData->multiturn_nvs_save_valid_mask));
    appendU32BigEndian(buffer, &idx, sharedAtomicLoad(&sharedData->multiturn_nvs_save_fail_count));
    appendU32BigEndian(buffer, &idx, sharedAtomicLoad(&sharedData->manual_reference_valid_bitmap));
    appendU32BigEndian(buffer, &idx, sharedAtomicLoad(&sharedData->manual_reference_crc));
    appendU32BigEndian(buffer, &idx, sharedAtomicLoad(&sharedData->manual_reference_nvs_sequence));
    buffer[idx++] = sharedData->manual_reference_nvs_valid ? 1 : 0;
    appendU32BigEndian(buffer, &idx, autoCalibrationValidMask(sharedData));
    buffer[idx++] = sharedData->manual_reference_save_status;
    buffer[idx++] = sharedData->rope_init_status;
    buffer[idx++] = sharedData->rope_init_reason;
    appendU32BigEndian(buffer, &idx, sharedData->rope_init_sequence);
    appendU32BigEndian(buffer, &idx, sharedData->rope_init_joint_mask);
    appendU32BigEndian(buffer, &idx, sharedData->rope_init_timestamp);
    appendU32BigEndian(buffer, &idx, sharedData->pending_direct_generation);
    appendU32BigEndian(buffer, &idx, sharedData->applied_direct_generation);
    buffer[idx++] = sharedData->direct_reject_reason;
    appendU32BigEndian(
        buffer, &idx,
        sharedAtomicLoad(&sharedData->calibration_warning_bitmap));
    {
        SharedSafetyGuard guard;
        for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; ++joint) {
            appendI16BigEndian(
                buffer,
                &idx,
                sharedData->calibration_auto_min_offset_counts[joint]);
        }
    }
    appendU32BigEndian(buffer, &idx,
                       sharedAtomicLoad(&sharedData->feedback_admission_sequence));
    buffer[idx++] = sharedAtomicLoad(&sharedData->feedback_admission_status);
    buffer[idx++] = sharedAtomicLoad(&sharedData->feedback_reject_reason);
    appendU32BigEndian(
        buffer, &idx,
        sharedAtomicLoad(&sharedData->feedback_collision_hold_bitmap));
    appendU32BigEndian(
        buffer, &idx,
        sharedAtomicLoad(&sharedData->feedback_collision_transition_count));
    // 保留原 hold/transition 的字节位置，在包尾追加 armed 位图。
    appendU32BigEndian(
        buffer, &idx,
        sharedAtomicLoad(&sharedData->feedback_collision_armed_bitmap));
    // Reset Zero 状态固定追加在旧状态字段之后，旧上位机仍可按原长度解析。
    appendU32BigEndian(
        buffer, &idx,
        sharedAtomicLoad(&sharedData->servo_turn_overflow_bitmap));
    appendU32BigEndian(
        buffer, &idx,
        sharedAtomicLoad(&sharedData->servo_reset_zero_request_token));
    appendU32BigEndian(
        buffer, &idx,
        sharedAtomicLoad(&sharedData->servo_reset_zero_result_token));
    appendU32BigEndian(
        buffer, &idx,
        sharedAtomicLoad(&sharedData->servo_reset_zero_success_mask));
    appendU32BigEndian(
        buffer, &idx,
        sharedAtomicLoad(&sharedData->servo_reset_zero_failed_mask));
    appendU32BigEndian(
        buffer, &idx,
        sharedAtomicLoad(
            &sharedData->servo_reset_zero_invalidated_joint_mask));
    buffer[idx++] =
        sharedAtomicLoad(&sharedData->servo_reset_zero_status);
    // 状态包时间表示本次状态快照在 P4 侧被序列化的时刻。
    appendU32BigEndian(buffer, &idx, millis());
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    return FW_SERIAL_WRITE(buffer, idx) == idx;
}

/** 上报 21 路 Manual 端点、有效位、CRC 和 NVS 保存状态。 */
static bool sendManualEncoderReferencePacket(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return false;
    }

    uint8_t buffer[144];
    size_t idx = 0;
    ManualEncoderReference references[ENCODER_TOTAL_NUM];
    uint32_t validMask = 0;
    uint32_t referenceCrc = 0;
    uint32_t nvsSequence = 0;
    uint8_t nvsValid = 0;
    {
        SharedSafetyGuard guard;
        memcpy(references,
               g_manualEncoderReference,
               sizeof(references));
        validMask = sharedAtomicLoad(
            &sharedData->manual_reference_valid_bitmap);
        referenceCrc = sharedAtomicLoad(
            &sharedData->manual_reference_crc);
        nvsSequence = sharedAtomicLoad(
            &sharedData->manual_reference_nvs_sequence);
        nvsValid = sharedData->manual_reference_nvs_valid ? 1 : 0;
    }

    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_MANUAL_ENCODER_REFERENCE;
    buffer[idx++] = 3; // Payload version: v3 appends P4 encoder directions.
    appendU32BigEndian(buffer, &idx, validMask);
    appendU32BigEndian(buffer, &idx, referenceCrc);
    appendU32BigEndian(buffer, &idx, nvsSequence);
    buffer[idx++] = nvsValid;
    appendU32BigEndian(buffer, &idx, autoCalibrationValidMask(sharedData));
    buffer[idx++] = sharedData->manual_reference_save_status;
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; joint++) {
        appendU16BigEndian(buffer, &idx, references[joint].minRaw);
        appendU16BigEndian(buffer, &idx, references[joint].maxRaw);
    }
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; joint++) {
        buffer[idx++] = (g_encoderDirection[joint] < 0) ? 0xFF : 0x01;
    }
    appendU32BigEndian(buffer, &idx, millis());
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    return FW_SERIAL_WRITE(buffer, idx) == idx;
}
/** 上报一个 TaskRuntimeSnapshot，字段单位由 TaskRuntimeMonitor.h 约定。 */
static void sendTaskRuntimePacket(const TaskRuntimeSnapshot& diag)
{
    uint8_t buffer[64];
    size_t idx = 0;
    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_TASK_DIAG;
    buffer[idx++] = 2; // Payload version.
    buffer[idx++] = diag.taskId;
    buffer[idx++] = diag.active;
    buffer[idx++] = diag.priority;
    buffer[idx++] = (uint8_t)diag.coreId;
    appendU32BigEndian(buffer, &idx, diag.stackAllocatedBytes);
    appendU32BigEndian(buffer, &idx, diag.stackCurrentFreeBytes);
    appendU32BigEndian(buffer, &idx, diag.stackMinFreeBytes);
    appendU32BigEndian(buffer, &idx, diag.loopCount);
    appendU32BigEndian(buffer, &idx, diag.maxGapUs);
    appendU32BigEndian(buffer, &idx, diag.maxExecUs);
    appendU32BigEndian(buffer, &idx, diag.overrunCount);
    appendU16BigEndian(buffer, &idx, diag.consecutiveOverruns);
    appendU16BigEndian(buffer, &idx, diag.maxConsecutiveOverruns);
    appendU16BigEndian(buffer, &idx, diag.core0IdlePermille);
    appendU16BigEndian(buffer, &idx, diag.core1IdlePermille);
    appendU32BigEndian(buffer, &idx, diag.systemStatusMaxGapMs);
    appendU32BigEndian(buffer, &idx, diag.stopLastLatencyUs);
    appendU32BigEndian(buffer, &idx, diag.stopMaxLatencyUs);
    appendU32BigEndian(buffer, &idx, diag.timestampMs);
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    FW_SERIAL_WRITE(buffer, idx);
}

// 上报舵机过载位图和每路锁存的触发 load。
static void sendFaultStatusPacket(TaskSharedData_t* sharedData, uint32_t overloadFaultBitmap)
{
    uint8_t buffer[12 + SERVO_TOTAL_NUM * 2];
    int16_t faultLoads[SERVO_TOTAL_NUM] = {0};
    if (sharedData) {
        SharedSafetyGuard guard;
        for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
            faultLoads[ch] = sharedData->overload_fault_load[ch];
        }
    }
    size_t idx = 0;

    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_FAULT_STATUS;
    buffer[idx++] = (uint8_t)((overloadFaultBitmap >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((overloadFaultBitmap >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((overloadFaultBitmap >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(overloadFaultBitmap & 0xFF);
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        const int16_t load = faultLoads[ch];
        const uint16_t encodedLoad = (uint16_t)load;
        buffer[idx++] = (uint8_t)((encodedLoad >> 8) & 0xFF);
        buffer[idx++] = (uint8_t)(encodedLoad & 0xFF);
    }
    appendU32BigEndian(buffer, &idx, millis());
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    FW_SERIAL_WRITE(buffer, idx);
}
/** 上报 27 路标定 ABS 边界、有效位和 LOCK 位。 */
static void sendMotorAbsLimitPacket(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return;
    }

    uint8_t buffer[240];
    size_t idx = 0;
    uint32_t validBitmap = 0;
    uint32_t lockedBitmap = 0;
    int32_t limitMin[SERVO_TOTAL_NUM] = {0};
    int32_t limitMax[SERVO_TOTAL_NUM] = {0};
    {
        SharedSafetyGuard guard;
        validBitmap = sharedAtomicLoad(&sharedData->motor_abs_limit_valid_bitmap);
        lockedBitmap = sharedAtomicLoad(&sharedData->motor_abs_limit_locked_bitmap);
        memcpy(limitMin, sharedData->motor_abs_limit_min, sizeof(limitMin));
        memcpy(limitMax, sharedData->motor_abs_limit_max, sizeof(limitMax));
    }

    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_MOTOR_ABS_LIMIT;
    buffer[idx++] = (uint8_t)((validBitmap >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((validBitmap >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((validBitmap >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(validBitmap & 0xFF);
    buffer[idx++] = (uint8_t)((lockedBitmap >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((lockedBitmap >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((lockedBitmap >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(lockedBitmap & 0xFF);
    for (int i = 0; i < SERVO_TOTAL_NUM; i++) {
        appendI32BigEndian(buffer, &idx, limitMin[i]);
        appendI32BigEndian(buffer, &idx, limitMax[i]);
    }
    appendU32BigEndian(buffer, &idx, millis());
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    FW_SERIAL_WRITE(buffer, idx);
}

// 上报每台舵机独立的动态逻辑行程。单台舵机掉电重建 offset 后，只有
// 对应 Mxx 的 min/max 会变化，其他26路保持原值。
static void sendServoLogicalRangePacket(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return;
    }
    uint8_t buffer[240];
    size_t idx = 0;
    uint32_t validBitmap = 0;
    int32_t rangeMin[SERVO_TOTAL_NUM] = {0};
    int32_t rangeMax[SERVO_TOTAL_NUM] = {0};
    {
        SharedSafetyGuard guard;
        validBitmap =
            sharedAtomicLoad(&sharedData->servo_logical_range_valid_bitmap);
        memcpy(rangeMin,
               sharedData->servo_logical_range_min,
               sizeof(rangeMin));
        memcpy(rangeMax,
               sharedData->servo_logical_range_max,
               sizeof(rangeMax));
    }

    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0;
    buffer[idx++] = PACKET_TYPE_SERVO_LOGICAL_RANGE;
    appendU32BigEndian(buffer, &idx, validBitmap);
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        appendI32BigEndian(buffer, &idx, rangeMin[ch]);
        appendI32BigEndian(buffer, &idx, rangeMax[ch]);
    }
    appendU32BigEndian(buffer, &idx, millis());
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    FW_SERIAL_WRITE(buffer, idx);
}

/** 上报一个关节的 V3 Auto、Tendon LUT 和动态范围快照。 */
static void sendJointCalibSnapshotPacket(uint8_t jointIndex)
{
    if (jointIndex >= ENCODER_TOTAL_NUM) {
        return;
    }

    const JointCalibrationResult& result = g_jointCalibResult[jointIndex];
    // V4 基础快照 209 bytes，附加13 bytes装配告警、4 bytes时间戳和
    // 帧头尾后共230 bytes；预留到240，避免局部数组越界。
    uint8_t buffer[240];
    size_t idx = 0;
    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_JOINT_CALIB_SNAPSHOT;
    buffer[idx++] = jointIndex;
    buffer[idx++] = result.success ? 1 : 0;
    appendI32BigEndian(buffer, &idx, result.encoderMin);
    appendI32BigEndian(buffer, &idx, result.encoderMax);
    appendI32BigEndian(buffer, &idx, result.offset);
    appendFloatBigEndian(buffer, &idx, result.angleMax);
    appendI32BigEndian(buffer, &idx, result.startEncoder);
    appendI32BigEndian(buffer, &idx, result.tightEncoder);
    appendI32BigEndian(buffer, &idx, result.zeroEncoder);
    appendI32BigEndian(buffer, &idx, result.encoderSpan);
    appendI32BigEndian(buffer, &idx, result.startServoAbs);
    appendI32BigEndian(buffer, &idx, result.tightServoAbs);
    appendI32BigEndian(buffer, &idx, result.zeroServoAbs);
    appendFloatBigEndian(buffer, &idx, result.startDeg);
    appendFloatBigEndian(buffer, &idx, result.tightDeg);
    appendFloatBigEndian(buffer, &idx, result.totalDeg);
    buffer[idx++] = result.scaled ? 1 : 0;
    buffer[idx++] = (g_encoderDirection[jointIndex] < 0) ? 0xFF : 0x01;
    buffer[idx++] = result.tendonLutValid ? 1 : 0;
    buffer[idx++] = result.tendonLutCount;
    buffer[idx++] = result.tendonLutMotor[0];
    buffer[idx++] = result.tendonLutMotor[1];
    appendFloatBigEndian(buffer, &idx, result.tendonLutAngleStartDeg);
    appendFloatBigEndian(buffer, &idx, result.tendonLutAngleStepDeg);
    for (uint8_t actuator = 0; actuator < TENDON_LUT_ACTUATOR_COUNT; actuator++) {
        // 以有效样本 min/max 中点作为锚点，可保证 ST3215 整个硬件行程内
        // 的相对量落入 int16；未使用样本统一写 0 相对量。
        int32_t anchor = 0;
        const uint8_t count =
            min(result.tendonLutCount, (uint8_t)TENDON_LUT_MAX_SAMPLES);
        if (count > 0) {
            int32_t minAbs = result.tendonLutMotorAbs[actuator][0];
            int32_t maxAbs = minAbs;
            for (uint8_t sample = 1; sample < count; sample++) {
                minAbs = min(minAbs, result.tendonLutMotorAbs[actuator][sample]);
                maxAbs = max(maxAbs, result.tendonLutMotorAbs[actuator][sample]);
            }
            anchor = minAbs + (maxAbs - minAbs) / 2;
        }
        appendI32BigEndian(buffer, &idx, anchor);
        for (uint8_t sample = 0; sample < TENDON_LUT_MAX_SAMPLES; sample++) {
            const int32_t delta =
                sample < count
                    ? result.tendonLutMotorAbs[actuator][sample] - anchor
                    : 0;
            appendI16BigEndian(buffer,
                               &idx,
                               clampI16Local(delta));
        }
    }
    buffer[idx++] = result.mappingVersion;
    appendI32BigEndian(buffer, &idx, result.encoderAtUrdfMin);
    appendI32BigEndian(buffer, &idx, result.encoderAtUrdfMax);
    appendFloatBigEndian(buffer, &idx, result.angleMin);
    appendU32BigEndian(buffer, &idx, result.urdfConfigCrc);
    appendU16BigEndian(buffer, &idx, result.autoMinRaw);
    appendU16BigEndian(buffer, &idx, result.autoMaxRaw);
    appendU32BigEndian(buffer, &idx, result.manualReferenceCrc);
    appendFloatBigEndian(buffer, &idx, result.currentMinDeg);
    appendFloatBigEndian(buffer, &idx, result.currentMaxDeg);
    CalibrationAssemblyWarningInfo warningInfo;
    if (!evaluateCalibrationAssemblyWarning(
            jointIndex, result, &warningInfo)) {
        memset(&warningInfo, 0, sizeof(warningInfo));
    }
    buffer[idx++] = warningInfo.reasonMask;
    appendI16BigEndian(
        buffer, &idx, warningInfo.autoMinOffsetCounts);
    appendI16BigEndian(
        buffer, &idx, warningInfo.autoMaxShortfallCounts);
    appendU16BigEndian(
        buffer, &idx, warningInfo.autoSpanCounts);
    appendU16BigEndian(
        buffer, &idx, warningInfo.manualSpanCounts);
    appendU16BigEndian(
        buffer, &idx, warningInfo.urdfSpanCounts);
    appendU16BigEndian(
        buffer, &idx, warningInfo.spanDifferenceCounts);
    appendU32BigEndian(buffer, &idx, millis());
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    FW_SERIAL_WRITE(buffer, idx);
}

/** 上报最近一次结构化自动标定失败或警告。 */
static void sendCalibrationDiagPacket(const CalibrationDiagData_t& diag)
{
    uint8_t buffer[40];
    size_t idx = 0;
    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_CALIB_DIAG;
    buffer[idx++] = diag.valid ? 1 : 0;
    buffer[idx++] = diag.reason;
    buffer[idx++] = diag.phase;
    buffer[idx++] = diag.joint;
    buffer[idx++] = diag.motor;
    appendI32BigEndian(buffer, &idx, diag.valueA);
    appendI32BigEndian(buffer, &idx, diag.valueB);
    appendI32BigEndian(buffer, &idx, diag.valueC);
    buffer[idx++] = (uint8_t)((diag.bitmap >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((diag.bitmap >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((diag.bitmap >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(diag.bitmap & 0xFF);
    buffer[idx++] = (uint8_t)((diag.timestamp >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((diag.timestamp >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((diag.timestamp >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(diag.timestamp & 0xFF);
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    FW_SERIAL_WRITE(buffer, idx);
}

/** 上报 21 路磁编 raw、有效状态和源快照时间。 */
static void sendEncoderRawPacket(const RemoteSensorData_t& sensorRaw)
{
    uint8_t buffer[68];
    size_t idx = 0;

    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_ENCODER_RAW;
    for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
    {
        const bool channelValid =
            sensorRaw.isValid &&
            (sensorRaw.errorFlags[i] == 0) &&
            (sensorRaw.encoderValues[i] != kEncoderDisconnectRaw);
        const uint16_t val = encodeRawEncoderForWire(sensorRaw.encoderValues[i], channelValid);
        buffer[idx++] = (uint8_t)((val >> 8) & 0xFF);
        buffer[idx++] = (uint8_t)(val & 0xFF);
    }
    // 使用 CAN 接收该批磁编原始数据时记录的 P4 时间，而不是串口发送时间。
    appendU32BigEndian(buffer, &idx, sensorRaw.timestamp);
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    FW_SERIAL_WRITE(buffer, idx);
}

/** 把一帧全量触觉拆成串口可承载的小包并保留同一采样时间戳。 */
static void sendTactileFullPackets(const RemoteTactileData_t& tactile)
{
    if (!tactile.fullValid) {
        return;
    }

    // 帧总长受 1 字节 LEN 和 256 字节发送缓冲区限制；预留 4 字节 P4 时间和尾字节。
    static const size_t kTactileFullSerialChunkBytes = 239;
    const uint8_t fragCount = (uint8_t)((TACTILE_FULL_BYTES + kTactileFullSerialChunkBytes - 1) /
                                        kTactileFullSerialChunkBytes);
    const uint16_t fullCrc = crc16Ccitt(tactile.fullValues, TACTILE_FULL_BYTES);

    for (uint8_t fragIdx = 0; fragIdx < fragCount; fragIdx++) {
        const size_t offset = (size_t)fragIdx * kTactileFullSerialChunkBytes;
        const size_t remaining = TACTILE_FULL_BYTES - offset;
        const uint8_t chunkLen = (uint8_t)((remaining > kTactileFullSerialChunkBytes)
            ? kTactileFullSerialChunkBytes
            : remaining);

        uint8_t buffer[256];
        size_t idx = 0;
        buffer[idx++] = PROTOCOL_HEADER;
        buffer[idx++] = 0x00;
        buffer[idx++] = PACKET_TYPE_TACTILE_FULL;
        buffer[idx++] = kTactileFullPacketMagic0;
        buffer[idx++] = kTactileFullPacketMagic1;
        buffer[idx++] = tactile.fullSeq;
        buffer[idx++] = tactile.seq;
        buffer[idx++] = fragIdx;
        buffer[idx++] = fragCount;
        buffer[idx++] = chunkLen;
        buffer[idx++] = (uint8_t)(fullCrc & 0xFF);
        buffer[idx++] = (uint8_t)((fullCrc >> 8) & 0xFF);
        memcpy(buffer + idx, tactile.fullValues + offset, chunkLen);
        idx += chunkLen;
        // 每个分片均携带同一批全量触觉数据的 P4 采集时间。
        appendU32BigEndian(buffer, &idx, tactile.fullTimestamp);
        buffer[idx++] = PROTOCOL_TAIL;
        buffer[1] = (uint8_t)(idx - 2);
        FW_SERIAL_WRITE(buffer, idx);
    }
}

// 汇总并发送一轮常规上行数据；各输入为空时跳过对应包。
static void sendDataPacket(ServoStatus_t* pServo,
                           RemoteSensorData_t* pSensorRaw,
                           MappedAngleData_t* pMappedAngle,
                           ServoAngleData_t* pServoAngle,
                           ServoAngleData_t* pServoRaw,
                           ServoTelemetryData_t* pTelemetry,
                           RemoteTactileData_t* pTactile,
                           JointDebugData_t* pJointDebug)
{
    (void)pServo;

    uint8_t buffer[256];
    size_t idx = 0;
    bool sendFullTactileAfter = false;
    const RemoteTactileData_t* fullTactile = NULL;
    uint32_t p4TimestampMs = millis();
    bool appendTrailingTimestamp = true;

    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00; // LEN placeholder.
    const uint8_t calibrationAckStatus = takeCalibrationAckStatusForPacket();
    if (calibrationAckStatus != CALIB_STATUS_IDLE)
    {
        buffer[idx++] = PACKET_TYPE_CALIB_ACK;
        buffer[idx++] = calibrationAckStatus;
    }
    else if (pSensorRaw)
    {
        p4TimestampMs = pSensorRaw->timestamp;
        buffer[idx++] = PACKET_TYPE_SENSOR;
        for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
        {
            const bool channelValid =
                pSensorRaw->isValid &&
                (pSensorRaw->errorFlags[i] == 0) &&
                (pSensorRaw->encoderValues[i] != kEncoderDisconnectRaw);
            const uint16_t val = encodeRawEncoderForWire(pSensorRaw->encoderValues[i], channelValid);
            buffer[idx++] = (uint8_t)((val >> 8) & 0xFF);
            buffer[idx++] = (uint8_t)(val & 0xFF);
        }
    }
    else if (pMappedAngle)
    {
        p4TimestampMs = pMappedAngle->timestamp;
        buffer[idx++] = PACKET_TYPE_SENSOR;
        for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
        {
            const int16_t val = (pMappedAngle->isValid && pMappedAngle->validFlags[i])
                ? pMappedAngle->angleValues[i]
                : (int16_t)PROTOCOL_DISCONNECT_SENTINEL;
            const uint16_t raw = (uint16_t)val;
            buffer[idx++] = (uint8_t)((raw >> 8) & 0xFF);
            buffer[idx++] = (uint8_t)(raw & 0xFF);
        }
    }
    else if (pServoAngle)
    {
        p4TimestampMs = pServoAngle->timestamp;
        buffer[idx++] = PACKET_TYPE_SERVO_ANGLE;
        for (int i = 0; i < SERVO_TOTAL_NUM; i++)
        {
            const int32_t angle = pServoAngle->servoAngles[i];
            buffer[idx++] = (uint8_t)((angle >> 24) & 0xFF);
            buffer[idx++] = (uint8_t)((angle >> 16) & 0xFF);
            buffer[idx++] = (uint8_t)((angle >> 8) & 0xFF);
            buffer[idx++] = (uint8_t)(angle & 0xFF);
        }
        for (int i = 0; i < SERVO_TOTAL_NUM; i++)
        {
            buffer[idx++] = pServoAngle->onlineStatus[i];
        }
    }
    else if (pServoRaw)
    {
        static_assert(
            3 + SERVO_TOTAL_NUM * 5 + 4 + 1 <= 256,
            "servo raw/FlashRaw packet exceeds serial buffer");
        p4TimestampMs = pServoRaw->timestamp;
        buffer[idx++] = PACKET_TYPE_SERVO_RAW;
        for (int i = 0; i < SERVO_TOTAL_NUM; i++)
        {
            const uint16_t raw = (uint16_t)pServoRaw->servoRawPositions[i];
            buffer[idx++] = (uint8_t)((raw >> 8) & 0xFF);
            buffer[idx++] = (uint8_t)(raw & 0xFF);
        }
        for (int i = 0; i < SERVO_TOTAL_NUM; i++)
        {
            // 第二段固定为最近一次成功写入 Flash 的单圈 raw。
            const uint16_t flashRaw =
                (uint16_t)pServoRaw->servoFlashRawPositions[i];
            buffer[idx++] = (uint8_t)((flashRaw >> 8) & 0xFF);
            buffer[idx++] = (uint8_t)(flashRaw & 0xFF);
        }
        for (int i = 0; i < SERVO_TOTAL_NUM; i++)
        {
            buffer[idx++] = pServoRaw->onlineStatus[i];
        }
    }
    else if (pTelemetry)
    {
        static_assert(
            3 + SERVO_TOTAL_NUM * 9 + 4 + 1 <= 256,
            "servo telemetry packet exceeds serial buffer");
        p4TimestampMs = pTelemetry->timestamp;
        buffer[idx++] = PACKET_TYPE_SERVO_TELEM;
        for (int i = 0; i < SERVO_TOTAL_NUM; i++)
        {
            const int16_t speed = pTelemetry->speed[i];
            const int16_t load = pTelemetry->load[i];
            const int16_t current = pTelemetry->current[i];
            buffer[idx++] = (uint8_t)(((uint16_t)speed >> 8) & 0xFF);
            buffer[idx++] = (uint8_t)((uint16_t)speed & 0xFF);
            buffer[idx++] = (uint8_t)(((uint16_t)load >> 8) & 0xFF);
            buffer[idx++] = (uint8_t)((uint16_t)load & 0xFF);
            // 电流保持ST3215原始有符号值，避免在固件中硬编码未经确认的mA比例。
            buffer[idx++] = (uint8_t)(((uint16_t)current >> 8) & 0xFF);
            buffer[idx++] = (uint8_t)((uint16_t)current & 0xFF);
            buffer[idx++] = pTelemetry->voltage[i];
            buffer[idx++] = pTelemetry->temperature[i];
            buffer[idx++] = pTelemetry->onlineStatus[i];
        }
    }
    else if (pTactile)
    {
        p4TimestampMs = pTactile->timestamp;
        buffer[idx++] = PACKET_TYPE_TACTILE;
        buffer[idx++] = pTactile->seq;
        for (int i = 0; i < TACTILE_SUMMARY_BYTES; i++)
        {
            buffer[idx++] = pTactile->values[i];
        }
        sendFullTactileAfter = pTactile->fullValid;
        fullTactile = pTactile;
    }
    else if (pJointDebug)
    {
        // Joint Debug 的业务结构本身已经包含 timestamp，禁止重复追加。
        appendTrailingTimestamp = false;
        buffer[idx++] = PACKET_TYPE_JOINT1_DEBUG;
        buffer[idx++] = pJointDebug->jointIndex;
        buffer[idx++] = pJointDebug->valid;
        appendFloatBigEndian(buffer, &idx, pJointDebug->targetDeg);
        appendFloatBigEndian(buffer, &idx, pJointDebug->magActualDeg);
        appendFloatBigEndian(buffer, &idx, pJointDebug->loop1Output);
        buffer[idx++] = pJointDebug->cmdValid;
        const uint16_t cmdPosRaw = (uint16_t)pJointDebug->cmdTargetPos;
        buffer[idx++] = (uint8_t)((cmdPosRaw >> 8) & 0xFF);
        buffer[idx++] = (uint8_t)(cmdPosRaw & 0xFF);
    }
    else
    {
        return;
    }

    // 除原生含时间字段的 Joint Debug 外，实时数据包统一追加一次 P4 采集时间。
    if (appendTrailingTimestamp) {
        appendU32BigEndian(buffer, &idx, p4TimestampMs);
    }
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    FW_SERIAL_WRITE(buffer, idx);
    if (sendFullTactileAfter && fullTactile) {
        sendTactileFullPackets(*fullTactile);
    }
}

/** 解码协议中少量本地使用的 int16 大端字段。 */
static int16_t readI16BigEndian(const uint8_t* p)
{
    const uint16_t raw = ((uint16_t)p[0] << 8) | (uint16_t)p[1];
    return (int16_t)raw;
}

/** 标识必须已有 active scope Manual 才能接受的运动命令。 */
static bool commandRequiresManualReference(uint8_t cmd)
{
    switch (cmd) {
        case CMD_CALIBRATE:
        case CMD_RUN_FEEDBACK:
        case CMD_ANGLE_CTRL:
        case CMD_ANGLE_CTRL_MASKED:
            return true;
        default:
            return false;
    }
}
/** Manual 写入仅允许在无运动所有权的静止状态执行。 */
static bool manualUpdateMotionActive(TaskSharedData_t* sharedData)
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

/** 比较写入前后 Manual，返回真正发生变化的 Jxx。 */
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

/** 由 HardwareMap 计算一个 Jxx 对应的全部 Mxx。 */
static uint32_t motorMaskForJoint(uint8_t jointIndex)
{
    if (jointIndex >= ENCODER_TOTAL_NUM) return 0;
    uint32_t motorMask = 0;
    JointActuatorGroup group;
    if (getJointActuatorGroup(jointIndex, &group)) {
        for (uint8_t actuator = 0; actuator < group.count; actuator++) {
            const uint8_t motorChannel = group.actuators[actuator].motorChannel;
            if (motorChannel < SERVO_TOTAL_NUM &&
                isMotorChannelValid(motorChannel)) {
                motorMask |= (1UL << motorChannel);
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

// P4 的 HardwareMap 是电机归属关系的唯一权威来源。上位机只发送需要
// 重建 turn0 的电机，P4 在这里推导必须重新标定的完整关节；双腱关节
// 任意一台电机丢圈，都会使整个关节的 LUT 失效。
static uint32_t jointMaskForMotorMask(uint32_t motorMask)
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

/** Manual 或 Reset Zero 变化后，只失效受影响 Jxx 的 Auto/LUT/ABS Limit。 */
static void invalidateAutomaticCalibration(TaskSharedData_t* sharedData,
                                           uint32_t jointMask)
{
    if (!sharedData) return;
    jointMask &= kManualEncoderReferenceFullMask;
    uint32_t motorMask = 0;
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; joint++) {
        if ((jointMask & (1UL << joint)) == 0) continue;
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
            if ((motorMask & (1UL << motor)) == 0) continue;
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
    g_calibrationAckStatus = g_calibrationStatus;
    g_calibrationAckRepeatRemaining = 0;
    sharedAtomicFetchAnd(&sharedData->system_fault_bitmap,
                         (uint32_t)~SYSTEM_FAULT_CALIBRATION);
    FW_LOG_INFO_PRINTF(
        "[NVS MANUAL] AUTO_INVALIDATE joints=0x%08lX motors=0x%08lX remaining=0x%08lX\r\n",
        (unsigned long)jointMask,
        (unsigned long)motorMask,
        (unsigned long)autoCalibrationValidMask(sharedData));
}

/**
 * 周期上报当前固件的产品指纹。
 *
 * 上位机将这些 CRC 写入每次 run_data 的 meta.json，后续分析可以精确确认
 * 当时使用的 URDF、舵机映射、控制参数和固件源码是否完全一致。
 */
static void sendProductFingerprintPacket()
{
    uint8_t buffer[40];
    size_t idx = 0;
    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_PRODUCT_FINGERPRINT;
    buffer[idx++] = kProductFingerprintSchemaVersion;
    buffer[idx++] = kUrdfFeedbackMappingVersion;
    buffer[idx++] = kManualScaledUrdfFeedbackMappingVersion;
    buffer[idx++] = 0; // 保留字节，后续扩展时不改变当前字段偏移。
    appendU32BigEndian(buffer, &idx, kProductControlModelCrc);
    appendU32BigEndian(buffer, &idx, kProductVisualModelCrc);
    appendU32BigEndian(buffer, &idx, kProductHardwareMapCrc);
    appendU32BigEndian(buffer, &idx, kProductParameterCrc);
    appendU32BigEndian(buffer, &idx, kProductFirmwareSourceCrc);
    appendU32BigEndian(buffer, &idx, kProductStructureCommitCrc);
    appendU32BigEndian(buffer, &idx, millis());
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    FW_SERIAL_WRITE(buffer, idx);
}

/** 消费舵机任务的 Reset Zero 结果，并联动失效关节标定。 */
static void processServoResetZeroResult(TaskSharedData_t* sharedData,
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

    // success mask 表示硬件坐标已经被重置；即使 Flash 随后失败，
    // 原 Auto/LUT/ABS Limit 也不能继续搭配新的 turn0 使用。
    const uint32_t resetMotorMask =
        sharedAtomicLoad(&sharedData->servo_reset_zero_success_mask) &
        validMotorChannelMask();
    const uint32_t jointMask = jointMaskForMotorMask(resetMotorMask);
    sharedAtomicStore(
        &sharedData->servo_reset_zero_invalidated_joint_mask,
        jointMask);
    if (jointMask != 0) {
        invalidateAutomaticCalibration(sharedData, jointMask);
        postSystemEvent(sharedData, SYSTEM_EVENT_REEVALUATE_READY);
    }
    FW_LOG_WARN_PRINTF(
        "[SERVO RESET ZERO] result token=%lu motors=0x%08lX "
        "failed=0x%08lX invalidatedJoints=0x%08lX status=%u\r\n",
        (unsigned long)resultToken,
        (unsigned long)resetMotorMask,
        (unsigned long)sharedAtomicLoad(
            &sharedData->servo_reset_zero_failed_mask),
        (unsigned long)jointMask,
        (unsigned)sharedAtomicLoad(
            &sharedData->servo_reset_zero_status));
}

/** 写 Manual 到 A/B NVS，读回 RAM 镜像并按变化范围失效 Auto。 */
static uint8_t commitManualReferences(
    TaskSharedData_t* sharedData,
    const ManualEncoderReference references[ENCODER_TOTAL_NUM])
{
    uint32_t validMask = 0;
    manualEncoderReferencesValidate(references, &validMask);
    if (!sharedData || validMask == 0) {
        return PROTO_ACK_STATUS_INVALID_DATA;
    }

    ManualEncoderReference previous[ENCODER_TOTAL_NUM];
    {
        SharedSafetyGuard guard;
        memcpy(previous, g_manualEncoderReference, sizeof(previous));
    }
    const uint32_t changedMask =
        changedManualReferenceMask(previous, references);

    uint32_t sequence = 0;
    uint32_t referenceCrc = 0;
    if (!manualCalibrationNvsSave(references, &sequence, &referenceCrc)) {
        sharedData->manual_reference_save_status = MANUAL_SAVE_STATUS_FAILED;
        return PROTO_ACK_STATUS_NVS_FAILURE;
    }

    sharedData->manual_reference_nvs_valid = 0;
    manualEncoderReferencesApplyRam(references);
    sharedAtomicStore(&sharedData->manual_reference_valid_bitmap, validMask);
    sharedAtomicStore(&sharedData->manual_reference_crc, referenceCrc);
    sharedAtomicStore(&sharedData->manual_reference_nvs_sequence, sequence);
    invalidateAutomaticCalibration(sharedData, changedMask);
    sharedData->manual_reference_save_status = MANUAL_SAVE_STATUS_OK;
    sharedData->manual_reference_nvs_valid = 1;
    postSystemEvent(sharedData, SYSTEM_EVENT_REEVALUATE_READY);
    return PROTO_ACK_STATUS_OK;
}
// 应用一帧已通过长度检查的下行命令，并返回对应协议 ACK。
static void handleParsedCommand(TaskSharedData_t* sharedData, const uint8_t* frame, size_t frameLen)
{
    if (!sharedData || !frame || frameLen == 0) {
        return;
    }

    const uint8_t cmd = frame[0];
    // 每一帧通过结构校验的上位机命令都刷新心跳，不要求它一定是运动命令。
    // StateMachineTask 在另一核心读取该字段，统一走原子接口避免租约边界读到撕裂值。
    sharedAtomicStore(&sharedData->host_rx_heartbeat_ms, millis());

    if (cmd == CMD_HOST_HEARTBEAT)
    {
        return;
    }
    if (cmd == CMD_MANUAL_ENCODER_QUERY)
    {
        const bool sent = sendManualEncoderReferencePacket(sharedData);
        sendProtoAckPacket(CMD_MANUAL_ENCODER_QUERY,
                           sharedData->system_state,
                           sent ? PROTO_ACK_STATUS_OK
                                : PROTO_ACK_STATUS_UNSUPPORTED_MODE);
        return;
    }

    if (cmd == CMD_ROPE_INIT)
    {
        const uint32_t requestedMask =
            readU32BigEndian(frame + 1) & kManualEncoderReferenceFullMask;
        const uint32_t activeMask =
            sharedAtomicLoad(&sharedData->active_joint_mask) &
            kManualEncoderReferenceFullMask;
        uint8_t status = PROTO_ACK_STATUS_QUEUED;
        if (requestedMask == 0 || requestedMask != activeMask) {
            status = PROTO_ACK_STATUS_INVALID_DATA;
        } else if (!activeManualReferencesReady(sharedData)) {
            status = PROTO_ACK_STATUS_MANUAL_REFERENCE_REQUIRED;
        } else if ((autoCalibrationValidMask(sharedData) & requestedMask) !=
                   requestedMask) {
            status = PROTO_ACK_STATUS_CALIBRATION_REQUIRED;
        } else if (sharedData->system_state != SYSTEM_STATE_IDLE &&
                   sharedData->system_state != SYSTEM_STATE_READY &&
                   sharedData->system_state != SYSTEM_STATE_RECOVERY_REQUIRED) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else if (!postSystemEvent(sharedData, SYSTEM_EVENT_ROPE_INIT)) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        }
        sendProtoAckPacket(CMD_ROPE_INIT, sharedData->system_state, status);
        return;
    }

    if (commandRequiresManualReference(cmd) &&
        !activeManualReferencesReady(sharedData)) {
        sendProtoAckPacket(cmd,
                           sharedData->system_state,
                           PROTO_ACK_STATUS_MANUAL_REFERENCE_REQUIRED);
        return;
    }

    if (cmd == CMD_CALIBRATE)
    {
        if (!activeManualReferencesReady(sharedData)) {
            sendProtoAckPacket(
                CMD_CALIBRATE,
                sharedData->system_state,
                PROTO_ACK_STATUS_MANUAL_REFERENCE_REQUIRED);
        } else {
            postSystemEvent(sharedData, SYSTEM_EVENT_CALIBRATE);
            sendProtoAckPacket(CMD_CALIBRATE,
                               sharedData->system_state,
                               PROTO_ACK_STATUS_QUEUED);
        }
        return;
    }

    if (cmd == CMD_START)
    {
        postSystemEvent(sharedData, SYSTEM_EVENT_START);
        sendProtoAckPacket(CMD_START, sharedData->system_state, PROTO_ACK_STATUS_QUEUED);
        return;
    }

    if (cmd == CMD_RUN_FEEDBACK)
    {
        const uint32_t requestSequence =
            sharedAtomicAddFetch(&sharedData->feedback_admission_sequence, (uint32_t)1);
        sharedAtomicStore(&sharedData->feedback_admission_status,
                          (uint8_t)FEEDBACK_ADMISSION_PENDING);
        sharedAtomicStore(&sharedData->feedback_reject_reason,
                          (uint8_t)FEEDBACK_REJECT_NONE);
        if (!activeManualReferencesReady(sharedData)) {
            sharedAtomicStore(&sharedData->feedback_admission_status,
                              (uint8_t)FEEDBACK_ADMISSION_REJECTED);
            sharedAtomicStore(&sharedData->feedback_reject_reason,
                              (uint8_t)FEEDBACK_REJECT_MANUAL_REFERENCE);
            sendProtoAckPacket(
                CMD_RUN_FEEDBACK,
                sharedData->system_state,
                PROTO_ACK_STATUS_MANUAL_REFERENCE_REQUIRED);
        } else {
            if (!postSystemEventForToken(sharedData,
                                         SYSTEM_EVENT_RUN_FEEDBACK,
                                         requestSequence)) {
                sharedAtomicStore(&sharedData->feedback_admission_status,
                                  (uint8_t)FEEDBACK_ADMISSION_REJECTED);
                sharedAtomicStore(&sharedData->feedback_reject_reason,
                                  (uint8_t)FEEDBACK_REJECT_EVENT_QUEUE);
            }
            sendProtoAckPacket(CMD_RUN_FEEDBACK,
                               sharedData->system_state,
                               PROTO_ACK_STATUS_QUEUED);
        }
        return;
    }

    if (cmd == CMD_PAUSE)
    {
        postSystemEvent(sharedData, SYSTEM_EVENT_PAUSE);
        sendProtoAckPacket(CMD_PAUSE, sharedData->system_state, PROTO_ACK_STATUS_QUEUED);
        return;
    }

    if (cmd == CMD_STOP)
    {
        taskRuntimeMarkStopReceived();
        const bool accepted = requestEmergencyStop(sharedData);
        sendProtoAckPacket(CMD_STOP,
                           sharedData->system_state,
                           accepted ? PROTO_ACK_STATUS_QUEUED
                                    : PROTO_ACK_STATUS_UNSUPPORTED_MODE);
        return;
    }

    if (cmd == CMD_RESET)
    {
        postSystemEvent(sharedData, SYSTEM_EVENT_RESET);
        sendProtoAckPacket(CMD_RESET, sharedData->system_state, PROTO_ACK_STATUS_QUEUED);
        return;
    }

    if (cmd == CMD_ANGLE_CTRL)
    {
        float parsedAngles[ENCODER_TOTAL_NUM] = {0.0f};
        if (parseFloatArrayLittleEndian(frame + 1, frameLen - 1, parsedAngles, ENCODER_TOTAL_NUM)) {
            upperApplyTargetAngles(sharedData, parsedAngles, ENCODER_TOTAL_NUM);
        }
        return;
    }

    if (cmd == CMD_ANGLE_CTRL_MASKED)
    {
        if (frameLen < 1 + kMaskedFloatPayloadBytes) {
            return;
        }
        const uint32_t jointMask =
            ((uint32_t)frame[1] << 24) |
            ((uint32_t)frame[2] << 16) |
            ((uint32_t)frame[3] << 8) |
            (uint32_t)frame[4];
        float parsedAngles[ENCODER_TOTAL_NUM] = {0.0f};
        if (parseFloatArrayLittleEndian(frame + 5,
                                        frameLen - 5,
                                        parsedAngles,
                                        ENCODER_TOTAL_NUM)) {
            upperApplyTargetAnglesMasked(sharedData,
                                         parsedAngles,
                                         ENCODER_TOTAL_NUM,
                                         jointMask);
        }
        return;
    }

    if (cmd == CMD_MOTOR_POS)
    {
        int32_t motorTargets[SERVO_TOTAL_NUM] = {0};
        if (parseInt16ArrayBigEndian(frame + 1, frameLen - 1, motorTargets, SERVO_TOTAL_NUM)) {
            upperApplyMotorTargets(sharedData, motorTargets, SERVO_TOTAL_NUM, MOTOR_DIRECT_SOURCE_TARGET);
        }
        return;
    }

    if (cmd == CMD_MOTOR_POS_SWEEP)
    {
        int32_t motorTargets[SERVO_TOTAL_NUM] = {0};
        if (parseInt16ArrayBigEndian(frame + 1, frameLen - 1, motorTargets, SERVO_TOTAL_NUM)) {
            upperApplyMotorSweepTargets(sharedData, motorTargets, SERVO_TOTAL_NUM);
        }
        return;
    }

    if (cmd == CMD_MOTOR_POS_ABS)
    {
        int32_t motorTargets[SERVO_TOTAL_NUM] = {0};
        if (parseInt32ArrayBigEndian(frame + 1,
                                     frameLen - 1,
                                     motorTargets,
                                     SERVO_TOTAL_NUM)) {
            upperApplyMotorTargets(sharedData, motorTargets, SERVO_TOTAL_NUM, MOTOR_DIRECT_SOURCE_ABSOLUTE);
        }
        return;
    }

    if (cmd == CMD_MOTOR_POS_ABS_ONE)
    {
        if (frameLen >= 1 + kMotorPosAbsOnePayloadBytes) {
            const uint8_t motorIndex = frame[1];
            upperApplySingleMotorTarget(sharedData,
                                        motorIndex,
                                        readI32BigEndian(frame + 2),
                                        MOTOR_DIRECT_SOURCE_ABSOLUTE);
        }
        return;
    }


    if (cmd == CMD_ACTIVE_SCOPE)
    {
        uint8_t status = PROTO_ACK_STATUS_OK;
        if (frameLen < 1 + kActiveScopePayloadBytes) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else {
            const uint32_t activeMotorMask = readU32BigEndian(frame + 1);
            const uint32_t activeJointMask = readU32BigEndian(frame + 5);
            const uint8_t requireCanFeedback = frame[9] ? 1 : 0;
            if (activeMotorMask == 0) {
                status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
            } else {
                if (!upperApplyActiveScope(sharedData,
                                           activeMotorMask,
                                           activeJointMask,
                                           requireCanFeedback)) {
                    status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
                }
            }
        }
        if (status == PROTO_ACK_STATUS_OK) {
            postSystemEvent(sharedData, SYSTEM_EVENT_REEVALUATE_READY);
        }
        sendProtoAckPacket(CMD_ACTIVE_SCOPE, sharedData->system_state, status);
        return;
    }

    if (cmd == CMD_RESTORE_MULTITURN)
    {
        uint8_t status = PROTO_ACK_STATUS_OK;
        if (frameLen < 1 + kRestoreMultiTurnPayloadBytes) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else if ((sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap) &
                    validMotorChannelMask()) == 0) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
            FW_LOG_INFO_PRINTLN(
                "[UPPER] RESTORE_MULTITURN rejected: all motor turns are already trusted");
        } else {
            const uint32_t motorMask = readU32BigEndian(frame + 1);
            const uint32_t unknownMask =
                sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap) &
                validMotorChannelMask();
            const uint32_t onlineMask =
                sharedAtomicLoad(&sharedData->servo_online_bitmap) &
                validMotorChannelMask();
            const uint32_t configuredMask =
                sharedAtomicLoad(&sharedData->servo_multiturn_configured_bitmap) &
                validMotorChannelMask();
            const uint32_t recoveryPendingMask =
                sharedAtomicLoad(&sharedData->servo_recovery_pending_bitmap) &
                validMotorChannelMask();
            const uint32_t recoveryFailedMask =
                sharedAtomicLoad(&sharedData->servo_recovery_failed_bitmap) &
                validMotorChannelMask();
            // 自动恢复仍在重试时不能插入人工 turn0，避免两条恢复路径竞争。
            // 已经明确失败的舵机必须允许 New Device Reset 接管，否则
            // recoveryFailed 会把唯一的人工恢复入口永久锁死。
            const uint32_t recoveryBusyMask =
                recoveryPendingMask & ~recoveryFailedMask;
            int32_t motorAbs[SERVO_TOTAL_NUM] = {0};
            // 局部 New Device 只处理“在线、多圈模式已配置、当前确实 Unknown”
            // 的舵机；自动恢复失败允许人工接管，但正常可信舵机仍不能重定义 turn0。
            if (motorMask == 0 ||
                (motorMask & ~unknownMask) != 0 ||
                (motorMask & ~onlineMask) != 0 ||
                (motorMask & ~configuredMask) != 0 ||
                (motorMask & recoveryBusyMask) != 0) {
                status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
                FW_LOG_WARN_PRINTF(
                    "[UPPER] RESTORE_MULTITURN scoped reject request=0x%08lX unknown=0x%08lX online=0x%08lX configured=0x%08lX recoveryBusy=0x%08lX recoveryFailed=0x%08lX\r\n",
                    (unsigned long)motorMask,
                    (unsigned long)unknownMask,
                    (unsigned long)onlineMask,
                    (unsigned long)configuredMask,
                    (unsigned long)recoveryBusyMask,
                    (unsigned long)recoveryFailedMask);
            } else if (!parseInt32ArrayBigEndian(frame + 5,
                                                 frameLen - 5,
                                                 motorAbs,
                                                 SERVO_TOTAL_NUM)) {
                status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
            } else {
                SemaphoreHandle_t lock = sharedData->commandStateMutex;
                if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
                    status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
                } else {
                    memcpy(sharedData->restore_multi_turn_abs,
                           motorAbs,
                           sizeof(sharedData->restore_multi_turn_abs));
                    sharedData->restore_multi_turn_mask =
                        motorMask & validMotorChannelMask();
                    sharedAtomicAddFetch(&sharedData->restore_multi_turn_request_token, (uint32_t)1);
                    if (lock) {
                        xSemaphoreGive(lock);
                    }
                }
            }
        }
        FW_LOG_INFO_PRINTF("[UPPER] RESTORE_MULTITURN request status=%u mask=0x%08lX token=%lu\r\n",
                      (unsigned)status,
                      (unsigned long)sharedData->restore_multi_turn_mask,
                      (unsigned long)sharedAtomicLoad(&sharedData->restore_multi_turn_request_token));
        sendProtoAckPacket(CMD_RESTORE_MULTITURN, sharedData->system_state, status);
        return;
    }

    if (cmd == CMD_MOTOR_ABS_LIMIT_LOCK)
    {
        uint8_t status = PROTO_ACK_STATUS_OK;
        if (frameLen < 1 + kMotorAbsLimitLockPayloadBytes) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else {
            const bool lockEnable = frame[1] != 0;
            const uint32_t requestedMask = readU32BigEndian(frame + 2) & validMotorChannelMask();
            uint32_t validMask = 0;
            uint32_t lockedMask = 0;
            {
                SharedSafetyGuard guard;
                validMask = sharedAtomicLoad(&sharedData->motor_abs_limit_valid_bitmap);
                if (lockEnable) {
                    sharedAtomicFetchOr(&sharedData->motor_abs_limit_locked_bitmap,
                                        requestedMask & validMask);
                } else {
                    sharedAtomicFetchAnd(&sharedData->motor_abs_limit_locked_bitmap,
                                         ~requestedMask);
                }
                lockedMask = sharedAtomicLoad(&sharedData->motor_abs_limit_locked_bitmap);
            }
            FW_LOG_INFO_PRINTF("[UPPER] ABS_LIMIT_%s mask=0x%08lX locked=0x%08lX valid=0x%08lX\r\n",
                          lockEnable ? "LOCK" : "UNLOCK",
                          (unsigned long)requestedMask,
                          (unsigned long)lockedMask,
                          (unsigned long)validMask);
        }
        sendProtoAckPacket(CMD_MOTOR_ABS_LIMIT_LOCK, sharedData->system_state, status);
        return;
    }

    if (cmd == CMD_CALIB_LOAD_LIMITS)
    {
        uint8_t status = PROTO_ACK_STATUS_OK;
        if (frameLen != 1 + kCalibLoadLimitsPayloadBytes) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else if (sharedData->system_state == SYSTEM_STATE_RUN_DIRECT ||
                   sharedData->system_state == SYSTEM_STATE_RUN_FEEDBACK ||
                   sharedData->system_state == SYSTEM_STATE_CALIBRATION_RUNNING ||
                   sharedData->system_state == SYSTEM_STATE_ROPE_INIT_RUNNING) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else {
            const uint32_t updateMask =
                readU32BigEndian(frame + 1) & validMotorChannelMask();
            const uint32_t validMask =
                readU32BigEndian(frame + 5) & validMotorChannelMask();
            const uint32_t lockedMask =
                readU32BigEndian(frame + 9) & validMotorChannelMask();
            int32_t loadedMin[SERVO_TOTAL_NUM] = {0};
            int32_t loadedMax[SERVO_TOTAL_NUM] = {0};
            const uint8_t* p = frame + 13;
            for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
                loadedMin[ch] = readI32BigEndian(p);
                p += 4;
                loadedMax[ch] = readI32BigEndian(p);
                p += 4;
            }
            if (updateMask == 0) {
                status = PROTO_ACK_STATUS_INVALID_DATA;
            } else {
                // 只覆盖快照声明的电机，保留其余关节已经加载或刚标定的 ABS 限位。
                SharedSafetyGuard guard;
                const uint32_t previousValid =
                    sharedAtomicLoad(&sharedData->motor_abs_limit_valid_bitmap);
                const uint32_t previousLocked =
                    sharedAtomicLoad(&sharedData->motor_abs_limit_locked_bitmap);
                for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
                    if ((updateMask & (1UL << ch)) == 0) {
                        continue;
                    }
                    sharedData->motor_abs_limit_min[ch] = loadedMin[ch];
                    sharedData->motor_abs_limit_max[ch] = loadedMax[ch];
                }
                const uint32_t appliedValidMask =
                    (previousValid & ~updateMask) | (validMask & updateMask);
                const uint32_t appliedLockedMask =
                    (previousLocked & ~updateMask) |
                    (lockedMask & validMask & updateMask);
                sharedAtomicStore(&sharedData->motor_abs_limit_valid_bitmap,
                                  appliedValidMask);
                sharedAtomicStore(&sharedData->motor_abs_limit_locked_bitmap,
                                  appliedLockedMask);
                FW_LOG_INFO_PRINTF(
                    "[UPPER] CALIB_LOAD_LIMITS update=0x%08lX valid=0x%08lX locked=0x%08lX\r\n",
                    (unsigned long)updateMask,
                    (unsigned long)appliedValidMask,
                    (unsigned long)appliedLockedMask);
            }
        }
        sendProtoAckPacket(CMD_CALIB_LOAD_LIMITS, sharedData->system_state, status);
        return;
    }

    if (cmd == CMD_CALIB_LOAD_JOINT)
    {
        uint8_t status = PROTO_ACK_STATUS_OK;
        if (frameLen != 1 + kCalibLoadJointPayloadBytes) {
            status = PROTO_ACK_STATUS_INVALID_DATA;
        } else if (sharedData->system_state == SYSTEM_STATE_RUN_DIRECT ||
                   sharedData->system_state == SYSTEM_STATE_RUN_FEEDBACK ||
                   sharedData->system_state == SYSTEM_STATE_CALIBRATION_RUNNING ||
                   sharedData->system_state == SYSTEM_STATE_ROPE_INIT_RUNNING) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else {
            const uint8_t* p = frame + 1;
            const uint8_t jointIndex = *p++;
            if (jointIndex >= ENCODER_TOTAL_NUM) {
                status = PROTO_ACK_STATUS_INVALID_DATA;
            } else {
                JointCalibrationResult result;
                memset(&result, 0, sizeof(result));
                result.success = (*p++ != 0);
                result.encoderMin = readI32BigEndian(p); p += 4;
                result.encoderMax = readI32BigEndian(p); p += 4;
                result.offset = readI32BigEndian(p); p += 4;
                result.angleMax = readFloatBigEndian(p); p += 4;
                result.startEncoder = readI32BigEndian(p); p += 4;
                result.tightEncoder = readI32BigEndian(p); p += 4;
                result.zeroEncoder = readI32BigEndian(p); p += 4;
                result.encoderSpan = readI32BigEndian(p); p += 4;
                result.startServoAbs = readI32BigEndian(p); p += 4;
                result.tightServoAbs = readI32BigEndian(p); p += 4;
                result.zeroServoAbs = readI32BigEndian(p); p += 4;
                result.startDeg = readFloatBigEndian(p); p += 4;
                result.tightDeg = readFloatBigEndian(p); p += 4;
                result.totalDeg = readFloatBigEndian(p); p += 4;
                result.scaled = (*p++ != 0);
                const int8_t direction = ((int8_t)(*p++)) < 0 ? -1 : 1;
                result.tendonLutValid = (*p++ != 0) ? 1 : 0;
                result.tendonLutCount = *p++;
                result.tendonLutMotor[0] = *p++;
                result.tendonLutMotor[1] = *p++;
                result.tendonLutAngleStartDeg = readFloatBigEndian(p); p += 4;
                result.tendonLutAngleStepDeg = readFloatBigEndian(p); p += 4;
                for (uint8_t actuator = 0;
                     actuator < TENDON_LUT_ACTUATOR_COUNT; actuator++) {
                    const int32_t anchor = readI32BigEndian(p);
                    p += 4;
                    for (uint8_t sample = 0;
                         sample < TENDON_LUT_MAX_SAMPLES; sample++) {
                        const int16_t encoded = readI16BigEndian(p);
                        p += 2;
                        result.tendonLutMotorAbs[actuator][sample] =
                            anchor + (int32_t)encoded;
                    }
                }
                const uint8_t incomingMappingVersion = *p++;
                result.encoderAtUrdfMin = readI32BigEndian(p); p += 4;
                result.encoderAtUrdfMax = readI32BigEndian(p); p += 4;
                result.angleMin = readFloatBigEndian(p); p += 4;
                const uint32_t incomingUrdfCrc = readU32BigEndian(p); p += 4;
                const uint16_t autoMinRaw = readU16BigEndian(p); p += 2;
                const uint16_t autoMaxRaw = readU16BigEndian(p); p += 2;
                const uint32_t incomingManualCrc = readU32BigEndian(p); p += 4;
                const float incomingCurrentMin = readFloatBigEndian(p); p += 4;
                const float incomingCurrentMax = readFloatBigEndian(p); p += 4;
                (void)incomingCurrentMin;
                (void)incomingCurrentMax;

                const uint32_t localManualEntryCrc =
                    manualEncoderReferenceEntryCrc(
                        jointIndex, g_manualEncoderReference[jointIndex]);
                const uint32_t localValidMask =
                    sharedAtomicLoad(&sharedData->manual_reference_valid_bitmap);
                // V3 Auto 端点必须是物理 14-bit raw；方向化标定坐标不能写入这些字段，
                // 否则 direction=-1 关节会产生错误的补码跨度。
                const uint16_t expectedAutoMinRaw =
                    (uint16_t)calibrationOrientEncoderRaw(
                        (uint16_t)calibrationWrapEncoderCount(
                            result.startEncoder),
                        direction);
                const uint16_t expectedAutoMaxRaw =
                    (uint16_t)calibrationOrientEncoderRaw(
                        (uint16_t)calibrationWrapEncoderCount(
                            result.tightEncoder),
                        direction);
                if (incomingMappingVersion !=
                        kManualScaledUrdfFeedbackMappingVersion ||
                    incomingUrdfCrc != kManualScaledUrdfConfigCrc) {
                    status = PROTO_ACK_STATUS_CALIBRATION_REQUIRED;
                } else if (incomingManualCrc != localManualEntryCrc) {
                    status = PROTO_ACK_STATUS_MANUAL_CRC_MISMATCH;
                } else if ((localValidMask & (1UL << jointIndex)) == 0) {
                    status = PROTO_ACK_STATUS_MANUAL_REFERENCE_REQUIRED;
                } else if (autoMinRaw != expectedAutoMinRaw ||
                           autoMaxRaw != expectedAutoMaxRaw) {
                    FW_LOG_WARN_PRINTF(
                        "[UPPER] CALIB_LOAD reject J%02u raw endpoint "
                        "contract: incoming=%u->%u expected=%u->%u dir=%d; "
                        "recalibration required\r\n",
                        (unsigned)jointIndex,
                        (unsigned)autoMinRaw,
                        (unsigned)autoMaxRaw,
                        (unsigned)expectedAutoMinRaw,
                        (unsigned)expectedAutoMaxRaw,
                        (int)direction);
                    status = PROTO_ACK_STATUS_INVALID_DATA;
                } else if (!applyManualScaledUrdfMappingToCalibration(
                               jointIndex,
                               g_manualEncoderReference[jointIndex],
                               autoMinRaw,
                               autoMaxRaw,
                               direction,
                               localManualEntryCrc,
                               &result)) {
                    status = PROTO_ACK_STATUS_INVALID_DATA;
                } else if (result.success &&
                           (!isfinite(result.startDeg) ||
                            !isfinite(result.tightDeg) ||
                            !isfinite(result.totalDeg) ||
                            result.encoderSpan <= 0 || result.totalDeg <= 0.0f ||
                            !calibrationTendonLutMatchesJoint(jointIndex,
                                                             result))) {
                    status = PROTO_ACK_STATUS_INVALID_DATA;
                } else {
                    g_jointCalibResult[jointIndex] = result;
                    setRuntimeEncoderDirection(jointIndex, direction);
                    if (result.success) {
                        updateCalibrationAssemblyWarning(
                            sharedData, jointIndex, result);
                        const uint8_t loadedStatus =
                            sharedAtomicLoad(
                                &sharedData->calibration_warning_bitmap) != 0
                                ? CALIB_STATUS_WARN
                                : CALIB_STATUS_SUCCESS;
                        g_calibrationStatus = loadedStatus;
                        g_calibrationAckStatus = loadedStatus;
                        g_calibrationAckRepeatRemaining = 5;
                        sharedAtomicFetchAnd(
                            &sharedData->system_fault_bitmap,
                            (uint32_t)~SYSTEM_FAULT_CALIBRATION);
                    }
                    FW_LOG_INFO_PRINTF(
                        "[UPPER] CALIB_LOAD J%02u auto=%u->%u manualCrc=0x%08lX current=%.3f..%.3f\r\n",
                        (unsigned)jointIndex,
                        (unsigned)result.autoMinRaw,
                        (unsigned)result.autoMaxRaw,
                        (unsigned long)result.manualReferenceCrc,
                        result.currentMinDeg,
                        result.currentMaxDeg);
                }
            }
        }
        sendProtoAckPacket(CMD_CALIB_LOAD_JOINT,
                           sharedData->system_state,
                           status);
        if (status == PROTO_ACK_STATUS_OK) {
            postSystemEvent(sharedData, SYSTEM_EVENT_REEVALUATE_READY);
        }
        return;
    }
    if (cmd == CMD_MANUAL_ENCODER_SINGLE)
    {
        uint8_t status = PROTO_ACK_STATUS_OK;
        const uint8_t joint = frame[1];
        const uint16_t minRaw = readU16BigEndian(frame + 2);
        const uint16_t maxRaw = readU16BigEndian(frame + 4);
        if (manualUpdateMotionActive(sharedData)) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else {
            if (joint >= ENCODER_TOTAL_NUM ||
                minRaw >= kEncoderModulo || maxRaw >= kEncoderModulo) {
                status = PROTO_ACK_STATUS_INVALID_DATA;
            } else {
                ManualEncoderReference references[ENCODER_TOTAL_NUM];
                {
                    SharedSafetyGuard guard;
                    memcpy(references, g_manualEncoderReference,
                           sizeof(references));
                }
                references[joint].valid = 1;
                references[joint].minRaw = minRaw;
                references[joint].maxRaw = maxRaw;
                if (!manualEncoderReferencesCoverMask(
                        references, (1UL << joint), NULL)) {
                    status = PROTO_ACK_STATUS_INVALID_DATA;
                } else {
                    status = commitManualReferences(sharedData, references);
                }
            }
        }
        sendProtoAckPacket(CMD_MANUAL_ENCODER_SINGLE,
                           sharedData->system_state,
                           status);
        if (status == PROTO_ACK_STATUS_OK) {
            sendManualEncoderReferencePacket(sharedData);
        }
        FW_LOG_INFO_PRINTF(
            "[NVS MANUAL] single J%02u min=%u max=%u status=%u "
            "nvs=%u save=%u valid=0x%08lX crc=0x%08lX seq=%lu\r\n",
            (unsigned)joint,
            (unsigned)minRaw,
            (unsigned)maxRaw,
            (unsigned)status,
            (unsigned)sharedData->manual_reference_nvs_valid,
            (unsigned)sharedData->manual_reference_save_status,
            (unsigned long)sharedAtomicLoad(
                &sharedData->manual_reference_valid_bitmap),
            (unsigned long)sharedAtomicLoad(
                &sharedData->manual_reference_crc),
            (unsigned long)sharedAtomicLoad(
                &sharedData->manual_reference_nvs_sequence));
        return;
    }
    if (cmd == CMD_MANUAL_ENCODER_REFERENCE)
    {
        uint8_t status = PROTO_ACK_STATUS_OK;
        const uint8_t action = frame[1];
        const bool motionActive =
            sharedData->control_enabled != 0 ||
            sharedData->servo_target_owner != SERVO_TARGET_OWNER_NONE ||
            sharedData->system_state == SYSTEM_STATE_RUN_DIRECT ||
            sharedData->system_state == SYSTEM_STATE_RUN_FEEDBACK ||
            sharedData->system_state == SYSTEM_STATE_CALIBRATION_RUNNING ||
            sharedData->system_state == SYSTEM_STATE_ROPE_INIT_RUNNING ||
            sharedData->system_state == SYSTEM_STATE_PAUSED;

        if (motionActive) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else if (action == 0) {
            if (!manualCalibrationNvsClear()) {
                status = PROTO_ACK_STATUS_NVS_FAILURE;
                sharedData->manual_reference_save_status =
                    MANUAL_SAVE_STATUS_FAILED;
            } else {
                sharedData->manual_reference_nvs_valid = 0;
                manualEncoderReferencesClearRam();
                sharedAtomicStore(&sharedData->manual_reference_valid_bitmap,
                                  (uint32_t)0);
                sharedAtomicStore(&sharedData->manual_reference_crc,
                                  (uint32_t)0);
                sharedAtomicStore(&sharedData->manual_reference_nvs_sequence,
                                  (uint32_t)0);
                sharedData->manual_reference_save_status =
                    MANUAL_SAVE_STATUS_NONE;
                invalidateAutomaticCalibration(
                    sharedData, kManualEncoderReferenceFullMask);
                postSystemEvent(sharedData, SYSTEM_EVENT_REEVALUATE_READY);
            }
        } else if (action == 1) {
            ManualEncoderReference references[ENCODER_TOTAL_NUM];
            const uint8_t* p = frame + 2;
            for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; joint++) {
                references[joint].valid = 1;
                references[joint].minRaw = readU16BigEndian(p);
                p += 2;
                references[joint].maxRaw = readU16BigEndian(p);
                p += 2;
            }

            uint32_t validMask = 0;
            if (!manualEncoderReferencesValidate(references, &validMask)) {
                status = PROTO_ACK_STATUS_INVALID_DATA;
            } else {
                status = commitManualReferences(sharedData, references);
            }
        } else {
            status = PROTO_ACK_STATUS_INVALID_DATA;
        }
        FW_LOG_INFO_PRINTF(
            "[NVS MANUAL] action=%u status=%u valid=0x%08lX "
            "crc=0x%08lX seq=%lu nvs=%u save=%u\r\n",
            (unsigned)action,
            (unsigned)status,
            (unsigned long)sharedAtomicLoad(
                &sharedData->manual_reference_valid_bitmap),
            (unsigned long)sharedAtomicLoad(
                &sharedData->manual_reference_crc),
            (unsigned long)sharedAtomicLoad(
                &sharedData->manual_reference_nvs_sequence),
            (unsigned)sharedData->manual_reference_nvs_valid,
            (unsigned)sharedData->manual_reference_save_status);
        sendProtoAckPacket(CMD_MANUAL_ENCODER_REFERENCE,
                           sharedData->system_state,
                           status);
        return;
    }
    if (cmd == CMD_CLEAR_CALIBRATION)
    {
        uint8_t status = PROTO_ACK_STATUS_OK;
        if (sharedData->system_state == SYSTEM_STATE_RUN_DIRECT ||
            sharedData->system_state == SYSTEM_STATE_RUN_FEEDBACK ||
            sharedData->system_state == SYSTEM_STATE_CALIBRATION_RUNNING ||
            sharedData->system_state == SYSTEM_STATE_ROPE_INIT_RUNNING) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else {
            {
                SharedSafetyGuard guard;
                sharedAtomicStore(&sharedData->motor_abs_limit_valid_bitmap, (uint32_t)0);
                sharedAtomicStore(&sharedData->motor_abs_limit_locked_bitmap, (uint32_t)0);
                memset(sharedData->motor_abs_limit_min, 0, sizeof(sharedData->motor_abs_limit_min));
                memset(sharedData->motor_abs_limit_max, 0, sizeof(sharedData->motor_abs_limit_max));
            }
            memset(g_jointCalibResult, 0, sizeof(g_jointCalibResult));
            clearCalibrationAssemblyWarnings(
                sharedData, kManualEncoderReferenceFullMask);
            g_calibrationStatus = CALIB_STATUS_IDLE;
            g_calibrationAckStatus = CALIB_STATUS_IDLE;
            g_calibrationAckRepeatRemaining = 0;
            sharedAtomicFetchAnd(&sharedData->system_fault_bitmap,
                                 (uint32_t)~SYSTEM_FAULT_CALIBRATION);
            postSystemEvent(sharedData, SYSTEM_EVENT_REEVALUATE_READY);
            FW_LOG_INFO_PRINTLN("[UPPER] CLEAR_CALIBRATION: joint calibration and ABS limits cleared");
        }
        sendProtoAckPacket(CMD_CLEAR_CALIBRATION, sharedData->system_state, status);
        return;
    }

    if (cmd == CMD_CLEAR_CALIBRATION_SCOPE)
    {
        uint8_t status = PROTO_ACK_STATUS_OK;
        const uint32_t motorMask =
            frameLen >= 1 + kClearCalibrationScopePayloadBytes
                ? (readU32BigEndian(frame + 1) & validMotorChannelMask())
                : 0;
        const uint32_t restoredMask =
            sharedAtomicLoad(&sharedData->restore_multi_turn_restored_mask) &
            validMotorChannelMask();
        const uint32_t jointMask = jointMaskForMotorMask(motorMask);
        if (sharedData->system_state == SYSTEM_STATE_RUN_DIRECT ||
            sharedData->system_state == SYSTEM_STATE_RUN_FEEDBACK ||
            sharedData->system_state == SYSTEM_STATE_CALIBRATION_RUNNING ||
            sharedData->system_state == SYSTEM_STATE_ROPE_INIT_RUNNING) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else if (motorMask == 0 || jointMask == 0 ||
                   (motorMask & ~restoredMask) != 0) {
            status = PROTO_ACK_STATUS_INVALID_DATA;
            FW_LOG_WARN_PRINTF(
                "[UPPER] CLEAR_CALIBRATION_SCOPE reject motors=0x%08lX restored=0x%08lX joints=0x%08lX\r\n",
                (unsigned long)motorMask,
                (unsigned long)restoredMask,
                (unsigned long)jointMask);
        } else {
            invalidateAutomaticCalibration(sharedData, jointMask);
            sharedAtomicFetchAnd(
                &sharedData->restore_multi_turn_restored_mask, ~motorMask);
            postSystemEvent(sharedData, SYSTEM_EVENT_REEVALUATE_READY);
            FW_LOG_INFO_PRINTF(
                "[UPPER] CLEAR_CALIBRATION_SCOPE motors=0x%08lX joints=0x%08lX remainingAuto=0x%08lX\r\n",
                (unsigned long)motorMask,
                (unsigned long)jointMask,
                (unsigned long)autoCalibrationValidMask(sharedData));
        }
        sendProtoAckPacket(
            CMD_CLEAR_CALIBRATION_SCOPE, sharedData->system_state, status);
        return;
    }

    if (cmd == CMD_SERVO_RESET_ZERO)
    {
        const uint32_t requestedMask =
            readU32BigEndian(frame + 1) & validMotorChannelMask();
        const uint32_t overflowMask =
            sharedAtomicLoad(&sharedData->servo_turn_overflow_bitmap) &
            validMotorChannelMask();
        const uint32_t onlineMask =
            sharedAtomicLoad(&sharedData->servo_online_bitmap) &
            validMotorChannelMask();
        uint8_t status = PROTO_ACK_STATUS_QUEUED;
        const bool safeState =
            sharedData->system_state == SYSTEM_STATE_UNCONTROLLABLE ||
            sharedData->system_state == SYSTEM_STATE_IDLE ||
            sharedData->system_state == SYSTEM_STATE_READY ||
            sharedData->system_state == SYSTEM_STATE_RECOVERY_REQUIRED;
        if (requestedMask == 0 ||
            (requestedMask & ~overflowMask) != 0 ||
            (requestedMask & ~onlineMask) != 0) {
            status = PROTO_ACK_STATUS_INVALID_DATA;
        } else if (!safeState ||
                   sharedData->control_enabled != 0 ||
                   sharedData->servo_target_owner !=
                       SERVO_TARGET_OWNER_NONE) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else if (!sharedData->multiturn_nvs_enabled ||
                   !sharedData->multiturnNvsSaveQueue) {
            status = PROTO_ACK_STATUS_NVS_FAILURE;
        } else if (sharedAtomicLoad(
                       &sharedData->servo_reset_zero_status) ==
                   SERVO_RESET_ZERO_STATUS_RUNNING) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else {
            sharedAtomicStore(
                &sharedData->servo_reset_zero_request_mask,
                requestedMask);
            sharedAtomicStore(
                &sharedData->servo_reset_zero_success_mask,
                (uint32_t)0);
            sharedAtomicStore(
                &sharedData->servo_reset_zero_failed_mask,
                (uint32_t)0);
            sharedAtomicStore(
                &sharedData->servo_reset_zero_invalidated_joint_mask,
                (uint32_t)0);
            sharedAtomicStore(
                &sharedData->servo_reset_zero_status,
                (uint8_t)SERVO_RESET_ZERO_STATUS_RUNNING);
            sharedAtomicAddFetch(
                &sharedData->servo_reset_zero_request_token,
                (uint32_t)1);
        }
        sendProtoAckPacket(
            CMD_SERVO_RESET_ZERO, sharedData->system_state, status);
        return;
    }

    if (cmd == CMD_MULTITURN_NVS_CONTROL)
    {
        uint8_t status = PROTO_ACK_STATUS_OK;
        bool setOk = false;
        bool clearOk = false;
        if (frameLen < 1 + kMultiTurnNvsControlPayloadBytes) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else {
            const uint8_t action = frame[1];
#if MULTITURN_NVS_PERSISTENCE_ENABLE
            if (action == 0) {
                setOk = multiTurnNvsSetEnabled(false);
                clearOk = multiTurnNvsClearSnapshots();
                if (setOk && clearOk) {
                    sharedData->multiturn_nvs_enabled = 0;
                    sharedData->multiturn_nvs_snapshot_valid = 0;
                    sharedData->multiturn_nvs_save_valid_mask = 0;
                    memset(sharedData->multiturn_nvs_snapshot_raw,
                           0,
                           sizeof(sharedData->multiturn_nvs_snapshot_raw));
                } else {
                    status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
                }
            } else if (action == 1) {
                setOk = multiTurnNvsSetEnabled(true);
                if (setOk) {
                    sharedData->multiturn_nvs_enabled = 1;
                    sharedData->multiturn_nvs_snapshot_valid = multiTurnNvsHasSnapshot() ? 1 : 0;
                } else {
                    status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
                }
            } else {
                status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
            }
#else
            (void)action;
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
#endif
        }
        // 机器可解析 ACK 优先于诊断文本；热插拔恢复时多个任务可能同时输出日志。
        sendProtoAckPacket(CMD_MULTITURN_NVS_CONTROL, sharedData->system_state, status);
        if (frameLen >= 1 + kMultiTurnNvsControlPayloadBytes && frame[1] == 0) {
            if (status == PROTO_ACK_STATUS_OK) {
                FW_LOG_INFO_PRINTLN("[NVS MULTITURN] control enabled=0 clear=1");
            } else {
                FW_LOG_INFO_PRINTF("[NVS MULTITURN] control disable failed set=%u clear=%u\r\n",
                              setOk ? 1U : 0U,
                              clearOk ? 1U : 0U);
            }
        } else if (frameLen >= 1 + kMultiTurnNvsControlPayloadBytes && frame[1] == 1) {
            if (status == PROTO_ACK_STATUS_OK) {
                FW_LOG_INFO_PRINTF("[NVS MULTITURN] control enabled=1 snapshot=%u\r\n",
                              sharedData->multiturn_nvs_snapshot_valid ? 1U : 0U);
            } else {
                FW_LOG_INFO_PRINTLN("[NVS MULTITURN] control enable failed");
            }
        }
        return;
    }

}

// 串口收发主任务：有限状态收包、命令分发、周期遥测和事务结果处理。
void upperCommunicationTask(void* parameter)
{
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    RemoteSensorData_t sensorRawData;
    MappedAngleData_t mappedAngleData;
    uint8_t rxBuffer[kSerialRxBufferSize];
    size_t rxLen = 0;
    uint32_t lastSensorTimestampSent = 0;
    uint32_t lastEncoderRawTimestampSent = 0;

    uint32_t lastFaultStatusSent = 0;
    uint32_t lastFaultStatusSentMs = 0;
    bool faultStatusSentInitialized = false;

    uint32_t lastSystemStatusTransitionSent = 0;
    uint32_t lastFeedbackCollisionTransitionSent = 0;
    uint32_t lastSystemStatusSentMs = 0;
    bool systemStatusSentInitialized = false;
    uint32_t lastManualReferenceSentMs = 0;
    uint32_t lastManualReferenceSequenceSent = 0xFFFFFFFFUL;
    uint32_t lastManualReferenceValidMaskSent = 0xFFFFFFFFUL;
    uint8_t lastManualReferenceNvsValidSent = 0xFF;
    uint32_t lastMotorAbsLimitSentMs = 0;
    uint32_t lastMotorAbsLimitValidSent = 0xFFFFFFFFUL;
    uint32_t lastMotorAbsLimitLockedSent = 0xFFFFFFFFUL;
    uint32_t lastServoLogicalRangeSentMs = 0;
    uint32_t lastServoLogicalRangeValidSent = 0xFFFFFFFFUL;
    uint32_t lastJointCalibSnapshotSentMs = 0;
    uint8_t nextJointCalibSnapshotIndex = 0;
    uint32_t lastCalibrationDiagSequenceSent = 0;
    uint32_t lastCalibrationDiagSentMs = 0;
    uint32_t lastTaskDiagSentMs = 0;
    uint32_t lastProductFingerprintSentMs = 0;
    uint8_t nextTaskDiagId = 0;
    uint32_t lastServoResetZeroResultToken =
        sharedAtomicLoad(&sharedData->servo_reset_zero_result_token);
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t taskPeriodTicks = pdMS_TO_TICKS(5);

    FW_LOG_INFO_PRINTLN("<<<SYS_READY>>>");

    for (;;)
    {
        taskRuntimeMarkLoopStart(TASK_RUNTIME_UPPER_COMM);
        while (Serial.available())
        {
            const int byteVal = Serial.read();
            if (byteVal < 0) {
                break;
            }

            if (rxLen < sizeof(rxBuffer)) {
                rxBuffer[rxLen++] = (uint8_t)byteVal;
            } else {
                memmove(rxBuffer, rxBuffer + 1, sizeof(rxBuffer) - 1);
                rxBuffer[sizeof(rxBuffer) - 1] = (uint8_t)byteVal;
                rxLen = sizeof(rxBuffer);
            }
        }

        size_t parseOffset = 0;
        while (parseOffset < rxLen)
        {
            const uint8_t cur = rxBuffer[parseOffset];

            if (cur != PROTOCOL_HEADER)
            {
                parseOffset += 1;
                continue;
            }

            if ((parseOffset + 4) > rxLen)
            {
                break;
            }

            const uint8_t wireLen = rxBuffer[parseOffset + 1];
            if (wireLen < 2)
            {
                parseOffset += 1;
                continue;
            }

            const size_t frameLen = (size_t)wireLen + 2;
            if ((parseOffset + frameLen) > rxLen)
            {
                break;
            }

            if (rxBuffer[parseOffset + frameLen - 1] != PROTOCOL_TAIL)
            {
                parseOffset += 1;
                continue;
            }

            const uint8_t cmd = rxBuffer[parseOffset + 2];
            const size_t payloadLen = (size_t)wireLen - 2; // CMD + payload + tail
            const size_t expectedPayloadLen = getCommandPayloadLength(cmd);
            if (cmd == CMD_CALIB_LOAD_JOINT) {
                if (payloadLen != kCalibLoadJointPayloadBytes) {
                    parseOffset += frameLen;
                    continue;
                }
            } else if (expectedPayloadLen == (size_t)-1 || expectedPayloadLen != payloadLen)
            {
                parseOffset += frameLen;
                continue;
            }

            handleParsedCommand(sharedData, rxBuffer + parseOffset + 2, payloadLen + 1);
            parseOffset += frameLen;
        }

        if (parseOffset > 0)
        {
            const size_t remain = rxLen - parseOffset;
            if (remain > 0) {
                memmove(rxBuffer, rxBuffer + parseOffset, remain);
            }
            rxLen = remain;
        }

        processServoResetZeroResult(
            sharedData, &lastServoResetZeroResultToken);

        bool sentPacket = false;
        if (sharedData->mappedAngleQueue &&
            xQueuePeek(sharedData->mappedAngleQueue, &mappedAngleData, 0) == pdTRUE &&
            mappedAngleData.timestamp != lastSensorTimestampSent)
        {
            sendDataPacket(NULL, NULL, &mappedAngleData, NULL, NULL, NULL, NULL, NULL);
            lastSensorTimestampSent = mappedAngleData.timestamp;
            sentPacket = true;
        }

        if (sharedData->canRxQueue &&
            xQueuePeek(sharedData->canRxQueue, &sensorRawData, 0) == pdTRUE &&
            sensorRawData.timestamp != lastEncoderRawTimestampSent)
        {
            sendEncoderRawPacket(sensorRawData);
            lastEncoderRawTimestampSent = sensorRawData.timestamp;
        }

        RemoteTactileData_t tactileData;
        if (sharedData->tactileQueue &&
            xQueueReceive(sharedData->tactileQueue, &tactileData, 0) == pdTRUE)
        {
            sendDataPacket(NULL, NULL, NULL, NULL, NULL, NULL, &tactileData, NULL);
            sentPacket = true;
        }

        JointDebugData_t jointDebugData;
        uint8_t jointDebugPacketsSent = 0;
        static const uint8_t kMaxJointDebugPacketsPerLoop = 4;
        while (jointDebugPacketsSent < kMaxJointDebugPacketsPerLoop &&
               xQueueReceive(sharedData->jointDebugQueue, &jointDebugData, 0) == pdTRUE)
        {
            sendDataPacket(NULL, NULL, NULL, NULL, NULL, NULL, NULL, &jointDebugData);
            jointDebugPacketsSent++;
            sentPacket = true;
        }

        ServoAngleData_t servoRawData;
        if (xQueueReceive(sharedData->servoRawQueue, &servoRawData, 0) == pdTRUE)
        {
            sendDataPacket(NULL, NULL, NULL, NULL, &servoRawData, NULL, NULL, NULL);
            sentPacket = true;
        }

        ServoTelemetryData_t telemetryData;
        if (xQueueReceive(sharedData->servoTelemetryQueue, &telemetryData, 0) == pdTRUE)
        {
            sendDataPacket(NULL, NULL, NULL, NULL, NULL, &telemetryData, NULL, NULL);
            sentPacket = true;
        }

        bool sentServoAngle = false;
        ServoAngleData_t servoAngleData;
        if (xQueueReceive(sharedData->servoAngleQueue, &servoAngleData, 0) == pdTRUE)
        {
            sendDataPacket(NULL, NULL, NULL, &servoAngleData, NULL, NULL, NULL, NULL);
            sentServoAngle = true;
            sentPacket = true;
        }

        if (!sentPacket && !sentServoAngle && calibrationAckPending())
        {
            sendDataPacket(NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
        }

        const uint32_t nowMs = millis();
        if (lastProductFingerprintSentMs == 0 ||
            (uint32_t)(nowMs - lastProductFingerprintSentMs) >=
                kProductFingerprintHeartbeatMs) {
            sendProductFingerprintPacket();
            lastProductFingerprintSentMs = nowMs;
        }
        const uint32_t faultBitmap =
            sharedAtomicLoad(&sharedData->overload_fault_bitmap);
        const bool bitmapChanged =
            (!faultStatusSentInitialized) || (faultBitmap != lastFaultStatusSent);
        const bool heartbeatDue =
            (!faultStatusSentInitialized) || ((nowMs - lastFaultStatusSentMs) >= kFaultStatusHeartbeatMs);
        if (bitmapChanged || heartbeatDue) {
            sendFaultStatusPacket(sharedData, faultBitmap);
            lastFaultStatusSent = faultBitmap;
            lastFaultStatusSentMs = nowMs;
            faultStatusSentInitialized = true;
        }


        const uint32_t stateTransitionCount = sharedData->state_transition_count;
        const uint32_t feedbackCollisionTransitionCount =
            sharedAtomicLoad(&sharedData->feedback_collision_transition_count);
        const bool systemStatusChanged =
            (!systemStatusSentInitialized) ||
            (stateTransitionCount != lastSystemStatusTransitionSent) ||
            (feedbackCollisionTransitionCount !=
             lastFeedbackCollisionTransitionSent);
        const bool systemStatusHeartbeatDue =
            (!systemStatusSentInitialized) ||
            ((nowMs - lastSystemStatusSentMs) >= kSystemStatusHeartbeatMs);
        if (systemStatusChanged || systemStatusHeartbeatDue) {
            if (sendSystemStatusPacket(sharedData)) {
                taskRuntimeMarkSystemStatusSent();
                lastSystemStatusTransitionSent = stateTransitionCount;
                lastFeedbackCollisionTransitionSent =
                    feedbackCollisionTransitionCount;
                lastSystemStatusSentMs = nowMs;
                systemStatusSentInitialized = true;
            }
        }

        const uint32_t manualReferenceSequence =
            sharedAtomicLoad(&sharedData->manual_reference_nvs_sequence);
        const uint32_t manualReferenceValidMask =
            sharedAtomicLoad(&sharedData->manual_reference_valid_bitmap);
        const uint8_t manualReferenceNvsValid =
            sharedData->manual_reference_nvs_valid ? 1 : 0;
        const bool manualReferenceChanged =
            manualReferenceSequence != lastManualReferenceSequenceSent ||
            manualReferenceValidMask != lastManualReferenceValidMaskSent ||
            manualReferenceNvsValid != lastManualReferenceNvsValidSent;
        const bool manualReferenceHeartbeatDue =
            (uint32_t)(nowMs - lastManualReferenceSentMs) >=
            kManualEncoderReferenceHeartbeatMs;
        if (manualReferenceChanged || manualReferenceHeartbeatDue) {
            if (sendManualEncoderReferencePacket(sharedData)) {
                lastManualReferenceSequenceSent = manualReferenceSequence;
                lastManualReferenceValidMaskSent = manualReferenceValidMask;
                lastManualReferenceNvsValidSent = manualReferenceNvsValid;
                lastManualReferenceSentMs = nowMs;
            }
        }
        if ((uint32_t)(nowMs - lastTaskDiagSentMs) >= kTaskDiagHeartbeatMs) {
            TaskRuntimeSnapshot runtimeDiag;
            if (taskRuntimeGetSnapshot(nextTaskDiagId, &runtimeDiag)) {
                sendTaskRuntimePacket(runtimeDiag);
            }
            nextTaskDiagId = (uint8_t)((nextTaskDiagId + 1) % TASK_RUNTIME_COUNT);
            lastTaskDiagSentMs = nowMs;
        }

        const uint32_t limitValidBitmap =
            sharedAtomicLoad(&sharedData->motor_abs_limit_valid_bitmap);
        const uint32_t limitLockedBitmap =
            sharedAtomicLoad(&sharedData->motor_abs_limit_locked_bitmap);
        if (limitValidBitmap != lastMotorAbsLimitValidSent ||
            limitLockedBitmap != lastMotorAbsLimitLockedSent ||
            (uint32_t)(nowMs - lastMotorAbsLimitSentMs) >= kMotorAbsLimitHeartbeatMs) {
            sendMotorAbsLimitPacket(sharedData);
            lastMotorAbsLimitValidSent = limitValidBitmap;
            lastMotorAbsLimitLockedSent = limitLockedBitmap;
            lastMotorAbsLimitSentMs = nowMs;
        }

        const uint32_t logicalRangeValidBitmap =
            sharedAtomicLoad(&sharedData->servo_logical_range_valid_bitmap);
        if (logicalRangeValidBitmap != lastServoLogicalRangeValidSent ||
            (uint32_t)(nowMs - lastServoLogicalRangeSentMs) >=
                kServoLogicalRangeHeartbeatMs) {
            sendServoLogicalRangePacket(sharedData);
            lastServoLogicalRangeValidSent = logicalRangeValidBitmap;
            lastServoLogicalRangeSentMs = nowMs;
        }

        if (calibrationStatusHasUsableResult(g_calibrationStatus) &&
            (uint32_t)(nowMs - lastJointCalibSnapshotSentMs) >= kJointCalibSnapshotHeartbeatMs) {
            for (uint8_t n = 0; n < ENCODER_TOTAL_NUM; n++) {
                const uint8_t jointIndex = (uint8_t)((nextJointCalibSnapshotIndex + n) % ENCODER_TOTAL_NUM);
                if (g_jointCalibResult[jointIndex].success) {
                    sendJointCalibSnapshotPacket(jointIndex);
                }
            }
            nextJointCalibSnapshotIndex = 0;
            lastJointCalibSnapshotSentMs = nowMs;
        }

        CalibrationDiagData_t calibDiag;
        {
            SharedSafetyGuard guard;
            calibDiag = sharedData->calibration_diag;
        }
        if (calibDiag.sequence != lastCalibrationDiagSequenceSent ||
            (calibDiag.valid &&
             (uint32_t)(nowMs - lastCalibrationDiagSentMs) >= kSystemStatusHeartbeatMs)) {
            sendCalibrationDiagPacket(calibDiag);
            lastCalibrationDiagSequenceSent = calibDiag.sequence;
            lastCalibrationDiagSentMs = nowMs;
        }

        taskRuntimeMarkLoopEnd(TASK_RUNTIME_UPPER_COMM);
        const TickType_t nextWakeTime = lastWakeTime + taskPeriodTicks;
        const bool periodOverrun = (int32_t)(xTaskGetTickCount() - nextWakeTime) >= 0;
        taskRuntimeMarkPeriodResult(TASK_RUNTIME_UPPER_COMM, periodOverrun);
        vTaskDelayUntil(&lastWakeTime, taskPeriodTicks);
        if (periodOverrun) vTaskDelay(1);
    }
}
