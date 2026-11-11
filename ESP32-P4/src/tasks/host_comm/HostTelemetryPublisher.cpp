/**
 * @file HostTelemetryPublisher.cpp
 * @brief 实现 P4 状态快照的协议编码与上位机遥测发送。
 * @details 本文件不处理下行命令，也不改变系统状态；所有输出统一经过 HostSerialPort。
 */
#include "HostTelemetryPublisher.h"
#include "../../system/time/Clock.h"
#include "../../system/identity/DeviceIdentity.h"
#include "HostProtocolCodec.h"
#include "HostCalibrationCommandHandler.h"
#include "../../data/SafetyTypes.h"
#include "../../system/SystemRuntimeContext.h"
#include "../../system/rtos/SharedSafetyState.h"
#include "../../system/calibration/CalibrationMath.h"
#include "../../system/calibration/CalibrationRuntimeStore.h"
#include "../../config/protocol/Crc16Ccitt.h"
#include "../../system/calibration/ManualEncoderReferenceStore.h"
#include "../../config/protocol/HostProtocol.h"
#include "../../generated/UrdfJointLimits.generated.h"
#include "../../generated/ProductFingerprint.generated.h"
#include "../../hal/serial/HostSerialPort.h"
#include "../../config/topology/HardwareMap.h"
#include "../../system/rtos/AtomicValue.h"
#include "../../system/log/FirmwareLog.h"
#include "../../system/ports/SystemEventPort.h"
#include "../../system/AppBootstrap.h"
#include "../../system/TaskRuntimeMetrics.h"
#include "HostCommandRouter.h"

#include <math.h>
#include <string.h>

using namespace p4::host;

namespace p4::host {

static_assert(kP4TimestampBytes == sizeof(uint32_t),
              "P4 trailing timestamp must match uint32 serializer");

// 职责：解析下行命令并更新共享命令状态；发送传感器/舵机/故障遥测；
// 维护协议 ACK、周期心跳和长触觉包的发送节奏。
static const uint16_t kEncoderDisconnectRaw = 0xFFFF; ///< CAN 磁编断开原始哨兵。
static const uint16_t kEncoderOpenRaw = 0x7FFE;       ///< S3 自检：开路。
static const uint16_t kEncoderShortRaw = 0x7FFD;      ///< S3 自检：短路。
static const uint8_t kTactileFullPacketMagic0 = 0x54; ///< 全量触觉分包魔数 'T'。
static const uint8_t kTactileFullPacketMagic1 = 0x32; ///< 全量触觉分包版本 '2'。

/** 校验快照 LUT 行数、采样数和 Mxx 归属是否与当前 HardwareMap 一致。 */
bool calibrationTendonLutMatchesJoint(
    uint8_t jointIndex,
    const JointCalibrationResult& result)
{
    if (!result.success) {
        return true;
    }
    if (jointIndex >= ENCODER_TOTAL_NUM ||
        result.tendonLutValid == 0 ||
        result.tendonLutCount < 2 ||
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
/** 检查当前 active scope 是否已有已提交的 RAM Manual 且每路跨度非零。 */
bool activeManualReferencesReady(SystemRuntimeContext* sharedData)
{
    if (!sharedData || !sharedData->manual_reference_committed) return false;
    const uint32_t activeMask =
        sharedAtomicLoad(&sharedData->active_joint_mask) &
        kManualEncoderReferenceFullMask;
    return manualEncoderReferencesCoverActiveMask(activeMask);
}

// 将 CAN 磁编 raw 转为线上值；无效通道统一发送断开哨兵。
static uint16_t encodeRawEncoderForWire(uint16_t rawValue, bool channelValid)
{
    if (rawValue == kEncoderOpenRaw || rawValue == kEncoderShortRaw) {
        return rawValue;  // 保留 S3 自检分类，供上位机显示 open/short。
    }

    if (!channelValid) {
        return (uint16_t)PROTOCOL_DISCONNECT_SENTINEL;
    }

    if (rawValue == kEncoderDisconnectRaw || rawValue == (uint16_t)PROTOCOL_DISCONNECT_SENTINEL) {
        return (uint16_t)PROTOCOL_DISCONNECT_SENTINEL;
    }

    return rawValue;
}

// 上报状态机、owner、active scope、在线/故障和事务 token。
bool sendSystemStatusPacket(SystemRuntimeContext* sharedData)
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
    buffer[idx++] = 0; // 旧自动标定状态字段保留布局，固件不再运行标定任务。
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
    appendU32BigEndian(buffer, &idx, sharedAtomicLoad(&sharedData->manual_reference_valid_bitmap));
    appendU32BigEndian(buffer, &idx, sharedAtomicLoad(&sharedData->manual_reference_crc));
    appendU32BigEndian(buffer, &idx, sharedAtomicLoad(&sharedData->manual_reference_revision));
    buffer[idx++] = sharedData->manual_reference_committed ? 1 : 0;
    appendU32BigEndian(buffer, &idx, autoCalibrationValidMask(sharedData));
    buffer[idx++] = sharedData->manual_reference_commit_status;
    {
        SharedSafetyGuard guard;
        buffer[idx++] = sharedData->pull_joint_into_workspace_status;
        buffer[idx++] = sharedData->pull_joint_into_workspace_reason;
        appendU32BigEndian(buffer, &idx, sharedData->pull_joint_into_workspace_sequence);
        appendU32BigEndian(buffer, &idx, sharedData->pull_joint_into_workspace_joint_mask);
        appendU32BigEndian(buffer, &idx, sharedData->pull_joint_into_workspace_timestamp);
    }
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
    // hold/transition 后紧跟 armed 位图，三个字段构成完整碰撞状态快照。
    appendU32BigEndian(
        buffer, &idx,
        sharedAtomicLoad(&sharedData->feedback_collision_armed_bitmap));
    // 原生ABS越界位图；旧圈数复位结果字段已删除。
    appendU32BigEndian(
        buffer, &idx,
        sharedAtomicLoad(&sharedData->servo_turn_overflow_bitmap));
    // 状态包时间表示本次状态快照在 P4 侧被序列化的时刻。
    appendU32BigEndian(buffer, &idx, p4::platform::time::monotonicMillis());
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    return FW_SERIAL_WRITE(buffer, idx) == idx;
}

/** 上报 21 路 Manual 端点、有效位、CRC 和 RAM 提交状态。 */
bool sendManualEncoderReferencePacket(SystemRuntimeContext* sharedData)
{
    if (!sharedData) {
        return false;
    }

    uint8_t buffer[144];
    size_t idx = 0;
    ManualEncoderReference references[ENCODER_TOTAL_NUM];
    uint32_t validMask = 0;
    uint32_t referenceCrc = 0;
    uint32_t revision = 0;
    uint8_t committed = 0;
    {
        SharedSafetyGuard guard;
        memcpy(references,
               g_manualEncoderReference,
               sizeof(references));
        validMask = sharedAtomicLoad(
            &sharedData->manual_reference_valid_bitmap);
        referenceCrc = sharedAtomicLoad(
            &sharedData->manual_reference_crc);
        revision = sharedAtomicLoad(
            &sharedData->manual_reference_revision);
        committed = sharedData->manual_reference_committed ? 1 : 0;
    }

    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_MANUAL_ENCODER_REFERENCE;
    buffer[idx++] = 4; // Payload version: v4; S3 raw is already direction-normalized.
    appendU32BigEndian(buffer, &idx, validMask);
    appendU32BigEndian(buffer, &idx, referenceCrc);
    appendU32BigEndian(buffer, &idx, revision);
    buffer[idx++] = committed;
    appendU32BigEndian(buffer, &idx, autoCalibrationValidMask(sharedData));
    buffer[idx++] = sharedData->manual_reference_commit_status;
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; joint++) {
        appendU16BigEndian(buffer, &idx, references[joint].minRaw);
        appendU16BigEndian(buffer, &idx, references[joint].maxRaw);
    }
    appendU32BigEndian(buffer, &idx, p4::platform::time::monotonicMillis());
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    return FW_SERIAL_WRITE(buffer, idx) == idx;
}
/** 上报一个 TaskRuntimeSnapshot，字段单位由 TaskRuntimeMonitor.h 约定。 */
bool sendTaskRuntimePacket(const TaskRuntimeSnapshot& diag)
{
    // v4 追加四条 BUS 的同步读统计；224 byte 足以容纳固定字段且不使用堆内存。
    uint8_t buffer[224];
    size_t idx = 0;
    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_TASK_DIAG;
    buffer[idx++] = 4; // Payload version.
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
    appendU32BigEndian(buffer, &idx, diag.windowSequence);
    appendU32BigEndian(buffer, &idx, diag.windowDurationUs);
    appendU32BigEndian(buffer, &idx, diag.windowLoopCount);
    appendU32BigEndian(buffer, &idx, diag.windowPeriodicCount);
    appendU32BigEndian(buffer, &idx, diag.windowEventCount);
    appendU32BigEndian(buffer, &idx, diag.avgExecUs);
    appendU32BigEndian(buffer, &idx, diag.p95ExecUs);
    appendU32BigEndian(buffer, &idx, diag.p99ExecUs);
    appendU32BigEndian(buffer, &idx, diag.windowMaxExecUs);
    appendU16BigEndian(buffer, &idx, diag.cpuPermille);
    appendU32BigEndian(buffer, &idx, diag.overrunCount);
    appendU16BigEndian(buffer, &idx, diag.consecutiveOverruns);
    appendU16BigEndian(buffer, &idx, diag.maxConsecutiveOverruns);
    appendU16BigEndian(buffer, &idx, diag.core0IdlePermille);
    appendU16BigEndian(buffer, &idx, diag.core1IdlePermille);
    appendU32BigEndian(buffer, &idx, diag.systemStatusMaxGapMs);
    appendU32BigEndian(buffer, &idx, diag.stopLastLatencyUs);
    appendU32BigEndian(buffer, &idx, diag.stopMaxLatencyUs);
    appendU32BigEndian(buffer, &idx, diag.timestampMs);
    for (uint8_t channel = 0; channel < TASK_RUNTIME_IO_CHANNEL_COUNT; ++channel)
        appendU32BigEndian(buffer, &idx, diag.ioSampleCount[channel]);
    for (uint8_t channel = 0; channel < TASK_RUNTIME_IO_CHANNEL_COUNT; ++channel)
        appendU32BigEndian(buffer, &idx, diag.ioRequestedCount[channel]);
    for (uint8_t channel = 0; channel < TASK_RUNTIME_IO_CHANNEL_COUNT; ++channel)
        appendU32BigEndian(buffer, &idx, diag.ioSucceededCount[channel]);
    for (uint8_t channel = 0; channel < TASK_RUNTIME_IO_CHANNEL_COUNT; ++channel)
        appendU32BigEndian(buffer, &idx, diag.ioAvgUs[channel]);
    for (uint8_t channel = 0; channel < TASK_RUNTIME_IO_CHANNEL_COUNT; ++channel)
        appendU32BigEndian(buffer, &idx, diag.ioP95Us[channel]);
    for (uint8_t channel = 0; channel < TASK_RUNTIME_IO_CHANNEL_COUNT; ++channel)
        appendU32BigEndian(buffer, &idx, diag.ioP99Us[channel]);
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    return FW_SERIAL_WRITE(buffer, idx) == idx;
}

// 上报舵机过载位图和每路锁存的触发 load。
bool sendFaultStatusPacket(SystemRuntimeContext* sharedData, uint32_t overloadFaultBitmap)
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
    appendU32BigEndian(buffer, &idx, p4::platform::time::monotonicMillis());
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    return FW_SERIAL_WRITE(buffer, idx) == idx;
}
/** 上报 27 路标定 ABS 范围及有效位；该范围仅用于诊断与快照恢复。 */
bool sendMotorAbsLimitPacket(SystemRuntimeContext* sharedData)
{
    if (!sharedData) {
        return false;
    }

    uint8_t buffer[240];
    size_t idx = 0;
    uint32_t validBitmap = 0;
    int32_t limitMin[SERVO_TOTAL_NUM] = {0};
    int32_t limitMax[SERVO_TOTAL_NUM] = {0};
    {
        SharedSafetyGuard guard;
        validBitmap = sharedAtomicLoad(&sharedData->motor_abs_limit_valid_bitmap);
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
    for (int i = 0; i < SERVO_TOTAL_NUM; i++) {
        appendI32BigEndian(buffer, &idx, limitMin[i]);
        appendI32BigEndian(buffer, &idx, limitMax[i]);
    }
    appendU32BigEndian(buffer, &idx, p4::platform::time::monotonicMillis());
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    return FW_SERIAL_WRITE(buffer, idx) == idx;
}

// 上报实际 ABS 的硬件行程；每台舵机的范围有效性仍随在线及可信状态变化。
bool sendServoLogicalRangePacket(SystemRuntimeContext* sharedData)
{
    if (!sharedData) {
        return false;
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
    appendU32BigEndian(buffer, &idx, p4::platform::time::monotonicMillis());
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    return FW_SERIAL_WRITE(buffer, idx) == idx;
}

/** 上报一个关节的 V4 Auto、Tendon LUT 和静态 URDF 范围快照。 */
bool sendJointCalibSnapshotPacket(uint8_t jointIndex)
{
    if (jointIndex >= ENCODER_TOTAL_NUM) {
        return false;
    }

    JointCalibrationRuntimeSnapshot snapshot;
    if (!readJointCalibrationRuntimeSnapshot(jointIndex, &snapshot)) {
        return false;
    }
    const JointCalibrationResult& result = snapshot.result;
    // 当前唯一 V4 基础快照 188 bytes，附加 13 bytes 装配告警、4 bytes
    // 时间戳和帧头尾后共 209 bytes；预留到 224，避免局部数组越界。
    uint8_t buffer[224];
    size_t idx = 0;
    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_JOINT_CALIB_SNAPSHOT;
    buffer[idx++] = jointIndex;
    buffer[idx++] = result.success ? 1 : 0;
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
    appendFloatBigEndian(buffer, &idx, result.angleMin);
    appendU32BigEndian(buffer, &idx, result.urdfConfigCrc);
    appendU16BigEndian(buffer, &idx, result.autoMinRaw);
    appendU16BigEndian(buffer, &idx, result.autoMaxRaw);
    appendU32BigEndian(buffer, &idx, result.manualReferenceCrc);
    // V5: 原 currentMin/Max 保留槽承载 angleK/angleB；payload 长度仍为 188 字节。
    appendFloatBigEndian(buffer, &idx, result.angleK);
    appendFloatBigEndian(buffer, &idx, result.angleB);
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
    appendU32BigEndian(buffer, &idx, p4::platform::time::monotonicMillis());
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    return FW_SERIAL_WRITE(buffer, idx) == idx;
}

/** 上报 21 路磁编 raw、有效状态和源快照时间。 */
bool sendEncoderRawPacket(const RemoteSensorData_t& sensorRaw)
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
    return FW_SERIAL_WRITE(buffer, idx) == idx;
}

/** 上报 21 路映射角；角度流与 raw 流保持独立，避免不可达的 raw 分支。 */
bool sendMappedAnglePacket(const MappedAngleData_t& mappedAngle)
{
    uint8_t buffer[64];
    size_t idx = 0;
    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0;
    buffer[idx++] = PACKET_TYPE_SENSOR;
    for (uint8_t i = 0; i < ENCODER_TOTAL_NUM; ++i) {
        const int16_t value = mappedAngle.isValid && mappedAngle.validFlags[i]
            ? mappedAngle.angleValues[i]
            : static_cast<int16_t>(PROTOCOL_DISCONNECT_SENTINEL);
        appendI16BigEndian(buffer, &idx, value);
    }
    appendU32BigEndian(buffer, &idx, mappedAngle.timestamp);
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = static_cast<uint8_t>(idx - 2U);
    return FW_SERIAL_WRITE(buffer, idx) == idx;
}

/** 上报同一 10 ms 控制周期采用的目标和反馈；包尾时间即 controlTimestamp。 */
bool sendControlTracePacket(const ControlTraceData_t& trace)
{
    // v4 cycle: components, motor detail, then joint/base. All share sequence and timestamp.
    uint8_t components[224];
    size_t ci = 0;
    components[ci++] = PROTOCOL_HEADER; components[ci++] = 0;
    components[ci++] = PACKET_TYPE_CONTROL_COMPONENTS;
    components[ci++] = 1;
    appendU32BigEndian(components, &ci, trace.sequence);
    components[ci++] = trace.sampleDivisor;
    for (uint8_t i=0; i<SERVO_TOTAL_NUM; ++i) appendI16BigEndian(components, &ci, trace.proportionalCounts[i]);
    for (uint8_t i=0; i<SERVO_TOTAL_NUM; ++i) appendI16BigEndian(components, &ci, trace.integralCounts[i]);
    for (uint8_t i=0; i<SERVO_TOTAL_NUM; ++i) appendI16BigEndian(components, &ci, trace.reversalCounts[i]);
    for (uint8_t i=0; i<ENCODER_TOTAL_NUM; ++i) appendI16BigEndian(components, &ci, trace.slowDegCenti[i]);
    appendU32BigEndian(components, &ci, trace.timestamp);
    components[ci++] = PROTOCOL_TAIL; components[1] = ci - 2;
    if (FW_SERIAL_WRITE(components, ci) != ci) return false;
    uint8_t detail[256];
    size_t di = 0;
    detail[di++] = PROTOCOL_HEADER;
    detail[di++] = 0;
    detail[di++] = PACKET_TYPE_CONTROL_TRACE;
    appendU32BigEndian(detail, &di, trace.sequence);
    appendU32BigEndian(detail, &di, trace.servoTimestamp);
    appendU32BigEndian(detail, &di, trace.dtMicros);
    appendU32BigEndian(detail, &di, trace.servoValidMask);
    appendU32BigEndian(detail, &di, trace.recoveryAppliedMask);
    appendU32BigEndian(detail, &di, trace.droppedFrames);
    for (uint8_t i = 0; i < SERVO_TOTAL_NUM; ++i) appendI32BigEndian(detail, &di, trace.motorActualAbs[i]);
    for (uint8_t i = 0; i < SERVO_TOTAL_NUM; ++i) appendI16BigEndian(detail, &di, trace.pidCounts[i]);
    for (uint8_t i = 0; i < SERVO_TOTAL_NUM; ++i) appendI16BigEndian(detail, &di, trace.recoveryCounts[i]);
    appendU32BigEndian(detail, &di, trace.timestamp);
    detail[di++] = PROTOCOL_TAIL;
    detail[1] = static_cast<uint8_t>(di - 2U);
    static_assert(24 + SERVO_TOTAL_NUM * 8 + 8 <= 256, "trace detail packet too large");
    if (FW_SERIAL_WRITE(detail, di) != di) return false;
    // 21 路目标角/反馈角与 27 路实际舵机 CMD 共用同一个控制周期时间戳。
    // 完整帧为 228 bytes，仍在单字节协议长度上限内。
    uint8_t buffer[256];
    size_t idx = 0;
    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0;
    buffer[idx++] = PACKET_TYPE_CONTROL_TRACE;
    appendU32BigEndian(buffer, &idx, trace.sequence);
    appendU32BigEndian(buffer, &idx, trace.targetToken);
    appendU32BigEndian(buffer, &idx, trace.targetAcceptedTimestamp);
    appendU32BigEndian(buffer, &idx, trace.encoderTimestamp);
    appendU32BigEndian(buffer, &idx, trace.activeJointMask);
    appendU32BigEndian(buffer, &idx, trace.validJointMask);
    appendU32BigEndian(buffer, &idx, trace.motorCommandMask);
    for (uint8_t i = 0; i < ENCODER_TOTAL_NUM; ++i) {
        appendI16BigEndian(buffer, &idx, trace.targetDegCenti[i]);
    }
    for (uint8_t i = 0; i < ENCODER_TOTAL_NUM; ++i) {
        appendI16BigEndian(buffer, &idx, trace.actualDegCenti[i]);
    }
    for (uint8_t i = 0; i < SERVO_TOTAL_NUM; ++i) {
        appendI32BigEndian(buffer, &idx, trace.motorCommandAbs[i]);
    }
    appendU32BigEndian(buffer, &idx, trace.timestamp);
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = static_cast<uint8_t>(idx - 2U);
    return FW_SERIAL_WRITE(buffer, idx) == idx;
}

/** 原生ABS与采样时间，一帧一个位置来源。 */
bool sendServoPositionPackets(const ServoAngleData_t& position)
{
    uint8_t buffer[3U + SERVO_TOTAL_NUM * 4U + 4U + 1U];
    const size_t length = encodeServoPositionPacket(buffer, sizeof(buffer), position);
    return length != 0U && FW_SERIAL_WRITE(buffer, length) == length;
}

/** 100 Hz 遥测不重复携带在线数组，在线状态统一来自 SYSTEM_STATUS。 */
bool sendServoOnlineStatusPacket(const ServoTelemetryData_t& telemetry)
{
    uint8_t buffer[256];
    const auto length = encodeServoOnlineStatusPacket(buffer, sizeof(buffer), telemetry);
    return length && FW_SERIAL_WRITE(buffer, length) == length;
}

bool sendServoTelemetryPacket(const ServoTelemetryData_t& telemetry)
{
    static_assert(3 + SERVO_TOTAL_NUM * 8 + kP4TimestampBytes + 1 <= 256,
                  "servo telemetry packet exceeds serial buffer");
    uint8_t buffer[256];
    size_t idx = 0;
    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0;
    buffer[idx++] = PACKET_TYPE_SERVO_TELEM;
    for (uint8_t i = 0; i < SERVO_TOTAL_NUM; ++i) {
        appendI16BigEndian(buffer, &idx, telemetry.speed[i]);
        appendI16BigEndian(buffer, &idx, telemetry.load[i]);
        // 电流保持 ST3215 原始有符号值，避免固件硬编码未经确认的 mA 比例。
        appendI16BigEndian(buffer, &idx, telemetry.current[i]);
        buffer[idx++] = telemetry.voltage[i];
        buffer[idx++] = telemetry.temperature[i];
    }
    appendU32BigEndian(buffer, &idx, telemetry.timestamp);
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = static_cast<uint8_t>(idx - 2U);
    return FW_SERIAL_WRITE(buffer, idx) == idx;
}

/** 1 Hz 上报每关节自适应倍率与实际 Kp；运行值只来自 MotionControlTask。 */
bool sendAdaptiveKpPacket(SystemRuntimeContext* sharedData)
{
    if (!sharedData) return false;
    uint8_t buffer[3 + 1 + ENCODER_TOTAL_NUM * 4 + 1];
    size_t idx = 0;
    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0;
    buffer[idx++] = PACKET_TYPE_ADAPTIVE_KP;
    buffer[idx++] = sharedData->adaptive_kp_enabled ? 1 : 0;
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; ++joint) {
        appendI16BigEndian(buffer, &idx, sharedData->adaptive_kp_scale_milli[joint]);
    }
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; ++joint) {
        appendI16BigEndian(buffer, &idx, sharedData->adaptive_kp_effective_milli[joint]);
    }
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = static_cast<uint8_t>(idx - 2U);
    return FW_SERIAL_WRITE(buffer, idx) == idx;
}

/** 发送一帧触觉摘要；全量事件不得再次经过本函数。 */
bool sendTactileSummaryPacket(const RemoteTactileData_t& tactile)
{
    if (!tactile.isValid) {
        return false;
    }
    uint8_t buffer[128];
    size_t idx = 0;
    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0;
    buffer[idx++] = PACKET_TYPE_TACTILE;
    buffer[idx++] = tactile.seq;
    memcpy(buffer + idx, tactile.values, TACTILE_SUMMARY_BYTES);
    idx += TACTILE_SUMMARY_BYTES;
    // 模块有效位图紧跟45字节摘要；bit=1表示本轮S3摘要SPI读取及CRC8均成功。
    buffer[idx++] = static_cast<uint8_t>((tactile.moduleValidMask >> 8) & 0xFFU);
    buffer[idx++] = static_cast<uint8_t>(tactile.moduleValidMask & 0xFFU);
    appendU32BigEndian(buffer, &idx, tactile.timestamp);
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = static_cast<uint8_t>(idx - 2U);
    return FW_SERIAL_WRITE(buffer, idx) == idx;
}

/** 把一帧全量触觉拆成串口小包；任一分片失败则保留队首并从分片 0 重试。 */
bool sendTactileFullPackets(const RemoteTactileData_t& tactile)
{
    if (!tactile.fullValid) {
        return false;
    }
    static const size_t kTactileFullSerialChunkBytes = 239;
    const uint8_t fragCount = static_cast<uint8_t>(
        (TACTILE_FULL_BYTES + kTactileFullSerialChunkBytes - 1U) /
        kTactileFullSerialChunkBytes);
    const uint16_t fullCrc = p4::integrity::crc16CcittFalse(
        tactile.fullValues,
        TACTILE_FULL_BYTES);
    for (uint8_t fragIdx = 0; fragIdx < fragCount; ++fragIdx) {
        const size_t offset = static_cast<size_t>(fragIdx) *
                              kTactileFullSerialChunkBytes;
        const size_t remaining = TACTILE_FULL_BYTES - offset;
        const uint8_t chunkLen = static_cast<uint8_t>(
            remaining > kTactileFullSerialChunkBytes
                ? kTactileFullSerialChunkBytes
                : remaining);
        uint8_t buffer[256];
        size_t idx = 0;
        buffer[idx++] = PROTOCOL_HEADER;
        buffer[idx++] = 0;
        buffer[idx++] = PACKET_TYPE_TACTILE_FULL;
        buffer[idx++] = kTactileFullPacketMagic0;
        buffer[idx++] = kTactileFullPacketMagic1;
        buffer[idx++] = tactile.fullSeq;
        buffer[idx++] = tactile.seq;
        buffer[idx++] = fragIdx;
        buffer[idx++] = fragCount;
        buffer[idx++] = chunkLen;
        buffer[idx++] = static_cast<uint8_t>(fullCrc & 0xFFU);
        buffer[idx++] = static_cast<uint8_t>((fullCrc >> 8) & 0xFFU);
        memcpy(buffer + idx, tactile.fullValues + offset, chunkLen);
        idx += chunkLen;
        appendU32BigEndian(buffer, &idx, tactile.fullTimestamp);
        buffer[idx++] = PROTOCOL_TAIL;
        buffer[1] = static_cast<uint8_t>(idx - 2U);
        if (FW_SERIAL_WRITE(buffer, idx) != idx) {
            return false;
        }
    }
    return true;
}

bool sendDirectTargetResultPacket(uint32_t token,
                                           uint32_t requestedMask,
                                           uint32_t appliedMask,
                                           uint32_t rejectedMask,
                                           uint8_t status)
{
    uint8_t buffer[25];
    size_t idx = 0;
    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0;
    buffer[idx++] = PACKET_TYPE_DIRECT_TARGET_RESULT;
    appendU32BigEndian(buffer, &idx, token);
    appendU32BigEndian(buffer, &idx, requestedMask);
    appendU32BigEndian(buffer, &idx, appliedMask);
    appendU32BigEndian(buffer, &idx, rejectedMask);
    buffer[idx++] = status;
    appendU32BigEndian(buffer, &idx, p4::platform::time::monotonicMillis());
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = static_cast<uint8_t>(idx - 2U);
    return FW_SERIAL_WRITE(buffer, idx) == idx;
}

bool sendServoRebaseZeroResultPacket(uint32_t resultToken,
                                     uint32_t successMask,
                                     uint32_t failedMask)
{
    // FE + length + type + token/masks + 原帧保留区 + timestamp + FF。
    uint8_t buffer[4U + 12U + 4U * SERVO_TOTAL_NUM + kP4TimestampBytes];
    static_assert(sizeof(buffer) - 2U <= UINT8_MAX,
                  "Rebase result must fit the one-byte frame length");
    size_t idx = 0;
    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0;
    buffer[idx++] = PACKET_TYPE_SERVO_REBASE_ZERO_RESULT;
    appendU32BigEndian(buffer, &idx, resultToken);
    appendU32BigEndian(buffer, &idx, successMask);
    appendU32BigEndian(buffer, &idx, failedMask);
    constexpr size_t kReservedBytes = 4U * SERVO_TOTAL_NUM;
    memset(buffer + idx, 0, kReservedBytes);
    idx += kReservedBytes;
    appendU32BigEndian(buffer, &idx, p4::platform::time::monotonicMillis());
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = static_cast<uint8_t>(idx - 2U);
    return FW_SERIAL_WRITE(buffer, idx) == idx;
}

/** 解码协议中少量本地使用的 int16 大端字段。 */
bool sendServoRouteStatusPacket(SystemRuntimeContext* sharedData)
{
    if (!sharedData) {
        return false;
    }

    uint32_t generation = 0;
    uint32_t validMask = 0;
    uint32_t controllableMask = 0;
    uint32_t conflictMask = 0;
    uint8_t overloadMask = 0;
    uint8_t scanState = 0;
    uint8_t scanProgress = 0;
    uint8_t busCounts[NUM_BUSES] = {0};
    uint8_t routeBus[SERVO_TOTAL_NUM] = {0};
    uint8_t routeState[SERVO_TOTAL_NUM] = {0};
    {
        SharedSafetyGuard guard;
        generation = sharedAtomicLoad(&sharedData->servo_route_generation);
        validMask = sharedAtomicLoad(&sharedData->servo_route_valid_bitmap);
        controllableMask =
            sharedAtomicLoad(&sharedData->servo_route_controllable_bitmap);
        conflictMask =
            sharedAtomicLoad(&sharedData->servo_route_conflict_bitmap);
        overloadMask = sharedAtomicLoad(&sharedData->servo_bus_overload_mask);
        scanState = sharedAtomicLoad(&sharedData->servo_route_scan_state);
        scanProgress = sharedAtomicLoad(&sharedData->servo_route_scan_progress);
        // 这些共享数组带 volatile，不能直接传给 memcpy。持锁后逐项生成普通局部快照，
        // 既保留 volatile 的并发语义，也保证后续组包期间看到的是同一代路由数据。
        for (uint8_t bus = 0; bus < NUM_BUSES; ++bus) {
            busCounts[bus] = sharedData->servo_bus_device_count[bus];
        }
        for (uint8_t motor = 0; motor < SERVO_TOTAL_NUM; ++motor) {
            routeBus[motor] = sharedData->servo_route_bus[motor];
            routeState[motor] = sharedData->servo_route_state[motor];
        }
    }

    uint8_t buffer[96];
    size_t idx = 0;
    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0;
    buffer[idx++] = PACKET_TYPE_SERVO_ROUTE_STATUS;
    appendU32BigEndian(buffer, &idx, generation);
    appendU32BigEndian(buffer, &idx, validMask);
    appendU32BigEndian(buffer, &idx, controllableMask);
    appendU32BigEndian(buffer, &idx, conflictMask);
    buffer[idx++] = overloadMask;
    buffer[idx++] = scanState;
    buffer[idx++] = scanProgress;
    for (uint8_t bus = 0; bus < NUM_BUSES; ++bus) {
        buffer[idx++] = busCounts[bus];
    }
    for (uint8_t motor = 0; motor < SERVO_TOTAL_NUM; ++motor) {
        buffer[idx++] = routeBus[motor];
    }
    for (uint8_t motor = 0; motor < SERVO_TOTAL_NUM; ++motor) {
        buffer[idx++] = routeState[motor];
    }
    appendU32BigEndian(buffer, &idx, p4::platform::time::monotonicMillis());
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = static_cast<uint8_t>(idx - 2U);
    return FW_SERIAL_WRITE(buffer, idx) == idx;
}

bool sendSafetyEventPacket(const SafetyEventRecord& event,
                           uint32_t overflowCount)
{
    uint8_t buffer[32];
    size_t idx = 0;
    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0;
    buffer[idx++] = PACKET_TYPE_SAFETY_EVENT;
    appendU32BigEndian(buffer, &idx, event.sequence);
    appendU16BigEndian(buffer, &idx, static_cast<uint16_t>(event.ruleId));
    buffer[idx++] = event.motorChannel;
    buffer[idx++] = event.jointIndex;
    buffer[idx++] = event.systemState;
    buffer[idx++] = event.owner;
    appendI32BigEndian(buffer, &idx, event.requestedValue);
    appendI32BigEndian(buffer, &idx, event.observedValue);
    appendU32BigEndian(buffer, &idx, overflowCount);
    // 公共尾时间戳使用触发时刻，不使用“发送时刻”，便于跨 Topic 对齐故障证据。
    appendU32BigEndian(buffer, &idx, event.timestampMs);
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = static_cast<uint8_t>(idx - 2U);
    return FW_SERIAL_WRITE(buffer, idx) == idx;
}

/**
 * 周期上报当前固件的产品指纹。
 *
 * 上位机将这些 CRC 写入每次 run_data 的 meta.json，后续分析可以精确确认
 * 当时使用的 URDF、舵机映射、控制参数和固件源码是否完全一致。
 */
bool sendProductFingerprintPacket()
{
    uint8_t buffer[48];
    size_t idx = 0;
    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_PRODUCT_FINGERPRINT;
    buffer[idx++] = kProductFingerprintSchemaVersion;
    buffer[idx++] = kUrdfFeedbackMappingVersion;
    buffer[idx++] = kManualScaledUrdfFeedbackMappingVersion;
    buffer[idx++] = kHostProtocolVersion;
    appendU32BigEndian(buffer, &idx, kProductControlModelCrc);
    appendU32BigEndian(buffer, &idx, kProductVisualModelCrc);
    appendU32BigEndian(buffer, &idx, kProductHardwareMapCrc);
    appendU32BigEndian(buffer, &idx, kProductParameterCrc);
    appendU32BigEndian(buffer, &idx, kProductFirmwareSourceCrc);
    appendU32BigEndian(buffer, &idx, kProductStructureCommitCrc);
    const uint64_t deviceId = p4::platform::deviceIdentity();
    appendU32BigEndian(buffer, &idx, static_cast<uint32_t>(deviceId >> 32U));
    appendU32BigEndian(buffer, &idx, static_cast<uint32_t>(deviceId));
    appendU32BigEndian(buffer, &idx, p4::platform::time::monotonicMillis());
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    return FW_SERIAL_WRITE(buffer, idx) == idx;
}

} // namespace p4::host
