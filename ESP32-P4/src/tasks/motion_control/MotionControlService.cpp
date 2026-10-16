/**
 * @file MotionControlService.cpp
 * @brief 运行 P4 的 10 ms 控制编排任务，读取一致快照并调用控制域模块生成舵机目标。
 *
 * 本文件只负责周期调度和模式编排。磁编映射、PID/LUT 计算、碰撞状态机、
 * 输出构建与命令发布分别由独立模块实现，禁止在此直接调用飞特 SDK。
 */
#include "MotionControlService.h"

#include <limits.h>
#include <math.h>
#include <string.h>

#include "../../control/JointFeedbackController.h"
#include "../../control/FeedbackFilter.h"
#include "JointAngleMapper.h"
#include "ControlCommandSnapshotReader.h"
#include "ServoTargetBuilder.h"
#include "../../system/ActuatorCommandGateway.h"
#include "../../system/calibration/CalibrationMath.h"
#include "../../system/calibration/CalibrationRuntimeStore.h"
#include "../../system/calibration/TendonCalibrationLut.h"
#include "../../system/ServiceCyclePolicy.h"
#include "../../config/ControlConfig.h"
#include "../../config/topology/HardwareMap.h"
#include "../../hal/servo/ServoValue.h"
#include "../../generated/UrdfJointLimits.generated.h"
#include "../../system/rtos/SharedSafetyState.h"
#include "../../system/log/FirmwareLog.h"
#include "../../system/SystemRuntimeContext.h"
#include "../../system/ports/SystemEventPort.h"
#include "../../system/TaskRuntimeMetrics.h"
#include "../../safety/policies/ControlInputSafetyPolicy.h"
#include "../../safety/policies/OverloadSafetyPolicy.h"
#include "../../safety/policies/CollisionSafetyPolicy.h"
#include "../../safety/policies/ControlBoundarySafetyPolicy.h"
#include "../../safety/SafetyEventReporter.h"

using namespace mmhand::config::control;

static mmhand::control::JointFeedbackController g_jointController;
static mmhand::control::FeedbackFilter g_feedbackFilters[ENCODER_TOTAL_NUM];
static mmhand::control::FeedbackEstimate g_feedbackEstimates[ENCODER_TOTAL_NUM];
// 碰撞检测历史放在静态区，避免挤占 10 ms 控制任务栈。
static FeedbackCollisionGuard g_feedbackCollisionGuard;

/** 将控制角度压成协议 0.01 degree；非有限值使用断开哨兵。 */
static int16_t controlTraceAngleCenti(float angleDeg)
{
    if (!isfinite(angleDeg)) return INT16_MAX;
    const long value = lroundf(angleDeg * 100.0f);
    if (value >= INT16_MAX) return INT16_MAX - 1;
    if (value < INT16_MIN) return INT16_MIN;
    return (int16_t)value;
}

/**
 * 控制服务跨周期状态。所有成员只由 MotionControlTask 访问，固定放在静态区，
 * 避免 100 Hz 实时任务反复分配内存或占用大块任务栈。
 */
struct ControlServiceRuntime {
    bool initialized;
    uint32_t reversalFailureMask;
    float localTargets[ENCODER_TOTAL_NUM];
    float magAngles[ENCODER_TOTAL_NUM];
    RemoteSensorData_t sensorData;
    MappedAngleData_t mappedData;
    // 上位机上传的 LUT 由 HostComm 跨核更新；控制周期只使用同代一致快照。
    JointCalibrationRuntimeSnapshot calibrationSnapshot[ENCODER_TOTAL_NUM];
    uint32_t calibrationSnapshotValidMask;
    uint32_t overloadFaultLatched;
    uint8_t overloadHighCount[SERVO_TOTAL_NUM];
    int32_t feedbackCollisionHoldAbs[ENCODER_TOTAL_NUM][MAX_JOINT_ACTUATORS];
    bool feedbackCollisionPidPushActive[ENCODER_TOTAL_NUM];
    // 上一控制周期采用的碰撞窗口档位；单电机按物理方向更新，双电机固定 RELEASE。
    uint8_t feedbackCollisionDriveMotionMode[ENCODER_TOTAL_NUM];
    uint32_t lastFeedbackCollisionSensorTimestamp;
    uint8_t upperWorkspaceHighCount[ENCODER_TOTAL_NUM];
    uint32_t upperWorkspaceWarningBitmap;
    // 目标钳位按关节锁存；目标回到动态范围后清除，下一次越界才重新上报。
    uint32_t targetClampEventLatchedBitmap;
    // 单腱和双腱 LUT 钳位按关节做边沿锁存，避免贴边时每 10 ms 重复上报。
    uint32_t lutClampEventLatchedBitmap;
    uint32_t lutClampObservedBitmap;
    uint32_t lastOverloadFaultResetToken;
    ControlCommandSnapshot_t commandSnapshot;
    uint32_t lastPidComputeMs;
    bool prevMotorOnline[SERVO_TOTAL_NUM];
    bool servoOnlineBaselineValid;
    bool hotplugHoldMotor[SERVO_TOTAL_NUM];
    uint32_t lastMotorCommandToken;
    uint32_t lastJointCommandToken;
    uint32_t controlTraceSequence;
    uint32_t controlTraceDropped;
    ControlTraceData_t cycleTrace;
};

static ControlServiceRuntime g_controlRuntime;

/** 把控制层规则写入统一事件缓存，记录触发时的状态和输出所有者。 */
static void recordControlRuleEvent(const ServiceOutputPort& output,
                                   uint8_t systemState,
                                   uint8_t owner,
                                   SafetyRuleId ruleId,
                                   uint8_t jointIndex,
                                   uint8_t motorChannel,
                                   int32_t requestedValue,
                                   int32_t observedValue)
{
    (void)output;
    p4::safety::reportSafetyRuleEvent(
        ruleId,
        systemState,
        owner,
        p4::safety::makeSafetyEventEvidence(jointIndex,
                                            motorChannel,
                                            requestedValue,
                                            observedValue));
}

/** 按关节位图清除已经恢复的控制规则，使下一次独立触发能够重新记录。 */
static void clearRecoveredControlRule(SafetyRuleId ruleId,
                                      uint32_t recoveredJointMask)
{
    for (uint8_t jointIndex = 0;
         jointIndex < ENCODER_TOTAL_NUM;
         ++jointIndex) {
        if ((recoveredJointMask & (1UL << jointIndex)) != 0) {
            p4::safety::clearSafetyRuleEvent(ruleId, jointIndex);
        }
    }
}

/** 记录腱绳 LUT 钳位；持续贴边只记录一次，离开边界后允许再次触发。 */
static void noteTendonLutClamp(SystemRuntimeContext* sharedData,
                              uint8_t jointIndex,
                              uint8_t motorChannel,
                              int32_t requestedAbs,
                              int32_t appliedAbs)
{
    if (!sharedData || jointIndex >= ENCODER_TOTAL_NUM) {
        return;
    }
    const uint32_t bit = 1UL << jointIndex;
    g_controlRuntime.lutClampObservedBitmap |= bit;
    if ((g_controlRuntime.lutClampEventLatchedBitmap & bit) != 0) {
        return;
    }
    g_controlRuntime.lutClampEventLatchedBitmap |= bit;
    p4::safety::reportSafetyRuleEvent(
        sharedData,
        SafetyRuleId::ControlLutBoundaryClamp,
        p4::safety::makeSafetyEventEvidence(jointIndex,
                                            motorChannel,
                                            requestedAbs,
                                            appliedAbs));
}

// 协议字段使用 int16，保留 +32767 作为哨兵/无效值。
static int16_t clampMappedCountForProtocol(int32_t value)
{
    if (value > 32766) return 32766;
    if (value < -32768) return -32768;
    return (int16_t)value;
}

static uint8_t appendJointActuatorHoldTargets(ServoTargetBatch_t* batch,
                                              uint8_t jointIndex,
                                              const ServoAngleData_t& servoData,
                                              uint16_t speed,
                                              uint8_t acc)
{
    // 为该关节每个在线执行器根据实时舵机反馈生成保持命令；
    // 故障和热插拔稳定阶段使用该路径。
    if (!batch) {
        return 0;
    }

    JointActuatorRuntimeTarget targets[MAX_JOINT_ACTUATORS];
    uint8_t targetCount = 0;
    const uint8_t actuatorCount = getJointActuatorCount(jointIndex);
    for (uint8_t i = 0; i < actuatorCount && targetCount < MAX_JOINT_ACTUATORS; i++) {
        JointActuatorItem actuator;
        if (!getJointActuator(jointIndex, i, &actuator) ||
            actuator.motorChannel >= SERVO_TOTAL_NUM ||
            servoData.onlineStatus[actuator.motorChannel] == 0) {
            continue;
        }
        JointActuatorRuntimeTarget& target = targets[targetCount++];
        target.motorChannel = actuator.motorChannel;
        target.sign = actuator.sign;
        target.targetAbs = servoData.servoAngles[actuator.motorChannel];
    }

    return appendJointActuatorRuntimeTargets(batch, targets, targetCount, speed, acc);
}

// 碰撞触发后持续下发触发瞬间的 ABS 快照，阻止位置式 PID 继续推进。
static uint8_t appendJointActuatorSnapshotTargets(
    ServoTargetBatch_t* batch,
    uint8_t jointIndex,
    const ServoAngleData_t& servoData,
    const int32_t holdAbs[MAX_JOINT_ACTUATORS],
    uint16_t speed,
    uint8_t acc)
{
    if (!batch || !holdAbs) {
        return 0;
    }

    JointActuatorRuntimeTarget targets[MAX_JOINT_ACTUATORS];
    uint8_t targetCount = 0;
    const uint8_t actuatorCount = getJointActuatorCount(jointIndex);
    for (uint8_t i = 0; i < actuatorCount && i < MAX_JOINT_ACTUATORS; i++) {
        JointActuatorItem actuator;
        if (!getJointActuator(jointIndex, i, &actuator) ||
            actuator.motorChannel >= SERVO_TOTAL_NUM ||
            servoData.onlineStatus[actuator.motorChannel] == 0) {
            continue;
        }
        JointActuatorRuntimeTarget& target = targets[targetCount++];
        target.motorChannel = actuator.motorChannel;
        target.sign = actuator.sign;
        target.targetAbs = holdAbs[i];
    }

    return appendJointActuatorRuntimeTargets(batch, targets, targetCount, speed, acc);
}

// 将监测段复位原因转成稳定日志名，避免实机排查时只看到数字枚举。
static const char* feedbackCollisionResetReasonName(uint8_t reason)
{
    switch (reason) {
        case FEEDBACK_COLLISION_MONITOR_RESET_TARGET_CHANGED:
            return "target_changed";
        case FEEDBACK_COLLISION_MONITOR_RESET_DIRECTION_CHANGED:
            return "direction_changed";
        case FEEDBACK_COLLISION_MONITOR_RESET_ERROR_DEADBAND:
            return "error_deadband";
        case FEEDBACK_COLLISION_MONITOR_RESET_HOLD_RELEASED:
            return "hold_released";
        case FEEDBACK_COLLISION_MONITOR_RESET_MOTION_MODE_CHANGED:
            return "motion_mode_changed";
        default:
            return "none";
    }
}

// 将 HOLD 解除原因写成可读文本，避免日志只有 reason 数字。
static const char* feedbackCollisionReleaseReasonName(uint8_t reason)
{
    switch (reason) {
        case FEEDBACK_COLLISION_RELEASE_CONTEXT:
            return "context";
        case FEEDBACK_COLLISION_RELEASE_INVALID_FEEDBACK:
            return "invalid_feedback";
        case FEEDBACK_COLLISION_RELEASE_TARGET_RETREAT:
            return "target_retreat";
        default:
            return "none";
    }
}

// 发布碰撞监测与保持状态。状态变化序号使上位机无需等待心跳即可刷新。
static void publishFeedbackCollisionState(SystemRuntimeContext* sharedData,
                                          uint8_t jointIndex,
                                          bool armed,
                                          bool holdActive)
{
    if (!sharedData || jointIndex >= ENCODER_TOTAL_NUM) {
        return;
    }

    const uint32_t bit = 1UL << jointIndex;
    const uint32_t previousArmed = armed
        ? sharedAtomicFetchOr(&sharedData->feedback_collision_armed_bitmap, bit)
        : sharedAtomicFetchAnd(&sharedData->feedback_collision_armed_bitmap, ~bit);
    const uint32_t nextArmed = armed
        ? (previousArmed | bit)
        : (previousArmed & ~bit);

    const uint32_t previousHold = holdActive
        ? sharedAtomicFetchOr(&sharedData->feedback_collision_hold_bitmap, bit)
        : sharedAtomicFetchAnd(&sharedData->feedback_collision_hold_bitmap, ~bit);
    const uint32_t nextHold = holdActive
        ? (previousHold | bit)
        : (previousHold & ~bit);
    if (previousArmed != nextArmed || previousHold != nextHold) {
        sharedAtomicAddFetch(&sharedData->feedback_collision_transition_count,
                             (uint32_t)1);
    }
}

/** 初始化控制域的跨周期状态；本函数不创建任务，也不进入循环。 */
bool initMotionControlService(SystemRuntimeContext* sharedData, uint32_t nowMs)
{
    if (!sharedData) {
        return false;
    }

    memset(&g_controlRuntime, 0, sizeof(g_controlRuntime));
    g_jointController.resetAll();
    for (auto& filter : g_feedbackFilters) filter.reset();
    g_controlRuntime.lastOverloadFaultResetToken =
        sharedAtomicLoad(&sharedData->overload_fault_reset_token);
    // 控制服务启动时清空对外公开的故障状态。
    SharedSafetyGuard guard;
    sharedAtomicStore(&sharedData->overload_fault_bitmap, (uint32_t)0);
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        sharedData->overload_fault_load[ch] = 0;
    }
    p4::control::readCommandSnapshot(sharedData,
                                     &g_controlRuntime.commandSnapshot);
    g_controlRuntime.lastMotorCommandToken =
        g_controlRuntime.commandSnapshot.motorCommandToken;
    g_controlRuntime.lastJointCommandToken =
        g_controlRuntime.commandSnapshot.jointCommandToken;
    g_controlRuntime.lastPidComputeMs = nowMs;
    g_controlRuntime.initialized = true;
    return true;
}

/**
 * 执行单个 10 ms 控制周期。循环、截止时间统计和让出 CPU 均由 MotionControlTask 负责。
 */
bool stepMotionControlService(SystemRuntimeContext* sharedData,
                        const ControlServiceInputSnapshot& input,
                        const ServiceOutputPort& output)
{
    if (!sharedData || !g_controlRuntime.initialized) {
        return false;
    }
    if (!controlSnapshotHasActiveScope(input) && input.controlEnabled != 0) {
        // 空 scope 下仍允许刷新反馈，但禁止误把本周期视为有效控制计算。
        return false;
    }
    const uint32_t nowMs = input.nowMs;
    const auto controlParameters = mmhand::control::readParameters();

    // 使用引用保留原控制公式的可读命名，同时把所有历史量集中到运行上下文。
    ControlServiceRuntime& runtime = g_controlRuntime;
    auto& localTargets = runtime.localTargets;
    auto& magAngles = runtime.magAngles;
    auto& sensorData = runtime.sensorData;
    auto& mappedData = runtime.mappedData;
    auto& calibrationSnapshot = runtime.calibrationSnapshot;
    auto& calibrationSnapshotValidMask = runtime.calibrationSnapshotValidMask;
    auto& overloadFaultLatched = runtime.overloadFaultLatched;
    auto& overloadHighCount = runtime.overloadHighCount;
    auto& feedbackCollisionHoldAbs = runtime.feedbackCollisionHoldAbs;
    auto& feedbackCollisionPidPushActive = runtime.feedbackCollisionPidPushActive;
    auto& feedbackCollisionDriveMotionMode =
        runtime.feedbackCollisionDriveMotionMode;
    auto& lastFeedbackCollisionSensorTimestamp =
        runtime.lastFeedbackCollisionSensorTimestamp;
    auto& upperWorkspaceHighCount = runtime.upperWorkspaceHighCount;
    auto& upperWorkspaceWarningBitmap = runtime.upperWorkspaceWarningBitmap;
    auto& lastOverloadFaultResetToken = runtime.lastOverloadFaultResetToken;
    auto& commandSnapshot = runtime.commandSnapshot;
    auto& prevMotorOnline = runtime.prevMotorOnline;
    auto& hotplugHoldMotor = runtime.hotplugHoldMotor;
    auto& lastMotorCommandToken = runtime.lastMotorCommandToken;
    auto& lastJointCommandToken = runtime.lastJointCommandToken;

        if (!p4::control::readCommandSnapshot(
                sharedData, &commandSnapshot)) {
            // 命令互斥锁短暂繁忙时跳过本帧；任务层仍保持固定周期。
            return false;
        }
        runtime.lutClampObservedBitmap = 0;

        const uint32_t motorCommandToken = commandSnapshot.motorCommandToken;
        if (motorCommandToken != lastMotorCommandToken)
        {
            // 新的电机直控命令会解除当前热插拔保持状态。
            memset(hotplugHoldMotor, 0, sizeof(hotplugHoldMotor));
            lastMotorCommandToken = motorCommandToken;
        }

        const uint32_t jointCommandToken = commandSnapshot.jointCommandToken;
        const bool newJointCommand = jointCommandToken != lastJointCommandToken;
        if (newJointCommand)
        {
            // 新关节目标解除热插拔保持，但控制持续运行。
            memset(hotplugHoldMotor, 0, sizeof(hotplugHoldMotor));
            lastJointCommandToken = jointCommandToken;
        }

        const uint32_t overloadFaultResetToken =
            sharedAtomicLoad(&sharedData->overload_fault_reset_token);
        if (overloadFaultResetToken != lastOverloadFaultResetToken)
        {
            // 操作端请求故障复位时，同时清理私有锁存状态和共享故障 bitmap/负载快照。
            overloadFaultLatched = 0;
            memset(overloadHighCount, 0, sizeof(overloadHighCount));
            {
                SharedSafetyGuard guard;
                sharedAtomicStore(&sharedData->overload_fault_bitmap, (uint32_t)0);
                for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
                    sharedData->overload_fault_load[ch] = 0;
                }
            }
            lastOverloadFaultResetToken = overloadFaultResetToken;
        }

        ServoTargetBatch_t targetBatch;
        runtime.cycleTrace = {};
        memset(&targetBatch, 0, sizeof(targetBatch));
        // 每个循环生成一个 batch 并用 overwrite 发布，下游始终看到最新完整命令集。
        targetBatch.timestamp = nowMs;
        targetBatch.source = SERVO_TARGET_OWNER_CONTROL;
        if (commandSnapshot.controlMode == CONTROL_MODE_DIRECT_MOTOR &&
            commandSnapshot.motorDirectCommandSource == MOTOR_DIRECT_SOURCE_ABSOLUTE) {
            targetBatch.directGeneration = commandSnapshot.motorDirectCommandGeneration;
            targetBatch.directToken = commandSnapshot.directToken;
        }

        ServoAngleData_t servoData;
        memset(&servoData, 0, sizeof(servoData));
        servoData.timestamp = nowMs;

        // 只 peek 不消费：控制、标定和上行可共读同一个最新反馈邮箱。
        sharedData->servoFeedbackQueue.peek(&servoData);

        ServoTelemetryData_t telemetryData;
        memset(&telemetryData, 0, sizeof(telemetryData));
        bool telemetryFresh = false;
        if (sharedData->servoTelemetryQueue.peek(&telemetryData)) {
            // 负载/电流数据用于过载保护，因此比编码器映射更快过期。
            telemetryFresh =
                (uint32_t)(nowMs - telemetryData.timestamp) <= kServoTelemetryTimeoutMs;
        }

        for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++)
        {
            // 检测电机从离线到在线的热插拔变化，并保持恢复电机直到下一条显式命令。
            const bool onlineNow = (servoData.onlineStatus[ch] != 0);
            const bool wasOnline = prevMotorOnline[ch];
            if (p4::safety::shouldEnterHotplugHold(wasOnline, onlineNow)) {
                hotplugHoldMotor[ch] = true;
                // 启动时首次建立在线基线不属于热插拔；只有经历过有效反馈后的重新上线才上报。
                if (p4::safety::shouldReportHotplugHold(
                        runtime.servoOnlineBaselineValid,
                        wasOnline,
                        onlineNow)) {
                    recordControlRuleEvent(
                        output,
                        input.systemState,
                        input.owner,
                        SafetyRuleId::ControlHotplugHold,
                        0xFF,
                        ch,
                        0,
                        servoData.servoAngles[ch]);
                }
            }
            prevMotorOnline[ch] = onlineNow;
        }
        if (servoData.timestamp != 0) {
            runtime.servoOnlineBaselineValid = true;
        }

        sharedAtomicFetchOr(&sharedData->overload_fault_bitmap, overloadFaultLatched);
        overloadFaultLatched = sharedAtomicLoad(&sharedData->overload_fault_bitmap);

        bool canBusOnline = false;
        if (sharedData->canRxQueue.peek(&sensorData) && sensorData.isValid) {
            // 超过 CAN 超时时间后，磁编码器反馈视为过期。
            canBusOnline = (nowMs - sensorData.timestamp) <= kCanBusOfflineTimeoutMs;
        }

        memset(&mappedData, 0, sizeof(mappedData));
        mappedData.timestamp = nowMs;
        mappedData.isValid = canBusOnline;

        calibrationSnapshotValidMask =
            readAllJointCalibrationRuntimeSnapshots(calibrationSnapshot)
                ? ((1UL << ENCODER_TOTAL_NUM) - 1UL)
                : 0;

        if (canBusOnline)
        {
            // 将 raw 磁编码器读数转换为已标定 URDF 角度，
            // 同时发布紧凑协议计数和 float 调试角度。
            uint32_t encoderOnlineMask = 0;
            uint32_t encoderFaultMask = 0;
            {
                SharedSafetyGuard guard;
                encoderOnlineMask = sharedAtomicLoad(&sharedData->encoder_online_bitmap);
                encoderFaultMask = sharedAtomicLoad(&sharedData->encoder_fault_bitmap);
            }
            for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
            {
                const uint32_t bit = (1UL << i);
                if (((encoderOnlineMask & bit) == 0) ||
                    ((encoderFaultMask & bit) != 0) ||
                    sensorData.encoderValues[i] == kEncoderDisconnectRaw) {
                    mappedData.validFlags[i] = 0;
                    magAngles[i] = 0.0f;
                    upperWorkspaceHighCount[i] = 0;
                    upperWorkspaceWarningBitmap &= ~bit;
                    continue;
                }

                if ((calibrationSnapshotValidMask & bit) == 0 ||
                    !calibrationSnapshot[i].result.success) {
                    // 未标定关节故意标记为无效，禁止进入反馈 PID。
                    mappedData.angleValues[i] = 0;
                    mappedData.validFlags[i] = 0;
                    magAngles[i] = 0.0f;
                    upperWorkspaceHighCount[i] = 0;
                    upperWorkspaceWarningBitmap &= ~bit;
                    continue;
                }

                const JointCalibrationRuntimeSnapshot& calibration =
                    calibrationSnapshot[i];
                const JointCalibrationResult& calib = calibration.result;
                float mappedDeg = 0.0f;
                if (calib.mappingVersion == kManualScaledUrdfFeedbackMappingVersion) {
                    // 控制环只接受当前 Manual-scaled 映射；旧 Auto 映射不得回退运行。
                    EncoderUrdfMapResult mapping;
                    const bool mapped =
                        mapManualScaledEncoderToUrdf(
                            (uint8_t)i,
                            calib,
                            calibration.manual,
                            sensorData.encoderValues[i],
                            calibration.encoderDirection,
                            &mapping);
                    if (!mapped) {
                        mappedData.angleValues[i] = 0;
                        mappedData.outsideCounts[i] = 0;
                        mappedData.validFlags[i] = 0;
                        magAngles[i] = 0.0f;
                        upperWorkspaceHighCount[i] = 0;
                        upperWorkspaceWarningBitmap &= ~bit;
                        continue;
                    }
                    mappedDeg = mapping.urdfDeg;
                    int32_t outsideCounts = mapping.outsideCounts;
                    if (outsideCounts > 32767) outsideCounts = 32767;
                    if (outsideCounts < -32768) outsideCounts = -32768;
                    mappedData.outsideCounts[i] = (int16_t)outsideCounts;
                    const p4::safety::WorkspaceWarningDecision warning =
                        p4::safety::evaluateUpperWorkspaceWarning(
                            outsideCounts,
                            kUpperWorkspaceWarningCounts,
                            kUpperWorkspaceWarningStableFrames,
                            upperWorkspaceHighCount[i],
                            (upperWorkspaceWarningBitmap & bit) != 0);
                    upperWorkspaceHighCount[i] = warning.nextStableCount;
                    if (warning.active) {
                        upperWorkspaceWarningBitmap |= bit;
                        if (warning.entered) {
                            recordControlRuleEvent(
                                output,
                                input.systemState,
                                input.owner,
                                SafetyRuleId::ControlUpperWorkspaceWarning,
                                static_cast<uint8_t>(i),
                                0xFF,
                                kUpperWorkspaceWarningCounts,
                                outsideCounts);
                            FW_LOG_WARN_PRINTF(
                                "[CONTROL] UPPER_WORKSPACE_WARN J%02u outside=%ld threshold=%ld raw=%u mapped=%.3f max=%.3f\r\n",
                                (unsigned)i,
                                (long)outsideCounts,
                                (long)kUpperWorkspaceWarningCounts,
                                (unsigned)sensorData.encoderValues[i],
                                mappedDeg,
                                calib.currentMaxDeg);
                        }
                    } else {
                        if (warning.recovered) {
                            FW_LOG_INFO_PRINTF(
                                "[CONTROL] UPPER_WORKSPACE_RECOVER J%02u outside=%ld\r\n",
                                (unsigned)i,
                                (long)outsideCounts);
                        }
                        upperWorkspaceWarningBitmap &= ~bit;
                    }
                } else {
                    // 非当前映射版本只能查看/备份，不能进入实时控制。
                    mappedData.angleValues[i] = 0;
                    mappedData.outsideCounts[i] = 0;
                    mappedData.validFlags[i] = 0;
                    magAngles[i] = 0.0f;
                    upperWorkspaceHighCount[i] = 0;
                    upperWorkspaceWarningBitmap &= ~bit;
                    continue;
                }
                const int32_t mappedCount =
                    p4::control::degToEncoderCount(mappedDeg);
                mappedData.angleValues[i] = clampMappedCountForProtocol(mappedCount);
                mappedData.validFlags[i] = 1;
                magAngles[i] = mappedDeg;
            }
        }

        for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; ++joint) {
            g_feedbackEstimates[joint] = g_feedbackFilters[joint].update(sensorData.timestamp, magAngles[joint],
                canBusOnline && mappedData.validFlags[joint], controlParameters);
            if (g_feedbackEstimates[joint].fresh || g_feedbackEstimates[joint].ready)
                magAngles[joint] = g_feedbackEstimates[joint].fast;
        }

        sharedData->mappedAngleQueue.overwrite(mappedData);

        memcpy(localTargets, commandSnapshot.targetAngles, sizeof(localTargets));
        const bool hostControlEnabled = (sharedData->control_enabled != 0);
        const uint8_t systemState = sharedData->system_state;
        const uint8_t targetOwner = sharedData->servo_target_owner;
        const bool controlOwnerActive = (targetOwner == SERVO_TARGET_OWNER_CONTROL);
        const bool feedbackRunningState = (systemState == SYSTEM_STATE_RUN_FEEDBACK);
        const bool directRunningState = (systemState == SYSTEM_STATE_RUN_DIRECT);
        const bool feedbackOutputState =
            feedbackRunningState && commandSnapshot.controlMode == CONTROL_MODE_JOINT;
        const bool directOutputState =
            directRunningState && commandSnapshot.controlMode == CONTROL_MODE_DIRECT_MOTOR;
        const bool controlEnabled = hostControlEnabled &&
                                    controlOwnerActive &&
                                    (feedbackOutputState || directOutputState);
        const bool normalOutputAllowed = controlEnabled;

        if (normalOutputAllowed && telemetryFresh)
        {
            // 被命令电机持续高负载后锁存过载故障。
            const uint32_t overloadMotorMask =
                p4::safety::overloadActiveMotorMask(
                    sharedData, commandSnapshot);
            for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++)
            {
                const uint32_t bit = 1UL << ch;
                if (((overloadMotorMask & bit) == 0) ||
                    telemetryData.onlineStatus[ch] == 0)
                {
                    overloadHighCount[ch] = 0;
                    continue;
                }

                const p4::safety::OverloadSampleDecision overloadDecision =
                    p4::safety::evaluateOverloadSample(
                        commandSnapshot.controlMode,
                        telemetryData.load[ch],
                        overloadHighCount[ch]);
                overloadHighCount[ch] = overloadDecision.nextHighCount;

                if (overloadDecision.shouldLatch &&
                    (overloadFaultLatched & bit) == 0)
                {
                    overloadFaultLatched |= bit;
                    {
                        SharedSafetyGuard guard;
                        sharedData->overload_fault_load[ch] = telemetryData.load[ch];
                    }
                    FW_LOG_INFO_PRINTF("[CONTROL] OVERLOAD_HOLD M%02u load=%d threshold=%d cycles=%u mode=%u\r\n",
                                  (unsigned)ch,
                                  mmhand::servo::loadMagnitude(telemetryData.load[ch]),
                                  overloadDecision.threshold,
                                  (unsigned)overloadDecision.latchCycles,
                                  (unsigned)commandSnapshot.controlMode);
                }
            }
        }
        else
        {
            memset(overloadHighCount, 0, sizeof(overloadHighCount));
        }
        sharedAtomicFetchOr(&sharedData->overload_fault_bitmap, overloadFaultLatched);
        overloadFaultLatched = sharedAtomicLoad(&sharedData->overload_fault_bitmap);

        if (commandSnapshot.controlMode == CONTROL_MODE_JOINT) {
            // 本地轨迹仍要限幅；只对本帧明确控制的关节上报越界，避免未选关节的默认值制造误告警。
            uint32_t targetClampObservedBitmap = 0;
            for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++) {
                const float requestedDeg = localTargets[jointIndex];
                const float appliedDeg = p4::control::clampJointTargetDeg(
                    requestedDeg, jointIndex);
                localTargets[jointIndex] = appliedDeg;

                const uint32_t bit = 1UL << jointIndex;
                const bool commandAddressesJoint =
                    (commandSnapshot.feedbackCommandJointMask & bit) != 0;
                if (!commandAddressesJoint ||
                    fabsf(requestedDeg - appliedDeg) <= 0.0001f) {
                    continue;
                }

                targetClampObservedBitmap |= bit;
                if ((runtime.targetClampEventLatchedBitmap & bit) == 0) {
                    runtime.targetClampEventLatchedBitmap |= bit;
                    recordControlRuleEvent(
                        output,
                        systemState,
                        targetOwner,
                        SafetyRuleId::ControlJointTargetClamp,
                        jointIndex,
                        0xFF,
                        static_cast<int32_t>(lroundf(requestedDeg * 1000.0f)),
                        static_cast<int32_t>(lroundf(appliedDeg * 1000.0f)));
                }
            }
            const uint32_t recoveredTargetClampMask =
                runtime.targetClampEventLatchedBitmap &
                ~targetClampObservedBitmap;
            clearRecoveredControlRule(SafetyRuleId::ControlJointTargetClamp,
                                      recoveredTargetClampMask);
            runtime.targetClampEventLatchedBitmap &= targetClampObservedBitmap;
        } else {
            clearRecoveredControlRule(
                SafetyRuleId::ControlJointTargetClamp,
                runtime.targetClampEventLatchedBitmap);
            runtime.targetClampEventLatchedBitmap = 0;
        }

        const uint32_t activeJointMask =
            sharedAtomicLoad(&sharedData->active_joint_mask) &
            ((1UL << ENCODER_TOTAL_NUM) - 1UL);
        // 普通 UI、测试、算法和遥操统一使用最近一帧角度命令携带的控制掩码，
        // 避免单行指令意外影响未选中的关节。
        const uint32_t feedbackCommandJointMask =
            commandSnapshot.feedbackCommandJointMask &
            activeJointMask;
        const bool feedbackCollisionContextValid =
            feedbackOutputState &&
            commandSnapshot.controlMode == CONTROL_MODE_JOINT &&
            (systemState == SYSTEM_STATE_READY ||
             systemState == SYSTEM_STATE_RUN_FEEDBACK);
        const bool newFeedbackCollisionSample =
            canBusOnline &&
            sensorData.timestamp != 0 &&
            sensorData.timestamp != lastFeedbackCollisionSensorTimestamp;

        if (newFeedbackCollisionSample) {
            lastFeedbackCollisionSensorTimestamp = sensorData.timestamp;
        }

        // 碰撞检测只消费磁编新帧，以 q_slow 计算速度，武装和检测各使用 COLLISION_WINDOW_MS，
        // PID 推进和 LUT 边界证据使用上一控制周期的输出，最多只引入 10 ms 延迟。
        for (uint8_t jointIndex = 0;
             jointIndex < ENCODER_TOTAL_NUM;
             jointIndex++) {
            const bool jointActive =
                (feedbackCommandJointMask & (1UL << jointIndex)) != 0;
            const bool contextActive =
                feedbackCollisionContextValid && jointActive;

            FeedbackCollisionResult collisionResult;
            memset(&collisionResult, 0, sizeof(collisionResult));
            if (!contextActive) {
                collisionResult = g_feedbackCollisionGuard.resetJoint(
                    jointIndex,
                    FEEDBACK_COLLISION_RELEASE_CONTEXT);
            } else if (!canBusOnline ||
                       mappedData.validFlags[jointIndex] == 0) {
                collisionResult = g_feedbackCollisionGuard.resetJoint(
                    jointIndex,
                    FEEDBACK_COLLISION_RELEASE_INVALID_FEEDBACK);
            } else if (newFeedbackCollisionSample) {
                FeedbackCollisionInput input;
                input.timestampMs = sensorData.timestamp;
                input.contextActive = true;
                input.feedbackValid = g_feedbackEstimates[jointIndex].ready;
                input.targetDeg = localTargets[jointIndex];
                input.feedbackDeg = g_feedbackEstimates[jointIndex].slow;
                input.pidPushActive =
                    feedbackCollisionPidPushActive[jointIndex];
                input.driveMotionMode =
                    feedbackCollisionDriveMotionMode[jointIndex];
                collisionResult =
                    g_feedbackCollisionGuard.update(jointIndex, input);
            } else {
                collisionResult.armed =
                    g_feedbackCollisionGuard.isArmed(jointIndex);
                collisionResult.holdActive =
                    g_feedbackCollisionGuard.isHoldActive(jointIndex);
                collisionResult.holdAngleDeg =
                    g_feedbackCollisionGuard.holdAngleDeg(jointIndex);
            }

            if (collisionResult.armedChanged && collisionResult.armed) {
                FW_LOG_INFO_PRINTF(
                    "[COLLISION_ARM] J%02u qRef=%.3f qFb=%.3f "
                    "mode=%s v_slow=%.3fdeg/s source=velocity candidate_ms=%lu\r\n",
                    (unsigned)jointIndex,
                    (double)localTargets[jointIndex],
                    (double)magAngles[jointIndex],
                    feedbackCollisionMotionModeName(collisionResult.motionMode),
                    (double)collisionResult.directionVelocityDegPerSec,
                    (unsigned long)collisionResult.candidateAgeMs);
            }

            if (collisionResult.monitorReset &&
                !collisionResult.holdReleased) {
                FW_LOG_INFO_PRINTF(
                    "[COLLISION_MON_RESET] J%02u reason=%s(%u) "
                    "target_delta=%.3f qRef=%.3f qFb=%.3f\r\n",
                    (unsigned)jointIndex,
                    feedbackCollisionResetReasonName(
                        collisionResult.monitorResetReason),
                    (unsigned)collisionResult.monitorResetReason,
                    (double)collisionResult.targetDeltaDeg,
                    (double)localTargets[jointIndex],
                    (double)magAngles[jointIndex]);
            }

            if (collisionResult.holdEntered) {
                memset(feedbackCollisionHoldAbs[jointIndex],
                       0,
                       sizeof(feedbackCollisionHoldAbs[jointIndex]));
                const uint8_t actuatorCount =
                    getJointActuatorCount(jointIndex);
                for (uint8_t actuatorIndex = 0;
                     actuatorIndex < actuatorCount &&
                     actuatorIndex < MAX_JOINT_ACTUATORS;
                     actuatorIndex++) {
                    JointActuatorItem actuator;
                    if (getJointActuator(jointIndex,
                                         actuatorIndex,
                                         &actuator) &&
                        actuator.motorChannel < SERVO_TOTAL_NUM) {
                        feedbackCollisionHoldAbs[jointIndex][actuatorIndex] =
                            servoData.servoAngles[actuator.motorChannel];
                    }
                }
                g_jointController.suspend(jointIndex, servoData, nowMs);
                recordControlRuleEvent(
                    output,
                    sharedAtomicLoad(&sharedData->system_state),
                    sharedAtomicLoad(&sharedData->servo_target_owner),
                    SafetyRuleId::ControlCollisionHold,
                    jointIndex,
                    0xFF,
                    static_cast<int32_t>(lroundf(collisionResult.holdTargetDeg * 1000.0f)),
                    static_cast<int32_t>(lroundf(collisionResult.holdAngleDeg * 1000.0f)));
                FW_LOG_INFO_PRINTF(
                    "[COLLISION_HOLD] J%02u blockedTarget=%.3f holdAngle=%.3f "
                    "mode=%s v_slow=%.3fdeg/s progress%lu=%.3fdeg min=%.3f "
                    "candidate_ms=%lu\r\n",
                    (unsigned)jointIndex,
                    (double)collisionResult.holdTargetDeg,
                    (double)collisionResult.holdAngleDeg,
                    feedbackCollisionMotionModeName(collisionResult.motionMode),
                    (double)collisionResult.directionVelocityDegPerSec,
                    (unsigned long)collisionResult.progressWindowMs,
                    (double)collisionResult.directionProgressDeg,
                    (double)collisionResult.minProgressDeg,
                    (unsigned long)collisionResult.candidateAgeMs);
            }

            if (collisionResult.holdReleased) {
                p4::safety::clearSafetyRuleEvent(
                    SafetyRuleId::ControlCollisionHold,
                    jointIndex);
                g_jointController.suspend(jointIndex, servoData, nowMs);
                FW_LOG_INFO_PRINTF(
                    "[COLLISION_RELEASE] J%02u reason=%s(%u) "
                    "blockedTarget=%.3f newTarget=%.3f retreat=%.3f "
                    "qFb=%.3f\r\n",
                    (unsigned)jointIndex,
                    feedbackCollisionReleaseReasonName(
                        collisionResult.releaseReason),
                    (unsigned)collisionResult.releaseReason,
                    (double)collisionResult.holdTargetDeg,
                    (double)localTargets[jointIndex],
                    (double)collisionResult.targetRetreatDeg,
                    (double)magAngles[jointIndex]);
            }

            publishFeedbackCollisionState(
                sharedData,
                jointIndex,
                collisionResult.armed,
                collisionResult.holdActive);

            if (collisionResult.holdActive) {
                localTargets[jointIndex] =
                    collisionResult.holdAngleDeg;
            }
            if (!contextActive) {
                feedbackCollisionPidPushActive[jointIndex] = false;
                feedbackCollisionDriveMotionMode[jointIndex] =
                    FEEDBACK_COLLISION_MOTION_RELEASE;
            }
        }
        if (!controlEnabled || !feedbackOutputState) g_jointController.resetAll();
        if (controlEnabled && controlParameters.configured) {
            const uint32_t elapsed = nowMs - runtime.lastPidComputeMs;
            runtime.lastPidComputeMs = nowMs;
            const float dt = elapsed >= 1 && elapsed <= 100 ? elapsed * 0.001f : kPeriodSec;
            runtime.cycleTrace.dtMicros = lroundf(dt * 1000000.0f);
            const bool servoFresh = servoData.timestamp != 0 && nowMs - servoData.timestamp <= kServoTelemetryTimeoutMs;
            if (directOutputState) {
                const uint32_t motors = commandSnapshot.directMotorMask & validMotorChannelMask();
                if (servoFresh) for (uint8_t motor = 0; motor < SERVO_TOTAL_NUM; ++motor) {
                    if (!(motors & (1UL << motor)) || !servoData.onlineStatus[motor]) continue;
                    const int32_t target = hotplugHoldMotor[motor] ? servoData.servoAngles[motor] : commandSnapshot.motorTargetRaw[motor];
                    appendServoTarget(&targetBatch, motor, target, commandSnapshot.directSpeed, commandSnapshot.directAcc);
                }
            } else {
                for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; ++joint) {
                    const uint32_t bit = 1UL << joint;
                    feedbackCollisionPidPushActive[joint] = false;
                    // Threshold groups follow joint flexion/extension, never motor ABS direction.
                    feedbackCollisionDriveMotionMode[joint] = localTargets[joint] >= magAngles[joint]
                        ? FEEDBACK_COLLISION_MOTION_TIGHTEN : FEEDBACK_COLLISION_MOTION_RELEASE;
                    if (!(activeJointMask & bit)) { g_jointController.reset(joint); continue; }
                    const bool encoderFault = (sharedAtomicLoad(&sharedData->encoder_fault_bitmap) & bit) != 0;
                    const auto safety = p4::safety::evaluateControlSafety(canBusOnline, sensorData, mappedData, joint, encoderFault);
                    bool eligible = safety.allowed && servoFresh && (feedbackCommandJointMask & bit) &&
                        (calibrationSnapshotValidMask & bit) && !p4::safety::jointHasOverloadFault(joint, overloadFaultLatched);
                    for (uint8_t axis = 0; axis < getJointActuatorCount(joint); ++axis) {
                        JointActuatorItem actuator;
                        if (!getJointActuator(joint, axis, &actuator) || hotplugHoldMotor[actuator.motorChannel]) eligible = false;
                    }
                    if (!safety.allowed) recordControlRuleEvent(output, systemState, targetOwner, safety.ruleId,
                        joint, 0xFF, safety.requestedValue, safety.observedValue);
                    if (g_feedbackCollisionGuard.isHoldActive(joint)) {
                        g_jointController.suspend(joint, servoData, nowMs);
                        appendJointActuatorSnapshotTargets(&targetBatch, joint, servoData, feedbackCollisionHoldAbs[joint],
                            kFeedbackServoSpeed, SERVO_TARGET_ACC_DEFAULT);
                        continue;
                    }
                    if (!eligible) {
                        g_jointController.reset(joint);
                        if (servoFresh) appendJointActuatorHoldTargets(&targetBatch, joint, servoData, kFeedbackServoSpeed, SERVO_TARGET_ACC_DEFAULT);
                        continue;
                    }
                    const auto result = g_jointController.compute(joint, localTargets[joint], g_feedbackEstimates[joint],
                        nowMs, dt, servoData, calibrationSnapshot[joint].result,
                        sharedAtomicLoad(&sharedData->active_motor_mask), sharedData, controlParameters);
                    if (!result.count) {
                        g_jointController.reset(joint);
                        appendJointActuatorHoldTargets(&targetBatch, joint, servoData, kFeedbackServoSpeed, SERVO_TARGET_ACC_DEFAULT);
                        continue;
                    }
                    feedbackCollisionPidPushActive[joint] = result.pushing;
                    for (uint8_t axis = 0; axis < result.count; ++axis) {
                        const uint8_t motor = result.motors[axis];
                        appendServoTarget(&targetBatch, motor, result.targets[axis], kFeedbackServoSpeed, SERVO_TARGET_ACC_DEFAULT);
                        if (result.boundaryClamped) noteTendonLutClamp(sharedData, joint, motor,
                            lroundf(result.rawTargets[axis]), result.targets[axis]);
                        runtime.cycleTrace.pidCounts[motor] = lroundf(fmaxf(-32768, fminf(32767,
                            result.proportional[axis] + result.integral[axis] + result.reversal[axis])));
                        runtime.cycleTrace.proportionalCounts[motor] = lroundf(fmaxf(-32768, fminf(32767, result.proportional[axis])));
                        runtime.cycleTrace.integralCounts[motor] = lroundf(fmaxf(-32768, fminf(32767, result.integral[axis])));
                        runtime.cycleTrace.reversalCounts[motor] = lroundf(fmaxf(-32768, fminf(32767, result.reversal[axis])));
                        runtime.cycleTrace.recoveryCounts[motor] = lroundf(fmaxf(-32768, fminf(32767, result.recovery[axis])));
                        if (result.recovery[axis] != 0) runtime.cycleTrace.recoveryAppliedMask |= 1UL << motor;
                    }
                    if (result.reversalFailed && !(runtime.reversalFailureMask & bit)) {
                        FW_LOG_INFO_PRINTF("[CONTROL] REVERSAL_NO_RESPONSE J%02u: holding measured motor positions\r\n", joint);
                        runtime.reversalFailureMask |= bit;
                    } else if (!result.reversalFailed) runtime.reversalFailureMask &= ~bit;
                }
            }
        }

        const uint32_t recoveredLutClampMask =
            runtime.lutClampEventLatchedBitmap & ~runtime.lutClampObservedBitmap;
        clearRecoveredControlRule(SafetyRuleId::ControlLutBoundaryClamp,
                                  recoveredLutClampMask);
        runtime.lutClampEventLatchedBitmap &= runtime.lutClampObservedBitmap;

        const uint8_t publishOwner =
            sharedAtomicLoad(&sharedData->servo_target_owner);
        const uint8_t publishState =
            sharedAtomicLoad(&sharedData->system_state);
        const bool publishControlEnabled =
            sharedAtomicLoad(&sharedData->control_enabled) != 0;
        const bool publishStateMatchesMode =
            (publishState == SYSTEM_STATE_RUN_FEEDBACK &&
             commandSnapshot.controlMode == CONTROL_MODE_JOINT) ||
            (publishState == SYSTEM_STATE_RUN_DIRECT &&
             commandSnapshot.controlMode == CONTROL_MODE_DIRECT_MOTOR);
        const bool controlTargetPublishAllowed =
            publishControlEnabled &&
            publishOwner == SERVO_TARGET_OWNER_CONTROL &&
            publishStateMatchesMode;
        bool targetBatchPublished = false;
        if (targetBatch.count > 0 && controlTargetPublishAllowed) {
            // 发布前再次检查 owner/state，防止状态切换时泄漏过期的 CONTROL 目标 batch。
            targetBatchPublished =
                p4::actuator::publishLatest(sharedData, targetBatch);
        }
        if (targetBatch.count > 0 && !targetBatchPublished) g_jointController.resetAll();

        // Target、Actual 与成功发布的 Servo CMD 在同一个控制周期内封装。
        // 只有 Gateway 接受的 batch 才记入 CMD，禁止把未执行的候选值画成指令。
        const uint8_t divisor = sharedAtomicLoad(&sharedData->controlTraceDivisor);
        ++runtime.controlTraceSequence;
        if (commandSnapshot.controlMode == CONTROL_MODE_JOINT && divisor != 0 &&
            sharedAtomicLoad(&sharedData->controlTraceRecordingEnabled) && runtime.controlTraceSequence % divisor == 0) {
            ControlTraceData_t& trace = runtime.cycleTrace;
            trace.sequence = runtime.controlTraceSequence;
            trace.sampleDivisor = divisor;
            trace.droppedFrames = runtime.controlTraceDropped;
            trace.servoTimestamp = servoData.timestamp;
            for (uint8_t motor = 0; motor < SERVO_TOTAL_NUM; ++motor) {
                trace.motorActualAbs[motor] = servoData.servoAngles[motor];
                if (servoData.onlineStatus[motor]) trace.servoValidMask |= 1UL << motor;
            }
            trace.targetToken = commandSnapshot.jointCommandToken;
            trace.targetAcceptedTimestamp = commandSnapshot.jointCommandAcceptedMs;
            trace.encoderTimestamp = sensorData.timestamp;
            trace.activeJointMask = feedbackCommandJointMask;
            trace.timestamp = nowMs;
            for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; ++jointIndex) {
                trace.targetDegCenti[jointIndex] =
                    controlTraceAngleCenti(localTargets[jointIndex]);
                // mappedData.angleValues 是上行协议使用的磁编计数，不是 0.01 度。
                trace.actualDegCenti[jointIndex] =
                    controlTraceAngleCenti(magAngles[jointIndex]);
                trace.slowDegCenti[jointIndex] = g_feedbackEstimates[jointIndex].ready
                    ? controlTraceAngleCenti(g_feedbackEstimates[jointIndex].slow) : INT16_MAX;
                if (mappedData.isValid && mappedData.validFlags[jointIndex]) {
                    trace.validJointMask |= 1UL << jointIndex;
                }
            }
            if (targetBatchPublished) {
                for (uint8_t commandIndex = 0;
                     commandIndex < targetBatch.count;
                     ++commandIndex) {
                    const ServoTargetCommand_t& command =
                        targetBatch.commands[commandIndex];
                    if (command.motorChannel < SERVO_TOTAL_NUM) {
                        trace.motorCommandMask |= 1UL << command.motorChannel;
                        trace.motorCommandAbs[command.motorChannel] = command.position;
                    }
                }
            }
            if (!sharedData->controlTraceQueue.send(trace)) ++runtime.controlTraceDropped;
        }

    return true;
}
