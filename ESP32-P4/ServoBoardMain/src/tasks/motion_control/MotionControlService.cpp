/**
 * @file MotionControlService.cpp
 * @brief 运行 P4 的 10 ms 控制编排任务，读取一致快照并调用控制域模块生成舵机目标。
 *
 * 本文件只负责周期调度和模式编排。磁编映射、PID/LUT 计算、碰撞状态机、
 * 输出构建与命令发布分别由独立模块实现，禁止在此直接调用飞特 SDK。
 */
#include "MotionControlService.h"

#include <math.h>
#include <string.h>

#include "ControlSolver.h"
#include "DualTendonControl.h"
#include "JointAngleMapper.h"
#include "ControlCommandSnapshotReader.h"
#include "MotionCompletionEvaluator.h"
#include "ServoTargetBuilder.h"
#include "SingleTendonWorkspace.h"
#include "../../modules/actuator/ActuatorCommandGateway.h"
#include "../../modules/calibration/CalibrationMath.h"
#include "../../modules/calibration/CalibrationRuntimeStore.h"
#include "../../modules/calibration/TendonCalibrationLut.h"
#include "../../modules/runtime/ServiceCyclePolicy.h"
#include "../../config/ControlConfig.h"
#include "../../config/topology/HardwareMap.h"
#include "../../hal/servo/ServoValue.h"
#include "../../generated/UrdfJointLimits.generated.h"
#include "../../platform/rtos/SharedSafetyState.h"
#include "../../platform/log/FirmwareLog.h"
#include "../../runtime/SystemRuntimeContext.h"
#include "../../runtime/ports/SystemEventPort.h"
#include "../../runtime/metrics/TaskRuntimeMetrics.h"
#include "../../safety/policies/ControlInputSafetyPolicy.h"
#include "../../safety/policies/OverloadSafetyPolicy.h"
#include "../../safety/policies/CollisionSafetyPolicy.h"
#include "../../safety/policies/ControlBoundarySafetyPolicy.h"
#include "../../safety/SafetyEventReporter.h"

using namespace mmhand::config::control;

static ControlSolver g_controlSolver;
// 碰撞检测历史放在静态区，避免挤占 10 ms 控制任务栈。
static FeedbackCollisionGuard g_feedbackCollisionGuard;

static uint32_t g_controlPidDiagLastLogMs = 0;
static uint8_t g_controlPidDiagNextJoint = 0;
// LUT 钳位事件按关节做边沿锁存；持续贴边期间只保留第一条证据。
static uint32_t g_lutClampEventLatchedBitmap = 0;
static uint32_t g_lutClampObservedBitmap = 0;

/**
 * 控制服务跨周期状态。所有成员只由 MotionControlTask 访问，固定放在静态区，
 * 避免 100 Hz 实时任务反复分配内存或占用大块任务栈。
 */
struct ControlServiceRuntime {
    bool initialized;
    float localTargets[ENCODER_TOTAL_NUM];
    float magAngles[ENCODER_TOTAL_NUM];
    int32_t absolutePositionRaw[ENCODER_TOTAL_NUM];
    int32_t absolutePositionTendon[ENCODER_TOTAL_NUM];
    int32_t outPulsesTendon[ENCODER_TOTAL_NUM];
    RemoteSensorData_t sensorData;
    MappedAngleData_t mappedData;
    uint32_t overloadFaultLatched;
    uint8_t overloadHighCount[SERVO_TOTAL_NUM];
    int32_t feedbackCollisionHoldAbs[ENCODER_TOTAL_NUM][MAX_JOINT_ACTUATORS];
    bool feedbackCollisionPidPushActive[ENCODER_TOTAL_NUM];
    bool feedbackCollisionWorkspaceBoundaryReached[ENCODER_TOTAL_NUM];
    uint32_t lastFeedbackCollisionSensorTimestamp;
    uint8_t upperWorkspaceHighCount[ENCODER_TOTAL_NUM];
    uint32_t upperWorkspaceWarningBitmap;
    // 目标钳位按关节锁存；目标回到动态范围后清除，下一次越界才重新上报。
    uint32_t targetClampEventLatchedBitmap;
    uint32_t lastOverloadFaultResetToken;
    ControlCommandSnapshot_t commandSnapshot;
    uint32_t lastPidComputeMs;
    bool prevMotorOnline[SERVO_TOTAL_NUM];
    bool servoOnlineBaselineValid;
    bool hotplugHoldMotor[SERVO_TOTAL_NUM];
    uint32_t lastMotorCommandToken;
    uint32_t lastMotorSweepCommandToken;
    uint32_t lastJointCommandToken;
    uint8_t lastMotorDirectSource;
    int16_t prevSweepRawTarget[SERVO_TOTAL_NUM];
    int32_t sweepExpandedTarget[SERVO_TOTAL_NUM];
    bool sweepAnchorValid[SERVO_TOTAL_NUM];
    uint8_t directRunDoneStableCount;
    uint8_t feedbackRunDoneStableCount;
    uint32_t feedbackRunToken;
    uint32_t feedbackRunStartMs;
    uint32_t feedbackRunJointMask;
    float feedbackRunTargetDeg[ENCODER_TOTAL_NUM];
    bool feedbackRunTracking;
    bool feedbackRunDoneEventQueued;
    bool feedbackRunTimeoutEventQueued;
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

/** 记录本周期发生的 LUT 钳位；同一关节离开边界后才会重新上报。 */
static void noteLutClampEvent(SystemRuntimeContext* sharedData,
                              const ServiceOutputPort& output,
                              uint8_t jointIndex,
                              uint8_t motorChannel,
                              int32_t requestedAbs,
                              int32_t appliedAbs)
{
    if (jointIndex >= ENCODER_TOTAL_NUM) {
        return;
    }
    const uint32_t bit = 1UL << jointIndex;
    g_lutClampObservedBitmap |= bit;
    if ((g_lutClampEventLatchedBitmap & bit) != 0) {
        return;
    }
    g_lutClampEventLatchedBitmap |= bit;
    recordControlRuleEvent(output,
                           sharedAtomicLoad(&sharedData->system_state),
                           sharedAtomicLoad(&sharedData->servo_target_owner),
                           SafetyRuleId::ControlLutBoundaryClamp,
                           jointIndex,
                           motorChannel,
                           requestedAbs,
                           appliedAbs);
}

static bool shouldLogControlPidDiag(uint8_t jointIndex,
                                    uint32_t nowMs,
                                    uint32_t activeJointMask)
{
    if constexpr (kPidDiagLogEnabled) {
        if (jointIndex >= ENCODER_TOTAL_NUM) {
            return false;
        }
        // 全手控制仍只允许每 50 ms 输出一条 PID 日志，但在 active joint 之间
        // 轮询。旧实现总由循环首位 J00 抢占全局时间槽，导致日志长期只有 M00/M01。
        if ((uint32_t)(nowMs - g_controlPidDiagLastLogMs) <
            kPidDiagLogPeriodMs) {
            return false;
        }

        const uint32_t validMask = activeJointMask &
            ((1UL << ENCODER_TOTAL_NUM) - 1UL);
        if (validMask == 0) {
            return false;
        }
        uint8_t selectedJoint = ENCODER_TOTAL_NUM;
        for (uint8_t offset = 0; offset < ENCODER_TOTAL_NUM; offset++) {
            const uint8_t candidate =
                (uint8_t)((g_controlPidDiagNextJoint + offset) % ENCODER_TOTAL_NUM);
            if ((validMask & (1UL << candidate)) != 0) {
                selectedJoint = candidate;
                break;
            }
        }
        const uint8_t nextJointAfterLog =
            (uint8_t)((selectedJoint + 1U) % ENCODER_TOTAL_NUM);
        if (jointIndex != selectedJoint) {
            // 被选中的关节可能因离线或保护分支没有进入 PID。超过两个日志周期
            // 后允许下一个实际运行到这里的关节接管，避免诊断轮询永久卡死。
            if ((uint32_t)(nowMs - g_controlPidDiagLastLogMs) <
                (kPidDiagLogPeriodMs * 2U)) {
                return false;
            }
        }
        g_controlPidDiagLastLogMs = nowMs;
        g_controlPidDiagNextJoint = nextJointAfterLog;
        return true;
    }
    return false;
}

// 判断舵机实际 ABS 是否已抵达当前推进方向的 LUT 边界。
// pushDeltaAbs 只表示正负方向，是否抵边始终由 actualAbs 与 LUT Min/Max 决定。
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

static uint8_t appendDualTendonTargets(ServoTargetBatch_t* batch,
                                       SystemRuntimeContext* sharedData,
                                       const ServiceOutputPort& output,
                                       uint32_t nowMs,
                                       uint8_t jointIndex,
                                       float qRefDeg,
                                       float qFbDeg,
                                       const ServoAngleData_t& servoData,
                                       const ServoTelemetryData_t& telemetryData,
                                       bool telemetryFresh,
                                       uint16_t speed,
                                       uint8_t acc,
                                       bool* outPidPushActive,
                                       bool* outWorkspaceBoundaryReached)
{
    // 将逻辑关节目标转换成两个拮抗电机目标，并返回下一周期碰撞检测需要的
    // PID 推进与“实际抵达 LUT 边界”证据；用户 LOCK 限位仍在最后独立执行。
    if (outPidPushActive) *outPidPushActive = false;
    if (outWorkspaceBoundaryReached) *outWorkspaceBoundaryReached = false;
    if (!batch || !sharedData) {
        return 0;
    }

    DualTendonMotorTargets tendonTargets;
    memset(&tendonTargets, 0, sizeof(tendonTargets));
    if (!computeDualTendonMotorTargets(jointIndex,
                                       qRefDeg,
                                       qFbDeg,
                                       &servoData,
                                       &tendonTargets)) {
        resetDualTendonController(jointIndex);
        return 0;
    }

    JointActuatorRuntimeTarget runtimeTargets[MAX_JOINT_ACTUATORS];
    uint8_t actualBoundaryReached[2] = {0, 0};
    bool anyLutTargetClamped = false;
    uint8_t firstClampedMotor = 0xFF;
    int32_t firstRequestedAbs = 0;
    int32_t firstAppliedAbs = 0;
    uint8_t targetCount = 0;
    const uint32_t diagNowMs = nowMs;
    const bool logPidDiag =
        sharedAtomicLoad(&sharedData->system_state) == SYSTEM_STATE_RUN_FEEDBACK &&
        sharedAtomicLoad(&sharedData->control_enabled) != 0 &&
        shouldLogControlPidDiag(
            jointIndex,
            diagNowMs,
            sharedAtomicLoad(&sharedData->active_joint_mask));
    for (uint8_t i = 0; i < tendonTargets.count && targetCount < MAX_JOINT_ACTUATORS; i++) {
        const uint8_t motorChannel = tendonTargets.motorChannel[i];
        if (motorChannel >= SERVO_TOTAL_NUM || !isMotorChannelValid(motorChannel)) {
            resetDualTendonController(jointIndex);
            return 0;
        }
        const uint32_t bit = (1UL << motorChannel);
        uint32_t validBitmap = 0;
        {
            SharedSafetyGuard guard;
            validBitmap = sharedAtomicLoad(&sharedData->motor_abs_limit_valid_bitmap);
        }
        // A valid calibrated range remains the feedback eligibility gate. LOCK
        // independently decides whether that range also clamps motor ABS output.
        // LOCK only enables the ABS min/max output clamp; it must not disable
        // feedback control for an otherwise calibrated dual-tendon joint.
        if ((validBitmap & bit) == 0) {
            resetDualTendonController(jointIndex);
            return 0;
        }
        JointActuatorRuntimeTarget& target = runtimeTargets[targetCount++];
        target.motorChannel = motorChannel;
        target.sign = 1;
        // 与单电机反馈使用相同的 LOCK 语义：锁定通道限制在标定 ABS 范围内，
        // 未锁定通道继续保持反馈控制，并允许运动到该范围之外。
        target.targetAbs = clampServoPosByMotorAbsLimit(sharedData,
                                                        motorChannel,
                                                        tendonTargets.targetAbs[i]);
        if (tendonTargets.lutTargetClamped[i] != 0 && !anyLutTargetClamped) {
            anyLutTargetClamped = true;
            firstClampedMotor = motorChannel;
            firstRequestedAbs = tendonTargets.rawTargetAbs[i];
            firstAppliedAbs = tendonTargets.targetAbs[i];
        }
        if (outPidPushActive &&
            fabsf(tendonTargets.pidCorrection[i]) >= 1.0f &&
            tendonTargets.pidCorrection[i] * tendonTargets.motorErr[i] > 0.0f) {
            *outPidPushActive = true;
        }
        actualBoundaryReached[i] = p4::safety::actualAbsReachedDirectionalBoundary(
            servoData.servoAngles[motorChannel],
            tendonTargets.rawTargetAbs[i] -
                servoData.servoAngles[motorChannel],
            tendonTargets.lutMinAbs[i],
            tendonTargets.lutMaxAbs[i],
            kCollisionLutBoundaryToleranceCounts) ? 1 : 0;
        // 双电机任意一侧实际抵达其推进方向边界时，才认为关节工作空间耗尽。
        // “PID 原始目标被 LUT 钳位”本身不再禁止碰撞 HOLD。
        if (outWorkspaceBoundaryReached && actualBoundaryReached[i] != 0) {
            *outWorkspaceBoundaryReached = true;
        }
    }

    if (anyLutTargetClamped) {
        noteLutClampEvent(sharedData,
                          output,
                          jointIndex,
                          firstClampedMotor,
                          firstRequestedAbs,
                          firstAppliedAbs);
    }

    if (logPidDiag && targetCount == 2) {
        const uint8_t m0 = runtimeTargets[0].motorChannel;
        const uint8_t m1 = runtimeTargets[1].motorChannel;
        const int32_t abs0 = servoData.servoAngles[m0];
        const int32_t abs1 = servoData.servoAngles[m1];
        const int16_t load0 = telemetryFresh ? mmhand::servo::loadMagnitude(telemetryData.load[m0]) : 0;
        const int16_t load1 = telemetryFresh ? mmhand::servo::loadMagnitude(telemetryData.load[m1]) : 0;
        const int16_t speed0 = telemetryFresh ? telemetryData.speed[m0] : 0;
        const int16_t speed1 = telemetryFresh ? telemetryData.speed[m1] : 0;
        const uint32_t lockedBitmap =
            sharedAtomicLoad(&sharedData->motor_abs_limit_locked_bitmap);
        FW_LOG_INFO_PRINTF(
            "[PID_DUAL] t=%lu J=%02u qr=%.2f qf=%.2f qe=%.2f "
            "M%02u a=%ld rt=%ld c=%d l=%ld e=%.1f o=%.1f tc=%u br=%u ld=%d s=%d k=%u "
            "M%02u a=%ld rt=%ld c=%d l=%ld e=%.1f o=%.1f tc=%u br=%u ld=%d s=%d k=%u "
            "tel=%u\r\n",
            (unsigned long)diagNowMs,
            (unsigned)jointIndex,
            (double)qRefDeg,
            (double)qFbDeg,
            (double)(qRefDeg - qFbDeg),
            (unsigned)m0,
            (long)abs0,
            (long)tendonTargets.rawTargetAbs[0],
            (int)runtimeTargets[0].targetAbs,
            (long)((int32_t)runtimeTargets[0].targetAbs - abs0),
            (double)tendonTargets.motorErr[0],
            (double)tendonTargets.pidCorrection[0],
            (unsigned)tendonTargets.lutTargetClamped[0],
            (unsigned)actualBoundaryReached[0],
            (int)load0,
            (int)speed0,
            (unsigned)((lockedBitmap & (1UL << m0)) != 0),
            (unsigned)m1,
            (long)abs1,
            (long)tendonTargets.rawTargetAbs[1],
            (int)runtimeTargets[1].targetAbs,
            (long)((int32_t)runtimeTargets[1].targetAbs - abs1),
            (double)tendonTargets.motorErr[1],
            (double)tendonTargets.pidCorrection[1],
            (unsigned)tendonTargets.lutTargetClamped[1],
            (unsigned)actualBoundaryReached[1],
            (int)load1,
            (int)speed1,
            (unsigned)((lockedBitmap & (1UL << m1)) != 0),
            (unsigned)telemetryFresh);
    }

    return appendJointActuatorRuntimeTargets(batch, runtimeTargets, targetCount, speed, acc);
}

/** 初始化控制域的跨周期状态；本函数不创建任务，也不进入循环。 */
bool initMotionControlService(SystemRuntimeContext* sharedData, uint32_t nowMs)
{
    if (!sharedData) {
        return false;
    }

    memset(&g_controlRuntime, 0, sizeof(g_controlRuntime));
    g_controlSolver.begin();
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
    g_controlRuntime.lastMotorSweepCommandToken =
        g_controlRuntime.commandSnapshot.motorSweepCommandToken;
    g_controlRuntime.lastJointCommandToken =
        g_controlRuntime.commandSnapshot.jointCommandToken;
    g_controlRuntime.lastMotorDirectSource =
        g_controlRuntime.commandSnapshot.motorDirectCommandSource;
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

    // 使用引用保留原控制公式的可读命名，同时把所有历史量集中到运行上下文。
    ControlServiceRuntime& runtime = g_controlRuntime;
    auto& localTargets = runtime.localTargets;
    auto& magAngles = runtime.magAngles;
    auto& absolutePositionRaw = runtime.absolutePositionRaw;
    auto& absolutePositionTendon = runtime.absolutePositionTendon;
    auto& outPulsesTendon = runtime.outPulsesTendon;
    auto& sensorData = runtime.sensorData;
    auto& mappedData = runtime.mappedData;
    auto& overloadFaultLatched = runtime.overloadFaultLatched;
    auto& overloadHighCount = runtime.overloadHighCount;
    auto& feedbackCollisionHoldAbs = runtime.feedbackCollisionHoldAbs;
    auto& feedbackCollisionPidPushActive = runtime.feedbackCollisionPidPushActive;
    auto& feedbackCollisionWorkspaceBoundaryReached =
        runtime.feedbackCollisionWorkspaceBoundaryReached;
    auto& lastFeedbackCollisionSensorTimestamp =
        runtime.lastFeedbackCollisionSensorTimestamp;
    auto& upperWorkspaceHighCount = runtime.upperWorkspaceHighCount;
    auto& upperWorkspaceWarningBitmap = runtime.upperWorkspaceWarningBitmap;
    auto& lastOverloadFaultResetToken = runtime.lastOverloadFaultResetToken;
    auto& commandSnapshot = runtime.commandSnapshot;
    auto& prevMotorOnline = runtime.prevMotorOnline;
    auto& hotplugHoldMotor = runtime.hotplugHoldMotor;
    auto& lastMotorCommandToken = runtime.lastMotorCommandToken;
    auto& lastMotorSweepCommandToken = runtime.lastMotorSweepCommandToken;
    auto& lastJointCommandToken = runtime.lastJointCommandToken;
    auto& lastMotorDirectSource = runtime.lastMotorDirectSource;
    auto& prevSweepRawTarget = runtime.prevSweepRawTarget;
    auto& sweepExpandedTarget = runtime.sweepExpandedTarget;
    auto& sweepAnchorValid = runtime.sweepAnchorValid;
    auto& directRunDoneStableCount = runtime.directRunDoneStableCount;
    auto& feedbackRunDoneStableCount = runtime.feedbackRunDoneStableCount;
    auto& feedbackRunToken = runtime.feedbackRunToken;
    auto& feedbackRunStartMs = runtime.feedbackRunStartMs;
    auto& feedbackRunJointMask = runtime.feedbackRunJointMask;
    auto& feedbackRunTargetDeg = runtime.feedbackRunTargetDeg;
    auto& feedbackRunTracking = runtime.feedbackRunTracking;
    auto& feedbackRunDoneEventQueued = runtime.feedbackRunDoneEventQueued;
    auto& feedbackRunTimeoutEventQueued = runtime.feedbackRunTimeoutEventQueued;

        if (!p4::control::readCommandSnapshot(
                sharedData, &commandSnapshot)) {
            // 命令互斥锁短暂繁忙时跳过本帧；任务层仍保持固定周期。
            return false;
        }

        const uint32_t motorCommandToken = commandSnapshot.motorCommandToken;
        if (motorCommandToken != lastMotorCommandToken)
        {
            // 新的电机直控命令会解除当前热插拔保持状态。
            memset(hotplugHoldMotor, 0, sizeof(hotplugHoldMotor));
            lastMotorCommandToken = motorCommandToken;
        }

        const uint32_t motorSweepCommandToken = commandSnapshot.motorSweepCommandToken;
        if (motorSweepCommandToken != lastMotorSweepCommandToken)
        {
            // sweep 命令变化可能有意跨过 raw 回绕点。
            memset(hotplugHoldMotor, 0, sizeof(hotplugHoldMotor));
            lastMotorSweepCommandToken = motorSweepCommandToken;
        }

        const uint32_t jointCommandToken = commandSnapshot.jointCommandToken;
        if (jointCommandToken != lastJointCommandToken)
        {
            // 新关节目标会开启新的单次反馈完成判定窗口。
            memset(hotplugHoldMotor, 0, sizeof(hotplugHoldMotor));
            feedbackRunDoneStableCount = 0;
            lastJointCommandToken = jointCommandToken;
        }

        const uint8_t motorDirectSource = commandSnapshot.motorDirectCommandSource;
        if (motorDirectSource != lastMotorDirectSource)
        {
            // 单圈命令和 sweep 命令使用不同的锚定语义。
            memset(sweepAnchorValid, 0, sizeof(sweepAnchorValid));
            lastMotorDirectSource = motorDirectSource;
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
        memset(&targetBatch, 0, sizeof(targetBatch));
        // 每个循环生成一个 batch 并用 overwrite 发布，下游始终看到最新完整命令集。
        targetBatch.timestamp = nowMs;
        targetBatch.source = SERVO_TARGET_OWNER_CONTROL;

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
                sweepAnchorValid[ch] = false;
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
            if (!onlineNow) {
                sweepAnchorValid[ch] = false;
            }
            prevMotorOnline[ch] = onlineNow;
        }
        if (servoData.timestamp != 0) {
            runtime.servoOnlineBaselineValid = true;
        }

        sharedAtomicFetchOr(&sharedData->overload_fault_bitmap, overloadFaultLatched);
        overloadFaultLatched = sharedAtomicLoad(&sharedData->overload_fault_bitmap);

        // 按关节缓存当前电机位置，分别保存 raw 坐标和腱绳坐标。
        // PID 使用腱绳空间，总线命令使用 raw 空间。
        memset(absolutePositionRaw, 0, sizeof(absolutePositionRaw));
        memset(absolutePositionTendon, 0, sizeof(absolutePositionTendon));
        for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++)
        {
            const int ch = findPrimaryMotorChannelForJoint(jointIndex);
            if (ch >= 0 && servoData.onlineStatus[ch]) {
                const int32_t rawAbs = servoData.servoAngles[ch];
                absolutePositionRaw[jointIndex] = rawAbs;
                absolutePositionTendon[jointIndex] =
                    motorRawAbsToTendonAbs((uint8_t)ch, rawAbs);
            }
        }


        bool canBusOnline = false;
        if (sharedData->canRxQueue.peek(&sensorData) && sensorData.isValid) {
            // 超过 CAN 超时时间后，磁编码器反馈视为过期。
            canBusOnline = (nowMs - sensorData.timestamp) <= kCanBusOfflineTimeoutMs;
        }

        memset(&mappedData, 0, sizeof(mappedData));
        mappedData.timestamp = nowMs;
        mappedData.isValid = canBusOnline;

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

                if (!g_jointCalibResult[i].success) {
                    // 未标定关节故意标记为无效，禁止进入反馈 PID。
                    mappedData.angleValues[i] = 0;
                    mappedData.validFlags[i] = 0;
                    magAngles[i] = 0.0f;
                    upperWorkspaceHighCount[i] = 0;
                    upperWorkspaceWarningBitmap &= ~bit;
                    continue;
                }

                const JointCalibrationResult& calib = g_jointCalibResult[i];
                float mappedDeg = 0.0f;
                if (calib.mappingVersion == kManualScaledUrdfFeedbackMappingVersion) {
                    // 控制环只接受当前 Manual-scaled 映射；旧 Auto 映射不得回退运行。
                    EncoderUrdfMapResult mapping;
                    const bool mapped =
                        mapManualScaledEncoderToUrdf(
                            (uint8_t)i,
                            calib,
                            g_manualEncoderReference[i],
                            sensorData.encoderValues[i],
                            g_encoderDirection[i],
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

        // 碰撞检测只消费磁编新帧。200 ms 速度窗与 400 ms 进展窗同步滚动，
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
                input.feedbackValid = true;
                input.targetDeg = localTargets[jointIndex];
                input.feedbackDeg = magAngles[jointIndex];
                input.pidPushActive =
                    feedbackCollisionPidPushActive[jointIndex];
                input.workspaceBoundaryReached =
                    feedbackCollisionWorkspaceBoundaryReached[jointIndex];
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
                    "v200=%.3fdeg/s source=velocity candidate_ms=%lu\r\n",
                    (unsigned)jointIndex,
                    (double)localTargets[jointIndex],
                    (double)magAngles[jointIndex],
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
                g_controlSolver.resetJoint(jointIndex);
                if (isDualTendonJoint(jointIndex)) {
                    resetDualTendonController(jointIndex);
                }
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
                    "v200=%.3fdeg/s progress400=%.3fdeg "
                    "candidate_ms=%lu\r\n",
                    (unsigned)jointIndex,
                    (double)collisionResult.holdTargetDeg,
                    (double)collisionResult.holdAngleDeg,
                    (double)collisionResult.directionVelocityDegPerSec,
                    (double)collisionResult.directionProgressDeg,
                    (unsigned long)collisionResult.candidateAgeMs);
            }

            if (collisionResult.holdReleased) {
                p4::safety::clearSafetyRuleEvent(
                    SafetyRuleId::ControlCollisionHold,
                    jointIndex);
                g_controlSolver.resetJoint(jointIndex);
                if (isDualTendonJoint(jointIndex)) {
                    resetDualTendonController(jointIndex);
                }
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
                feedbackCollisionWorkspaceBoundaryReached[jointIndex] = false;
            }
        }
        const bool feedbackOneShotActive =
            feedbackRunningState &&
            commandSnapshot.controlMode == CONTROL_MODE_JOINT;
        if (feedbackOneShotActive) {
            // 为本次命令 generation 记录限幅后的目标快照。
            // 完成判定必须针对一个稳定目标，而不是后续 UI 更新。
            if (!feedbackRunTracking || feedbackRunToken != jointCommandToken) {
                memcpy(feedbackRunTargetDeg, localTargets, sizeof(feedbackRunTargetDeg));
                feedbackRunJointMask = feedbackCommandJointMask;
                feedbackRunToken = jointCommandToken;
                feedbackRunStartMs = nowMs;
                feedbackRunDoneStableCount = 0;
                feedbackRunTracking = true;
                feedbackRunDoneEventQueued = false;
                feedbackRunTimeoutEventQueued = false;
            }
        } else {
            feedbackRunTracking = false;
            feedbackRunDoneStableCount = 0;
            feedbackRunDoneEventQueued = false;
            feedbackRunTimeoutEventQueued = false;
        }

        // 只有 CONTROL owner 可以构建舵机目标。标定期间，上面的角度映射、
        // 遥测和调试发布仍会继续，但不能与标定生产者竞争单槽 servoTargetQueue。
        const bool controlTargetBuildAllowed = controlEnabled;
        if (!controlTargetBuildAllowed || !feedbackOutputState) {
            // 双腱 PID 含历史项；只要反馈输出没有实际驱动关节，就清空这些状态。
            for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++) {
                if (isDualTendonJoint(jointIndex)) {
                    resetDualTendonController(jointIndex);
                }
            }
        }
        if (controlTargetBuildAllowed) {
            const uint32_t elapsedPidMs = nowMs - runtime.lastPidComputeMs;
            runtime.lastPidComputeMs = nowMs;
            float pidDtSeconds = (float)elapsedPidMs * 0.001f;
            // 首次进入或调度停顿后使用标称周期，避免过期时间间隔造成 I/D 冲击。
            if (!isfinite(pidDtSeconds) || pidDtSeconds < 0.001f || pidDtSeconds > 0.100f) {
                pidDtSeconds = 0.010f;
            }
            // PID 使用腱绳空间 ABS，使正修正在 raw ABS 方向相反的电机上
            // 仍表示相同的“收紧腱绳”含义。单电机输出会把最终腱绳空间目标转回 raw。
            g_controlSolver.compute(localTargets,
                                    magAngles,
                                    absolutePositionTendon,
                                    outPulsesTendon,
                                    pidDtSeconds);
            for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++) {
                if (g_feedbackCollisionGuard.isHoldActive(jointIndex)) {
                    g_controlSolver.resetJoint(jointIndex);
                }
            }
            if (commandSnapshot.controlMode == CONTROL_MODE_DIRECT_MOTOR)
            {
                // 直控模式绕过关节 PID，将上位机电机目标直接发给对应物理通道。
                const uint32_t activeMotorMask =
                    sharedAtomicLoad(&sharedData->active_motor_mask) &
                    validMotorChannelMask();
                for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++)
                {
                    if (((activeMotorMask >> ch) & 0x1UL) == 0) {
                        continue;
                    }
                    int32_t targetPos = servoData.servoAngles[ch];
                    if (!hotplugHoldMotor[ch]) {
                        if (motorDirectSource == MOTOR_DIRECT_SOURCE_SWEEP) {
                            // sweep 目标是单圈 raw 值，通过增量展开实现连续旋转。
                            const int16_t rawTarget =
                                clampServoSingleTurnRaw(commandSnapshot.motorSweepTargetRaw[ch]);
                            if (!sweepAnchorValid[ch]) {
                                sweepExpandedTarget[ch] =
                                    expandSingleTurnTargetNearCurrent(servoData.servoAngles[ch], rawTarget);
                                prevSweepRawTarget[ch] = rawTarget;
                                sweepAnchorValid[ch] = true;
                            } else if (rawTarget != prevSweepRawTarget[ch]) {
                                const int32_t d = (int32_t)rawTarget - (int32_t)prevSweepRawTarget[ch];
                                sweepExpandedTarget[ch] += wrapServoRawDelta(d);
                                prevSweepRawTarget[ch] = rawTarget;
                            }
                            targetPos = sweepExpandedTarget[ch];
                            sweepExpandedTarget[ch] = targetPos;
                        } else if (motorDirectSource == MOTOR_DIRECT_SOURCE_ABSOLUTE) {
                            // absolute 来源本身已经使用多圈舵机计数。
                            sweepAnchorValid[ch] = false;
                            targetPos = commandSnapshot.motorTargetRaw[ch];
                        } else {
                            // 旧单圈来源会选择当前反馈附近最近的等价绝对目标。
                            sweepAnchorValid[ch] = false;
                            targetPos = expandSingleTurnTargetNearCurrent(
                                servoData.servoAngles[ch],
                                clampServoSingleTurnRaw(commandSnapshot.motorTargetRaw[ch])
                            );
                        }
                    } else {
                        sweepAnchorValid[ch] = false;
                    }
                    appendServoTarget(&targetBatch, ch, targetPos,
                                      SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
                }
            }
            else
            {
            // 每个控制周期重新收集仍处于 LUT 钳位的关节，周期结束后解除已离边界的锁存。
            g_lutClampObservedBitmap = 0;
            for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++)
            {
                // 关节模式会对每个关节独立决定：保持、使用双腱控制或输出普通 PID 目标。
                const int jointMotorCh =
                    findPrimaryMotorChannelForJoint(jointIndex);
                const bool servoOnline = (jointMotorCh >= 0) && (servoData.onlineStatus[jointMotorCh] != 0);
                bool overloadFaultActive =
                    p4::safety::jointHasOverloadFault(
                        jointIndex, overloadFaultLatched);

                const bool encoderFeedbackFault =
                    (sharedAtomicLoad(&sharedData->encoder_fault_bitmap) &
                     (1UL << jointIndex)) != 0;
                const p4::safety::ControlSafetyDecision safetyDecision =
                    p4::safety::evaluateControlSafety(canBusOnline,
                                                      sensorData,
                                                      mappedData,
                                                      jointIndex,
                                                      encoderFeedbackFault);
                const SafetyRuleId inputRules[] = {
                    SafetyRuleId::ControlCanFeedbackInvalid,
                    SafetyRuleId::ControlJointIndexInvalid,
                    SafetyRuleId::ControlEncoderFault,
                    SafetyRuleId::ControlMappedAngleInvalid,
                };
                for (SafetyRuleId inputRule : inputRules) {
                    if (inputRule == safetyDecision.ruleId) continue;
                    p4::safety::clearSafetyRuleEvent(inputRule, jointIndex);
                }
                if (!safetyDecision.allowed) {
                    recordControlRuleEvent(
                        output,
                        systemState,
                        targetOwner,
                        safetyDecision.ruleId,
                        jointIndex,
                        0xFF,
                        safetyDecision.requestedValue,
                        safetyDecision.observedValue);
                }
                bool emergency =
                    (!normalOutputAllowed) || !safetyDecision.allowed;
                emergency = emergency || overloadFaultActive;
                const bool hotplugHoldActive =
                    (jointMotorCh >= 0) &&
                    (jointMotorCh < SERVO_TOTAL_NUM) &&
                    hotplugHoldMotor[jointMotorCh];
                const bool jointInScope =
                    (activeJointMask & (1UL << jointIndex)) != 0;
                const bool jointActive =
                    (feedbackCommandJointMask & (1UL << jointIndex)) != 0;
                feedbackCollisionPidPushActive[jointIndex] = false;
                feedbackCollisionWorkspaceBoundaryReached[jointIndex] = false;

                if (!jointInScope)
                {
                    // 未激活关节不发布主动命令，并丢弃双腱状态，便于重新启用时干净启动。
                    if (isDualTendonJoint(jointIndex)) {
                        resetDualTendonController(jointIndex);
                    }
                    continue;
                }

                if (emergency)
                {
                    // 反馈故障、总线丢失、输出关闭和过载都会统一进入保持当前位置行为。
                    if (isDualTendonJoint(jointIndex)) {
                        resetDualTendonController(jointIndex);
                    }
                    appendJointActuatorHoldTargets(&targetBatch, jointIndex, servoData,
                                                   SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
                    continue;
                }

                if (hotplugHoldActive)
                {
                    // 刚上线的电机会保持在反馈位置，直到操作端命令它有意移动。
                    if (isDualTendonJoint(jointIndex)) {
                        resetDualTendonController(jointIndex);
                    }
                    appendJointActuatorHoldTargets(&targetBatch, jointIndex, servoData,
                                                   SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
                    continue;
                }

                if (!jointActive)
                {
                    // 单行角度命令只允许掩码内关节进入 PID。其余 active joint
                    // 持续跟随本周期实际 ABS，避免同一手指受力时因磁编联动而
                    // 追踪旧角度目标，进而误带动 J01/J02 等相邻关节。
                    g_controlSolver.resetJoint(jointIndex);
                    if (isDualTendonJoint(jointIndex)) {
                        resetDualTendonController(jointIndex);
                    }
                    appendJointActuatorHoldTargets(&targetBatch,
                                                   jointIndex,
                                                   servoData,
                                                   SERVO_TARGET_SPEED_DEFAULT,
                                                   SERVO_TARGET_ACC_DEFAULT);
                    continue;
                }

                if (g_feedbackCollisionGuard.isHoldActive(jointIndex))
                {
                    if (isDualTendonJoint(jointIndex)) {
                        resetDualTendonController(jointIndex);
                    }
                    appendJointActuatorSnapshotTargets(
                        &targetBatch,
                        jointIndex,
                        servoData,
                        feedbackCollisionHoldAbs[jointIndex],
                        kFeedbackServoSpeed,
                        SERVO_TARGET_ACC_DEFAULT);
                    localTargets[jointIndex] =
                        g_feedbackCollisionGuard.holdAngleDeg(jointIndex);
                    continue;
                }

                int32_t targetPos = absolutePositionRaw[jointIndex];

                // 只有激活关节才跟随 PID 并输出舵机目标。
                const bool dualTendonJoint = isDualTendonJoint(jointIndex);
                if (jointActive && dualTendonJoint) {
                    // 拮抗关节需要已标定的双电机目标；计算失败时回退为保持所有执行器。
                    const uint8_t appended =
                        appendDualTendonTargets(&targetBatch,
                                                sharedData,
                                                output,
                                                nowMs,
                                                jointIndex,
                                                localTargets[jointIndex],
                                                magAngles[jointIndex],
                                                servoData,
                                                telemetryData,
                                                telemetryFresh,
                                                kFeedbackServoSpeed,
                                                SERVO_TARGET_ACC_DEFAULT,
                                                &feedbackCollisionPidPushActive[jointIndex],
                                                &feedbackCollisionWorkspaceBoundaryReached[jointIndex]);
                    if (appended == 0) {
                        appendJointActuatorHoldTargets(&targetBatch, jointIndex, servoData,
                                                       SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
                    }
                    continue;
                }

                if (jointActive) {
                    // 普通单电机关节：先把腱绳空间 PID 结果转回电机 raw ABS，
                    // 再强制限制到 LUT 实测工作空间，最后应用用户 LOCK 限位。
                    bool lutWorkspaceValid = false;
                    bool pidTargetLutClamped = false;
                    int32_t lutWorkspaceMinAbs = 0;
                    int32_t lutWorkspaceMaxAbs = 0;
                    int32_t actualMotorAbs = targetPos;
                    int32_t rawPidTargetAbs = targetPos;
                    if (jointMotorCh >= 0 && jointMotorCh < SERVO_TOTAL_NUM) {
                        const uint8_t motorCh = (uint8_t)jointMotorCh;
                        actualMotorAbs = servoData.servoAngles[motorCh];
                        rawPidTargetAbs =
                            motorTendonAbsToRawAbs(motorCh, outPulsesTendon[jointIndex]);
                        int32_t lutClampedTarget = rawPidTargetAbs;
                        lutWorkspaceValid =
                            p4::control::clampSingleTendonTargetByLut(
                            jointIndex,
                            motorCh,
                            rawPidTargetAbs,
                            &lutClampedTarget,
                            &lutWorkspaceMinAbs,
                            &lutWorkspaceMaxAbs);
                        if (lutWorkspaceValid) {
                            pidTargetLutClamped =
                                lutClampedTarget != rawPidTargetAbs;
                            if (pidTargetLutClamped) {
                                noteLutClampEvent(sharedData,
                                                  output,
                                                  jointIndex,
                                                  motorCh,
                                                  rawPidTargetAbs,
                                                  lutClampedTarget);
                            }
                            targetPos = clampServoPosByMotorAbsLimit(sharedData,
                                                                     motorCh,
                                                                     lutClampedTarget);
                        } else {
                            // 标定结果或电机映射异常时保持当前位置，不能绕过
                            // LUT 工作空间继续发送位置式 PID 输出。
                            targetPos = servoData.servoAngles[motorCh];
                        }
                    } else {
                        targetPos = outPulsesTendon[jointIndex];
                    }
                    const float pidOutput =
                        g_controlSolver.getPidOutput(jointIndex);
                    const float angleError =
                        localTargets[jointIndex] - magAngles[jointIndex];
                    feedbackCollisionPidPushActive[jointIndex] =
                        fabsf(pidOutput) >= 1.0f &&
                        pidOutput * angleError > 0.0f;
                    // PID 目标是否被 LUT 钳位只用于诊断。碰撞检测必须读取当前实际 ABS，
                    // 并仅在实际位置已进入本次推进方向的 LUT 边界容差时排除 HOLD。
                    const bool actualBoundaryReached =
                        lutWorkspaceValid &&
                        p4::safety::actualAbsReachedDirectionalBoundary(
                            actualMotorAbs,
                            rawPidTargetAbs - actualMotorAbs,
                            lutWorkspaceMinAbs,
                            lutWorkspaceMaxAbs,
                            kCollisionLutBoundaryToleranceCounts);
                    feedbackCollisionWorkspaceBoundaryReached[jointIndex] =
                        !lutWorkspaceValid || actualBoundaryReached;

                    if (feedbackOutputState && shouldLogControlPidDiag(
                            jointIndex,
                            nowMs,
                            activeJointMask)) {
                        const int32_t currentAbs =
                            (jointMotorCh >= 0 && jointMotorCh < SERVO_TOTAL_NUM)
                                ? servoData.servoAngles[jointMotorCh]
                                : absolutePositionRaw[jointIndex];
                        const int32_t targetLead = (int32_t)targetPos - currentAbs;
                        const int16_t rawLoad =
                            (telemetryFresh && jointMotorCh >= 0 && jointMotorCh < SERVO_TOTAL_NUM)
                                ? telemetryData.load[jointMotorCh]
                                : 0;
                        const int16_t loadMag = telemetryFresh
                            ? mmhand::servo::loadMagnitude(rawLoad)
                            : 0;
                        const int16_t servoSpeed =
                            (telemetryFresh && jointMotorCh >= 0 && jointMotorCh < SERVO_TOTAL_NUM)
                                ? telemetryData.speed[jointMotorCh]
                                : 0;
                        const uint32_t motorBit =
                            (jointMotorCh >= 0 && jointMotorCh < SERVO_TOTAL_NUM)
                                ? (1UL << jointMotorCh)
                                : 0;
                        const uint32_t lockedBitmap =
                            sharedAtomicLoad(&sharedData->motor_abs_limit_locked_bitmap);
                        float lutRefAbs = 0.0f;
                        float lutFbAbs = 0.0f;
                        bool lutValid = false;
                        if (jointMotorCh >= 0 && jointMotorCh < SERVO_TOTAL_NUM) {
                            const JointCalibrationResult& calibration =
                                g_jointCalibResult[jointIndex];
                            // 单环 PID 仍使用角度误差；这里插值得到的模型位置只用于
                            // 诊断，LUT 的 ABS 最小/最大值已在上方参与最终硬钳位。
                            lutValid = calibration.tendonLutMotor[0] ==
                                           (uint8_t)jointMotorCh &&
                                interpolateTendonCalibrationMotorAbs(
                                    calibration,
                                    0,
                                    localTargets[jointIndex],
                                    &lutRefAbs) &&
                                interpolateTendonCalibrationMotorAbs(
                                    calibration,
                                    0,
                                    magAngles[jointIndex],
                                    &lutFbAbs);
                        }
                        FW_LOG_INFO_PRINTF(
                            "[PID_SINGLE] t=%lu J=%02u M=%02d qRef=%.3f qFb=%.3f qErr=%.3f "
                            "out=%.2f abs=%ld cmd=%d lead=%ld load=%d mag=%d speed=%d "
                            "lock=%u telem=%u lutValid=%u lutRef=%.2f lutFb=%.2f "
                            "lutDelta=%.2f rawTarget=%ld lutRange=[%ld,%ld] "
                            "lutClamp=%u boundary=%u\r\n",
                            (unsigned long)nowMs,
                            (unsigned)jointIndex,
                            jointMotorCh,
                            (double)localTargets[jointIndex],
                            (double)magAngles[jointIndex],
                            (double)(localTargets[jointIndex] - magAngles[jointIndex]),
                            (double)g_controlSolver.getPidOutput(jointIndex),
                            (long)currentAbs,
                            (int)targetPos,
                            (long)targetLead,
                            (int)rawLoad,
                            (int)loadMag,
                            (int)servoSpeed,
                            (unsigned)((lockedBitmap & motorBit) != 0),
                            (unsigned)telemetryFresh,
                            lutValid ? 1U : 0U,
                            (double)lutRefAbs,
                            (double)lutFbAbs,
                            (double)(lutRefAbs - lutFbAbs),
                            (long)rawPidTargetAbs,
                            (long)lutWorkspaceMinAbs,
                            (long)lutWorkspaceMaxAbs,
                            pidTargetLutClamped ? 1U : 0U,
                            actualBoundaryReached ? 1U : 0U);
                    }
                }

                if (dualTendonJoint) {
                    // 如果双腱关节走到这个回退路径，不尝试通用镜像，
                    // 而是保持各执行器当前位置。
                    appendJointActuatorHoldTargets(&targetBatch, jointIndex, servoData,
                                                   SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
                } else {
                    appendJointActuatorTargets(&targetBatch, jointIndex, targetPos,
                                               jointActive ? kFeedbackServoSpeed : SERVO_TARGET_SPEED_DEFAULT,
                                               SERVO_TARGET_ACC_DEFAULT);
                }
            }
            const uint32_t recoveredLutClampMask =
                g_lutClampEventLatchedBitmap & ~g_lutClampObservedBitmap;
            clearRecoveredControlRule(SafetyRuleId::ControlLutBoundaryClamp,
                                      recoveredLutClampMask);
            g_lutClampEventLatchedBitmap &= g_lutClampObservedBitmap;
            }
        }

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
        if (targetBatch.count > 0 &&
            controlTargetPublishAllowed) {
            // 发布前再次检查 owner/state，防止状态切换时泄漏过期的 CONTROL 目标 batch。
            p4::actuator::publishLatest(sharedData, targetBatch);
        }

        if (directRunningState &&
            commandSnapshot.controlMode == CONTROL_MODE_DIRECT_MOTOR &&
            commandSnapshot.motorDirectCommandSource == MOTOR_DIRECT_SOURCE_ABSOLUTE) {
            // 当所有激活电机连续多个控制帧保持在容差内时，直控 absolute RUN 完成。
            if (p4::control::directTargetsReached(
                    sharedData, servoData, commandSnapshot)) {
                if (directRunDoneStableCount < 255) {
                    directRunDoneStableCount++;
                }
                if (directRunDoneStableCount >= kDirectDoneStableCycles) {
                    servicePostEvent(
                        output,
                        SYSTEM_EVENT_RUN_DIRECT_DONE,
                        commandSnapshot.motorDirectCommandGeneration,
                        true);
                    directRunDoneStableCount = 0;
                }
            } else {
                directRunDoneStableCount = 0;
            }
        } else {
            directRunDoneStableCount = 0;
        }

        if (feedbackOneShotActive && feedbackRunTracking) {
            // 单次 feedback RUN 根据命令开始时捕获的快照判定完成。
            // 正弦测试会一直运行，直到被显式停止。
            const bool targetsReached =
                p4::control::feedbackTargetsReached(
                sharedData,
                mappedData,
                feedbackRunTargetDeg,
                magAngles,
                feedbackRunJointMask
            );
            if (targetsReached) {
                if (feedbackRunDoneStableCount < 255) {
                    feedbackRunDoneStableCount++;
                }
                if (feedbackRunDoneStableCount >= kFeedbackDoneStableCycles &&
                    !feedbackRunDoneEventQueued) {
                    // 队列满是暂态问题；保留完成事件并重试，
                    // 避免丢弃事件导致 RUN_FEEDBACK 一直锁定。
                    feedbackRunDoneEventQueued = servicePostEvent(
                        output,
                        SYSTEM_EVENT_RUN_FEEDBACK_DONE,
                        feedbackRunToken,
                        true
                    );
                }
            } else {
                feedbackRunDoneStableCount = 0;
            }

            const uint32_t elapsedMs = (uint32_t)(nowMs - feedbackRunStartMs);
            const bool collisionHoldActive =
                sharedAtomicLoad(&sharedData->feedback_collision_hold_bitmap) != 0;
            // 碰撞保持必须持续锁住触发时的舵机 ABS，直到反向目标或模式退出；
            // 此时不能让旧的单次负反馈超时提前结束 RUN_FEEDBACK。
            if (!targetsReached &&
                !collisionHoldActive &&
                !feedbackRunDoneEventQueued &&
                elapsedMs >= kFeedbackRunTimeoutMs &&
                !feedbackRunTimeoutEventQueued) {
                feedbackRunTimeoutEventQueued = servicePostEvent(
                    output,
                    SYSTEM_EVENT_RUN_FEEDBACK_TIMEOUT,
                    feedbackRunToken,
                    true
                );
            }
        }

    return true;
}
