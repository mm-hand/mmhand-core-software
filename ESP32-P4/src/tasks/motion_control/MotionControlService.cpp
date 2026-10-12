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

#include "ControlSolver.h"
#include "DualTendonControl.h"
#include "JointAngleMapper.h"
#include "ControlCommandSnapshotReader.h"
#include "MotionCompletionEvaluator.h"
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

static ControlSolver g_controlSolver;
// 碰撞检测历史放在静态区，避免挤占 10 ms 控制任务栈。
static FeedbackCollisionGuard g_feedbackCollisionGuard;
static uint32_t g_controlPidDiagLastLogMs = 0;
static uint8_t g_controlPidDiagNextJoint = 0;

/** 单腱换向连续补绳状态，只由 10 ms 控制服务访问。 */
struct SingleTendonDirectionalTakeupState {
    bool targetInitialized;
    bool active;
    bool responseStartCaptured;
    int8_t establishedTargetDirection;
    int8_t candidateTargetDirection;
    int8_t takeupTendonDirection;
    float lastAcceptedTargetDeg;
    float candidateAnchorDeg;
    float responseStartJointDeg;
    float appliedCounts;
};

/** 单腱目标速度前馈状态；仅由 10 ms MotionControlService 访问。 */
struct SingleTendonVelocityFeedforwardState {
    bool initialized;
    float previousTargetDeg;
    float filteredVelocityDegPerSec;
};

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
    float localTargets[ENCODER_TOTAL_NUM];
    float pidTargetsDeg[ENCODER_TOTAL_NUM];
    float magAngles[ENCODER_TOTAL_NUM];
    int32_t absolutePositionRaw[ENCODER_TOTAL_NUM];
    int32_t absolutePositionTendon[ENCODER_TOTAL_NUM];
    int32_t outPulsesTendon[ENCODER_TOTAL_NUM];
    // 单腱每周期有效 Kp 与 LUT 局部斜率，仅供控制和低频诊断使用。
    float singleTendonKp[ENCODER_TOTAL_NUM];
    float singleTendonLutSlope[ENCODER_TOTAL_NUM];
    SingleTendonDirectionalTakeupState
        singleTendonDirectionalTakeup[ENCODER_TOTAL_NUM];
    SingleTendonVelocityFeedforwardState
        singleTendonVelocityFf[ENCODER_TOTAL_NUM];
    float singleTendonVelocityLeadDeg[ENCODER_TOTAL_NUM];
    float singleTendonVelocityFfCounts[ENCODER_TOTAL_NUM];
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
    uint8_t directRunDoneStableCount;
    uint32_t directRunDoneGeneration;
    uint32_t feedbackRunToken;
    uint32_t feedbackRunStartMs;
    uint32_t feedbackRunInsideToleranceSinceMs;
    uint32_t feedbackRunJointMask;
    float feedbackRunTargetDeg[ENCODER_TOTAL_NUM];
    bool feedbackRunTracking;
    bool feedbackRunInsideTolerance;
    bool feedbackRunDoneEventQueued;
    bool feedbackRunTimeoutEventQueued;
};

static ControlServiceRuntime g_controlRuntime;

/** 清除一个关节的换向目标历史和连续补绳状态。 */
static void resetSingleTendonDirectionalTakeup(
    SingleTendonDirectionalTakeupState* state)
{
    if (state) memset(state, 0, sizeof(*state));
}

static void resetSingleTendonVelocityFeedforward(
    SingleTendonVelocityFeedforwardState* state)
{
    if (state) memset(state, 0, sizeof(*state));
}

/**
 * 计算由目标角速度带来的 LUT counts 提前量。PID 始终使用真实 qRef；
 * 预测角只用于同一 LUT 内的前馈差分，绝不将越界角度传给 LUT。
 */
static float computeSingleTendonVelocityFeedforward(
    SingleTendonVelocityFeedforwardState* state,
    uint8_t motorCh,
    float targetDeg,
    float pidDtSeconds,
    const JointCalibrationResult& calibration,
    float* outLeadDeg)
{
    if (outLeadDeg) *outLeadDeg = 0.0f;
    if (!state || !kSingleTendonVelocityFeedforwardEnabled ||
        !isfinite(targetDeg) || !isfinite(pidDtSeconds) ||
        pidDtSeconds <= 0.0f || calibration.tendonLutValid == 0 ||
        calibration.tendonLutCount < 3 ||
        calibration.tendonLutMotor[0] != motorCh ||
        !isfinite(calibration.currentMinDeg) ||
        !isfinite(calibration.currentMaxDeg) ||
        calibration.currentMaxDeg <= calibration.currentMinDeg) {
        if (state) resetSingleTendonVelocityFeedforward(state);
        return 0.0f;
    }

    if (!state->initialized) {
        state->initialized = true;
        state->previousTargetDeg = targetDeg;
        return 0.0f;
    }

    const float rawVelocityDegPerSec =
        (targetDeg - state->previousTargetDeg) / pidDtSeconds;
    state->previousTargetDeg = targetDeg;
    const float alpha = pidDtSeconds /
        (kSingleTendonTargetVelocityFilterTauSec + pidDtSeconds);
    state->filteredVelocityDegPerSec += alpha *
        (rawVelocityDegPerSec - state->filteredVelocityDegPerSec);
    if (!isfinite(state->filteredVelocityDegPerSec)) {
        resetSingleTendonVelocityFeedforward(state);
        return 0.0f;
    }

    const float referenceDeg = fmaxf(calibration.currentMinDeg,
                                     fminf(calibration.currentMaxDeg, targetDeg));
    const float requestedLeadDeg = fmaxf(
        -kSingleTendonVelocityLeadMaxDeg,
        fminf(kSingleTendonVelocityLeadMaxDeg,
              kSingleTendonVelocityLeadTimeSec *
                  state->filteredVelocityDegPerSec));
    const float predictedDeg = fmaxf(
        calibration.currentMinDeg,
        fminf(calibration.currentMaxDeg, referenceDeg + requestedLeadDeg));
    float referenceRawAbs = 0.0f;
    float predictedRawAbs = 0.0f;
    const int8_t shortenSign = getMotorTendonShortenAbsSign(motorCh);
    if (shortenSign == 0 ||
        !interpolateTendonCalibrationMotorAbs(
            calibration, 0, referenceDeg, &referenceRawAbs) ||
        !interpolateTendonCalibrationMotorAbs(
            calibration, 0, predictedDeg, &predictedRawAbs)) {
        resetSingleTendonVelocityFeedforward(state);
        return 0.0f;
    }

    const float counts = (float)shortenSign *
        (predictedRawAbs - referenceRawAbs);
    if (!isfinite(counts)) return 0.0f;
    if (outLeadDeg) *outLeadDeg = predictedDeg - referenceDeg;
    return counts;
}

/** 将角度增量转换为带迟滞的方向，极小的浮点抖动不参与换向判断。 */
static int8_t singleTendonTargetDirection(float deltaDeg)
{
    if (deltaDeg > 0.0001f) return 1;
    if (deltaDeg < -0.0001f) return -1;
    return 0;
}

/**
 * 只在收到新目标时累计反向行程；同一方向最多触发一次，直到确认下一次真实反向。
 * 实际补绳方向由 LUT 和电机腱绳坐标共同推导，不能按关节索引或 raw ABS 假定。
 */
static void updateSingleTendonDirectionalTakeupTarget(
    SingleTendonDirectionalTakeupState* state,
    bool newCommand,
    float targetDeg,
    uint8_t motorCh,
    const JointCalibrationResult& calibration)
{
    if (!state || !newCommand || !isfinite(targetDeg)) return;
    if (!state->targetInitialized) {
        state->targetInitialized = true;
        state->lastAcceptedTargetDeg = targetDeg;
        return;
    }

    const float previousTargetDeg = state->lastAcceptedTargetDeg;
    const int8_t direction =
        singleTendonTargetDirection(targetDeg - previousTargetDeg);
    state->lastAcceptedTargetDeg = targetDeg;
    if (direction == 0) return;

    if (state->active && direction != state->establishedTargetDirection) {
        state->active = false;
    }
    if (state->establishedTargetDirection == 0) {
        state->establishedTargetDirection = direction;
        state->candidateTargetDirection = 0;
        return;
    }
    if (direction == state->establishedTargetDirection) {
        state->candidateTargetDirection = 0;
        return;
    }

    if (state->candidateTargetDirection != direction) {
        state->candidateTargetDirection = direction;
        state->candidateAnchorDeg = previousTargetDeg;
    }
    const float reverseTravelDeg =
        (targetDeg - state->candidateAnchorDeg) * (float)direction;
    if (reverseTravelDeg < kSingleTendonDirectionalTakeupConfirmDeg) return;

    float anchorRawAbs = 0.0f;
    float targetRawAbs = 0.0f;
    const int8_t shortenSign = getMotorTendonShortenAbsSign(motorCh);
    const float tendonDelta =
        (shortenSign != 0 && calibration.tendonLutMotor[0] == motorCh &&
         interpolateTendonCalibrationMotorAbs(
             calibration, 0, state->candidateAnchorDeg, &anchorRawAbs) &&
         interpolateTendonCalibrationMotorAbs(
             calibration, 0, targetDeg, &targetRawAbs))
            ? (float)shortenSign * (targetRawAbs - anchorRawAbs)
            : 0.0f;
    const int8_t tendonDirection = singleTendonTargetDirection(tendonDelta);
    state->establishedTargetDirection = direction;
    state->candidateTargetDirection = 0;
    if (tendonDirection == 0) {
        state->active = false;
        return;
    }

    state->takeupTendonDirection = tendonDirection;
    state->responseStartCaptured = false;
    state->appliedCounts =
        (float)kSingleTendonDirectionalTakeupInitialCounts;
    state->active = true;
}

/**
 * 输出连续补绳 counts。首帧立即输出初始 counts，之后每控制周期按真实 dt 增加；
 * 只有关节沿新目标方向推进到退出距离才停止，不等待舵机分段到位。
 */
static float computeSingleTendonDirectionalTakeup(
    SingleTendonDirectionalTakeupState* state,
    float feedbackDeg,
    float pidDtSeconds)
{
    if (!state || !state->active ||
        state->takeupTendonDirection == 0 || !isfinite(feedbackDeg)) {
        return 0.0f;
    }

    if (!state->responseStartCaptured) {
        state->responseStartCaptured = true;
        state->responseStartJointDeg = feedbackDeg;
        return (float)state->takeupTendonDirection * state->appliedCounts;
    }

    const float jointTravel = (float)state->establishedTargetDirection *
        (feedbackDeg - state->responseStartJointDeg);
    if (jointTravel >= kSingleTendonDirectionalTakeupExitJointTravelDeg) {
        state->active = false;
        return 0.0f;
    }

    if (isfinite(pidDtSeconds) && pidDtSeconds > 0.0f) {
        state->appliedCounts = fminf(
            (float)kSingleTendonDirectionalTakeupMaxCounts,
            state->appliedCounts +
                kSingleTendonDirectionalTakeupRateCountsPerSec * pidDtSeconds);
    }
    return (float)state->takeupTendonDirection * state->appliedCounts;
}

/**
 * 根据单腱 LUT 将角度误差换算为本周期有效 Kp。
 * 近端采用动态 LUT 增益，大误差采用独立 LUT 增益；
 * 具体倍率由 ControlConfig 配置，LUT 不可信时才回退固定 Kp。
 */
static float computeSingleTendonEffectiveKp(
    uint8_t motorCh,
    float targetDeg,
    float feedbackDeg,
    const JointCalibrationResult& calibration,
    float* outLutSlope)
{
    if (outLutSlope) {
        *outLutSlope = 0.0f;
    }
    const float angleError = targetDeg - feedbackDeg;
    const float absError = fabsf(angleError);
    if (!isfinite(angleError) || calibration.tendonLutMotor[0] != motorCh) {
        return kSingleTendonKp;
    }
    // 连续目标跟随不能把完成阈值当作低增益死区；有效单腱 LUT 在
    // 0.8 度内仍使用固定 Kp，保证换向初段具备可观察的推进量。
    if (absError <= kFeedbackDoneToleranceDeg) {
        return kSingleTendonNearTargetKp;
    }

    float lutRefAbs = 0.0f;
    float lutFbAbs = 0.0f;
    if (!interpolateTendonCalibrationMotorAbs(
            calibration, 0, targetDeg, &lutRefAbs) ||
        !interpolateTendonCalibrationMotorAbs(
            calibration, 0, feedbackDeg, &lutFbAbs)) {
        return kSingleTendonKp;
    }

    const float lutDelta = lutRefAbs - lutFbAbs;
    const int8_t shortenSign = getMotorTendonShortenAbsSign(motorCh);
    const float lutSlope = fabsf(lutDelta / angleError);
    if (shortenSign == 0 || !isfinite(lutSlope)) {
        return kSingleTendonKp;
    }
    if (outLutSlope) {
        *outLutSlope = lutSlope;
    }
    const uint8_t lastIndex = calibration.tendonLutCount - 1U;
    const float lutAngleSpan =
        calibration.currentMaxDeg - calibration.currentMinDeg;
    const float lutAbsSpan = fabsf((float)
        (calibration.tendonLutMotorAbs[0][lastIndex] -
         calibration.tendonLutMotorAbs[0][0]));
    if (!isfinite(lutAngleSpan) || lutAngleSpan <= 0.0f ||
        !isfinite(lutAbsSpan) || lutAbsSpan <= 0.0f) {
        return kSingleTendonKp;
    }

    // 平均斜率负责大范围快速推进，局部斜率负责末端克服静摩擦；
    // 两者都来自本关节标定 LUT，因此不会把某个关节的增益套给其他关节。
    const float averageKp = kSingleTendonLargeErrorLutGain *
        (lutAbsSpan / lutAngleSpan);
    const bool tightening = (float)shortenSign * lutDelta > 0.0f;
    const float localGain = tightening
        ? kSingleTendonDynamicLutGain
        : kSingleTendonLargeErrorLutGain;
    const float localKp = localGain * lutSlope;
    if (!isfinite(averageKp) || averageKp <= 0.0f ||
        !isfinite(localKp) || localKp <= 0.0f) {
        return kSingleTendonKp;
    }
    if (absError >= kSingleTendonDynamicKpErrorMaxDeg) {
        return averageKp;
    }
    if (absError <= kSingleTendonDynamicKpBlendStartDeg) {
        return localKp;
    }

    const float blend =
        (absError - kSingleTendonDynamicKpBlendStartDeg) /
        (kSingleTendonDynamicKpErrorMaxDeg -
         kSingleTendonDynamicKpBlendStartDeg);
    return localKp + blend * (averageKp - localKp);
}

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

/** 仅为单腱 PID 调参日志在 active scope 内轮询选择关节。 */
static bool shouldLogControlPidDiag(uint8_t jointIndex,
                                    uint32_t nowMs,
                                    uint32_t activeJointMask)
{
    if constexpr (kPidDiagLogEnabled) {
        if (jointIndex >= ENCODER_TOTAL_NUM ||
            (uint32_t)(nowMs - g_controlPidDiagLastLogMs) <
                kPidDiagLogPeriodMs) {
            return false;
        }
        const uint32_t validMask =
            activeJointMask & ((1UL << ENCODER_TOTAL_NUM) - 1UL);
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
            // 选中的关节可能在保护分支中跳过 PID；两周期后允许实际运行的关节接管。
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

static uint8_t appendDualTendonTargets(ServoTargetBatch_t* batch,
                                       SystemRuntimeContext* sharedData,
                                       uint8_t jointIndex,
                                       float qRefDeg,
                                       float qFbDeg,
                                       float pidDtSeconds,
                                       const ServoAngleData_t& servoData,
                                       uint16_t speed,
                                       uint8_t acc,
                                       bool* outPidPushActive,
                                       ControlTraceData_t* trace)
{
    // 将逻辑关节目标转换成两个拮抗电机目标，并返回下一周期碰撞检测需要的
    // PID 推进证据；双腱 LUT 先按扩展边界钳位，再应用会话 logical range。
    if (outPidPushActive) *outPidPushActive = false;
    if (!batch || !sharedData) {
        return 0;
    }

    DualTendonMotorTargets tendonTargets;
    memset(&tendonTargets, 0, sizeof(tendonTargets));
    if (!computeDualTendonMotorTargets(jointIndex,
                                       qRefDeg,
                                       qFbDeg,
                                       pidDtSeconds,
                                       &servoData,
                                       &tendonTargets)) {
        resetDualTendonController(jointIndex);
        return 0;
    }

    JointActuatorRuntimeTarget runtimeTargets[MAX_JOINT_ACTUATORS];
    bool anyLutTargetClamped = false;
    uint8_t targetCount = 0;
    // 双腱两台电机始终反向运动，不能用其中一台的物理方向代表整个关节。
    // 双电机关节统一锁定为 RELEASE 档，即固定使用 400 ms / 0.4 度阈值。
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
        // 有效标定范围仍是负反馈准入条件；运行边界统一由 LUT 与会话 logical range 负责。
        if ((validBitmap & bit) == 0) {
            resetDualTendonController(jointIndex);
            return 0;
        }
        JointActuatorRuntimeTarget& target = runtimeTargets[targetCount++];
        target.motorChannel = motorChannel;
        target.sign = 1;
        target.targetAbs = clampServoPosToSessionRange(sharedData,
                                                       motorChannel,
                                                       tendonTargets.targetAbs[i]);
        if (tendonTargets.lutTargetClamped[i] != 0) {
            anyLutTargetClamped = true;
            noteTendonLutClamp(sharedData,
                               jointIndex,
                               motorChannel,
                               tendonTargets.rawTargetAbs[i],
                               tendonTargets.targetAbs[i]);
        }
        if (outPidPushActive &&
            tendonTargets.lutTargetClamped[i] == 0 &&
            fabsf(tendonTargets.pidCorrection[i]) >= 1.0f &&
            tendonTargets.pidCorrection[i] * tendonTargets.motorErr[i] > 0.0f) {
            *outPidPushActive = true;
        }
    }
    // 恢复不越过任一侧 LUT/会话边界；触界时整对取消，保留原 PID 输出。
    int32_t recovered[2] = {};
    bool recoveryAllowed = targetCount == 2 && !anyLutTargetClamped;
    for (uint8_t i = 0; i < targetCount && i < 2; ++i) {
        const auto& target = runtimeTargets[i];
        recovered[i] = target.targetAbs +
            static_cast<int32_t>(lroundf(tendonTargets.recoveryCounts[i]));
        if (target.targetAbs != tendonTargets.targetAbs[i] ||
            recovered[i] < tendonTargets.lutMinAbs[i] ||
            recovered[i] > tendonTargets.lutMaxAbs[i] ||
            clampServoPosToSessionRange(sharedData, target.motorChannel, recovered[i]) != recovered[i])
            recoveryAllowed = false;
    }
    if (recoveryAllowed) {
        for (uint8_t i = 0; i < 2; ++i)
            runtimeTargets[i].targetAbs = recovered[i];
    }
    if (trace) {
        for (uint8_t i = 0; i < targetCount; ++i) {
            const uint8_t motor = runtimeTargets[i].motorChannel;
            trace->pidCounts[motor] = static_cast<int16_t>(lroundf(
                fmaxf(-32768.0f, fminf(32767.0f, tendonTargets.pidCorrection[i]))));
            if (recoveryAllowed) {
                trace->recoveryAppliedMask |= 1UL << motor;
                trace->recoveryCounts[motor] = static_cast<int16_t>(
                    recovered[i] - tendonTargets.targetAbs[i]);
            }
        }
    }
    if (outPidPushActive && *outPidPushActive) {
        *outPidPushActive = false;
        for (uint8_t i = 0; i < targetCount; ++i) {
            const float applied = runtimeTargets[i].targetAbs -
                servoData.servoAngles[runtimeTargets[i].motorChannel];
            if (applied * tendonTargets.motorErr[i] > 0.0f &&
                tendonTargets.pidCorrection[i] * tendonTargets.motorErr[i] > 0.0f)
                *outPidPushActive = true;
        }
    }
    if (outPidPushActive && anyLutTargetClamped) {
        // 实际下发目标已无法继续沿 PID 方向推进，不能把 LUT 边界误判成外物碰撞。
        *outPidPushActive = false;
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

    // 使用引用保留原控制公式的可读命名，同时把所有历史量集中到运行上下文。
    ControlServiceRuntime& runtime = g_controlRuntime;
    auto& localTargets = runtime.localTargets;
    auto& pidTargetsDeg = runtime.pidTargetsDeg;
    auto& magAngles = runtime.magAngles;
    auto& absolutePositionRaw = runtime.absolutePositionRaw;
    auto& absolutePositionTendon = runtime.absolutePositionTendon;
    auto& outPulsesTendon = runtime.outPulsesTendon;
    auto& singleTendonKp = runtime.singleTendonKp;
    auto& singleTendonLutSlope = runtime.singleTendonLutSlope;
    auto& singleTendonDirectionalTakeup =
        runtime.singleTendonDirectionalTakeup;
    auto& singleTendonVelocityFf = runtime.singleTendonVelocityFf;
    auto& singleTendonVelocityLeadDeg = runtime.singleTendonVelocityLeadDeg;
    auto& singleTendonVelocityFfCounts = runtime.singleTendonVelocityFfCounts;
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
    auto& directRunDoneStableCount = runtime.directRunDoneStableCount;
    auto& feedbackRunToken = runtime.feedbackRunToken;
    auto& feedbackRunStartMs = runtime.feedbackRunStartMs;
    auto& feedbackRunInsideToleranceSinceMs =
        runtime.feedbackRunInsideToleranceSinceMs;
    auto& feedbackRunJointMask = runtime.feedbackRunJointMask;
    auto& feedbackRunTargetDeg = runtime.feedbackRunTargetDeg;
    auto& feedbackRunTracking = runtime.feedbackRunTracking;
    auto& feedbackRunInsideTolerance = runtime.feedbackRunInsideTolerance;
    auto& feedbackRunDoneEventQueued = runtime.feedbackRunDoneEventQueued;
    auto& feedbackRunTimeoutEventQueued = runtime.feedbackRunTimeoutEventQueued;

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
            // 新关节目标会开启新的单次反馈完成判定窗口。
            memset(hotplugHoldMotor, 0, sizeof(hotplugHoldMotor));
            feedbackRunInsideTolerance = false;
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
        // 只在实际反馈输出期间累计换向目标；无效关节和非反馈模式立即丢弃历史，
        // 防止暂停、标定或直控后的旧方向重新触发补绳。
        for (uint8_t jointIndex = 0;
             jointIndex < ENCODER_TOTAL_NUM;
             ++jointIndex) {
            const uint32_t bit = 1UL << jointIndex;
            const int motorCh = findPrimaryMotorChannelForJoint(jointIndex);
            const JointCalibrationResult& calibration =
                calibrationSnapshot[jointIndex].result;
            const bool takeupEligible =
                kSingleTendonDirectionalTakeupEnabled &&
                feedbackOutputState &&
                (feedbackCommandJointMask & bit) != 0 &&
                (calibrationSnapshotValidMask & bit) != 0 &&
                mappedData.validFlags[jointIndex] != 0 &&
                !isDualTendonJoint(jointIndex) &&
                motorCh >= 0 && motorCh < SERVO_TOTAL_NUM &&
                servoData.onlineStatus[motorCh] != 0 &&
                calibration.tendonLutValid != 0 &&
                calibration.tendonLutCount >= 3 &&
                calibration.tendonLutMotor[0] == (uint8_t)motorCh;
            if (!takeupEligible) {
                resetSingleTendonDirectionalTakeup(
                    &singleTendonDirectionalTakeup[jointIndex]);
                continue;
            }
            updateSingleTendonDirectionalTakeupTarget(
                &singleTendonDirectionalTakeup[jointIndex],
                newJointCommand,
                localTargets[jointIndex],
                (uint8_t)motorCh,
                calibration);
        }
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

        // 碰撞检测只消费磁编新帧。200 ms 速度窗与按物理方向选择的进展窗同步滚动，
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
                    "mode=%s v200=%.3fdeg/s source=velocity candidate_ms=%lu\r\n",
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
                    "mode=%s v200=%.3fdeg/s progress%lu=%.3fdeg min=%.3f "
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
                feedbackCollisionDriveMotionMode[jointIndex] =
                    FEEDBACK_COLLISION_MOTION_RELEASE;
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
                feedbackRunInsideToleranceSinceMs = 0;
                feedbackRunInsideTolerance = false;
                feedbackRunTracking = true;
                feedbackRunDoneEventQueued = false;
                feedbackRunTimeoutEventQueued = false;
            }
        } else {
            feedbackRunTracking = false;
            feedbackRunInsideTolerance = false;
            feedbackRunDoneEventQueued = false;
            feedbackRunTimeoutEventQueued = false;
        }

        // 只有 CONTROL owner 可以构建舵机目标。标定期间，上面的角度映射、
        // 遥测和调试发布仍会继续，但不能与标定生产者竞争单槽 servoTargetQueue。
        const bool controlTargetBuildAllowed = controlEnabled;
        if (!controlTargetBuildAllowed || !feedbackOutputState) {
            // 双腱 PID 含历史项；只要反馈输出没有实际驱动关节，就清空这些状态。
            for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++) {
                resetSingleTendonVelocityFeedforward(
                    &singleTendonVelocityFf[jointIndex]);
                resetSingleTendonDirectionalTakeup(
                    &singleTendonDirectionalTakeup[jointIndex]);
                singleTendonVelocityLeadDeg[jointIndex] = 0.0f;
                singleTendonVelocityFfCounts[jointIndex] = 0.0f;
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
            runtime.cycleTrace.dtMicros = static_cast<uint32_t>(lroundf(pidDtSeconds * 1000000.0f));
            // 默认保持固定 Kp；有效单腱关节按各自 LUT 计算双向增益，
            // 双腱关节继续使用独立的双电机 PID，不进入该分支。
            for (uint8_t jointIndex = 0;
                 jointIndex < ENCODER_TOTAL_NUM;
                 ++jointIndex) {
                singleTendonKp[jointIndex] = kSingleTendonKp;
                singleTendonLutSlope[jointIndex] = 0.0f;
                pidTargetsDeg[jointIndex] = localTargets[jointIndex];
                singleTendonVelocityLeadDeg[jointIndex] = 0.0f;
                singleTendonVelocityFfCounts[jointIndex] = 0.0f;
                const uint32_t bit = 1UL << jointIndex;
                if ((feedbackCommandJointMask & bit) == 0 ||
                    (calibrationSnapshotValidMask & bit) == 0 ||
                    mappedData.validFlags[jointIndex] == 0 ||
                    isDualTendonJoint(jointIndex) ||
                    g_feedbackCollisionGuard.isHoldActive(jointIndex)) {
                    resetSingleTendonVelocityFeedforward(
                        &singleTendonVelocityFf[jointIndex]);
                    resetSingleTendonDirectionalTakeup(
                        &singleTendonDirectionalTakeup[jointIndex]);
                    continue;
                }
                const int motorCh = findPrimaryMotorChannelForJoint(jointIndex);
                if (motorCh < 0 || motorCh >= SERVO_TOTAL_NUM ||
                    servoData.onlineStatus[motorCh] == 0) {
                    resetSingleTendonDirectionalTakeup(
                        &singleTendonDirectionalTakeup[jointIndex]);
                    resetSingleTendonVelocityFeedforward(
                        &singleTendonVelocityFf[jointIndex]);
                    continue;
                }
                singleTendonKp[jointIndex] = computeSingleTendonEffectiveKp(
                    (uint8_t)motorCh,
                    localTargets[jointIndex],
                    magAngles[jointIndex],
                    calibrationSnapshot[jointIndex].result,
                    &singleTendonLutSlope[jointIndex]);
                singleTendonVelocityFfCounts[jointIndex] =
                    computeSingleTendonVelocityFeedforward(
                        &singleTendonVelocityFf[jointIndex],
                        (uint8_t)motorCh,
                        localTargets[jointIndex],
                        pidDtSeconds,
                        calibrationSnapshot[jointIndex].result,
                        &singleTendonVelocityLeadDeg[jointIndex]);
            }
            // PID 使用腱绳空间 ABS，使正修正在 raw ABS 方向相反的电机上
            // 仍表示相同的“收紧腱绳”含义。单电机输出会把最终腱绳空间目标转回 raw。
            g_controlSolver.compute(pidTargetsDeg,
                                    magAngles,
                                    absolutePositionTendon,
                                    outPulsesTendon,
                                    singleTendonKp,
                                    pidDtSeconds);
            for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++) {
                if (g_feedbackCollisionGuard.isHoldActive(jointIndex)) {
                    g_controlSolver.resetJoint(jointIndex);
                    resetSingleTendonVelocityFeedforward(
                        &singleTendonVelocityFf[jointIndex]);
                    resetSingleTendonDirectionalTakeup(
                        &singleTendonDirectionalTakeup[jointIndex]);
                    singleTendonVelocityLeadDeg[jointIndex] = 0.0f;
                    singleTendonVelocityFfCounts[jointIndex] = 0.0f;
                } else if (!isDualTendonJoint(jointIndex) &&
                           singleTendonVelocityFfCounts[jointIndex] != 0.0f) {
                    outPulsesTendon[jointIndex] += static_cast<int32_t>(
                        lroundf(singleTendonVelocityFfCounts[jointIndex]));
                }
                if (!isDualTendonJoint(jointIndex)) {
                    outPulsesTendon[jointIndex] += static_cast<int32_t>(
                        lroundf(computeSingleTendonDirectionalTakeup(
                            &singleTendonDirectionalTakeup[jointIndex],
                            magAngles[jointIndex],
                            pidDtSeconds)));
                }
            }
            if (commandSnapshot.controlMode == CONTROL_MODE_DIRECT_MOTOR)
            {
                // 直控模式绕过关节 PID，将上位机电机目标直接发给对应物理通道。
                const uint32_t activeMotorMask =
                    commandSnapshot.directMotorMask & validMotorChannelMask();
                for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++)
                {
                    if (((activeMotorMask >> ch) & 0x1UL) == 0) {
                        continue;
                    }
                    int32_t targetPos = servoData.servoAngles[ch];
                    if (!hotplugHoldMotor[ch]) {
                        targetPos = commandSnapshot.motorTargetRaw[ch];
                    }
                    appendServoTarget(&targetBatch, ch, targetPos,
                                      commandSnapshot.directSpeed, commandSnapshot.directAcc);
                }
            }
            else
            {
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
                feedbackCollisionDriveMotionMode[jointIndex] =
                    FEEDBACK_COLLISION_MOTION_RELEASE;

                if (!jointInScope)
                {
                    // 未激活关节不发布主动命令，并丢弃双腱状态，便于重新启用时干净启动。
                    resetSingleTendonVelocityFeedforward(
                        &singleTendonVelocityFf[jointIndex]);
                    resetSingleTendonDirectionalTakeup(
                        &singleTendonDirectionalTakeup[jointIndex]);
                    if (isDualTendonJoint(jointIndex)) {
                        resetDualTendonController(jointIndex);
                    }
                    continue;
                }

                if (emergency)
                {
                    // 反馈故障、总线丢失、输出关闭和过载都会统一进入保持当前位置行为。
                    resetSingleTendonVelocityFeedforward(
                        &singleTendonVelocityFf[jointIndex]);
                    resetSingleTendonDirectionalTakeup(
                        &singleTendonDirectionalTakeup[jointIndex]);
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
                    resetSingleTendonVelocityFeedforward(
                        &singleTendonVelocityFf[jointIndex]);
                    resetSingleTendonDirectionalTakeup(
                        &singleTendonDirectionalTakeup[jointIndex]);
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
                    resetSingleTendonVelocityFeedforward(
                        &singleTendonVelocityFf[jointIndex]);
                    resetSingleTendonDirectionalTakeup(
                        &singleTendonDirectionalTakeup[jointIndex]);
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
                    resetSingleTendonVelocityFeedforward(
                        &singleTendonVelocityFf[jointIndex]);
                    resetSingleTendonDirectionalTakeup(
                        &singleTendonDirectionalTakeup[jointIndex]);
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
                                                jointIndex,
                                                localTargets[jointIndex],
                                                magAngles[jointIndex],
                                                pidDtSeconds,
                                                servoData,
                                                kFeedbackServoSpeed,
                                                SERVO_TARGET_ACC_DEFAULT,
                                                &feedbackCollisionPidPushActive[jointIndex],
                                                &runtime.cycleTrace);
                    if (appended == 0) {
                        appendJointActuatorHoldTargets(&targetBatch, jointIndex, servoData,
                                                       SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
                    }
                    continue;
                }

                if (jointActive) {
                    // 普通单电机关节：把腱绳空间 PID 结果转回电机 raw ABS。
                    // 松端沿用 LUT 裕量；紧端必须使用标定实测机械端点，不能使用
                    // 带迟滞的松绳回程 LUT 在 URDF Max 处的样本。
                    int32_t actualMotorAbs = targetPos;
                    int32_t rawPidTargetAbs = targetPos;
                    // LUT 无效或目标被钳位时，禁止把“目标无法继续推进”误判成碰撞。
                    bool singleLutBlocksPidPush = true;
                    if (jointMotorCh >= 0 && jointMotorCh < SERVO_TOTAL_NUM) {
                        const uint8_t motorCh = (uint8_t)jointMotorCh;
                        actualMotorAbs = servoData.servoAngles[motorCh];
                        rawPidTargetAbs =
                            motorTendonAbsToRawAbs(motorCh, outPulsesTendon[jointIndex]);
                        const JointCalibrationResult& calibration =
                            calibrationSnapshot[jointIndex].result;
                        int32_t lutMinAbs = 0;
                        int32_t lutMaxAbs = 0;
                        const bool lutMatchesMotor =
                            calibration.tendonLutMotor[0] == motorCh &&
                            singleTendonCalibrationMotorAbsRange(
                                calibration,
                                0,
                                motorCh,
                                kTendonLutReleaseMarginCounts,
                                kTendonLutTightMarginCounts,
                                &lutMinAbs,
                                &lutMaxAbs);
                        if (!lutMatchesMotor) {
                            // LUT 缺失或属于其他电机时保持当前位置，禁止无边界闭环。
                            targetPos = actualMotorAbs;
                        } else {
                            const p4::safety::AbsBoundaryClampDecision lutDecision =
                                p4::safety::clampAbsToBoundary(rawPidTargetAbs,
                                                               lutMinAbs,
                                                               lutMaxAbs);
                            if (!lutDecision.valid) {
                                targetPos = actualMotorAbs;
                            } else {
                                targetPos = lutDecision.applied;
                                singleLutBlocksPidPush = lutDecision.clamped;
                                if (lutDecision.clamped) {
                                    noteTendonLutClamp(sharedData,
                                                       jointIndex,
                                                       motorCh,
                                                       rawPidTargetAbs,
                                                       targetPos);
                                }
                            }
                        }
                        targetPos = clampServoPosToSessionRange(sharedData,
                                                                motorCh,
                                                                targetPos);
                    } else {
                        targetPos = outPulsesTendon[jointIndex];
                    }
                    const float pidOutput =
                        g_controlSolver.getPidOutput(jointIndex);
                    const float velocityFfCounts =
                        singleTendonVelocityFfCounts[jointIndex];
                    const SingleTendonDirectionalTakeupState& takeup =
                        singleTendonDirectionalTakeup[jointIndex];
                    const float takeupCounts = takeup.active
                        ? (float)takeup.takeupTendonDirection * takeup.appliedCounts
                        : 0.0f;
                    const float finalOutput =
                        pidOutput + velocityFfCounts + takeupCounts;
                    const float angleError =
                        localTargets[jointIndex] - magAngles[jointIndex];
                    feedbackCollisionPidPushActive[jointIndex] =
                        !singleLutBlocksPidPush &&
                        ((fabsf(pidOutput + velocityFfCounts) >= 1.0f &&
                          (pidOutput + velocityFfCounts) * angleError > 0.0f) ||
                         takeupCounts != 0.0f);
                    // 单电机直接使用本周期原始 PID 目标与实际 ABS 的差值。
                    // ABS 增加统一代表收绳；相等时保留本轮默认放绳，且不会触发
                    // HOLD（pidPushActive 通常同时为 false）。
                    if (rawPidTargetAbs > actualMotorAbs) {
                        feedbackCollisionDriveMotionMode[jointIndex] =
                            FEEDBACK_COLLISION_MOTION_TIGHTEN;
                    } else if (rawPidTargetAbs < actualMotorAbs) {
                        feedbackCollisionDriveMotionMode[jointIndex] =
                            FEEDBACK_COLLISION_MOTION_RELEASE;
                    }
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
                        float lutRefAbs = 0.0f;
                        float lutFbAbs = 0.0f;
                        bool lutValid = false;
                        if (jointMotorCh >= 0 && jointMotorCh < SERVO_TOTAL_NUM) {
                            const JointCalibrationResult& calibration =
                                calibrationSnapshot[jointIndex].result;
                            // 单环 PID 仍以角度误差为输入；同一 LUT 插值同时用于
                            // 近目标动态 Kp 和低频诊断，模型位置本身不直接作为目标。
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
                            "kp=%.2f slope=%.2f qDot=%.2f qLead=%.3f ff=%.2f "
                            "outPid=%.2f outFinal=%.2f abs=%ld cmd=%d lead=%ld "
                            "load=%d mag=%d speed=%d "
                            "telem=%u lutValid=%u lutRef=%.2f lutFb=%.2f "
                            "lutDelta=%.2f rawTarget=%ld tu=%u tuDir=%d tuTravel=%.3f tuCounts=%.1f\r\n",
                            (unsigned long)nowMs,
                            (unsigned)jointIndex,
                            jointMotorCh,
                            (double)localTargets[jointIndex],
                            (double)magAngles[jointIndex],
                            (double)(localTargets[jointIndex] - magAngles[jointIndex]),
                            (double)singleTendonKp[jointIndex],
                            (double)singleTendonLutSlope[jointIndex],
                            (double)singleTendonVelocityFf[jointIndex]
                                .filteredVelocityDegPerSec,
                            (double)singleTendonVelocityLeadDeg[jointIndex],
                            (double)velocityFfCounts,
                            (double)pidOutput,
                            (double)finalOutput,
                            (long)currentAbs,
                            (int)targetPos,
                            (long)targetLead,
                            (int)rawLoad,
                            (int)loadMag,
                            (int)servoSpeed,
                            (unsigned)telemetryFresh,
                            lutValid ? 1U : 0U,
                            (double)lutRefAbs,
                            (double)lutFbAbs,
                            (double)(lutRefAbs - lutFbAbs),
                            (long)rawPidTargetAbs,
                            takeup.active ? 1U : 0U,
                            (int)takeup.takeupTendonDirection,
                            takeup.responseStartCaptured
                                ? (double)takeup.establishedTargetDirection *
                                    (double)(magAngles[jointIndex] -
                                        takeup.responseStartJointDeg)
                                : 0.0,
                            (double)takeupCounts);
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

        // Target、Actual 与成功发布的 Servo CMD 在同一个控制周期内封装。
        // 只有 Gateway 接受的 batch 才记入 CMD，禁止把未执行的候选值画成指令。
        if (commandSnapshot.controlMode == CONTROL_MODE_JOINT &&
            sharedAtomicLoad(&sharedData->controlTraceRecordingEnabled)) {
            ControlTraceData_t& trace = runtime.cycleTrace;
            trace.sequence = ++runtime.controlTraceSequence;
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

        if (runtime.directRunDoneGeneration != commandSnapshot.motorDirectCommandGeneration) {
            runtime.directRunDoneGeneration = commandSnapshot.motorDirectCommandGeneration;
            directRunDoneStableCount = 0;
        }
        const bool directBatchExecuted =
            sharedAtomicLoad(&sharedData->direct_target_result_generation) ==
                commandSnapshot.motorDirectCommandGeneration &&
            sharedAtomicLoad(&sharedData->direct_target_status) == DIRECT_TARGET_APPLY_APPLIED;
        if (directRunningState && directBatchExecuted &&
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
                if (!feedbackRunInsideTolerance) {
                    feedbackRunInsideTolerance = true;
                    feedbackRunInsideToleranceSinceMs = nowMs;
                }
                if ((uint32_t)(nowMs - feedbackRunInsideToleranceSinceMs) >=
                        kFeedbackDoneStableMs &&
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
                feedbackRunInsideTolerance = false;
            }

            const uint32_t elapsedMs = (uint32_t)(nowMs - feedbackRunStartMs);
            const bool collisionHoldActive =
                sharedAtomicLoad(&sharedData->feedback_collision_hold_bitmap) != 0;
            // 碰撞保持需要等待反向目标主动解除，不能被普通 8 s 超时抢先释放。
            if (!targetsReached && !collisionHoldActive &&
                !feedbackRunDoneEventQueued &&
                elapsedMs >= kFeedbackRunTimeoutMs &&
                !feedbackRunTimeoutEventQueued) {
                feedbackRunTimeoutEventQueued = servicePostEvent(
                    output,
                    SYSTEM_EVENT_RUN_FEEDBACK_TIMEOUT,
                    feedbackRunToken,
                    true);
            }
        }

    return true;
}
