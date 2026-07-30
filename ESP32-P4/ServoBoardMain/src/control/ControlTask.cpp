#include "ControlTask.h"
#include "../actuator/ActuatorCommandGateway.h"

#include <math.h>
#include <string.h>

#include "../calibration/CalibrationMath.h"
#include "../calibration/CalibrationTask.h"
#include "../hardware/HardwareMap.h"
#include "../generated/UrdfJointLimits.generated.h"
#include "../shared/SharedSafetyState.h"
#include "../system/FirmwareLog.h"
#include "../system/StateMachineTask.h"
#include "../system/TaskRuntimeMonitor.h"
#include "ControlOutputBuilder.h"
#include "ControlSafety.h"
#include "ControlSolver.h"
#include "DualTendonControl.h"
#include "FeedbackCollisionGuard.h"
#include "TendonCalibrationLut.h"

static ControlSolver g_controlSolver;
// 碰撞检测历史放在静态区，避免挤占 10 ms 控制任务栈。
static FeedbackCollisionGuard g_feedbackCollisionGuard;

// 磁编码器数值是 14 bit 环形计数。
static const int32_t kEncoderModulo = 16384;
// 标定行程溢出判定为回绕前，允许少量边界余量。
static const int32_t kEncoderMarginCounts = (kEncoderModulo * 10 + 180) / 360;
// 每个周期只发布少量关节调试数据，避免队列流量过大。
static const uint8_t kDebugJointIndices[] = {0, 1, 2, 3};
static const uint8_t kDebugJointCount =
    (uint8_t)(sizeof(kDebugJointIndices) / sizeof(kDebugJointIndices[0]));

#define CONTROL_PID_DIAG_LOG_ENABLE 1
static const uint32_t kControlPidDiagLogPeriodMs = 50;
static uint32_t g_controlPidDiagLastLogMs = 0;
static uint8_t g_controlPidDiagNextJoint = 0;

static const uint32_t kCanBusOfflineTimeoutMs = 1000;
static const uint32_t kServoTelemetryTimeoutMs = 500;
// 磁编码器断开时传感器协议使用的标记值。
static const uint16_t kEncoderDisconnectRaw = 0xFFFF;
// 直控模式对负载尖峰更敏感；关节 PID 模式允许更高负载。
static const int16_t kControlOverloadLoadThreshold = 800;
static const int16_t kJointControlOverloadLoadThreshold = 2000;
// 800/1000 counts 仅用于独立硬过载故障；碰撞保持不再使用负载值判定。
// 只有连续多个高负载帧后才锁存硬故障。
static const uint8_t kControlOverloadLatchCycles = 20;
static const uint8_t kJointControlOverloadLatchCycles = 20;
// 速度 0 表示让舵机固件使用适合反馈控制的默认速度。
static const uint16_t kJointControlFeedbackSpeed = 0;
// 完成判定同时要求位置误差满足容差，并持续稳定若干帧。
static const int32_t kDirectRunDoneAbsToleranceCounts = 40;
static const uint8_t kDirectRunDoneStableCycles = 5;
static const float kFeedbackRunDoneToleranceDeg = 0.5f;
static const uint8_t kFeedbackRunDoneStableCycles = 10;
static const uint32_t kFeedbackRunTimeoutMs = 8000;
static const int32_t kUpperWorkspaceWarningCounts = 48;
static const uint8_t kUpperWorkspaceWarningStableFrames = 3;
// 碰撞检测只在实际舵机进入 LUT 边界附近时排除“工作空间耗尽”。
// 不能使用 PID 计算目标被钳位作为证据，否则双电机大误差时会大量漏检。
static const int32_t kCollisionLutBoundaryToleranceCounts = 32;

static bool shouldLogControlPidDiag(uint8_t jointIndex,
                                    uint32_t nowMs,
                                    uint32_t activeJointMask)
{
#if CONTROL_PID_DIAG_LOG_ENABLE
    if (jointIndex >= ENCODER_TOTAL_NUM) {
        return false;
    }
    // 全手控制仍只允许每 50 ms 输出一条 PID 日志，但在 active joint 之间
    // 轮询。旧实现总由循环首位 J00 抢占全局时间槽，导致日志长期只有 M00/M01。
    if ((uint32_t)(nowMs - g_controlPidDiagLastLogMs) <
        kControlPidDiagLogPeriodMs) {
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
            (kControlPidDiagLogPeriodMs * 2U)) {
            return false;
        }
    }
    g_controlPidDiagLastLogMs = nowMs;
    g_controlPidDiagNextJoint = nextJointAfterLog;
    return true;
#else
    (void)jointIndex;
    (void)nowMs;
    (void)activeJointMask;
    return false;
#endif
}

// 判断舵机实际 ABS 是否已抵达当前推进方向的 LUT 边界。
// pushDeltaAbs 只表示正负方向，是否抵边始终由 actualAbs 与 LUT Min/Max 决定。
static bool isActualAbsAtDirectionalLutBoundary(int32_t actualAbs,
                                                int32_t pushDeltaAbs,
                                                int32_t lutMinAbs,
                                                int32_t lutMaxAbs)
{
    if (lutMinAbs > lutMaxAbs) {
        return true;
    }
    if (pushDeltaAbs >= 1) {
        return actualAbs >=
            (lutMaxAbs - kCollisionLutBoundaryToleranceCounts);
    }
    if (pushDeltaAbs <= -1) {
        return actualAbs <=
            (lutMinAbs + kCollisionLutBoundaryToleranceCounts);
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

static int32_t orientEncoderRaw(uint16_t rawValue, int8_t direction)
{
    // 应用标定映射前，先统一磁编码器方向。
    int32_t oriented = (int32_t)rawValue & (kEncoderModulo - 1);
    if (direction < 0) {
        oriented = (kEncoderModulo - oriented) & (kEncoderModulo - 1);
    }
    return oriented;
}

static int32_t wrapEncoderCount(int32_t value)
{
    // 回绕到 0..16383 编码器计数范围。
    value %= kEncoderModulo;
    if (value < 0) value += kEncoderModulo;
    return value;
}

static int32_t forwardEncoderDelta(int32_t from, int32_t to)
{
    // 从一个 raw 编码器计数到另一个计数的正向环形距离。
    return wrapEncoderCount(to - from);
}

// 编码器计数转角度。
static float convertEncoderCountToDeg(int32_t encoderCount)
{
    return (float)encoderCount * 360.0f / (float)kEncoderModulo;
}

static int32_t degToEncoderCount(float deg)
{
    // 按远离 0 的方向取整，使小的正负角度映射保持对称。
    const float scaled = deg * (float)kEncoderModulo / 360.0f;
    return (int32_t)(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
}

static float mapCalibratedEncoderToDeg(const JointCalibrationResult& calib, int32_t orientedRaw)
{
    // 新版缩放标定使用 startEncoder/encoderSpan/totalDeg 表示实测行程；
    // 旧版标定回退到 offset 和硬限位最大值。
    if (calib.scaled && calib.encoderSpan > 0 && calib.totalDeg > 0.0f) {
        int32_t delta = forwardEncoderDelta(calib.startEncoder, orientedRaw);
        if (delta > calib.encoderSpan + kEncoderMarginCounts) {
            delta -= kEncoderModulo;
        }
        return ((float)delta / (float)calib.encoderSpan) * calib.totalDeg + calib.startDeg;
    }

    const int32_t offset = calib.offset;
    int32_t delta = forwardEncoderDelta(offset, orientedRaw);
    int32_t hardMaxFromOffset = forwardEncoderDelta(offset, calib.encoderMax);
    if (hardMaxFromOffset <= 0) {
        hardMaxFromOffset = degToEncoderCount(calib.angleMax);
    }
    if (delta > hardMaxFromOffset + kEncoderMarginCounts) {
        delta -= kEncoderModulo;
    }
    return convertEncoderCountToDeg(delta);
}

static float clampJointTargetDegByCalib(float targetDeg, uint8_t jointIndex)
{
    // 在 PID 接收目标前，先把上位机目标限制到可用标定关节范围。
    if (!isfinite(targetDeg)) {
        targetDeg = 0.0f;
    }
    if (jointIndex >= ENCODER_TOTAL_NUM) {
        return 0.0f;
    }

    float minDeg = 0.0f;
    float maxDeg = 0.0f;
    if (g_jointCalibResult[jointIndex].success) {
        const JointCalibrationResult& calibration = g_jointCalibResult[jointIndex];
        if (calibration.mappingVersion >= 2) {
            minDeg = calibration.angleMin;
        }
        maxDeg = calibration.angleMax;
    } else {
        // 未生成 Auto 结果时仅保留 URDF 静态范围作为输入校验；状态机仍会
        // 阻止该关节进入负反馈，不能把静态范围误认为有效标定。
        minDeg = kUrdfJointLimits[jointIndex].minDeg;
        maxDeg = kUrdfJointLimits[jointIndex].maxDeg;
    }

    if (!isfinite(maxDeg) || maxDeg <= 0.0f) {
        return 0.0f;
    }
    if (!isfinite(minDeg) || minDeg >= maxDeg) {
        return 0.0f;
    }
    if (targetDeg < minDeg) {
        return minDeg;
    }
    if (targetDeg > maxDeg) {
        return maxDeg;
    }
    return targetDeg;
}

// 单电机关节的角度 PID 会持续在“当前 ABS + 修正量”上推进，因此仅限制
// 关节角目标还不够。这里使用自动标定 LUT 的实测 ABS 包络做最终硬钳位，
// 防止弹簧受阻、静差或累计命令把舵机推出已经验证过的机械工作空间。
static bool clampSingleTendonTargetByLut(uint8_t jointIndex,
                                         uint8_t motorChannel,
                                         int32_t targetRawAbs,
                                         int32_t* outTargetRawAbs,
                                         int32_t* outLutMinAbs,
                                         int32_t* outLutMaxAbs)
{
    if (!outTargetRawAbs || !outLutMinAbs || !outLutMaxAbs ||
        jointIndex >= ENCODER_TOTAL_NUM ||
        motorChannel >= SERVO_TOTAL_NUM) {
        return false;
    }

    const JointCalibrationResult& calibration = g_jointCalibResult[jointIndex];
    if (calibration.tendonLutMotor[0] != motorChannel ||
        !tendonCalibrationMotorAbsRange(calibration,
                                        0,
                                        outLutMinAbs,
                                        outLutMaxAbs)) {
        return false;
    }

    int32_t clamped = targetRawAbs;
    if (clamped < *outLutMinAbs) clamped = *outLutMinAbs;
    if (clamped > *outLutMaxAbs) clamped = *outLutMaxAbs;
    *outTargetRawAbs = clampServoPos(clamped);
    return true;
}

// 根据物理 bus/id 查找 motorMap 通道。
static int findMotorChannel(uint8_t bus, uint8_t id)
{
    return findMotorChannelByBusId(bus, id);
}

static int16_t servoLoadMagnitude(int16_t rawLoad);

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
        const MotorMapItem& motor = motorMap[actuator.motorChannel];
        JointActuatorRuntimeTarget& target = targets[targetCount++];
        target.motorChannel = actuator.motorChannel;
        target.busIndex = motor.busIndex;
        target.servoId = motor.servoID;
        target.sign = actuator.sign;
        target.targetAbs = clampServoPos(servoData.servoAngles[actuator.motorChannel]);
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
        const MotorMapItem& motor = motorMap[actuator.motorChannel];
        JointActuatorRuntimeTarget& target = targets[targetCount++];
        target.motorChannel = actuator.motorChannel;
        target.busIndex = motor.busIndex;
        target.servoId = motor.servoID;
        target.sign = actuator.sign;
        target.targetAbs = clampServoPos(holdAbs[i]);
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
static void publishFeedbackCollisionState(TaskSharedData_t* sharedData,
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
                                       TaskSharedData_t* sharedData,
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
    uint8_t targetCount = 0;
    const uint32_t diagNowMs = millis();
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
        const MotorMapItem& motor = motorMap[motorChannel];
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
        target.busIndex = motor.busIndex;
        target.servoId = motor.servoID;
        target.sign = 1;
        // 与单电机反馈使用相同的 LOCK 语义：锁定通道限制在标定 ABS 范围内，
        // 未锁定通道继续保持反馈控制，并允许运动到该范围之外。
        target.targetAbs = clampServoPosByMotorAbsLimit(sharedData,
                                                        motorChannel,
                                                        tendonTargets.targetAbs[i]);
        if (outPidPushActive &&
            fabsf(tendonTargets.pidCorrection[i]) >= 1.0f &&
            tendonTargets.pidCorrection[i] * tendonTargets.motorErr[i] > 0.0f) {
            *outPidPushActive = true;
        }
        actualBoundaryReached[i] = isActualAbsAtDirectionalLutBoundary(
            servoData.servoAngles[motorChannel],
            tendonTargets.rawTargetAbs[i] -
                servoData.servoAngles[motorChannel],
            tendonTargets.lutMinAbs[i],
            tendonTargets.lutMaxAbs[i]) ? 1 : 0;
        // 双电机任意一侧实际抵达其推进方向边界时，才认为关节工作空间耗尽。
        // “PID 原始目标被 LUT 钳位”本身不再禁止碰撞 HOLD。
        if (outWorkspaceBoundaryReached && actualBoundaryReached[i] != 0) {
            *outWorkspaceBoundaryReached = true;
        }
    }

    if (logPidDiag && targetCount == 2) {
        const uint8_t m0 = runtimeTargets[0].motorChannel;
        const uint8_t m1 = runtimeTargets[1].motorChannel;
        const int32_t abs0 = servoData.servoAngles[m0];
        const int32_t abs1 = servoData.servoAngles[m1];
        const int16_t load0 = telemetryFresh ? servoLoadMagnitude(telemetryData.load[m0]) : 0;
        const int16_t load1 = telemetryFresh ? servoLoadMagnitude(telemetryData.load[m1]) : 0;
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

static int16_t servoLoadMagnitude(int16_t rawLoad)
{
    // 舵机负载的符号/高位编码方向；故障逻辑只关心 10 bit 幅值。
    int16_t mag = rawLoad;
    if (mag < 0) {
        mag = -mag;
    }
    return (int16_t)(mag & 0x03FF);
}

static uint32_t activeOverloadMotorMask(TaskSharedData_t* sharedData,
                                        const ControlCommandSnapshot_t& commandSnapshot)
{
    // 过载检查只关注当前控制模式正在主动命令的电机；
    // 空闲电机的负载可能是旧值或与当前动作无关。
    if (!sharedData) {
        return 0;
    }
    const uint32_t validMask = validMotorChannelMask();
    if (commandSnapshot.controlMode == CONTROL_MODE_DIRECT_MOTOR) {
        return sharedAtomicLoad(&sharedData->active_motor_mask) & validMask;
    }

    uint32_t mask = 0;
    const uint32_t activeJointMask =
        sharedAtomicLoad(&sharedData->active_joint_mask) &
        ((1UL << ENCODER_TOTAL_NUM) - 1UL);
    for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++) {
        if ((activeJointMask & (1UL << jointIndex)) == 0) {
            continue;
        }
        const uint8_t actuatorCount = getJointActuatorCount(jointIndex);
        for (uint8_t actuatorIndex = 0; actuatorIndex < actuatorCount; actuatorIndex++) {
            JointActuatorItem actuator;
            if (getJointActuator(jointIndex, actuatorIndex, &actuator) &&
                actuator.motorChannel < SERVO_TOTAL_NUM) {
                mask |= (1UL << actuator.motorChannel);
            }
        }
    }
    return mask & validMask;
}

static bool jointHasOverloadFault(uint8_t jointIndex, uint32_t overloadFaultBitmap)
{
    // 一个关节的任一执行器存在电机故障时，整个关节都应保持。
    const uint8_t actuatorCount = getJointActuatorCount(jointIndex);
    for (uint8_t actuatorIndex = 0; actuatorIndex < actuatorCount; actuatorIndex++) {
        JointActuatorItem actuator;
        if (getJointActuator(jointIndex, actuatorIndex, &actuator) &&
            actuator.motorChannel < SERVO_TOTAL_NUM &&
            (overloadFaultBitmap & (1UL << actuator.motorChannel)) != 0) {
            return true;
        }
    }
    return false;
}

static int32_t abs32Local(int32_t value)
{
    // 本地小工具函数，避免嵌入式构建中依赖重载 abs()。
    return value < 0 ? -value : value;
}

static bool directAbsoluteTargetsReached(TaskSharedData_t* sharedData,
                                         const ServoAngleData_t& servoData,
                                         const ControlCommandSnapshot_t& commandSnapshot)
{
    // 直控 RUN 完成判定只对绝对多圈命令有意义。
    if (!sharedData ||
        commandSnapshot.controlMode != CONTROL_MODE_DIRECT_MOTOR ||
        commandSnapshot.motorDirectCommandSource != MOTOR_DIRECT_SOURCE_ABSOLUTE) {
        return false;
    }

    const uint32_t requiredMask =
        sharedAtomicLoad(&sharedData->active_motor_mask) & validMotorChannelMask();
    if (requiredMask == 0) {
        return false;
    }

    bool checkedAny = false;
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        if (((requiredMask >> ch) & 0x1UL) == 0) {
            continue;
        }
        checkedAny = true;
        if (servoData.onlineStatus[ch] == 0) {
            return false;
        }
        const int32_t targetAbs =
            (int32_t)clampServoPos(commandSnapshot.motorTargetRaw[ch]);
        if (abs32Local(servoData.servoAngles[ch] - targetAbs) > kDirectRunDoneAbsToleranceCounts) {
            return false;
        }
    }
    return checkedAny;
}

static bool feedbackJointTargetsReached(TaskSharedData_t* sharedData,
                                        const MappedAngleData_t& mappedData,
                                        const float* targetDegs,
                                        const float* actualDegs,
                                        uint32_t commandJointMask)
{
    // 反馈 RUN 完成要求每个激活关节都有有效映射反馈，
    // 并连续多个周期保持在角度容差内。
    if (!sharedData || !targetDegs || !actualDegs || !mappedData.isValid) {
        return false;
    }

    const uint32_t requiredMask = commandJointMask &
        sharedAtomicLoad(&sharedData->active_joint_mask) &
        ((1UL << ENCODER_TOTAL_NUM) - 1UL);
    if (requiredMask == 0) {
        return false;
    }

    bool checkedAny = false;
    for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++) {
        if (((requiredMask >> jointIndex) & 0x1UL) == 0) {
            continue;
        }
        checkedAny = true;
        if (mappedData.validFlags[jointIndex] == 0) {
            return false;
        }
        if (fabsf(actualDegs[jointIndex] - targetDegs[jointIndex]) > kFeedbackRunDoneToleranceDeg) {
            return false;
        }
    }

    return checkedAny;
}

static bool readControlCommandSnapshot(TaskSharedData_t* sharedData, ControlCommandSnapshot_t* out)
{
    // 每个控制周期只读取一次上位机命令，避免混入部分更新。
    if (!sharedData || !out) {
        return false;
    }

    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(2)) != pdTRUE) {
        return false;
    }

    memcpy(out->targetAngles, sharedData->targetAngles, sizeof(out->targetAngles));
    memcpy(out->motorTargetRaw, sharedData->motorTargetRaw, sizeof(out->motorTargetRaw));
    memcpy(out->motorSweepTargetRaw, sharedData->motorSweepTargetRaw, sizeof(out->motorSweepTargetRaw));
    out->motorCommandToken = sharedData->motor_command_token;
    out->motorSweepCommandToken = sharedData->motor_sweep_command_token;
    out->jointCommandToken = sharedData->joint_command_token;
    out->feedbackCommandJointMask = sharedData->feedback_command_joint_mask;
    out->motorDirectCommandGeneration = sharedData->motor_direct_command_generation;
    out->motorDirectCommandSource = sharedData->motor_direct_command_source;
    out->controlMode = sharedData->control_mode;
    if (lock) {
        xSemaphoreGive(lock);
    }
    return true;
}

// ============================================================================
// ControlTask - 核心实时控制任务。
// ============================================================================
// 运行频率为 100 Hz（10 ms 周期）。职责：
// 1. 读取舵机绝对位置反馈和磁编码器角度。
// 2. 将上位机命令快照到 ControlCommandSnapshot_t。
// 3. 按关节闭环模式或电机直控模式分发。
// 4. 执行安全检查：停止状态、过载、编码器故障、工作空间限位。
// 5. 运行 ControlSolver，并向总线层发布一个舵机目标 batch。
// ============================================================================
void controlTask(void* parameter)
{
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    if (!sharedData) {
        vTaskDelete(NULL);
        return;
    }

    g_controlSolver.begin();

    // 每 10 ms 复用的工作缓冲区，避免实时任务中进行堆分配。
    float localTargets[ENCODER_TOTAL_NUM] = {0.0f};
    float magAngles[ENCODER_TOTAL_NUM] = {0.0f};
    int32_t absolutePositionRaw[ENCODER_TOTAL_NUM] = {0};
    int32_t absolutePositionTendon[ENCODER_TOTAL_NUM] = {0};
    int32_t outPulsesTendon[ENCODER_TOTAL_NUM] = {0};

    RemoteSensorData_t sensorData;
    MappedAngleData_t mappedData;
    uint32_t overloadFaultLatched = 0;
    uint8_t overloadHighCount[SERVO_TOTAL_NUM] = {0};
    // 碰撞冻结使用触发瞬间的舵机实际 ABS；PID/LUT 证据取上一控制周期结果。
    int32_t feedbackCollisionHoldAbs[ENCODER_TOTAL_NUM][MAX_JOINT_ACTUATORS] = {{0}};
    bool feedbackCollisionPidPushActive[ENCODER_TOTAL_NUM] = {false};
    bool feedbackCollisionWorkspaceBoundaryReached[ENCODER_TOTAL_NUM] = {false};
    uint32_t lastFeedbackCollisionSensorTimestamp = 0;
    uint8_t upperWorkspaceHighCount[ENCODER_TOTAL_NUM] = {0};
    uint32_t upperWorkspaceWarningBitmap = 0;
    uint32_t lastOverloadFaultResetToken =
        sharedAtomicLoad(&sharedData->overload_fault_reset_token);
    {
        // 控制任务启动时清空对外公开的故障状态。
        SharedSafetyGuard guard;
        sharedAtomicStore(&sharedData->overload_fault_bitmap, (uint32_t)0);
        for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
            sharedData->overload_fault_load[ch] = 0;
        }
    }
    ControlCommandSnapshot_t commandSnapshot;
    memset(&commandSnapshot, 0, sizeof(commandSnapshot));
    readControlCommandSnapshot(sharedData, &commandSnapshot);

    TickType_t lastWakeTime = xTaskGetTickCount();
    TickType_t lastPidComputeTick = lastWakeTime;
    const TickType_t solverPeriodTicks = pdMS_TO_TICKS(10);
    // 新上线电机会保持一个命令周期，避免 direct/joint 模式立即跳到过期 UI 目标。
    bool prevMotorOnline[SERVO_TOTAL_NUM] = {false};
    bool hotplugHoldMotor[SERVO_TOTAL_NUM] = {false};
    // token 由上位机/状态机单调更新，使本任务无需逐项比较目标即可发现新命令。
    uint32_t lastMotorCommandToken = commandSnapshot.motorCommandToken;
    uint32_t lastMotorSweepCommandToken = commandSnapshot.motorSweepCommandToken;
    uint32_t lastJointCommandToken = commandSnapshot.jointCommandToken;
    uint8_t lastMotorDirectSource = commandSnapshot.motorDirectCommandSource;
    // sweep 模式把 0..4095 raw 命令展开为连续多圈运动。
    int16_t prevSweepRawTarget[SERVO_TOTAL_NUM] = {0};
    int32_t sweepExpandedTarget[SERVO_TOTAL_NUM] = {0};
    bool sweepAnchorValid[SERVO_TOTAL_NUM] = {false};
    // RUN 完成判定使用去抖计数，避免单个噪声帧误判动作完成。
    uint8_t directRunDoneStableCount = 0;
    uint8_t feedbackRunDoneStableCount = 0;
    uint32_t feedbackRunToken = 0;
    uint32_t feedbackRunStartMs = 0;
    uint32_t feedbackRunJointMask = 0;
    float feedbackRunTargetDeg[ENCODER_TOTAL_NUM] = {0.0f};
    bool feedbackRunTracking = false;
    bool feedbackRunDoneEventQueued = false;
    bool feedbackRunTimeoutEventQueued = false;

    while (1)
    {
        taskRuntimeMarkLoopStart(TASK_RUNTIME_CONTROL);
        if (!readControlCommandSnapshot(sharedData, &commandSnapshot)) {
            // 如果命令互斥锁短暂繁忙，本帧跳过输出，但保持周期调度对齐。
            taskRuntimeMarkLoopEnd(TASK_RUNTIME_CONTROL);
            const TickType_t nextWakeTime = lastWakeTime + solverPeriodTicks;
            const bool periodOverrun = (int32_t)(xTaskGetTickCount() - nextWakeTime) >= 0;
            taskRuntimeMarkPeriodResult(TASK_RUNTIME_CONTROL, periodOverrun);
            vTaskDelayUntil(&lastWakeTime, solverPeriodTicks);
            if (periodOverrun) vTaskDelay(1);
            continue;
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
        targetBatch.timestamp = millis();
        targetBatch.source = SERVO_TARGET_OWNER_CONTROL;
        int32_t jointCmdPos[ENCODER_TOTAL_NUM] = {0};
        uint8_t jointCmdValid[ENCODER_TOTAL_NUM] = {0};

        ServoAngleData_t servoData;
        memset(&servoData, 0, sizeof(servoData));
        servoData.timestamp = millis();

        if (sharedData->servoFeedbackQueue) {
            // 只 peek 不消费：多个任务都可能需要查看最新反馈。
            xQueuePeek(sharedData->servoFeedbackQueue, &servoData, 0);
        }

        ServoTelemetryData_t telemetryData;
        memset(&telemetryData, 0, sizeof(telemetryData));
        bool telemetryFresh = false;
        if (sharedData->servoTelemetrySnapshotQueue &&
            xQueuePeek(sharedData->servoTelemetrySnapshotQueue, &telemetryData, 0) == pdTRUE) {
            // 负载/电流数据用于过载保护，因此比编码器映射更快过期。
            telemetryFresh =
                (uint32_t)(millis() - telemetryData.timestamp) <= kServoTelemetryTimeoutMs;
        }

        for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++)
        {
            // 检测电机从离线到在线的热插拔变化，并保持恢复电机直到下一条显式命令。
            const bool onlineNow = (servoData.onlineStatus[ch] != 0);
            if (onlineNow && !prevMotorOnline[ch]) {
                hotplugHoldMotor[ch] = true;
                sweepAnchorValid[ch] = false;
            }
            if (!onlineNow) {
                sweepAnchorValid[ch] = false;
            }
            prevMotorOnline[ch] = onlineNow;
        }

        sharedAtomicFetchOr(&sharedData->overload_fault_bitmap, overloadFaultLatched);
        overloadFaultLatched = sharedAtomicLoad(&sharedData->overload_fault_bitmap);

        // 按关节缓存当前电机位置，分别保存 raw 坐标和腱绳坐标。
        // PID 使用腱绳空间，总线命令使用 raw 空间。
        memset(absolutePositionRaw, 0, sizeof(absolutePositionRaw));
        memset(absolutePositionTendon, 0, sizeof(absolutePositionTendon));
        for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++)
        {
            const uint8_t bus = jointMap[jointIndex].busIndex;
            const uint8_t id = jointMap[jointIndex].servoID;
            const int ch = findMotorChannel(bus, id);
            if (ch >= 0 && servoData.onlineStatus[ch]) {
                const int32_t rawAbs = servoData.servoAngles[ch];
                absolutePositionRaw[jointIndex] = rawAbs;
                absolutePositionTendon[jointIndex] =
                    motorRawAbsToTendonAbs((uint8_t)ch, rawAbs);
            }
        }


        bool canBusOnline = false;
        if (xQueuePeek(sharedData->canRxQueue, &sensorData, 0) == pdTRUE && sensorData.isValid) {
            // 超过 CAN 超时时间后，磁编码器反馈视为过期。
            const uint32_t nowMs = millis();
            canBusOnline = (nowMs - sensorData.timestamp) <= kCanBusOfflineTimeoutMs;
        }

        memset(&mappedData, 0, sizeof(mappedData));
        mappedData.timestamp = millis();
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

                const int32_t orientedRaw = orientEncoderRaw(sensorData.encoderValues[i], g_encoderDirection[i]);
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
                if (calib.mappingVersion >= 2) {
                    // 2 版及以上映射可检测超出标定工作空间的计数，
                    // 并上报去抖后的上限警告。
                    EncoderUrdfMapResult mapping;
                    const bool mapped =
                        calib.mappingVersion == kManualScaledUrdfFeedbackMappingVersion
                        ? mapManualScaledEncoderToUrdf(
                              (uint8_t)i,
                              calib,
                              g_manualEncoderReference[i],
                              sensorData.encoderValues[i],
                              g_encoderDirection[i],
                              &mapping)
                        : mapJointCalibrationEncoderToUrdf(
                              (uint8_t)i,
                              calib,
                              orientedRaw,
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
                    if (outsideCounts > kUpperWorkspaceWarningCounts) {
                        if (upperWorkspaceHighCount[i] < 255) {
                            upperWorkspaceHighCount[i]++;
                        }
                        if (upperWorkspaceHighCount[i] >=
                                kUpperWorkspaceWarningStableFrames &&
                            (upperWorkspaceWarningBitmap & bit) == 0) {
                            upperWorkspaceWarningBitmap |= bit;
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
                        if ((upperWorkspaceWarningBitmap & bit) != 0) {
                            FW_LOG_INFO_PRINTF(
                                "[CONTROL] UPPER_WORKSPACE_RECOVER J%02u outside=%ld\r\n",
                                (unsigned)i,
                                (long)outsideCounts);
                        }
                        upperWorkspaceHighCount[i] = 0;
                        upperWorkspaceWarningBitmap &= ~bit;
                    }
                } else {
                    mappedDeg = mapCalibratedEncoderToDeg(calib, orientedRaw);
                    mappedData.outsideCounts[i] = 0;
                }
                const int32_t mappedCount = degToEncoderCount(mappedDeg);
                mappedData.angleValues[i] = clampMappedCountForProtocol(mappedCount);
                mappedData.validFlags[i] = 1;
                magAngles[i] = mappedDeg;
            }
        }

        if (sharedData->mappedAngleQueue) {
            xQueueOverwrite(sharedData->mappedAngleQueue, &mappedData);
        }

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
                activeOverloadMotorMask(sharedData, commandSnapshot);
            for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++)
            {
                const uint32_t bit = 1UL << ch;
                if (((overloadMotorMask & bit) == 0) ||
                    telemetryData.onlineStatus[ch] == 0)
                {
                    overloadHighCount[ch] = 0;
                    continue;
                }

                const bool jointControlMode =
                    (commandSnapshot.controlMode == CONTROL_MODE_JOINT);
                const int16_t overloadThreshold = jointControlMode
                                                      ? kJointControlOverloadLoadThreshold
                                                      : kControlOverloadLoadThreshold;
                const uint8_t overloadLatchCycles = jointControlMode
                                                        ? kJointControlOverloadLatchCycles
                                                        : kControlOverloadLatchCycles;
                const int16_t loadMag = servoLoadMagnitude(telemetryData.load[ch]);
                if (loadMag >= overloadThreshold) {
                    if (overloadHighCount[ch] < 255) {
                        overloadHighCount[ch]++;
                    }
                } else {
                    overloadHighCount[ch] = 0;
                }

                if (overloadHighCount[ch] >= overloadLatchCycles &&
                    (overloadFaultLatched & bit) == 0)
                {
                    overloadFaultLatched |= bit;
                    {
                        SharedSafetyGuard guard;
                        sharedData->overload_fault_load[ch] = telemetryData.load[ch];
                    }
                    FW_LOG_INFO_PRINTF("[CONTROL] OVERLOAD_HOLD M%02u load=%d threshold=%d cycles=%u mode=%u\r\n",
                                  (unsigned)ch,
                                  loadMag,
                                  overloadThreshold,
                                  (unsigned)overloadLatchCycles,
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
            // 本地正弦生成后仍要限幅，确保测试轨迹不会请求超出标定关节工作空间。
            for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++) {
                localTargets[jointIndex] = clampJointTargetDegByCalib(localTargets[jointIndex], jointIndex);
            }
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
                feedbackRunStartMs = millis();
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
            const TickType_t pidComputeTick = xTaskGetTickCount();
            const TickType_t elapsedPidTicks = pidComputeTick - lastPidComputeTick;
            lastPidComputeTick = pidComputeTick;
            float pidDtSeconds =
                (float)elapsedPidTicks * (float)portTICK_PERIOD_MS * 0.001f;
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
                    const uint8_t bus = motorMap[ch].busIndex;
                    const uint8_t id = motorMap[ch].servoID;
                    if (bus >= NUM_BUSES) {
                        continue;
                    }

                    int32_t targetPos = clampServoPos(servoData.servoAngles[ch]);
                    if (!hotplugHoldMotor[ch]) {
                        if (motorDirectSource == MOTOR_DIRECT_SOURCE_SWEEP) {
                            // sweep 目标是单圈 raw 值，通过增量展开实现连续旋转。
                            const int16_t rawTarget =
                                clampServoSingleTurnRaw(commandSnapshot.motorSweepTargetRaw[ch]);
                            if (!sweepAnchorValid[ch]) {
                                sweepExpandedTarget[ch] =
                                    anchorSingleTurnSweepToCurrent(servoData.servoAngles[ch], rawTarget);
                                prevSweepRawTarget[ch] = rawTarget;
                                sweepAnchorValid[ch] = true;
                            } else if (rawTarget != prevSweepRawTarget[ch]) {
                                const int32_t d = (int32_t)rawTarget - (int32_t)prevSweepRawTarget[ch];
                                sweepExpandedTarget[ch] += wrapServoRawDelta(d);
                                prevSweepRawTarget[ch] = rawTarget;
                            }
                            targetPos = clampServoPos(sweepExpandedTarget[ch]);
                            sweepExpandedTarget[ch] = targetPos;
                        } else if (motorDirectSource == MOTOR_DIRECT_SOURCE_ABSOLUTE) {
                            // absolute 来源本身已经使用多圈舵机计数。
                            sweepAnchorValid[ch] = false;
                            targetPos = clampServoPos(
                                (int32_t)commandSnapshot.motorTargetRaw[ch]);
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
                    appendServoTarget(&targetBatch, bus, id, targetPos,
                                      SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
                }
            }
            else
            {
            for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++)
            {
                // 关节模式会对每个关节独立决定：保持、使用双腱控制或输出普通 PID 目标。
                const uint8_t bus = jointMap[jointIndex].busIndex;
                const uint8_t id = jointMap[jointIndex].servoID;
                const int jointMotorCh = findMotorChannel(bus, id);
                const bool servoOnline = (jointMotorCh >= 0) && (servoData.onlineStatus[jointMotorCh] != 0);
                bool overloadFaultActive = jointHasOverloadFault(jointIndex, overloadFaultLatched);

                bool emergency =
                    (!normalOutputAllowed) ||
                    shouldEmergencyStop(canBusOnline,
                                        sensorData,
                                        mappedData,
                                        jointIndex,
                                        ((sharedAtomicLoad(&sharedData->encoder_fault_bitmap) &
                                          (1UL << jointIndex)) != 0));
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

                if (bus >= NUM_BUSES) {
                    continue;
                }

                if (!jointInScope)
                {
                    // 未激活关节不发布主动命令，并丢弃双腱状态，便于重新启用时干净启动。
                    if (isDualTendonJoint(jointIndex)) {
                        resetDualTendonController(jointIndex);
                    }
                    jointCmdPos[jointIndex] = clampServoPos(absolutePositionRaw[jointIndex]);
                    jointCmdValid[jointIndex] = 0;
                    continue;
                }

                if (emergency)
                {
                    // 反馈故障、总线丢失、输出关闭和过载都会统一进入保持当前位置行为。
                    if (isDualTendonJoint(jointIndex)) {
                        resetDualTendonController(jointIndex);
                    }
                    const int32_t holdPos = clampServoPos(absolutePositionRaw[jointIndex]);
                    appendJointActuatorHoldTargets(&targetBatch, jointIndex, servoData,
                                                   SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
                    jointCmdPos[jointIndex] = holdPos;
                    jointCmdValid[jointIndex] = servoOnline ? 1 : 0;
                    continue;
                }

                if (hotplugHoldActive)
                {
                    // 刚上线的电机会保持在反馈位置，直到操作端命令它有意移动。
                    if (isDualTendonJoint(jointIndex)) {
                        resetDualTendonController(jointIndex);
                    }
                    const int32_t holdPos = clampServoPos(absolutePositionRaw[jointIndex]);
                    appendJointActuatorHoldTargets(&targetBatch, jointIndex, servoData,
                                                   SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
                    jointCmdPos[jointIndex] = holdPos;
                    jointCmdValid[jointIndex] = servoOnline ? 1 : 0;
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
                    jointCmdPos[jointIndex] =
                        clampServoPos(absolutePositionRaw[jointIndex]);
                    jointCmdValid[jointIndex] = servoOnline ? 1 : 0;
                    continue;
                }

                if (g_feedbackCollisionGuard.isHoldActive(jointIndex))
                {
                    if (isDualTendonJoint(jointIndex)) {
                        resetDualTendonController(jointIndex);
                    }
                    const uint8_t appended = appendJointActuatorSnapshotTargets(
                        &targetBatch,
                        jointIndex,
                        servoData,
                        feedbackCollisionHoldAbs[jointIndex],
                        kJointControlFeedbackSpeed,
                        SERVO_TARGET_ACC_DEFAULT);
                    localTargets[jointIndex] =
                        g_feedbackCollisionGuard.holdAngleDeg(jointIndex);
                    jointCmdPos[jointIndex] =
                        clampServoPos(feedbackCollisionHoldAbs[jointIndex][0]);
                    jointCmdValid[jointIndex] = appended > 0 ? 1 : 0;
                    continue;
                }

                int32_t targetPos = clampServoPos(absolutePositionRaw[jointIndex]);

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
                                                servoData,
                                                telemetryData,
                                                telemetryFresh,
                                                kJointControlFeedbackSpeed,
                                                SERVO_TARGET_ACC_DEFAULT,
                                                &feedbackCollisionPidPushActive[jointIndex],
                                                &feedbackCollisionWorkspaceBoundaryReached[jointIndex]);
                    if (appended == 0) {
                        appendJointActuatorHoldTargets(&targetBatch, jointIndex, servoData,
                                                       SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
                    }
                    jointCmdPos[jointIndex] = targetPos;
                    jointCmdValid[jointIndex] = (appended > 0) ? 1 : 0;
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
                        lutWorkspaceValid = clampSingleTendonTargetByLut(
                            jointIndex,
                            motorCh,
                            rawPidTargetAbs,
                            &lutClampedTarget,
                            &lutWorkspaceMinAbs,
                            &lutWorkspaceMaxAbs);
                        if (lutWorkspaceValid) {
                            pidTargetLutClamped =
                                lutClampedTarget != rawPidTargetAbs;
                            targetPos = clampServoPosByMotorAbsLimit(sharedData,
                                                                     motorCh,
                                                                     lutClampedTarget);
                        } else {
                            // 标定结果或电机映射异常时保持当前位置，不能绕过
                            // LUT 工作空间继续发送位置式 PID 输出。
                            targetPos = clampServoPos(servoData.servoAngles[motorCh]);
                        }
                    } else {
                        targetPos = clampServoPos(outPulsesTendon[jointIndex]);
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
                        isActualAbsAtDirectionalLutBoundary(
                            actualMotorAbs,
                            rawPidTargetAbs - actualMotorAbs,
                            lutWorkspaceMinAbs,
                            lutWorkspaceMaxAbs);
                    feedbackCollisionWorkspaceBoundaryReached[jointIndex] =
                        !lutWorkspaceValid || actualBoundaryReached;

                    if (feedbackOutputState && shouldLogControlPidDiag(
                            jointIndex,
                            millis(),
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
                            ? servoLoadMagnitude(rawLoad)
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
                            (unsigned long)millis(),
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
                                               jointActive ? kJointControlFeedbackSpeed : SERVO_TARGET_SPEED_DEFAULT,
                                               SERVO_TARGET_ACC_DEFAULT);
                }
                jointCmdPos[jointIndex] = targetPos;
                jointCmdValid[jointIndex] = 1;
            }
            }
        }

        if (sharedData->jointDebugQueue)
        {
            // 发布紧凑的滚动调试样本。队列满时丢弃最旧样本，
            // 保证 UI 看到新鲜的控制环数据。
            for (uint8_t di = 0; di < kDebugJointCount; di++)
            {
                const uint8_t jointIndex = kDebugJointIndices[di];
                JointDebugData_t debugData;
                memset(&debugData, 0, sizeof(debugData));
                debugData.jointIndex = jointIndex;
                debugData.timestamp = millis();
                debugData.valid = (mappedData.validFlags[jointIndex] != 0) ? 1 : 0;
                debugData.targetDeg = localTargets[jointIndex];
                debugData.magActualDeg = magAngles[jointIndex];
                debugData.loop1Output = g_controlSolver.getPidOutput(jointIndex);
                debugData.cmdTargetPos = jointCmdPos[jointIndex];
                debugData.cmdValid = jointCmdValid[jointIndex];

                if (xQueueSend(sharedData->jointDebugQueue, &debugData, 0) != pdTRUE) {
                    JointDebugData_t drop;
                    xQueueReceive(sharedData->jointDebugQueue, &drop, 0);
                    xQueueSend(sharedData->jointDebugQueue, &debugData, 0);
                }
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
        if (sharedData->servoTargetQueue &&
            targetBatch.count > 0 &&
            controlTargetPublishAllowed) {
            // 发布前再次检查 owner/state，防止状态切换时泄漏过期的 CONTROL 目标 batch。
            p4::actuator::publishLatest(sharedData, targetBatch);
        }

        if (directRunningState &&
            commandSnapshot.controlMode == CONTROL_MODE_DIRECT_MOTOR &&
            commandSnapshot.motorDirectCommandSource == MOTOR_DIRECT_SOURCE_ABSOLUTE) {
            // 当所有激活电机连续多个控制帧保持在容差内时，直控 absolute RUN 完成。
            if (directAbsoluteTargetsReached(sharedData, servoData, commandSnapshot)) {
                if (directRunDoneStableCount < 255) {
                    directRunDoneStableCount++;
                }
                if (directRunDoneStableCount >= kDirectRunDoneStableCycles) {
                    postSystemEventForToken(
                        sharedData,
                        SYSTEM_EVENT_RUN_DIRECT_DONE,
                        commandSnapshot.motorDirectCommandGeneration);
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
            const bool targetsReached = feedbackJointTargetsReached(
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
                if (feedbackRunDoneStableCount >= kFeedbackRunDoneStableCycles &&
                    !feedbackRunDoneEventQueued) {
                    // 队列满是暂态问题；保留完成事件并重试，
                    // 避免丢弃事件导致 RUN_FEEDBACK 一直锁定。
                    feedbackRunDoneEventQueued = postSystemEventForToken(
                        sharedData,
                        SYSTEM_EVENT_RUN_FEEDBACK_DONE,
                        feedbackRunToken
                    );
                }
            } else {
                feedbackRunDoneStableCount = 0;
            }

            const uint32_t elapsedMs = (uint32_t)(millis() - feedbackRunStartMs);
            const bool collisionHoldActive =
                sharedAtomicLoad(&sharedData->feedback_collision_hold_bitmap) != 0;
            // 碰撞保持必须持续锁住触发时的舵机 ABS，直到反向目标或模式退出；
            // 此时不能让旧的单次负反馈超时提前结束 RUN_FEEDBACK。
            if (!targetsReached &&
                !collisionHoldActive &&
                !feedbackRunDoneEventQueued &&
                elapsedMs >= kFeedbackRunTimeoutMs &&
                !feedbackRunTimeoutEventQueued) {
                feedbackRunTimeoutEventQueued = postSystemEventForToken(
                    sharedData,
                    SYSTEM_EVENT_RUN_FEEDBACK_TIMEOUT,
                    feedbackRunToken
                );
            }
        }

        taskRuntimeMarkLoopEnd(TASK_RUNTIME_CONTROL);
        const TickType_t nextWakeTime = lastWakeTime + solverPeriodTicks;
        const bool periodOverrun = (int32_t)(xTaskGetTickCount() - nextWakeTime) >= 0;
        // 运行时监视器记录本循环是否错过 10 ms 截止时间。
        taskRuntimeMarkPeriodResult(TASK_RUNTIME_CONTROL, periodOverrun);
        vTaskDelayUntil(&lastWakeTime, solverPeriodTicks);
        if (periodOverrun) vTaskDelay(1);
    }
}
