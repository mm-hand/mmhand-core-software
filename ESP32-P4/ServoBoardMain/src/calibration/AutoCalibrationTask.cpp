/**
 * @file AutoCalibrationTask.cpp
 * @brief 编排单腱与双腱关节自动标定、工作空间准备和标定结果提交。
 *
 * 本任务拥有标定期间的舵机命令 owner。底层舵机通信统一经
 * ActuatorCommandGateway，Manual/Auto 数据格式和协议布局保持不变。
 */
#include "AutoCalibrationTask.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "CalibrationMath.h"
#include "CalibrationFeedbackReader.h"
#include "CalibrationTask.h"
#include "../control/ControlTask.h"
#include "../control/TendonCalibrationLut.h"
#include "../config/CalibrationConfig.h"
#include "../hardware/HardwareMap.h"
#include "../hardware/servo/ServoValue.h"
#include "../persistence/ManualCalibrationNvsStore.h"
#include "../shared/SharedSafetyState.h"
#include "../platform/TaskFactory.h"
#include "../actuator/ActuatorCommandGateway.h"
#include "../system/FirmwareLog.h"
#include "../system/StateMachineTask.h"
#include "../system/TaskRuntimeMonitor.h"

using namespace mmhand::config::calibration;
using namespace mmhand::calibration;

// 非空表示自动标定任务正在运行，用于拒绝重复创建。
static TaskHandle_t g_autoCalibrationTaskHandle = NULL;

// 默认全手标定顺序：四指依次完成后，再按 J20、J16~J19 标定拇指。
const uint8_t kDefaultAutoCalibrationJoints[] = {
    0, 1, 2, 3,
    4, 5, 6, 7,
    8, 9, 10, 11,
    12, 13, 14, 15,
    20, 16, 17, 18, 19
};
// 默认数组元素数，供全手标定和 active scope 过滤使用。
const uint8_t kDefaultAutoCalibrationJointsCount =
    sizeof(kDefaultAutoCalibrationJoints) /
    sizeof(kDefaultAutoCalibrationJoints[0]);
// 只有四指侧摆双腱关节参与“为相邻手指让出标定空间”的姿态规划；
// 拇指机械独立，不进入该规划。
static const uint8_t kWorkspaceDualJoints[] = {0, 4, 8, 12};
static const uint8_t kWorkspaceDualJointsCount =
    sizeof(kWorkspaceDualJoints) / sizeof(kWorkspaceDualJoints[0]);
static const uint8_t kWorkspaceDualHelperCount = kWorkspaceDualJointsCount - 1;

// 将用户选择的关节转换为机械安全顺序。四指保持 J00~J15，拇指固定为
// J20 -> J16 -> J17 -> J18 -> J19。
static uint8_t autoCalibrationSequenceRank(uint8_t jointIndex)
{
    if (jointIndex < 16) {
        return jointIndex;
    }
    switch (jointIndex) {
        case 20:
            return 16;
        case 16:
            return 17;
        case 17:
            return 18;
        case 18:
            return 19;
        case 19:
            return 20;
        default:
            return jointIndex;
    }
}

// 反馈新鲜度、行程合法性和机械搜索参数。
// 热插拔恢复预检期间不发送任何舵机目标；只有驱动恢复、多圈模式配置和反馈连续稳定后才允许运动。
// 单关节标定成功后，通过舵机通信任务提交本次会话可信状态的最长等待时间。
// 接近整圈时端点方向会产生歧义，因此拒绝跨度大于 15360 counts 的结果。
// 原始缓冲容量为最终 LUT 的三倍，为不等速采样和端点样本预留空间。
static const uint8_t kTendonRawSampleCapacity =
    TENDON_LUT_MAX_SAMPLES * 3;

// 一次原始 LUT 采样：磁编方向坐标和两路舵机多圈 ABS。
struct TendonRawFitSample {
    int32_t encoder;      // 方向归一化后的磁编坐标。
    int32_t primaryAbs;   // 主电机实测多圈 ABS。
    int32_t secondaryAbs; // 副电机实测多圈 ABS；单腱时为 0。
};

// 标定期间的稠密原始样本缓冲，最终会重采样为固定角度步长 LUT。
struct TendonRawFitBuffer {
    uint8_t count;             // 当前有效原始样本数。
    float nextTravelDeg;       // 下一次应采样的内部轨迹角。
    TendonRawFitSample samples[kTendonRawSampleCapacity]; // 固定容量，避免堆分配。
};

enum DualWorkspacePose : uint8_t {
    DUAL_WORKSPACE_POSE_NONE = 0,
    // A：主电机放绳 3200 counts，副电机收绳至 load 190。
    DUAL_WORKSPACE_POSE_A = 1,
    // B：副电机放绳 3200 counts，主电机收绳至 load 190。
    DUAL_WORKSPACE_POSE_B = 2,
};

// 四指侧摆关节的 RAM 姿态缓存。缓存记录已实测的 A/B 目标，供同一次标定
// 序列复用，避免重复松绳。
struct DualWorkspaceCacheEntry {
    uint8_t valid;               // 1 表示本轮任务中已实测该缓存。
    uint8_t currentPose;         // 当前 DUAL_WORKSPACE_POSE_*。
    uint8_t primaryMotor;        // 主执行器 Mxx。
    uint8_t secondaryMotor;      // 副执行器 Mxx。
    int32_t primaryTightAbs;     // 主电机 load 190 时的 ABS。
    int32_t primaryReleaseAbs;   // 主电机从紧端放 3200 counts 后的 ABS。
    int32_t secondaryTightAbs;   // 副电机 load 190 时的 ABS。
    int32_t secondaryReleaseAbs; // 副电机从紧端放 3200 counts 后的 ABS。
};

// 一项工作空间动作：把指定双腱关节调整到 A 或 B。
struct DualWorkspacePlanStep {
    uint8_t jointIndex;    // 要移动的侧摆关节。
    DualWorkspacePose pose; // 目标 A/B 姿态。
};

// 写入结构化诊断 valueC 的工作空间失败阶段。
enum DualWorkspacePlanStage : int32_t {
    DUAL_WORKSPACE_PLAN_STAGE_PREFLIGHT = 101,
    DUAL_WORKSPACE_PLAN_STAGE_EXECUTE = 102,
};

// 只保存在 RAM。同一标定序列结束、重启或人工运动后都不能继续信任。
static DualWorkspaceCacheEntry g_dualWorkspaceCache[kWorkspaceDualJointsCount];

// 32 位整数绝对值辅助函数，输入不会取 INT32_MIN。
static int32_t abs32(int32_t value)
{
    return value < 0 ? -value : value;
}

// 将原始多圈 ABS 统一到“收绳为正”的 tendon 坐标，计算从紧端实际放出的 counts。
// 若电机仍在收绳方向，返回 0，避免机械回弹或反馈抖动被当成过度松绳。
static int32_t singleTendonReleaseCounts(uint8_t motorChannel,
                                         int32_t tightAbs,
                                         int32_t currentAbs)
{
    const int32_t tightTendonAbs =
        motorRawAbsToTendonAbs(motorChannel, tightAbs);
    const int32_t currentTendonAbs =
        motorRawAbsToTendonAbs(motorChannel, currentAbs);
    const int32_t releaseCounts = tightTendonAbs - currentTendonAbs;
    return releaseCounts > 0 ? releaseCounts : 0;
}

// 将多圈 ABS 压入飞特位置命令的 int16 安全范围。
static int32_t clampToServoPosLocal(int32_t value)
{
    // 标定轨迹使用 P4 逻辑 ABS，实际舵机行程由通信任务结合单路
    // logicalOffset 做最终校验，不能在这里按固定七圈截断。
    return value;
}

// 与上函数相同，但保留 int32 返回类型，便于后续差值计算。
static int32_t clampToServoAbsRangeLocal(int32_t value)
{
    return (int32_t)clampToServoPosLocal(value);
}

// 根据紧端 ABS 登记一个保守的五圈电机工作区，并增加 500 counts 紧端余量。
static void setCalibratedMotorAbsLimit(TaskSharedData_t* sharedData,
                                       uint8_t motorChannel,
                                       int32_t tightAbs)
{
    if (!sharedData ||
        motorChannel >= SERVO_TOTAL_NUM ||
        !isMotorChannelValid(motorChannel)) {
        return;
    }

    const int32_t tightClamped = clampToServoAbsRangeLocal(tightAbs);
    const int32_t slackClamped = clampToServoAbsRangeLocal(
        motorApplyTendonDelta(motorChannel, tightClamped, -kMotorAbsWindowCounts));
    const int32_t minAbs = (slackClamped < tightClamped) ? slackClamped : tightClamped;
    const int32_t baseMaxAbs = (slackClamped > tightClamped) ? slackClamped : tightClamped;
    const int32_t maxAbs = clampToServoAbsRangeLocal(
        baseMaxAbs + kMotorAbsMaxMarginCounts);
    {
        SharedSafetyGuard guard;
        sharedData->motor_abs_limit_min[motorChannel] = minAbs;
        sharedData->motor_abs_limit_max[motorChannel] = maxAbs;
        sharedAtomicFetchOr(&sharedData->motor_abs_limit_valid_bitmap,
                            (uint32_t)(1UL << motorChannel));
    }
    FW_LOG_INFO_PRINTF("[AUTO CALIB] ABS_LIMIT M%02u min=%ld max=%ld baseMax=%ld tight=%ld slack=%ld shortenSign=%d window=%ld maxMargin=%ld\r\n",
                   (unsigned)motorChannel,
                   (long)minAbs,
                   (long)maxAbs,
                   (long)baseMaxAbs,
                   (long)tightClamped,
                   (long)slackClamped,
                   (int)getMotorTendonShortenAbsSign(motorChannel),
                   (long)kMotorAbsWindowCounts,
                   (long)kMotorAbsMaxMarginCounts);
}

// 单腱标定完成后，以 LUT 实测包络登记电机 ABS 工作区。
static void setSingleTendonLutAbsLimit(TaskSharedData_t* sharedData,
                                      uint8_t motorChannel,
                                      int32_t lutMinAbs,
                                      int32_t lutMaxAbs,
                                      int32_t lutTightAbs)
{
    if (!sharedData || motorChannel >= SERVO_TOTAL_NUM ||
        !isMotorChannelValid(motorChannel)) {
        return;
    }

    // 单腱工作区以 LUT 的实测包络为基础，并沿收绳方向保留原有的 500
    // counts 紧端余量。这里只登记有效范围，不自动 LOCK，保持当前 UI
    // 的“默认解锁、用户按 LOCK 后才硬钳位”语义。
    lutMinAbs = clampToServoAbsRangeLocal(lutMinAbs);
    lutMaxAbs = clampToServoAbsRangeLocal(lutMaxAbs);
    if (lutMinAbs > lutMaxAbs) {
        const int32_t tmp = lutMinAbs;
        lutMinAbs = lutMaxAbs;
        lutMaxAbs = tmp;
    }
    const int32_t tightWithMargin = clampToServoAbsRangeLocal(
        motorApplyTendonDelta(motorChannel,
                              lutTightAbs,
                              kMotorAbsMaxMarginCounts));
    int32_t limitMin = lutMinAbs;
    int32_t limitMax = lutMaxAbs;
    if (tightWithMargin < limitMin) limitMin = tightWithMargin;
    if (tightWithMargin > limitMax) limitMax = tightWithMargin;

    const uint32_t bit = (uint32_t)(1UL << motorChannel);
    {
        SharedSafetyGuard guard;
        sharedData->motor_abs_limit_min[motorChannel] = limitMin;
        sharedData->motor_abs_limit_max[motorChannel] = limitMax;
        sharedAtomicFetchOr(&sharedData->motor_abs_limit_valid_bitmap, bit);
        sharedAtomicFetchAnd(&sharedData->motor_abs_limit_locked_bitmap, ~bit);
    }
    FW_LOG_INFO_PRINTF(
        "[AUTO CALIB] SINGLE_LUT_ABS_LIMIT M%02u lut=[%ld,%ld] "
        "limit=[%ld,%ld] tight=%ld margin=%ld locked=0\r\n",
        (unsigned)motorChannel,
        (long)lutMinAbs,
        (long)lutMaxAbs,
        (long)limitMin,
        (long)limitMax,
        (long)lutTightAbs,
        (long)kMotorAbsMaxMarginCounts);
}

// 清除指定关节所有执行器的旧 ABS 限位，避免新标定被旧范围拦截。
static void clearCalibratedMotorAbsLimitsForJoint(TaskSharedData_t* sharedData,
                                                  uint8_t jointIndex)
{
    if (!sharedData || jointIndex >= ENCODER_TOTAL_NUM) {
        return;
    }

    uint32_t clearedMask = 0;
    JointActuatorGroup group;
    if (getJointActuatorGroup(jointIndex, &group)) {
        for (uint8_t i = 0; i < group.count; i++) {
            const uint8_t motorChannel = group.actuators[i].motorChannel;
            if (motorChannel < SERVO_TOTAL_NUM && isMotorChannelValid(motorChannel)) {
                clearedMask |= (1UL << motorChannel);
            }
        }
    } else {
        const int primaryCh = findMotorChannelByBusId(jointMap[jointIndex].busIndex,
                                                      jointMap[jointIndex].servoID);
        if (primaryCh >= 0 && primaryCh < SERVO_TOTAL_NUM) {
            clearedMask |= (1UL << (uint8_t)primaryCh);
        }
    }

    if (clearedMask == 0) {
        return;
    }

    {
        SharedSafetyGuard guard;
        sharedAtomicFetchAnd(&sharedData->motor_abs_limit_valid_bitmap, ~clearedMask);
        sharedAtomicFetchAnd(&sharedData->motor_abs_limit_locked_bitmap, ~clearedMask);
        for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
            if ((clearedMask & (1UL << ch)) != 0) {
                sharedData->motor_abs_limit_min[ch] = 0;
                sharedData->motor_abs_limit_max[ch] = 0;
            }
        }
    }
    FW_LOG_INFO_PRINTF("[AUTO CALIB] ABS_LIMIT_CLEAR J%02u mask=0x%08lX\r\n",
                  (unsigned)jointIndex,
                  (unsigned long)clearedMask);
}

// 从多圈 ABS 中提取有符号圈数，负数采用向下取整语义。
static int32_t servoTurnFromAbs(int32_t abs)
{
    const int32_t raw = mmhand::servo::normalizeRawSingleTurn(abs);
    return (abs - raw) / 4096;
}

// 原子写入一次结构化标定失败诊断，供上位机弹窗解释具体阶段与对象。
static void setCalibrationDiag(TaskSharedData_t* sharedData,
                               uint8_t reason,
                               uint8_t phase,
                               uint8_t joint,
                               uint8_t motor,
                               int32_t valueA,
                               int32_t valueB,
                               int32_t valueC,
                               uint32_t bitmap)
{
    if (!sharedData || reason == CALIB_DIAG_REASON_NONE) {
        return;
    }
    CalibrationDiagData_t diag;
    memset(&diag, 0, sizeof(diag));
    diag.valid = 1;
    diag.reason = reason;
    diag.phase = phase;
    diag.joint = joint;
    diag.motor = motor;
    diag.valueA = valueA;
    diag.valueB = valueB;
    diag.valueC = valueC;
    diag.bitmap = bitmap;
    diag.timestamp = millis();
    {
        SharedSafetyGuard guard;
        diag.sequence = sharedData->calibration_diag.sequence + 1;
        sharedData->calibration_diag = diag;
    }
}

// 清空上一轮标定诊断，防止旧错误污染本轮结果。
static void clearCalibrationDiag(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return;
    }
    SharedSafetyGuard guard;
    const uint32_t nextSequence = sharedData->calibration_diag.sequence + 1;
    memset(&sharedData->calibration_diag, 0, sizeof(sharedData->calibration_diag));
    sharedData->calibration_diag.sequence = nextSequence;
}

// 判断当前是否已经记录了有效诊断原因。
static bool hasCalibrationDiag(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return false;
    }
    SharedSafetyGuard guard;
    return sharedData->calibration_diag.valid != 0;
}

// 读取当前结构化诊断原因。
static uint8_t calibrationDiagReason(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return CALIB_DIAG_REASON_NONE;
    }
    SharedSafetyGuard guard;
    return sharedData->calibration_diag.valid
        ? sharedData->calibration_diag.reason
        : CALIB_DIAG_REASON_NONE;
}

// 标定循环每帧检查状态机 owner，失去控制权后立即停止发命令。
static bool autoCalibrationStillOwnsControl(TaskSharedData_t* sharedData)
{
    return sharedData &&
           sharedData->system_state == SYSTEM_STATE_CALIBRATION_RUNNING &&
           sharedData->servo_target_owner == SERVO_TARGET_OWNER_CALIBRATION &&
           sharedData->control_enabled == 0;
}

// 读取单台舵机当前上电会话的 logical ABS 可达范围。范围由通信任务依据
// logicalOffset 独立生成，因此一台舵机掉电重建不会改变其他26台舵机。
static bool getServoLogicalCommandRange(TaskSharedData_t* sharedData,
                                        uint8_t motorChannel,
                                        int32_t* outMin,
                                        int32_t* outMax)
{
    if (!sharedData || motorChannel >= SERVO_TOTAL_NUM ||
        !outMin || !outMax) {
        return false;
    }
    const uint32_t bit = 1UL << motorChannel;
    SharedSafetyGuard guard;
    if ((sharedAtomicLoad(&sharedData->servo_logical_range_valid_bitmap) &
         bit) == 0) {
        return false;
    }
    *outMin = sharedData->servo_logical_range_min[motorChannel];
    *outMax = sharedData->servo_logical_range_max[motorChannel];
    return *outMin <= *outMax;
}

// 检查标定命令能否转换成该舵机当前会话的真实硬件多圈目标。
static bool isServoCommandInsideLimit(TaskSharedData_t* sharedData,
                                      uint8_t motorChannel,
                                      int32_t commandAbs)
{
    int32_t rangeMin = 0;
    int32_t rangeMax = 0;
    return getServoLogicalCommandRange(
               sharedData, motorChannel, &rangeMin, &rangeMax) &&
           commandAbs >= rangeMin && commandAbs <= rangeMax;
}

// 根据 bus/id 查找扁平化电机通道 Mxx。
static int findMotorChannel(uint8_t bus, uint8_t id)
{
    return findMotorChannelByBusId(bus, id);
}

// 检查单关节主执行器的多圈状态是否可信。
static bool isJointPrimaryServoMultiTurnKnown(TaskSharedData_t* sharedData, uint8_t jointIndex)
{
    if (!sharedData || jointIndex >= ENCODER_TOTAL_NUM) {
        return false;
    }
    const int ch = findMotorChannel(jointMap[jointIndex].busIndex,
                                    jointMap[jointIndex].servoID);
    if (ch < 0 || ch >= SERVO_TOTAL_NUM) {
        return false;
    }
    return (sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap) &
            (1UL << ch)) == 0;
}

// 汇总指定关节实际使用的执行器。双腱关节返回两台电机，单腱关节返回主电机。
static uint32_t calibrationJointMotorMask(uint8_t jointIndex)
{
    if (jointIndex >= ENCODER_TOTAL_NUM) {
        return 0;
    }
    uint32_t motorMask = 0;
    JointActuatorGroup group;
    if (getJointActuatorGroup(jointIndex, &group)) {
        for (uint8_t i = 0; i < group.count; i++) {
            const uint8_t motorChannel = group.actuators[i].motorChannel;
            if (motorChannel < SERVO_TOTAL_NUM && isMotorChannelValid(motorChannel)) {
                motorMask |= (1UL << motorChannel);
            }
        }
    } else {
        const int primaryCh = findMotorChannel(jointMap[jointIndex].busIndex,
                                               jointMap[jointIndex].servoID);
        if (primaryCh >= 0 && primaryCh < SERVO_TOTAL_NUM) {
            motorMask |= (1UL << primaryCh);
        }
    }
    return motorMask & validMotorChannelMask();
}

// 标定开始前等待热插拔恢复闭环完成。该函数只观察共享状态，不发送任何运动目标。
static bool waitForCalibrationServoRecovery(TaskSharedData_t* sharedData,
                                            uint32_t requiredMask)
{
    if (!sharedData || requiredMask == 0) {
        return false;
    }
    requiredMask &= validMotorChannelMask();
    const uint32_t beginMs = millis();
    uint32_t lastLogMs = 0;
    uint32_t lastFeedbackTimestamp = 0;
    uint8_t stableSamples = 0;

    while (autoCalibrationStillOwnsControl(sharedData)) {
        const uint32_t nowMs = millis();
        const uint32_t onlineMask =
            sharedAtomicLoad(&sharedData->servo_online_bitmap);
        const uint32_t configuredMask =
            sharedAtomicLoad(&sharedData->servo_multiturn_configured_bitmap);
        const uint32_t pendingMask =
            sharedAtomicLoad(&sharedData->servo_recovery_pending_bitmap);
        const uint32_t failedMask =
            sharedAtomicLoad(&sharedData->servo_recovery_failed_bitmap);

        ServoAngleData_t snapshot;
        const bool hasSnapshot =
            sharedData->servoFeedbackQueue &&
            xQueuePeek(sharedData->servoFeedbackQueue, &snapshot, 0) == pdTRUE;
        const bool freshSnapshot =
            hasSnapshot &&
            (uint32_t)(nowMs - snapshot.timestamp) <=
                kFeedbackReadyServoTimeoutMs;
        uint32_t snapshotOnlineMask = 0;
        if (freshSnapshot) {
            for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
                if (snapshot.onlineStatus[ch] != 0) {
                    snapshotOnlineMask |= (1UL << ch);
                }
            }
        }

        const bool allOnline =
            (onlineMask & requiredMask) == requiredMask &&
            (snapshotOnlineMask & requiredMask) == requiredMask;
        const bool allConfigured =
            (configuredMask & requiredMask) == requiredMask;
        const bool recoveryIdle = (pendingMask & requiredMask) == 0;
        const bool recoveryFailed = (failedMask & requiredMask) != 0;
        const bool ready =
            freshSnapshot && allOnline && allConfigured &&
            recoveryIdle && !recoveryFailed;

        // 只按新的舵机快照累计稳定帧，避免同一帧被 10 ms 循环重复计数。
        if (ready && snapshot.timestamp != lastFeedbackTimestamp) {
            lastFeedbackTimestamp = snapshot.timestamp;
            if (stableSamples < kServoRecoveryStableSamples) {
                stableSamples++;
            }
            if (stableSamples >= kServoRecoveryStableSamples) {
                FW_LOG_INFO_PRINTF(
                    "[AUTO CALIB] SERVO_RECOVERY_READY required=0x%08lX "
                    "online=0x%08lX configured=0x%08lX\r\n",
                    (unsigned long)requiredMask,
                    (unsigned long)onlineMask,
                    (unsigned long)configuredMask);
                return true;
            }
        } else if (!ready) {
            stableSamples = 0;
        }

        if (recoveryFailed ||
            (uint32_t)(nowMs - beginMs) >=
                kServoRecoveryTimeoutMs) {
            const uint32_t unresolvedMask =
                requiredMask &
                (failedMask | pendingMask | ~onlineMask | ~configuredMask);
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_SERVO_RECOVERY_TIMEOUT,
                               CALIB_DIAG_PHASE_INIT,
                               0xFF,
                               0xFF,
                               (int32_t)onlineMask,
                               (int32_t)configuredMask,
                               (int32_t)pendingMask,
                               unresolvedMask);
            FW_LOG_WARN_PRINTF(
                "[AUTO CALIB] SERVO_RECOVERY_FAIL required=0x%08lX "
                "online=0x%08lX configured=0x%08lX pending=0x%08lX "
                "failed=0x%08lX unresolved=0x%08lX\r\n",
                (unsigned long)requiredMask,
                (unsigned long)onlineMask,
                (unsigned long)configuredMask,
                (unsigned long)pendingMask,
                (unsigned long)failedMask,
                (unsigned long)unresolvedMask);
            return false;
        }

        if ((uint32_t)(nowMs - lastLogMs) >=
            kServoRecoveryLogIntervalMs) {
            lastLogMs = nowMs;
            FW_LOG_INFO_PRINTF(
                "[AUTO CALIB] SERVO_RECOVERY_WAIT required=0x%08lX "
                "online=0x%08lX configured=0x%08lX pending=0x%08lX "
                "failed=0x%08lX fresh=%u stable=%u/%u\r\n",
                (unsigned long)requiredMask,
                (unsigned long)onlineMask,
                (unsigned long)configuredMask,
                (unsigned long)pendingMask,
                (unsigned long)failedMask,
                freshSnapshot ? 1U : 0U,
                (unsigned)stableSamples,
                (unsigned)kServoRecoveryStableSamples);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return false;
}

// 标定成功后由舵机通信任务提交本关节的可信状态，避免只改共享位图而驱动仍拒绝多圈 ABS。
static bool commitJointServoMultiTurnTrust(TaskSharedData_t* sharedData,
                                           uint8_t jointIndex)
{
    if (!sharedData) {
        return false;
    }
    const uint32_t requestMask = calibrationJointMotorMask(jointIndex);
    if (requestMask == 0) {
        return false;
    }

    sharedAtomicStore(&sharedData->calibration_trust_request_mask,
                      requestMask);
    sharedAtomicStore(&sharedData->calibration_trust_accepted_mask,
                      (uint32_t)0);
    const uint32_t token =
        sharedAtomicAddFetch(&sharedData->calibration_trust_request_token,
                             (uint32_t)1);
    const uint32_t beginMs = millis();
    while (autoCalibrationStillOwnsControl(sharedData) &&
           (uint32_t)(millis() - beginMs) <
               kTrustCommitTimeoutMs) {
        if (sharedAtomicLoad(&sharedData->calibration_trust_result_token) ==
            token) {
            const uint32_t acceptedMask =
                sharedAtomicLoad(
                    &sharedData->calibration_trust_accepted_mask);
            if ((acceptedMask & requestMask) == requestMask) {
                FW_LOG_INFO_PRINTF(
                    "[AUTO CALIB] MULTITURN_TRUST J%02u mask=0x%08lX "
                    "unknown=0x%08lX\r\n",
                    (unsigned)jointIndex,
                    (unsigned long)requestMask,
                    (unsigned long)sharedAtomicLoad(
                        &sharedData->multi_turn_unknown_bitmap));
                return true;
            }
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    const uint32_t acceptedMask =
        sharedAtomicLoad(&sharedData->calibration_trust_accepted_mask);
    setCalibrationDiag(sharedData,
                       CALIB_DIAG_REASON_SERVO_RECOVERY_TIMEOUT,
                       CALIB_DIAG_PHASE_SEQUENCE,
                       jointIndex,
                       0xFF,
                       (int32_t)token,
                       (int32_t)sharedAtomicLoad(
                           &sharedData->calibration_trust_result_token),
                       (int32_t)acceptedMask,
                       requestMask & ~acceptedMask);
    FW_LOG_WARN_PRINTF(
        "[AUTO CALIB] MULTITURN_TRUST_FAIL J%02u token=%lu "
        "request=0x%08lX accepted=0x%08lX\r\n",
        (unsigned)jointIndex,
        (unsigned long)token,
        (unsigned long)requestMask,
        (unsigned long)acceptedMask);
    return false;
}

// 在发布一批标定目标前检查涉及电机的负载；任一路达到 800 即拒绝整批。
static bool calibrationBatchOverloadSafe(TaskSharedData_t* sharedData,
                                         const ServoTargetBatch_t* batch)
{
    if (!sharedData || !batch) {
        return false;
    }
    for (uint8_t i = 0; i < batch->count; i++) {
        const ServoTargetCommand_t& cmd = batch->commands[i];
        const int ch = findMotorChannel(cmd.busIndex, cmd.servoId);
        if (ch < 0 || ch >= SERVO_TOTAL_NUM || !isMotorChannelValid((uint8_t)ch)) {
            continue;
        }

        int16_t loadRaw = 0;
        if (!readLatestMotorServoLoad(sharedData, (uint8_t)ch, &loadRaw)) {
            continue;
        }
        const int16_t loadMag = mmhand::servo::loadMagnitude(loadRaw);
        if (loadMag >= kOverloadLoadThreshold) {
            const uint32_t bit = 1UL << (uint8_t)ch;
            {
                SharedSafetyGuard guard;
                sharedAtomicFetchOr(&sharedData->overload_fault_bitmap, bit);
                sharedData->overload_fault_load[ch] = loadRaw;
            }
            FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL OVERLOAD M%02d load=%d threshold=%d\r\n",
                          ch,
                          loadMag,
                          kOverloadLoadThreshold);
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_OVERLOAD,
                               CALIB_DIAG_PHASE_SEQUENCE,
                               0xFF,
                               (uint8_t)ch,
                               loadMag,
                               kOverloadLoadThreshold,
                               loadRaw,
                               bit);
            return false;
        }
    }
    return true;
}

// float 四舍五入为 int32，正负数保持对称。
static int32_t roundToInt32Local(float value)
{
    return (int32_t)(value + (value >= 0.0f ? 0.5f : -0.5f));
}

// 计算两个方向坐标间的最短有符号磁编差。
static int32_t signedEncoderDeltaLocal(int32_t fromEncoder, int32_t toEncoder)
{
    return calibrationWrapEncoderDelta(toEncoder, fromEncoder);
}

// 当实际机械正方向与原始方向坐标相反时镜像坐标，统一 LUT 正行程方向。
static int32_t encoderForPositiveTravelResult(int32_t encoder, int8_t travelSign)
{
    if (travelSign >= 0) {
        return calibrationWrapEncoderCount(encoder);
    }
    return calibrationWrapEncoderCount(kCalibrationEncoderModulo - encoder);
}

// 清空一次 LUT 原始采样缓冲。
static void resetTendonRawFitBuffer(TendonRawFitBuffer* buffer)
{
    if (!buffer) {
        return;
    }
    memset(buffer, 0, sizeof(*buffer));
}

// 追加一个去重后的磁编/舵机 ABS 样本；相同三元组不会重复占用容量。
static bool appendTendonRawFitSample(TendonRawFitBuffer* buffer,
                                         int32_t encoder,
                                         int32_t primaryAbs,
                                         int32_t secondaryAbs)
{
    if (!buffer || buffer->count >= kTendonRawSampleCapacity) {
        return false;
    }
    if (buffer->count > 0) {
        TendonRawFitSample& prev = buffer->samples[buffer->count - 1];
        if (prev.encoder == encoder &&
            prev.primaryAbs == primaryAbs &&
            prev.secondaryAbs == secondaryAbs) {
            return true;
        }
    }
    TendonRawFitSample& sample = buffer->samples[buffer->count++];
    sample.encoder = calibrationWrapEncoderCount(encoder);
    sample.primaryAbs = primaryAbs;
    sample.secondaryAbs = secondaryAbs;
    return true;
}

// 当磁编行程跨过下一个 5° 采样点时，记录当前单/双电机 ABS。
static void maybeAppendTendonTravelFitSample(TendonRawFitBuffer* buffer,
                                                 float travelDeg,
                                                 int32_t encoder,
                                                 int32_t primaryAbs,
                                                 int32_t secondaryAbs)
{
    if (!buffer || !isfinite(travelDeg)) {
        return;
    }
    if (travelDeg + 0.1f < buffer->nextTravelDeg) {
        return;
    }
    while (travelDeg + 0.1f >= buffer->nextTravelDeg) {
        buffer->nextTravelDeg += kTendonFitSampleStepDeg;
    }
    appendTendonRawFitSample(buffer, encoder, primaryAbs, secondaryAbs);
}

// 将原始样本的磁编位置转换为 LUT 内部轨迹角。
static float tendonFitSampleAngleDeg(const JointCalibrationResult& result,
                                   int32_t encoder)
{
    if (result.encoderSpan <= 0 || result.totalDeg <= 0.0f) {
        return NAN;
    }
    const int32_t delta =
        calibrationForwardEncoderDelta(result.startEncoder,
                                       calibrationWrapEncoderCount(encoder));
    return result.startDeg +
        ((float)delta / (float)result.encoderSpan) * result.totalDeg;
}

// 在原始样本中按角度线性插值，得到指定执行器的理论 ABS。
static bool interpolateTendonFitAbs(float targetDeg,
                                  const float* sampleDeg,
                                  const int32_t* sampleAbs,
                                  uint8_t sampleCount,
                                  int32_t* outAbs)
{
    if (!sampleDeg || !sampleAbs || !outAbs || sampleCount == 0) {
        return false;
    }
    if (targetDeg <= sampleDeg[0]) {
        *outAbs = sampleAbs[0];
        return true;
    }
    for (uint8_t i = 1; i < sampleCount; i++) {
        if (targetDeg <= sampleDeg[i]) {
            const float span = sampleDeg[i] - sampleDeg[i - 1];
            if (span <= 0.0f) {
                return false;
            }
            const float t = (targetDeg - sampleDeg[i - 1]) / span;
            const float value =
                (float)sampleAbs[i - 1] +
                ((float)sampleAbs[i] - (float)sampleAbs[i - 1]) * t;
            *outAbs = roundToInt32Local(value);
            return true;
        }
    }
    *outAbs = sampleAbs[sampleCount - 1];
    return true;
}

// 单、双腱共用同一套固定两行存储：单电机只写第 0 行，第 1 行保持
// motor=0xFF；双电机同时写两行。LUT 保存的是舵机 raw ABS counts。
// 将不等间隔原始样本重采样为固定 5° 步长 Tendon LUT。
static bool buildTendonLutFromRawSamples(JointCalibrationResult* result,
                                         uint8_t primaryMotor,
                                         uint8_t secondaryMotor,
                                         uint8_t actuatorCount,
                                         const TendonRawFitBuffer& raw)
{
    if (!result || actuatorCount == 0 ||
        actuatorCount > TENDON_LUT_ACTUATOR_COUNT) {
        return false;
    }
    result->tendonLutValid = 0;
    result->tendonLutCount = 0;
    memset(result->tendonLutMotor, 0xFF, sizeof(result->tendonLutMotor));
    memset(result->tendonLutMotorAbs, 0, sizeof(result->tendonLutMotorAbs));
    result->tendonLutAngleStartDeg = result->startDeg;
    result->tendonLutAngleStepDeg = kTendonFitSampleStepDeg;

    if (!result->success || !result->scaled ||
        result->encoderSpan <= 0 ||
        result->totalDeg <= 0.0f ||
        raw.count < 3) {
        return false;
    }

    float sampleDeg[kTendonRawSampleCapacity];
    int32_t primaryAbs[kTendonRawSampleCapacity];
    int32_t secondaryAbs[kTendonRawSampleCapacity];
    uint8_t sampleCount = 0;

    for (uint8_t i = 0; i < raw.count; i++) {
        const float angleDeg =
            tendonFitSampleAngleDeg(*result, raw.samples[i].encoder);
        if (!isfinite(angleDeg) ||
            angleDeg < result->startDeg - kTendonFitSampleStepDeg ||
            angleDeg > result->tightDeg + kTendonFitSampleStepDeg) {
            continue;
        }
        if (sampleCount >= kTendonRawSampleCapacity) {
            continue;
        }

        uint8_t insertAt = sampleCount;
        while (insertAt > 0 && sampleDeg[insertAt - 1] > angleDeg) {
            sampleDeg[insertAt] = sampleDeg[insertAt - 1];
            primaryAbs[insertAt] = primaryAbs[insertAt - 1];
            secondaryAbs[insertAt] = secondaryAbs[insertAt - 1];
            insertAt--;
        }
        if (insertAt > 0 && fabsf(sampleDeg[insertAt - 1] - angleDeg) < 0.2f) {
            sampleDeg[insertAt - 1] = angleDeg;
            primaryAbs[insertAt - 1] = raw.samples[i].primaryAbs;
            secondaryAbs[insertAt - 1] = raw.samples[i].secondaryAbs;
            continue;
        }
        sampleDeg[insertAt] = angleDeg;
        primaryAbs[insertAt] = raw.samples[i].primaryAbs;
        secondaryAbs[insertAt] = raw.samples[i].secondaryAbs;
        sampleCount++;
    }

    if (sampleCount < 3 ||
        sampleDeg[sampleCount - 1] - sampleDeg[0] < kTendonFitSampleStepDeg * 2.0f ||
        sampleDeg[0] > result->startDeg + kTendonFitSampleStepDeg ||
        sampleDeg[sampleCount - 1] < result->tightDeg - kTendonFitSampleStepDeg) {
        return false;
    }

    uint8_t lutCount =
        (uint8_t)ceilf(result->totalDeg / kTendonFitSampleStepDeg) + 1;
    if (lutCount > TENDON_LUT_MAX_SAMPLES) {
        lutCount = TENDON_LUT_MAX_SAMPLES;
    }
    if (lutCount < 3) {
        return false;
    }

    result->tendonLutMotor[0] = primaryMotor;
    if (actuatorCount > 1) {
        result->tendonLutMotor[1] = secondaryMotor;
    }
    result->tendonLutAngleStartDeg = result->startDeg;
    // 固定容量下仍强制包含两个机械端点。行程过长时步长会略大于
    // 5 度，但不会因简单截断而丢失紧端样本。
    result->tendonLutAngleStepDeg =
        result->totalDeg / (float)(lutCount - 1);
    for (uint8_t i = 0; i < lutCount; i++) {
        const float targetDeg =
            result->tendonLutAngleStartDeg +
            result->tendonLutAngleStepDeg * (float)i;
        if (!interpolateTendonFitAbs(targetDeg,
                                     sampleDeg,
                                     primaryAbs,
                                     sampleCount,
                                     &result->tendonLutMotorAbs[0][i]) ||
            (actuatorCount > 1 &&
             !interpolateTendonFitAbs(targetDeg,
                                      sampleDeg,
                                      secondaryAbs,
                                      sampleCount,
                                      &result->tendonLutMotorAbs[1][i]))) {
            result->tendonLutValid = 0;
            result->tendonLutCount = 0;
            return false;
        }
    }
    result->tendonLutCount = lutCount;
    result->tendonLutValid = 1;
    return true;
}

// 由松端/紧端、舵机 ABS 和 Manual 参考构造完整 V3 标定结果。
static bool buildAutoZeroTightCalibrationResultWithRange(uint8_t jointIndex,
                                                         int32_t tautEncoder,
                                                         int32_t zeroEncoder,
                                                         int32_t tightEncoder,
                                                         int32_t tautServoAbs,
                                                         int32_t zeroServoAbs,
                                                         int32_t tightServoAbs,
                                                         float startDeg,
                                                         JointCalibrationResult* out)
{
    (void)zeroEncoder;
    if (!out) {
        return false;
    }

    memset(out, 0, sizeof(*out));

    const int32_t startToTightSpan =
        calibrationForwardEncoderDelta(tautEncoder, tightEncoder);
    const float totalDeg = calibrationEncoderCountToDeg(startToTightSpan);
    if (startToTightSpan <= 0 ||
        !isfinite(totalDeg) ||
        totalDeg < kMinTravelDeg ||
        totalDeg >= kMaxTravelDeg) {
        return false;
    }
    const float tightDeg = startDeg + totalDeg;

    const float zeroFraction =
        (kLutZeroDeg - startDeg) / totalDeg;
    if (zeroFraction <= 0.0f || zeroFraction >= 1.0f) {
        return false;
    }

    const int32_t zeroOffsetCount =
        roundToInt32Local((float)startToTightSpan * zeroFraction);
    const int32_t computedZeroEncoder =
        calibrationWrapEncoderCount(tautEncoder + zeroOffsetCount);

    out->success = true;
    out->encoderMin = tautEncoder;
    out->encoderMax = tightEncoder;
    out->offset = computedZeroEncoder;
    out->angleMax = tightDeg;
    out->startEncoder = tautEncoder;
    out->tightEncoder = tightEncoder;
    out->zeroEncoder = computedZeroEncoder;
    out->encoderSpan = startToTightSpan;
    out->startServoAbs = tautServoAbs;
    out->tightServoAbs = tightServoAbs;
    out->zeroServoAbs = zeroServoAbs;
    out->startDeg = startDeg;
    out->tightDeg = tightDeg;
    out->totalDeg = totalDeg;
    out->scaled = true;

    // 上面的历史标定字段使用按关节方向统一后的磁编坐标；V3 的
    // autoMinRaw/autoMaxRaw 属于通信与 NVS 契约，必须保存物理 14-bit raw。
    // calibrationOrientEncoderRaw() 在 direction=-1 时是自逆变换，因此这里
    // 再调用一次即可还原物理 raw，同时不会改变 direction=+1 的关节。
    const int8_t encoderDirection = g_encoderDirection[jointIndex];
    const uint16_t autoMinRaw = (uint16_t)calibrationOrientEncoderRaw(
        (uint16_t)calibrationWrapEncoderCount(tautEncoder), encoderDirection);
    const uint16_t autoMaxRaw = (uint16_t)calibrationOrientEncoderRaw(
        (uint16_t)calibrationWrapEncoderCount(tightEncoder), encoderDirection);
    if (!applyManualScaledUrdfMappingToCalibration(
            jointIndex,
            g_manualEncoderReference[jointIndex],
            autoMinRaw,
            autoMaxRaw,
            encoderDirection,
            manualEncoderReferenceEntryCrc(
                jointIndex, g_manualEncoderReference[jointIndex]),
            out)) {
        memset(out, 0, sizeof(*out));
        return false;
    }
    return true;
}

// 单电机关节结果构造包装：使用当前 URDF 上限和 Manual 条目。
static bool buildAutoZeroTightCalibrationResult(uint8_t jointIndex,
                                                int32_t tautEncoder,
                                                int32_t zeroEncoder,
                                                int32_t tightEncoder,
                                                int32_t tautServoAbs,
                                                int32_t zeroServoAbs,
                                                int32_t tightServoAbs,
                                                JointCalibrationResult* out)
{
    return buildAutoZeroTightCalibrationResultWithRange(jointIndex,
                                                        tautEncoder,
                                                        zeroEncoder,
                                                        tightEncoder,
                                                        tautServoAbs,
                                                        zeroServoAbs,
                                                        tightServoAbs,
                                                        kLutStartDeg,
                                                        out);
}

// 双腱关节结果构造包装：两路舵机共享同一磁编 Auto Min/Max。
static bool buildDualZeroTightCalibrationResult(uint8_t jointIndex,
                                                int32_t startEncoder,
                                                int32_t zeroEncoder,
                                                int32_t tightEncoder,
                                                int32_t startServoAbs,
                                                int32_t zeroServoAbs,
                                                int32_t tightServoAbs,
                                                JointCalibrationResult* out)
{
    return buildAutoZeroTightCalibrationResultWithRange(jointIndex,
                                                        startEncoder,
                                                        zeroEncoder,
                                                        tightEncoder,
                                                        startServoAbs,
                                                        zeroServoAbs,
                                                        tightServoAbs,
                                                        kLutStartDeg,
                                                        out);
}

// 从 active_joint_mask 生成本轮关节列表，并拒绝缺失 Manual 的活动关节。
static uint8_t buildAutoCalibrationJointListFromActiveScope(TaskSharedData_t* sharedData,
                                                            uint8_t* outJoints,
                                                            uint8_t maxCount)
{
    if (!sharedData || !outJoints || maxCount == 0) {
        return 0;
    }

    uint32_t activeMask = 0;
    uint32_t selectedActiveMotorMask = 0;
    {
        SharedSafetyGuard guard;
        activeMask = sharedAtomicLoad(&sharedData->active_joint_mask) &
                     ((1UL << ENCODER_TOTAL_NUM) - 1UL);
        selectedActiveMotorMask = sharedAtomicLoad(&sharedData->active_motor_mask);
    }
    if (activeMask == 0 && selectedActiveMotorMask != 0) {
        const uint32_t activeMotorMask =
            selectedActiveMotorMask & validMotorChannelMask();
        for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++) {
            JointActuatorGroup group;
            if (!getJointActuatorGroup(jointIndex, &group)) {
                continue;
            }
            for (uint8_t i = 0; i < group.count; i++) {
                const uint8_t motorChannel = group.actuators[i].motorChannel;
                if (motorChannel < SERVO_TOTAL_NUM &&
                    (activeMotorMask & (1UL << motorChannel)) != 0) {
                    activeMask |= (1UL << jointIndex);
                    break;
                }
            }
        }

        if (activeMask != 0) {
            FW_LOG_INFO_PRINTF("[AUTO CALIB] ACTIVE_SCOPE inferred joints=0x%08lX from motor=0x%08lX\r\n",
                          (unsigned long)activeMask,
                          (unsigned long)activeMotorMask);
        }
    }

    uint8_t count = 0;
    for (uint8_t jointIndex = 0;
         jointIndex < ENCODER_TOTAL_NUM && count < maxCount;
         jointIndex++) {
        if ((activeMask & (1UL << jointIndex)) == 0) {
            continue;
        }
        outJoints[count++] = jointIndex;
    }

    return count;
}

// 原子更新某关节目标角和命令 token，使标定完成后的 UI/PID 目标同步到实测角。
static void syncJointTargetToDeg(TaskSharedData_t* sharedData,
                                 uint8_t jointIndex,
                                 float targetDeg)
{
    if (!sharedData || jointIndex >= ENCODER_TOTAL_NUM) {
        return;
    }

    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }

    sharedData->targetAngles[jointIndex] = targetDeg;
    sharedData->joint_command_token++;

    if (lock) {
        xSemaphoreGive(lock);
    }
}

// 构造默认标定参数；所有单/双电机状态机共享这组周期和停稳判据。
AutoCalibrationParams defaultAutoCalibrationParams(void)
{
    AutoCalibrationParams params;
    params.tightenSpeedCountsPerSec = 400;
    params.servoSpeed = 400;
    params.servoAcc = 20;
    params.loopMs = 20;
    params.windowMs = 200;
    params.traceMs = 200;
    params.windowServoDeltaCounts = 50;
    params.zeroReachAbsTolerance = 200;
    params.stallActualDeg = 0.75f;
    params.startLoadDelta = 25;
    params.tightLoadHigh = 250;
    params.startLoadStableSamples = 3;
    return params;
}

// 单电机关节的非阻塞标定状态。
enum AutoCalibState {
    AUTO_CALIB_STATE_IDLE = 0,
    AUTO_CALIB_STATE_INIT,
    AUTO_CALIB_STATE_TIGHTEN,
    AUTO_CALIB_STATE_PRE_RELEASE_SYNC,
    AUTO_CALIB_STATE_DONE,
    AUTO_CALIB_STATE_FAILED
};

// 单电机标定上下文。所有跨帧变量均集中在这里，状态机每 20 ms 推进一步。
struct JointCalibContext {
    AutoCalibState state;       // 当前状态。
    uint8_t jointIndex;         // J00~J20。
    uint8_t bus;                // 主舵机总线。
    uint8_t servoId;            // 主舵机总线 ID。
    uint8_t motorChannel;       // 扁平化 Mxx 通道。
    int32_t directionSign;      // 舵机收绳时 ABS 增减方向。
    int8_t encoderTravelSign;   // 本轮磁编实测行程方向。
    uint16_t commandSpeed;      // 舵机命令速度。
    uint8_t commandAcc;         // 舵机命令加速度。
    uint32_t stateBeginMs;      // 当前状态开始时间。
    uint32_t lastCommandMs;     // 上次推进目标时间。
    uint32_t lastTraceMs;       // 上次低频日志时间。
    int32_t commandAbs;         // 当前逐步推进的命令 ABS。
    int32_t commandRemainder;   // counts/s 换算到单帧后的余数累积。
    int32_t targetAbs;          // 当前阶段最终目标 ABS。
    int16_t baselineLoad;       // 起始负载基线。
    bool startFound;            // 紧端是否找到。
    bool zeroFound;             // 内部零点是否找到。
    uint8_t startLoadStable;    // 紧端负载连续稳定计数。
    uint16_t startSearchRaw;    // 搜索开始时磁编 raw。
    int32_t startSearchEncoder; // 搜索开始时方向坐标。
    int32_t startSearchAbs;     // 搜索开始时舵机 ABS。
    int32_t startEncoder;       // 找到紧端后的磁编方向坐标。
    int32_t startServoAbs;      // 找到紧端后的舵机 ABS。
    int32_t zeroEncoder;        // 找到 Auto Min 时的磁编方向坐标。
    int32_t zeroServoAbs;       // 找到 Auto Min 时的舵机 ABS。
    int32_t windowStartEncoder; // 机械停稳窗口起始磁编坐标。
    int32_t windowStartAbs;     // 机械停稳窗口起始舵机 ABS。
    uint32_t windowStartMs;     // 机械停稳窗口起始时间。
    JointCalibrationResult result; // 本轮待提交结果。
    bool resultValid;           // result 是否已通过全部校验。
    bool initStarted;           // 初始反馈等待是否已启动。
    bool releaseCommandStarted; // 松绳阶段是否已发出首帧命令。
    int32_t preReleaseSyncTargetAbs; // 松绳前锁定的当前 ABS。
    int32_t preReleaseSyncWindowStartAbs; // 同步窗口起始 ABS。
    uint32_t preReleaseSyncWindowStartMs; // 同步窗口起始时间。
    uint8_t preReleaseSyncStableSamples;  // 同步连续稳定帧数。
    uint8_t releaseAngleStallStable;      // 松端角度停稳连续帧数。
    TendonRawFitBuffer fitSamples;        // 本轮 Tendon LUT 原始样本。
};

// 向批次追加或覆盖同一 bus/id 的目标，保证每帧每台舵机只有一条命令。
static bool appendServoTargetLocal(ServoTargetBatch_t* batch,
                                   uint8_t busIndex,
                                   uint8_t servoId,
                                   int32_t position,
                                   uint16_t speed,
                                   uint8_t acc)
{
    if (!batch) {
        return false;
    }
    for (uint8_t i = 0; i < batch->count; i++) {
        ServoTargetCommand_t& existing = batch->commands[i];
        if (existing.busIndex == busIndex && existing.servoId == servoId) {
            existing.position = position;
            existing.speed = speed;
            existing.acc = acc;
            return true;
        }
    }
    if (batch->count >= SERVO_TARGET_BATCH_MAX) {
        return false;
    }
    ServoTargetCommand_t& cmd = batch->commands[batch->count++];
    cmd.busIndex = busIndex;
    cmd.servoId = servoId;
    cmd.position = position;
    cmd.speed = speed;
    cmd.acc = acc;
    return true;
}

// 将单电机关节的标定目标写入批次。
// 双腱关节由专用状态机显式生成两路目标，不能在这里叠加旧主从映射。
static bool appendJointCalibrationTargetToBatch(ServoTargetBatch_t* batch,
                                                uint8_t jointIndex,
                                                int32_t primaryTarget,
                                                uint16_t speed,
                                                uint8_t acc)
{
    if (!batch || jointIndex >= ENCODER_TOTAL_NUM) {
        return false;
    }
    return appendServoTargetLocal(batch,
                                  jointMap[jointIndex].busIndex,
                                  jointMap[jointIndex].servoID,
                                  primaryTarget,
                                  speed,
                                  acc);
}

// 发布 CALIBRATION owner 的目标批次；发布前再次执行负载硬保护。
static bool publishCalibrationBatch(TaskSharedData_t* sharedData, ServoTargetBatch_t* batch)
{
    if (!sharedData || !batch || batch->count == 0) {
        return true;
    }
    if (!sharedData->servoTargetQueue ||
        sharedData->servo_target_owner != SERVO_TARGET_OWNER_CALIBRATION) {
        return false;
    }
    if (!calibrationBatchOverloadSafe(sharedData, batch)) {
        return false;
    }
    batch->source = SERVO_TARGET_OWNER_CALIBRATION;
    batch->timestamp = millis();
    return p4::actuator::publishLatest(sharedData, *batch);
}

// 串行标定时给非活动关节追加“保持当前 ABS”，防止共享绳路被当前动作带动。
static bool appendCalibrationHoldTargetsExcept(TaskSharedData_t* sharedData,
                                               ServoTargetBatch_t* batch,
                                               const uint8_t* jointList,
                                               uint8_t jointCount,
                                               uint8_t activeJoint,
                                               uint32_t skipJointMask,
                                               const AutoCalibrationParams& params)
{
    if (!sharedData || !batch || !jointList) {
        return false;
    }

    for (uint8_t i = 0; i < jointCount; i++) {
        const uint8_t jointIndex = jointList[i];
        if (jointIndex == activeJoint || jointIndex >= ENCODER_TOTAL_NUM ||
            ((skipJointMask & (1UL << jointIndex)) != 0)) {
            continue;
        }

        JointActuatorGroup group;
        if (getJointActuatorGroup(jointIndex, &group)) {
            for (uint8_t actuatorIndex = 0; actuatorIndex < group.count; actuatorIndex++) {
                const uint8_t motorChannel = group.actuators[actuatorIndex].motorChannel;
                int32_t holdAbs = 0;
                if (!readLatestMotorServoAbs(sharedData, motorChannel, &holdAbs)) {
                    FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL hold feedback missing J%02u M%02u\r\n",
                                  jointIndex,
                                  motorChannel);
                    return false;
                }
                const MotorMapItem& map = motorMap[motorChannel];
                if (!appendServoTargetLocal(batch,
                                            map.busIndex,
                                            map.servoID,
                                            clampToServoPosLocal(holdAbs),
                                            params.servoSpeed,
                                            params.servoAcc)) {
                    FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL append hold target J%02u M%02u abs=%ld\r\n",
                                  jointIndex,
                                  motorChannel,
                                  (long)holdAbs);
                    return false;
                }
            }
        } else {
            const uint8_t bus = jointMap[jointIndex].busIndex;
            const uint8_t id = jointMap[jointIndex].servoID;
            int32_t holdAbs = 0;
            if (!readLatestServoAbs(sharedData, bus, id, &holdAbs)) {
                FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL hold feedback missing J%02u bus%u id%u\r\n",
                              jointIndex,
                              bus,
                              id);
                return false;
            }
            if (!appendServoTargetLocal(batch,
                                        bus,
                                        id,
                                        clampToServoPosLocal(holdAbs),
                                        params.servoSpeed,
                                        params.servoAcc)) {
                FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL append hold target J%02u abs=%ld\r\n",
                              jointIndex,
                              (long)holdAbs);
                return false;
            }
        }
    }

    return true;
}

// 从 HardwareMap 读取双腱关节的主/副电机 Mxx；单电机关节返回 false。
static bool getDualAntagonistAutoCalibrationMotors(uint8_t jointIndex,
                                                   uint8_t* primaryMotor,
                                                   uint8_t* secondaryMotor)
{
    // 所有由 jointActuatorGroups 描述的双执行器关节都走拮抗双腱标定流程，
    // 包括拇指 J16/J20。
    if (!primaryMotor || !secondaryMotor || jointIndex >= ENCODER_TOTAL_NUM) {
        return false;
    }

    JointActuatorGroup group;
    if (!getJointActuatorGroup(jointIndex, &group) || group.count < 2) {
        return false;
    }
    if (group.actuators[0].sign <= 0 || group.actuators[1].sign >= 0) {
        return false;
    }
    if (!isMotorChannelValid(group.actuators[0].motorChannel) ||
        !isMotorChannelValid(group.actuators[1].motorChannel)) {
        return false;
    }

    *primaryMotor = group.actuators[0].motorChannel;
    *secondaryMotor = group.actuators[1].motorChannel;
    return true;
}

// 按 Mxx 将一个多圈 ABS 目标写入标定批次。
static bool appendMotorCalibrationTargetToBatch(ServoTargetBatch_t* batch,
                                                uint8_t motorChannel,
                                                int32_t targetAbs,
                                                const AutoCalibrationParams& params)
{
    if (!batch || !isMotorChannelValid(motorChannel)) {
        return false;
    }
    const MotorMapItem& map = motorMap[motorChannel];
    return appendServoTargetLocal(batch,
                                  map.busIndex,
                                  map.servoID,
                                  clampToServoPosLocal(targetAbs),
                                  params.servoSpeed,
                                  params.servoAcc);
}

// 发布单台电机目标，并可选择同时保持另一台电机。
static bool publishMotorCalibrationTarget(TaskSharedData_t* sharedData,
                                          uint8_t motorChannel,
                                          int32_t targetAbs,
                                          const AutoCalibrationParams& params)
{
    if (!isServoCommandInsideLimit(sharedData, motorChannel, targetAbs)) {
        FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL motor target outside limit M%02u target=%ld\r\n",
                      motorChannel,
                      (long)targetAbs);
        return false;
    }

    ServoTargetBatch_t batch;
    memset(&batch, 0, sizeof(batch));
    batch.source = SERVO_TARGET_OWNER_CALIBRATION;
    batch.timestamp = millis();
    if (!appendMotorCalibrationTargetToBatch(&batch, motorChannel, targetAbs, params)) {
        return false;
    }
    return publishCalibrationBatch(sharedData, &batch);
}

// 显式发布标定目标：目标值归轨迹生成逻辑所有，不能在这里用反馈值重写。
// 直接发布单台电机目标，不附带另一电机保持命令。
static bool publishMotorCalibrationTargetDirect(TaskSharedData_t* sharedData,
                                                uint8_t motorChannel,
                                                int32_t targetAbs,
                                                const AutoCalibrationParams& params)
{
    if (!isServoCommandInsideLimit(sharedData, motorChannel, targetAbs)) {
        FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL motor target outside limit M%02u target=%ld\r\n",
                      motorChannel,
                      (long)targetAbs);
        return false;
    }

    ServoTargetBatch_t batch;
    memset(&batch, 0, sizeof(batch));
    batch.source = SERVO_TARGET_OWNER_CALIBRATION;
    batch.timestamp = millis();
    if (!appendMotorCalibrationTargetToBatch(&batch, motorChannel, targetAbs, params)) {
        return false;
    }
    return publishCalibrationBatch(sharedData, &batch);
}

// 双电机目标发布包装：只移动 movingMotor，可选固定 holdMotor。
static bool publishDualMotorCalibrationTargets(TaskSharedData_t* sharedData,
                                               uint8_t movingMotor,
                                               int32_t movingTargetAbs,
                                               uint8_t holdMotor,
                                               int32_t holdTargetAbs,
                                               const AutoCalibrationParams& params);

// 双电机目标发布实现，统一检查范围、owner、负载和批次数量。
static bool publishDualMotorCalibrationTargets(TaskSharedData_t* sharedData,
                                               uint8_t movingMotor,
                                               int32_t movingTargetAbs,
                                               uint8_t holdMotor,
                                               int32_t holdTargetAbs,
                                               const AutoCalibrationParams& params)
{
    if (!isServoCommandInsideLimit(sharedData,
                                   movingMotor,
                                   movingTargetAbs) ||
        !isServoCommandInsideLimit(sharedData,
                                   holdMotor,
                                   holdTargetAbs)) {
        FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL dual target outside limit moving=M%02u target=%ld hold=M%02u target=%ld\r\n",
                      movingMotor,
                      (long)movingTargetAbs,
                      holdMotor,
                      (long)holdTargetAbs);
        return false;
    }

    ServoTargetBatch_t batch;
    memset(&batch, 0, sizeof(batch));
    batch.source = SERVO_TARGET_OWNER_CALIBRATION;
    batch.timestamp = millis();
    if (!appendMotorCalibrationTargetToBatch(&batch, movingMotor, movingTargetAbs, params)) {
        return false;
    }
    if (holdMotor != movingMotor &&
        !appendMotorCalibrationTargetToBatch(&batch, holdMotor, holdTargetAbs, params)) {
        return false;
    }
    return publishCalibrationBatch(sharedData, &batch);
}

// 等待一台电机到达目标容差并停稳；超时或 owner 丢失时返回 false。
static bool waitMotorTargetReached(TaskSharedData_t* sharedData,
                                   uint8_t jointIndex,
                                   const char* phase,
                                   uint8_t motorChannel,
                                   int32_t targetAbs,
                                   const AutoCalibrationParams& params)
{
    uint32_t lastTraceMs = 0;
    while (autoCalibrationStillOwnsControl(sharedData)) {
        // 此辅助函数只服务于 DUAL_SLACK_SECONDARY。放绳 3200 counts 是
        // 标定状态机生成的有明确终点的轨迹，不允许在此继续扩展目标。
        if (!publishMotorCalibrationTargetDirect(sharedData,
                                                 motorChannel,
                                                 targetAbs,
                                                 params)) {
            return false;
        }

        int32_t currentAbs = 0;
        if (readLatestMotorServoAbs(sharedData, motorChannel, &currentAbs)) {
            const uint32_t nowMs = millis();
            const int32_t absErr = abs32(currentAbs - targetAbs);
            if (absErr <= params.zeroReachAbsTolerance) {
                FW_LOG_INFO_PRINTF("[AUTO CALIB] %s_REACHED J%02u M%02u targetAbs=%ld currentAbs=%ld absErr=%ld\r\n",
                              phase,
                              jointIndex,
                              motorChannel,
                              (long)targetAbs,
                              (long)currentAbs,
                              (long)absErr);
                return true;
            }

            if ((uint32_t)(nowMs - lastTraceMs) >= params.traceMs) {
                lastTraceMs = nowMs;
                FW_LOG_INFO_PRINTF("[AUTO CALIB] %s_WAIT J%02u M%02u targetAbs=%ld currentAbs=%ld absErr=%ld/%ld\r\n",
                              phase,
                              jointIndex,
                              motorChannel,
                              (long)targetAbs,
                              (long)currentAbs,
                              (long)absErr,
                              (long)params.zeroReachAbsTolerance);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(params.loopMs));
    }
    return false;
}

// 双腱标定开始前等待磁编、两路 ABS 和在线状态同时有效。
static bool waitDualInitialFeedback(TaskSharedData_t* sharedData,
                                    uint8_t jointIndex,
                                    uint8_t primaryMotor,
                                    uint8_t secondaryMotor,
                                    int32_t* primaryAbs,
                                    int32_t* secondaryAbs,
                                    uint16_t* encoderRaw,
                                    int32_t* encoderOriented,
                                    const AutoCalibrationParams& params)
{
    (void)params;
    uint32_t lastTraceMs = 0;
    while (autoCalibrationStillOwnsControl(sharedData)) {
        const bool primaryOk =
            readLatestMotorServoAbs(sharedData, primaryMotor, primaryAbs);
        const bool secondaryOk =
            readLatestMotorServoAbs(sharedData, secondaryMotor, secondaryAbs);
        const bool encoderOk =
            readLatestEncoderSample(sharedData, jointIndex, encoderRaw, encoderOriented);
        if (primaryOk && secondaryOk && encoderOk) {
            return true;
        }

        const uint32_t nowMs = millis();
        if ((uint32_t)(nowMs - lastTraceMs) >= 200) {
            lastTraceMs = nowMs;
            FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_INIT_WAIT J%02u M%02u=%u M%02u=%u encoder=%u\r\n",
                          jointIndex,
                          primaryMotor,
                          primaryOk ? 1 : 0,
                          secondaryMotor,
                          secondaryOk ? 1 : 0,
                          encoderOk ? 1 : 0);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return false;
}

// 沿收绳方向逐步推进指定电机，直到负载阈值或机械角停稳条件成立。
static bool findDualTightPoint(TaskSharedData_t* sharedData,
                               uint8_t jointIndex,
                               const char* phase,
                               uint8_t motorChannel,
                               int8_t motorShortenSign,
                               int32_t startAbs,
                               int32_t startEncoder,
                               int16_t loadThreshold,
                               bool holdOtherMotor,
                               uint8_t holdMotorChannel,
                               int32_t holdMotorAbs,
                               const AutoCalibrationParams& params,
                               int32_t* outTightAbs,
                               uint16_t* outTightRaw,
                               int32_t* outTightEncoder,
                               int16_t* outTightLoad)
{
    int32_t commandAbs = startAbs;
    int32_t commandRemainder = 0;
    uint32_t lastCommandMs = millis();
    uint32_t lastTraceMs = 0;
    uint32_t windowStartMs = millis();
    int32_t windowStartAbs = startAbs;
    int32_t windowStartEncoder = startEncoder;
    uint8_t angleStallStable = 0;

    while (autoCalibrationStillOwnsControl(sharedData)) {
        const uint32_t nowMs = millis();
        uint16_t currentRaw = 0;
        int32_t currentEncoder = 0;
        int32_t currentAbs = 0;
        if (!readLatestEncoderSample(sharedData, jointIndex, &currentRaw, &currentEncoder) ||
            !readLatestMotorServoAbs(sharedData, motorChannel, &currentAbs)) {
            FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL %s lost feedback J%02u M%02u\r\n",
                          phase,
                          jointIndex,
                          motorChannel);
            return false;
        }

        int16_t loadRaw = 0;
        const bool hasLoad = readLatestMotorServoLoad(sharedData, motorChannel, &loadRaw);
        const int16_t loadMag = hasLoad ? mmhand::servo::loadMagnitude(loadRaw) : 0;
        const int32_t totalAbsDelta = abs32(currentAbs - startAbs);
        const int32_t totalEncoderDelta =
            calibrationWrapEncoderDelta(currentEncoder, startEncoder);
        const float totalAngleDeltaDeg =
            fabsf(calibrationEncoderCountToDeg(totalEncoderDelta));
        const bool loadHigh = hasLoad && loadMag >= loadThreshold;
        bool tightFoundByStall = false;

        if ((uint32_t)(nowMs - windowStartMs) >= params.windowMs) {
            const int32_t windowAbsDelta = abs32(currentAbs - windowStartAbs);
            const int32_t windowEncoderDelta =
                calibrationWrapEncoderDelta(currentEncoder, windowStartEncoder);
            const float windowAngleDeltaDeg =
                fabsf(calibrationEncoderCountToDeg(windowEncoderDelta));
            const bool servoStillTightening =
                windowAbsDelta >= params.windowServoDeltaCounts;
            const bool angleStalled =
                windowAngleDeltaDeg <= params.stallActualDeg;
            if (angleStalled) {
                if (angleStallStable < 255) {
                    angleStallStable++;
                }
            } else {
                angleStallStable = 0;
            }

            FW_LOG_INFO_PRINTF("[AUTO CALIB] CHECK_%s J%02u M%02u abs=%ld raw=%u oriented=%ld dCount=%ld winCount=%ld winAbs=%ld/%ld winDeg=%.3f/%.3f totalDeg=%.3f load=%d/%d cond servo=%u angle=%u angleStable=%u/%u load=%u hold=%u\r\n",
                          phase,
                          jointIndex,
                          motorChannel,
                          (long)currentAbs,
                          (unsigned)currentRaw,
                          (long)currentEncoder,
                          (long)totalEncoderDelta,
                          (long)windowEncoderDelta,
                          (long)windowAbsDelta,
                          (long)params.windowServoDeltaCounts,
                          windowAngleDeltaDeg,
                          params.stallActualDeg,
                          totalAngleDeltaDeg,
                          loadMag,
                          loadThreshold,
                          servoStillTightening ? 1 : 0,
                          angleStalled ? 1 : 0,
                          (unsigned)angleStallStable,
                          (unsigned)kAngleStallStableSamples,
                          loadHigh ? 1 : 0,
                          holdOtherMotor ? 1 : 0);

            tightFoundByStall =
                loadHigh &&
                servoStillTightening &&
                angleStallStable >= kAngleStallStableSamples;
            windowStartAbs = currentAbs;
            windowStartEncoder = currentEncoder;
            windowStartMs = nowMs;
        }

        if (tightFoundByStall) {
            if (outTightAbs) {
                *outTightAbs = currentAbs;
            }
            if (outTightRaw) {
                *outTightRaw = currentRaw;
            }
            if (outTightEncoder) {
                *outTightEncoder = currentEncoder;
            }
            if (outTightLoad) {
                *outTightLoad = loadMag;
            }
            FW_LOG_INFO_PRINTF("[AUTO CALIB] %s_FOUND J%02u M%02u raw=%u oriented=%ld abs=%ld shortenSign=%d load=%d byStall=%u totalDeg=%.3f loadThreshold=%d hold=%u\r\n",
                          phase,
                          jointIndex,
                          motorChannel,
                          (unsigned)currentRaw,
                          (long)currentEncoder,
                          (long)currentAbs,
                          (int)motorShortenSign,
                          loadMag,
                          tightFoundByStall ? 1 : 0,
                          totalAngleDeltaDeg,
                          loadThreshold,
                          holdOtherMotor ? 1 : 0);
            if (holdOtherMotor) {
                if (!publishDualMotorCalibrationTargets(sharedData,
                                                        motorChannel,
                                                        currentAbs,
                                                        holdMotorChannel,
                                                        holdMotorAbs,
                                                        params)) {
                    return false;
                }
            } else if (!publishMotorCalibrationTarget(sharedData, motorChannel, currentAbs, params)) {
                return false;
            }
            return true;
        }

        const uint32_t dtMs = nowMs - lastCommandMs;
        if (dtMs >= params.loopMs) {
            const int32_t numerator =
                (int32_t)dtMs * params.tightenSpeedCountsPerSec + commandRemainder;
            const int32_t stepCounts = numerator / 1000;
            commandRemainder = numerator % 1000;
            if (stepCounts > 0) {
                commandAbs = motorApplyTendonDelta(motorChannel, commandAbs, stepCounts);
                if (holdOtherMotor) {
                    if (!publishDualMotorCalibrationTargets(sharedData,
                                                            motorChannel,
                                                            commandAbs,
                                                            holdMotorChannel,
                                                            holdMotorAbs,
                                                            params)) {
                        return false;
                    }
                } else {
                    if (!publishMotorCalibrationTarget(sharedData, motorChannel, commandAbs, params)) {
                        return false;
                    }
                }
            }
            lastCommandMs = nowMs;
        }

        if ((uint32_t)(nowMs - lastTraceMs) >= params.traceMs) {
            lastTraceMs = nowMs;
            FW_LOG_INFO_PRINTF("[AUTO CALIB] TRACE_%s J%02u M%02u cmd=%ld abs=%ld shortenSign=%d raw=%u oriented=%ld dCount=%ld totalAbs=%ld totalDeg=%.3f load=%d\r\n",
                          phase,
                          jointIndex,
                          motorChannel,
                          (long)commandAbs,
                          (long)currentAbs,
                          (int)motorShortenSign,
                          (unsigned)currentRaw,
                          (long)currentEncoder,
                          (long)totalEncoderDelta,
                          (long)totalAbsDelta,
                          totalAngleDeltaDeg,
                          loadMag);
        }

        vTaskDelay(pdMS_TO_TICKS(params.loopMs));
    }
    FW_LOG_INFO_PRINTF("[AUTO CALIB] CANCEL %s J%02u M%02u state=%u owner=%u enabled=%u\r\n",
                  phase,
                  jointIndex,
                  motorChannel,
                  sharedData ? (unsigned)sharedData->system_state : 255,
                  sharedData ? (unsigned)sharedData->servo_target_owner : 255,
                  sharedData ? (unsigned)sharedData->control_enabled : 255);
    return false;
}

// 把 J00/J04/J08/J12 映射到工作空间缓存索引。
static int8_t workspaceCacheIndex(uint8_t jointIndex)
{
    for (uint8_t i = 0; i < kWorkspaceDualJointsCount; ++i) {
        if (kWorkspaceDualJoints[i] == jointIndex) {
            return (int8_t)i;
        }
    }
    return -1;
}

// 判断关节是否属于四指侧摆工作空间规划集合。
static bool isWorkspaceDualJoint(uint8_t jointIndex)
{
    return workspaceCacheIndex(jointIndex) >= 0;
}

// 每轮标定开始时清空 RAM 姿态缓存，禁止跨任务复用旧机械姿态。
static void clearDualWorkspaceCache(void)
{
    memset(g_dualWorkspaceCache, 0, sizeof(g_dualWorkspaceCache));
}

// 一对辅助双腱执行器在预检阶段的可用性分类。
enum DualWorkspacePairAvailability : uint8_t {
    DUAL_WORKSPACE_PAIR_READY = 0,
    DUAL_WORKSPACE_PAIR_ALL_OFFLINE,
    DUAL_WORKSPACE_PAIR_PARTIAL_ONLINE,
    DUAL_WORKSPACE_PAIR_TURN_UNKNOWN,
    DUAL_WORKSPACE_PAIR_FEEDBACK_STALE,
    DUAL_WORKSPACE_PAIR_MAP_INVALID,
};

// 检查辅助关节两台电机是否全在线、反馈新鲜且多圈可信。
static DualWorkspacePairAvailability classifyDualWorkspacePair(
    TaskSharedData_t* sharedData,
    uint8_t jointIndex,
    uint8_t* outPrimaryMotor,
    uint8_t* outSecondaryMotor,
    uint32_t* outMotorMask)
{
    uint8_t primaryMotor = 0;
    uint8_t secondaryMotor = 0;
    if (!sharedData ||
        !getDualAntagonistAutoCalibrationMotors(jointIndex,
                                                &primaryMotor,
                                                &secondaryMotor)) {
        return DUAL_WORKSPACE_PAIR_MAP_INVALID;
    }
    const uint32_t motorMask =
        (1UL << primaryMotor) | (1UL << secondaryMotor);
    if (outPrimaryMotor) *outPrimaryMotor = primaryMotor;
    if (outSecondaryMotor) *outSecondaryMotor = secondaryMotor;
    if (outMotorMask) *outMotorMask = motorMask;

    ServoAngleData_t servoData;
    if (!sharedData->servoFeedbackQueue ||
        xQueuePeek(sharedData->servoFeedbackQueue, &servoData, 0) != pdTRUE ||
        (uint32_t)(millis() - servoData.timestamp) >
            kFeedbackReadyServoTimeoutMs) {
        return DUAL_WORKSPACE_PAIR_FEEDBACK_STALE;
    }

    const bool primaryOnline = servoData.onlineStatus[primaryMotor] != 0;
    const bool secondaryOnline = servoData.onlineStatus[secondaryMotor] != 0;
    if (!primaryOnline && !secondaryOnline) {
        return DUAL_WORKSPACE_PAIR_ALL_OFFLINE;
    }
    if (primaryOnline != secondaryOnline) {
        return DUAL_WORKSPACE_PAIR_PARTIAL_ONLINE;
    }
    if ((sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap) & motorMask) != 0) {
        return DUAL_WORKSPACE_PAIR_TURN_UNKNOWN;
    }
    return DUAL_WORKSPACE_PAIR_READY;
}

// 将工作空间可用性枚举转换为日志文本。
static const char* workspacePairAvailabilityText(
    DualWorkspacePairAvailability availability)
{
    switch (availability) {
        case DUAL_WORKSPACE_PAIR_READY: return "ready";
        case DUAL_WORKSPACE_PAIR_ALL_OFFLINE: return "all-offline";
        case DUAL_WORKSPACE_PAIR_PARTIAL_ONLINE: return "partial-online";
        case DUAL_WORKSPACE_PAIR_TURN_UNKNOWN: return "turn-unknown";
        case DUAL_WORKSPACE_PAIR_FEEDBACK_STALE: return "feedback-stale";
        default: return "map-invalid";
    }
}

// 只把本轮目标关节和真正参与让位的在线辅助电机加入标定读取范围。
static bool buildDynamicWorkspaceMotorMask(
    TaskSharedData_t* sharedData,
    const uint8_t* activeJoints,
    uint8_t activeJointCount,
    uint32_t* outMotorMask)
{
    if (!sharedData || !activeJoints || !outMotorMask) return false;
    uint32_t motorMask = 0;
    for (uint8_t active = 0; active < activeJointCount; ++active) {
        const uint8_t targetJoint = activeJoints[active];
        if (!isWorkspaceDualJoint(targetJoint)) continue;
        for (uint8_t i = 0; i < kWorkspaceDualJointsCount; ++i) {
            const uint8_t candidateJoint = kWorkspaceDualJoints[i];
            uint8_t primaryMotor = 0;
            uint8_t secondaryMotor = 0;
            uint32_t pairMask = 0;
            const DualWorkspacePairAvailability availability =
                classifyDualWorkspacePair(sharedData,
                                          candidateJoint,
                                          &primaryMotor,
                                          &secondaryMotor,
                                          &pairMask);
            if (availability == DUAL_WORKSPACE_PAIR_READY) {
                motorMask |= pairMask;
                continue;
            }
            // 目标关节允许通过自动标定重建 Turn Unknown；但前提是热插拔恢复和多圈模式配置已经完成。
            const bool targetCanRebuildTurn =
                candidateJoint == targetJoint &&
                availability == DUAL_WORKSPACE_PAIR_TURN_UNKNOWN &&
                (sharedAtomicLoad(
                    &sharedData->servo_multiturn_configured_bitmap) &
                 pairMask) == pairMask &&
                (sharedAtomicLoad(
                    &sharedData->servo_recovery_pending_bitmap) &
                 pairMask) == 0 &&
                (sharedAtomicLoad(
                    &sharedData->servo_recovery_failed_bitmap) &
                 pairMask) == 0;
            if (targetCanRebuildTurn) {
                motorMask |= pairMask;
                FW_LOG_INFO_PRINTF(
                    "[AUTO CALIB] WORKSPACE_MASK_TARGET_REBUILD "
                    "target=J%02u mask=0x%08lX\r\n",
                    (unsigned)targetJoint,
                    (unsigned long)pairMask);
                continue;
            }
            if (candidateJoint != targetJoint &&
                availability == DUAL_WORKSPACE_PAIR_ALL_OFFLINE) {
                FW_LOG_INFO_PRINTF(
                    "[AUTO CALIB] WORKSPACE_MASK_SKIP target=J%02u helper=J%02u "
                    "M%02u/M%02u reason=all-offline\r\n",
                    (unsigned)targetJoint,
                    (unsigned)candidateJoint,
                    (unsigned)primaryMotor,
                    (unsigned)secondaryMotor);
                continue;
            }
            FW_LOG_WARN_PRINTF(
                "[AUTO CALIB] WORKSPACE_MASK_REJECT target=J%02u joint=J%02u "
                "M%02u/M%02u reason=%s unknown=0x%08lX\r\n",
                (unsigned)targetJoint,
                (unsigned)candidateJoint,
                (unsigned)primaryMotor,
                (unsigned)secondaryMotor,
                workspacePairAvailabilityText(availability),
                (unsigned long)(sharedAtomicLoad(
                    &sharedData->multi_turn_unknown_bitmap) & pairMask));
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED,
                               CALIB_DIAG_PHASE_DUAL_PRELOAD,
                               targetJoint,
                               primaryMotor,
                               candidateJoint,
                               (int32_t)availability,
                               secondaryMotor,
                               pairMask);
            return false;
        }
    }
    *outMotorMask = motorMask & validMotorChannelMask();
    return true;
}

// 原子更新标定期间需持续读取的辅助电机位图。
static void setCalibrationWorkspaceReadMask(TaskSharedData_t* sharedData,
                                            uint32_t motorMask)
{
    if (!sharedData) {
        return;
    }
    SharedSafetyGuard guard;
    sharedAtomicStore(&sharedData->calibration_aux_motor_mask,
                      motorMask & validMotorChannelMask());
}

// 根据目标侧摆关节生成最短 A/B 姿态序列；已缓存正确姿态的步骤会跳过。
static uint8_t buildDualWorkspacePlan(TaskSharedData_t* sharedData,
                                      uint8_t targetJoint,
                                      DualWorkspacePlanStep* steps,
                                      uint8_t capacity)
{
    if (!steps || capacity < kWorkspaceDualHelperCount ||
        !isWorkspaceDualJoint(targetJoint)) {
        return 0;
    }

    uint8_t count = 0;
    // 先为所有非目标手指建立工作空间：编号较小的手指按升序进入 A，
    // 编号较大的手指按降序进入 B。目标关节有意排除在这里之外：
    // 它会在自身标定流程中执行放绳/主电机收紧，或复用先前作为辅助关节
    // 已建立的 B 姿态缓存。
    for (uint8_t i = 0; i < kWorkspaceDualJointsCount; ++i) {
        const uint8_t jointIndex = kWorkspaceDualJoints[i];
        if (jointIndex >= targetJoint) {
            continue;
        }
        const DualWorkspacePairAvailability availability =
            classifyDualWorkspacePair(sharedData, jointIndex, NULL, NULL, NULL);
        if (availability == DUAL_WORKSPACE_PAIR_ALL_OFFLINE) {
            FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_PLAN_SKIP target=J%02u helper=J%02u reason=all-offline\r\n",
                               (unsigned)targetJoint,
                               (unsigned)jointIndex);
            continue;
        }
        if (availability != DUAL_WORKSPACE_PAIR_READY || count >= capacity) {
            FW_LOG_WARN_PRINTF("[AUTO CALIB] WORKSPACE_PLAN_REJECT target=J%02u helper=J%02u reason=%s\r\n",
                               (unsigned)targetJoint,
                               (unsigned)jointIndex,
                               workspacePairAvailabilityText(availability));
            return 0xFF;
        }
        steps[count++] = {jointIndex, DUAL_WORKSPACE_POSE_A};
    }

    for (int8_t i = (int8_t)kWorkspaceDualJointsCount - 1; i >= 0; --i) {
        const uint8_t jointIndex = kWorkspaceDualJoints[(uint8_t)i];
        if (jointIndex <= targetJoint) {
            continue;
        }
        const DualWorkspacePairAvailability availability =
            classifyDualWorkspacePair(sharedData, jointIndex, NULL, NULL, NULL);
        if (availability == DUAL_WORKSPACE_PAIR_ALL_OFFLINE) {
            FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_PLAN_SKIP target=J%02u helper=J%02u reason=all-offline\r\n",
                               (unsigned)targetJoint,
                               (unsigned)jointIndex);
            continue;
        }
        if (availability != DUAL_WORKSPACE_PAIR_READY || count >= capacity) {
            FW_LOG_WARN_PRINTF("[AUTO CALIB] WORKSPACE_PLAN_REJECT target=J%02u helper=J%02u reason=%s\r\n",
                               (unsigned)targetJoint,
                               (unsigned)jointIndex,
                               workspacePairAvailabilityText(availability));
            return 0xFF;
        }
        steps[count++] = {jointIndex, DUAL_WORKSPACE_POSE_B};
    }

    return count;
}

// 把工作空间预检/执行失败写成结构化标定诊断。
static void setWorkspacePlanFailureDiag(TaskSharedData_t* sharedData,
                                        uint8_t targetJoint,
                                        uint8_t helperJoint,
                                        DualWorkspacePose pose,
                                        DualWorkspacePlanStage stage)
{
    const uint8_t existingReason = calibrationDiagReason(sharedData);
    if (existingReason != CALIB_DIAG_REASON_NONE &&
        existingReason != CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED) {
        return;
    }

    uint8_t primaryMotor = 0xFF;
    uint8_t secondaryMotor = 0xFF;
    uint32_t motorMask = 0;
    if (getDualAntagonistAutoCalibrationMotors(helperJoint,
                                               &primaryMotor,
                                               &secondaryMotor)) {
        motorMask = (1UL << primaryMotor) | (1UL << secondaryMotor);
    }
    setCalibrationDiag(sharedData,
                       CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED,
                       CALIB_DIAG_PHASE_DUAL_PRELOAD,
                       targetJoint,
                       primaryMotor,
                       helperJoint,
                       (int32_t)pose,
                       (int32_t)stage,
                       motorMask);
}

// 输出最终工作空间计划，便于核对各手指实际动作顺序。
static void logDualWorkspacePlan(uint8_t targetJoint,
                                 const DualWorkspacePlanStep* steps,
                                 uint8_t count)
{
    FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_PLAN target=J%02u helpers=%u\r\n",
                       (unsigned)targetJoint,
                       (unsigned)count);
    for (uint8_t i = 0; i < count; ++i) {
        FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_PLAN_HELPER target=J%02u joint=J%02u pose=%c\r\n",
                           (unsigned)targetJoint,
                           (unsigned)steps[i].jointIndex,
                           steps[i].pose == DUAL_WORKSPACE_POSE_A ? 'A' : 'B');
    }
}

// 等待一对辅助电机到达 A/B 目标 ABS，并要求误差连续进入 60 counts 窗口。
static bool waitDualWorkspaceTargets(TaskSharedData_t* sharedData,
                                     uint8_t jointIndex,
                                     const char* phase,
                                     uint8_t primaryMotor,
                                     int32_t primaryTargetAbs,
                                     uint8_t secondaryMotor,
                                     int32_t secondaryTargetAbs,
                                     const AutoCalibrationParams& params)
{
    uint32_t lastTraceMs = 0;
    while (autoCalibrationStillOwnsControl(sharedData)) {
        int32_t primaryAbs = 0;
        int32_t secondaryAbs = 0;
        if (!readLatestMotorServoAbs(sharedData, primaryMotor, &primaryAbs) ||
            !readLatestMotorServoAbs(sharedData, secondaryMotor, &secondaryAbs)) {
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED,
                               CALIB_DIAG_PHASE_DUAL_PRELOAD,
                               jointIndex,
                               primaryMotor,
                               primaryMotor,
                               secondaryMotor,
                               0,
                               (1UL << primaryMotor) | (1UL << secondaryMotor));
            return false;
        }
        if (!publishDualMotorCalibrationTargets(sharedData,
                                                primaryMotor,
                                                primaryTargetAbs,
                                                secondaryMotor,
                                                secondaryTargetAbs,
                                                params)) {
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
                               CALIB_DIAG_PHASE_DUAL_PRELOAD,
                               jointIndex,
                               primaryMotor,
                               primaryTargetAbs,
                               secondaryTargetAbs,
                               0,
                               (1UL << primaryMotor) | (1UL << secondaryMotor));
            FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_%s publish failed J%02u M%02u/M%02u\r\n",
                               phase,
                               (unsigned)jointIndex,
                               (unsigned)primaryMotor,
                               (unsigned)secondaryMotor);
            return false;
        }

        const int32_t primaryErr = abs32(primaryAbs - primaryTargetAbs);
        const int32_t secondaryErr = abs32(secondaryAbs - secondaryTargetAbs);
        if (primaryErr <= kWorkspaceTargetToleranceCounts &&
            secondaryErr <= kWorkspaceTargetToleranceCounts) {
            FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_%s_REACHED J%02u M%02u=%ld/%ld M%02u=%ld/%ld\r\n",
                               phase,
                               (unsigned)jointIndex,
                               (unsigned)primaryMotor,
                               (long)primaryAbs,
                               (long)primaryTargetAbs,
                               (unsigned)secondaryMotor,
                               (long)secondaryAbs,
                               (long)secondaryTargetAbs);
            return true;
        }

        const uint32_t nowMs = millis();
        if ((uint32_t)(nowMs - lastTraceMs) >= params.traceMs) {
            lastTraceMs = nowMs;
            FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_%s_WAIT J%02u M%02u err=%ld M%02u err=%ld\r\n",
                               phase,
                               (unsigned)jointIndex,
                               (unsigned)primaryMotor,
                               (long)primaryErr,
                               (unsigned)secondaryMotor,
                               (long)secondaryErr);
        }
        vTaskDelay(pdMS_TO_TICKS(params.loopMs));
    }
    setCalibrationDiag(sharedData,
                       CALIB_DIAG_REASON_CONTROL_LOST,
                       CALIB_DIAG_PHASE_DUAL_PRELOAD,
                       jointIndex,
                       primaryMotor,
                       sharedData ? sharedData->system_state : -1,
                       sharedData ? sharedData->servo_target_owner : -1,
                       sharedData ? sharedData->control_enabled : -1,
                       sharedData ? sharedAtomicLoad(&sharedData->active_joint_mask) : 0);
    FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_%s cancel J%02u state=%u owner=%u enabled=%u\r\n",
                       phase,
                       (unsigned)jointIndex,
                       sharedData ? (unsigned)sharedData->system_state : 255U,
                       sharedData ? (unsigned)sharedData->servo_target_owner : 255U,
                       sharedData ? (unsigned)sharedData->control_enabled : 255U);
    return false;
}

// 等待工作空间涉及电机全部上线且多圈可信。
static bool waitWorkspaceFeedbackKnown(TaskSharedData_t* sharedData,
                                       uint8_t jointIndex,
                                       uint8_t primaryMotor,
                                       uint8_t secondaryMotor,
                                       int32_t* primaryAbs,
                                       int32_t* secondaryAbs)
{
    const uint32_t motorMask = (1UL << primaryMotor) | (1UL << secondaryMotor);
    const uint32_t startMs = millis();
    while (autoCalibrationStillOwnsControl(sharedData)) {
        const uint32_t unknownMask =
            sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap) & motorMask;
        if (unknownMask != 0) {
            const uint8_t firstUnknown = (unknownMask & (1UL << primaryMotor))
                ? primaryMotor : secondaryMotor;
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED,
                               CALIB_DIAG_PHASE_DUAL_PRELOAD,
                               jointIndex,
                               firstUnknown,
                               (int32_t)unknownMask,
                               0,
                               0,
                               unknownMask);
            FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE fail Turn UNKNOWN J%02u mask=0x%08lX; New Device Reset required\r\n",
                               (unsigned)jointIndex,
                               (unsigned long)unknownMask);
            return false;
        }

        const bool primaryOk = readLatestMotorServoAbs(sharedData, primaryMotor, primaryAbs);
        const bool secondaryOk = readLatestMotorServoAbs(sharedData, secondaryMotor, secondaryAbs);
        int16_t primaryLoad = 0;
        int16_t secondaryLoad = 0;
        const bool primaryLoadOk = readLatestMotorServoLoad(sharedData, primaryMotor, &primaryLoad);
        const bool secondaryLoadOk = readLatestMotorServoLoad(sharedData, secondaryMotor, &secondaryLoad);
        if (primaryOk && secondaryOk && primaryLoadOk && secondaryLoadOk) {
            return true;
        }

        if ((uint32_t)(millis() - startMs) >= kInitialFeedbackWaitMs) {
            const uint32_t missingMask =
                (primaryOk && primaryLoadOk ? 0UL : (1UL << primaryMotor)) |
                (secondaryOk && secondaryLoadOk ? 0UL : (1UL << secondaryMotor));
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED,
                               CALIB_DIAG_PHASE_DUAL_PRELOAD,
                               jointIndex,
                               primaryOk && primaryLoadOk ? secondaryMotor : primaryMotor,
                               primaryOk ? 1 : 0,
                               secondaryOk ? 1 : 0,
                               0,
                               missingMask);
            FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE fail feedback J%02u primary=%u secondary=%u load=%u/%u\r\n",
                               (unsigned)jointIndex,
                               primaryOk ? 1U : 0U,
                               secondaryOk ? 1U : 0U,
                               primaryLoadOk ? 1U : 0U,
                               secondaryLoadOk ? 1U : 0U);
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    setCalibrationDiag(sharedData,
                       CALIB_DIAG_REASON_CONTROL_LOST,
                       CALIB_DIAG_PHASE_DUAL_PRELOAD,
                       jointIndex,
                       primaryMotor,
                       sharedData ? sharedData->system_state : -1,
                       sharedData ? sharedData->servo_target_owner : -1,
                       sharedData ? sharedData->control_enabled : -1,
                       sharedData ? sharedAtomicLoad(&sharedData->active_joint_mask) : 0);
    FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE feedback cancel J%02u M%02u/M%02u state=%u owner=%u enabled=%u\r\n",
                       (unsigned)jointIndex,
                       (unsigned)primaryMotor,
                       (unsigned)secondaryMotor,
                       sharedData ? (unsigned)sharedData->system_state : 255U,
                       sharedData ? (unsigned)sharedData->servo_target_owner : 255U,
                       sharedData ? (unsigned)sharedData->control_enabled : 255U);
    return false;
}

// 在任何机械动作前完成全部辅助关节预检，避免执行到一半才发现部分离线。
static bool preflightDualWorkspacePlan(TaskSharedData_t* sharedData,
                                       uint8_t targetJoint,
                                       const DualWorkspacePlanStep* steps,
                                       uint8_t count)
{
    for (uint8_t i = 0; i < count; ++i) {
        const DualWorkspacePlanStep& step = steps[i];
        uint8_t primaryMotor = 0;
        uint8_t secondaryMotor = 0;
        if (!getDualAntagonistAutoCalibrationMotors(step.jointIndex,
                                                    &primaryMotor,
                                                    &secondaryMotor)) {
            setWorkspacePlanFailureDiag(sharedData,
                                        targetJoint,
                                        step.jointIndex,
                                        step.pose,
                                        DUAL_WORKSPACE_PLAN_STAGE_PREFLIGHT);
            FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_PREFLIGHT fail target=J%02u helper=J%02u no dual map\r\n",
                               (unsigned)targetJoint,
                               (unsigned)step.jointIndex);
            return false;
        }

        int32_t primaryAbs = 0;
        int32_t secondaryAbs = 0;
        FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_PREFLIGHT target=J%02u role=HELPER joint=J%02u M%02u/M%02u\r\n",
                           (unsigned)targetJoint,
                           (unsigned)step.jointIndex,
                           (unsigned)primaryMotor,
                           (unsigned)secondaryMotor);
        if (!waitWorkspaceFeedbackKnown(sharedData,
                                        step.jointIndex,
                                        primaryMotor,
                                        secondaryMotor,
                                        &primaryAbs,
                                        &secondaryAbs)) {
            setWorkspacePlanFailureDiag(sharedData,
                                        targetJoint,
                                        step.jointIndex,
                                        step.pose,
                                        DUAL_WORKSPACE_PLAN_STAGE_PREFLIGHT);
            FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_PREFLIGHT fail target=J%02u helper=J%02u pose=%c\r\n",
                               (unsigned)targetJoint,
                               (unsigned)step.jointIndex,
                               step.pose == DUAL_WORKSPACE_POSE_A ? 'A' : 'B');
            return false;
        }
    }
    return true;
}

// 收紧一台辅助电机到 load 190，同时保持同关节另一台电机不动。
static bool tightenWorkspaceMotor(TaskSharedData_t* sharedData,
                                  uint8_t jointIndex,
                                  const char* phase,
                                  uint8_t motor,
                                  uint8_t holdMotor,
                                  int32_t holdAbs,
                                  const AutoCalibrationParams& params,
                                  int32_t* outTightAbs)
{
    int32_t startAbs = 0;
    if (!readLatestMotorServoAbs(sharedData, motor, &startAbs)) {
        return false;
    }
    int32_t commandAbs = startAbs;
    int32_t commandRemainder = 0;
    uint32_t lastCommandMs = millis();
    uint32_t lastTraceMs = 0;
    while (autoCalibrationStillOwnsControl(sharedData)) {
        int32_t currentAbs = 0;
        int16_t loadRaw = 0;
        if (!readLatestMotorServoAbs(sharedData, motor, &currentAbs) ||
            !readLatestMotorServoLoad(sharedData, motor, &loadRaw)) {
            return false;
        }
        const int16_t loadMag = mmhand::servo::loadMagnitude(loadRaw);
        if (loadMag >= kWorkspaceTightLoadThreshold) {
            if (!publishDualMotorCalibrationTargets(sharedData,
                                                    motor,
                                                    currentAbs,
                                                    holdMotor,
                                                    holdAbs,
                                                    params)) {
                setCalibrationDiag(sharedData,
                                   CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
                                   CALIB_DIAG_PHASE_DUAL_PRELOAD,
                                   jointIndex,
                                   motor,
                                   currentAbs,
                                   holdAbs,
                                   0,
                                   (1UL << motor) | (1UL << holdMotor));
                return false;
            }
            if (outTightAbs) {
                *outTightAbs = currentAbs;
            }
            FW_LOG_INFO_PRINTF("[AUTO CALIB] %s_FOUND J%02u M%02u abs=%ld load=%d/%d hold=M%02u abs=%ld\r\n",
                               phase,
                               (unsigned)jointIndex,
                               (unsigned)motor,
                               (long)currentAbs,
                               (int)loadMag,
                               (int)kWorkspaceTightLoadThreshold,
                               (unsigned)holdMotor,
                               (long)holdAbs);
            return true;
        }
        const uint32_t nowMs = millis();
        const uint32_t dtMs = nowMs - lastCommandMs;
        if (dtMs >= params.loopMs) {
            const int32_t numerator =
                (int32_t)dtMs * params.tightenSpeedCountsPerSec + commandRemainder;
            const int32_t stepCounts = numerator / 1000;
            commandRemainder = numerator % 1000;
            if (stepCounts > 0) {
                commandAbs = motorApplyTendonDelta(motor, commandAbs, stepCounts);
                if (!publishDualMotorCalibrationTargets(sharedData,
                                                        motor,
                                                        commandAbs,
                                                        holdMotor,
                                                        holdAbs,
                                                        params)) {
                    setCalibrationDiag(sharedData,
                                       CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
                                       CALIB_DIAG_PHASE_DUAL_PRELOAD,
                                       jointIndex,
                                       motor,
                                       commandAbs,
                                       holdAbs,
                                       0,
                                       (1UL << motor) | (1UL << holdMotor));
                    return false;
                }
            }
            lastCommandMs = nowMs;
        }
        if ((uint32_t)(nowMs - lastTraceMs) >= params.traceMs) {
            lastTraceMs = nowMs;
            FW_LOG_INFO_PRINTF("[AUTO CALIB] TRACE_%s J%02u M%02u cmd=%ld abs=%ld load=%d/%d\r\n",
                               phase,
                               (unsigned)jointIndex,
                               (unsigned)motor,
                               (long)commandAbs,
                               (long)currentAbs,
                               (int)loadMag,
                               (int)kWorkspaceTightLoadThreshold);
        }
        vTaskDelay(pdMS_TO_TICKS(params.loopMs));
    }
    setCalibrationDiag(sharedData,
                       CALIB_DIAG_REASON_CONTROL_LOST,
                       CALIB_DIAG_PHASE_DUAL_PRELOAD,
                       jointIndex,
                       motor,
                       sharedData ? sharedData->system_state : -1,
                       sharedData ? sharedData->servo_target_owner : -1,
                       sharedData ? sharedData->control_enabled : -1,
                       sharedData ? sharedAtomicLoad(&sharedData->active_joint_mask) : 0);
    FW_LOG_INFO_PRINTF("[AUTO CALIB] %s cancel J%02u M%02u state=%u owner=%u enabled=%u\r\n",
                       phase,
                       (unsigned)jointIndex,
                       (unsigned)motor,
                       sharedData ? (unsigned)sharedData->system_state : 255U,
                       sharedData ? (unsigned)sharedData->servo_target_owner : 255U,
                       sharedData ? (unsigned)sharedData->control_enabled : 255U);
    return false;
}

// 将一个辅助双腱关节调整到指定 A/B 姿态，并把实测 ABS 写入 RAM 缓存。
static bool ensureWorkspaceDualPose(TaskSharedData_t* sharedData,
                                    uint8_t jointIndex,
                                    DualWorkspacePose desiredPose,
                                    const AutoCalibrationParams& params)
{
    const int8_t cacheIndex = workspaceCacheIndex(jointIndex);
    if (cacheIndex < 0 || desiredPose == DUAL_WORKSPACE_POSE_NONE) {
        return false;
    }

    uint8_t primaryMotor = 0;
    uint8_t secondaryMotor = 0;
    if (!getDualAntagonistAutoCalibrationMotors(jointIndex, &primaryMotor, &secondaryMotor)) {
        return false;
    }
    int32_t primaryAbs = 0;
    int32_t secondaryAbs = 0;
    if (!waitWorkspaceFeedbackKnown(sharedData,
                                    jointIndex,
                                    primaryMotor,
                                    secondaryMotor,
                                    &primaryAbs,
                                    &secondaryAbs)) {
        return false;
    }

    DualWorkspaceCacheEntry* entry = &g_dualWorkspaceCache[(uint8_t)cacheIndex];
    if (entry->valid &&
        (entry->primaryMotor != primaryMotor || entry->secondaryMotor != secondaryMotor)) {
        memset(entry, 0, sizeof(*entry));
    }
    entry->primaryMotor = primaryMotor;
    entry->secondaryMotor = secondaryMotor;

    if (entry->valid && entry->currentPose == desiredPose) {
        const int32_t primaryTarget = desiredPose == DUAL_WORKSPACE_POSE_A
            ? entry->primaryReleaseAbs : entry->primaryTightAbs;
        const int32_t secondaryTarget = desiredPose == DUAL_WORKSPACE_POSE_A
            ? entry->secondaryTightAbs : entry->secondaryReleaseAbs;
        const int32_t primaryError = abs32(primaryAbs - primaryTarget);
        const int32_t secondaryError = abs32(secondaryAbs - secondaryTarget);
        if (primaryError <= kWorkspaceTargetToleranceCounts &&
            secondaryError <= kWorkspaceTargetToleranceCounts) {
            // 该姿态已由前序工作空间步骤或已完成的标定建立；直接复用，
            // 不再重复发送舵机目标。
            FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_REUSE_NO_COMMAND J%02u pose=%c M%02u=%ld/%ld err=%ld M%02u=%ld/%ld err=%ld\r\n",
                               (unsigned)jointIndex,
                               desiredPose == DUAL_WORKSPACE_POSE_A ? 'A' : 'B',
                               (unsigned)primaryMotor,
                               (long)primaryAbs,
                               (long)primaryTarget,
                               (long)primaryError,
                               (unsigned)secondaryMotor,
                               (long)secondaryAbs,
                               (long)secondaryTarget,
                               (long)secondaryError);
            return true;
        }

        // RAM 缓存已不能准确描述当前机械姿态。使缓存失效，并依据实测反馈
        // 重新建立目标姿态，禁止使用过期 ABS 缓存继续下发。
        FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_REUSE_INVALIDATE J%02u pose=%c M%02u err=%ld M%02u err=%ld tolerance=%ld\r\n",
                           (unsigned)jointIndex,
                           desiredPose == DUAL_WORKSPACE_POSE_A ? 'A' : 'B',
                           (unsigned)primaryMotor,
                           (long)primaryError,
                           (unsigned)secondaryMotor,
                           (long)secondaryError,
                           (long)kWorkspaceTargetToleranceCounts);
        memset(entry, 0, sizeof(*entry));
        entry->primaryMotor = primaryMotor;
        entry->secondaryMotor = secondaryMotor;
    }

    if (entry->valid && entry->currentPose == DUAL_WORKSPACE_POSE_A &&
        desiredPose == DUAL_WORKSPACE_POSE_B) {
        entry->secondaryReleaseAbs = motorApplyTendonDelta(secondaryMotor,
                                                            entry->secondaryTightAbs,
                                                            -kWorkspaceReleaseCounts);
        if (!waitDualWorkspaceTargets(sharedData, jointIndex, "A_TO_B_RELEASE",
                                      primaryMotor, entry->primaryReleaseAbs,
                                      secondaryMotor, entry->secondaryReleaseAbs, params) ||
            !tightenWorkspaceMotor(sharedData, jointIndex, "WORKSPACE_B_TIGHT",
                                   primaryMotor, secondaryMotor, entry->secondaryReleaseAbs,
                                   params, &entry->primaryTightAbs)) {
            return false;
        }
        entry->currentPose = DUAL_WORKSPACE_POSE_B;
        entry->valid = 1;
        return true;
    }

    if (entry->valid && entry->currentPose == DUAL_WORKSPACE_POSE_B &&
        desiredPose == DUAL_WORKSPACE_POSE_A) {
        entry->primaryReleaseAbs = motorApplyTendonDelta(primaryMotor,
                                                          entry->primaryTightAbs,
                                                          -kWorkspaceReleaseCounts);
        if (!waitDualWorkspaceTargets(sharedData, jointIndex, "B_TO_A_RELEASE",
                                      primaryMotor, entry->primaryReleaseAbs,
                                      secondaryMotor, entry->secondaryReleaseAbs, params) ||
            !tightenWorkspaceMotor(sharedData, jointIndex, "WORKSPACE_A_TIGHT",
                                   secondaryMotor, primaryMotor, entry->primaryReleaseAbs,
                                   params, &entry->secondaryTightAbs)) {
            return false;
        }
        entry->currentPose = DUAL_WORKSPACE_POSE_A;
        entry->valid = 1;
        return true;
    }

    // 当前序列没有可信姿态缓存，必须从实测姿态建立工作空间：先将一侧肌腱
    // 放绳 3200 counts，同时保持拮抗侧不动，再把拮抗侧收紧到工作空间
    // 负载阈值。该顺序与目标 A/B 姿态一致，可避免本应放松的肌腱被提前收紧。
    const uint8_t releaseMotor = desiredPose == DUAL_WORKSPACE_POSE_A
        ? primaryMotor : secondaryMotor;
    const uint8_t tightenMotor = desiredPose == DUAL_WORKSPACE_POSE_A
        ? secondaryMotor : primaryMotor;
    const int32_t releaseStartAbs = desiredPose == DUAL_WORKSPACE_POSE_A
        ? primaryAbs : secondaryAbs;
    const int32_t holdAbs = desiredPose == DUAL_WORKSPACE_POSE_A
        ? secondaryAbs : primaryAbs;
    const int32_t releaseTargetAbs = motorApplyTendonDelta(releaseMotor,
                                                            releaseStartAbs,
                                                            -kWorkspaceReleaseCounts);
    if (!isServoCommandInsideLimit(sharedData,
                                   releaseMotor,
                                   releaseTargetAbs)) {
        int32_t rangeMin = 0;
        int32_t rangeMax = 0;
        getServoLogicalCommandRange(
            sharedData, releaseMotor, &rangeMin, &rangeMax);
        setCalibrationDiag(sharedData,
                           CALIB_DIAG_REASON_ABS_LIMIT_INVALID,
                           CALIB_DIAG_PHASE_DUAL_PRELOAD,
                           jointIndex,
                           releaseMotor,
                           releaseTargetAbs,
                           rangeMin,
                           rangeMax,
                           1UL << releaseMotor);
        return false;
    }

    const int32_t primaryReleaseTarget = desiredPose == DUAL_WORKSPACE_POSE_A
        ? releaseTargetAbs : holdAbs;
    const int32_t secondaryReleaseTarget = desiredPose == DUAL_WORKSPACE_POSE_A
        ? holdAbs : releaseTargetAbs;
    if (!waitDualWorkspaceTargets(sharedData,
                                  jointIndex,
                                  "DIRECT_RELEASE",
                                  primaryMotor,
                                  primaryReleaseTarget,
                                  secondaryMotor,
                                  secondaryReleaseTarget,
                                  params)) {
        return false;
    }

    int32_t tightenAbs = 0;
    if (!tightenWorkspaceMotor(sharedData,
                               jointIndex,
                               desiredPose == DUAL_WORKSPACE_POSE_A
                                   ? "WORKSPACE_A_TIGHT" : "WORKSPACE_B_TIGHT",
                               tightenMotor,
                               releaseMotor,
                               releaseTargetAbs,
                               params,
                               &tightenAbs)) {
        return false;
    }

    if (desiredPose == DUAL_WORKSPACE_POSE_A) {
        entry->primaryReleaseAbs = releaseTargetAbs;
        entry->secondaryTightAbs = tightenAbs;
    } else {
        entry->primaryTightAbs = tightenAbs;
        entry->secondaryReleaseAbs = releaseTargetAbs;
    }
    entry->currentPose = desiredPose;
    entry->valid = 1;
    FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE established J%02u pose=%c release=M%02u start=%ld target=%ld tight=M%02u abs=%ld load=%d\r\n",
                       (unsigned)jointIndex,
                       desiredPose == DUAL_WORKSPACE_POSE_A ? 'A' : 'B',
                       (unsigned)releaseMotor,
                       (long)releaseStartAbs,
                       (long)releaseTargetAbs,
                       (unsigned)tightenMotor,
                       (long)tightenAbs,
                       (int)kWorkspaceTightLoadThreshold);
    return true;
}

// 为当前目标关节执行工作空间预检、计划生成和必要的辅助姿态动作。
static bool prepareWorkspaceForDualCalibration(TaskSharedData_t* sharedData,
                                               uint8_t targetJoint,
                                               const AutoCalibrationParams& params)
{
    if (!isWorkspaceDualJoint(targetJoint)) {
        return true;
    }

    uint8_t targetPrimaryMotor = 0;
    uint8_t targetSecondaryMotor = 0;
    int32_t targetPrimaryAbs = 0;
    int32_t targetSecondaryAbs = 0;
    if (!getDualAntagonistAutoCalibrationMotors(targetJoint,
                                                &targetPrimaryMotor,
                                                &targetSecondaryMotor) ||
        !waitWorkspaceFeedbackKnown(sharedData,
                                    targetJoint,
                                    targetPrimaryMotor,
                                    targetSecondaryMotor,
                                    &targetPrimaryAbs,
                                    &targetSecondaryAbs)) {
        setWorkspacePlanFailureDiag(sharedData,
                                    targetJoint,
                                    targetJoint,
                                    DUAL_WORKSPACE_POSE_NONE,
                                    DUAL_WORKSPACE_PLAN_STAGE_PREFLIGHT);
        FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_TARGET_PREFLIGHT fail target=J%02u\r\n",
                           (unsigned)targetJoint);
        return false;
    }
    FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_TARGET_PREFLIGHT ok target=J%02u M%02u=%ld M%02u=%ld; no target pose command\r\n",
                       (unsigned)targetJoint,
                       (unsigned)targetPrimaryMotor,
                       (long)targetPrimaryAbs,
                       (unsigned)targetSecondaryMotor,
                       (long)targetSecondaryAbs);

    DualWorkspacePlanStep steps[kWorkspaceDualHelperCount] = {};
    const uint8_t stepCount = buildDualWorkspacePlan(sharedData,
                                                      targetJoint,
                                                      steps,
                                                      kWorkspaceDualHelperCount);
    if (stepCount == 0xFF) {
        setWorkspacePlanFailureDiag(sharedData,
                                    targetJoint,
                                    targetJoint,
                                    DUAL_WORKSPACE_POSE_B,
                                    DUAL_WORKSPACE_PLAN_STAGE_PREFLIGHT);
        FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_PLAN invalid target=J%02u count=%u\r\n",
                           (unsigned)targetJoint,
                           (unsigned)stepCount);
        return false;
    }

    logDualWorkspacePlan(targetJoint, steps, stepCount);
    // 目标关节及全部辅助双电机都必须具备可信且可读的 ABS/load 反馈，
    // 才允许移动任何辅助关节，避免通道缺失时留下半完成的工作空间姿态。
    if (!preflightDualWorkspacePlan(sharedData, targetJoint, steps, stepCount)) {
        return false;
    }

    for (uint8_t i = 0; i < stepCount; ++i) {
        const DualWorkspacePlanStep& step = steps[i];
        FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_HELPER_BEGIN target=J%02u joint=J%02u pose=%c\r\n",
                           (unsigned)targetJoint,
                           (unsigned)step.jointIndex,
                           step.pose == DUAL_WORKSPACE_POSE_A ? 'A' : 'B');
        if (!ensureWorkspaceDualPose(sharedData,
                                     step.jointIndex,
                                     step.pose,
                                     params)) {
            setWorkspacePlanFailureDiag(sharedData,
                                        targetJoint,
                                        step.jointIndex,
                                        step.pose,
                                        DUAL_WORKSPACE_PLAN_STAGE_EXECUTE);
            FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_HELPER_FAIL target=J%02u joint=J%02u pose=%c\r\n",
                               (unsigned)targetJoint,
                               (unsigned)step.jointIndex,
                               step.pose == DUAL_WORKSPACE_POSE_A ? 'A' : 'B');
            return false;
        }
        FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_HELPER_DONE target=J%02u joint=J%02u pose=%c\r\n",
                           (unsigned)targetJoint,
                           (unsigned)step.jointIndex,
                           step.pose == DUAL_WORKSPACE_POSE_A ? 'A' : 'B');
    }
    return true;
}

// 主电机保持紧端时，逐步收紧副电机到预紧负载 190。
static bool findDualSecondaryPreloadPoint(TaskSharedData_t* sharedData,
                                          uint8_t jointIndex,
                                          uint8_t secondaryMotor,
                                          int8_t secondaryShortenSign,
                                          int32_t secondaryStartAbs,
                                          uint8_t primaryMotor,
                                          int32_t primaryHoldAbs,
                                          int16_t loadThreshold,
                                          const AutoCalibrationParams& params,
                                          int32_t* outPreloadAbs,
                                          uint16_t* outPreloadRaw,
                                          int32_t* outPreloadEncoder,
                                          int16_t* outPreloadLoad)
{
    uint32_t lastTraceMs = 0;
    int32_t commandAbs = secondaryStartAbs;
    int32_t commandRemainder = 0;
    uint32_t lastCommandMs = millis();

    // 与 CUI 版本轨迹保持一致：从实测 ABS 起步，按配置速度逐步推进，
    // 不预先设置远距离预紧目标。
    FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_PRELOAD_SECONDARY_BEGIN J%02u secondary=M%02u startAbs=%ld shortenSign=%d primaryHold=M%02u holdAbs=%ld loadThreshold=%d\r\n",
                  jointIndex,
                  secondaryMotor,
                  (long)secondaryStartAbs,
                  (int)secondaryShortenSign,
                  primaryMotor,
                  (long)primaryHoldAbs,
                  loadThreshold);

    while (autoCalibrationStillOwnsControl(sharedData)) {
        const uint32_t nowMs = millis();
        uint16_t currentRaw = 0;
        int32_t currentEncoder = 0;
        int32_t currentAbs = 0;
        if (!readLatestEncoderSample(sharedData, jointIndex, &currentRaw, &currentEncoder) ||
            !readLatestMotorServoAbs(sharedData, secondaryMotor, &currentAbs)) {
            FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL DUAL_PRELOAD_SECONDARY lost feedback J%02u M%02u\r\n",
                          jointIndex,
                          secondaryMotor);
            return false;
        }

        int16_t loadRaw = 0;
        const bool hasLoad =
            readLatestMotorServoLoad(sharedData, secondaryMotor, &loadRaw);
        const int16_t loadMag = hasLoad ? mmhand::servo::loadMagnitude(loadRaw) : 0;
        if (hasLoad && loadMag >= loadThreshold) {
            if (outPreloadAbs) {
                *outPreloadAbs = currentAbs;
            }
            if (outPreloadRaw) {
                *outPreloadRaw = currentRaw;
            }
            if (outPreloadEncoder) {
                *outPreloadEncoder = currentEncoder;
            }
            if (outPreloadLoad) {
                *outPreloadLoad = loadMag;
            }
            FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_PRELOAD_SECONDARY_FOUND J%02u M%02u raw=%u oriented=%ld abs=%ld load=%d/%d primaryHoldAbs=%ld\r\n",
                          jointIndex,
                          secondaryMotor,
                          (unsigned)currentRaw,
                          (long)currentEncoder,
                          (long)currentAbs,
                          loadMag,
                          loadThreshold,
                          (long)primaryHoldAbs);
            return publishDualMotorCalibrationTargets(sharedData,
                                                      secondaryMotor,
                                                      currentAbs,
                                                      primaryMotor,
                                                      primaryHoldAbs,
                                                      params);
        }

        const uint32_t dtMs = nowMs - lastCommandMs;
        if (dtMs >= params.loopMs) {
            const int32_t numerator =
                (int32_t)dtMs * params.tightenSpeedCountsPerSec + commandRemainder;
            const int32_t stepCounts = numerator / 1000;
            commandRemainder = numerator % 1000;
            if (stepCounts > 0) {
                commandAbs = motorApplyTendonDelta(secondaryMotor,
                                                    commandAbs,
                                                    stepCounts);
                if (!publishDualMotorCalibrationTargets(
                        sharedData,
                        secondaryMotor,
                        commandAbs,
                        primaryMotor,
                        primaryHoldAbs,
                        params)) {
                    return false;
                }
            }
            lastCommandMs = nowMs;
        }

        if ((uint32_t)(nowMs - lastTraceMs) >= params.traceMs) {
            lastTraceMs = nowMs;
            FW_LOG_INFO_PRINTF("[AUTO CALIB] TRACE_DUAL_PRELOAD_SECONDARY J%02u M%02u cmd=%ld abs=%ld raw=%u oriented=%ld load=%d/%d primaryHoldAbs=%ld\r\n",
                          jointIndex,
                          secondaryMotor,
                          (long)commandAbs,
                          (long)currentAbs,
                          (unsigned)currentRaw,
                          (long)currentEncoder,
                          loadMag,
                          loadThreshold,
                          (long)primaryHoldAbs);
        }

        vTaskDelay(pdMS_TO_TICKS(params.loopMs));
    }
    FW_LOG_INFO_PRINTF("[AUTO CALIB] CANCEL DUAL_PRELOAD_SECONDARY J%02u secondary=M%02u primary=M%02u cmdAbs=%ld holdAbs=%ld state=%u owner=%u enabled=%u\r\n",
                  jointIndex,
                  secondaryMotor,
                  primaryMotor,
                  (long)commandAbs,
                  (long)primaryHoldAbs,
                  sharedData ? (unsigned)sharedData->system_state : 255,
                  sharedData ? (unsigned)sharedData->servo_target_owner : 255,
                  sharedData ? (unsigned)sharedData->control_enabled : 255);
    return false;
}

// 副电机已达到最终 250 后，仅补紧主电机到最终阈值并验证 ABS 稳定。
static bool finishDualTendonReadyPreload(TaskSharedData_t* sharedData,
                                         uint8_t jointIndex,
                                         uint8_t primaryMotor,
                                         int32_t primaryStartAbs,
                                         int16_t primaryLoadThreshold,
                                         uint8_t secondaryMotor,
                                         int32_t secondaryStartAbs,
                                         const AutoCalibrationParams& params,
                                         int32_t* outPrimaryAbs,
                                         int32_t* outSecondaryAbs,
                                         uint16_t* outEncoderRaw,
                                         int32_t* outEncoder,
                                         int16_t* outPrimaryLoad,
                                         int16_t* outSecondaryLoad)
{
    int32_t primaryCommandAbs = primaryStartAbs;
    int32_t secondaryCommandAbs = secondaryStartAbs;
    int32_t commandRemainder = 0;
    uint32_t lastCommandMs = millis();
    uint32_t lastTraceMs = 0;

    FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_FINAL_READY_PRELOAD_BEGIN J%02u primary=M%02u startAbs=%ld loadThreshold=%d secondary=M%02u holdAbs=%ld; secondary already reached final threshold\r\n",
                       jointIndex,
                       primaryMotor,
                       (long)primaryStartAbs,
                       primaryLoadThreshold,
                       secondaryMotor,
                       (long)secondaryStartAbs);

    // 主电机开始收紧前，先把两台电机命令同步到交换阶段的实测终点，
    // 防止旧交换目标残留到最终预紧阶段。
    if (!publishDualMotorCalibrationTargets(sharedData,
                                            primaryMotor,
                                            primaryCommandAbs,
                                            secondaryMotor,
                                            secondaryCommandAbs,
                                            params)) {
        return false;
    }

    while (autoCalibrationStillOwnsControl(sharedData)) {
        const uint32_t nowMs = millis();
        uint16_t currentRaw = 0;
        int32_t currentEncoder = 0;
        int32_t currentPrimaryAbs = 0;
        int32_t currentSecondaryAbs = 0;
        int16_t primaryLoadRaw = 0;
        int16_t secondaryLoadRaw = 0;
        if (!readLatestEncoderSample(sharedData,
                                     jointIndex,
                                     &currentRaw,
                                     &currentEncoder) ||
            !readLatestMotorServoAbs(sharedData,
                                     primaryMotor,
                                     &currentPrimaryAbs) ||
            !readLatestMotorServoAbs(sharedData,
                                     secondaryMotor,
                                     &currentSecondaryAbs) ||
            !readLatestMotorServoLoad(sharedData,
                                      primaryMotor,
                                      &primaryLoadRaw) ||
            !readLatestMotorServoLoad(sharedData,
                                      secondaryMotor,
                                      &secondaryLoadRaw)) {
            FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL DUAL_FINAL_READY_PRELOAD lost feedback J%02u primary=M%02u secondary=M%02u\r\n",
                               jointIndex,
                               primaryMotor,
                               secondaryMotor);
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED,
                               CALIB_DIAG_PHASE_DUAL_PRELOAD,
                               jointIndex,
                               primaryMotor,
                               primaryMotor,
                               secondaryMotor,
                               0,
                               (1UL << primaryMotor) | (1UL << secondaryMotor));
            return false;
        }

        const int16_t primaryLoad = mmhand::servo::loadMagnitude(primaryLoadRaw);
        const int16_t secondaryLoad = mmhand::servo::loadMagnitude(secondaryLoadRaw);
        const bool primaryReady = primaryLoad > primaryLoadThreshold;
        if (primaryReady) {
            if (outPrimaryAbs) {
                *outPrimaryAbs = currentPrimaryAbs;
            }
            if (outSecondaryAbs) {
                *outSecondaryAbs = currentSecondaryAbs;
            }
            if (outEncoderRaw) {
                *outEncoderRaw = currentRaw;
            }
            if (outEncoder) {
                *outEncoder = currentEncoder;
            }
            if (outPrimaryLoad) {
                *outPrimaryLoad = primaryLoad;
            }
            if (outSecondaryLoad) {
                *outSecondaryLoad = secondaryLoad;
            }
            FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_FINAL_READY_PRELOAD_FOUND J%02u primary=M%02u abs=%ld load=%d/%d secondary=M%02u abs=%ld load=%d diagnostic raw=%u oriented=%ld\r\n",
                               jointIndex,
                               primaryMotor,
                               (long)currentPrimaryAbs,
                               primaryLoad,
                               primaryLoadThreshold,
                               secondaryMotor,
                               (long)currentSecondaryAbs,
                               secondaryLoad,
                               (unsigned)currentRaw,
                               (long)currentEncoder);
            return publishDualMotorCalibrationTargets(sharedData,
                                                      primaryMotor,
                                                      currentPrimaryAbs,
                                                      secondaryMotor,
                                                      currentSecondaryAbs,
                                                      params);
        }

        const uint32_t dtMs = nowMs - lastCommandMs;
        if (dtMs >= params.loopMs) {
            const int32_t numerator =
                (int32_t)dtMs * params.tightenSpeedCountsPerSec +
                commandRemainder;
            const int32_t stepCounts = numerator / 1000;
            commandRemainder = numerator % 1000;
            if (stepCounts > 0) {
                primaryCommandAbs = motorApplyTendonDelta(primaryMotor,
                                                           primaryCommandAbs,
                                                           stepCounts);
                if (!publishDualMotorCalibrationTargets(sharedData,
                                                        primaryMotor,
                                                        primaryCommandAbs,
                                                        secondaryMotor,
                                                        secondaryCommandAbs,
                                                        params)) {
                    return false;
                }
            }
            lastCommandMs = nowMs;
        }

        if ((uint32_t)(nowMs - lastTraceMs) >= params.traceMs) {
            lastTraceMs = nowMs;
            FW_LOG_INFO_PRINTF("[AUTO CALIB] TRACE_DUAL_FINAL_READY_PRELOAD J%02u primary=M%02u cmd=%ld abs=%ld load=%d/%d secondary=M%02u holdCmd=%ld abs=%ld load=%d diagnostic raw=%u oriented=%ld\r\n",
                               jointIndex,
                               primaryMotor,
                               (long)primaryCommandAbs,
                               (long)currentPrimaryAbs,
                               primaryLoad,
                               primaryLoadThreshold,
                               secondaryMotor,
                               (long)secondaryCommandAbs,
                               (long)currentSecondaryAbs,
                               secondaryLoad,
                               (unsigned)currentRaw,
                               (long)currentEncoder);
        }

        vTaskDelay(pdMS_TO_TICKS(params.loopMs));
    }

    FW_LOG_INFO_PRINTF("[AUTO CALIB] CANCEL DUAL_FINAL_READY_PRELOAD J%02u primary=M%02u cmd=%ld secondary=M%02u cmd=%ld state=%u owner=%u enabled=%u\r\n",
                       jointIndex,
                       primaryMotor,
                       (long)primaryCommandAbs,
                       secondaryMotor,
                       (long)secondaryCommandAbs,
                       sharedData ? (unsigned)sharedData->system_state : 255,
                       sharedData ? (unsigned)sharedData->servo_target_owner : 255,
                       sharedData ? (unsigned)sharedData->control_enabled : 255);
    return false;
}

// 主放/副收同步交换，直到副电机达到最终负载且磁编行程满足要求。
static bool findDualExchangeLimitPoint(TaskSharedData_t* sharedData,
                                       uint8_t jointIndex,
                                       uint8_t secondaryMotor,
                                       int8_t secondaryShortenSign,
                                       int32_t secondaryStartAbs,
                                       uint8_t primaryMotor,
                                       int8_t primaryShortenSign,
                                       int32_t primaryStartAbs,
                                       int32_t startEncoder,
                                       int16_t finalLoadThreshold,
                                       const AutoCalibrationParams& params,
                                       int32_t* outSecondaryLimitAbs,
                                       int32_t* outPrimaryFinalAbs,
                                       uint16_t* outLimitRaw,
                                       int32_t* outLimitEncoder,
                                       int16_t* outSecondaryLoad,
                                       TendonRawFitBuffer* fitBuffer)
{
    int32_t secondaryCommandAbs = secondaryStartAbs;
    int32_t primaryCommandAbs = primaryStartAbs;
    int32_t commandRemainder = 0;
    const uint32_t startMs = millis();
    uint32_t lastCommandMs = startMs;
    uint32_t lastTraceMs = 0;
    uint32_t windowStartMs = startMs;
    int32_t windowStartEncoder = startEncoder;
    int32_t windowStartSecondaryAbs = secondaryStartAbs;
    int32_t windowStartPrimaryAbs = primaryStartAbs;
    uint8_t angleStallStable = 0;

    FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_EXCHANGE_NEG3_BEGIN J%02u secondary=M%02u secondaryAbs=%ld secondaryShortenSign=%d primary=M%02u primaryAbs=%ld primaryShortenSign=%d requiredTravelDeg=%.3f finalLoadThreshold=%d\r\n",
                  jointIndex,
                  secondaryMotor,
                  (long)secondaryStartAbs,
                  (int)secondaryShortenSign,
                  primaryMotor,
                  (long)primaryStartAbs,
                  (int)primaryShortenSign,
                  kReverseMinFollowDeg,
                  finalLoadThreshold);

    while (autoCalibrationStillOwnsControl(sharedData)) {
        const uint32_t nowMs = millis();
        uint16_t currentRaw = 0;
        int32_t currentEncoder = 0;
        int32_t currentSecondaryAbs = 0;
        int32_t currentPrimaryAbs = 0;
        if (!readLatestEncoderSample(sharedData, jointIndex, &currentRaw, &currentEncoder) ||
            !readLatestMotorServoAbs(sharedData, secondaryMotor, &currentSecondaryAbs) ||
            !readLatestMotorServoAbs(sharedData, primaryMotor, &currentPrimaryAbs)) {
            FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL DUAL_EXCHANGE_NEG3 lost feedback J%02u M%02u/M%02u\r\n",
                          jointIndex,
                          secondaryMotor,
                          primaryMotor);
            return false;
        }

        int16_t loadRaw = 0;
        const bool hasLoad =
            readLatestMotorServoLoad(sharedData, secondaryMotor, &loadRaw);
        const int16_t loadMag = hasLoad ? mmhand::servo::loadMagnitude(loadRaw) : 0;
        const bool secondaryLoadHigh = hasLoad && loadMag >= finalLoadThreshold;
        const int32_t totalEncoderDelta =
            calibrationWrapEncoderDelta(currentEncoder, startEncoder);
        const float totalAngleDeltaDeg =
            fabsf(calibrationEncoderCountToDeg(totalEncoderDelta));
        maybeAppendTendonTravelFitSample(fitBuffer,
                                             totalAngleDeltaDeg,
                                             currentEncoder,
                                             currentPrimaryAbs,
                                             currentSecondaryAbs);
        bool limitFound = false;

        if ((uint32_t)(nowMs - windowStartMs) >= params.windowMs) {
            const int32_t windowEncoderDelta =
                calibrationWrapEncoderDelta(currentEncoder, windowStartEncoder);
            const float windowAngleDeltaDeg =
                fabsf(calibrationEncoderCountToDeg(windowEncoderDelta));
            const int32_t windowSecondaryAbsDelta =
                abs32(currentSecondaryAbs - windowStartSecondaryAbs);
            const int32_t windowPrimaryAbsDelta =
                abs32(currentPrimaryAbs - windowStartPrimaryAbs);
            const bool enoughEncoderTravel =
                totalAngleDeltaDeg >= kReverseMinFollowDeg;
            const bool angleStalled =
                windowAngleDeltaDeg <= params.stallActualDeg;
            const bool secondaryStillReeling =
                windowSecondaryAbsDelta >= params.windowServoDeltaCounts;
            const bool primaryStillPaying =
                windowPrimaryAbsDelta >= params.windowServoDeltaCounts;
            if (angleStalled) {
                if (angleStallStable < 255) {
                    angleStallStable++;
                }
            } else {
                angleStallStable = 0;
            }
            FW_LOG_INFO_PRINTF("[AUTO CALIB] CHECK_DUAL_EXCHANGE_NEG3 J%02u secondary=M%02u abs=%ld primary=M%02u abs=%ld raw=%u oriented=%ld dCount=%ld winCount=%ld winSecAbs=%ld/%ld winPriAbs=%ld/%ld winDeg=%.3f/%.3f totalDeg=%.3f/%.3f load=%d/%d cond load=%u sec=%u pri=%u angle=%u angleStable=%u/%u travel=%u\r\n",
                          jointIndex,
                          secondaryMotor,
                          (long)currentSecondaryAbs,
                          primaryMotor,
                          (long)currentPrimaryAbs,
                          (unsigned)currentRaw,
                          (long)currentEncoder,
                          (long)totalEncoderDelta,
                          (long)windowEncoderDelta,
                          (long)windowSecondaryAbsDelta,
                          (long)params.windowServoDeltaCounts,
                          (long)windowPrimaryAbsDelta,
                          (long)params.windowServoDeltaCounts,
                          windowAngleDeltaDeg,
                          params.stallActualDeg,
                          totalAngleDeltaDeg,
                          kReverseMinFollowDeg,
                          loadMag,
                          finalLoadThreshold,
                          secondaryLoadHigh ? 1 : 0,
                          secondaryStillReeling ? 1 : 0,
                          primaryStillPaying ? 1 : 0,
                          angleStalled ? 1 : 0,
                          (unsigned)angleStallStable,
                          (unsigned)kAngleStallStableSamples,
                          enoughEncoderTravel ? 1 : 0);

            limitFound =
                secondaryLoadHigh &&
                enoughEncoderTravel &&
                secondaryStillReeling &&
                primaryStillPaying;
            windowStartEncoder = currentEncoder;
            windowStartSecondaryAbs = currentSecondaryAbs;
            windowStartPrimaryAbs = currentPrimaryAbs;
            windowStartMs = nowMs;
        }

        if (limitFound) {
            if (outSecondaryLimitAbs) {
                *outSecondaryLimitAbs = currentSecondaryAbs;
            }
            if (outPrimaryFinalAbs) {
                *outPrimaryFinalAbs = currentPrimaryAbs;
            }
            if (outLimitRaw) {
                *outLimitRaw = currentRaw;
            }
            if (outLimitEncoder) {
                *outLimitEncoder = currentEncoder;
            }
            if (outSecondaryLoad) {
                *outSecondaryLoad = loadMag;
            }
            FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_EXCHANGE_NEG3_FOUND J%02u secondary=M%02u raw=%u oriented=%ld secondaryAbs=%ld primaryAbs=%ld totalDeg=%.3f load=%d/%d\r\n",
                          jointIndex,
                          secondaryMotor,
                          (unsigned)currentRaw,
                          (long)currentEncoder,
                          (long)currentSecondaryAbs,
                          (long)currentPrimaryAbs,
                          totalAngleDeltaDeg,
                          loadMag,
                          finalLoadThreshold);
            return publishDualMotorCalibrationTargets(sharedData,
                                                      secondaryMotor,
                                                      currentSecondaryAbs,
                                                      primaryMotor,
                                                      currentPrimaryAbs,
                                                      params);
        }

        const uint32_t dtMs = nowMs - lastCommandMs;
        if (dtMs >= params.loopMs) {
            const int32_t numerator =
                (int32_t)dtMs * params.tightenSpeedCountsPerSec + commandRemainder;
            const int32_t stepCounts = numerator / 1000;
            commandRemainder = numerator % 1000;
            if (stepCounts > 0) {
                secondaryCommandAbs = motorApplyTendonDelta(secondaryMotor,
                                                            secondaryCommandAbs,
                                                            stepCounts);
                primaryCommandAbs = motorApplyTendonDelta(primaryMotor,
                                                          primaryCommandAbs,
                                                          -stepCounts);
                if (!publishDualMotorCalibrationTargets(sharedData,
                                                        secondaryMotor,
                                                        secondaryCommandAbs,
                                                        primaryMotor,
                                                        primaryCommandAbs,
                                                        params)) {
                    return false;
                }
            }
            lastCommandMs = nowMs;
        }

        if ((uint32_t)(nowMs - lastTraceMs) >= params.traceMs) {
            lastTraceMs = nowMs;
            FW_LOG_INFO_PRINTF("[AUTO CALIB] TRACE_DUAL_EXCHANGE_NEG3 J%02u secondary=M%02u cmd=%ld abs=%ld primary=M%02u cmd=%ld abs=%ld raw=%u oriented=%ld totalDeg=%.3f load=%d/%d\r\n",
                          jointIndex,
                          secondaryMotor,
                          (long)secondaryCommandAbs,
                          (long)currentSecondaryAbs,
                          primaryMotor,
                          (long)primaryCommandAbs,
                          (long)currentPrimaryAbs,
                          (unsigned)currentRaw,
                          (long)currentEncoder,
                          totalAngleDeltaDeg,
                          loadMag,
                          finalLoadThreshold);
        }

        vTaskDelay(pdMS_TO_TICKS(params.loopMs));
    }
    FW_LOG_INFO_PRINTF("[AUTO CALIB] CANCEL DUAL_EXCHANGE_NEG3 J%02u secondary=M%02u cmd=%ld primary=M%02u cmd=%ld state=%u owner=%u enabled=%u\r\n",
                  jointIndex,
                  secondaryMotor,
                  (long)secondaryCommandAbs,
                  primaryMotor,
                  (long)primaryCommandAbs,
                  sharedData ? (unsigned)sharedData->system_state : 255,
                  sharedData ? (unsigned)sharedData->servo_target_owner : 255,
                  sharedData ? (unsigned)sharedData->control_enabled : 255);
    return false;
}

// 双腱关节完整标定流程：
// 主紧端 -> 副松 3200 -> 副预紧 190 -> 主放副收至副 250 ->
// 主补紧 -> 构建 V3 映射与两路 Tendon LUT -> 登记 READY 姿态。
static bool runDualAntagonistAutoCalibrationJoint(TaskSharedData_t* sharedData,
                                                  uint8_t jointIndex,
                                                  const AutoCalibrationParams& params,
                                                  DualWorkspaceCacheEntry* workspaceState)
{
    uint8_t primaryMotor = 0;
    uint8_t secondaryMotor = 0;
    if (!getDualAntagonistAutoCalibrationMotors(jointIndex, &primaryMotor, &secondaryMotor)) {
        return false;
    }
    setRuntimeEncoderDirection(jointIndex, kDefaultEncoderDirection[jointIndex]);
    clearCalibratedMotorAbsLimitsForJoint(sharedData, jointIndex);
    const int8_t primaryShortenSign = getMotorTendonShortenAbsSign(primaryMotor);
    const int8_t secondaryShortenSign = getMotorTendonShortenAbsSign(secondaryMotor);
    const int16_t primaryTightLoadThreshold = kDualPrimaryTightLoadThreshold;
    const int16_t secondaryPreloadLoadThreshold = kDualSecondaryPreloadLoadThreshold;
    const int16_t secondaryFinalLoadThreshold = kDualSecondaryFinalLoadThreshold;
    const int16_t primaryFinalLoadThreshold = kDualPrimaryFinalLoadThreshold;

    uint16_t initialRaw = 0;
    int32_t initialEncoder = 0;
    int32_t primaryAbs = 0;
    int32_t secondaryAbs = 0;
    if (!waitDualInitialFeedback(sharedData,
                                 jointIndex,
                                 primaryMotor,
                                 secondaryMotor,
                                 &primaryAbs,
                                 &secondaryAbs,
                                 &initialRaw,
                                 &initialEncoder,
                                 params)) {
        FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL DUAL_INIT J%02u M%02u/M%02u missing feedback\r\n",
                      jointIndex,
                      primaryMotor,
                      secondaryMotor);
        return false;
    }

    FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_BEGIN J%02u primary=M%02u primaryShortenSign=%d secondary=M%02u secondaryShortenSign=%d slack=%ld primaryLoad=%d secondaryPreloadLoad=%d secondaryFinalLoad=%d primaryFinalLoad=%d startDeg=%.3f tightDeg=dynamic initialEncoder=%ld primaryAbs=%ld secondaryAbs=%ld\r\n",
                  jointIndex,
                  primaryMotor,
                  (int)primaryShortenSign,
                  secondaryMotor,
                  (int)secondaryShortenSign,
                  (long)kDualInitialSlackCounts,
                   primaryTightLoadThreshold,
                   secondaryPreloadLoadThreshold,
                   secondaryFinalLoadThreshold,
                   primaryFinalLoadThreshold,
                   kLutStartDeg,
                  (long)initialEncoder,
                  (long)primaryAbs,
                  (long)secondaryAbs);

    int32_t primary80Encoder = 0;
    int32_t primary80Abs = 0;
    uint16_t primary80Raw = 0;
    int16_t primary80Load = 0;
    const bool workspaceBMatchesMotors = workspaceState && workspaceState->valid &&
        workspaceState->currentPose == DUAL_WORKSPACE_POSE_B &&
        workspaceState->primaryMotor == primaryMotor &&
        workspaceState->secondaryMotor == secondaryMotor;
    bool reuseWorkspaceStart = false;
    if (workspaceBMatchesMotors) {
        const int32_t primaryWorkspaceError =
            abs32(primaryAbs - workspaceState->primaryTightAbs);
        const int32_t secondaryWorkspaceError =
            abs32(secondaryAbs - workspaceState->secondaryReleaseAbs);
        if (primaryWorkspaceError <= kWorkspaceTargetToleranceCounts &&
            secondaryWorkspaceError <= kWorkspaceTargetToleranceCounts) {
            reuseWorkspaceStart = true;
        } else {
            FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_TARGET_B_INVALIDATE J%02u M%02u err=%ld M%02u err=%ld tolerance=%ld\r\n",
                               (unsigned)jointIndex,
                               (unsigned)primaryMotor,
                               (long)primaryWorkspaceError,
                               (unsigned)secondaryMotor,
                               (long)secondaryWorkspaceError,
                               (long)kWorkspaceTargetToleranceCounts);
            workspaceState->valid = 0;
        }
    }
    if (reuseWorkspaceStart) {
        int16_t primaryLoadRaw = 0;
        const bool primaryLoadAvailable =
            readLatestMotorServoLoad(sharedData, primaryMotor, &primaryLoadRaw);
        primary80Abs = primaryAbs;
        primary80Raw = initialRaw;
        primary80Encoder = initialEncoder;
        // B 是位置姿态缓存。上一条收紧命令稳定后，舵机瞬时 load 常会回到零，
        // 因此是否允许复用只由已验证的 ABS 姿态决定，不能用瞬时负载否定缓存。
        primary80Load = primaryLoadAvailable ? mmhand::servo::loadMagnitude(primaryLoadRaw) : 0;
        FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_WORKSPACE_B_REUSE J%02u primary=M%02u abs=%ld load=%d available=%u secondary=M%02u abs=%ld; ABS matched, skip duplicate slack/POS80\r\n",
                           (unsigned)jointIndex,
                           (unsigned)primaryMotor,
                           (long)primary80Abs,
                           (int)primary80Load,
                           primaryLoadAvailable ? 1U : 0U,
                           (unsigned)secondaryMotor,
                           (long)secondaryAbs);
    } else {
        const int32_t secondarySlackTarget =
            motorApplyTendonDelta(secondaryMotor, secondaryAbs, -kDualInitialSlackCounts);
        FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_SLACK_SECONDARY_BEGIN J%02u M%02u targetAbs=%ld from=%ld shortenSign=%d\r\n",
                      jointIndex,
                      secondaryMotor,
                      (long)secondarySlackTarget,
                      (long)secondaryAbs,
                      (int)secondaryShortenSign);
        if (!waitMotorTargetReached(sharedData,
                                    jointIndex,
                                    "DUAL_SLACK_SECONDARY",
                                    secondaryMotor,
                                    secondarySlackTarget,
                                    params)) {
            return false;
        }

        if (!readLatestEncoderOriented(sharedData, jointIndex, &initialEncoder) ||
            !readLatestMotorServoAbs(sharedData, primaryMotor, &primaryAbs)) {
            FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL DUAL_PRIMARY_START J%02u missing feedback\r\n",
                          jointIndex);
            return false;
        }
        if (!findDualTightPoint(sharedData,
                                jointIndex,
                                "POS80",
                                primaryMotor,
                                primaryShortenSign,
                                primaryAbs,
                                initialEncoder,
                                primaryTightLoadThreshold,
                                false,
                                0,
                                0,
                                params,
                                &primary80Abs,
                                &primary80Raw,
                                &primary80Encoder,
                                &primary80Load)) {
            return false;
        }
    }

    int32_t secondaryStartAbs = 0;
    int32_t secondaryStartEncoder = 0;
    if (!readLatestMotorServoAbs(sharedData, secondaryMotor, &secondaryStartAbs) ||
        !readLatestEncoderOriented(sharedData, jointIndex, &secondaryStartEncoder)) {
        FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL DUAL_SECONDARY_START J%02u missing feedback\r\n",
                      jointIndex);
        return false;
    }

    int32_t neg3Encoder = 0;
    int32_t secondaryNeg3Abs = 0;
    int32_t primaryNeg3Abs = primary80Abs;
    uint16_t neg3Raw = 0;
    int16_t neg3Load = 0;
    int32_t secondaryPreloadAbs = secondaryStartAbs;
    uint16_t secondaryPreloadRaw = 0;
    int32_t secondaryPreloadEncoder = secondaryStartEncoder;
    int16_t secondaryPreloadLoad = 0;
    TendonRawFitBuffer fitSamples;
    resetTendonRawFitBuffer(&fitSamples);
    if (!findDualSecondaryPreloadPoint(sharedData,
                                       jointIndex,
                                       secondaryMotor,
                                       secondaryShortenSign,
                                       secondaryStartAbs,
                                       primaryMotor,
                                       primary80Abs,
                                       secondaryPreloadLoadThreshold,
                                       params,
                                       &secondaryPreloadAbs,
                                       &secondaryPreloadRaw,
                                       &secondaryPreloadEncoder,
                                       &secondaryPreloadLoad)) {
        return false;
    }

    int32_t exchangeStartEncoder = 0;
    if (!readLatestEncoderOriented(sharedData, jointIndex, &exchangeStartEncoder)) {
        FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL DUAL_EXCHANGE_START J%02u missing encoder\r\n",
                      jointIndex);
        return false;
    }
    appendTendonRawFitSample(&fitSamples,
                                 exchangeStartEncoder,
                                 primary80Abs,
                                 secondaryPreloadAbs);
    fitSamples.nextTravelDeg = kTendonFitSampleStepDeg;

    if (!findDualExchangeLimitPoint(sharedData,
                                    jointIndex,
                                    secondaryMotor,
                                    secondaryShortenSign,
                                    secondaryPreloadAbs,
                                    primaryMotor,
                                    primaryShortenSign,
                                    primary80Abs,
                                    exchangeStartEncoder,
                                    secondaryFinalLoadThreshold,
                                    params,
                                    &secondaryNeg3Abs,
                                    &primaryNeg3Abs,
                                    &neg3Raw,
                                    &neg3Encoder,
                                    &neg3Load,
                                    &fitSamples)) {
        return false;
    }
    appendTendonRawFitSample(&fitSamples,
                                 neg3Encoder,
                                 primaryNeg3Abs,
                                 secondaryNeg3Abs);

    // 上面记录的机械 Auto Min/LUT 端点必须保留。READY 使用另一套双绳均有
    // 张力的最终姿态：副电机已达到 250，随后保持副电机交换终点命令，
    // 再把主电机恢复到负载 240。
    int32_t primaryReadyAbs = primaryNeg3Abs;
    int32_t secondaryReadyAbs = secondaryNeg3Abs;
    uint16_t readyRaw = neg3Raw;
    int32_t readyEncoder = neg3Encoder;
    int16_t primaryReadyLoad = 0;
    int16_t secondaryReadyLoad = neg3Load;
    if (!finishDualTendonReadyPreload(sharedData,
                                     jointIndex,
                                     primaryMotor,
                                     primaryNeg3Abs,
                                     primaryFinalLoadThreshold,
                                     secondaryMotor,
                                     secondaryNeg3Abs,
                                     params,
                                     &primaryReadyAbs,
                                     &secondaryReadyAbs,
                                     &readyRaw,
                                     &readyEncoder,
                                     &primaryReadyLoad,
                                     &secondaryReadyLoad)) {
        return false;
    }

    const int32_t signedSpan = signedEncoderDeltaLocal(neg3Encoder, primary80Encoder);
    const int8_t encoderTravelSign = 1;
    const int32_t resultStartEncoder =
        encoderForPositiveTravelResult(neg3Encoder, encoderTravelSign);
    const int32_t resultTightEncoder =
        encoderForPositiveTravelResult(primary80Encoder, encoderTravelSign);
    const int32_t resultSpan =
        calibrationForwardEncoderDelta(resultStartEncoder, resultTightEncoder);
    if (resultSpan <= 0 || resultSpan >= kRejectNearFullWrapSpanCounts) {
        FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL DUAL_RESULT J%02u invalid span=%ld neg3=%ld pos80=%ld signed=%ld\r\n",
                      jointIndex,
                      (long)resultSpan,
                      (long)neg3Encoder,
                      (long)primary80Encoder,
                      (long)signedSpan);
        return false;
    }

    const float dynamicDualTotalDeg = calibrationEncoderCountToDeg(resultSpan);
    const float zeroFraction =
        (dynamicDualTotalDeg > 0.0f)
            ? ((kLutZeroDeg - kLutStartDeg) /
               dynamicDualTotalDeg)
            : 0.0f;
    const int32_t resultZeroEncoder =
        calibrationWrapEncoderCount(
            resultStartEncoder + roundToInt32Local((float)resultSpan * zeroFraction));
    JointCalibrationResult result;
    if (!buildDualZeroTightCalibrationResult(jointIndex,
                                             resultStartEncoder,
                                             resultZeroEncoder,
                                             resultTightEncoder,
                                             secondaryNeg3Abs,
                                             primary80Abs,
                                             primary80Abs,
                                             &result)) {
        FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL DUAL_RESULT_BUILD J%02u\r\n", jointIndex);
        return false;
    }
    const bool lutBuilt =
        buildTendonLutFromRawSamples(&result,
                                     primaryMotor,
                                     secondaryMotor,
                                     2,
                                     fitSamples);
    EncoderUrdfMapResult readyMap = {};
    if (!mapManualScaledEncoderToUrdf(
            jointIndex,
            result,
            g_manualEncoderReference[jointIndex],
            readyRaw,
            g_encoderDirection[jointIndex],
            &readyMap) ||
        !readyMap.valid ||
        readyMap.outsideCounts < -kFeedbackReadyEnterOutsideToleranceCounts) {
        FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL DUAL_FINAL_READY_MAP J%02u raw=%u oriented=%ld valid=%u ready=%u outside=%ld enterTolerance=%ld\r\n",
                           jointIndex,
                           (unsigned)readyRaw,
                           (long)readyEncoder,
                           readyMap.valid ? 1U : 0U,
                           readyMap.feedbackReady ? 1U : 0U,
                           (long)readyMap.outsideCounts,
                           (long)kFeedbackReadyEnterOutsideToleranceCounts);
        setCalibrationDiag(sharedData,
                           CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED,
                           CALIB_DIAG_PHASE_DUAL_PRELOAD,
                           jointIndex,
                           primaryMotor,
                           readyEncoder,
                           readyMap.outsideCounts,
                           secondaryMotor,
                           (1UL << primaryMotor) | (1UL << secondaryMotor));
        return false;
    }

    FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_ENCODER_RESULT J%02u defaultDir=%d runtimeDir=%d signed=%ld resultStart=%ld resultZero=%ld resultTight=%ld\r\n",
                  jointIndex,
                  (int)kDefaultEncoderDirection[jointIndex],
                  (int)g_encoderDirection[jointIndex],
                  (long)signedSpan,
                  (long)result.startEncoder,
                  (long)result.zeroEncoder,
                  (long)result.tightEncoder);
    FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_LUT J%02u built=%u rawSamples=%u lutSamples=%u startDeg=%.3f stepDeg=%.3f primary=M%02u secondary=M%02u\r\n",
                  jointIndex,
                  lutBuilt ? 1U : 0U,
                  (unsigned)fitSamples.count,
                  (unsigned)result.tendonLutCount,
                  result.tendonLutAngleStartDeg,
                  result.tendonLutAngleStepDeg,
                  primaryMotor,
                  secondaryMotor);

    g_jointCalibResult[jointIndex] = result;
    updateCalibrationAssemblyWarning(sharedData, jointIndex, result);
    syncJointTargetToDeg(sharedData, jointIndex, readyMap.urdfDeg);
    if (!commitJointServoMultiTurnTrust(sharedData, jointIndex)) {
        return false;
    }
    const int32_t primaryLimitTightAbs =
        motorRawAbsToTendonAbs(primaryMotor, primaryReadyAbs) >
                motorRawAbsToTendonAbs(primaryMotor, primary80Abs)
            ? primaryReadyAbs
            : primary80Abs;
    const int32_t secondaryLimitTightAbs =
        motorRawAbsToTendonAbs(secondaryMotor, secondaryReadyAbs) >
                motorRawAbsToTendonAbs(secondaryMotor, secondaryNeg3Abs)
            ? secondaryReadyAbs
            : secondaryNeg3Abs;
    setCalibratedMotorAbsLimit(sharedData, primaryMotor, primaryLimitTightAbs);
    setCalibratedMotorAbsLimit(sharedData, secondaryMotor, secondaryLimitTightAbs);

    if (workspaceState) {
        // 缓存最终双绳预紧姿态，而不是主电机处于放绳状态的早期机械端点。
        // 后续关节可直接复用该 A 姿态，不会再次引入松绳。
        workspaceState->primaryMotor = primaryMotor;
        workspaceState->secondaryMotor = secondaryMotor;
        workspaceState->primaryReleaseAbs = primaryReadyAbs;
        workspaceState->secondaryTightAbs = secondaryReadyAbs;
        workspaceState->currentPose = DUAL_WORKSPACE_POSE_A;
        workspaceState->valid = 1;
        FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_POST_CALIB_CACHE J%02u pose=A primaryReady=M%02u:%ld secondaryReady=M%02u:%ld load=%d/%d\r\n",
                           (unsigned)jointIndex,
                           (unsigned)primaryMotor,
                           (long)primaryReadyAbs,
                           (unsigned)secondaryMotor,
                           (long)secondaryReadyAbs,
                           primaryReadyLoad,
                           secondaryReadyLoad);
    }

    FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_RESULT J%02u neg3Raw=%u neg3=%ld pos80Raw=%u pos80=%ld span=%ld zero=%ld primary80Abs=%ld primaryNeg3Abs=%ld secondaryPreloadRaw=%u secondaryPreloadEncoder=%ld secondaryPreloadAbs=%ld secondaryNeg3Abs=%ld loadPrimary=%d loadSecondaryPreload=%d loadSecondaryNeg3=%d readyRaw=%u readyEncoder=%ld readyDeg=%.3f primaryReadyAbs=%ld primaryReadyLoad=%d secondaryReadyAbs=%ld secondaryReadyLoad=%d\r\n",
                  jointIndex,
                  (unsigned)neg3Raw,
                  (long)result.startEncoder,
                  (unsigned)primary80Raw,
                  (long)result.tightEncoder,
                  (long)result.encoderSpan,
                  (long)result.zeroEncoder,
                  (long)primary80Abs,
                  (long)primaryNeg3Abs,
                  (unsigned)secondaryPreloadRaw,
                  (long)secondaryPreloadEncoder,
                  (long)secondaryPreloadAbs,
                  (long)secondaryNeg3Abs,
                   primary80Load,
                   secondaryPreloadLoad,
                   neg3Load,
                   (unsigned)readyRaw,
                   (long)readyEncoder,
                   readyMap.urdfDeg,
                   (long)primaryReadyAbs,
                   primaryReadyLoad,
                   (long)secondaryReadyAbs,
                   secondaryReadyLoad);
    FW_LOG_INFO_PRINTF("[AUTO CALIB] AUTO_RANGE_DYNAMIC J%02u startDeg=%.3f zeroDeg=0.000 tightDeg=%.3f totalDeg=%.3f angleMax=%.3f span=%ld\r\n",
                  jointIndex,
                  result.startDeg,
                  result.tightDeg,
                  result.totalDeg,
                  result.angleMax,
                  (long)result.encoderSpan);

    return true;
}

// 将单电机上下文置为失败并记录简短日志。
static void failAutoCalibContext(JointCalibContext* ctx, const char* reason)
{
    if (!ctx || ctx->state == AUTO_CALIB_STATE_FAILED) {
        return;
    }
    FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL J%02u %s\r\n",
                  ctx->jointIndex,
                  reason ? reason : "unknown");
    ctx->state = AUTO_CALIB_STATE_FAILED;
}

// 将单电机上下文置为失败，同时写入上位机可解析的结构化诊断。
static void failAutoCalibContextWithDiag(JointCalibContext* ctx,
                                         TaskSharedData_t* sharedData,
                                         const char* reasonText,
                                         uint8_t reason,
                                         uint8_t phase,
                                         uint8_t motor,
                                         int32_t valueA,
                                         int32_t valueB,
                                         int32_t valueC,
                                         uint32_t bitmap)
{
    if (!ctx || ctx->state == AUTO_CALIB_STATE_FAILED) {
        return;
    }
    setCalibrationDiag(sharedData,
                       reason,
                       phase,
                       ctx->jointIndex,
                       motor,
                       valueA,
                       valueB,
                       valueC,
                       bitmap);
    failAutoCalibContext(ctx, reasonText);
}

// 首帧反馈有效后初始化紧端搜索基线、命令方向和停稳窗口。
static bool initTightenStateFromFeedback(JointCalibContext* ctx,
                                         TaskSharedData_t* sharedData,
                                         const AutoCalibrationParams& params)
{
    if (!ctx || !sharedData) {
        return false;
    }

    uint16_t raw = 0;
    int32_t encoder = 0;
    int32_t absNow = 0;
    if (!readLatestEncoderSample(sharedData, ctx->jointIndex, &raw, &encoder) ||
        !readLatestServoAbs(sharedData, ctx->bus, ctx->servoId, &absNow)) {
        return false;
    }

    int16_t baselineLoadRaw = 0;
    const bool hasBaselineLoad =
        readLatestServoLoad(sharedData, ctx->bus, ctx->servoId, &baselineLoadRaw);
    ctx->baselineLoad = hasBaselineLoad ? mmhand::servo::loadMagnitude(baselineLoadRaw) : 0;
    ctx->startSearchRaw = raw;
    ctx->startSearchEncoder = encoder;
    ctx->startSearchAbs = absNow;
    ctx->startFound = false;
    ctx->zeroFound = false;
    ctx->startLoadStable = 0;
    ctx->startEncoder = 0;
    ctx->startServoAbs = 0;
    ctx->zeroEncoder = 0;
    ctx->zeroServoAbs = 0;
    ctx->encoderTravelSign = 1;
    ctx->windowStartEncoder = encoder;
    ctx->windowStartAbs = absNow;
    ctx->windowStartMs = millis();
    ctx->commandAbs = absNow;
    ctx->commandRemainder = 0;
    ctx->lastCommandMs = millis();
    ctx->lastTraceMs = 0;
    ctx->stateBeginMs = millis();
    ctx->state = AUTO_CALIB_STATE_TIGHTEN;

    FW_LOG_INFO_PRINTF("[AUTO CALIB] AUTO_RANGE_DYNAMIC_BEGIN J%02u startDeg=%.3f tightDeg=dynamic totalDeg=dynamic angleMax=dynamic\r\n",
                  ctx->jointIndex,
                  kLutStartDeg);
    FW_LOG_INFO_PRINTF("[AUTO CALIB] START J%02u bus%u id%u abs=%ld raw=%u oriented=%ld dir=%d baselineLoad=%d speedCps=%ld\r\n",
                  ctx->jointIndex,
                  ctx->bus,
                  ctx->servoId,
                  (long)ctx->startSearchAbs,
                  (unsigned)ctx->startSearchRaw,
                  (long)ctx->startSearchEncoder,
                  (int)g_encoderDirection[ctx->jointIndex],
                  ctx->baselineLoad,
                  (long)params.tightenSpeedCountsPerSec);
    return true;
}

// 初始化一个单电机标定上下文，不读取硬件、不发送命令。
static void initAutoCalibContext(JointCalibContext* ctx, uint8_t jointIndex)
{
    if (!ctx) {
        return;
    }
    memset(ctx, 0, sizeof(*ctx));
    ctx->state = AUTO_CALIB_STATE_INIT;
    ctx->jointIndex = jointIndex;
    ctx->motorChannel = 0xFF;
    resetTendonRawFitBuffer(&ctx->fitSamples);
}

// ============================================================================
// stepAutoCalibrationJoint - 单关节自动标定的并行状态机步进函数
// ============================================================================
// 核心逻辑（非阻塞）：
// 单电机非阻塞状态机单步函数。
// INIT 等待首帧；TIGHTEN 搜索负载/角度停稳的紧端；PRE_RELEASE_SYNC 把目标
// 锁回实际 ABS 并停稳；随后反向松到机械端点，生成 V3 映射和单路 Tendon LUT。
static void stepAutoCalibrationJoint(JointCalibContext* ctx,
                                     TaskSharedData_t* sharedData,
                                     const AutoCalibrationParams& params,
                                     ServoTargetBatch_t* batch)
{
    if (!ctx ||
        ctx->state == AUTO_CALIB_STATE_DONE ||
        ctx->state == AUTO_CALIB_STATE_FAILED) {
        return;
    }

    const uint32_t nowMs = millis();
    if (!autoCalibrationStillOwnsControl(sharedData)) {
        FW_LOG_INFO_PRINTF("[AUTO CALIB] CANCEL J%02u state=%u owner=%u enabled=%u\r\n",
                      ctx->jointIndex,
                      sharedData ? (unsigned)sharedData->system_state : 255,
                      sharedData ? (unsigned)sharedData->servo_target_owner : 255,
                      sharedData ? (unsigned)sharedData->control_enabled : 255);
        ctx->state = AUTO_CALIB_STATE_FAILED;
        return;
    }

    switch (ctx->state) {
        case AUTO_CALIB_STATE_INIT:
        {
            if (!sharedData ||
                ctx->jointIndex >= ENCODER_TOTAL_NUM ||
                sharedData->servo_target_owner != SERVO_TARGET_OWNER_CALIBRATION) {
                failAutoCalibContext(ctx, "invalid init or owner");
                return;
            }

            // 当前非阻塞标定状态机直接使用本轮 AutoCalibrationParams，
            // 不再经过已经淘汰的逐关节静态参数表。
            ctx->commandSpeed = params.servoSpeed;
            ctx->commandAcc = params.servoAcc;
            ctx->bus = jointMap[ctx->jointIndex].busIndex;
            ctx->servoId = jointMap[ctx->jointIndex].servoID;
            const int motorChannel = findMotorChannel(ctx->bus, ctx->servoId);
            if (motorChannel < 0 || motorChannel >= SERVO_TOTAL_NUM) {
                failAutoCalibContext(ctx, "primary motor channel not mapped");
                return;
            }
            ctx->motorChannel = (uint8_t)motorChannel;
            ctx->directionSign = getMotorTendonShortenAbsSign(ctx->motorChannel);
            if (!ctx->initStarted) {
                ctx->initStarted = true;
                ctx->stateBeginMs = nowMs;
                ctx->lastCommandMs = 0;
                ctx->lastTraceMs = 0;
                setRuntimeEncoderDirection(ctx->jointIndex,
                                           kDefaultEncoderDirection[ctx->jointIndex]);
                clearCalibratedMotorAbsLimitsForJoint(sharedData, ctx->jointIndex);
            }

            if (!isJointPrimaryServoMultiTurnKnown(sharedData, ctx->jointIndex)) {
                FW_LOG_INFO_PRINTF("[AUTO CALIB] WARN J%02u multi-turn unknown; auto calibration will rebuild zero from current relative abs unknown=0x%08lX\r\n",
                              ctx->jointIndex,
                              (unsigned long)sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap));
            }

            uint16_t raw = 0;
            int32_t encoder = 0;
            int32_t absNow = 0;
            int16_t rawServo = 0;
            const bool encoderOk =
                readLatestEncoderSample(sharedData, ctx->jointIndex, &raw, &encoder);
            const bool absOk =
                readLatestServoAbs(sharedData, ctx->bus, ctx->servoId, &absNow);
            const bool rawOk =
                readLatestServoRaw(sharedData, ctx->bus, ctx->servoId, &rawServo);
            if (!encoderOk || !absOk || !rawOk) {
                if ((nowMs - ctx->lastTraceMs) >= params.traceMs) {
                    ctx->lastTraceMs = nowMs;
                    logInitialFeedbackDiagnostic(sharedData,
                                               ctx->jointIndex,
                                               ctx->bus,
                                               ctx->servoId,
                                               encoderOk,
                                               absOk,
                                               rawOk);
                }
                if ((uint32_t)(nowMs - ctx->stateBeginMs) < kInitialFeedbackWaitMs) {
                    return;
                }
                logInitialFeedbackDiagnostic(sharedData,
                                           ctx->jointIndex,
                                           ctx->bus,
                                           ctx->servoId,
                                           encoderOk,
                                           absOk,
                                           rawOk);
                failAutoCalibContextWithDiag(ctx,
                                             sharedData,
                                             "missing initial encoder/servo feedback after wait",
                                             !encoderOk ? CALIB_DIAG_REASON_ENCODER_ERROR : CALIB_DIAG_REASON_SERVO_OFFLINE,
                                             CALIB_DIAG_PHASE_INIT,
                                             ctx->motorChannel,
                                             encoderOk ? 1 : 0,
                                             absOk ? 1 : 0,
                                             rawOk ? 1 : 0,
                                             sharedData
                                                 ? sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap)
                                                 : 0);
                return;
            }

            const int32_t turn = servoTurnFromAbs(absNow);
            if (turn != 0) {
                FW_LOG_INFO_PRINTF("[AUTO CALIB] SKIP_NORMALIZE_TURN J%02u bus%u id%u abs=%ld raw=%d turn=%ld; calibrate from current abs\r\n",
                              ctx->jointIndex,
                              ctx->bus,
                              ctx->servoId,
                              (long)absNow,
                              (int)rawServo,
                              (long)turn);
            }

            if (!initTightenStateFromFeedback(ctx, sharedData, params)) {
                failAutoCalibContextWithDiag(ctx,
                                             sharedData,
                                             "missing feedback after init",
                                             CALIB_DIAG_REASON_FEEDBACK_LOST,
                                             CALIB_DIAG_PHASE_INIT,
                                             ctx->motorChannel,
                                             0,
                                             0,
                                             0,
                                             0);
            }
            return;
        }

        case AUTO_CALIB_STATE_TIGHTEN:
        {
            uint16_t currentRaw = 0;
            int32_t currentEncoder = 0;
            int32_t currentAbs = 0;
            if (!readLatestEncoderSample(sharedData, ctx->jointIndex, &currentRaw, &currentEncoder) ||
                !readLatestServoAbs(sharedData, ctx->bus, ctx->servoId, &currentAbs)) {
                failAutoCalibContextWithDiag(ctx,
                                             sharedData,
                                             "lost encoder/servo feedback",
                                             CALIB_DIAG_REASON_FEEDBACK_LOST,
                                             ctx->startFound ? CALIB_DIAG_PHASE_RELEASE : CALIB_DIAG_PHASE_FIND_MAX,
                                             ctx->motorChannel,
                                             0,
                                             0,
                                             0,
                                             0);
                return;
            }

            int16_t loadRaw = 0;
            const bool hasLoad = readLatestServoLoad(sharedData, ctx->bus, ctx->servoId, &loadRaw);
            const int16_t loadMag = hasLoad ? mmhand::servo::loadMagnitude(loadRaw) : 0;
            const int32_t totalAbsDelta = abs32(currentAbs - ctx->startSearchAbs);
            const int32_t totalEncoderDelta =
                calibrationWrapEncoderDelta(currentEncoder, ctx->startSearchEncoder);
            const float totalAngleDeltaDeg =
                fabsf(calibrationEncoderCountToDeg(totalEncoderDelta));
            const bool loadHigh = hasLoad && loadMag >= params.tightLoadHigh;

            if (ctx->startFound) {
                const int32_t actualReleaseCounts =
                    singleTendonReleaseCounts(ctx->motorChannel,
                                              ctx->startServoAbs,
                                              currentAbs);
                if (actualReleaseCounts > kSingleTendonMaxReleaseCounts) {
                    // 超限后只保持实测 ABS，不再沿旧的松绳目标继续推进。
                    ctx->commandAbs = currentAbs;
                    appendJointCalibrationTargetToBatch(
                        batch,
                        ctx->jointIndex,
                        clampToServoPosLocal(currentAbs),
                        ctx->commandSpeed,
                        ctx->commandAcc);
                    failAutoCalibContextWithDiag(
                        ctx,
                        sharedData,
                        "single tendon release exceeded two turns",
                        CALIB_DIAG_REASON_RELEASE_OVER_TWO_TURNS,
                        CALIB_DIAG_PHASE_RELEASE,
                        ctx->motorChannel,
                        actualReleaseCounts,
                        kSingleTendonMaxReleaseCounts,
                        actualReleaseCounts,
                        1UL << ctx->motorChannel);
                    return;
                }

                // 单电机从紧端向松端运动期间，约每 5 度保存一次磁编位置与
                // 舵机 raw ABS，供标定结束后重采样成等间隔 Tendon LUT。
                const int32_t releaseDelta =
                    calibrationWrapEncoderDelta(currentEncoder, ctx->startEncoder);
                const float releaseTravelDeg =
                    fabsf(calibrationEncoderCountToDeg(releaseDelta));
                maybeAppendTendonTravelFitSample(&ctx->fitSamples,
                                                 releaseTravelDeg,
                                                 currentEncoder,
                                                 currentAbs,
                                                 0);
            }

            if ((nowMs - ctx->lastTraceMs) >= params.traceMs) {
                ctx->lastTraceMs = nowMs;
                FW_LOG_INFO_PRINTF("[AUTO CALIB] TRACE J%02u phase=%s cmd=%ld abs=%ld raw=%u oriented=%ld dCount=%ld totalAbs=%ld totalDeg=%.3f load=%d startFound=%u elapsed=%lu\r\n",
                              ctx->jointIndex,
                              ctx->startFound ? "RELEASE_TO_TIGHT" : "FIND_MAX",
                              (long)ctx->commandAbs,
                              (long)currentAbs,
                              (unsigned)currentRaw,
                              (long)currentEncoder,
                              (long)totalEncoderDelta,
                              (long)totalAbsDelta,
                              totalAngleDeltaDeg,
                              loadMag,
                              ctx->startFound ? 1 : 0,
                              (unsigned long)(nowMs - ctx->stateBeginMs));
            }

            if (!ctx->startFound) {
                bool maxFoundByStall = false;
                if ((nowMs - ctx->windowStartMs) >= params.windowMs) {
                    const int32_t windowAbsDelta = abs32(currentAbs - ctx->windowStartAbs);
                    const int32_t windowEncoderDelta =
                        calibrationWrapEncoderDelta(currentEncoder, ctx->windowStartEncoder);
                    const float windowAngleDeltaDeg =
                        fabsf(calibrationEncoderCountToDeg(windowEncoderDelta));
                    const bool servoStillTightening =
                        windowAbsDelta >= params.windowServoDeltaCounts;
                    const bool angleStalled =
                        windowAngleDeltaDeg <= params.stallActualDeg;
                    FW_LOG_INFO_PRINTF("[AUTO CALIB] CHECK_MAX J%02u abs=%ld raw=%u oriented=%ld dCount=%ld winCount=%ld winAbs=%ld/%ld winDeg=%.3f/%.3f totalDeg=%.3f load=%d/%d cond servo=%u angle=%u requireTravel=0 load=%u\r\n",
                                  ctx->jointIndex,
                                  (long)currentAbs,
                                  (unsigned)currentRaw,
                                  (long)currentEncoder,
                                  (long)totalEncoderDelta,
                                  (long)windowEncoderDelta,
                                  (long)windowAbsDelta,
                                  (long)params.windowServoDeltaCounts,
                                  windowAngleDeltaDeg,
                                  params.stallActualDeg,
                                  totalAngleDeltaDeg,
                                  loadMag,
                                  params.tightLoadHigh,
                                  servoStillTightening ? 1 : 0,
                                  angleStalled ? 1 : 0,
                                  loadHigh ? 1 : 0);

                    maxFoundByStall =
                        loadHigh && servoStillTightening && angleStalled;
                    ctx->windowStartAbs = currentAbs;
                    ctx->windowStartEncoder = currentEncoder;
                    ctx->windowStartMs = nowMs;
                }

                if (maxFoundByStall) {
                    ctx->encoderTravelSign = 1;
                    ctx->startFound = true;
                    ctx->startEncoder = currentEncoder;
                    ctx->startServoAbs = currentAbs;
                    ctx->windowStartEncoder = currentEncoder;
                    ctx->windowStartAbs = currentAbs;
                    ctx->windowStartMs = nowMs;
                    ctx->lastCommandMs = nowMs;
                    ctx->commandAbs = currentAbs;
                    ctx->commandRemainder = 0;
                    ctx->releaseCommandStarted = false;
                    ctx->preReleaseSyncTargetAbs = currentAbs;
                    ctx->preReleaseSyncWindowStartAbs = currentAbs;
                    ctx->preReleaseSyncWindowStartMs = nowMs;
                    ctx->preReleaseSyncStableSamples = 0;
                    ctx->releaseAngleStallStable = 0;
                    ctx->state = AUTO_CALIB_STATE_PRE_RELEASE_SYNC;
                    if (!appendJointCalibrationTargetToBatch(batch,
                                                             ctx->jointIndex,
                                                             clampToServoPosLocal(ctx->preReleaseSyncTargetAbs),
                                                             ctx->commandSpeed,
                                                             ctx->commandAcc)) {
                        failAutoCalibContextWithDiag(ctx,
                                                     sharedData,
                                                     "pre-release sync target append failed",
                                                     CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
                                                     CALIB_DIAG_PHASE_RELEASE,
                                                     ctx->motorChannel,
                                                     ctx->preReleaseSyncTargetAbs,
                                                     0,
                                                     0,
                                                     0);
                        return;
                    }
                    FW_LOG_INFO_PRINTF("[AUTO CALIB] START_FOUND J%02u maxDeg=dynamic raw=%u oriented=%ld abs=%ld load=%d byLoad=%u byStall=%u totalDeg=%.3f loadThreshold=%d encTravelSign=%d\r\n",
                                  ctx->jointIndex,
                                  (unsigned)currentRaw,
                                  (long)ctx->startEncoder,
                                  (long)ctx->startServoAbs,
                                  loadMag,
                                  loadHigh ? 1 : 0,
                                  maxFoundByStall ? 1 : 0,
                                  totalAngleDeltaDeg,
                                  params.tightLoadHigh,
                                  (int)ctx->encoderTravelSign);
                    FW_LOG_INFO_PRINTF("[AUTO CALIB] PRE_RELEASE_SYNC_BEGIN J%02u targetAbs=%ld tolerance=%ld settleDelta=%ld stable=%u\r\n",
                                  ctx->jointIndex,
                                  (long)ctx->preReleaseSyncTargetAbs,
                                  (long)kPreReleaseSyncTargetToleranceCounts,
                                  (long)kPreReleaseSyncSettleDeltaCounts,
                                  (unsigned)kPreReleaseSyncStableSamples);
                    return;
                }

                const uint32_t dtMs = nowMs - ctx->lastCommandMs;
                if (dtMs >= params.loopMs) {
                    const int32_t numerator =
                        (int32_t)dtMs * params.tightenSpeedCountsPerSec + ctx->commandRemainder;
                    const int32_t stepCounts = numerator / 1000;
                    ctx->commandRemainder = numerator % 1000;
                    if (stepCounts > 0) {
                        ctx->commandAbs = motorApplyTendonDelta(ctx->motorChannel,
                                                                ctx->commandAbs,
                                                                stepCounts);
                        if (!isServoCommandInsideLimit(sharedData,
                                                       ctx->motorChannel,
                                                       ctx->commandAbs)) {
                            int32_t rangeMin = 0;
                            int32_t rangeMax = 0;
                            getServoLogicalCommandRange(
                                sharedData,
                                ctx->motorChannel,
                                &rangeMin,
                                &rangeMax);
                            appendJointCalibrationTargetToBatch(batch,
                                                                ctx->jointIndex,
                                                                clampToServoPosLocal(ctx->commandAbs),
                                                                ctx->commandSpeed,
                                                                ctx->commandAcc);
                            failAutoCalibContextWithDiag(ctx,
                                                         sharedData,
                                                         "find max command outside session range",
                                                         CALIB_DIAG_REASON_ABS_LIMIT_INVALID,
                                                         CALIB_DIAG_PHASE_FIND_MAX,
                                                         ctx->motorChannel,
                                                         ctx->commandAbs,
                                                         rangeMin,
                                                         rangeMax,
                                                         0);
                            return;
                        }
                        if (!appendJointCalibrationTargetToBatch(batch,
                                                                 ctx->jointIndex,
                                                                 clampToServoPosLocal(ctx->commandAbs),
                                                                 ctx->commandSpeed,
                                                                 ctx->commandAcc)) {
                            failAutoCalibContextWithDiag(ctx,
                                                         sharedData,
                                                         "find max target append failed",
                                                         CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
                                                         CALIB_DIAG_PHASE_FIND_MAX,
                                                         ctx->motorChannel,
                                                         ctx->commandAbs,
                                                         0,
                                                         0,
                                                         0);
                            return;
                        }
                    }
                    ctx->lastCommandMs = nowMs;
                }
                return;
            }
            const uint32_t dtMs = nowMs - ctx->lastCommandMs;
            if (dtMs >= params.loopMs) {
                const int32_t numerator =
                    (int32_t)dtMs * params.tightenSpeedCountsPerSec + ctx->commandRemainder;
                const int32_t stepCounts = numerator / 1000;
                ctx->commandRemainder = numerator % 1000;
                if (stepCounts > 0) {
                    const int32_t nextCommandAbs =
                        motorApplyTendonDelta(ctx->motorChannel,
                                             ctx->commandAbs,
                                             -stepCounts);
                    const int32_t actualReleaseCounts =
                        singleTendonReleaseCounts(ctx->motorChannel,
                                                  ctx->startServoAbs,
                                                  currentAbs);
                    const int32_t commandReleaseCounts =
                        singleTendonReleaseCounts(ctx->motorChannel,
                                                  ctx->startServoAbs,
                                                  nextCommandAbs);
                    if (commandReleaseCounts > kSingleTendonMaxReleaseCounts) {
                        // 在下一帧目标越过两圈前终止，避免 cmd 领先实际 ABS 后继续放绳。
                        ctx->commandAbs = currentAbs;
                        appendJointCalibrationTargetToBatch(
                            batch,
                            ctx->jointIndex,
                            clampToServoPosLocal(currentAbs),
                            ctx->commandSpeed,
                            ctx->commandAcc);
                        failAutoCalibContextWithDiag(
                            ctx,
                            sharedData,
                            "single tendon release command would exceed two turns",
                            CALIB_DIAG_REASON_RELEASE_OVER_TWO_TURNS,
                            CALIB_DIAG_PHASE_RELEASE,
                            ctx->motorChannel,
                            actualReleaseCounts,
                            kSingleTendonMaxReleaseCounts,
                            commandReleaseCounts,
                            1UL << ctx->motorChannel);
                        return;
                    }
                    ctx->commandAbs = nextCommandAbs;
                    if (!isServoCommandInsideLimit(sharedData,
                                                   ctx->motorChannel,
                                                   ctx->commandAbs)) {
                        int32_t rangeMin = 0;
                        int32_t rangeMax = 0;
                        getServoLogicalCommandRange(
                            sharedData,
                            ctx->motorChannel,
                            &rangeMin,
                            &rangeMax);
                        appendJointCalibrationTargetToBatch(batch,
                                                            ctx->jointIndex,
                                                            clampToServoPosLocal(ctx->commandAbs),
                                                            ctx->commandSpeed,
                                                            ctx->commandAcc);
                        failAutoCalibContextWithDiag(ctx,
                                                     sharedData,
                                                     "release command outside session range",
                                                     CALIB_DIAG_REASON_ABS_LIMIT_INVALID,
                                                     CALIB_DIAG_PHASE_RELEASE,
                                                     ctx->motorChannel,
                                                     ctx->commandAbs,
                                                     rangeMin,
                                                     rangeMax,
                                                     0);
                        return;
                    }
                    if (!appendJointCalibrationTargetToBatch(batch,
                                                             ctx->jointIndex,
                                                             clampToServoPosLocal(ctx->commandAbs),
                                                             ctx->commandSpeed,
                                                             ctx->commandAcc)) {
                        failAutoCalibContextWithDiag(ctx,
                                                     sharedData,
                                                     "release target append failed",
                                                     CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
                                                     CALIB_DIAG_PHASE_RELEASE,
                                                     ctx->motorChannel,
                                                     ctx->commandAbs,
                                                     0,
                                                     0,
                                                     0);
                        return;
                    }
                    if (!ctx->releaseCommandStarted) {
                        ctx->releaseCommandStarted = true;
                        FW_LOG_INFO_PRINTF("[AUTO CALIB] BEGIN_RELEASE J%02u fromMaxAbs=%ld firstTargetAbs=%ld step=%ld directionSign=%ld targetDeg=%.3f\r\n",
                                      ctx->jointIndex,
                                      (long)ctx->startServoAbs,
                                      (long)ctx->commandAbs,
                                      (long)stepCounts,
                                      (long)ctx->directionSign,
                                      kLutStartDeg);
                    }
                }
                ctx->lastCommandMs = nowMs;
            }

            if ((nowMs - ctx->windowStartMs) >= params.windowMs) {
                const int32_t windowAbsDelta = abs32(currentAbs - ctx->windowStartAbs);
                const int32_t windowEncoderDelta =
                    calibrationWrapEncoderDelta(currentEncoder, ctx->windowStartEncoder);
                const float windowAngleDeltaDeg =
                    fabsf(calibrationEncoderCountToDeg(windowEncoderDelta));
                const bool servoStillReleasing =
                    windowAbsDelta >= params.windowServoDeltaCounts;
                const bool angleStalled =
                    windowAngleDeltaDeg <= params.stallActualDeg;
                if (angleStalled) {
                    if (ctx->releaseAngleStallStable < 255) {
                        ctx->releaseAngleStallStable++;
                    }
                } else {
                    ctx->releaseAngleStallStable = 0;
                }
                const int32_t releaseFromMaxCount =
                    calibrationWrapEncoderDelta(currentEncoder, ctx->startEncoder);
                const float releaseFromMaxDeg =
                    fabsf(calibrationEncoderCountToDeg(releaseFromMaxCount));
                const bool enoughReleaseTravel =
                    releaseFromMaxDeg >= kReverseMinFollowDeg;

                FW_LOG_INFO_PRINTF("[AUTO CALIB] CHECK_RELEASE J%02u abs=%ld raw=%u oriented=%ld releaseCount=%ld releaseDeg=%.3f/%.3f winCount=%ld winAbs=%ld/%ld winDeg=%.3f/%.3f load=%d cond servo=%u angle=%u angleStable=%u/%u travel=%u\r\n",
                              ctx->jointIndex,
                              (long)currentAbs,
                              (unsigned)currentRaw,
                              (long)currentEncoder,
                              (long)releaseFromMaxCount,
                              releaseFromMaxDeg,
                              kReverseMinFollowDeg,
                              (long)windowEncoderDelta,
                              (long)windowAbsDelta,
                              (long)params.windowServoDeltaCounts,
                              windowAngleDeltaDeg,
                              params.stallActualDeg,
                              loadMag,
                              servoStillReleasing ? 1 : 0,
                              angleStalled ? 1 : 0,
                              (unsigned)ctx->releaseAngleStallStable,
                              (unsigned)kAngleStallStableSamples,
                              enoughReleaseTravel ? 1 : 0);

                if (servoStillReleasing &&
                    ctx->releaseAngleStallStable >= kAngleStallStableSamples &&
                    enoughReleaseTravel) {
                    ctx->zeroFound = true;
                    ctx->zeroEncoder = currentEncoder;
                    ctx->zeroServoAbs = currentAbs;
                    const int32_t signedTightToMax =
                        signedEncoderDeltaLocal(ctx->zeroEncoder, ctx->startEncoder);
                    ctx->encoderTravelSign = 1;

                    const int32_t maxEncoder = ctx->startEncoder;
                    const int32_t maxServoAbs = ctx->startServoAbs;
                    const int32_t tightEncoder = ctx->zeroEncoder;
                    const int32_t tightServoAbs = ctx->zeroServoAbs;
                    if (maxServoAbs == tightServoAbs) {
                        failAutoCalibContext(ctx, "zero find has no servo travel");
                        return;
                    }
                    const int32_t resultTautEncoder =
                        encoderForPositiveTravelResult(ctx->zeroEncoder, ctx->encoderTravelSign);
                    const int32_t resultTightEncoder =
                        encoderForPositiveTravelResult(ctx->startEncoder, ctx->encoderTravelSign);
                    const int32_t resultSpan =
                        calibrationForwardEncoderDelta(resultTautEncoder, resultTightEncoder);
                    const float dynamicTotalDeg = calibrationEncoderCountToDeg(resultSpan);
                    const float zeroFraction =
                        (dynamicTotalDeg > 0.0f)
                            ? ((kLutZeroDeg - kLutStartDeg) /
                               dynamicTotalDeg)
                            : 0.0f;
                    const int32_t resultZeroEncoder =
                        calibrationWrapEncoderCount(
                            resultTautEncoder +
                            roundToInt32Local((float)resultSpan * zeroFraction));
                    const int32_t servoZeroAbs =
                        motorApplyTendonDelta(ctx->motorChannel,
                                              tightServoAbs,
                                              kServoZeroPullCounts);

                    // 强制记录松端，确保 LUT 同时包含两个机械端点。
                    appendTendonRawFitSample(&ctx->fitSamples,
                                             tightEncoder,
                                             tightServoAbs,
                                             0);

                    JointCalibrationResult result;
                    if (!buildAutoZeroTightCalibrationResult(ctx->jointIndex,
                                                             resultTautEncoder,
                                                             resultZeroEncoder,
                                                             resultTightEncoder,
                                                             tightServoAbs,
                                                             servoZeroAbs,
                                                             maxServoAbs,
                                                             &result)) {
                        failAutoCalibContextWithDiag(ctx,
                                                     sharedData,
                                                     "invalid max/release result",
                                                     CALIB_DIAG_REASON_ZERO_CROSS_INVALID,
                                                     CALIB_DIAG_PHASE_RELEASE,
                                                     ctx->motorChannel,
                                                     resultTautEncoder,
                                                     resultZeroEncoder,
                                                     resultTightEncoder,
                                                     0);
                        return;
                    }
                    const bool lutBuilt =
                        buildTendonLutFromRawSamples(&result,
                                                     ctx->motorChannel,
                                                     0xFF,
                                                     1,
                                                     ctx->fitSamples);
                    if (!lutBuilt) {
                        failAutoCalibContextWithDiag(ctx,
                                                     sharedData,
                                                     "single tendon LUT build failed",
                                                     CALIB_DIAG_REASON_ZERO_CROSS_INVALID,
                                                     CALIB_DIAG_PHASE_RELEASE,
                                                     ctx->motorChannel,
                                                     ctx->fitSamples.count,
                                                     result.encoderSpan,
                                                     result.tendonLutCount,
                                                     0);
                        return;
                    }
                    int32_t lutMinAbs = 0;
                    int32_t lutMaxAbs = 0;
                    if (result.tendonLutMotor[0] != ctx->motorChannel ||
                        !tendonCalibrationMotorAbsRange(result,
                                                        0,
                                                        &lutMinAbs,
                                                        &lutMaxAbs)) {
                        failAutoCalibContextWithDiag(ctx,
                                                     sharedData,
                                                     "single tendon LUT range invalid",
                                                     CALIB_DIAG_REASON_ZERO_CROSS_INVALID,
                                                     CALIB_DIAG_PHASE_RELEASE,
                                                     ctx->motorChannel,
                                                     result.tendonLutCount,
                                                     result.tendonLutMotor[0],
                                                     lutMinAbs,
                                                     lutMaxAbs);
                        return;
                    }
                    if (result.encoderSpan >= kRejectNearFullWrapSpanCounts) {
                        FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL J%02u rejected wrap-like span=%ld start=%ld tight=%ld; endpoint span is invalid\r\n",
                                      ctx->jointIndex,
                                      (long)result.encoderSpan,
                                      (long)tightEncoder,
                                      (long)maxEncoder);
                        failAutoCalibContextWithDiag(ctx,
                                                     sharedData,
                                                     "rejected wrap-like span",
                                                     CALIB_DIAG_REASON_WRAP_TOO_LARGE,
                                                     CALIB_DIAG_PHASE_RELEASE,
                                                     ctx->motorChannel,
                                                     result.encoderSpan,
                                                     tightEncoder,
                                                     maxEncoder,
                                                     0);
                        return;
                    }

                    FW_LOG_INFO_PRINTF("[AUTO CALIB] ENCODER_RESULT J%02u defaultDir=%d runtimeDir=%d tightToMax=%ld resultTaut=%ld resultZero=%ld resultTight=%ld\r\n",
                                  ctx->jointIndex,
                                  (int)kDefaultEncoderDirection[ctx->jointIndex],
                                  (int)g_encoderDirection[ctx->jointIndex],
                                  (long)signedTightToMax,
                                  (long)resultTautEncoder,
                                  (long)resultZeroEncoder,
                                  (long)resultTightEncoder);
                    FW_LOG_INFO_PRINTF(
                        "[AUTO CALIB] SINGLE_LUT J%02u built=%u rawSamples=%u "
                        "lutSamples=%u startDeg=%.3f stepDeg=%.3f motor=M%02u\r\n",
                        ctx->jointIndex,
                        lutBuilt ? 1U : 0U,
                        (unsigned)ctx->fitSamples.count,
                        (unsigned)result.tendonLutCount,
                        result.tendonLutAngleStartDeg,
                        result.tendonLutAngleStepDeg,
                        ctx->motorChannel);

                    g_jointCalibResult[ctx->jointIndex] = result;
                    updateCalibrationAssemblyWarning(sharedData,
                                                     ctx->jointIndex,
                                                     result);
                    ctx->result = result;
                    ctx->resultValid = true;
                    ctx->targetAbs = currentAbs;
                    syncJointTargetToDeg(sharedData,
                                         ctx->jointIndex,
                                         result.currentMinDeg);
                    if (!commitJointServoMultiTurnTrust(sharedData,
                                                        ctx->jointIndex)) {
                        failAutoCalibContext(ctx,
                                             "multi-turn trust commit failed");
                        return;
                    }
                    setSingleTendonLutAbsLimit(
                        sharedData,
                        ctx->motorChannel,
                        lutMinAbs,
                        lutMaxAbs,
                        result.tendonLutMotorAbs[0][result.tendonLutCount - 1]);
                    ctx->state = AUTO_CALIB_STATE_DONE;
                    ctx->stateBeginMs = nowMs;
                    ctx->lastTraceMs = nowMs;

                    FW_LOG_INFO_PRINTF("[AUTO CALIB] TIGHT_FOUND J%02u startDeg=%.3f raw=%u encoder=%ld abs=%ld load=%d releaseDeg=%.3f winDeg=%.3f\r\n",
                                  ctx->jointIndex,
                                  result.startDeg,
                                  (unsigned)currentRaw,
                                  (long)tightEncoder,
                                  (long)tightServoAbs,
                                  loadMag,
                                  releaseFromMaxDeg,
                                  windowAngleDeltaDeg);
                    FW_LOG_INFO_PRINTF("[AUTO CALIB] AUTO_RANGE_DYNAMIC J%02u startDeg=%.3f zeroDeg=0.000 tightDeg=%.3f totalDeg=%.3f angleMax=%.3f span=%ld\r\n",
                                  ctx->jointIndex,
                                  result.startDeg,
                                  result.tightDeg,
                                  result.totalDeg,
                                  result.angleMax,
                                  (long)result.encoderSpan);
                    FW_LOG_INFO_PRINTF("[AUTO CALIB] ZERO_FOUND J%02u zeroDeg=%.3f encoder=%ld servoZero=%ld formula=tendon(tightAbs)+pullCounts(%ld) shortenSign=%ld\r\n",
                                  ctx->jointIndex,
                                  kLutZeroDeg,
                                  (long)result.zeroEncoder,
                                  (long)result.zeroServoAbs,
                                  (long)kServoZeroPullCounts,
                                  (long)ctx->directionSign);
                    FW_LOG_INFO_PRINTF("[AUTO CALIB] RESULT J%02u start=%ld zero=%ld tight=%ld span=%ld servoStart=%ld servoZero=%ld servoTight=%ld\r\n",
                                  ctx->jointIndex,
                                  (long)result.startEncoder,
                                  (long)result.zeroEncoder,
                                  (long)result.tightEncoder,
                                  (long)result.encoderSpan,
                                  (long)result.startServoAbs,
                                  (long)result.zeroServoAbs,
                                  (long)result.tightServoAbs);
                    FW_LOG_INFO_PRINTF("[AUTO CALIB] SINGLE_DONE_AT_AUTO_MIN J%02u holdAbs=%ld currentMinDeg=%.3f; no return motion\r\n",
                                  ctx->jointIndex,
                                  (long)ctx->targetAbs,
                                  result.currentMinDeg);

                    if (!appendJointCalibrationTargetToBatch(
                            batch,
                            ctx->jointIndex,
                            clampToServoPosLocal(ctx->targetAbs),
                            ctx->commandSpeed,
                            ctx->commandAcc)) {
                        failAutoCalibContextWithDiag(
                            ctx,
                            sharedData,
                            "auto-min hold target append failed",
                            CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
                            CALIB_DIAG_PHASE_RELEASE,
                            ctx->motorChannel,
                            ctx->targetAbs,
                            0,
                            0,
                            0);
                    }
                    return;
                }

                ctx->windowStartAbs = currentAbs;
                ctx->windowStartEncoder = currentEncoder;
                ctx->windowStartMs = nowMs;
            }
            return;
        }

        case AUTO_CALIB_STATE_PRE_RELEASE_SYNC:
        {
            int32_t currentEncoder = 0;
            int32_t currentAbs = 0;
            if (!readLatestEncoderOriented(sharedData,
                                           ctx->jointIndex,
                                           &currentEncoder) ||
                !readLatestMotorServoAbs(sharedData,
                                         ctx->motorChannel,
                                         &currentAbs)) {
                failAutoCalibContextWithDiag(ctx,
                                             sharedData,
                                             "pre-release sync lost servo feedback",
                                             CALIB_DIAG_REASON_FEEDBACK_LOST,
                                             CALIB_DIAG_PHASE_RELEASE,
                                             ctx->motorChannel,
                                             0,
                                             0,
                                             0,
                                             0);
                return;
            }

            const int32_t targetError =
                abs32(currentAbs - ctx->preReleaseSyncTargetAbs);
            if (targetError > kPreReleaseSyncTargetToleranceCounts) {
                // 前一段收紧轨迹仍在执行。将保持目标更新为当前实测 ABS，
                // 并重新开始停稳计时。
                ctx->preReleaseSyncTargetAbs = currentAbs;
                ctx->preReleaseSyncWindowStartAbs = currentAbs;
                ctx->preReleaseSyncWindowStartMs = nowMs;
                ctx->preReleaseSyncStableSamples = 0;
            }

            if (!appendJointCalibrationTargetToBatch(batch,
                                                     ctx->jointIndex,
                                                     clampToServoPosLocal(ctx->preReleaseSyncTargetAbs),
                                                     ctx->commandSpeed,
                                                     ctx->commandAcc)) {
                failAutoCalibContextWithDiag(ctx,
                                             sharedData,
                                             "pre-release sync target append failed",
                                             CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
                                             CALIB_DIAG_PHASE_RELEASE,
                                             ctx->motorChannel,
                                             ctx->preReleaseSyncTargetAbs,
                                             0,
                                             0,
                                             0);
                return;
            }

            if ((uint32_t)(nowMs - ctx->preReleaseSyncWindowStartMs) < params.windowMs) {
                return;
            }

            const int32_t windowAbsDelta =
                abs32(currentAbs - ctx->preReleaseSyncWindowStartAbs);
            const bool targetReached =
                abs32(currentAbs - ctx->preReleaseSyncTargetAbs) <=
                kPreReleaseSyncTargetToleranceCounts;
            const bool absSettled =
                windowAbsDelta <= kPreReleaseSyncSettleDeltaCounts;
            if (targetReached && absSettled) {
                if (ctx->preReleaseSyncStableSamples < 255) {
                    ctx->preReleaseSyncStableSamples++;
                }
            } else {
                ctx->preReleaseSyncStableSamples = 0;
            }

            FW_LOG_INFO_PRINTF("[AUTO CALIB] PRE_RELEASE_SYNC J%02u targetAbs=%ld currentAbs=%ld err=%ld/%ld windowAbs=%ld/%ld stable=%u/%u\r\n",
                          ctx->jointIndex,
                          (long)ctx->preReleaseSyncTargetAbs,
                          (long)currentAbs,
                          (long)abs32(currentAbs - ctx->preReleaseSyncTargetAbs),
                          (long)kPreReleaseSyncTargetToleranceCounts,
                          (long)windowAbsDelta,
                          (long)kPreReleaseSyncSettleDeltaCounts,
                          (unsigned)ctx->preReleaseSyncStableSamples,
                          (unsigned)kPreReleaseSyncStableSamples);

            if (ctx->preReleaseSyncStableSamples < kPreReleaseSyncStableSamples) {
                ctx->preReleaseSyncWindowStartAbs = currentAbs;
                ctx->preReleaseSyncWindowStartMs = nowMs;
                return;
            }

            ctx->commandAbs = currentAbs;
            ctx->commandRemainder = 0;
            ctx->windowStartAbs = currentAbs;
            ctx->windowStartMs = nowMs;
            ctx->lastCommandMs = nowMs;
            ctx->releaseAngleStallStable = 0;
            // 以已经停稳的紧端作为单电机 LUT 第一个样本。
            // 后续释放过程按磁编行程采点，不按命令位置猜测绳长关系。
            ctx->startEncoder = currentEncoder;
            ctx->startServoAbs = currentAbs;
            resetTendonRawFitBuffer(&ctx->fitSamples);
            appendTendonRawFitSample(&ctx->fitSamples,
                                     currentEncoder,
                                     currentAbs,
                                     0);
            ctx->fitSamples.nextTravelDeg = kTendonFitSampleStepDeg;
            ctx->state = AUTO_CALIB_STATE_TIGHTEN;
            FW_LOG_INFO_PRINTF("[AUTO CALIB] PRE_RELEASE_SYNC_READY J%02u releaseStartAbs=%ld\r\n",
                          ctx->jointIndex,
                          (long)currentAbs);
            return;
        }

        default:
            return;
    }
}

// 单关节同步包装，内部仍使用统一串行序列，确保初始化与诊断行为一致。
bool runAutoCalibrationJoint(TaskSharedData_t* sharedData,
                             uint8_t jointIndex,
                             const AutoCalibrationParams& params)
{
    return runAutoCalibrationSequence(sharedData, &jointIndex, 1, params);
}

// 自动标定总调度：
// 1. 校验并按机械规则重排关节；
// 2. 双腱关节执行工作空间规划和专用流程；
// 3. 单电机关节逐帧运行非阻塞状态机；
// 4. 成功结果逐关节保留，失败位图用于下次只重标失败关节。
bool runAutoCalibrationSequence(TaskSharedData_t* sharedData,
                                const uint8_t* jointList,
                                uint8_t jointCount,
                                const AutoCalibrationParams& params)
{
    if (!sharedData || !jointList || jointCount == 0 || jointCount > ENCODER_TOTAL_NUM) {
        setCalibrationDiag(sharedData,
                           CALIB_DIAG_REASON_JOINT_LIST_INVALID,
                           CALIB_DIAG_PHASE_SEQUENCE,
                           0xFF,
                           0xFF,
                           jointCount,
                           ENCODER_TOTAL_NUM,
                           jointList ? 1 : 0,
                           0);
        return false;
    }

    clearDualWorkspaceCache();

    uint32_t seenJoints = 0;
    uint8_t orderedJoints[ENCODER_TOTAL_NUM] = {0};
    for (uint8_t i = 0; i < jointCount; i++) {
        if (jointList[i] >= ENCODER_TOTAL_NUM) {
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_JOINT_LIST_INVALID,
                               CALIB_DIAG_PHASE_SEQUENCE,
                               jointList[i],
                               0xFF,
                               jointList[i],
                               ENCODER_TOTAL_NUM,
                               i,
                               seenJoints);
            return false;
        }
        const uint32_t bit = (1UL << jointList[i]);
        if ((seenJoints & bit) != 0) {
            FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL duplicate joint J%02u in sequence\r\n",
                          jointList[i]);
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_DUPLICATE_JOINT,
                               CALIB_DIAG_PHASE_SEQUENCE,
                               jointList[i],
                               0xFF,
                               i,
                               jointCount,
                               0,
                               seenJoints);
            return false;
        }
        seenJoints |= bit;
        orderedJoints[i] = jointList[i];
    }

    for (uint8_t i = 0; i < jointCount; i++) {
        for (uint8_t j = (uint8_t)(i + 1); j < jointCount; j++) {
            if (autoCalibrationSequenceRank(orderedJoints[j]) <
                autoCalibrationSequenceRank(orderedJoints[i])) {
                const uint8_t tmp = orderedJoints[i];
                orderedJoints[i] = orderedJoints[j];
                orderedJoints[j] = tmp;
            }
        }
    }

    // 标定刻意采用串行方式：收紧一根肌腱会改变相邻关节的机械负载和磁编角度，
    // 并行标定可能使检测到的零点/紧端产生偏差。单关节内部仍使用非阻塞状态机，
    // 但必须等当前关节完整结束后再启动下一个关节。
    uint32_t completedJointMask = 0;
    uint32_t failedJointMask = 0;
    uint8_t firstFailedJoint = 0xFF;

    for (uint8_t i = 0; i < jointCount; i++) {
        const uint32_t currentJointBit = (1UL << orderedJoints[i]);
        uint8_t dualPrimaryMotor = 0;
        uint8_t dualSecondaryMotor = 0;
        if (getDualAntagonistAutoCalibrationMotors(orderedJoints[i],
                                                   &dualPrimaryMotor,
                                                   &dualSecondaryMotor)) {
            const bool isWorkspaceTarget = isWorkspaceDualJoint(orderedJoints[i]);
            DualWorkspaceCacheEntry* workspaceState = NULL;
            if (isWorkspaceTarget) {
                if (!prepareWorkspaceForDualCalibration(sharedData,
                                                        orderedJoints[i],
                                                        params)) {
                    FW_LOG_INFO_PRINTF("[AUTO CALIB] SEQUENCE_SERIAL workspace failed J%02u; stop remaining sequence\r\n",
                                       orderedJoints[i]);
                    failedJointMask |= currentJointBit;
                    if (firstFailedJoint == 0xFF) {
                        firstFailedJoint = orderedJoints[i];
                    }
                    break;
                }
                const int8_t cacheIndex = workspaceCacheIndex(orderedJoints[i]);
                if (cacheIndex < 0) {
                    setWorkspacePlanFailureDiag(sharedData,
                                                orderedJoints[i],
                                                orderedJoints[i],
                                                DUAL_WORKSPACE_POSE_NONE,
                                                DUAL_WORKSPACE_PLAN_STAGE_EXECUTE);
                    failedJointMask |= currentJointBit;
                    if (firstFailedJoint == 0xFF) {
                        firstFailedJoint = orderedJoints[i];
                    }
                    break;
                }
                workspaceState = &g_dualWorkspaceCache[(uint8_t)cacheIndex];
                if (workspaceState->valid &&
                    workspaceState->currentPose == DUAL_WORKSPACE_POSE_B) {
                    FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_TARGET_B_CANDIDATE J%02u M%02u/M%02u; verify ABS before reuse\r\n",
                                       (unsigned)orderedJoints[i],
                                       (unsigned)dualPrimaryMotor,
                                       (unsigned)dualSecondaryMotor);
                } else {
                    FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_TARGET_DIRECT J%02u M%02u/M%02u; no target B preset\r\n",
                                       (unsigned)orderedJoints[i],
                                       (unsigned)dualPrimaryMotor,
                                       (unsigned)dualSecondaryMotor);
                }
            }
            FW_LOG_INFO_PRINTF("[AUTO CALIB] SEQUENCE_SERIAL begin J%02u (%u/%u) dual M%02u/M%02u\r\n",
                          orderedJoints[i],
                          (unsigned)(i + 1),
                          (unsigned)jointCount,
                          dualPrimaryMotor,
                          dualSecondaryMotor);
            if (!runDualAntagonistAutoCalibrationJoint(sharedData,
                                                       orderedJoints[i],
                                                       params,
                                                       workspaceState)) {
                FW_LOG_INFO_PRINTF("[AUTO CALIB] SEQUENCE_SERIAL failed J%02u dual\r\n",
                              orderedJoints[i]);
                if (!hasCalibrationDiag(sharedData)) {
                    setCalibrationDiag(sharedData,
                                       CALIB_DIAG_REASON_JOINT_STEP_FAILED,
                                       CALIB_DIAG_PHASE_DUAL_EXCHANGE,
                                       orderedJoints[i],
                                       dualPrimaryMotor,
                                       i,
                                       jointCount,
                                       dualSecondaryMotor,
                                       seenJoints);
                }
                failedJointMask |= currentJointBit;
                if (firstFailedJoint == 0xFF) {
                    firstFailedJoint = orderedJoints[i];
                }
                continue;
            }
            FW_LOG_INFO_PRINTF("[AUTO CALIB] SEQUENCE_SERIAL done J%02u dual\r\n",
                          orderedJoints[i]);
            completedJointMask |= currentJointBit;
            continue;
        }

        JointCalibContext ctx;
        initAutoCalibContext(&ctx, orderedJoints[i]);
        FW_LOG_INFO_PRINTF("[AUTO CALIB] SEQUENCE_SERIAL begin J%02u (%u/%u)\r\n",
                      orderedJoints[i],
                      (unsigned)(i + 1),
                      (unsigned)jointCount);

        while (ctx.state != AUTO_CALIB_STATE_DONE &&
               ctx.state != AUTO_CALIB_STATE_FAILED) {
            if (!autoCalibrationStillOwnsControl(sharedData)) {
                FW_LOG_INFO_PRINTF("[AUTO CALIB] CANCEL sequence state=%u owner=%u enabled=%u\r\n",
                              sharedData ? (unsigned)sharedData->system_state : 255,
                              sharedData ? (unsigned)sharedData->servo_target_owner : 255,
                              sharedData ? (unsigned)sharedData->control_enabled : 255);
                setCalibrationDiag(sharedData,
                                   CALIB_DIAG_REASON_CONTROL_LOST,
                                   CALIB_DIAG_PHASE_SEQUENCE,
                                   orderedJoints[i],
                                   0xFF,
                                   sharedData ? sharedData->system_state : -1,
                                   sharedData ? sharedData->servo_target_owner : -1,
                                   sharedData ? sharedData->control_enabled : -1,
                                   sharedData
                                       ? sharedAtomicLoad(&sharedData->active_joint_mask)
                                       : 0);
                return false;
            }

            ServoTargetBatch_t batch;
            memset(&batch, 0, sizeof(batch));
            batch.source = SERVO_TARGET_OWNER_CALIBRATION;
            batch.timestamp = millis();

            if (!appendCalibrationHoldTargetsExcept(sharedData,
                                                    &batch,
                                                    orderedJoints,
                                                    jointCount,
                                                    ctx.jointIndex,
                                                    failedJointMask,
                                                    params)) {
                FW_LOG_INFO_PRINTLN("[AUTO CALIB] FAIL sequence append non-active holds");
                setCalibrationDiag(sharedData,
                                   CALIB_DIAG_REASON_HOLD_TARGET_FAILED,
                                   CALIB_DIAG_PHASE_SEQUENCE,
                                   ctx.jointIndex,
                                   0xFF,
                                   i,
                                   jointCount,
                                   batch.count,
                                   seenJoints);
                return false;
            }

            stepAutoCalibrationJoint(&ctx, sharedData, params, &batch);

            if (!publishCalibrationBatch(sharedData, &batch)) {
                FW_LOG_INFO_PRINTLN("[AUTO CALIB] FAIL sequence publish target batch");
                setCalibrationDiag(sharedData,
                                   CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
                                   CALIB_DIAG_PHASE_SEQUENCE,
                                   ctx.jointIndex,
                                   0xFF,
                                   batch.count,
                                   sharedData ? sharedData->servo_target_owner : -1,
                                   sharedData ? sharedData->system_state : -1,
                                   seenJoints);
                return false;
            }

            vTaskDelay(pdMS_TO_TICKS(params.loopMs));
        }

        if (ctx.state == AUTO_CALIB_STATE_FAILED) {
            FW_LOG_INFO_PRINTF("[AUTO CALIB] SEQUENCE_SERIAL failed J%02u\r\n", orderedJoints[i]);
            if (!hasCalibrationDiag(sharedData)) {
                setCalibrationDiag(sharedData,
                                   CALIB_DIAG_REASON_JOINT_STEP_FAILED,
                                   CALIB_DIAG_PHASE_SEQUENCE,
                                   orderedJoints[i],
                                   ctx.motorChannel,
                                   ctx.state,
                                   ctx.releaseAngleStallStable,
                                   i,
                                   seenJoints);
            }
            failedJointMask |= currentJointBit;
            if (firstFailedJoint == 0xFF) {
                firstFailedJoint = orderedJoints[i];
            }
            continue;
        }

        FW_LOG_INFO_PRINTF("[AUTO CALIB] SEQUENCE_SERIAL done J%02u\r\n", orderedJoints[i]);
        completedJointMask |= currentJointBit;
    }

    if (failedJointMask != 0) {
        FW_LOG_INFO_PRINTF("[AUTO CALIB] SEQUENCE_SERIAL partial failed=0x%08lX completed=0x%08lX selected=0x%08lX\r\n",
                      (unsigned long)failedJointMask,
                      (unsigned long)completedJointMask,
                      (unsigned long)seenJoints);
        if (!hasCalibrationDiag(sharedData)) {
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_JOINT_STEP_FAILED,
                               CALIB_DIAG_PHASE_SEQUENCE,
                               firstFailedJoint,
                               0xFF,
                               completedJointMask,
                               failedJointMask,
                               jointCount,
                               failedJointMask);
        }
        return false;
    }

    return true;
}

// 创建 FreeRTOS 自动标定任务，并注册运行时/堆栈监控。
static bool startAutoCalibrationTaskInternal(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return false;
    }
    if (g_autoCalibrationTaskHandle != NULL) {
        FW_LOG_INFO_PRINTLN("[AUTO CALIB] task already running");
        return true;
    }
    clearCalibrationDiag(sharedData);

    const bool created = p4::platform::createTask(
        {autoCalibrationTask,
         "autoCalibrationTask",
         AUTO_CALIB_TASK_STACK_SIZE,
         sharedData,
         TASK_AUTO_CALIB_PRIORITY},
        &g_autoCalibrationTaskHandle);
    if (created) {
        taskRuntimeRegister(
            TASK_RUNTIME_AUTO_CALIB,
            g_autoCalibrationTaskHandle,
            AUTO_CALIB_TASK_STACK_SIZE
        );
    }
    return created;
}

// 对外启动接口，保留单一创建路径。
bool startAutoCalibrationTask(TaskSharedData_t* sharedData)
{
    return startAutoCalibrationTaskInternal(sharedData);
}

// 将本次 active joint 列表转换成真正参与标定的电机位图，供热插拔恢复门禁使用。
static uint32_t buildCalibrationMotorMask(const uint8_t* joints,
                                          uint8_t jointCount)
{
    if (!joints || jointCount == 0) {
        return 0;
    }
    uint32_t motorMask = 0;
    for (uint8_t i = 0; i < jointCount; i++) {
        motorMask |= calibrationJointMotorMask(joints[i]);
    }
    return motorMask & validMotorChannelMask();
}

// FreeRTOS 任务入口：读取 active scope、执行标定、更新状态并释放 owner。
void autoCalibrationTask(void* parameter)
{
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    const AutoCalibrationParams params = defaultAutoCalibrationParams();
    if (sharedData) {
        // 预检期间禁止通信任务把热插拔恢复误判为“标定运动中”而跳过。
        sharedAtomicStore(&sharedData->calibration_motion_started,
                          (uint8_t)0);
    }

    uint8_t activeJoints[ENCODER_TOTAL_NUM] = {0};
    const uint8_t activeJointCount =
        buildAutoCalibrationJointListFromActiveScope(sharedData,
                                                     activeJoints,
                                                     ENCODER_TOTAL_NUM);
    if (activeJointCount == 0) {
        FW_LOG_INFO_PRINTLN("[AUTO CALIB] FAIL active scope has no joints");
        setCalibrationDiag(sharedData,
                           CALIB_DIAG_REASON_ACTIVE_SCOPE_EMPTY,
                           CALIB_DIAG_PHASE_SEQUENCE,
                           0xFF,
                           0xFF,
                           sharedData
                               ? (int32_t)sharedAtomicLoad(&sharedData->active_joint_mask)
                               : 0,
                           sharedData
                               ? (int32_t)sharedAtomicLoad(&sharedData->active_motor_mask)
                               : 0,
                           0,
                           0);
    } else {
        char activeScopeLog[128] = {0};
        size_t used = 0;
        int written = snprintf(activeScopeLog,
                               sizeof(activeScopeLog),
                               "[AUTO CALIB] ACTIVE_SCOPE joints=");
        if (written > 0) {
            used = (size_t)written;
            if (used >= sizeof(activeScopeLog)) {
                used = sizeof(activeScopeLog) - 1;
            }
        }
        for (uint8_t i = 0; i < activeJointCount; i++) {
            if (used >= sizeof(activeScopeLog) - 1) {
                break;
            }
            written = snprintf(activeScopeLog + used,
                               sizeof(activeScopeLog) - used,
                               "%sJ%02u",
                               (i == 0) ? "" : ",",
                               activeJoints[i]);
            if (written <= 0) {
                break;
            }
            const size_t appended = (size_t)written;
            used += appended;
            if (used >= sizeof(activeScopeLog)) {
                used = sizeof(activeScopeLog) - 1;
                break;
            }
        }
        activeScopeLog[used] = '\0';
        FW_LOG_INFO_PRINTLN(activeScopeLog);
    }

    const uint32_t selectedMotorMask =
        buildCalibrationMotorMask(activeJoints, activeJointCount);
    const bool selectedRecoveryReady =
        activeJointCount > 0 &&
        selectedMotorMask != 0 &&
        waitForCalibrationServoRecovery(sharedData, selectedMotorMask);

    bool needsWorkspaceReadMask = false;
    for (uint8_t i = 0; i < activeJointCount; ++i) {
        if (isWorkspaceDualJoint(activeJoints[i])) {
            needsWorkspaceReadMask = true;
            break;
        }
    }
    uint32_t workspaceReadMask = 0;
    const bool workspaceReady =
        selectedRecoveryReady &&
        (!needsWorkspaceReadMask ||
         buildDynamicWorkspaceMotorMask(sharedData,
                                        activeJoints,
                                        activeJointCount,
                                        &workspaceReadMask));
    setCalibrationWorkspaceReadMask(sharedData,
                                    workspaceReady ? workspaceReadMask : 0);
    const uint32_t requiredRecoveryMask =
        (selectedMotorMask | workspaceReadMask) & validMotorChannelMask();
    const bool allRecoveryReady =
        workspaceReady &&
        waitForCalibrationServoRecovery(sharedData, requiredRecoveryMask);
    if (allRecoveryReady && sharedData) {
        // 从这一刻起标定状态机才允许发布电机目标；掉线将恢复为正常运动故障。
        sharedAtomicStore(&sharedData->calibration_motion_started,
                          (uint8_t)1);
    }
    const bool ok = (activeJointCount > 0) &&
                    allRecoveryReady &&
                    runAutoCalibrationSequence(sharedData,
                                               activeJoints,
                                               activeJointCount,
                                               params);
    if (sharedData) {
        sharedAtomicStore(&sharedData->calibration_motion_started,
                          (uint8_t)0);
    }
    setCalibrationWorkspaceReadMask(sharedData, 0);

    if (!ok && sharedData && !hasCalibrationDiag(sharedData)) {
        setCalibrationDiag(sharedData,
                           CALIB_DIAG_REASON_JOINT_STEP_FAILED,
                           CALIB_DIAG_PHASE_SEQUENCE,
                           0xFF,
                           0xFF,
                           0,
                           0,
                           0,
                           0);
    }

    postSystemEvent(sharedData, ok ? SYSTEM_EVENT_CALIBRATION_DONE : SYSTEM_EVENT_CALIBRATION_FAILED);
    taskRuntimeUnregister(TASK_RUNTIME_AUTO_CALIB);
    g_autoCalibrationTaskHandle = NULL;
    vTaskDelete(NULL);
}
