/**
 * @file CalibrationMotionSupport.cpp
 * @brief 实现自动标定共享的反馈读取、目标发布、范围校验和多圈提交原语。
 */
#include "CalibrationMotionSupport.h"
#include "CalibrationRuntimePort.h"
#include "AutoCalibrationCoordinator.h"

#include <math.h>
#include <string.h>

#include "CalibrationResultCommitter.h"
#include "CalibrationFeedbackReader.h"
#include "CalibrationLutBuilder.h"
#include "CalibrationDiagnostics.h"
#include "CalibrationSequencePlanner.h"
#include "../../modules/actuator/ActuatorCommandGateway.h"
#include "../../config/CalibrationConfig.h"
#include "../../config/topology/HardwareMap.h"
#include "../../hal/servo/ServoValue.h"
#include "../../platform/rtos/SharedSafetyState.h"
#include "../../platform/rtos/AtomicValue.h"
#include "../../platform/log/FirmwareLog.h"
#include "../../runtime/metrics/TaskRuntimeMetrics.h"
#include "../../safety/policies/CalibrationSafetyPolicy.h"

using namespace mmhand::config::calibration;
using namespace mmhand::calibration;

int32_t calibrationAbs32(int32_t value)
{
    return value < 0 ? -value : value;
}

// 将原始多圈 ABS 统一到“收绳为正”的 tendon 坐标，计算从紧端实际放出的 counts。
// 若电机仍在收绳方向，返回 0，避免机械回弹或反馈抖动被当成过度松绳。
int32_t singleTendonReleaseCounts(uint8_t motorChannel,
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
int32_t clampCalibrationServoAbs(int32_t value)
{
    // 标定轨迹使用 P4 逻辑 ABS，实际舵机行程由通信任务结合单路
    // logicalOffset 做最终校验，不能在这里按固定七圈截断。
    return value;
}

// 根据紧端 ABS 登记一个保守的五圈电机工作区，并增加 500 counts 紧端余量。
void setCalibratedMotorAbsLimit(SystemRuntimeContext* sharedData,
                                       uint8_t motorChannel,
                                       int32_t tightAbs)
{
    if (!sharedData ||
        motorChannel >= SERVO_TOTAL_NUM ||
        !isMotorChannelValid(motorChannel)) {
        return;
    }

    const int32_t tightClamped = clampCalibrationServoAbs(tightAbs);
    const int32_t slackClamped = clampCalibrationServoAbs(
        motorApplyTendonDelta(motorChannel, tightClamped, -kMotorAbsWindowCounts));
    const int32_t minAbs = (slackClamped < tightClamped) ? slackClamped : tightClamped;
    const int32_t baseMaxAbs = (slackClamped > tightClamped) ? slackClamped : tightClamped;
    const int32_t maxAbs = clampCalibrationServoAbs(
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
void setSingleTendonLutAbsLimit(SystemRuntimeContext* sharedData,
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
    // counts 紧端余量。这里只登记标定诊断范围；负反馈边界由 LUT 统一负责。
    lutMinAbs = clampCalibrationServoAbs(lutMinAbs);
    lutMaxAbs = clampCalibrationServoAbs(lutMaxAbs);
    if (lutMinAbs > lutMaxAbs) {
        const int32_t tmp = lutMinAbs;
        lutMinAbs = lutMaxAbs;
        lutMaxAbs = tmp;
    }
    const int32_t tightWithMargin = clampCalibrationServoAbs(
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
    }
    FW_LOG_INFO_PRINTF(
        "[AUTO CALIB] SINGLE_LUT_ABS_LIMIT M%02u lut=[%ld,%ld] "
        "limit=[%ld,%ld] tight=%ld margin=%ld\r\n",
        (unsigned)motorChannel,
        (long)lutMinAbs,
        (long)lutMaxAbs,
        (long)limitMin,
        (long)limitMax,
        (long)lutTightAbs,
        (long)kMotorAbsMaxMarginCounts);
}

// 清除指定关节所有执行器的旧 ABS 限位，避免新标定被旧范围拦截。
void clearCalibratedMotorAbsLimitsForJoint(SystemRuntimeContext* sharedData,
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
        const int primaryCh = findPrimaryMotorChannelForJoint(jointIndex);
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
int32_t servoTurnFromAbs(int32_t abs)
{
    const int32_t raw = mmhand::servo::normalizeRawSingleTurn(abs);
    return (abs - raw) / 4096;
}

// 标定循环每帧检查状态机 owner，失去控制权后立即停止发命令。
bool autoCalibrationStillOwnsControl(SystemRuntimeContext* sharedData)
{
    return sharedData &&
           sharedData->system_state == SYSTEM_STATE_CALIBRATION_RUNNING &&
           sharedData->servo_target_owner == SERVO_TARGET_OWNER_CALIBRATION &&
           sharedData->control_enabled == 0;
}

// 读取单台舵机当前上电会话的 logical ABS 可达范围。范围由通信任务依据
// logicalOffset 独立生成，因此一台舵机掉电重建不会改变其他26台舵机。
bool getServoLogicalCommandRange(SystemRuntimeContext* sharedData,
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
bool isServoCommandInsideLimit(SystemRuntimeContext* sharedData,
                                      uint8_t motorChannel,
                                      int32_t commandAbs)
{
    int32_t rangeMin = 0;
    int32_t rangeMax = 0;
    const bool rangeValid = getServoLogicalCommandRange(
        sharedData, motorChannel, &rangeMin, &rangeMax);
    return p4::safety::calibrationCommandInsideRange(
        rangeValid, commandAbs, rangeMin, rangeMax);
}

// 检查单关节主执行器的多圈状态是否可信。
bool isJointPrimaryServoMultiTurnKnown(SystemRuntimeContext* sharedData, uint8_t jointIndex)
{
    if (!sharedData || jointIndex >= ENCODER_TOTAL_NUM) {
        return false;
    }
    const int ch = findPrimaryMotorChannelForJoint(jointIndex);
    if (ch < 0 || ch >= SERVO_TOTAL_NUM) {
        return false;
    }
    return (sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap) &
            (1UL << ch)) == 0;
}

// 汇总指定关节实际使用的执行器。双腱关节返回两台电机，单腱关节返回主电机。
/**
 * 标定启动前等待所选磁编连续稳定。
 * active scope 切换时健康位可能短暂抖动，因此按新时间戳累计稳定帧，
 * 确认 S3/CAN 快照、在线位和故障位一致后才允许机械动作。
 */
bool waitForCalibrationEncoderRecovery(SystemRuntimeContext* sharedData,
                                       uint32_t requiredJointMask)
{
    if (!sharedData || requiredJointMask == 0) {
        return false;
    }

    const uint32_t beginMs = calibrationNowMs();
    uint32_t lastLogMs = 0;
    uint32_t lastFeedbackTimestamp = 0;
    uint8_t stableSamples = 0;
    while (autoCalibrationStillOwnsControl(sharedData)) {
        const uint32_t nowMs = calibrationNowMs();
        const uint32_t onlineMask =
            sharedAtomicLoad(&sharedData->encoder_online_bitmap);
        const uint32_t faultMask =
            sharedAtomicLoad(&sharedData->encoder_fault_bitmap);
        const uint8_t canOnline =
            sharedAtomicLoad(&sharedData->can_link_online);

        RemoteSensorData_t snapshot;
        memset(&snapshot, 0, sizeof(snapshot));
        const bool hasSnapshot = sharedData->canRxQueue.peek(&snapshot);
        const bool freshSnapshot =
            hasSnapshot && snapshot.isValid && snapshot.timestamp != 0 &&
            (uint32_t)(nowMs - snapshot.timestamp) <=
                kEncoderTrustedSampleMaxAgeMs;
        const bool ready =
            freshSnapshot && canOnline != 0 &&
            (onlineMask & requiredJointMask) == requiredJointMask &&
            (faultMask & requiredJointMask) == 0;

        if (ready && snapshot.timestamp != lastFeedbackTimestamp) {
            lastFeedbackTimestamp = snapshot.timestamp;
            if (stableSamples < kEncoderRecoveryStableSamples) {
                ++stableSamples;
            }
            if (stableSamples >= kEncoderRecoveryStableSamples) {
                FW_LOG_INFO_PRINTF(
                    "[AUTO CALIB] ENCODER_RECOVERY_READY required=0x%08lX "
                    "online=0x%08lX\r\n",
                    (unsigned long)requiredJointMask,
                    (unsigned long)onlineMask);
                return true;
            }
        } else if (!ready) {
            stableSamples = 0;
        }

        if ((uint32_t)(nowMs - beginMs) >= kEncoderRecoveryTimeoutMs) {
            uint32_t unresolvedMask =
                requiredJointMask & (~onlineMask | faultMask);
            if (!freshSnapshot || canOnline == 0) {
                unresolvedMask |= requiredJointMask;
            }
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_ENCODER_ERROR,
                               CALIB_DIAG_PHASE_INIT,
                               0xFF,
                               0xFF,
                               (int32_t)onlineMask,
                               (int32_t)faultMask,
                               (int32_t)canOnline,
                               unresolvedMask);
            FW_LOG_WARN_PRINTF(
                "[AUTO CALIB] ENCODER_RECOVERY_FAIL required=0x%08lX "
                "online=0x%08lX fault=0x%08lX can=%u fresh=%u "
                "unresolved=0x%08lX\r\n",
                (unsigned long)requiredJointMask,
                (unsigned long)onlineMask,
                (unsigned long)faultMask,
                (unsigned)canOnline,
                freshSnapshot ? 1U : 0U,
                (unsigned long)unresolvedMask);
            return false;
        }

        if ((uint32_t)(nowMs - lastLogMs) >=
            kEncoderRecoveryLogIntervalMs) {
            lastLogMs = nowMs;
            FW_LOG_INFO_PRINTF(
                "[AUTO CALIB] ENCODER_RECOVERY_WAIT required=0x%08lX "
                "online=0x%08lX fault=0x%08lX can=%u fresh=%u stable=%u/%u\r\n",
                (unsigned long)requiredJointMask,
                (unsigned long)onlineMask,
                (unsigned long)faultMask,
                (unsigned)canOnline,
                freshSnapshot ? 1U : 0U,
                (unsigned)stableSamples,
                (unsigned)kEncoderRecoveryStableSamples);
        }
        calibrationWaitMs(10);
    }

    if (!hasCalibrationDiag(sharedData)) {
        setCalibrationDiag(sharedData,
                           CALIB_DIAG_REASON_CONTROL_LOST,
                           CALIB_DIAG_PHASE_INIT,
                           0xFF,
                           0xFF,
                           sharedData ? (int32_t)sharedData->system_state : -1,
                           sharedData ? (int32_t)sharedData->servo_target_owner : -1,
                           sharedData ? (int32_t)sharedData->control_enabled : -1,
                           requiredJointMask);
    }
    return false;
}

// 标定开始前等待热插拔恢复闭环完成。该函数只观察共享状态，不发送任何运动目标。
bool waitForCalibrationServoRecovery(SystemRuntimeContext* sharedData,
                                     uint32_t requiredMask)
{
    if (!sharedData || requiredMask == 0) {
        return false;
    }
    requiredMask &= validMotorChannelMask();
    const uint32_t beginMs = calibrationNowMs();
    uint32_t lastLogMs = 0;
    uint32_t lastFeedbackTimestamp = 0;
    uint8_t stableSamples = 0;

    while (autoCalibrationStillOwnsControl(sharedData)) {
        const uint32_t nowMs = calibrationNowMs();
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
            sharedData->servoFeedbackQueue.peek(&snapshot);
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
        calibrationWaitMs(10);
    }
    if (!hasCalibrationDiag(sharedData)) {
        setCalibrationDiag(sharedData,
                           CALIB_DIAG_REASON_CONTROL_LOST,
                           CALIB_DIAG_PHASE_INIT,
                           0xFF,
                           0xFF,
                           sharedData ? (int32_t)sharedData->system_state : -1,
                           sharedData ? (int32_t)sharedData->servo_target_owner : -1,
                           sharedData ? (int32_t)sharedData->control_enabled : -1,
                           requiredMask);
    }
    return false;
}

// 标定成功后由舵机通信任务提交本关节的可信状态，避免只改共享位图而驱动仍拒绝多圈 ABS。
bool commitJointServoMultiTurnTrust(SystemRuntimeContext* sharedData,
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
    const uint32_t beginMs = calibrationNowMs();
    while (autoCalibrationStillOwnsControl(sharedData) &&
           (uint32_t)(calibrationNowMs() - beginMs) <
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
        calibrationWaitMs(5);
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
static bool calibrationBatchOverloadSafe(SystemRuntimeContext* sharedData,
                                         const ServoTargetBatch_t* batch)
{
    if (!sharedData || !batch) {
        return false;
    }
    for (uint8_t i = 0; i < batch->count; i++) {
        const ServoTargetCommand_t& cmd = batch->commands[i];
        const int ch = cmd.motorChannel < SERVO_TOTAL_NUM
            ? cmd.motorChannel
            : -1;
        if (ch < 0 || ch >= SERVO_TOTAL_NUM || !isMotorChannelValid((uint8_t)ch)) {
            continue;
        }

        int16_t loadRaw = 0;
        if (!readLatestMotorServoLoad(sharedData, (uint8_t)ch, &loadRaw)) {
            continue;
        }
        const int16_t loadMag = mmhand::servo::loadMagnitude(loadRaw);
        if (p4::safety::calibrationLoadOverloaded(
                loadMag, kOverloadLoadThreshold)) {
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

bool appendCalibrationServoTarget(ServoTargetBatch_t* batch,
                                   uint8_t motorChannel,
                                   int32_t position,
                                   uint16_t speed,
                                   uint8_t acc)
{
    if (!batch) {
        return false;
    }
    if (batch->buildFailed != 0 ||
        batch->count > SERVO_TARGET_BATCH_MAX ||
        !isMotorChannelValid(motorChannel)) {
        batch->buildFailed = 1;
        return false;
    }
    for (uint8_t i = 0; i < batch->count; i++) {
        ServoTargetCommand_t& existing = batch->commands[i];
        if (existing.motorChannel == motorChannel) {
            existing.position = position;
            existing.speed = speed;
            existing.acc = acc;
            return true;
        }
    }
    if (batch->count >= SERVO_TARGET_BATCH_MAX) {
        // 标定批次同样执行全有或全无；容量不足不能让部分舵机继续动作。
        batch->buildFailed = 1;
        return false;
    }
    ServoTargetCommand_t& cmd = batch->commands[batch->count++];
    cmd.motorChannel = motorChannel;
    cmd.position = position;
    cmd.speed = speed;
    cmd.acc = acc;
    return true;
}

// 将单电机关节的标定目标写入批次。
// 双腱关节由专用状态机显式生成两路目标，不能在这里叠加旧主从映射。
bool appendJointCalibrationTarget(ServoTargetBatch_t* batch,
                                                uint8_t jointIndex,
                                                int32_t primaryTarget,
                                                uint16_t speed,
                                                uint8_t acc)
{
    if (!batch || jointIndex >= ENCODER_TOTAL_NUM) {
        return false;
    }
    return appendCalibrationServoTarget(batch,
                                  (uint8_t)findPrimaryMotorChannelForJoint(jointIndex),
                                  primaryTarget,
                                  speed,
                                  acc);
}

// 发布 CALIBRATION owner 的目标批次，并等待 ServoBus 确认整批已进入同步写。
bool publishCalibrationBatch(SystemRuntimeContext* sharedData, ServoTargetBatch_t* batch)
{
    if (!sharedData || !batch) {
        return false;
    }
    if (batch->buildFailed != 0) {
        return false;
    }
    if (batch->count == 0) {
        return true;
    }
    if (sharedData->servo_target_owner != SERVO_TARGET_OWNER_CALIBRATION) {
        return false;
    }
    if (!calibrationBatchOverloadSafe(sharedData, batch)) {
        return false;
    }
    batch->source = SERVO_TARGET_OWNER_CALIBRATION;
    batch->purpose = SERVO_TARGET_PURPOSE_CALIBRATION_CONFIRMED;
    batch->timestamp = calibrationNowMs();
    uint32_t token = sharedAtomicAddFetch(
        &sharedData->calibration_target_request_token, (uint32_t)1);
    if (token == 0) {
        token = sharedAtomicAddFetch(
            &sharedData->calibration_target_request_token, (uint32_t)1);
    }
    uint32_t requestedMask = 0;
    for (uint8_t i = 0; i < batch->count; ++i) {
        requestedMask |= 1UL << batch->commands[i].motorChannel;
    }
    batch->transactionToken = token;
    sharedAtomicStore(&sharedData->calibration_target_requested_mask,
                      requestedMask);
    sharedAtomicStore(&sharedData->calibration_target_applied_mask,
                      (uint32_t)0);
    sharedAtomicStore(&sharedData->calibration_target_rejected_mask,
                      (uint32_t)0);
    sharedAtomicStore(&sharedData->calibration_target_apply_status,
                      (uint8_t)CALIBRATION_TARGET_APPLY_PENDING);
    if (!p4::actuator::publishLatest(sharedData, *batch)) {
        sharedAtomicStore(&sharedData->calibration_target_rejected_mask,
                          requestedMask);
        sharedAtomicStore(&sharedData->calibration_target_apply_status,
                          (uint8_t)CALIBRATION_TARGET_APPLY_REJECTED);
        return false;
    }

    const uint32_t beginMs = calibrationNowMs();
    while (autoCalibrationStillOwnsControl(sharedData) &&
           (uint32_t)(calibrationNowMs() - beginMs) <
               kTargetApplyAckTimeoutMs) {
        if (sharedAtomicLoad(&sharedData->calibration_target_result_token) ==
            token) {
            const uint8_t status = sharedAtomicLoad(
                &sharedData->calibration_target_apply_status);
            const uint32_t appliedMask = sharedAtomicLoad(
                &sharedData->calibration_target_applied_mask);
            const uint32_t rejectedMask = sharedAtomicLoad(
                &sharedData->calibration_target_rejected_mask);
            if (status == CALIBRATION_TARGET_APPLY_APPLIED &&
                appliedMask == requestedMask && rejectedMask == 0) {
                return true;
            }
            FW_LOG_WARN_PRINTF(
                "[AUTO CALIB] TARGET_APPLY_REJECT token=%lu request=0x%08lX "
                "applied=0x%08lX rejected=0x%08lX status=%u\r\n",
                (unsigned long)token,
                (unsigned long)requestedMask,
                (unsigned long)appliedMask,
                (unsigned long)rejectedMask,
                (unsigned)status);
            return false;
        }
        calibrationWaitMs(1);
    }
    FW_LOG_WARN_PRINTF(
        "[AUTO CALIB] TARGET_APPLY_TIMEOUT token=%lu request=0x%08lX "
        "resultToken=%lu timeoutMs=%lu\r\n",
        (unsigned long)token,
        (unsigned long)requestedMask,
        (unsigned long)sharedAtomicLoad(
            &sharedData->calibration_target_result_token),
        (unsigned long)kTargetApplyAckTimeoutMs);
    return false;
}

// 串行标定时给非活动关节追加“保持当前 ABS”，防止共享绳路被当前动作带动。
bool appendCalibrationHoldTargetsExcept(SystemRuntimeContext* sharedData,
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
                if (!appendCalibrationServoTarget(batch,
                                            motorChannel,
                                            clampCalibrationServoAbs(holdAbs),
                                            params.servoSpeed,
                                            params.servoAcc)) {
                    FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL append hold target J%02u M%02u abs=%ld\r\n",
                                  jointIndex,
                                  motorChannel,
                                  (long)holdAbs);
                    return false;
                }
            }
        }
    }

    return true;
}

// 从 HardwareMap 读取双腱关节的主/副电机 Mxx；单电机关节返回 false。
bool getDualCalibrationMotors(uint8_t jointIndex,
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
    return appendCalibrationServoTarget(batch,
                                  motorChannel,
                                  clampCalibrationServoAbs(targetAbs),
                                  params.servoSpeed,
                                  params.servoAcc);
}

// 发布单台电机目标，并可选择同时保持另一台电机。
bool publishMotorCalibrationTarget(SystemRuntimeContext* sharedData,
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
    batch.timestamp = calibrationNowMs();
    if (!appendMotorCalibrationTargetToBatch(&batch, motorChannel, targetAbs, params)) {
        return false;
    }
    return publishCalibrationBatch(sharedData, &batch);
}

// 显式发布标定目标：目标值归轨迹生成逻辑所有，不能在这里用反馈值重写。
// 直接发布单台电机目标，不附带另一电机保持命令。
bool publishMotorCalibrationTargetDirect(SystemRuntimeContext* sharedData,
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
    batch.timestamp = calibrationNowMs();
    if (!appendMotorCalibrationTargetToBatch(&batch, motorChannel, targetAbs, params)) {
        return false;
    }
    return publishCalibrationBatch(sharedData, &batch);
}

// 双电机目标发布包装：只移动 movingMotor，可选固定 holdMotor。
bool publishDualMotorCalibrationTargets(SystemRuntimeContext* sharedData,
                                               uint8_t movingMotor,
                                               int32_t movingTargetAbs,
                                               uint8_t holdMotor,
                                               int32_t holdTargetAbs,
                                               const AutoCalibrationParams& params);

// 双电机目标发布实现，统一检查范围、owner、负载和批次数量。
bool publishDualMotorCalibrationTargets(SystemRuntimeContext* sharedData,
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
    batch.timestamp = calibrationNowMs();
    if (!appendMotorCalibrationTargetToBatch(&batch, movingMotor, movingTargetAbs, params)) {
        return false;
    }
    if (holdMotor != movingMotor &&
        !appendMotorCalibrationTargetToBatch(&batch, holdMotor, holdTargetAbs, params)) {
        return false;
    }
    return publishCalibrationBatch(sharedData, &batch);
}
