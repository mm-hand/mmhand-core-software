/**
 * @file DualMotorCalibrationMachine.cpp
 * @brief 实现双腱关节工作空间准备、机械端搜索、预紧和 LUT 提交状态机。
 */
#include "DualMotorCalibrationMachine.h"

#include <math.h>
#include <string.h>

#include "../CalibrationDiagnostics.h"
#include "../CalibrationFeedbackReader.h"
#include "../CalibrationLutBuilder.h"
#include "../CalibrationMotionSupport.h"
#include "../CalibrationResultCommitter.h"
#include "../CalibrationRuntimePort.h"
#include "../CalibrationSequencePlanner.h"
#include "../../../config/CalibrationConfig.h"
#include "../../../config/topology/HardwareMap.h"
#include "../../../hal/servo/ServoValue.h"
#include "../../../modules/calibration/CalibrationMath.h"
#include "../../../modules/calibration/CalibrationRuntimeStore.h"
#include "../../../modules/calibration/TendonCalibrationLut.h"
#include "../../../modules/calibration/ManualEncoderReferenceStore.h"
#include "../../../platform/log/FirmwareLog.h"
#include "../../../platform/rtos/AtomicValue.h"
#include "../../../platform/rtos/SharedSafetyState.h"
#include "../../../runtime/ports/SystemEventPort.h"
#include "../../../runtime/metrics/TaskRuntimeMetrics.h"

using namespace mmhand::config::calibration;
using namespace mmhand::calibration;

// 只有四指侧摆双腱关节参与“为相邻手指让出标定空间”的姿态规划；
// 拇指机械独立，不进入该规划。
static const uint8_t kWorkspaceDualJoints[] = {0, 4, 8, 12};
static const uint8_t kWorkspaceDualJointsCount =
    sizeof(kWorkspaceDualJoints) / sizeof(kWorkspaceDualJoints[0]);
static const uint8_t kWorkspaceDualHelperCount = kWorkspaceDualJointsCount - 1;


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

// 双腱子流程统一登记反馈、目标发布和控制权三类根因；外层只负责补充结果类诊断。
static void setDualFeedbackLostDiag(SystemRuntimeContext* sharedData,
                                    uint8_t phase,
                                    uint8_t jointIndex,
                                    uint8_t motor,
                                    int32_t valueA,
                                    int32_t valueB,
                                    uint32_t motorMask)
{
    setCalibrationDiagIfEmpty(sharedData,
                              CALIB_DIAG_REASON_FEEDBACK_LOST,
                              phase,
                              jointIndex,
                              motor,
                              valueA,
                              valueB,
                              0,
                              motorMask);
}

static void setDualPublishFailedDiag(SystemRuntimeContext* sharedData,
                                     uint8_t phase,
                                     uint8_t jointIndex,
                                     uint8_t motor,
                                     int32_t targetAbs,
                                     uint32_t motorMask)
{
    setCalibrationDiagIfEmpty(
        sharedData,
        CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
        phase,
        jointIndex,
        motor,
        targetAbs,
        sharedData ? sharedData->servo_target_owner : -1,
        sharedData ? sharedData->system_state : -1,
        motorMask);
}

static void setDualControlLostDiag(SystemRuntimeContext* sharedData,
                                   uint8_t phase,
                                   uint8_t jointIndex,
                                   uint8_t motor,
                                   uint32_t motorMask)
{
    setCalibrationDiagIfEmpty(
        sharedData,
        CALIB_DIAG_REASON_CONTROL_LOST,
        phase,
        jointIndex,
        motor,
        sharedData ? sharedData->system_state : -1,
        sharedData ? sharedData->servo_target_owner : -1,
        sharedData ? sharedData->control_enabled : -1,
        motorMask);
}

// 等待一台电机到达目标容差并停稳；超时或 owner 丢失时返回 false。
static bool waitMotorTargetReached(SystemRuntimeContext* sharedData,
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
            setDualPublishFailedDiag(sharedData,
                                     CALIB_DIAG_PHASE_DUAL_PRELOAD,
                                     jointIndex,
                                     motorChannel,
                                     targetAbs,
                                     (1UL << motorChannel));
            return false;
        }

        int32_t currentAbs = 0;
        if (readLatestMotorServoAbs(sharedData, motorChannel, &currentAbs)) {
            const uint32_t nowMs = calibrationNowMs();
            const int32_t absErr = calibrationAbs32(currentAbs - targetAbs);
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

        calibrationWaitMs(params.loopMs);
    }
    setDualControlLostDiag(sharedData,
                           CALIB_DIAG_PHASE_DUAL_PRELOAD,
                           jointIndex,
                           motorChannel,
                           (1UL << motorChannel));
    return false;
}

// 双腱标定开始前等待磁编、两路 ABS 和在线状态同时有效。
static bool waitDualInitialFeedback(SystemRuntimeContext* sharedData,
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

        const uint32_t nowMs = calibrationNowMs();
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
        calibrationWaitMs(20);
    }
    setDualControlLostDiag(sharedData,
                           CALIB_DIAG_PHASE_INIT,
                           jointIndex,
                           primaryMotor,
                           (1UL << primaryMotor) | (1UL << secondaryMotor));
    return false;
}

// 沿收绳方向逐步推进指定电机，直到负载阈值或机械角停稳条件成立。
static bool findDualTightPoint(SystemRuntimeContext* sharedData,
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
    uint32_t lastCommandMs = calibrationNowMs();
    uint32_t lastTraceMs = 0;
    uint32_t windowStartMs = calibrationNowMs();
    int32_t windowStartAbs = startAbs;
    int32_t windowStartEncoder = startEncoder;
    uint8_t angleStallStable = 0;

    while (autoCalibrationStillOwnsControl(sharedData)) {
        const uint32_t nowMs = calibrationNowMs();
        uint16_t currentRaw = 0;
        int32_t currentEncoder = 0;
        int32_t currentAbs = 0;
        if (!readLatestEncoderSample(sharedData, jointIndex, &currentRaw, &currentEncoder) ||
            !readLatestMotorServoAbs(sharedData, motorChannel, &currentAbs)) {
            FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL %s lost feedback J%02u M%02u\r\n",
                          phase,
                          jointIndex,
                          motorChannel);
            setDualFeedbackLostDiag(sharedData,
                                    CALIB_DIAG_PHASE_FIND_MAX,
                                    jointIndex,
                                    motorChannel,
                                    currentEncoder,
                                    currentAbs,
                                    (1UL << motorChannel));
            return false;
        }

        int16_t loadRaw = 0;
        const bool hasLoad = readLatestMotorServoLoad(sharedData, motorChannel, &loadRaw);
        const int16_t loadMag = hasLoad ? mmhand::servo::loadMagnitude(loadRaw) : 0;
        const int32_t totalAbsDelta = calibrationAbs32(currentAbs - startAbs);
        const int32_t totalEncoderDelta =
            calibrationWrapEncoderDelta(currentEncoder, startEncoder);
        const float totalAngleDeltaDeg =
            fabsf(calibrationEncoderCountToDeg(totalEncoderDelta));
        const bool loadHigh = hasLoad && loadMag >= loadThreshold;
        bool tightFoundByStall = false;

        if ((uint32_t)(nowMs - windowStartMs) >= params.windowMs) {
            const int32_t windowAbsDelta = calibrationAbs32(currentAbs - windowStartAbs);
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
                    setDualPublishFailedDiag(sharedData,
                                             CALIB_DIAG_PHASE_FIND_MAX,
                                             jointIndex,
                                             motorChannel,
                                             currentAbs,
                                             (1UL << motorChannel) |
                                                 (1UL << holdMotorChannel));
                    return false;
                }
            } else if (!publishMotorCalibrationTarget(sharedData, motorChannel, currentAbs, params)) {
                setDualPublishFailedDiag(sharedData,
                                         CALIB_DIAG_PHASE_FIND_MAX,
                                         jointIndex,
                                         motorChannel,
                                         currentAbs,
                                         (1UL << motorChannel));
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
                        setDualPublishFailedDiag(sharedData,
                                                 CALIB_DIAG_PHASE_FIND_MAX,
                                                 jointIndex,
                                                 motorChannel,
                                                 commandAbs,
                                                 (1UL << motorChannel) |
                                                     (1UL << holdMotorChannel));
                        return false;
                    }
                } else {
                    if (!publishMotorCalibrationTarget(sharedData, motorChannel, commandAbs, params)) {
                        setDualPublishFailedDiag(sharedData,
                                                 CALIB_DIAG_PHASE_FIND_MAX,
                                                 jointIndex,
                                                 motorChannel,
                                                 commandAbs,
                                                 (1UL << motorChannel));
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

        calibrationWaitMs(params.loopMs);
    }
    FW_LOG_INFO_PRINTF("[AUTO CALIB] CANCEL %s J%02u M%02u state=%u owner=%u enabled=%u\r\n",
                  phase,
                  jointIndex,
                  motorChannel,
                  sharedData ? (unsigned)sharedData->system_state : 255,
                  sharedData ? (unsigned)sharedData->servo_target_owner : 255,
                  sharedData ? (unsigned)sharedData->control_enabled : 255);
    setDualControlLostDiag(sharedData,
                           CALIB_DIAG_PHASE_FIND_MAX,
                           jointIndex,
                           motorChannel,
                           (1UL << motorChannel) |
                               (holdOtherMotor ? (1UL << holdMotorChannel) : 0));
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
bool isWorkspaceDualJoint(uint8_t jointIndex)
{
    return workspaceCacheIndex(jointIndex) >= 0;
}

// 每轮标定开始时清空 RAM 姿态缓存，禁止跨任务复用旧机械姿态。
void resetDualWorkspaceCalibrationCache(void)
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
    SystemRuntimeContext* sharedData,
    uint8_t jointIndex,
    uint8_t* outPrimaryMotor,
    uint8_t* outSecondaryMotor,
    uint32_t* outMotorMask)
{
    uint8_t primaryMotor = 0;
    uint8_t secondaryMotor = 0;
    if (!sharedData ||
        !getDualCalibrationMotors(jointIndex,
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
    if (!sharedData->servoFeedbackQueue.peek(&servoData) ||
        (uint32_t)(calibrationNowMs() - servoData.timestamp) >
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
bool buildDynamicWorkspaceMotorMask(
    SystemRuntimeContext* sharedData,
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
void setCalibrationWorkspaceReadMask(SystemRuntimeContext* sharedData,
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
static uint8_t buildDualWorkspacePlan(SystemRuntimeContext* sharedData,
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
static void setWorkspacePlanFailureDiag(SystemRuntimeContext* sharedData,
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
    if (getDualCalibrationMotors(helperJoint,
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
static bool waitDualWorkspaceTargets(SystemRuntimeContext* sharedData,
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
                               sharedAtomicLoad(
                                   &sharedData->calibration_target_apply_status) ==
                                       CALIBRATION_TARGET_APPLY_PENDING
                                   ? CALIB_DIAG_REASON_TARGET_APPLY_TIMEOUT
                                   : CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
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

        const int32_t primaryErr = calibrationAbs32(primaryAbs - primaryTargetAbs);
        const int32_t secondaryErr = calibrationAbs32(secondaryAbs - secondaryTargetAbs);
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

        const uint32_t nowMs = calibrationNowMs();
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
        calibrationWaitMs(params.loopMs);
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
static bool waitWorkspaceFeedbackKnown(SystemRuntimeContext* sharedData,
                                       uint8_t jointIndex,
                                       uint8_t primaryMotor,
                                       uint8_t secondaryMotor,
                                       int32_t* primaryAbs,
                                       int32_t* secondaryAbs)
{
    const uint32_t motorMask = (1UL << primaryMotor) | (1UL << secondaryMotor);
    const uint32_t startMs = calibrationNowMs();
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
            FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE fail Turn UNKNOWN J%02u mask=0x%08lX; select joint and recalibrate\r\n",
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

        if ((uint32_t)(calibrationNowMs() - startMs) >= kInitialFeedbackWaitMs) {
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
        calibrationWaitMs(20);
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
static bool preflightDualWorkspacePlan(SystemRuntimeContext* sharedData,
                                       uint8_t targetJoint,
                                       const DualWorkspacePlanStep* steps,
                                       uint8_t count)
{
    for (uint8_t i = 0; i < count; ++i) {
        const DualWorkspacePlanStep& step = steps[i];
        uint8_t primaryMotor = 0;
        uint8_t secondaryMotor = 0;
        if (!getDualCalibrationMotors(step.jointIndex,
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
static bool tightenWorkspaceMotor(SystemRuntimeContext* sharedData,
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
    uint32_t lastCommandMs = calibrationNowMs();
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
                                   sharedAtomicLoad(
                                       &sharedData->calibration_target_apply_status) ==
                                           CALIBRATION_TARGET_APPLY_PENDING
                                       ? CALIB_DIAG_REASON_TARGET_APPLY_TIMEOUT
                                       : CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
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
        const uint32_t nowMs = calibrationNowMs();
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
                                       sharedAtomicLoad(
                                           &sharedData->calibration_target_apply_status) ==
                                               CALIBRATION_TARGET_APPLY_PENDING
                                           ? CALIB_DIAG_REASON_TARGET_APPLY_TIMEOUT
                                           : CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
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
        calibrationWaitMs(params.loopMs);
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
static bool ensureWorkspaceDualPose(SystemRuntimeContext* sharedData,
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
    if (!getDualCalibrationMotors(jointIndex, &primaryMotor, &secondaryMotor)) {
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
        const int32_t primaryError = calibrationAbs32(primaryAbs - primaryTarget);
        const int32_t secondaryError = calibrationAbs32(secondaryAbs - secondaryTarget);
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
static bool prepareWorkspaceForDualCalibration(SystemRuntimeContext* sharedData,
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
    if (!getDualCalibrationMotors(targetJoint,
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
static bool findDualSecondaryPreloadPoint(SystemRuntimeContext* sharedData,
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
    uint32_t lastCommandMs = calibrationNowMs();

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
        const uint32_t nowMs = calibrationNowMs();
        uint16_t currentRaw = 0;
        int32_t currentEncoder = 0;
        int32_t currentAbs = 0;
        if (!readLatestEncoderSample(sharedData, jointIndex, &currentRaw, &currentEncoder) ||
            !readLatestMotorServoAbs(sharedData, secondaryMotor, &currentAbs)) {
            FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL DUAL_PRELOAD_SECONDARY lost feedback J%02u M%02u\r\n",
                          jointIndex,
                          secondaryMotor);
            setDualFeedbackLostDiag(sharedData,
                                    CALIB_DIAG_PHASE_DUAL_PRELOAD,
                                    jointIndex,
                                    secondaryMotor,
                                    currentEncoder,
                                    currentAbs,
                                    (1UL << primaryMotor) |
                                        (1UL << secondaryMotor));
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
            const bool published = publishDualMotorCalibrationTargets(
                sharedData,
                secondaryMotor,
                currentAbs,
                primaryMotor,
                primaryHoldAbs,
                params);
            if (!published) {
                setDualPublishFailedDiag(sharedData,
                                         CALIB_DIAG_PHASE_DUAL_PRELOAD,
                                         jointIndex,
                                         secondaryMotor,
                                         currentAbs,
                                         (1UL << primaryMotor) |
                                             (1UL << secondaryMotor));
            }
            return published;
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
                    setDualPublishFailedDiag(sharedData,
                                             CALIB_DIAG_PHASE_DUAL_PRELOAD,
                                             jointIndex,
                                             secondaryMotor,
                                             commandAbs,
                                             (1UL << primaryMotor) |
                                                 (1UL << secondaryMotor));
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

        calibrationWaitMs(params.loopMs);
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
    setDualControlLostDiag(sharedData,
                           CALIB_DIAG_PHASE_DUAL_PRELOAD,
                           jointIndex,
                           secondaryMotor,
                           (1UL << primaryMotor) | (1UL << secondaryMotor));
    return false;
}

// 主放/副收同步交换，直到副电机达到最终负载且磁编行程满足要求。
static bool findDualExchangeLimitPoint(SystemRuntimeContext* sharedData,
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
    const uint32_t startMs = calibrationNowMs();
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
        const uint32_t nowMs = calibrationNowMs();
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
            setDualFeedbackLostDiag(sharedData,
                                    CALIB_DIAG_PHASE_DUAL_EXCHANGE,
                                    jointIndex,
                                    secondaryMotor,
                                    currentSecondaryAbs,
                                    currentPrimaryAbs,
                                    (1UL << primaryMotor) |
                                        (1UL << secondaryMotor));
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
                calibrationAbs32(currentSecondaryAbs - windowStartSecondaryAbs);
            const int32_t windowPrimaryAbsDelta =
                calibrationAbs32(currentPrimaryAbs - windowStartPrimaryAbs);
            const bool enoughEncoderTravel =
                totalAngleDeltaDeg >= kReverseMinFollowDeg;
            const bool angleStalled =
                windowAngleDeltaDeg <= params.stallActualDeg;
            const bool secondaryStillReeling =
                windowSecondaryAbsDelta >= params.windowServoDeltaCounts;
            const bool primaryStillPaying =
                windowPrimaryAbsDelta >= params.windowServoDeltaCounts;
            // 副腱已经达到终点负载，但关节累计行程仍不足时，连续两个窗口
            // 没有角度变化说明绳路可能卡死或磁编码器没有跟随，不能继续收绳。
            if (secondaryLoadHigh && !enoughEncoderTravel && angleStalled) {
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

            if (angleStallStable >= kAngleStallStableSamples) {
                FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL DUAL_EXCHANGE_STALLED J%02u secondary=M%02u totalDeg=%.3f/%.3f load=%d/%d stable=%u/%u\r\n",
                              jointIndex,
                              secondaryMotor,
                              totalAngleDeltaDeg,
                              kReverseMinFollowDeg,
                              loadMag,
                              finalLoadThreshold,
                              (unsigned)angleStallStable,
                              (unsigned)kAngleStallStableSamples);
                setCalibrationDiag(sharedData,
                                   CALIB_DIAG_REASON_JOINT_STEP_FAILED,
                                   CALIB_DIAG_PHASE_DUAL_EXCHANGE,
                                   jointIndex,
                                   secondaryMotor,
                                   loadMag,
                                   finalLoadThreshold,
                                   totalEncoderDelta,
                                   (1UL << primaryMotor) |
                                       (1UL << secondaryMotor));
                return false;
            }

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
            const bool published = publishDualMotorCalibrationTargets(
                sharedData,
                secondaryMotor,
                currentSecondaryAbs,
                primaryMotor,
                currentPrimaryAbs,
                params);
            if (!published) {
                setDualPublishFailedDiag(sharedData,
                                         CALIB_DIAG_PHASE_DUAL_EXCHANGE,
                                         jointIndex,
                                         secondaryMotor,
                                         currentSecondaryAbs,
                                         (1UL << primaryMotor) |
                                             (1UL << secondaryMotor));
            }
            return published;
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
                    setDualPublishFailedDiag(sharedData,
                                             CALIB_DIAG_PHASE_DUAL_EXCHANGE,
                                             jointIndex,
                                             secondaryMotor,
                                             secondaryCommandAbs,
                                             (1UL << primaryMotor) |
                                                 (1UL << secondaryMotor));
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

        calibrationWaitMs(params.loopMs);
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
    setDualControlLostDiag(sharedData,
                           CALIB_DIAG_PHASE_DUAL_EXCHANGE,
                           jointIndex,
                           secondaryMotor,
                           (1UL << primaryMotor) | (1UL << secondaryMotor));
    return false;
}

// 双腱关节完整标定流程：
// 主紧端 -> 副松 3200 -> 副预紧 190 -> 主放副收至副 250 ->
// 主补紧 -> 构建 V4 映射与两路 Tendon LUT -> 登记 READY 姿态。
static bool runDualAntagonistAutoCalibrationJoint(SystemRuntimeContext* sharedData,
                                                  uint8_t jointIndex,
                                                  const AutoCalibrationParams& params,
                                                  DualWorkspaceCacheEntry* workspaceState)
{
    uint8_t primaryMotor = 0;
    uint8_t secondaryMotor = 0;
    if (!getDualCalibrationMotors(jointIndex, &primaryMotor, &secondaryMotor)) {
        setCalibrationDiagIfEmpty(sharedData,
                                  CALIB_DIAG_REASON_MOTOR_MAPPING_INVALID,
                                  CALIB_DIAG_PHASE_INIT,
                                  jointIndex,
                                  0xFF,
                                  jointIndex,
                                  -1,
                                  -1,
                                  0);
        return false;
    }
    setRuntimeEncoderDirection(jointIndex, kDefaultEncoderDirection[jointIndex]);
    clearCalibratedMotorAbsLimitsForJoint(sharedData, jointIndex);
    const int8_t primaryShortenSign = getMotorTendonShortenAbsSign(primaryMotor);
    const int8_t secondaryShortenSign = getMotorTendonShortenAbsSign(secondaryMotor);
    const int16_t primaryTightLoadThreshold = kDualPrimaryTightLoadThreshold;
    const int16_t secondaryPreloadLoadThreshold = kDualSecondaryPreloadLoadThreshold;
    const int16_t secondaryFinalLoadThreshold = kDualSecondaryFinalLoadThreshold;

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
        setDualFeedbackLostDiag(sharedData,
                                CALIB_DIAG_PHASE_INIT,
                                jointIndex,
                                primaryMotor,
                                primaryAbs,
                                secondaryAbs,
                                (1UL << primaryMotor) |
                                    (1UL << secondaryMotor));
        return false;
    }

    FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_BEGIN J%02u primary=M%02u primaryShortenSign=%d secondary=M%02u secondaryShortenSign=%d slack=%ld primaryLoad=%d secondaryPreloadLoad=%d secondaryFinalLoad=%d startDeg=%.3f tightDeg=dynamic initialEncoder=%ld primaryAbs=%ld secondaryAbs=%ld\r\n",
                  jointIndex,
                  primaryMotor,
                  (int)primaryShortenSign,
                  secondaryMotor,
                  (int)secondaryShortenSign,
                  (long)kDualInitialSlackCounts,
                   primaryTightLoadThreshold,
                   secondaryPreloadLoadThreshold,
                   secondaryFinalLoadThreshold,
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
            calibrationAbs32(primaryAbs - workspaceState->primaryTightAbs);
        const int32_t secondaryWorkspaceError =
            calibrationAbs32(secondaryAbs - workspaceState->secondaryReleaseAbs);
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
            setDualFeedbackLostDiag(sharedData,
                                    CALIB_DIAG_PHASE_FIND_MAX,
                                    jointIndex,
                                    primaryMotor,
                                    initialEncoder,
                                    primaryAbs,
                                    (1UL << primaryMotor));
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
        setDualFeedbackLostDiag(sharedData,
                                CALIB_DIAG_PHASE_DUAL_PRELOAD,
                                jointIndex,
                                secondaryMotor,
                                secondaryStartEncoder,
                                secondaryStartAbs,
                                (1UL << secondaryMotor));
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
        setDualFeedbackLostDiag(sharedData,
                                CALIB_DIAG_PHASE_DUAL_EXCHANGE,
                                jointIndex,
                                secondaryMotor,
                                exchangeStartEncoder,
                                0,
                                (1UL << primaryMotor) |
                                    (1UL << secondaryMotor));
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

    // 副电机达到最终 250 后直接提交交换端点。这里不再额外补紧主腱；
    // 整轮标定结束后由状态机自动执行 Rope Init，统一建立 READY 姿态。
    int32_t primaryReadyAbs = primaryNeg3Abs;
    int32_t secondaryReadyAbs = secondaryNeg3Abs;
    uint16_t readyRaw = neg3Raw;
    int32_t readyEncoder = neg3Encoder;
    int16_t secondaryReadyLoad = neg3Load;

    const int32_t signedSpan = signedEncoderDelta(neg3Encoder, primary80Encoder);
    const int8_t encoderTravelSign = 1;
    const int32_t resultStartEncoder =
        encoderForPositiveTravel(neg3Encoder, encoderTravelSign);
    const int32_t resultTightEncoder =
        encoderForPositiveTravel(primary80Encoder, encoderTravelSign);
    const int32_t resultSpan =
        calibrationForwardEncoderDelta(resultStartEncoder, resultTightEncoder);
    if (resultSpan <= 0 || resultSpan >= kRejectNearFullWrapSpanCounts) {
        FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL DUAL_RESULT J%02u invalid span=%ld neg3=%ld pos80=%ld signed=%ld\r\n",
                      jointIndex,
                      (long)resultSpan,
                      (long)neg3Encoder,
                      (long)primary80Encoder,
                      (long)signedSpan);
        setCalibrationDiagIfEmpty(sharedData,
                                  CALIB_DIAG_REASON_RESULT_SPAN_INVALID,
                                  CALIB_DIAG_PHASE_DUAL_EXCHANGE,
                                  jointIndex,
                                  primaryMotor,
                                  resultSpan,
                                  neg3Encoder,
                                  primary80Encoder,
                                  (1UL << primaryMotor) |
                                      (1UL << secondaryMotor));
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
            resultStartEncoder + roundToInt32((float)resultSpan * zeroFraction));
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
        setCalibrationDiagIfEmpty(sharedData,
                                  CALIB_DIAG_REASON_RESULT_BUILD_FAILED,
                                  CALIB_DIAG_PHASE_DUAL_EXCHANGE,
                                  jointIndex,
                                  primaryMotor,
                                  fitSamples.count,
                                  resultSpan,
                                  0,
                                  (1UL << primaryMotor) |
                                      (1UL << secondaryMotor));
        return false;
    }
    const bool lutBuilt =
        buildTendonLutFromRawSamples(jointIndex,
                                     &result,
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
        !readyMap.valid) {
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

    // 标定成功的判据是机械端点、Auto 区间和 LUT 已经建立完成，当前交换
    // 端点不要求立即位于 READY 工作区。若仍低于下边界，保留真实角度并
    // 提交结果；整轮标定结束后由状态机自动执行 Rope Init 回到工作区。
    const bool finalPoseInsideReadyWindow =
        readyMap.outsideCounts >= -kFeedbackReadyEnterOutsideToleranceCounts;
    if (!finalPoseInsideReadyWindow) {
        FW_LOG_WARN_PRINTF("[AUTO CALIB] DUAL_FINAL_OUTSIDE_READY J%02u raw=%u oriented=%ld outside=%ld enterTolerance=%ld; calibration result kept, Rope Init required\r\n",
                           jointIndex,
                           (unsigned)readyRaw,
                           (long)readyEncoder,
                           (long)readyMap.outsideCounts,
                           (long)kFeedbackReadyEnterOutsideToleranceCounts);
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

    if (!commitCalibrationResult(sharedData,
                                 jointIndex,
                                 result,
                                 readyMap.urdfDeg)) {
        setCalibrationDiagIfEmpty(sharedData,
                                  CALIB_DIAG_REASON_RESULT_COMMIT_FAILED,
                                  CALIB_DIAG_PHASE_DUAL_EXCHANGE,
                                  jointIndex,
                                  primaryMotor,
                                  jointIndex,
                                  result.encoderSpan,
                                  result.tendonLutCount,
                                  (1UL << primaryMotor) |
                                      (1UL << secondaryMotor));
        return false;
    }
    if (!commitJointServoMultiTurnTrust(sharedData, jointIndex)) {
        setCalibrationDiagIfEmpty(sharedData,
                                  CALIB_DIAG_REASON_MULTITURN_TRUST_FAILED,
                                  CALIB_DIAG_PHASE_DUAL_EXCHANGE,
                                  jointIndex,
                                  primaryMotor,
                                  jointIndex,
                                  primaryMotor,
                                  secondaryMotor,
                                  (1UL << primaryMotor) |
                                      (1UL << secondaryMotor));
        return false;
    }
    setCalibratedMotorAbsLimit(sharedData, primaryMotor, primary80Abs);
    setCalibratedMotorAbsLimit(sharedData, secondaryMotor, secondaryNeg3Abs);

    if (workspaceState && finalPoseInsideReadyWindow) {
        // 交换端点若已在 READY 窗口内，可作为后续同组关节的 A 姿态缓存；
        // 整轮标定完成后仍会执行一次 Rope Init 统一确认工作空间。
        workspaceState->primaryMotor = primaryMotor;
        workspaceState->secondaryMotor = secondaryMotor;
        workspaceState->primaryReleaseAbs = primaryReadyAbs;
        workspaceState->secondaryTightAbs = secondaryReadyAbs;
        workspaceState->currentPose = DUAL_WORKSPACE_POSE_A;
        workspaceState->valid = 1;
        FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_POST_CALIB_CACHE J%02u pose=A primaryExchange=M%02u:%ld secondaryExchange=M%02u:%ld secondaryLoad=%d\r\n",
                           (unsigned)jointIndex,
                           (unsigned)primaryMotor,
                           (long)primaryReadyAbs,
                           (unsigned)secondaryMotor,
                           (long)secondaryReadyAbs,
                           secondaryReadyLoad);
    } else if (workspaceState) {
        // 区间外姿态不能登记为可复用 A 姿态，否则后续标定会错误跳过 Rope Init。
        workspaceState->valid = 0;
        FW_LOG_WARN_PRINTF("[AUTO CALIB] WORKSPACE_POST_CALIB_CACHE_SKIP J%02u outside=%ld; pose remains unknown until Rope Init\r\n",
                           (unsigned)jointIndex,
                           (long)readyMap.outsideCounts);
    }

    FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_RESULT J%02u neg3Raw=%u neg3=%ld pos80Raw=%u pos80=%ld span=%ld zero=%ld primary80Abs=%ld primaryNeg3Abs=%ld secondaryPreloadRaw=%u secondaryPreloadEncoder=%ld secondaryPreloadAbs=%ld secondaryNeg3Abs=%ld loadPrimary=%d loadSecondaryPreload=%d loadSecondaryNeg3=%d exchangeRaw=%u exchangeEncoder=%ld exchangeDeg=%.3f primaryExchangeAbs=%ld secondaryExchangeAbs=%ld secondaryExchangeLoad=%d\r\n",
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
                   (long)secondaryReadyAbs,
                   secondaryReadyLoad);
    FW_LOG_INFO_PRINTF("[AUTO CALIB] AUTO_SAMPLE_RANGE J%02u startDeg=%.3f zeroDeg=0.000 tightDeg=%.3f totalDeg=%.3f urdfMax=%.3f span=%ld\r\n",
                  jointIndex,
                  result.startDeg,
                  result.tightDeg,
                  result.totalDeg,
                  result.angleMax,
                  (long)result.encoderSpan);

    return true;
}

DualMotorCalibrationRunResult runDualMotorCalibrationMachine(
    SystemRuntimeContext* sharedData,
    uint8_t jointIndex,
    uint8_t sequenceIndex,
    uint8_t jointCount,
    uint32_t selectedJointMask,
    const AutoCalibrationParams& params)
{
    uint8_t primaryMotor = 0;
    uint8_t secondaryMotor = 0;
    if (!getDualCalibrationMotors(jointIndex, &primaryMotor, &secondaryMotor)) {
        setCalibrationDiagIfEmpty(sharedData,
                                  CALIB_DIAG_REASON_MOTOR_MAPPING_INVALID,
                                  CALIB_DIAG_PHASE_INIT,
                                  jointIndex,
                                  0xFF,
                                  jointIndex,
                                  -1,
                                  -1,
                                  0);
        return DUAL_MOTOR_CALIBRATION_FAILED;
    }

    DualWorkspaceCacheEntry* workspaceState = NULL;
    if (isWorkspaceDualJoint(jointIndex)) {
        if (!prepareWorkspaceForDualCalibration(sharedData, jointIndex, params)) {
            return DUAL_MOTOR_CALIBRATION_PREFLIGHT_FAILED;
        }
        const int8_t cacheIndex = workspaceCacheIndex(jointIndex);
        if (cacheIndex < 0) {
            setWorkspacePlanFailureDiag(sharedData,
                                        jointIndex,
                                        jointIndex,
                                        DUAL_WORKSPACE_POSE_NONE,
                                        DUAL_WORKSPACE_PLAN_STAGE_EXECUTE);
            return DUAL_MOTOR_CALIBRATION_PREFLIGHT_FAILED;
        }
        workspaceState = &g_dualWorkspaceCache[(uint8_t)cacheIndex];
    }

    FW_LOG_INFO_PRINTF("[AUTO CALIB] SEQUENCE_SERIAL begin J%02u (%u/%u) dual M%02u/M%02u\r\n",
                       jointIndex,
                       (unsigned)(sequenceIndex + 1),
                       (unsigned)jointCount,
                       primaryMotor,
                       secondaryMotor);
    if (!runDualAntagonistAutoCalibrationJoint(sharedData,
                                               jointIndex,
                                               params,
                                               workspaceState)) {
        if (!hasCalibrationDiag(sharedData)) {
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_JOINT_STEP_FAILED,
                               CALIB_DIAG_PHASE_DUAL_EXCHANGE,
                               jointIndex,
                               primaryMotor,
                               sequenceIndex,
                               jointCount,
                               secondaryMotor,
                               selectedJointMask);
        }
        return DUAL_MOTOR_CALIBRATION_FAILED;
    }

    FW_LOG_INFO_PRINTF("[AUTO CALIB] SEQUENCE_SERIAL done J%02u dual\r\n", jointIndex);
    return DUAL_MOTOR_CALIBRATION_COMPLETED;
}
