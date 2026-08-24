/**
 * @file SingleMotorCalibrationMachine.cpp
 * @brief 实现单电机关节的紧端搜索、松端搜索、LUT 构建和结果提交状态机。
 */
#include "SingleMotorCalibrationMachine.h"

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
#include "../../../safety/policies/CalibrationSafetyPolicy.h"

using namespace mmhand::config::calibration;
using namespace mmhand::calibration;

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
    bool initStarted;           // 初始反馈等待是否已启动。
    bool releaseCommandStarted; // 松绳阶段是否已发出首帧命令。
    int32_t preReleaseSyncTargetAbs; // 松绳前锁定的当前 ABS。
    int32_t preReleaseSyncWindowStartAbs; // 同步窗口起始 ABS。
    uint32_t preReleaseSyncWindowStartMs; // 同步窗口起始时间。
    uint8_t preReleaseSyncStableSamples;  // 同步连续稳定帧数。
    uint8_t releaseAngleStallStable;      // 松端角度停稳连续帧数。
    TendonRawFitBuffer fitSamples;        // 本轮 Tendon LUT 原始样本。
};
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
                                         SystemRuntimeContext* sharedData,
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
                                         SystemRuntimeContext* sharedData,
                                         const AutoCalibrationParams& params)
{
    if (!ctx || !sharedData) {
        return false;
    }

    uint16_t raw = 0;
    int32_t encoder = 0;
    int32_t absNow = 0;
    if (!readLatestEncoderSample(sharedData, ctx->jointIndex, &raw, &encoder) ||
        !readLatestMotorServoAbs(sharedData, ctx->motorChannel, &absNow)) {
        return false;
    }

    int16_t baselineLoadRaw = 0;
    const bool hasBaselineLoad =
        readLatestMotorServoLoad(sharedData, ctx->motorChannel, &baselineLoadRaw);
    ctx->baselineLoad = hasBaselineLoad ? mmhand::servo::loadMagnitude(baselineLoadRaw) : 0;
    ctx->startSearchRaw = raw;
    ctx->startSearchEncoder = encoder;
    ctx->startSearchAbs = absNow;
    ctx->startFound = false;
    ctx->startEncoder = 0;
    ctx->startServoAbs = 0;
    ctx->zeroEncoder = 0;
    ctx->zeroServoAbs = 0;
    ctx->encoderTravelSign = 1;
    ctx->windowStartEncoder = encoder;
    ctx->windowStartAbs = absNow;
    ctx->windowStartMs = calibrationNowMs();
    ctx->commandAbs = absNow;
    ctx->commandRemainder = 0;
    ctx->lastCommandMs = calibrationNowMs();
    ctx->lastTraceMs = 0;
    ctx->stateBeginMs = calibrationNowMs();
    ctx->state = AUTO_CALIB_STATE_TIGHTEN;

    FW_LOG_INFO_PRINTF("[AUTO CALIB] AUTO_SAMPLE_RANGE_BEGIN J%02u startDeg=%.3f tightDeg=pending totalDeg=pending\r\n",
                  ctx->jointIndex,
                  kLutStartDeg);
    FW_LOG_INFO_PRINTF("[AUTO CALIB] START J%02u M%02u abs=%ld raw=%u oriented=%ld dir=%d baselineLoad=%d speedCps=%ld\r\n",
                  ctx->jointIndex,
                  ctx->motorChannel,
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
// 锁回实际 ABS 并停稳；随后反向松到机械端点，生成 V4 映射和单路 Tendon LUT。
static void stepAutoCalibrationJoint(JointCalibContext* ctx,
                                     SystemRuntimeContext* sharedData,
                                     const AutoCalibrationParams& params,
                                     ServoTargetBatch_t* batch)
{
    if (!ctx ||
        ctx->state == AUTO_CALIB_STATE_DONE ||
        ctx->state == AUTO_CALIB_STATE_FAILED) {
        return;
    }

    const uint32_t nowMs = calibrationNowMs();
    if (!autoCalibrationStillOwnsControl(sharedData)) {
        FW_LOG_INFO_PRINTF("[AUTO CALIB] CANCEL J%02u state=%u owner=%u enabled=%u\r\n",
                      ctx->jointIndex,
                      sharedData ? (unsigned)sharedData->system_state : 255,
                      sharedData ? (unsigned)sharedData->servo_target_owner : 255,
                      sharedData ? (unsigned)sharedData->control_enabled : 255);
        setCalibrationDiagIfEmpty(
            sharedData,
            CALIB_DIAG_REASON_CONTROL_LOST,
            CALIB_DIAG_PHASE_SEQUENCE,
            ctx->jointIndex,
            ctx->motorChannel,
            sharedData ? sharedData->system_state : -1,
            sharedData ? sharedData->servo_target_owner : -1,
            sharedData ? sharedData->control_enabled : -1,
            sharedData ? sharedAtomicLoad(&sharedData->active_joint_mask) : 0);
        ctx->state = AUTO_CALIB_STATE_FAILED;
        return;
    }

    switch (ctx->state) {
        case AUTO_CALIB_STATE_INIT:
        {
            if (!sharedData ||
                ctx->jointIndex >= ENCODER_TOTAL_NUM ||
                sharedData->servo_target_owner != SERVO_TARGET_OWNER_CALIBRATION) {
                setCalibrationDiagIfEmpty(
                    sharedData,
                    CALIB_DIAG_REASON_CONTROL_LOST,
                    CALIB_DIAG_PHASE_INIT,
                    ctx->jointIndex,
                    0xFF,
                    sharedData ? sharedData->system_state : -1,
                    sharedData ? sharedData->servo_target_owner : -1,
                    sharedData ? sharedData->control_enabled : -1,
                    sharedData ? sharedAtomicLoad(&sharedData->active_joint_mask) : 0);
                failAutoCalibContext(ctx, "invalid init or owner");
                return;
            }

            // 当前非阻塞标定状态机直接使用本轮 AutoCalibrationParams，
            // 不再经过已经淘汰的逐关节静态参数表。
            ctx->commandSpeed = params.servoSpeed;
            ctx->commandAcc = params.servoAcc;
            const int motorChannel =
                findPrimaryMotorChannelForJoint(ctx->jointIndex);
            if (motorChannel < 0 || motorChannel >= SERVO_TOTAL_NUM) {
                setCalibrationDiagIfEmpty(sharedData,
                                          CALIB_DIAG_REASON_MOTOR_MAPPING_INVALID,
                                          CALIB_DIAG_PHASE_INIT,
                                          ctx->jointIndex,
                                          0xFF,
                                          ctx->jointIndex,
                                          motorChannel,
                                          -1,
                                          0);
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
                readLatestMotorServoAbs(sharedData, ctx->motorChannel, &absNow);
            const bool rawOk =
                readLatestMotorServoRaw(sharedData, ctx->motorChannel, &rawServo);
            if (!encoderOk || !absOk || !rawOk) {
                if ((nowMs - ctx->lastTraceMs) >= params.traceMs) {
                    ctx->lastTraceMs = nowMs;
                    logInitialFeedbackDiagnostic(sharedData,
                                               ctx->jointIndex,
                                               ctx->motorChannel,
                                               encoderOk,
                                               absOk,
                                               rawOk);
                }
                if ((uint32_t)(nowMs - ctx->stateBeginMs) < kInitialFeedbackWaitMs) {
                    return;
                }
                logInitialFeedbackDiagnostic(sharedData,
                                           ctx->jointIndex,
                                           ctx->motorChannel,
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
                FW_LOG_INFO_PRINTF("[AUTO CALIB] SKIP_NORMALIZE_TURN J%02u M%02u abs=%ld raw=%d turn=%ld; calibrate from current abs\r\n",
                              ctx->jointIndex,
                              ctx->motorChannel,
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
                !readLatestMotorServoAbs(sharedData, ctx->motorChannel, &currentAbs)) {
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
            const bool hasLoad =
                readLatestMotorServoLoad(sharedData, ctx->motorChannel, &loadRaw);
            const int16_t loadMag = hasLoad ? mmhand::servo::loadMagnitude(loadRaw) : 0;
            const int32_t totalAbsDelta = calibrationAbs32(currentAbs - ctx->startSearchAbs);
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
                if (p4::safety::calibrationReleaseExceeded(
                        actualReleaseCounts,
                        kSingleTendonMaxReleaseCounts)) {
                    // 超限后只保持实测 ABS，不再沿旧的松绳目标继续推进。
                    ctx->commandAbs = currentAbs;
                    appendJointCalibrationTarget(
                        batch,
                        ctx->jointIndex,
                        clampCalibrationServoAbs(currentAbs),
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
                    const int32_t windowAbsDelta = calibrationAbs32(currentAbs - ctx->windowStartAbs);
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
                    if (!appendJointCalibrationTarget(batch,
                                                             ctx->jointIndex,
                                                             clampCalibrationServoAbs(ctx->preReleaseSyncTargetAbs),
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
                            appendJointCalibrationTarget(batch,
                                                                ctx->jointIndex,
                                                                clampCalibrationServoAbs(ctx->commandAbs),
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
                        if (!appendJointCalibrationTarget(batch,
                                                                 ctx->jointIndex,
                                                                 clampCalibrationServoAbs(ctx->commandAbs),
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
                    if (p4::safety::calibrationReleaseExceeded(
                            commandReleaseCounts,
                            kSingleTendonMaxReleaseCounts)) {
                        // 在下一帧目标越过两圈前终止，避免 cmd 领先实际 ABS 后继续放绳。
                        ctx->commandAbs = currentAbs;
                        appendJointCalibrationTarget(
                            batch,
                            ctx->jointIndex,
                            clampCalibrationServoAbs(currentAbs),
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
                        appendJointCalibrationTarget(batch,
                                                            ctx->jointIndex,
                                                            clampCalibrationServoAbs(ctx->commandAbs),
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
                    if (!appendJointCalibrationTarget(batch,
                                                             ctx->jointIndex,
                                                             clampCalibrationServoAbs(ctx->commandAbs),
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
                const int32_t windowAbsDelta = calibrationAbs32(currentAbs - ctx->windowStartAbs);
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
                    ctx->zeroEncoder = currentEncoder;
                    ctx->zeroServoAbs = currentAbs;
                    const int32_t signedTightToMax =
                        signedEncoderDelta(ctx->zeroEncoder, ctx->startEncoder);
                    ctx->encoderTravelSign = 1;

                    const int32_t maxEncoder = ctx->startEncoder;
                    const int32_t maxServoAbs = ctx->startServoAbs;
                    const int32_t tightEncoder = ctx->zeroEncoder;
                    const int32_t tightServoAbs = ctx->zeroServoAbs;
                    if (maxServoAbs == tightServoAbs) {
                        setCalibrationDiagIfEmpty(
                            sharedData,
                            CALIB_DIAG_REASON_RESULT_SPAN_INVALID,
                            CALIB_DIAG_PHASE_RELEASE,
                            ctx->jointIndex,
                            ctx->motorChannel,
                            0,
                            tightServoAbs,
                            maxServoAbs,
                            (1UL << ctx->motorChannel));
                        failAutoCalibContext(ctx, "zero find has no servo travel");
                        return;
                    }
                    const int32_t resultTautEncoder =
                        encoderForPositiveTravel(ctx->zeroEncoder, ctx->encoderTravelSign);
                    const int32_t resultTightEncoder =
                        encoderForPositiveTravel(ctx->startEncoder, ctx->encoderTravelSign);
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
                            roundToInt32((float)resultSpan * zeroFraction));
                    const int32_t servoZeroAbs =
                        motorApplyTendonDelta(ctx->motorChannel,
                                              tightServoAbs,
                                              kServoZeroPullCounts);

                    // PRE_RELEASE_SYNC 的首样本早于真正拉紧阶段，不能代表最终紧端。
                    // 建表前用最终实测值补齐紧端和松端，保证末段插值不会只靠外推。
                    const bool tightEndpointStored =
                        appendTendonRawFitSample(&ctx->fitSamples,
                                                 maxEncoder,
                                                 maxServoAbs,
                                                 0);
                    const bool releaseEndpointStored =
                        appendTendonRawFitSample(&ctx->fitSamples,
                                                 tightEncoder,
                                                 tightServoAbs,
                                                 0);
                    if (!tightEndpointStored || !releaseEndpointStored) {
                        failAutoCalibContextWithDiag(
                            ctx,
                            sharedData,
                            "single tendon LUT endpoint buffer full",
                            CALIB_DIAG_REASON_RESULT_BUILD_FAILED,
                            CALIB_DIAG_PHASE_RELEASE,
                            ctx->motorChannel,
                            ctx->fitSamples.count,
                            maxServoAbs,
                            tightServoAbs,
                            0);
                        return;
                    }

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
                        buildTendonLutFromRawSamples(ctx->jointIndex,
                                                     &result,
                                                     ctx->motorChannel,
                                                     0xFF,
                                                     1,
                                                     ctx->fitSamples);
                    if (!lutBuilt) {
                        failAutoCalibContextWithDiag(ctx,
                                                     sharedData,
                                                     "single tendon LUT build failed",
                                                     CALIB_DIAG_REASON_RESULT_BUILD_FAILED,
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
                                                     CALIB_DIAG_REASON_RESULT_BUILD_FAILED,
                                                     CALIB_DIAG_PHASE_RELEASE,
                                                     ctx->motorChannel,
                                                     result.tendonLutCount,
                                                     result.tendonLutMotor[0],
                                                     lutMinAbs,
                                                     lutMaxAbs);
                        return;
                    }
                    FW_LOG_INFO_PRINTF(
                        "[AUTO CALIB] SINGLE_LUT_ENDPOINT J%02u M%02u "
                        "releaseAbs=%ld tightAbs=%ld lut=[%ld,%ld] samples=%u\r\n",
                        (unsigned)ctx->jointIndex,
                        (unsigned)ctx->motorChannel,
                        (long)tightServoAbs,
                        (long)maxServoAbs,
                        (long)result.tendonLutMotorAbs[0][0],
                        (long)result.tendonLutMotorAbs[0][result.tendonLutCount - 1U],
                        (unsigned)ctx->fitSamples.count);
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

                    if (!commitCalibrationResult(sharedData,
                                                 ctx->jointIndex,
                                                 result,
                                                 result.currentMinDeg)) {
                        setCalibrationDiagIfEmpty(
                            sharedData,
                            CALIB_DIAG_REASON_RESULT_COMMIT_FAILED,
                            CALIB_DIAG_PHASE_RELEASE,
                            ctx->jointIndex,
                            ctx->motorChannel,
                            ctx->jointIndex,
                            result.encoderSpan,
                            result.tendonLutCount,
                            (1UL << ctx->motorChannel));
                        failAutoCalibContext(ctx,
                                             "calibration result commit failed");
                        return;
                    }
                    ctx->targetAbs = currentAbs;
                    if (!commitJointServoMultiTurnTrust(sharedData,
                                                        ctx->jointIndex)) {
                        setCalibrationDiagIfEmpty(
                            sharedData,
                            CALIB_DIAG_REASON_MULTITURN_TRUST_FAILED,
                            CALIB_DIAG_PHASE_RELEASE,
                            ctx->jointIndex,
                            ctx->motorChannel,
                            ctx->jointIndex,
                            ctx->motorChannel,
                            -1,
                            (1UL << ctx->motorChannel));
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
                    FW_LOG_INFO_PRINTF("[AUTO CALIB] AUTO_SAMPLE_RANGE J%02u startDeg=%.3f zeroDeg=0.000 tightDeg=%.3f totalDeg=%.3f urdfMax=%.3f span=%ld\r\n",
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

                    if (!appendJointCalibrationTarget(
                            batch,
                            ctx->jointIndex,
                            clampCalibrationServoAbs(ctx->targetAbs),
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
                calibrationAbs32(currentAbs - ctx->preReleaseSyncTargetAbs);
            if (targetError > kPreReleaseSyncTargetToleranceCounts) {
                // 前一段收紧轨迹仍在执行。将保持目标更新为当前实测 ABS，
                // 并重新开始停稳计时。
                ctx->preReleaseSyncTargetAbs = currentAbs;
                ctx->preReleaseSyncWindowStartAbs = currentAbs;
                ctx->preReleaseSyncWindowStartMs = nowMs;
                ctx->preReleaseSyncStableSamples = 0;
            }

            if (!appendJointCalibrationTarget(batch,
                                                     ctx->jointIndex,
                                                     clampCalibrationServoAbs(ctx->preReleaseSyncTargetAbs),
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
                calibrationAbs32(currentAbs - ctx->preReleaseSyncWindowStartAbs);
            const bool targetReached =
                calibrationAbs32(currentAbs - ctx->preReleaseSyncTargetAbs) <=
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
                          (long)calibrationAbs32(currentAbs - ctx->preReleaseSyncTargetAbs),
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

SingleMotorCalibrationRunResult runSingleMotorCalibrationMachine(
    SystemRuntimeContext* sharedData,
    uint8_t jointIndex,
    const uint8_t* orderedJoints,
    uint8_t jointCount,
    uint32_t failedJointMask,
    uint8_t sequenceIndex,
    uint32_t selectedJointMask,
    const AutoCalibrationParams& params)
{
    JointCalibContext ctx;
    initAutoCalibContext(&ctx, jointIndex);
    FW_LOG_INFO_PRINTF("[AUTO CALIB] SEQUENCE_SERIAL begin J%02u (%u/%u)\r\n",
                       jointIndex,
                       (unsigned)(sequenceIndex + 1),
                       (unsigned)jointCount);

    while (ctx.state != AUTO_CALIB_STATE_DONE &&
           ctx.state != AUTO_CALIB_STATE_FAILED) {
        if (!autoCalibrationStillOwnsControl(sharedData)) {
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_CONTROL_LOST,
                               CALIB_DIAG_PHASE_SEQUENCE,
                               jointIndex,
                               0xFF,
                               sharedData ? sharedData->system_state : -1,
                               sharedData ? sharedData->servo_target_owner : -1,
                               sharedData ? sharedData->control_enabled : -1,
                               sharedData ? sharedAtomicLoad(&sharedData->active_joint_mask) : 0);
            return SINGLE_MOTOR_CALIBRATION_ABORTED;
        }

        ServoTargetBatch_t batch;
        memset(&batch, 0, sizeof(batch));
        batch.source = SERVO_TARGET_OWNER_CALIBRATION;
        batch.timestamp = calibrationNowMs();
        if (!appendCalibrationHoldTargetsExcept(sharedData,
                                                &batch,
                                                orderedJoints,
                                                jointCount,
                                                ctx.jointIndex,
                                                failedJointMask,
                                                params)) {
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_HOLD_TARGET_FAILED,
                               CALIB_DIAG_PHASE_SEQUENCE,
                               ctx.jointIndex,
                               0xFF,
                               sequenceIndex,
                               jointCount,
                               batch.count,
                               selectedJointMask);
            return SINGLE_MOTOR_CALIBRATION_ABORTED;
        }

        stepAutoCalibrationJoint(&ctx, sharedData, params, &batch);
        if (!publishCalibrationBatch(sharedData, &batch)) {
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
                               CALIB_DIAG_PHASE_SEQUENCE,
                               ctx.jointIndex,
                               0xFF,
                               batch.count,
                               sharedData ? sharedData->servo_target_owner : -1,
                               sharedData ? sharedData->system_state : -1,
                               selectedJointMask);
            return SINGLE_MOTOR_CALIBRATION_ABORTED;
        }
        calibrationWaitMs(params.loopMs);
    }

    if (ctx.state == AUTO_CALIB_STATE_FAILED) {
        if (!hasCalibrationDiag(sharedData)) {
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_JOINT_STEP_FAILED,
                               CALIB_DIAG_PHASE_SEQUENCE,
                               jointIndex,
                               ctx.motorChannel,
                               ctx.state,
                               ctx.releaseAngleStallStable,
                               sequenceIndex,
                               selectedJointMask);
        }
        return SINGLE_MOTOR_CALIBRATION_FAILED;
    }

    FW_LOG_INFO_PRINTF("[AUTO CALIB] SEQUENCE_SERIAL done J%02u\r\n", jointIndex);
    return SINGLE_MOTOR_CALIBRATION_COMPLETED;
}
