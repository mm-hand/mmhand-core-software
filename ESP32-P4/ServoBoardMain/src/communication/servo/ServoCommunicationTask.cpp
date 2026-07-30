/**
 * @file ServoCommunicationTask.cpp
 * @brief 调度四条舵机总线、应用目标批次并发布统一反馈与恢复状态。
 *
 * 本任务不再直接调用飞特 SDK。底层串口协议由 FeetechServoDriver 封装；
 * 这里负责“何时收发”和跨任务状态同步，后续还会继续拆出恢复管理器。
 */
#include "ServoCommunicationTask.h"

#include <string.h>

#include "../../hardware/HardwareMap.h"
#include "../../shared/SharedSafetyState.h"
#include "../../system/FirmwareLog.h"
#include "../../system/StateMachineTask.h"
#include "../../system/TaskRuntimeMonitor.h"
#include "ServoPositionRange.h"
#include "ServoFeedbackPublisher.h"
#if MULTITURN_NVS_PERSISTENCE_ENABLE
#include "../../persistence/MultiTurnNvsStore.h"
#endif


#include "../../hardware/servo/FeetechServoDriver.h"
#include "../../hardware/servo/ServoBusRegistry.h"
#include "../../recovery/MultiTurnRecoveryPolicy.h"
#include "../../actuator/ActuatorCommandGateway.h"


static const uint32_t kServoLoopGapLogThresholdMs = 50;
static const uint32_t kServoApplyStallLogThresholdMs = 50;
static const uint32_t kServoQueueDropLogIntervalMs = 200;
#if MULTITURN_NVS_PERSISTENCE_ENABLE
static const uint32_t kNvsRestoreWaitWindowMs = 3000;
static const uint32_t kHostHeartbeatOfflineMs = 1000;
static const uint32_t kNvsSaveMinIntervalMs = 500;
static const uint32_t kNvsOfflinePeriodicSaveMs = 2000;
static const uint16_t kNvsRestoreRawToleranceCounts = 256;
static const int32_t kNvsMoveAbsThreshold = 64;
#endif

using p4::recovery::completedMotionSaveReason;
using p4::recovery::isMultiTurnMotionState;
using p4::recovery::isRawNearTurnEdge;

/** 过渡期局部别名；总线实例的实际所有权已经收敛到 ServoBusRegistry。 */
static FeetechServoDriver* getBusByIndex(uint8_t busIndex)
{
    return servoBusRegistry().driver(busIndex);
}

static bool addReadTarget(uint8_t busIds[NUM_BUSES][MAX_SERVOS_PER_BUS],
                          uint8_t busCounts[NUM_BUSES],
                          uint8_t bus,
                          uint8_t id)
{
    if (bus >= NUM_BUSES) {
        return false;
    }
    for (uint8_t i = 0; i < busCounts[bus]; i++) {
        if (busIds[bus][i] == id) {
            return true;
        }
    }
    if (busCounts[bus] >= MAX_SERVOS_PER_BUS) {
        return false;
    }
    busIds[bus][busCounts[bus]++] = id;
    return true;
}

static bool restoreServoMultiTurnFromHost(TaskSharedData_t* sharedData,
                                          uint32_t motorMask,
                                          const int32_t* motorAbs,
                                          uint16_t rawToleranceCounts,
                                          bool clearRecoveryTransaction,
                                          uint32_t* restoredMask,
                                          uint32_t* rejectedMask)
{
    uint32_t restored = 0;
    uint32_t rejected = 0;
    if (!sharedData || !motorAbs) {
        if (restoredMask) *restoredMask = 0;
        if (rejectedMask) *rejectedMask = motorMask;
        return false;
    }

    const uint32_t requestMask = motorMask & validMotorChannelMask();
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        const uint32_t bit = (1UL << ch);
        if ((requestMask & bit) == 0) {
            continue;
        }

        const uint8_t busIndex = motorMap[ch].busIndex;
        const uint8_t id = motorMap[ch].servoID;
        FeetechServoDriver* bus = getBusByIndex(busIndex);
        if (!bus || id > MAX_SERVO_ID ||
            !bus->isMultiTurnConfigured(id) ||
            !bus->restoreAbsolutePosition(id, motorAbs[ch], rawToleranceCounts)) {
            rejected |= bit;
            continue;
        }
        if (clearRecoveryTransaction) {
            // New Device Reset 是用户确认“当前 raw 即新 turn0”的显式恢复。
            // 恢复成功后必须同时结束热插拔事务并清除 recoveryFailed；
            // 否则共享 Unknown 虽已清除，状态机仍会被旧失败位图阻塞。
            bus->finishRecoveryAttempt(id, true, millis());
        }
        restored |= bit;
    }

    if (restored != 0) {
        sharedAtomicFetchAnd(&sharedData->multi_turn_unknown_bitmap, ~restored);
    }

    if (restoredMask) *restoredMask = restored;
    if (rejectedMask) *rejectedMask = rejected;
    FW_LOG_INFO_PRINTF("[SERVO RESTORE] request=0x%08lX restored=0x%08lX rejected=0x%08lX unknown=0x%08lX tol=%u\r\n",
                  (unsigned long)requestMask,
                  (unsigned long)restored,
                  (unsigned long)rejected,
                  (unsigned long)sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap),
                  (unsigned)rawToleranceCounts);
    return (requestMask != 0) && ((requestMask & ~restored) == 0);
}

static void beginServoBuses()
{
    servoBusRegistry().begin();
}

static void configureMappedServosForMultiTurn()
{
    servoBusRegistry().configureMappedServosForMultiTurn();
}

static void recordServoRangeReject(TaskSharedData_t* sharedData,
                                   uint8_t motorChannel,
                                   int32_t logicalTarget,
                                   int32_t logicalOffset)
{
    if (!sharedData || motorChannel >= SERVO_TOTAL_NUM) {
        return;
    }
    const ServoLogicalRange range =
        servoLogicalRangeFromOffset(logicalOffset);
    int64_t hardwareTarget =
        (int64_t)logicalTarget - (int64_t)logicalOffset;
    if (hardwareTarget > INT32_MAX) hardwareTarget = INT32_MAX;
    if (hardwareTarget < INT32_MIN) hardwareTarget = INT32_MIN;
    {
        SharedSafetyGuard guard;
        sharedData->servo_range_reject_motor = motorChannel;
        sharedData->servo_range_reject_logical_target = logicalTarget;
        sharedData->servo_range_reject_hardware_target =
            (int32_t)hardwareTarget;
        sharedData->servo_range_reject_logical_min = range.minAbs;
        sharedData->servo_range_reject_logical_max = range.maxAbs;
        sharedAtomicAddFetch(&sharedData->servo_range_reject_sequence,
                             (uint32_t)1);
    }
    FW_LOG_WARN_PRINTF(
        "[SERVO RANGE REJECT] M%02u logical=%ld hardware=%ld offset=%ld "
        "logicalRange=[%ld,%ld] hardwareRange=[%ld,%ld]\r\n",
        (unsigned)motorChannel,
        (long)logicalTarget,
        (long)hardwareTarget,
        (long)logicalOffset,
        (long)range.minAbs,
        (long)range.maxAbs,
        (long)kServoHardwareCommandMin,
        (long)kServoHardwareCommandMax);
}

static void applyServoTargetBatch(TaskSharedData_t* sharedData,
                                  const ServoTargetBatch_t& batch)
{
    bool busWritePending[NUM_BUSES] = {false};
    const uint32_t overflowMask = sharedData
        ? sharedAtomicLoad(&sharedData->servo_turn_overflow_bitmap)
        : 0;
    for (uint8_t i = 0; i < batch.count && i < SERVO_TARGET_BATCH_MAX; i++) {
        const ServoTargetCommand_t& cmd = batch.commands[i];
        FeetechServoDriver* bus = getBusByIndex(cmd.busIndex);
        if (!bus) {
            continue;
        }
        const int motorChannel =
            findMotorChannelByBusId(cmd.busIndex, cmd.servoId);
        if (motorChannel >= 0 &&
            (overflowMask & (1UL << (uint8_t)motorChannel)) != 0) {
            // hardware ABS 已越过 ST3215 的实际多圈行程，禁止任何控制命令
            // 再次打开力矩；用户必须先执行 Reset Zero。
            bus->setTorqueEnabled(cmd.servoId, false);
            continue;
        }
        if (bus->setTarget(cmd.servoId,
                           cmd.position,
                           cmd.speed,
                           cmd.acc)) {
            busWritePending[cmd.busIndex] = true;
        } else {
            if (motorChannel >= 0) {
                recordServoRangeReject(sharedData,
                                       (uint8_t)motorChannel,
                                       cmd.position,
                                       bus->getLogicalOffset(cmd.servoId));
            }
        }
    }

    for (uint8_t busIndex = 0; busIndex < NUM_BUSES; busIndex++) {
        if (busWritePending[busIndex]) {
            FeetechServoDriver* bus = getBusByIndex(busIndex);
            if (bus) {
                bus->syncWriteAll();
            }
        }
    }
}

static void emergencyTorqueOffMappedServos(TaskSharedData_t* sharedData)
{
    p4::actuator::clearPending(sharedData);

    uint8_t disabledIds[NUM_BUSES][MAX_SERVOS_PER_BUS] = {0};
    uint8_t disabledCounts[NUM_BUSES] = {0};
    uint32_t requestedMask = 0;
    uint32_t torqueDisabledMask = 0;
    uint32_t torqueDisableFailedMask = 0;

    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        const uint32_t bit = (1UL << ch);
        const uint8_t busIndex = motorMap[ch].busIndex;
        const uint8_t id = motorMap[ch].servoID;
        if (busIndex >= NUM_BUSES || id > MAX_SERVO_ID) {
            continue;
        }

        requestedMask |= bit;
        bool alreadyDisabled = false;
        for (uint8_t i = 0; i < disabledCounts[busIndex]; i++) {
            if (disabledIds[busIndex][i] == id) {
                alreadyDisabled = true;
                break;
            }
        }
        if (alreadyDisabled) {
            torqueDisabledMask |= bit;
            continue;
        }
        if (disabledCounts[busIndex] >= MAX_SERVOS_PER_BUS) {
            torqueDisableFailedMask |= bit;
            continue;
        }

        FeetechServoDriver* bus = getBusByIndex(busIndex);
        if (!bus) {
            torqueDisableFailedMask |= bit;
            continue;
        }

        if (bus->setTorqueEnabled(id, false)) {
            torqueDisabledMask |= bit;
            disabledIds[busIndex][disabledCounts[busIndex]++] = id;
        } else {
            torqueDisableFailedMask |= bit;
        }
    }

    FW_LOG_INFO_PRINTF("[SERVO E-STOP] torque-off request=0x%08lX disabled=0x%08lX failed=0x%08lX\r\n",
                  (unsigned long)requestedMask,
                  (unsigned long)torqueDisabledMask,
                  (unsigned long)torqueDisableFailedMask);
}

static void processHostMultiTurnRestore(TaskSharedData_t* sharedData, uint32_t* lastToken)
{
    if (!sharedData || !lastToken) {
        return;
    }
    const uint32_t token =
        sharedAtomicLoad(&sharedData->restore_multi_turn_request_token);
    if (token == *lastToken) {
        return;
    }
    *lastToken = token;

    uint32_t motorMask = 0;
    int32_t motorAbs[SERVO_TOTAL_NUM] = {0};
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(5)) != pdTRUE) {
        return;
    }
    motorMask = sharedData->restore_multi_turn_mask;
    memcpy(motorAbs, sharedData->restore_multi_turn_abs, sizeof(motorAbs));
    if (lock) {
        xSemaphoreGive(lock);
    }

    uint32_t restoredMask = 0;
    uint32_t rejectedMask = 0;
    restoreServoMultiTurnFromHost(sharedData,
                                  motorMask,
                                  motorAbs,
                                  256,
                                  true,
                                  &restoredMask,
                                  &rejectedMask);
    sharedData->restore_multi_turn_restored_mask = restoredMask;
    sharedData->restore_multi_turn_rejected_mask = rejectedMask;
    sharedAtomicStore(&sharedData->restore_multi_turn_result_token, token);
#if MULTITURN_NVS_PERSISTENCE_ENABLE
    if (restoredMask != 0) {
        sharedData->multiturn_nvs_save_reason = MULTITURN_NVS_SAVE_REASON_HOST_RESTORE;
        sharedAtomicAddFetch(&sharedData->multiturn_nvs_save_request_token, (uint32_t)1);
    }
#endif
}

// 标定成功后由唯一拥有舵机驱动的通信任务提交可信状态。只有在线、
// 多圈模式已配置且没有恢复事务悬空的电机才会被接受。
static void processCalibrationTrustCommit(TaskSharedData_t* sharedData,
                                          uint32_t* lastToken)
{
    if (!sharedData || !lastToken) {
        return;
    }
    const uint32_t token =
        sharedAtomicLoad(&sharedData->calibration_trust_request_token);
    if (token == *lastToken) {
        return;
    }
    *lastToken = token;

    const uint32_t requestMask =
        sharedAtomicLoad(&sharedData->calibration_trust_request_mask) &
        validMotorChannelMask();
    uint32_t acceptedMask = 0;
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        const uint32_t bit = 1UL << ch;
        if ((requestMask & bit) == 0) {
            continue;
        }
        FeetechServoDriver* bus = getBusByIndex(motorMap[ch].busIndex);
        if (bus && bus->acceptCurrentSessionAsTrusted(motorMap[ch].servoID)) {
            acceptedMask |= bit;
        }
    }
    if (acceptedMask != 0) {
        sharedAtomicFetchAnd(&sharedData->multi_turn_unknown_bitmap,
                             ~acceptedMask);
    }
    sharedAtomicStore(&sharedData->calibration_trust_accepted_mask,
                      acceptedMask);
    sharedAtomicStore(&sharedData->calibration_trust_result_token, token);
    FW_LOG_INFO_PRINTF(
        "[SERVO CALIB TRUST] token=%lu request=0x%08lX accepted=0x%08lX "
        "unknown=0x%08lX\r\n",
        (unsigned long)token,
        (unsigned long)requestMask,
        (unsigned long)acceptedMask,
        (unsigned long)sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap));
}

#if MULTITURN_NVS_PERSISTENCE_ENABLE
struct MultiTurnNvsRuntimeState {
    bool initialized;
    bool hostWasOnline;
    bool saveQueued;
    bool motionWasActive;
    bool deferredSavePending;
    uint8_t lastMotionState;
    uint32_t lastSaveMs;
    uint32_t lastValidMask;
    uint32_t pendingRequestToken;
    uint32_t pendingQueuedAtMs;
    uint32_t pendingValidMask;
    int32_t lastAbs[SERVO_TOTAL_NUM];
    int32_t pendingAbs[SERVO_TOTAL_NUM];
    int16_t lastRaw[SERVO_TOTAL_NUM];
    int16_t pendingRaw[SERVO_TOTAL_NUM];
    int16_t lastTurn[SERVO_TOTAL_NUM];
    int16_t pendingTurn[SERVO_TOTAL_NUM];
    uint8_t pendingReason;
    uint8_t pendingTurnChanged;
    uint8_t pendingEdgeChanged;
    uint32_t deferredRequestToken;
    uint32_t deferredValidMask;
    int32_t deferredAbs[SERVO_TOTAL_NUM];
    int16_t deferredRaw[SERVO_TOTAL_NUM];
    int16_t deferredTurn[SERVO_TOTAL_NUM];
    uint8_t deferredReason;
    uint8_t deferredTurnChanged;
    uint8_t deferredEdgeChanged;
};

// P4 持续供电而舵机单独掉电时使用的 RAM 恢复锚点。
// 该状态与 Flash 开关独立：Flash OFF 只禁止跨 P4 掉电持久化，
// 不能丢弃本次 P4 上电会话内已经冻结的可信多圈位置。
struct ServoHotplugRecoveryAnchors {
    uint32_t validMask;
    int32_t logicalAbs[SERVO_TOTAL_NUM];
    int16_t raw[SERVO_TOTAL_NUM];
    uint32_t capturedAtMs[SERVO_TOTAL_NUM];
};

enum ServoRecoveryAnchorSource : uint8_t {
    SERVO_RECOVERY_ANCHOR_NONE = 0,
    SERVO_RECOVERY_ANCHOR_RAM_POWER_LOSS = 1,
    SERVO_RECOVERY_ANCHOR_NVS_DEFERRED = 2,
    SERVO_RECOVERY_ANCHOR_NVS_PENDING = 3,
    SERVO_RECOVERY_ANCHOR_NVS_STORED = 4,
};

static bool collectTrustedMultiTurnSnapshot(TaskSharedData_t* sharedData,
                                            int32_t* motorAbs,
                                            int16_t* motorRaw,
                                            int16_t* motorTurn,
                                            uint32_t* validMask)
{
    if (!sharedData || !motorAbs || !motorRaw || !motorTurn || !validMask) {
        return false;
    }

    uint32_t mask = 0;
    const uint32_t allowedMask =
        validMotorChannelMask() & ~sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap);
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        const uint32_t bit = (1UL << ch);
        if ((allowedMask & bit) == 0) {
            continue;
        }
        const uint8_t busIndex = motorMap[ch].busIndex;
        const uint8_t id = motorMap[ch].servoID;
        FeetechServoDriver* bus = getBusByIndex(busIndex);
        if (!bus || !bus->isOnline(id)) {
            continue;
        }
        const int16_t raw = normalizeServoRawSingleTurn(bus->getRawPosition(id));
        const int32_t absPos = bus->getAbsolutePosition(id);
        motorAbs[ch] = absPos;
        motorRaw[ch] = raw;
        motorTurn[ch] = (int16_t)((absPos - raw) / 4096);
        mask |= bit;
    }
    *validMask = mask;
    return mask != 0;
}

static void seedNvsRuntimeStateFromSnapshot(MultiTurnNvsRuntimeState* state,
                                             const MultiTurnNvsSnapshot& snapshot)
{
    if (!state) {
        return;
    }
    memset(state, 0, sizeof(*state));
    state->initialized = true;
    state->hostWasOnline = true;
    state->lastValidMask = snapshot.validMask & validMotorChannelMask();
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        state->lastRaw[ch] = snapshot.raw[ch];
        state->lastTurn[ch] = snapshot.turns[ch];
        state->lastAbs[ch] = multiTurnSnapshotAbsForChannel(snapshot, ch);
    }
}

// 为重新上线的单台舵机选择最新可信锚点。刚冻结的 RAM 锚点优先于
// 异步 NVS 队列，避免舵机恢复速度快于 Flash 提交时出现 anchor=0。
static bool resolveServoRecoveryAnchor(
    TaskSharedData_t* sharedData,
    const MultiTurnNvsRuntimeState* nvsState,
    const ServoHotplugRecoveryAnchors* hotplugAnchors,
    uint8_t channel,
    int32_t* logicalAbs,
    ServoRecoveryAnchorSource* source)
{
    if (!sharedData || !nvsState || !hotplugAnchors ||
        !logicalAbs || !source || channel >= SERVO_TOTAL_NUM) {
        return false;
    }

    const uint32_t bit = 1UL << channel;
    if ((hotplugAnchors->validMask & bit) != 0) {
        *logicalAbs = hotplugAnchors->logicalAbs[channel];
        *source = SERVO_RECOVERY_ANCHOR_RAM_POWER_LOSS;
        return true;
    }

    // Flash 被运行时关闭后，持久化快照已经被清除，不能继续复用任务启动时
    // 留下的 NVS 缓存；但上面的本次会话 RAM 锚点仍然可以安全使用。
    if (sharedData->multiturn_nvs_enabled == 0) {
        return false;
    }
    if ((nvsState->deferredValidMask & bit) != 0) {
        *logicalAbs = nvsState->deferredAbs[channel];
        *source = SERVO_RECOVERY_ANCHOR_NVS_DEFERRED;
        return true;
    }
    if ((nvsState->pendingValidMask & bit) != 0) {
        *logicalAbs = nvsState->pendingAbs[channel];
        *source = SERVO_RECOVERY_ANCHOR_NVS_PENDING;
        return true;
    }
    if (nvsState->initialized &&
        (nvsState->lastValidMask & bit) != 0) {
        *logicalAbs = nvsState->lastAbs[channel];
        *source = SERVO_RECOVERY_ANCHOR_NVS_STORED;
        return true;
    }
    return false;
}

static void processServoOnlineTransitions(
    TaskSharedData_t* sharedData,
    const MultiTurnNvsRuntimeState* nvsState,
    ServoHotplugRecoveryAnchors* hotplugAnchors)
{
    if (!sharedData || !nvsState || !hotplugAnchors) {
        return;
    }
    static uint8_t nextChannel = 0;
    const uint32_t nowMs = millis();
    // 每个 10 ms 周期最多处理一台舵机，避免多台同时上线时连续 EEPROM
    // 事务长时间占用总线并拖慢反馈循环。
    for (uint8_t offset = 0; offset < SERVO_TOTAL_NUM; offset++) {
        const uint8_t ch =
            (uint8_t)((nextChannel + offset) % SERVO_TOTAL_NUM);
        const uint32_t bit = (1UL << ch);
        const uint8_t busIndex = motorMap[ch].busIndex;
        const uint8_t id = motorMap[ch].servoID;
        FeetechServoDriver* bus = getBusByIndex(busIndex);
        if (!bus || !bus->isRecoveryAttemptDue(id, nowMs)) {
            continue;
        }
        nextChannel = (uint8_t)((ch + 1) % SERVO_TOTAL_NUM);

        const bool torqueOffOk = bus->setTorqueEnabled(id, false);
        bool configured = bus->isMultiTurnConfigured(id);
        if (!configured) {
            configured = bus->configureMultiTurnMode(id);
        }

        bool trusted = bus->isAbsoluteReliable(id);
        int32_t recoveryLogicalAbs = 0;
        ServoRecoveryAnchorSource anchorSource =
            SERVO_RECOVERY_ANCHOR_NONE;
        const bool hasAnchor = resolveServoRecoveryAnchor(
            sharedData,
            nvsState,
            hotplugAnchors,
            ch,
            &recoveryLogicalAbs,
            &anchorSource);
        if (!trusted && torqueOffOk && configured && hasAnchor) {
            trusted = bus->restoreAbsolutePosition(id,
                                                   recoveryLogicalAbs,
                                                   kNvsRestoreRawToleranceCounts);
        }

        const uint8_t attemptNumber =
            (uint8_t)(bus->recoveryAttemptCount(id) + 1U);
        const bool recoveryReady = torqueOffOk && configured;
        // 无锚点的新舵机只需完成多圈模式配置并保持 Turn Unknown，后续允许
        // New Device/自动标定重建基准；已有锚点但 raw 校验失败才属于恢复失败，
        // 此时保留 pending 重试，不能把失败事务误当成成功消费。
        const bool recoveryTransactionComplete =
            recoveryReady && (trusted || !hasAnchor);
        bus->finishRecoveryAttempt(id, recoveryTransactionComplete, nowMs);
        if (recoveryReady) {
            if (trusted) {
                sharedAtomicFetchAnd(&sharedData->multi_turn_unknown_bitmap,
                                     ~bit);
                hotplugAnchors->validMask &= ~bit;
            } else {
                // 多圈模式已配置但旧锚点无法证明时保持 Turn Unknown；
                // 自动标定可以在安全预检后从当前 power-session ABS 重建。
                sharedAtomicFetchOr(&sharedData->multi_turn_unknown_bitmap,
                                    bit);
            }
        }
        FW_LOG_INFO_PRINTF("[SERVO HOTPLUG] M%02u bus=%u id=%u attempt=%u/%u "
                      "torque_off=%u configured=%u anchor=%u anchor_src=%u trusted=%u "
                      "ready=%u failed=%u raw=%d unknown=0x%08lX\r\n",
                      (unsigned)ch,
                      (unsigned)busIndex,
                      (unsigned)id,
                      (unsigned)attemptNumber,
                      (unsigned)kServoRecoveryMaxAttempts,
                      torqueOffOk ? 1U : 0U,
                      configured ? 1U : 0U,
                      hasAnchor ? 1U : 0U,
                      (unsigned)anchorSource,
                      trusted ? 1U : 0U,
                      recoveryReady ? 1U : 0U,
                      bus->isRecoveryFailed(id) ? 1U : 0U,
                      (int)bus->getRawPosition(id),
                      (unsigned long)sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap));
        break;
    }
}

static bool snapshotChanged(const MultiTurnNvsRuntimeState& state,
                            uint32_t validMask,
                            const int32_t* motorAbs,
                            const int16_t* motorRaw,
                            const int16_t* motorTurn,
                            bool* turnChanged,
                            bool* edgeChanged)
{
    bool changed = !state.initialized || state.lastValidMask != validMask;
    bool anyTurnChanged = false;
    bool anyEdgeChanged = false;
    const uint32_t compareMask = validMask & state.lastValidMask;

    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        const uint32_t bit = (1UL << ch);
        if ((compareMask & bit) == 0) {
            continue;
        }
        if (state.lastTurn[ch] != motorTurn[ch]) {
            changed = true;
            anyTurnChanged = true;
        }
        if (state.lastRaw[ch] != motorRaw[ch] ||
            state.lastAbs[ch] != motorAbs[ch]) {
            changed = true;
        }
        if (isRawNearTurnEdge(motorRaw[ch]) &&
            abs((int32_t)motorAbs[ch] - (int32_t)state.lastAbs[ch]) >= kNvsMoveAbsThreshold) {
            anyEdgeChanged = true;
        }
    }

    if (turnChanged) {
        *turnChanged = anyTurnChanged;
    }
    if (edgeChanged) {
        *edgeChanged = anyEdgeChanged;
    }
    return changed;
}

static bool snapshotMatchesPending(const MultiTurnNvsRuntimeState& state,
                                   uint32_t validMask,
                                   const int32_t* motorAbs,
                                   const int16_t* motorRaw,
                                   const int16_t* motorTurn)
{
    if (!state.saveQueued || state.pendingValidMask != validMask) {
        return false;
    }
    return memcmp(state.pendingAbs, motorAbs, sizeof(state.pendingAbs)) == 0 &&
        memcmp(state.pendingRaw, motorRaw, sizeof(state.pendingRaw)) == 0 &&
        memcmp(state.pendingTurn, motorTurn, sizeof(state.pendingTurn)) == 0;
}

static bool queueLatestMultiTurnNvsJob(QueueHandle_t queue,
                                       const MultiTurnNvsSaveJob_t* job)
{
    if (!queue || !job) {
        return false;
    }
    MultiTurnNvsSaveJob_t queuedJob;
    if (job->operation != MULTITURN_NVS_OPERATION_RESET_ZERO &&
        xQueuePeek(queue, &queuedJob, 0) == pdTRUE &&
        queuedJob.operation == MULTITURN_NVS_OPERATION_RESET_ZERO) {
        // Reset Zero 必须先把新 turn0 落盘，普通周期快照不得覆盖该事务。
        return false;
    }
    return xQueueOverwrite(queue, job) == pdPASS;
}

static void cacheDeferredMultiTurnSnapshot(MultiTurnNvsRuntimeState* state,
                                           uint32_t validMask,
                                           const int32_t* motorAbs,
                                           const int16_t* motorRaw,
                                           const int16_t* motorTurn,
                                           uint8_t reason,
                                           uint32_t requestToken,
                                           bool turnChanged,
                                           bool edgeChanged)
{
    if (!state || !motorAbs || !motorRaw || !motorTurn || validMask == 0) {
        return;
    }

    // 不能整块覆盖 deferred 快照：一台舵机掉线后，常规在线快照会少
    // 一个 bit；若直接赋值，刚冻结的掉电前位置会被后续循环抹掉。
    // 这里按通道合并，旧通道仅在有新可信样本时才更新。
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        const uint32_t bit = 1UL << ch;
        if ((validMask & bit) == 0) {
            continue;
        }
        state->deferredAbs[ch] = motorAbs[ch];
        state->deferredRaw[ch] = motorRaw[ch];
        state->deferredTurn[ch] = motorTurn[ch];
    }
    state->deferredSavePending = true;
    state->deferredValidMask |= validMask;
    state->deferredTurnChanged =
        (state->deferredTurnChanged || turnChanged) ? 1U : 0U;
    state->deferredEdgeChanged =
        (state->deferredEdgeChanged || edgeChanged) ? 1U : 0U;

    // 显式状态机保存请求拥有最终 reason/token。后台 RAM 快照仍可刷新各通道
    // 数据，但不能覆盖尚未落盘的显式请求；掉电快照原因也需保留到真正写盘。
    const bool preservePowerLossReason =
        state->deferredReason == MULTITURN_NVS_SAVE_REASON_SERVO_POWER_LOSS &&
        reason != MULTITURN_NVS_SAVE_REASON_SERVO_POWER_LOSS &&
        requestToken == 0;
    if (!preservePowerLossReason &&
        (requestToken != 0 || state->deferredRequestToken == 0)) {
        state->deferredReason = reason;
        state->deferredRequestToken = requestToken;
    }
}

// 收集各总线刚刚冻结的掉电前可信位置，并合并进 deferred NVS 快照。
// P4 仍有电时，这份 RAM 快照会在状态机退出运动所有权后由后台任务写入
// Flash；P4 与舵机同时掉电时仍只能依赖此前已经落盘的快照。
static void cacheServoPowerLossSnapshots(
    TaskSharedData_t* sharedData,
    MultiTurnNvsRuntimeState* state,
    ServoHotplugRecoveryAnchors* hotplugAnchors)
{
    if (!sharedData || !state || !hotplugAnchors) {
        return;
    }

    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        const uint8_t busIndex = motorMap[ch].busIndex;
        const uint8_t id = motorMap[ch].servoID;
        FeetechServoDriver* bus = getBusByIndex(busIndex);
        if (!bus) {
            continue;
        }

        int32_t logicalAbs = 0;
        int16_t raw = 0;
        int16_t turn = 0;
        uint32_t sampleTimestampMs = 0;
        if (!bus->consumeOfflineTrustedSnapshot(
                id, &logicalAbs, &raw, &turn, &sampleTimestampMs)) {
            continue;
        }

        const uint32_t bit = 1UL << ch;
        hotplugAnchors->logicalAbs[ch] = logicalAbs;
        hotplugAnchors->raw[ch] = raw;
        hotplugAnchors->capturedAtMs[ch] = sampleTimestampMs;
        hotplugAnchors->validMask |= bit;
        FW_LOG_WARN_PRINTF(
            "[SERVO HOTPLUG ANCHOR] M%02u logical=%ld raw=%d turn=%d ageMs=%lu\r\n",
            (unsigned)ch,
            (long)logicalAbs,
            (int)raw,
            (int)turn,
            (unsigned long)(millis() - sampleTimestampMs));

        if (!sharedData->multiturn_nvs_enabled) {
            FW_LOG_INFO_PRINTF(
                "[NVS MULTITURN] servo power-loss Flash save skipped; RAM anchor kept M%02u abs=%ld raw=%d\r\n",
                (unsigned)ch,
                (long)logicalAbs,
                (int)raw);
            continue;
        }

        int32_t motorAbs[SERVO_TOTAL_NUM] = {0};
        int16_t motorRaw[SERVO_TOTAL_NUM] = {0};
        int16_t motorTurn[SERVO_TOTAL_NUM] = {0};
        motorAbs[ch] = logicalAbs;
        motorRaw[ch] = raw;
        motorTurn[ch] = turn;
        cacheDeferredMultiTurnSnapshot(
            state,
            bit,
            motorAbs,
            motorRaw,
            motorTurn,
            MULTITURN_NVS_SAVE_REASON_SERVO_POWER_LOSS,
            0,
            false,
            isRawNearTurnEdge(raw));
        FW_LOG_WARN_PRINTF(
            "[NVS MULTITURN] servo power-loss frozen M%02u abs=%ld raw=%d turn=%d ageMs=%lu\r\n",
            (unsigned)ch,
            (long)logicalAbs,
            (int)raw,
            (int)turn,
            (unsigned long)(millis() - sampleTimestampMs));
    }
}

static bool enqueueMultiTurnNvsSnapshot(TaskSharedData_t* sharedData,
                                        MultiTurnNvsRuntimeState* state,
                                        uint32_t validMask,
                                        const int32_t* motorAbs,
                                        const int16_t* motorRaw,
                                        const int16_t* motorTurn,
                                        uint8_t reason,
                                        uint32_t requestToken,
                                        bool turnChanged,
                                        bool edgeChanged)
{
    if (!sharedData || !state || !motorAbs || !motorRaw || !motorTurn ||
        validMask == 0 || !sharedData->multiturn_nvs_enabled ||
        !sharedData->multiturnNvsSaveQueue) {
        return false;
    }
    if (snapshotMatchesPending(*state, validMask, motorAbs, motorRaw, motorTurn) &&
        (requestToken == 0 || requestToken == state->pendingRequestToken)) {
        return false;
    }

    MultiTurnNvsSaveJob_t job;
    memset(&job, 0, sizeof(job));
    job.requestToken = requestToken;
    job.validMask = validMask;
    job.queuedAtMs = millis();
    job.reason = reason;
    job.operation = MULTITURN_NVS_OPERATION_SAVE;
    job.turnChanged = turnChanged ? 1U : 0U;
    job.edgeChanged = edgeChanged ? 1U : 0U;
    memcpy(job.motorAbs, motorAbs, sizeof(job.motorAbs));
    memcpy(job.motorRaw, motorRaw, sizeof(job.motorRaw));
    memcpy(job.motorTurn, motorTurn, sizeof(job.motorTurn));

    if (!queueLatestMultiTurnNvsJob(
            sharedData->multiturnNvsSaveQueue, &job)) {
        return false;
    }
    state->saveQueued = true;
    state->pendingRequestToken = requestToken;
    state->pendingQueuedAtMs = job.queuedAtMs;
    state->pendingValidMask = validMask;
    state->pendingReason = reason;
    state->pendingTurnChanged = job.turnChanged;
    state->pendingEdgeChanged = job.edgeChanged;
    memcpy(state->pendingAbs, motorAbs, sizeof(state->pendingAbs));
    memcpy(state->pendingRaw, motorRaw, sizeof(state->pendingRaw));
    memcpy(state->pendingTurn, motorTurn, sizeof(state->pendingTurn));
    return true;
}

static bool resetZeroStateIsSafe(const TaskSharedData_t* sharedData)
{
    if (!sharedData || sharedData->control_enabled != 0 ||
        sharedData->servo_target_owner != SERVO_TARGET_OWNER_NONE) {
        return false;
    }
    const uint8_t state = sharedData->system_state;
    return state == SYSTEM_STATE_UNCONTROLLABLE ||
           state == SYSTEM_STATE_IDLE ||
           state == SYSTEM_STATE_READY ||
           state == SYSTEM_STATE_RECOVERY_REQUIRED;
}

// 执行单台或少量越界舵机的 Reset Zero。硬件复位成功后先保持力矩关闭，
// 再把新的 raw/turn0 交给独立 NVS 任务；Flash 完成前不发布最终成功。
static void processServoResetZeroRequest(TaskSharedData_t* sharedData,
                                         uint32_t* lastRequestToken)
{
    if (!sharedData || !lastRequestToken) {
        return;
    }
    const uint32_t token =
        sharedAtomicLoad(&sharedData->servo_reset_zero_request_token);
    if (token == *lastRequestToken) {
        return;
    }
    *lastRequestToken = token;

    const uint32_t requestedMask =
        sharedAtomicLoad(&sharedData->servo_reset_zero_request_mask) &
        validMotorChannelMask();
    const uint32_t overflowMask =
        sharedAtomicLoad(&sharedData->servo_turn_overflow_bitmap);
    const uint32_t onlineMask =
        p4::servo::collectOnlineMotorMask();
    const uint32_t executableMask =
        requestedMask & overflowMask & onlineMask;

    sharedAtomicStore(
        &sharedData->servo_reset_zero_success_mask, (uint32_t)0);
    sharedAtomicStore(
        &sharedData->servo_reset_zero_failed_mask,
        requestedMask & ~executableMask);
    sharedAtomicStore(
        &sharedData->servo_reset_zero_status,
        (uint8_t)SERVO_RESET_ZERO_STATUS_RUNNING);

    if (token == 0 || requestedMask == 0 || executableMask == 0 ||
        !resetZeroStateIsSafe(sharedData) ||
        !sharedData->multiturn_nvs_enabled ||
        !sharedData->multiturnNvsSaveQueue) {
        sharedAtomicFetchOr(
            &sharedData->servo_reset_zero_failed_mask, requestedMask);
        sharedAtomicStore(
            &sharedData->servo_reset_zero_status,
            (uint8_t)SERVO_RESET_ZERO_STATUS_FAILED);
        sharedAtomicStore(
            &sharedData->servo_reset_zero_result_token, token);
        return;
    }

    MultiTurnNvsSaveJob_t job;
    memset(&job, 0, sizeof(job));
    job.operation = MULTITURN_NVS_OPERATION_RESET_ZERO;
    job.reason = MULTITURN_NVS_SAVE_REASON_SERVO_RESET_ZERO;
    job.requestToken = token;
    job.queuedAtMs = millis();

    uint32_t hardwareResetMask = 0;
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        const uint32_t bit = 1UL << ch;
        if ((executableMask & bit) == 0) {
            continue;
        }
        FeetechServoDriver* bus =
            getBusByIndex(motorMap[ch].busIndex);
        int16_t raw = 0;
        if (!bus ||
            !bus->resetTurnCounter(motorMap[ch].servoID, &raw)) {
            sharedAtomicFetchOr(
                &sharedData->servo_reset_zero_failed_mask, bit);
            continue;
        }
        hardwareResetMask |= bit;
        job.motorRaw[ch] = raw;
        job.motorTurn[ch] = 0;
        job.motorAbs[ch] = raw;
        FW_LOG_WARN_PRINTF(
            "[SERVO RESET ZERO] hardware reset M%02u raw=%d token=%lu\r\n",
            (unsigned)ch,
            (int)raw,
            (unsigned long)token);
    }

    if (hardwareResetMask == 0) {
        sharedAtomicStore(
            &sharedData->servo_reset_zero_status,
            (uint8_t)SERVO_RESET_ZERO_STATUS_FAILED);
        sharedAtomicStore(
            &sharedData->servo_reset_zero_result_token, token);
        return;
    }

    job.validMask = hardwareResetMask;
    // 即使随后 Flash 写入失败，硬件坐标已经改变，也必须让上位机/P4
    // 失效对应关节标定；failed mask 用于额外表示持久化未闭环。
    sharedAtomicStore(
        &sharedData->servo_reset_zero_success_mask, hardwareResetMask);
    sharedAtomicFetchAnd(
        &sharedData->multi_turn_unknown_bitmap, ~hardwareResetMask);
    sharedAtomicFetchAnd(
        &sharedData->servo_turn_overflow_bitmap, ~hardwareResetMask);

    if (!queueLatestMultiTurnNvsJob(
            sharedData->multiturnNvsSaveQueue, &job)) {
        sharedAtomicFetchOr(
            &sharedData->servo_reset_zero_failed_mask, hardwareResetMask);
        sharedAtomicFetchOr(
            &sharedData->multi_turn_unknown_bitmap, hardwareResetMask);
        sharedAtomicStore(
            &sharedData->servo_reset_zero_status,
            (uint8_t)SERVO_RESET_ZERO_STATUS_FAILED);
        sharedAtomicStore(
            &sharedData->servo_reset_zero_result_token, token);
    }
}

static bool flushDeferredMultiTurnSnapshot(TaskSharedData_t* sharedData,
                                           MultiTurnNvsRuntimeState* state)
{
    if (!sharedData || !state || !state->deferredSavePending) {
        return false;
    }

    const bool alreadyPending =
        snapshotMatchesPending(*state,
                               state->deferredValidMask,
                               state->deferredAbs,
                               state->deferredRaw,
                               state->deferredTurn) &&
        (state->deferredRequestToken == 0 ||
         state->deferredRequestToken == state->pendingRequestToken);
    const bool accepted = alreadyPending ||
        enqueueMultiTurnNvsSnapshot(sharedData,
                                    state,
                                    state->deferredValidMask,
                                    state->deferredAbs,
                                    state->deferredRaw,
                                    state->deferredTurn,
                                    state->deferredReason,
                                    state->deferredRequestToken,
                                    state->deferredTurnChanged != 0,
                                    state->deferredEdgeChanged != 0);
    if (!accepted) {
        return false;
    }

    state->deferredSavePending = false;
    state->deferredRequestToken = 0;
    state->deferredValidMask = 0;
    state->deferredReason = 0;
    state->deferredTurnChanged = 0;
    state->deferredEdgeChanged = 0;
    return true;
}

static void applyCompletedNvsSave(MultiTurnNvsRuntimeState* state,
                                  const MultiTurnNvsSaveResult_t& result)
{
    if (!state || !result.success) {
        return;
    }
    state->initialized = true;
    state->lastSaveMs = result.completedAtMs;
    state->lastValidMask = result.validMask;
    memcpy(state->lastAbs, result.motorAbs, sizeof(state->lastAbs));
    memcpy(state->lastRaw, result.motorRaw, sizeof(state->lastRaw));
    memcpy(state->lastTurn, result.motorTurn, sizeof(state->lastTurn));
}

static void processMultiTurnNvsSaveResults(TaskSharedData_t* sharedData,
                                           MultiTurnNvsRuntimeState* state)
{
    if (!sharedData || !state || !sharedData->multiturnNvsResultQueue) {
        return;
    }

    MultiTurnNvsSaveResult_t result;
    while (xQueueReceive(sharedData->multiturnNvsResultQueue, &result, 0) == pdTRUE) {
        if (state->saveQueued && result.queuedAtMs == state->pendingQueuedAtMs) {
            state->saveQueued = false;
        }
        if (result.operation == MULTITURN_NVS_OPERATION_RESET_ZERO &&
            state->saveQueued) {
            MultiTurnNvsSaveJob_t queuedJob;
            const bool trackedOrdinarySaveStillQueued =
                xQueuePeek(
                    sharedData->multiturnNvsSaveQueue,
                    &queuedJob,
                    0) == pdTRUE &&
                queuedJob.operation == MULTITURN_NVS_OPERATION_SAVE &&
                queuedJob.queuedAtMs == state->pendingQueuedAtMs;
            if (!trackedOrdinarySaveStillQueued) {
                // Reset Zero 可能覆盖队列中尚未执行的普通快照。若不清除此标志，
                // 后续普通保存会一直误认为旧请求仍在队列中。
                state->saveQueued = false;
            }
        }

        if (result.success) {
            applyCompletedNvsSave(state, result);
            FW_LOG_INFO_PRINTF("[NVS MULTITURN] saved seq=%lu reason=%u mask=0x%08lX turnChanged=%u edge=%u\r\n",
                          (unsigned long)result.sequence,
                          (unsigned)result.reason,
                          (unsigned long)result.validMask,
                          result.turnChanged ? 1U : 0U,
                          result.edgeChanged ? 1U : 0U);
        } else {
            FW_LOG_INFO_PRINTF("[NVS MULTITURN] save failed reason=%u mask=0x%08lX fail=%lu\r\n",
                          (unsigned)result.reason,
                          (unsigned long)result.validMask,
                          (unsigned long)sharedData->multiturn_nvs_save_fail_count);
        }
    }
}

static bool saveCurrentMultiTurnSnapshot(TaskSharedData_t* sharedData,
                                         MultiTurnNvsRuntimeState* state,
                                         uint8_t reason,
                                         bool force,
                                         uint32_t requestToken)
{
    if (!sharedData || !state) {
        return false;
    }
    if (!sharedData->multiturn_nvs_enabled || !sharedData->multiturnNvsSaveQueue) {
        return false;
    }

    int32_t motorAbs[SERVO_TOTAL_NUM] = {0};
    int16_t motorRaw[SERVO_TOTAL_NUM] = {0};
    int16_t motorTurn[SERVO_TOTAL_NUM] = {0};
    uint32_t validMask = 0;
    if (!collectTrustedMultiTurnSnapshot(sharedData, motorAbs, motorRaw, motorTurn, &validMask)) {
        return false;
    }

    bool turnChanged = false;
    bool edgeChanged = false;
    const bool changed = snapshotChanged(*state,
                                         validMask,
                                         motorAbs,
                                         motorRaw,
                                         motorTurn,
                                         &turnChanged,
                                         &edgeChanged);
    const uint32_t nowMs = millis();
    if (!force && !changed) {
        return false;
    }
    if (!force && state->initialized &&
        (uint32_t)(nowMs - state->lastSaveMs) < kNvsSaveMinIntervalMs) {
        return false;
    }

    return enqueueMultiTurnNvsSnapshot(sharedData,
                                       state,
                                       validMask,
                                       motorAbs,
                                       motorRaw,
                                       motorTurn,
                                       reason,
                                       requestToken,
                                       turnChanged,
                                       edgeChanged);
}

static void processMultiTurnNvsSave(TaskSharedData_t* sharedData,
                                    MultiTurnNvsRuntimeState* state,
                                    uint32_t* lastSaveToken)
{
    if (!sharedData || !state || !lastSaveToken) {
        return;
    }

    processMultiTurnNvsSaveResults(sharedData, state);

    const uint32_t nowMs = millis();
    const uint32_t hostHeartbeatMs = sharedData->host_rx_heartbeat_ms;
    const bool hostOnline = hostHeartbeatMs != 0 &&
        (uint32_t)(nowMs - hostHeartbeatMs) <= kHostHeartbeatOfflineMs;
    const bool hostJustWentOffline = state->hostWasOnline && !hostOnline;
    state->hostWasOnline = hostOnline;

    const uint32_t token =
        sharedAtomicLoad(&sharedData->multiturn_nvs_save_request_token);
    const bool tokenChanged = token != *lastSaveToken;
    uint8_t requestedReason = 0;
    if (tokenChanged) {
        *lastSaveToken = token;
        requestedReason = sharedData->multiturn_nvs_save_reason;
    }

    int32_t motorAbs[SERVO_TOTAL_NUM] = {0};
    int16_t motorRaw[SERVO_TOTAL_NUM] = {0};
    int16_t motorTurn[SERVO_TOTAL_NUM] = {0};
    uint32_t validMask = 0;
    const bool hasSnapshot =
        collectTrustedMultiTurnSnapshot(sharedData,
                                        motorAbs,
                                        motorRaw,
                                        motorTurn,
                                        &validMask);

    bool turnChanged = false;
    bool edgeChanged = false;
    const bool changed = hasSnapshot &&
        snapshotChanged(*state,
                        validMask,
                        motorAbs,
                        motorRaw,
                        motorTurn,
                        &turnChanged,
                        &edgeChanged);

    const uint8_t systemState = sharedData->system_state;
    const bool motionActive = isMultiTurnMotionState(systemState);
    const bool motionJustEnded = state->motionWasActive && !motionActive;
    if (motionActive) {
        state->motionWasActive = true;
        state->lastMotionState = systemState;

        // During control/calibration motion only refresh the latest trusted RAM
        // snapshot. Flash/NVS is deliberately deferred until motion ownership ends.
        if (changed) {
            cacheDeferredMultiTurnSnapshot(state,
                                           validMask,
                                           motorAbs,
                                           motorRaw,
                                           motorTurn,
                                           completedMotionSaveReason(systemState),
                                           0,
                                           turnChanged,
                                           edgeChanged);
        }
        if (tokenChanged) {
            if (hasSnapshot) {
                cacheDeferredMultiTurnSnapshot(state,
                                               validMask,
                                               motorAbs,
                                               motorRaw,
                                               motorTurn,
                                               requestedReason,
                                               token,
                                               turnChanged,
                                               edgeChanged);
            } else if (state->deferredSavePending) {
                state->deferredReason = requestedReason;
                state->deferredRequestToken = token;
            }
        }
        if (hostJustWentOffline && !tokenChanged) {
            if (hasSnapshot) {
                cacheDeferredMultiTurnSnapshot(state,
                                               validMask,
                                               motorAbs,
                                               motorRaw,
                                               motorTurn,
                                               MULTITURN_NVS_SAVE_REASON_HOST_OFFLINE,
                                               0,
                                               turnChanged,
                                               edgeChanged);
            } else if (state->deferredSavePending &&
                       state->deferredRequestToken == 0) {
                state->deferredReason = MULTITURN_NVS_SAVE_REASON_HOST_OFFLINE;
            }
        }
        return;
    }

    const uint8_t completedState = state->lastMotionState;
    state->motionWasActive = false;

    if (motionJustEnded && changed) {
        cacheDeferredMultiTurnSnapshot(state,
                                       validMask,
                                       motorAbs,
                                       motorRaw,
                                       motorTurn,
                                       completedMotionSaveReason(completedState),
                                       0,
                                       turnChanged,
                                       edgeChanged);
    }
    if (tokenChanged) {
        if (hasSnapshot) {
            cacheDeferredMultiTurnSnapshot(state,
                                           validMask,
                                           motorAbs,
                                           motorRaw,
                                           motorTurn,
                                           requestedReason,
                                           token,
                                           turnChanged,
                                           edgeChanged);
        } else if (state->deferredSavePending) {
            state->deferredReason = requestedReason;
            state->deferredRequestToken = token;
        }
    }
    if (hostJustWentOffline && !tokenChanged) {
        if (hasSnapshot) {
            cacheDeferredMultiTurnSnapshot(state,
                                           validMask,
                                           motorAbs,
                                           motorRaw,
                                           motorTurn,
                                           MULTITURN_NVS_SAVE_REASON_HOST_OFFLINE,
                                           0,
                                           turnChanged,
                                           edgeChanged);
        } else if (state->deferredSavePending &&
                   state->deferredRequestToken == 0) {
            state->deferredReason = MULTITURN_NVS_SAVE_REASON_HOST_OFFLINE;
        }
    }

    if (state->deferredSavePending) {
        flushDeferredMultiTurnSnapshot(sharedData, state);
        return;
    }

    if (hostOnline) {
        return;
    }
    if (!changed) {
        return;
    }

    const uint32_t elapsed = state->initialized
        ? (uint32_t)(nowMs - state->lastSaveMs)
        : 0xFFFFFFFFUL;
    if ((turnChanged || edgeChanged) && elapsed >= kNvsSaveMinIntervalMs) {
        saveCurrentMultiTurnSnapshot(sharedData,
                                     state,
                                     MULTITURN_NVS_SAVE_REASON_OFFLINE_TURN_EDGE,
                                     true,
                                     0);
    } else if (!turnChanged && !edgeChanged &&
               elapsed >= kNvsOfflinePeriodicSaveMs) {
        saveCurrentMultiTurnSnapshot(sharedData,
                                     state,
                                     MULTITURN_NVS_SAVE_REASON_OFFLINE_MOVE,
                                     true,
                                     0);
    }
}

static void processBootNvsRestore(TaskSharedData_t* sharedData,
                                  const MultiTurnNvsSnapshot* snapshot,
                                  bool* restoreDone,
                                  uint32_t restoreDeadlineMs)
{
    if (!sharedData || !snapshot || !restoreDone || *restoreDone) {
        return;
    }

    const uint32_t requestMask = snapshot->validMask & validMotorChannelMask();
    if (requestMask == 0) {
        *restoreDone = true;
        return;
    }

    const uint32_t onlineMask =
        p4::servo::collectOnlineMotorMask();
    const uint32_t nowMs = millis();
    if ((onlineMask & requestMask) != requestMask &&
        (int32_t)(nowMs - restoreDeadlineMs) <= 0) {
        return;
    }

    int32_t motorAbs[SERVO_TOTAL_NUM] = {0};
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        motorAbs[ch] = multiTurnSnapshotAbsForChannel(*snapshot, ch);
    }

    uint32_t restoredMask = 0;
    uint32_t rejectedMask = 0;
    restoreServoMultiTurnFromHost(sharedData,
                                  requestMask & onlineMask,
                                  motorAbs,
                                  kNvsRestoreRawToleranceCounts,
                                  false,
                                  &restoredMask,
                                  &rejectedMask);
    FW_LOG_INFO_PRINTF("[NVS MULTITURN] boot restore seq=%lu request=0x%08lX online=0x%08lX restored=0x%08lX rejected=0x%08lX\r\n",
                  (unsigned long)snapshot->sequence,
                  (unsigned long)requestMask,
                  (unsigned long)onlineMask,
                  (unsigned long)restoredMask,
                  (unsigned long)rejectedMask);
    *restoreDone = true;
}
#endif

void servoCommunicationTask(void* parameter)
{
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    if (!sharedData) {
        vTaskDelete(NULL);
        return;
    }

    beginServoBuses();
    configureMappedServosForMultiTurn();

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t taskPeriodTicks = pdMS_TO_TICKS(10);
    uint8_t readBusPhase = 0;
    uint32_t lastEmergencyStopToken =
        sharedAtomicLoad(&sharedData->servo_emergency_stop_token);
    uint32_t lastRestoreToken =
        sharedAtomicLoad(&sharedData->restore_multi_turn_request_token);
    uint32_t lastCalibrationTrustToken =
        sharedAtomicLoad(&sharedData->calibration_trust_request_token);
#if MULTITURN_NVS_PERSISTENCE_ENABLE
    uint32_t lastServoResetZeroToken =
        sharedAtomicLoad(&sharedData->servo_reset_zero_request_token);
    uint32_t lastNvsSaveToken =
        sharedAtomicLoad(&sharedData->multiturn_nvs_save_request_token);
#endif
#if MULTITURN_NVS_PERSISTENCE_ENABLE
    const uint32_t nvsRestoreDeadlineMs = millis() + kNvsRestoreWaitWindowMs;
    bool nvsEnabled = true;
    if (!multiTurnNvsLoadEnabled(&nvsEnabled)) {
        FW_LOG_INFO_PRINTLN("[NVS MULTITURN] enabled flag load failed, default enabled");
        nvsEnabled = true;
    }
    sharedData->multiturn_nvs_enabled = nvsEnabled ? 1 : 0;
    MultiTurnNvsSnapshot bootNvsSnapshot;
    const bool hasBootNvsSnapshot = nvsEnabled && multiTurnNvsLoadLatest(&bootNvsSnapshot);
    sharedData->multiturn_nvs_snapshot_valid = hasBootNvsSnapshot ? 1 : 0;
    sharedData->multiturn_nvs_save_valid_mask =
        hasBootNvsSnapshot
            ? (bootNvsSnapshot.validMask & validMotorChannelMask())
            : 0;
    if (hasBootNvsSnapshot) {
        // 启动恢复与 UI 必须引用同一份已通过 CRC 校验的 Flash 快照。
        memcpy(sharedData->multiturn_nvs_snapshot_raw,
               bootNvsSnapshot.raw,
               sizeof(sharedData->multiturn_nvs_snapshot_raw));
    } else {
        memset(sharedData->multiturn_nvs_snapshot_raw,
               0,
               sizeof(sharedData->multiturn_nvs_snapshot_raw));
    }
    bool bootNvsRestoreDone = !hasBootNvsSnapshot;
    MultiTurnNvsRuntimeState nvsSaveState;
    memset(&nvsSaveState, 0, sizeof(nvsSaveState));
    nvsSaveState.hostWasOnline = true;
    ServoHotplugRecoveryAnchors hotplugRecoveryAnchors;
    memset(&hotplugRecoveryAnchors, 0, sizeof(hotplugRecoveryAnchors));
    if (!nvsEnabled) {
        FW_LOG_INFO_PRINTLN("[NVS MULTITURN] disabled by user, boot restore skipped");
    } else if (hasBootNvsSnapshot) {
        seedNvsRuntimeStateFromSnapshot(&nvsSaveState, bootNvsSnapshot);
        FW_LOG_INFO_PRINTF("[NVS MULTITURN] loaded seq=%lu mask=0x%08lX\r\n",
                      (unsigned long)bootNvsSnapshot.sequence,
                      (unsigned long)(bootNvsSnapshot.validMask & validMotorChannelMask()));
    } else {
        FW_LOG_INFO_PRINTLN("[NVS MULTITURN] no valid boot snapshot");
    }
#else
    FW_LOG_INFO_PRINTLN("[NVS MULTITURN] persistence disabled");
#endif

    uint32_t lastLoopStartMs = 0;
    uint32_t queueDropTotal = 0;
    uint32_t lastQueueDropLogMs = 0;

    for (;;)
    {
        const uint32_t loopStartMs = millis();
        if (lastLoopStartMs != 0) {
            const uint32_t loopGapMs = loopStartMs - lastLoopStartMs;
            if (loopGapMs > kServoLoopGapLogThresholdMs) {
                const UBaseType_t queueDepth =
                    p4::actuator::pendingCount(sharedData);
                FW_LOG_WARN_PRINTF("[SERVO LOOP GAP] gapMs=%lu state=%u owner=%u queueDepth=%u activeMotor=0x%08lX\r\n",
                                   (unsigned long)loopGapMs,
                                   (unsigned)sharedData->system_state,
                                   (unsigned)sharedData->servo_target_owner,
                                   (unsigned)queueDepth,
                                   (unsigned long)sharedAtomicLoad(&sharedData->active_motor_mask));
            }
        }
        lastLoopStartMs = loopStartMs;
        taskRuntimeMarkLoopStart(TASK_RUNTIME_SERVO_COMM);
        const uint32_t emergencyStopToken =
            sharedAtomicLoad(&sharedData->servo_emergency_stop_token);
        if (emergencyStopToken != lastEmergencyStopToken) {
            lastEmergencyStopToken = emergencyStopToken;
            emergencyTorqueOffMappedServos(sharedData);
            taskRuntimeMarkStopTorqueOffComplete();
        }

        processHostMultiTurnRestore(sharedData, &lastRestoreToken);
        processCalibrationTrustCommit(sharedData, &lastCalibrationTrustToken);
        const bool calibrationMotionActive =
            sharedData->system_state == SYSTEM_STATE_CALIBRATION_RUNNING &&
            sharedData->servo_target_owner == SERVO_TARGET_OWNER_CALIBRATION &&
            sharedAtomicLoad(&sharedData->calibration_motion_started) != 0;
        ServoTargetBatch_t batch;
        while (p4::actuator::receiveLatest(sharedData, &batch)) {
            const uint8_t owner = sharedData->servo_target_owner;
            if (owner != SERVO_TARGET_OWNER_NONE && batch.source == owner) {
                const uint32_t applyBeginMs = millis();
                applyServoTargetBatch(sharedData, batch);
                const uint32_t applyElapsedMs = millis() - applyBeginMs;
                if (applyElapsedMs > kServoApplyStallLogThresholdMs) {
                    FW_LOG_WARN_PRINTF("[SERVO LOOP STALL] stage=apply_target elapsedMs=%lu source=%u owner=%u ageMs=%lu count=%u\r\n",
                                       (unsigned long)applyElapsedMs,
                                       (unsigned)batch.source,
                                       (unsigned)owner,
                                       (unsigned long)(millis() - batch.timestamp),
                                       (unsigned)batch.count);
                }
            } else {
                queueDropTotal++;
                const uint32_t nowMs = millis();
                if ((uint32_t)(nowMs - lastQueueDropLogMs) >=
                    kServoQueueDropLogIntervalMs) {
                    lastQueueDropLogMs = nowMs;
                    FW_LOG_WARN_PRINTF("[SERVO QUEUE DROP] source=%u owner=%u ageMs=%lu count=%u droppedTotal=%lu state=%u\r\n",
                                       (unsigned)batch.source,
                                       (unsigned)owner,
                                       (unsigned long)(nowMs - batch.timestamp),
                                       (unsigned)batch.count,
                                       (unsigned long)queueDropTotal,
                                       (unsigned)sharedData->system_state);
                }
            }
        }

        uint8_t readIds[NUM_BUSES][MAX_SERVOS_PER_BUS] = {0};
        uint8_t readCounts[NUM_BUSES] = {0};
        const uint32_t calibrationReadMask = calibrationMotionActive
            ? (sharedAtomicLoad(&sharedData->active_motor_mask) |
               sharedAtomicLoad(&sharedData->calibration_aux_motor_mask)) &
                  validMotorChannelMask()
            : 0;
        for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
            if (calibrationMotionActive &&
                calibrationReadMask != 0 &&
                (calibrationReadMask & (1UL << ch)) == 0) {
                continue;
            }
            addReadTarget(readIds, readCounts, motorMap[ch].busIndex, motorMap[ch].servoID);
        }

        const uint8_t readStartBus = (readBusPhase == 0) ? 0 : 2;
        for (uint8_t offset = 0; offset < 2; offset++) {
            const uint8_t busIndex = (uint8_t)(readStartBus + offset);
            if (busIndex >= NUM_BUSES || readCounts[busIndex] == 0) {
                continue;
            }
            const uint32_t readBeginMs = millis();
            FeetechServoDriver* bus = getBusByIndex(busIndex);
            const int readSuccess =
                bus ? bus->syncReadPositions(readIds[busIndex], readCounts[busIndex]) : 0;
            const uint32_t readElapsedMs = millis() - readBeginMs;
            if (readElapsedMs > 50) {
                FW_LOG_WARN_PRINTF("[SERVO LOOP STALL] stage=sync_read bus=%u elapsedMs=%lu requested=%u success=%d calibration=%u activeMask=0x%08lX\r\n",
                                   (unsigned)busIndex,
                                   (unsigned long)readElapsedMs,
                                   (unsigned)readCounts[busIndex],
                                   readSuccess,
                                   calibrationMotionActive ? 1U : 0U,
                                   (unsigned long)calibrationReadMask);
            }
        }
        readBusPhase ^= 1;
        // Keep regular synchronised feedback active. Defer extra single-servo
        // probes and hotplug recovery transactions while calibration owns the
        // targets, so they cannot interrupt a dual-tendon move on the bus.
        if (!calibrationMotionActive) {
            servoBusRegistry().probeOneOfflineMappedServo(millis());
        }

#if MULTITURN_NVS_PERSISTENCE_ENABLE
        // 掉线检测会在 sync read 中冻结最后可信反馈。必须先把它搬入独立
        // RAM 锚点，再处理重新上线事务，避免快速断电重启时恢复抢在保存前面。
        cacheServoPowerLossSnapshots(
            sharedData, &nvsSaveState, &hotplugRecoveryAnchors);
        // 先吸收异步 Flash 写入结果，使恢复逻辑能看到最新已提交快照。
        processMultiTurnNvsSaveResults(sharedData, &nvsSaveState);
        processBootNvsRestore(sharedData,
                              hasBootNvsSnapshot ? &bootNvsSnapshot : NULL,
                              &bootNvsRestoreDone,
                              nvsRestoreDeadlineMs);
        if (!calibrationMotionActive) {
            processServoOnlineTransitions(
                sharedData, &nvsSaveState, &hotplugRecoveryAnchors);
        }
#endif
        p4::servo::publishTurnOverflowState(sharedData);
#if MULTITURN_NVS_PERSISTENCE_ENABLE
        processServoResetZeroRequest(
            sharedData, &lastServoResetZeroToken);
#endif
        p4::servo::publishRecoveryState(sharedData);
        p4::servo::publishFeedback(sharedData);
#if MULTITURN_NVS_PERSISTENCE_ENABLE
        processMultiTurnNvsSave(sharedData, &nvsSaveState, &lastNvsSaveToken);
#endif
        taskRuntimeMarkLoopEnd(TASK_RUNTIME_SERVO_COMM);
        const TickType_t nextWakeTime = lastWakeTime + taskPeriodTicks;
        const bool periodOverrun = (int32_t)(xTaskGetTickCount() - nextWakeTime) >= 0;
        taskRuntimeMarkPeriodResult(TASK_RUNTIME_SERVO_COMM, periodOverrun);
        vTaskDelayUntil(&lastWakeTime, taskPeriodTicks);
        if (periodOverrun) {
            // Offline bus scans can overrun the 10 ms period. Force one blocked
            // tick so lower-priority status/desktop communication still runs.
            vTaskDelay(1);
        }
    }
}
