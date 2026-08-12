/**
 * @file ServoBusService.cpp
 * @brief 调度四条舵机总线、应用目标批次并发布统一反馈与恢复状态。
 *
 * 本任务不再直接调用飞特 SDK。底层串口协议由 FeetechServoDriver 封装；
 * 这里负责“何时收发”和跨任务状态同步，后续还会继续拆出恢复管理器。
 */
#include "ServoBusService.h"

#include <string.h>

#include "../../safety/policies/MultiTurnSafetyPolicy.h"
#include "../../safety/policies/PersistenceSafetyPolicy.h"
#include "ServoDiscoveryManager.h"
#include "ServoFeedbackPublisher.h"
#include "ServoRouteTable.h"
#include "ServoBusRouting.h"
#include "../../modules/actuator/ActuatorCommandGateway.h"
#include "../../modules/persistence/MultiTurnNvsStore.h"
#include "../../modules/runtime/ServiceCyclePolicy.h"
#include "../../config/topology/HardwareMap.h"
#include "../../config/DiagnosticsConfig.h"
#include "../../config/SafetyConfig.h"
#include "../../config/ServoConfig.h"
#include "../../platform/rtos/SharedSafetyState.h"
#include "../../platform/time/Clock.h"
#include "../../platform/log/FirmwareLog.h"
#include "../../runtime/SystemRuntimeContext.h"
#include "../../runtime/ports/SystemEventPort.h"
#include "../../runtime/metrics/TaskRuntimeMetrics.h"
#include "../../config/safety/ServoPositionRange.h"
#if MULTITURN_NVS_PERSISTENCE_ENABLE
#include "../../modules/persistence/MultiTurnNvsStore.h"
#endif


#include "../../hal/servo/FeetechServoDriver.h"
#include "../../hal/servo/ServoBusRegistry.h"
#include "../../safety/policies/ActuatorSafetyPolicy.h"
#include "../../safety/SafetyEventReporter.h"


using namespace mmhand::config::diagnostics;
using namespace mmhand::config::safety;
using namespace mmhand::config::servo;

using p4::recovery::completedMotionSaveReason;
using p4::recovery::isMultiTurnMotionState;
using p4::recovery::isRawNearTurnEdge;

/** 仅在统一事件缓存接受新事件时输出一次现场详情，避免重复维护日志节流状态。 */
static void logServoSafetyReject(SystemRuntimeContext* sharedData,
                                 const ServoSafetyDecision& decision)
{
    if (!sharedData || decision.motorChannel >= SERVO_TOTAL_NUM) {
        return;
    }
    if (decision.reason == ServoRejectReason::LogicalRange ||
        decision.reason == ServoRejectReason::HardwareRange) {
        ServoRoute route;
        FeetechServoDriver* bus =
            resolveMotorDriver(decision.motorChannel, &route, false);
        const int32_t logicalOffset =
            bus ? bus->getLogicalOffset(route.servoId) : 0;
        const ServoLogicalRange range =
            servoLogicalRangeFromOffset(logicalOffset);
        int64_t hardwareTarget =
            (int64_t)decision.requestedAbs - (int64_t)logicalOffset;
        if (hardwareTarget > INT32_MAX) hardwareTarget = INT32_MAX;
        if (hardwareTarget < INT32_MIN) hardwareTarget = INT32_MIN;
        FW_LOG_WARN_PRINTF(
            "[SERVO SAFETY REJECT] M%02u reason=%s logical=%ld hardware=%ld "
            "offset=%ld logicalRange=[%ld,%ld] hardwareRange=[%ld,%ld] "
            "owner=%u state=%u\r\n",
            (unsigned)decision.motorChannel,
            p4::actuator::rejectReasonName(decision.reason),
            (long)decision.requestedAbs,
            (long)hardwareTarget,
            (long)logicalOffset,
            (long)range.minAbs,
            (long)range.maxAbs,
            (long)kServoHardwareCommandMin,
            (long)kServoHardwareCommandMax,
            (unsigned)sharedData->servo_target_owner,
            (unsigned)sharedData->system_state);
        return;
    }
    FW_LOG_WARN_PRINTF(
        "[SERVO SAFETY REJECT] M%02u reason=%s requested=%ld hardware=%ld "
        "owner=%u state=%u\r\n",
        (unsigned)decision.motorChannel,
        p4::actuator::rejectReasonName(decision.reason),
        (long)decision.requestedAbs,
        (long)decision.hardwareAbs,
        (unsigned)sharedData->servo_target_owner,
        (unsigned)sharedData->system_state);
}

/**
 * 应用一个目标批次，并在真正进入飞特驱动前执行统一最终安全裁决。
 * 返回实际进入任一总线同步写缓存的命令数量；0 表示整批被拒绝或为空。
 */
static uint8_t applyServoTargetBatch(SystemRuntimeContext* sharedData,
                                     const ServoTargetBatch_t& batch,
                                     uint32_t nowMs)
{
    if (!sharedData) {
        return 0;
    }
    // 队列属于跨任务边界：即使发布入口已经校验，也必须在消费端再次验证批次结构，
    // 避免损坏数据在检查 busIndex/servoId 前被用于访问驱动数组。
    const uint8_t systemState = sharedAtomicLoad(&sharedData->system_state);
    const uint8_t activeOwner =
        sharedAtomicLoad(&sharedData->servo_target_owner);
    const ServoSafetyDecision batchDecision =
        p4::actuator::evaluateBatchStructure(batch);
    if (!batchDecision.allowed) {
        if (p4::safety::reportServoSafetyDecision(batchDecision,
                                                  systemState,
                                                  activeOwner)) {
            FW_LOG_WARN_PRINTF(
                "[SERVO SAFETY REJECT] batch reason=%s command_index=%u\r\n",
                p4::actuator::rejectReasonName(batchDecision.reason),
                (unsigned)batchDecision.commandIndex);
        }
        return 0;
    }
    bool busWritePending[NUM_BUSES] = {false};
    int32_t logicalMin[SERVO_TOTAL_NUM] = {0};
    int32_t logicalMax[SERVO_TOTAL_NUM] = {0};
    p4::actuator::ServoSafetyContext safetyContext;
    safetyContext.activeOwner =
        sharedAtomicLoad(&sharedData->servo_target_owner);
    safetyContext.systemState =
        sharedAtomicLoad(&sharedData->system_state);
    safetyContext.emergencyStopLatched =
        sharedAtomicLoad(&sharedData->emergency_stop_latched) != 0;
    safetyContext.nowMs = nowMs;
    safetyContext.maxTargetAgeMs = kServoTargetMaxAgeMs;
    safetyContext.onlineMotorMask =
        sharedAtomicLoad(&sharedData->servo_online_bitmap);
    safetyContext.routedMotorMask =
        sharedAtomicLoad(&sharedData->servo_route_valid_bitmap);
    safetyContext.controllableRouteMask =
        sharedAtomicLoad(&sharedData->servo_route_controllable_bitmap);
    safetyContext.routeConflictMask =
        sharedAtomicLoad(&sharedData->servo_route_conflict_bitmap);
    safetyContext.multiTurnUnknownMask =
        sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap);
    safetyContext.turnOverflowMask =
        sharedAtomicLoad(&sharedData->servo_turn_overflow_bitmap);
    safetyContext.logicalRangeValidMask =
        sharedAtomicLoad(&sharedData->servo_logical_range_valid_bitmap);
    {
        // logical range 是多字段快照，复制到栈上后再逐命令裁决，避免跨核撕裂。
        SharedSafetyGuard guard;
        memcpy(logicalMin,
               sharedData->servo_logical_range_min,
               sizeof(logicalMin));
        memcpy(logicalMax,
               sharedData->servo_logical_range_max,
               sizeof(logicalMax));
    }
    safetyContext.logicalMin = logicalMin;
    safetyContext.logicalMax = logicalMax;
    safetyContext.requireOnline = true;
    // 自动标定必须允许从 Turn Unknown 重建基准；直控也承担人工恢复用途。
    // 负反馈和 Rope Init 则必须依赖可信 logical ABS。
    safetyContext.requireMultiTurnKnown =
        sharedData->system_state == SYSTEM_STATE_RUN_FEEDBACK ||
        sharedData->system_state == SYSTEM_STATE_ROPE_INIT_RUNNING;

    uint8_t appliedCount = 0;
    for (uint8_t i = 0; i < batch.count && i < SERVO_TARGET_BATCH_MAX; i++) {
        const ServoTargetCommand_t& cmd = batch.commands[i];
        ServoRoute route;
        FeetechServoDriver* bus =
            resolveMotorDriver(cmd.motorChannel, &route, true);
        const int32_t logicalOffset = bus
                                          ? bus->getLogicalOffset(route.servoId)
                                          : 0;
        const ServoSafetyDecision decision =
            p4::actuator::evaluateCommand(batch,
                                          i,
                                          logicalOffset,
                                          safetyContext);
        if (!decision.allowed || !bus) {
            const bool eventRecorded =
                p4::safety::reportServoSafetyDecision(decision,
                                                      safetyContext.systemState,
                                                      safetyContext.activeOwner);
            if (bus && decision.reason == ServoRejectReason::TurnOverflow) {
                // 越圈后必须保持扭矩关闭，直到用户显式执行 Reset Zero。
                bus->setTorqueEnabled(route.servoId, false);
            }
            if (eventRecorded) logServoSafetyReject(sharedData, decision);
            continue;
        }
        if (bus->setTarget(route.servoId,
                           decision.appliedAbs,
                           cmd.speed,
                           cmd.acc)) {
            busWritePending[route.busIndex] = true;
            appliedCount++;
        } else {
            // 驱动硬防线再次拒绝意味着策略上下文与驱动状态发生竞态，必须留下诊断。
            ServoSafetyDecision driverReject = decision;
            driverReject.allowed = false;
            driverReject.reason = ServoRejectReason::HardwareRange;
            driverReject.ruleId = SafetyRuleId::ActuatorHardwareRange;
            if (p4::safety::reportServoSafetyDecision(
                    driverReject,
                    safetyContext.systemState,
                    safetyContext.activeOwner)) {
                logServoSafetyReject(sharedData, driverReject);
            }
        }
    }

    for (uint8_t busIndex = 0; busIndex < NUM_BUSES; busIndex++) {
        if (busWritePending[busIndex]) {
            FeetechServoDriver* bus = servoBusRegistry().driver(busIndex);
            if (bus) {
                bus->syncWriteAll();
            }
        }
    }
    return appliedCount;
}

/**
 * Host Hold 是需要执行层确认的安全事务：只有整批命令都进入总线同步写缓存后才回报 APPLIED。
 * 状态机只读取 token 相同的结果，旧事务完成不会释放新一轮 owner。
 */
static void reportHostControlHoldResult(SystemRuntimeContext* sharedData,
                                        const ServoTargetBatch_t& batch,
                                        uint8_t appliedCount)
{
    if (!sharedData ||
        batch.purpose != SERVO_TARGET_PURPOSE_HOST_LEASE_HOLD) {
        return;
    }
    const bool fullyApplied =
        batch.count != 0 && appliedCount == batch.count;
    sharedAtomicStore(&sharedData->host_control_hold_result_token,
                      batch.transactionToken);
    sharedAtomicStore(&sharedData->host_control_hold_status,
                      (uint8_t)(fullyApplied
                          ? HOST_CONTROL_HOLD_APPLIED
                          : HOST_CONTROL_HOLD_FAILED));
    FW_LOG_WARN_PRINTF(
        "[HOST LEASE HOLD] token=%lu count=%u applied=%u status=%s\r\n",
        (unsigned long)batch.transactionToken,
        (unsigned)batch.count,
        (unsigned)appliedCount,
        fullyApplied ? "applied" : "failed");
}

/** 清空目标队列并关闭所有已映射在线舵机力矩。 */
static void emergencyTorqueOffMappedServos(SystemRuntimeContext* sharedData)
{
    p4::actuator::clearPending(sharedData);

    uint32_t requestedMask = 0;
    uint32_t torqueDisabledMask = 0;
    uint32_t torqueDisableFailedMask = 0;

    // E-STOP 不经过“唯一映射/每 BUS 十台”的运动门禁。即使同一 ID 被误接到
    // 两条 BUS，也要对扫描到的每个物理地址尝试关闭力矩。
    const ServoRouteSnapshot routes = servoRouteTable().snapshot();
    for (uint8_t busIndex = 0; busIndex < NUM_BUSES; ++busIndex) {
        FeetechServoDriver* bus = servoBusRegistry().driver(busIndex);
        uint32_t detectedIds = routes.detectedIdMask[busIndex];
        for (uint8_t id = 0; id <= MAX_SERVO_ID; ++id) {
            if ((detectedIds & (1UL << id)) == 0) {
                continue;
            }
            const int motor = findMotorChannelByServoId(id);
            if (motor < 0) {
                continue;
            }
            const uint32_t bit = 1UL << static_cast<uint8_t>(motor);
            requestedMask |= bit;
            if (bus && bus->setTorqueEnabled(id, false)) {
                torqueDisabledMask |= bit;
            } else {
                torqueDisableFailedMask |= bit;
            }
        }
    }
    torqueDisabledMask &= ~torqueDisableFailedMask;

    FW_LOG_INFO_PRINTF("[SERVO E-STOP] torque-off request=0x%08lX disabled=0x%08lX failed=0x%08lX\r\n",
                  (unsigned long)requestedMask,
                  (unsigned long)torqueDisabledMask,
                  (unsigned long)torqueDisableFailedMask);
}

/** 消费 New Device Reset 请求，并用 token 发布恢复结果。 */
static void processHostMultiTurnRestore(SystemRuntimeContext* sharedData, uint32_t* lastToken)
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
static void processCalibrationTrustCommit(SystemRuntimeContext* sharedData,
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
        ServoRoute route;
        FeetechServoDriver* bus = resolveMotorDriver(ch, &route, true);
        if (bus && bus->acceptCurrentSessionAsTrusted(route.servoId)) {
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
    bool initialized;                   // 是否已经完成本次 P4 上电会话的 NVS 恢复。
    bool hostWasOnline;                 // 上一周期上位机在线状态，用于检测心跳掉线边沿。
    bool saveQueued;                    // 当前是否有一个保存请求正在 NVS 任务中执行。
    bool motionWasActive;               // 上一周期是否处于控制/标定/直控等运动状态。
    bool deferredSavePending;           // 运动期间产生的保存需求是否等待停机后补写。
    uint8_t lastMotionState;             // 上一次观察到的系统状态，辅助记录保存触发边沿。
    uint32_t lastSaveMs;                 // 最近一次成功排队保存的 P4 毫秒时间。
    uint32_t lastValidMask;              // 最近成功落盘快照中包含的可信舵机位图。
    uint32_t pendingRequestToken;        // 正在执行的保存事务号；用于匹配异步完成结果。
    uint32_t pendingQueuedAtMs;          // 当前保存事务进入队列的时间。
    uint32_t pendingValidMask;           // 当前保存事务携带的可信舵机位图。
    int32_t lastAbs[SERVO_TOTAL_NUM];     // 最近成功落盘的逻辑多圈 ABS。
    int32_t pendingAbs[SERVO_TOTAL_NUM];  // 当前保存事务提交的逻辑多圈 ABS。
    int16_t lastRaw[SERVO_TOTAL_NUM];     // 最近成功落盘的单圈 raw 锚点。
    int16_t pendingRaw[SERVO_TOTAL_NUM];  // 当前保存事务提交的单圈 raw 锚点。
    int16_t lastTurn[SERVO_TOTAL_NUM];    // 最近成功落盘的逻辑圈数。
    int16_t pendingTurn[SERVO_TOTAL_NUM]; // 当前保存事务提交的逻辑圈数。
    uint32_t deferredRequestToken;        // 延迟保存事务号，停机后转入 pending。
    uint32_t deferredValidMask;           // 延迟快照包含的可信舵机位图。
    int32_t deferredAbs[SERVO_TOTAL_NUM];  // 运动期间冻结的逻辑多圈 ABS。
    int16_t deferredRaw[SERVO_TOTAL_NUM];  // 运动期间冻结的单圈 raw 锚点。
    int16_t deferredTurn[SERVO_TOTAL_NUM]; // 运动期间冻结的逻辑圈数。
    uint8_t deferredReason;               // 延迟事务的保存原因。
    uint8_t deferredTurnChanged;           // 延迟事务是否包含圈数变化。
    uint8_t deferredEdgeChanged;           // 延迟事务是否包含状态边沿。
};

// P4 持续供电而舵机单独掉电时使用的 RAM 恢复锚点。
// 该状态与 Flash 开关独立：Flash OFF 只禁止跨 P4 掉电持久化，
// 不能丢弃本次 P4 上电会话内已经冻结的可信多圈位置。
struct ServoHotplugRecoveryAnchors {
    uint32_t validMask;                    // 已冻结可信锚点的舵机位图。
    int32_t logicalAbs[SERVO_TOTAL_NUM];    // 舵机掉线前最后可信的 P4 逻辑 ABS。
    int16_t raw[SERVO_TOTAL_NUM];           // 与 logicalAbs 同帧的单圈 raw。
};

// 热插拔恢复按 RAM 会话锚点、未完成事务、已落盘快照的优先级选择来源。
enum ServoRecoveryAnchorSource : uint8_t {
    SERVO_RECOVERY_ANCHOR_NONE = 0,
    SERVO_RECOVERY_ANCHOR_RAM_POWER_LOSS = 1,
    SERVO_RECOVERY_ANCHOR_NVS_DEFERRED = 2,
    SERVO_RECOVERY_ANCHOR_NVS_PENDING = 3,
    SERVO_RECOVERY_ANCHOR_NVS_STORED = 4,
};

/** 收集当前可写入 Flash 的可信 logical ABS/raw/turn 快照。 */
static bool collectTrustedMultiTurnSnapshot(SystemRuntimeContext* sharedData,
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
        ServoRoute route;
        FeetechServoDriver* bus = resolveMotorDriver(ch, &route, false);
        if (!bus || !bus->isOnline(route.servoId)) {
            continue;
        }
        const int16_t raw =
            normalizeServoRawSingleTurn(bus->getRawPosition(route.servoId));
        const int32_t absPos = bus->getAbsolutePosition(route.servoId);
        motorAbs[ch] = absPos;
        motorRaw[ch] = raw;
        motorTurn[ch] = (int16_t)((absPos - raw) / 4096);
        mask |= bit;
    }
    *validMask = mask;
    return mask != 0;
}

/** 用启动时 NVS 快照初始化运行时去重缓存。 */
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
    SystemRuntimeContext* sharedData,
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

/** 处理每路离线->在线事务，选择 RAM/NVS 锚点并完成 raw 一致性恢复。 */
static void processServoOnlineTransitions(
    SystemRuntimeContext* sharedData,
    const MultiTurnNvsRuntimeState* nvsState,
    ServoHotplugRecoveryAnchors* hotplugAnchors)
{
    if (!sharedData || !nvsState || !hotplugAnchors) {
        return;
    }
    static uint8_t nextChannel = 0;
    const uint32_t nowMs = p4::platform::time::monotonicMillis();
    // 每个 10 ms 周期最多处理一台舵机，避免多台同时上线时连续 EEPROM
    // 事务长时间占用总线并拖慢反馈循环。
    for (uint8_t offset = 0; offset < SERVO_TOTAL_NUM; offset++) {
        const uint8_t ch =
            (uint8_t)((nextChannel + offset) % SERVO_TOTAL_NUM);
        const uint32_t bit = (1UL << ch);
        ServoRoute route;
        // 恢复过程会关闭力矩、改多圈模式并恢复 offset，属于物理写操作。
        // BUS 超过十台时整条 BUS 必须保持只读，不能借恢复分支绕过容量门禁。
        FeetechServoDriver* bus = resolveMotorDriver(ch, &route, true);
        if (!bus || !bus->isRecoveryAttemptDue(route.servoId, nowMs)) {
            continue;
        }
        nextChannel = (uint8_t)((ch + 1) % SERVO_TOTAL_NUM);

        const uint8_t id = route.servoId;
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
                      (unsigned)route.busIndex,
                      (unsigned)id,
                      (unsigned)attemptNumber,
                      (unsigned)mmhand::config::servo::kRecoveryMaxAttempts,
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

/** 比较当前快照与最近落盘快照，决定是否值得写 Flash。 */
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

/** 判断同一快照是否已经进入异步保存队列，避免重复排队。 */
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

/** 容量为 1 的 NVS 队列只保留最新请求。 */
static bool queueLatestMultiTurnNvsJob(
                                       MessageChannel<MultiTurnNvsSaveJob_t>& queue,
                                       const MultiTurnNvsSaveJob_t* job)
{
    if (!job) {
        return false;
    }
    MultiTurnNvsSaveJob_t queuedJob;
    if (job->operation != MULTITURN_NVS_OPERATION_RESET_ZERO &&
        queue.peek(&queuedJob) &&
        queuedJob.operation == MULTITURN_NVS_OPERATION_RESET_ZERO) {
        // Reset Zero 必须先把新 turn0 落盘，普通周期快照不得覆盖该事务。
        return false;
    }
    return queue.overwrite(*job);
}

/** 运动期间缓存最新可信快照，等 owner 释放后再写 Flash。 */
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
    SystemRuntimeContext* sharedData,
    MultiTurnNvsRuntimeState* state,
    ServoHotplugRecoveryAnchors* hotplugAnchors)
{
    if (!sharedData || !state || !hotplugAnchors) {
        return;
    }

    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        ServoRoute route;
        FeetechServoDriver* bus = resolveMotorDriver(ch, &route, false);
        if (!bus) {
            continue;
        }

        int32_t logicalAbs = 0;
        int16_t raw = 0;
        int16_t turn = 0;
        uint32_t sampleTimestampMs = 0;
        if (!bus->consumeOfflineTrustedSnapshot(
                route.servoId, &logicalAbs, &raw, &turn, &sampleTimestampMs)) {
            continue;
        }

        const uint32_t bit = 1UL << ch;
        hotplugAnchors->logicalAbs[ch] = logicalAbs;
        hotplugAnchors->raw[ch] = raw;
        hotplugAnchors->validMask |= bit;
        FW_LOG_WARN_PRINTF(
            "[SERVO HOTPLUG ANCHOR] M%02u logical=%ld raw=%d turn=%d ageMs=%lu\r\n",
            (unsigned)ch,
            (long)logicalAbs,
            (int)raw,
            (int)turn,
            (unsigned long)(p4::platform::time::monotonicMillis() - sampleTimestampMs));

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
            (unsigned long)(p4::platform::time::monotonicMillis() - sampleTimestampMs));
    }
}

/** 将可信快照封装成异步 NVS 作业，并更新 pending 去重状态。 */
static bool enqueueMultiTurnNvsSnapshot(SystemRuntimeContext* sharedData,
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
        !sharedData->multiturnNvsSaveQueue.valid()) {
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
    job.queuedAtMs = p4::platform::time::monotonicMillis();
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
    memcpy(state->pendingAbs, motorAbs, sizeof(state->pendingAbs));
    memcpy(state->pendingRaw, motorRaw, sizeof(state->pendingRaw));
    memcpy(state->pendingTurn, motorTurn, sizeof(state->pendingTurn));
    return true;
}

/** Reset Zero 只允许在无运动 owner、控制关闭且非 PAUSED 时执行。 */
static bool resetZeroStateIsSafe(const SystemRuntimeContext* sharedData)
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
static void processServoResetZeroRequest(SystemRuntimeContext* sharedData,
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
        !sharedData->multiturnNvsSaveQueue.valid()) {
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
    job.queuedAtMs = p4::platform::time::monotonicMillis();

    uint32_t hardwareResetMask = 0;
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        const uint32_t bit = 1UL << ch;
        if ((executableMask & bit) == 0) {
            continue;
        }
        ServoRoute route;
        FeetechServoDriver* bus = resolveMotorDriver(ch, &route, true);
        int16_t raw = 0;
        if (!bus ||
            !bus->resetTurnCounter(route.servoId, &raw)) {
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

/** 运动结束后提交先前缓存的掉电/边沿快照。 */
static bool flushDeferredMultiTurnSnapshot(SystemRuntimeContext* sharedData,
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

/** 把 NVS 任务成功结果合并为新的去重基线。 */
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

/** 消费异步 NVS 结果，发布成功/失败 token 并完成 Reset Zero 闭环。 */
static void processMultiTurnNvsSaveResults(SystemRuntimeContext* sharedData,
                                           MultiTurnNvsRuntimeState* state)
{
    if (!sharedData || !state ||
        !sharedData->multiturnNvsResultQueue.valid()) {
        return;
    }

    MultiTurnNvsSaveResult_t result;
    while (sharedData->multiturnNvsResultQueue.receive(&result)) {
        if (state->saveQueued && result.queuedAtMs == state->pendingQueuedAtMs) {
            state->saveQueued = false;
        }
        if (result.operation == MULTITURN_NVS_OPERATION_RESET_ZERO &&
            state->saveQueued) {
            MultiTurnNvsSaveJob_t queuedJob;
            const bool trackedOrdinarySaveStillQueued =
                sharedData->multiturnNvsSaveQueue.peek(&queuedJob) &&
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

/** 采集并按变化/强制原因提交一次当前多圈快照。 */
static bool saveCurrentMultiTurnSnapshot(SystemRuntimeContext* sharedData,
                                         MultiTurnNvsRuntimeState* state,
                                         uint8_t reason,
                                         bool force,
                                         uint32_t requestToken)
{
    if (!sharedData || !state) {
        return false;
    }
    if (!sharedData->multiturn_nvs_enabled ||
        !sharedData->multiturnNvsSaveQueue.valid()) {
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
    const uint32_t nowMs = p4::platform::time::monotonicMillis();
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

/** 根据运动完成、心跳中断、跨圈和掉电边沿决定保存时机。 */
static void processMultiTurnNvsSave(SystemRuntimeContext* sharedData,
                                    MultiTurnNvsRuntimeState* state,
                                    uint32_t* lastSaveToken)
{
    if (!sharedData || !state || !lastSaveToken) {
        return;
    }

    processMultiTurnNvsSaveResults(sharedData, state);

    const uint32_t nowMs = p4::platform::time::monotonicMillis();
    // 与状态机共用同一份原子心跳快照，NVS 在线判定和控制租约不会各读到不同半帧。
    const uint32_t hostHeartbeatMs =
        sharedAtomicLoad(&sharedData->host_rx_heartbeat_ms);
    const bool hostOnline = hostHeartbeatMs != 0 &&
        (uint32_t)(nowMs - hostHeartbeatMs) <= kHostControlLeaseTimeoutMs;
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

        // 控制/标定运动期间只刷新可信 RAM 快照；Flash/NVS 写入延迟到 owner 释放之后。
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

/** 用启动快照逐路校验当前 raw，并只恢复通过容差的通道。 */
static void processBootNvsRestore(SystemRuntimeContext* sharedData,
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
    const uint32_t nowMs = p4::platform::time::monotonicMillis();
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

/**
 * ServoBusTask 独占的跨周期运行上下文。把路由、恢复与 NVS 事务历史集中保存，
 * 业务 step 不再依赖隐藏在无限循环栈帧中的局部变量。
 */
struct ServoRuntimeContext {
    bool initialized;
    uint32_t configuredRouteGeneration;
    uint8_t readBusPhase;
    uint32_t lastEmergencyStopToken;
    uint32_t lastRestoreToken;
    uint32_t lastCalibrationTrustToken;
    uint32_t lastLoopStartMs;
    uint32_t queueDropTotal;
    uint32_t lastQueueDropLogMs;
#if MULTITURN_NVS_PERSISTENCE_ENABLE
    uint32_t lastServoResetZeroToken;
    uint32_t lastNvsSaveToken;
    uint32_t nvsRestoreDeadlineMs;
    bool hasBootNvsSnapshot;
    bool bootNvsRestoreDone;
    MultiTurnNvsSnapshot bootNvsSnapshot;
    MultiTurnNvsRuntimeState nvsSaveState;
    ServoHotplugRecoveryAnchors hotplugRecoveryAnchors;
#endif
};

static ServoRuntimeContext g_servoRuntime;

/** 初始化舵机总线与恢复上下文；本函数不创建任务、不进入循环。 */
bool initServoBusService(SystemRuntimeContext* sharedData, uint32_t nowMs)
{
    if (!sharedData) {
        return false;
    }

    memset(&g_servoRuntime, 0, sizeof(g_servoRuntime));
    beginServoBuses();
    servoDiscoveryManager().begin();
    publishServoRouteSnapshot(sharedData);
    g_servoRuntime.lastEmergencyStopToken =
        sharedAtomicLoad(&sharedData->servo_emergency_stop_token);
    g_servoRuntime.lastRestoreToken =
        sharedAtomicLoad(&sharedData->restore_multi_turn_request_token);
    g_servoRuntime.lastCalibrationTrustToken =
        sharedAtomicLoad(&sharedData->calibration_trust_request_token);
#if MULTITURN_NVS_PERSISTENCE_ENABLE
    g_servoRuntime.lastServoResetZeroToken =
        sharedAtomicLoad(&sharedData->servo_reset_zero_request_token);
    g_servoRuntime.lastNvsSaveToken =
        sharedAtomicLoad(&sharedData->multiturn_nvs_save_request_token);
    g_servoRuntime.nvsRestoreDeadlineMs = nowMs + kNvsRestoreWaitWindowMs;
    bool nvsEnabled = true;
    if (!multiTurnNvsLoadEnabled(&nvsEnabled)) {
        FW_LOG_INFO_PRINTLN("[NVS MULTITURN] enabled flag load failed, default enabled");
        nvsEnabled = true;
    }
    sharedData->multiturn_nvs_enabled = nvsEnabled ? 1 : 0;
    g_servoRuntime.hasBootNvsSnapshot =
        nvsEnabled && multiTurnNvsLoadLatest(&g_servoRuntime.bootNvsSnapshot);
    sharedData->multiturn_nvs_snapshot_valid =
        g_servoRuntime.hasBootNvsSnapshot ? 1 : 0;
    sharedData->multiturn_nvs_save_valid_mask =
        g_servoRuntime.hasBootNvsSnapshot
            ? (g_servoRuntime.bootNvsSnapshot.validMask & validMotorChannelMask())
            : 0;
    if (g_servoRuntime.hasBootNvsSnapshot) {
        // 启动恢复与 UI 必须引用同一份已通过 CRC 校验的 Flash 快照。
        memcpy(sharedData->multiturn_nvs_snapshot_raw,
               g_servoRuntime.bootNvsSnapshot.raw,
               sizeof(sharedData->multiturn_nvs_snapshot_raw));
    } else {
        memset(sharedData->multiturn_nvs_snapshot_raw,
               0,
               sizeof(sharedData->multiturn_nvs_snapshot_raw));
    }
    g_servoRuntime.bootNvsRestoreDone = !g_servoRuntime.hasBootNvsSnapshot;
    g_servoRuntime.nvsSaveState.hostWasOnline = true;
    if (!nvsEnabled) {
        FW_LOG_INFO_PRINTLN("[NVS MULTITURN] disabled by user, boot restore skipped");
    } else if (g_servoRuntime.hasBootNvsSnapshot) {
        seedNvsRuntimeStateFromSnapshot(&g_servoRuntime.nvsSaveState,
                                        g_servoRuntime.bootNvsSnapshot);
        FW_LOG_INFO_PRINTF("[NVS MULTITURN] loaded seq=%lu mask=0x%08lX\r\n",
                      (unsigned long)g_servoRuntime.bootNvsSnapshot.sequence,
                      (unsigned long)(g_servoRuntime.bootNvsSnapshot.validMask & validMotorChannelMask()));
    } else {
        FW_LOG_INFO_PRINTLN("[NVS MULTITURN] no valid boot snapshot");
        // type=2 表示多圈快照；与 SystemAssembly 上报的 Manual 快照(type=1)共用 PST-7004。
        const SafetyRuleId snapshotRule =
            p4::safety::persistenceSnapshotValidityRule(
                nvsEnabled, g_servoRuntime.hasBootNvsSnapshot);
        if (snapshotRule != SafetyRuleId::None) {
            p4::safety::reportSafetyRuleEvent(
                sharedData,
                snapshotRule,
                p4::safety::makeSafetyEventEvidence(0xFF, 0xFF, 2, 0));
        }
    }
#else
    FW_LOG_INFO_PRINTLN("[NVS MULTITURN] persistence disabled");
#endif

    g_servoRuntime.initialized = true;
    return true;
}

/** 推进一个舵机服务周期；周期等待和运行时统计由 ServoBusTask 负责。 */
bool stepServoBusService(SystemRuntimeContext* sharedData,
                             const ServoServiceInputSnapshot& input,
                             const ServiceOutputPort& output)
{
    if (!sharedData || !g_servoRuntime.initialized) {
        return false;
    }
    if (!servoSnapshotIsConsistent(input)) {
        return false;
    }
    const uint32_t nowMs = input.nowMs;
    ServoRuntimeContext& runtime = g_servoRuntime;
    const uint32_t loopStartMs = nowMs;
    auto& configuredRouteGeneration = runtime.configuredRouteGeneration;
    auto& readBusPhase = runtime.readBusPhase;
    auto& lastEmergencyStopToken = runtime.lastEmergencyStopToken;
    auto& lastRestoreToken = runtime.lastRestoreToken;
    auto& lastCalibrationTrustToken = runtime.lastCalibrationTrustToken;
    auto& lastLoopStartMs = runtime.lastLoopStartMs;
    auto& queueDropTotal = runtime.queueDropTotal;
    auto& lastQueueDropLogMs = runtime.lastQueueDropLogMs;
#if MULTITURN_NVS_PERSISTENCE_ENABLE
    auto& lastServoResetZeroToken = runtime.lastServoResetZeroToken;
    auto& lastNvsSaveToken = runtime.lastNvsSaveToken;
    auto& nvsSaveState = runtime.nvsSaveState;
    auto& hotplugRecoveryAnchors = runtime.hotplugRecoveryAnchors;
#endif
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
        const uint8_t currentSystemState =
            sharedAtomicLoad(&sharedData->system_state);
        const bool motionActive =
            currentSystemState == SYSTEM_STATE_RUN_DIRECT ||
            currentSystemState == SYSTEM_STATE_RUN_FEEDBACK ||
            currentSystemState == SYSTEM_STATE_CALIBRATION_RUNNING ||
            currentSystemState == SYSTEM_STATE_ROPE_INIT_RUNNING;
        const bool discoveryUsedBus =
            servoDiscoveryManager().step(loopStartMs, motionActive);
        publishServoRouteSnapshot(sharedData);
        const uint32_t routeGeneration =
            sharedAtomicLoad(&sharedData->servo_route_generation);
        if (!servoDiscoveryManager().active() &&
            routeGeneration != configuredRouteGeneration) {
            configureRoutedServosForMultiTurn();
            configuredRouteGeneration = routeGeneration;
        }
        if (discoveryUsedBus) {
            // 扫描周期不混入同步读写，确保一轮最多产生一次短超时 UART 访问。
            return true;
        }
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
            const uint32_t applyBeginMs =
                p4::platform::time::monotonicMillis();
            const uint8_t appliedCount =
                applyServoTargetBatch(sharedData, batch, nowMs);
            reportHostControlHoldResult(sharedData, batch, appliedCount);
            const uint32_t applyElapsedMs =
                p4::platform::time::monotonicMillis() - applyBeginMs;
            if (applyElapsedMs > kServoApplyStallLogThresholdMs) {
                FW_LOG_WARN_PRINTF("[SERVO LOOP STALL] stage=apply_target elapsedMs=%lu source=%u owner=%u ageMs=%lu count=%u applied=%u\r\n",
                                   (unsigned long)applyElapsedMs,
                                   (unsigned)batch.source,
                                   (unsigned)owner,
                                   (unsigned long)(p4::platform::time::monotonicMillis() - batch.timestamp),
                                   (unsigned)batch.count,
                                   (unsigned)appliedCount);
            }
            if (batch.count != 0 && appliedCount == 0) {
                queueDropTotal++;
                const uint32_t sampleNowMs =
                    p4::platform::time::monotonicMillis();
                if ((uint32_t)(sampleNowMs - lastQueueDropLogMs) >=
                    kServoQueueDropLogIntervalMs) {
                    lastQueueDropLogMs = sampleNowMs;
                    FW_LOG_WARN_PRINTF("[SERVO QUEUE DROP] source=%u owner=%u ageMs=%lu count=%u droppedTotal=%lu state=%u\r\n",
                                       (unsigned)batch.source,
                                       (unsigned)owner,
                                       (unsigned long)(sampleNowMs - batch.timestamp),
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
            ServoRoute route;
            if (servoRouteTable().resolveControllable(ch, &route)) {
                addServoReadTarget(readIds,
                              readCounts,
                              route.busIndex,
                              route.servoId);
            }
        }

        const uint8_t readStartBus = (readBusPhase == 0) ? 0 : 2;
        for (uint8_t offset = 0; offset < 2; offset++) {
            const uint8_t busIndex = (uint8_t)(readStartBus + offset);
            if (busIndex >= NUM_BUSES || readCounts[busIndex] == 0) {
                continue;
            }
            const uint32_t readBeginMs =
                p4::platform::time::monotonicMillis();
            FeetechServoDriver* bus = servoBusRegistry().driver(busIndex);
            const int readSuccess =
                bus ? bus->syncReadPositions(readIds[busIndex], readCounts[busIndex]) : 0;
            const uint32_t readElapsedMs =
                p4::platform::time::monotonicMillis() - readBeginMs;
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
#if MULTITURN_NVS_PERSISTENCE_ENABLE
        // 必须在撤销动态路由之前消费驱动冻结的最后可信反馈；否则路由失效后
        // 无法再定位原 BUS，RAM 热插拔锚点与掉电 NVS 快照都会丢失。
        cacheServoPowerLossSnapshots(
            sharedData, &nvsSaveState, &hotplugRecoveryAnchors);
#endif
        // 掉线只清除该 Mxx 的运行时路由；定向重扫将在退出运动状态后逐 BUS 执行。
        const ServoRouteSnapshot routeSnapshot = servoRouteTable().snapshot();
        for (uint8_t motor = 0; motor < SERVO_TOTAL_NUM; ++motor) {
            const ServoRoute& route = routeSnapshot.routes[motor];
            if (route.busIndex >= NUM_BUSES ||
                route.state != ServoRouteState::Online) {
                continue;
            }
            FeetechServoDriver* driver =
                servoBusRegistry().driver(route.busIndex);
            if (driver && driver->isOfflineInvalidated(route.servoId)) {
                servoRouteTable().markMotorOffline(motor);
                servoDiscoveryManager().requestMotorScan(motor);
            }
        }
        publishServoRouteSnapshot(sharedData);

#if MULTITURN_NVS_PERSISTENCE_ENABLE
        // 掉线检测会在 sync read 中冻结最后可信反馈。必须先把它搬入独立
        // RAM 锚点，再处理重新上线事务，避免快速断电重启时恢复抢在保存前面。
        // 先吸收异步 Flash 写入结果，使恢复逻辑能看到最新已提交快照。
        processMultiTurnNvsSaveResults(sharedData, &nvsSaveState);
        processBootNvsRestore(sharedData,
                              runtime.hasBootNvsSnapshot
                                  ? &runtime.bootNvsSnapshot
                                  : NULL,
                              &runtime.bootNvsRestoreDone,
                              runtime.nvsRestoreDeadlineMs);
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
    return true;
}
