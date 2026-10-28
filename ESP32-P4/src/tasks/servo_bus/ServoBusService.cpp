/**
 * @file ServoBusService.cpp
 * @brief 调度四条舵机总线、应用目标批次并发布统一反馈与恢复状态。
 *
 * 本任务不再直接调用飞特 SDK。底层串口协议由 FeetechServoDriver 封装；
 * 这里负责收发调度；唯一AbsoluteRestore执行器处理所有0x0B恢复。
 */
#include "ServoBusService.h"
#include "ServoAbsoluteRestore.h"
#include "ServoMaintenance.h"

#include <string.h>

#include "ServoDiscoveryManager.h"
#include "ServoFeedbackPublisher.h"
#include "ParallelServoFeedbackCycle.h"
#include "ServoRouteTable.h"
#include "ServoBusRouting.h"
#include "../host_comm/HostCalibrationCommandHandler.h"
#include "../../system/ActuatorCommandGateway.h"
#include "../../system/ServiceCyclePolicy.h"
#include "../../config/topology/HardwareMap.h"
#include "../../config/DiagnosticsConfig.h"
#include "../../config/SafetyConfig.h"
#include "../../config/ServoConfig.h"
#include "../../system/rtos/SharedSafetyState.h"
#include "../../system/rtos/TaskSignal.h"
#include "../../system/time/Clock.h"
#include "../../system/log/FirmwareLog.h"
#include "../../system/SystemRuntimeContext.h"
#include "../../system/ports/SystemEventPort.h"
#include "../../system/ports/DirectTargetResult.h"
#include "../../system/TaskRuntimeMetrics.h"
#include "../../config/safety/ServoPositionRange.h"


#include "../../hal/servo/FeetechServoDriver.h"
#include "../../hal/servo/ServoBusRegistry.h"
#include "../../safety/policies/ActuatorSafetyPolicy.h"
#include "../../safety/SafetyEventReporter.h"


using namespace mmhand::config::diagnostics;
using namespace mmhand::config::safety;
using namespace mmhand::config::servo;


/** 仅在统一事件缓存接受新事件时输出一次现场详情，避免重复维护日志节流状态。 */
static void logServoSafetyReject(SystemRuntimeContext* sharedData,
                                 const ServoSafetyDecision& decision)
{
    if (!sharedData || decision.motorChannel >= SERVO_TOTAL_NUM) {
        return;
    }
    if (decision.reason == ServoRejectReason::HardwareRange) {
        const ServoLogicalRange range = servoPositionRange();
        FW_LOG_WARN_PRINTF(
            "[SERVO SAFETY REJECT] M%02u reason=%s abs=%ld range=[%ld,%ld] "
            "owner=%u state=%u\r\n",
            (unsigned)decision.motorChannel,
            p4::actuator::rejectReasonName(decision.reason),
            (long)decision.requestedAbs,
            (long)range.minAbs,
            (long)range.maxAbs,
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

struct ServoBatchApplyResult {
    uint8_t appliedCount;   ///< 成功进入同步写的命令数。
    uint32_t requestedMask; ///< 批次声明的全部 Mxx。
    uint32_t appliedMask;   ///< 原子成功时等于 requestedMask。
    uint32_t rejectedMask;  ///< 原子失败时等于 requestedMask。
};

/** 计算批次涉及的有效电机位图；损坏的 Mxx 由结构检查单独报告。 */
static uint32_t batchMotorMask(const ServoTargetBatch_t& batch)
{
    uint32_t mask = 0;
    const uint8_t count = batch.count < SERVO_TARGET_BATCH_MAX
                              ? batch.count
                              : SERVO_TARGET_BATCH_MAX;
    for (uint8_t i = 0; i < count; ++i) {
        if (batch.commands[i].motorChannel < SERVO_TOTAL_NUM) {
            mask |= 1UL << batch.commands[i].motorChannel;
        }
    }
    return mask;
}

/**
 * 应用一个目标批次，并在写入任一驱动缓存前预检全部命令。
 * 任一命令不满足路由、安全范围或缓存容量时整批拒绝，双腱目标不会只执行一侧。
 */
static ServoBatchApplyResult applyServoTargetBatch(
    SystemRuntimeContext* sharedData,
    const ServoTargetBatch_t& batch,
    uint32_t nowMs)
{
    ServoBatchApplyResult result = {};
    result.requestedMask = batchMotorMask(batch);
    if (!sharedData) {
        result.rejectedMask = result.requestedMask;
        return result;
    }
    // 所有普通直控在消费端核对应用代际；token 仅随代际关联回执。
    if (batch.directGeneration != 0 &&
        (batch.source != SERVO_TARGET_OWNER_CONTROL ||
         batch.purpose != SERVO_TARGET_PURPOSE_NORMAL ||
         sharedAtomicLoad(&sharedData->applied_direct_generation) != batch.directGeneration ||
         sharedAtomicLoad(&sharedData->applied_direct_token) != batch.directToken ||
         sharedAtomicLoad(&sharedData->host_control_hold_status) ==
             HOST_CONTROL_HOLD_PENDING)) {
        result.rejectedMask = result.requestedMask;
        return result;
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
        result.rejectedMask = result.requestedMask;
        return result;
    }
    bool busWritePending[NUM_BUSES] = {false};
    uint8_t busCommandCount[NUM_BUSES] = {0};
    FeetechServoDriver* commandDrivers[SERVO_TARGET_BATCH_MAX] = {nullptr};
    ServoRoute commandRoutes[SERVO_TARGET_BATCH_MAX] = {};
    ServoSafetyDecision commandDecisions[SERVO_TARGET_BATCH_MAX] = {};
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
    safetyContext.requireOnline = true;
    // 直控承担人工恢复用途，允许由上位机重建基准。
    // 负反馈和 Rope Init 则必须依赖可信 logical ABS。
    safetyContext.requireMultiTurnKnown =
        sharedData->system_state == SYSTEM_STATE_RUN_FEEDBACK ||
        sharedData->system_state == SYSTEM_STATE_ROPE_INIT_RUNNING;

    // 第一遍只解析和裁决，不改驱动缓存。这样第二台失败时第一台不会提前留下目标。
    for (uint8_t i = 0; i < batch.count && i < SERVO_TARGET_BATCH_MAX; i++) {
        const ServoTargetCommand_t& cmd = batch.commands[i];
        FeetechServoDriver* bus =
            resolveMotorDriver(cmd.motorChannel, &commandRoutes[i], true);
        commandDecisions[i] =
            p4::actuator::evaluateCommand(batch,
                                          i,
                                          safetyContext);
        const ServoSafetyDecision& decision = commandDecisions[i];
        if (!decision.allowed || !bus) {
            const bool eventRecorded =
                p4::safety::reportServoSafetyDecision(decision,
                                                      safetyContext.systemState,
                                                      safetyContext.activeOwner);
            if (bus && decision.reason == ServoRejectReason::TurnOverflow) {
                // 越圈后必须保持扭矩关闭，直到用户显式执行 Reset Zero。
                bus->setTorqueEnabled(commandRoutes[i].servoId, false);
            }
            if (eventRecorded) logServoSafetyReject(sharedData, decision);
            result.rejectedMask = result.requestedMask;
            return result;
        }
        const uint8_t busIndex = commandRoutes[i].busIndex;
        if (busIndex >= NUM_BUSES ||
            ++busCommandCount[busIndex] > MAX_SERVOS_PER_BUS) {
            ServoSafetyDecision capacityReject = decision;
            capacityReject.allowed = false;
            capacityReject.reason = ServoRejectReason::BusOverCapacity;
            capacityReject.ruleId = SafetyRuleId::ActuatorBusOverCapacity;
            if (p4::safety::reportServoSafetyDecision(
                    capacityReject,
                    safetyContext.systemState,
                    safetyContext.activeOwner)) {
                logServoSafetyReject(sharedData, capacityReject);
            }
            result.rejectedMask = result.requestedMask;
            return result;
        }
        commandDrivers[i] = bus;
    }

    // 第二遍把完整批次写入各 BUS 缓存；意外失败时清空所有缓存，仍不发送半批。
    for (uint8_t i = 0; i < batch.count; ++i) {
        const ServoTargetCommand_t& cmd = batch.commands[i];
        if (!commandDrivers[i]->setTarget(commandRoutes[i].servoId,
                                          commandDecisions[i].appliedAbs,
                                          cmd.speed,
                                          cmd.acc)) {
            FW_LOG_WARN_PRINTF(
                "[SERVO TARGET] reject ch=%u id=%u online=%u reliable=%u mt=%u torqueOff=%u\r\n",
                (unsigned)cmd.motorChannel,
                (unsigned)commandRoutes[i].servoId,
                (unsigned)commandDrivers[i]->isOnline(commandRoutes[i].servoId),
                (unsigned)commandDrivers[i]->isAbsoluteReliable(commandRoutes[i].servoId),
                (unsigned)commandDrivers[i]->isMultiTurnConfigured(commandRoutes[i].servoId),
                (unsigned)commandDrivers[i]->isTorqueDisabled(commandRoutes[i].servoId));
            for (uint8_t busIndex = 0; busIndex < NUM_BUSES; ++busIndex) {
                FeetechServoDriver* driver = servoBusRegistry().driver(busIndex);
                if (driver) driver->clearQueuedTargets();
            }
            ServoSafetyDecision driverReject = commandDecisions[i];
            driverReject.allowed = false;
            driverReject.reason = ServoRejectReason::InvalidCommand;
            driverReject.ruleId = SafetyRuleId::ActuatorInvalidCommand;
            if (p4::safety::reportServoSafetyDecision(
                    driverReject,
                    safetyContext.systemState,
                    safetyContext.activeOwner)) {
                logServoSafetyReject(sharedData, driverReject);
            }
            result.rejectedMask = result.requestedMask;
            return result;
        }
        busWritePending[commandRoutes[i].busIndex] = true;
    }

    for (uint8_t busIndex = 0; busIndex < NUM_BUSES; busIndex++) {
        if (busWritePending[busIndex]) {
            FeetechServoDriver* bus = servoBusRegistry().driver(busIndex);
            if (bus) {
                bus->syncWriteAll();
            }
        }
    }
    result.appliedCount = batch.count;
    result.appliedMask = result.requestedMask;
    return result;
}

/**
 * Host Hold 是需要执行层确认的安全事务：只有整批命令都进入总线同步写缓存后才回报 APPLIED。
 * 状态机只读取 token 相同的结果，旧事务完成不会释放新一轮 owner。
 */
static void reportHostControlHoldResult(SystemRuntimeContext* sharedData,
                                        const ServoTargetBatch_t& batch,
                                        const ServoBatchApplyResult& result)
{
    if (!sharedData ||
        batch.purpose != SERVO_TARGET_PURPOSE_HOST_LEASE_HOLD) {
        return;
    }
    const bool fullyApplied =
        batch.count != 0 && result.appliedCount == batch.count;
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
        (unsigned)result.appliedCount,
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

/** 标定 F0 / Direct F1 共用的实际 ABS=0 事务；完成前保留上次结果快照。 */
struct ServoRebaseZeroContext {
    bool active;
    uint32_t token;
    uint32_t requestMask;
    uint32_t pendingMask;
    uint32_t successMask;
    uint32_t failedMask;
    uint32_t preparedMask;
};

/** RESET/坐标重建只能在没有运动 owner、控制关闭时执行。 */
static bool servoCoordinateMaintenanceSafe(SystemRuntimeContext* sharedData)
{
    if (!sharedData || sharedAtomicLoad(&sharedData->emergency_stop_latched) != 0 ||
        sharedAtomicLoad(&sharedData->control_enabled) != 0 ||
        sharedAtomicLoad(&sharedData->servo_target_owner) !=
            SERVO_TARGET_OWNER_NONE) {
        return false;
    }
    const uint8_t state = sharedAtomicLoad(&sharedData->system_state);
    return state == SYSTEM_STATE_UNCONTROLLABLE ||
           state == SYSTEM_STATE_IDLE ||
           state == SYSTEM_STATE_READY ||
           state == SYSTEM_STATE_RECOVERY_REQUIRED;
}

/** 上位机对账后认账：把当前原生 ABS 认作标定坐标系，不写电机。 */
static void captureTrustMultiTurn(SystemRuntimeContext* sharedData,
                                  uint32_t* lastToken)
{
    if (!sharedData || !lastToken) return;
    const uint32_t token =
        sharedAtomicLoad(&sharedData->trust_multi_turn_request_token);
    if (token == *lastToken) return;
    *lastToken = token;
    const uint32_t mask =
        sharedAtomicLoad(&sharedData->trust_multi_turn_request_mask) &
        validMotorChannelMask();
    if (!servoCoordinateMaintenanceSafe(sharedData) || mask == 0) {
        FW_LOG_INFO_PRINTF("[SERVO TRUST] rejected mask=0x%08lX\r\n",
                           (unsigned long)mask);
        return;
    }
    uint32_t trustedMask = 0;
    for (uint8_t channel = 0; channel < SERVO_TOTAL_NUM; ++channel) {
        const uint32_t bit = 1UL << channel;
        if ((mask & bit) == 0) continue;
        ServoRoute route = {};
        FeetechServoDriver* driver = resolveMotorDriver(channel, &route, true);
        if (!driver || !driver->isOnline(route.servoId) ||
            !driver->isMultiTurnConfigured(route.servoId)) continue;
        const int32_t absolute = driver->getAbsolutePosition(route.servoId);
        if (absolute < kServoHardwareCommandMin ||
            absolute > kServoHardwareCommandMax) continue;
        if (!driver->trustCurrentAbsolute(route.servoId)) continue;
        sharedAtomicFetchAnd(&sharedData->multi_turn_unknown_bitmap, ~bit);
        trustedMask |= bit;
    }
    FW_LOG_INFO_PRINTF(
        "[SERVO TRUST] request=0x%08lX trusted=0x%08lX unknown=0x%08lX\r\n",
        (unsigned long)mask, (unsigned long)trustedMask,
        (unsigned long)sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap));
}

struct ServoOnlineRecoveryContext {
    uint8_t activeChannel;
    uint8_t nextChannel;
};

static void processServoOnlineTransitions(
    SystemRuntimeContext* sharedData, ServoOnlineRecoveryContext* recovery)
{
    if (!servoCoordinateMaintenanceSafe(sharedData)) return;
    const uint32_t nowMs = p4::platform::time::monotonicMillis();
    if (recovery->activeChannel >= SERVO_TOTAL_NUM) {
        for (uint8_t offset = 0; offset < SERVO_TOTAL_NUM; ++offset) {
            const uint8_t channel = (recovery->nextChannel + offset) % SERVO_TOTAL_NUM;
            ServoRoute route;
            FeetechServoDriver* driver = resolveMotorDriver(channel, &route, true);
            if (driver && driver->isRecoveryAttemptDue(route.servoId, nowMs)) {
                recovery->activeChannel = channel;
                recovery->nextChannel = (channel + 1) % SERVO_TOTAL_NUM;
                break;
            }
        }
    }
    const uint8_t channel = recovery->activeChannel;
    if (channel >= SERVO_TOTAL_NUM) return;
    ServoRoute route;
    FeetechServoDriver* driver = resolveMotorDriver(channel, &route, true);
    if (!driver || !driver->isOnline(route.servoId)) {
        recovery->activeChannel = 0xFF;
        return;
    }
    if (!driver->checkOnlineConfiguration(route.servoId)) {
        driver->finishRecoveryAttempt(route.servoId, false, nowMs);
        recovery->activeChannel = 0xFF;
        return;
    }
    // 配置完成仍保持Unknown；标定由上位机平移、加载并显式认账。
    driver->finishRecoveryAttempt(route.servoId, true, nowMs);
    recovery->activeChannel = 0xFF;
}

/** 标定失效已经完成后，才允许 Host 发布本次最终结果。 */
static void finishServoRebaseZero(SystemRuntimeContext* sharedData,
                                  ServoRebaseZeroContext* rebase)
{
    {
        SharedSafetyGuard guard;
        sharedAtomicStore(&sharedData->servo_rebase_zero_success_mask,
                          rebase->successMask);
        sharedAtomicStore(&sharedData->servo_rebase_zero_failed_mask,
                          rebase->failedMask);
        sharedAtomicStore(&sharedData->servo_rebase_zero_result_token,
                          rebase->token);
    }
    rebase->active = false;
    postSystemEvent(sharedData, SYSTEM_EVENT_REEVALUATE_READY);
}

/** 仅冻结请求；UART 操作留到完整反馈轮次结束，且每轮最多一台。 */
static void captureServoRebaseZero(SystemRuntimeContext* sharedData,
                                   uint32_t* lastToken,
                                   ServoRebaseZeroContext* rebase)
{
    if (rebase->active) return;
    const uint32_t token =
        sharedAtomicLoad(&sharedData->servo_rebase_zero_request_token);
    if (token == *lastToken) return;
    *lastToken = token;
    *rebase = ServoRebaseZeroContext{};
    rebase->token = token;
    rebase->requestMask =
        sharedAtomicLoad(&sharedData->servo_rebase_zero_request_mask) &
        validMotorChannelMask();
    rebase->pendingMask = rebase->requestMask;
    rebase->active = true;
    p4::actuator::clearPending(sharedData);
    if (!servoCoordinateMaintenanceSafe(sharedData) ||
        rebase->requestMask == 0) {
        rebase->failedMask = rebase->requestMask;
        rebase->pendingMask = 0;
        finishServoRebaseZero(sharedData, rebase);
    }
}

static bool stepServoRebaseZero(SystemRuntimeContext* sharedData,
    ServoRebaseZeroContext* rebase,
    p4::servo::AbsoluteRestore* writer)
{
    if (!rebase->active) return false;
    if (!servoCoordinateMaintenanceSafe(sharedData)) {
        writer->cancel();
        rebase->failedMask |= rebase->pendingMask;
        rebase->pendingMask = 0;
    }
    if (rebase->pendingMask) {
        uint8_t channel = 0;
        while (!(rebase->pendingMask & (1UL << channel))) ++channel;
        const uint32_t bit = 1UL << channel;
        ServoRoute route = {};
        FeetechServoDriver* driver = resolveMotorDriver(channel, &route, true);
        if (!(rebase->preparedMask & bit)) {
            invalidateAutomaticCalibration(sharedData, jointMaskForMotorMask(bit));
            sharedAtomicFetchOr(&sharedData->multi_turn_unknown_bitmap, bit);
            if (driver) driver->discardAbsoluteRestore(route.servoId);
            rebase->preparedMask |= bit;
        }
        const auto result = writer->step(driver, route.servoId,
                                         p4::platform::time::monotonicMillis());
        if (result == p4::servo::RestoreResult::Pending) return true;
        if (result == p4::servo::RestoreResult::Success) {
            rebase->successMask |= bit;
            sharedAtomicFetchAnd(&sharedData->multi_turn_unknown_bitmap, ~bit);
            sharedAtomicFetchAnd(&sharedData->servo_turn_overflow_bitmap, ~bit);
        } else rebase->failedMask |= bit;
        FW_LOG_INFO_PRINTF("[SERVO ZERO] M%02u success=%u phase=%s abs=%ld\r\n",
            (unsigned)channel, result == p4::servo::RestoreResult::Success ? 1U : 0U,
            writer->phase(), driver ? (long)driver->getAbsolutePosition(route.servoId) : 0L);
        rebase->pendingMask &= ~bit;
    }
    if (!rebase->pendingMask) finishServoRebaseZero(sharedData, rebase);
    return true;
}

struct ServoRuntimeContext {
    bool initialized;
    ParallelServoFeedbackCycle feedbackCycle; ///< 四条UART共享轮次的两阶段协调器。
    ServoFeedbackCycleMeta_t completedFeedback; ///< 最近完成轮次，发布时复制到三类快照。
    bool deferredTargetWake; ///< 读事务期间到达的新目标，收集完成后再发送。
    uint32_t lastEmergencyStopToken;
    uint32_t lastTrustToken;
    uint32_t lastServoRebaseZeroToken;
    uint32_t feedbackFreshnessGraceUntilMs; ///< Reset Zero 后暂停 stale 判定的截止时刻。
    uint32_t lastLoopStartMs;
    uint32_t queueDropTotal;
    uint32_t lastQueueDropLogMs;
    // 上线恢复不是持久化功能；核验多圈模式后才能让上位机 JSON 提交 logical ABS。
    ServoOnlineRecoveryContext onlineRecovery;
    ServoRebaseZeroContext rebaseZero; ///< 当前上位机 logical ABS 归零事务。
    p4::servo::AbsoluteRestore coordinateWriter; ///< 唯一0x0B执行器。
};

static ServoRuntimeContext g_servoRuntime;

/** 初始化舵机总线与恢复上下文；本函数不创建任务、不进入循环。 */
bool initServoBusService(SystemRuntimeContext* sharedData, uint32_t nowMs)
{
    if (!sharedData) {
        return false;
    }

    g_servoRuntime = ServoRuntimeContext{};
    beginServoBuses();
    servoDiscoveryManager().begin();
    publishServoRouteSnapshot(sharedData);
    g_servoRuntime.lastEmergencyStopToken =
        sharedAtomicLoad(&sharedData->servo_emergency_stop_token);
    g_servoRuntime.lastTrustToken =
        sharedAtomicLoad(&sharedData->trust_multi_turn_request_token);
    g_servoRuntime.lastServoRebaseZeroToken =
        sharedAtomicLoad(&sharedData->servo_rebase_zero_request_token);
    g_servoRuntime.onlineRecovery.activeChannel = 0xFF;
    FW_LOG_INFO_PRINTLN("[SERVO RECOVERY] waiting for desktop JSON restore");

    g_servoRuntime.initialized = true;
    return true;
}

/** 推进一个舵机服务周期；周期等待和运行时统计由 ServoBusTask 负责。 */
bool stepServoBusService(SystemRuntimeContext* sharedData,
                         const ServoServiceInputSnapshot& input,
                         const ServiceOutputPort& output,
                         const ServoBusStepRequest& request)
{
    if (!sharedData || !g_servoRuntime.initialized) {
        return false;
    }
    if (!servoSnapshotIsConsistent(input)) {
        return false;
    }
    const uint32_t nowMs = input.nowMs;
    if (servoMaintenanceBlocksCommands()) g_servoRuntime.feedbackCycle.abort();
    const bool maintenanceHeld=stepServoMaintenance(sharedData, nowMs);
    const uint32_t changedIds=takeServoMaintenanceChangedMask();
    if(changedIds) {
        g_servoRuntime.coordinateWriter.cancel();
        g_servoRuntime.onlineRecovery.activeChannel = 0xFF;
    }
    if (maintenanceHeld) return true;
    ServoRuntimeContext& runtime = g_servoRuntime;
    const uint32_t loopStartMs = nowMs;
    auto& lastEmergencyStopToken = runtime.lastEmergencyStopToken;
    auto& lastTrustToken = runtime.lastTrustToken;
    auto& lastServoRebaseZeroToken = runtime.lastServoRebaseZeroToken;
    auto& lastLoopStartMs = runtime.lastLoopStartMs;
    auto& queueDropTotal = runtime.queueDropTotal;
    auto& lastQueueDropLogMs = runtime.lastQueueDropLogMs;
    auto& onlineRecovery = runtime.onlineRecovery;
    auto& rebaseZero = runtime.rebaseZero;
    auto& coordinateWriter = runtime.coordinateWriter;
        // 事件快速路径不能改变固定反馈周期的 gap 统计口径。
        if (request.periodicDue && lastLoopStartMs != 0) {
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
        if (request.periodicDue) lastLoopStartMs = loopStartMs;
        const uint8_t currentSystemState =
            sharedAtomicLoad(&sharedData->system_state);
        const bool motionActive =
            currentSystemState == SYSTEM_STATE_RUN_DIRECT ||
            currentSystemState == SYSTEM_STATE_RUN_FEEDBACK ||
            currentSystemState == SYSTEM_STATE_ROPE_INIT_RUNNING;
        const uint32_t emergencyStopToken =
            sharedAtomicLoad(&sharedData->servo_emergency_stop_token);
        if (emergencyStopToken != lastEmergencyStopToken) {
            lastEmergencyStopToken = emergencyStopToken;
            runtime.feedbackCycle.abort();
            coordinateWriter.cancel();
            onlineRecovery.activeChannel = 0xFF;
            if (rebaseZero.active) {
                rebaseZero.failedMask |= rebaseZero.pendingMask;
                rebaseZero.pendingMask = 0;
                finishServoRebaseZero(sharedData, &rebaseZero);
            }
            emergencyTorqueOffMappedServos(sharedData);
            taskRuntimeMarkStopTorqueOffComplete();
        }

        if (request.periodicDue) {
            captureTrustMultiTurn(sharedData, &lastTrustToken);
            captureServoRebaseZero(sharedData, &lastServoRebaseZeroToken,
                                  &rebaseZero);
        }
        const bool targetWake =
            (request.wakeReasons & p4::runtime::wake::kServoTarget) != 0U;
        if (targetWake) runtime.deferredTargetWake = true;

        bool feedbackCycleCompleted = false;
        if (request.feedbackCollectDue) {
            feedbackCycleCompleted = runtime.feedbackCycle.collect(
                p4::platform::time::monotonicMicros(),
                &runtime.completedFeedback);
        }

        ServoTargetBatch_t batch;
        while (!runtime.feedbackCycle.active() &&
               (request.periodicDue || runtime.deferredTargetWake) &&
               p4::actuator::receiveLatest(sharedData, &batch)) {
            const uint8_t owner = sharedData->servo_target_owner;
            // 同步读可能让本次 step 跨越数毫秒；必须在真正消费批次时重新取时钟。
            // 使用 step 入口旧 nowMs 会把期间新生成的目标误判为 stale-target。
            const uint32_t applyBeginMs =
                p4::platform::time::monotonicMillis();
            const ServoBatchApplyResult applyResult =
                applyServoTargetBatch(sharedData, batch, applyBeginMs);
            reportHostControlHoldResult(sharedData, batch, applyResult);
            if (batch.directGeneration != 0) {
                commitDirectTargetResult(
                    sharedData, batch.directGeneration, batch.directToken,
                    applyResult.requestedMask, applyResult.appliedMask,
                    applyResult.rejectedMask);
            }
            const uint32_t applyElapsedMs =
                p4::platform::time::monotonicMillis() - applyBeginMs;
            if (applyElapsedMs > kServoApplyStallLogThresholdMs) {
                const uint32_t targetAgeMs =
                    p4::actuator::computeTargetAgeMs(
                        p4::platform::time::monotonicMillis(),
                        batch.timestamp);
                FW_LOG_WARN_PRINTF("[SERVO LOOP STALL] stage=apply_target elapsedMs=%lu source=%u owner=%u ageMs=%lu count=%u applied=%u\r\n",
                                   (unsigned long)applyElapsedMs,
                                   (unsigned)batch.source,
                                   (unsigned)owner,
                                   (unsigned long)targetAgeMs,
                                   (unsigned)batch.count,
                                   (unsigned)applyResult.appliedCount);
            }
            if (batch.count != 0 && applyResult.appliedCount == 0) {
                queueDropTotal++;
                const uint32_t sampleNowMs =
                    p4::platform::time::monotonicMillis();
                const uint32_t targetAgeMs =
                    p4::actuator::computeTargetAgeMs(sampleNowMs,
                                                      batch.timestamp);
                if ((uint32_t)(sampleNowMs - lastQueueDropLogMs) >=
                    kServoQueueDropLogIntervalMs) {
                    lastQueueDropLogMs = sampleNowMs;
                    FW_LOG_WARN_PRINTF("[SERVO QUEUE DROP] source=%u owner=%u ageMs=%lu count=%u droppedTotal=%lu state=%u\r\n",
                                       (unsigned)batch.source,
                                       (unsigned)owner,
                                       (unsigned long)targetAgeMs,
                                       (unsigned)batch.count,
                                       (unsigned long)queueDropTotal,
                                       (unsigned)sharedData->system_state);
                }
            }
        }
        if (!runtime.feedbackCycle.active() && runtime.deferredTargetWake) {
            // receiveLatest() 已取尽“最新值邮箱”；即使队列为空也消费本次通知，
            // 后续新目标会重新发送 kServoTarget，避免旧唤醒永久锁存。
            runtime.deferredTargetWake = false;
        }

        if (!feedbackCycleCompleted) {
            if (request.periodicDue && !runtime.feedbackCycle.active()) {
                uint8_t readIds[NUM_BUSES][MAX_SERVOS_PER_BUS] = {0};
                uint8_t readCounts[NUM_BUSES] = {0};
                for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
                    ServoRoute route;
                    if (servoRouteTable().resolveControllable(ch, &route)) {
                        addServoReadTarget(readIds, readCounts,
                                           route.busIndex, route.servoId);
                    }
                }
                runtime.feedbackCycle.begin(
                    readIds, readCounts,
                    p4::platform::time::monotonicMicros());
            }
            // Begin之后UART硬件自行接收；本任务等待约3 ms收集时点，不忙等。
            return true;
        }

        // 正常反馈读取拥有最高优先级。即使后台路由扫描持续进行，只要某个
        // 已映射舵机超过新鲜度上限未更新，也必须冻结锚点并撤销在线信任。
        // 必须在同步读结束后重新取时钟，不能复用周期入口的 nowMs；否则本轮
        // 刚更新的 lastUpdate 会略晚于 nowMs，无符号相减后被误判为超时。
        const uint32_t feedbackCheckNowMs =
            p4::platform::time::monotonicMillis();
        const ServoRouteSnapshot freshnessRoutes = servoRouteTable().snapshot();
        const bool freshnessGraceActive =
            (int32_t)(feedbackCheckNowMs -
                      runtime.feedbackFreshnessGraceUntilMs) < 0;
        for (uint8_t motor = 0;
             !freshnessGraceActive && motor < SERVO_TOTAL_NUM;
             ++motor) {
            const ServoRoute& route = freshnessRoutes.routes[motor];
            if (route.busIndex >= NUM_BUSES ||
                route.state != ServoRouteState::Online) {
                continue;
            }
            FeetechServoDriver* driver =
                servoBusRegistry().driver(route.busIndex);
            if (driver && driver->invalidateIfFeedbackStale(
                    route.servoId,
                    feedbackCheckNowMs,
                    mmhand::config::servo::kFeedbackStaleTimeoutMs)) {
                FW_LOG_WARN_PRINTF(
                    "[SERVO FEEDBACK STALE] M%02u BUS%u timeout=%lu ms -> offline\r\n",
                    (unsigned)motor,
                    (unsigned)route.busIndex,
                    (unsigned long)mmhand::config::servo::kFeedbackStaleTimeoutMs);
            }
        }
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
                const uint8_t offlineBus = route.busIndex;
                servoRouteTable().markMotorOffline(motor);
                servoDiscoveryManager().requestMotorScan(motor, offlineBus);
            }
        }
        publishServoRouteSnapshot(sharedData);

        // 同一执行器串行处理归零、主机恢复和热插拔；不允许互相抢占。
        bool rebaseZeroStepped = false;
        if (onlineRecovery.activeChannel < SERVO_TOTAL_NUM) {
            processServoOnlineTransitions(sharedData, &onlineRecovery);
        } else if (rebaseZero.active) {
            rebaseZeroStepped = stepServoRebaseZero(sharedData, &rebaseZero,
                &coordinateWriter);
        } else {
            processServoOnlineTransitions(sharedData, &onlineRecovery);
        }
        if (coordinateWriter.active()) {
            runtime.feedbackFreshnessGraceUntilMs = p4::platform::time::monotonicMillis() +
                mmhand::config::servo::kFeedbackMaintenanceGraceMs;
        }
        p4::servo::publishTurnOverflowState(sharedData);
        p4::servo::publishRecoveryState(sharedData);
        p4::servo::publishFeedback(sharedData, runtime.completedFeedback);
        // 路由扫描放在正常同步反馈、掉线失效和恢复事务之后。每轮 step()
        // 仍只探测一个 BUS/ID，但不再因扫描提前 return 而饿死在线反馈。
        if (!coordinateWriter.active() && !rebaseZeroStepped && !rebaseZero.active) {
            servoDiscoveryManager().step(loopStartMs, motionActive);
        }
        publishServoRouteSnapshot(sharedData);
    return true;
}

bool servoBusFeedbackCollectDue(uint64_t nowUs)
{
    return g_servoRuntime.initialized &&
           g_servoRuntime.feedbackCycle.collectDue(nowUs);
}

uint32_t servoBusFeedbackWaitMs(uint64_t nowUs)
{
    if (!g_servoRuntime.initialized) {
        return 0;
    }
    return g_servoRuntime.feedbackCycle.waitMs(nowUs);
}
