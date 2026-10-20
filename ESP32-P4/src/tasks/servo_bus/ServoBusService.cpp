/**
 * @file ServoBusService.cpp
 * @brief 调度四条舵机总线、应用目标批次并发布统一反馈与恢复状态。
 *
 * 本任务不再直接调用飞特 SDK。底层串口协议由 FeetechServoDriver 封装；
 * 这里负责“何时收发”和跨任务状态同步，后续还会继续拆出恢复管理器。
 */
#include "ServoBusService.h"
#include "ServoMaintenance.h"

#include <string.h>

#include "ServoDiscoveryManager.h"
#include "ServoFeedbackPublisher.h"
#include "ParallelServoFeedbackCycle.h"
#include "ServoRouteTable.h"
#include "ServoBusRouting.h"
#include "../host_comm/HostCalibrationCommandHandler.h"
#include "../../data/RecoveryTypes.h"
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

/** 上位机 JSON 恢复的逐台事务；只由 ServoBusTask 读写。 */
struct ServoHostRestoreContext {
    bool active; ///< 已冻结一个 token，尚有 Mxx 未处理。
    uint32_t token;
    uint32_t requestMask;
    uint32_t pendingMask;
    uint32_t restoredMask;
    uint32_t rejectedMask;
    int32_t logicalAbs[SERVO_TOTAL_NUM];
};

/** 标定 F0 / Direct F1 共用的实际 ABS=0 事务；完成前保留上次结果快照。 */
struct ServoRebaseZeroContext {
    bool active;
    uint32_t token;
    uint32_t requestMask;
    uint32_t pendingMask;
    uint32_t successMask;
    uint32_t failedMask;
};

/** RESET/坐标重建只能在没有运动 owner、控制关闭时执行。 */
static bool servoCoordinateMaintenanceSafe(SystemRuntimeContext* sharedData)
{
    if (!sharedData || sharedAtomicLoad(&sharedData->control_enabled) != 0 ||
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

/** 完成整批事务后一次性发布结果，不能让上位机看到中间态。 */
static void finishHostMultiTurnRestore(
    SystemRuntimeContext* sharedData,
    ServoHostRestoreContext* restore)
{
    sharedData->restore_multi_turn_restored_mask = restore->restoredMask;
    sharedData->restore_multi_turn_rejected_mask = restore->rejectedMask;
    sharedAtomicStore(&sharedData->restore_multi_turn_result_token,
                      restore->token);
    restore->active = false;
}

/** 冻结一笔上位机 JSON 请求；这里只复制数据，不访问舵机 UART。 */
static void captureHostMultiTurnRestore(
    SystemRuntimeContext* sharedData,
    uint32_t* lastToken,
    ServoHostRestoreContext* restore)
{
    if (!sharedData || !lastToken || !restore || restore->active) {
        return;
    }
    const uint32_t token =
        sharedAtomicLoad(&sharedData->restore_multi_turn_request_token);
    if (token == *lastToken) {
        return;
    }
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
    *lastToken = token;
    *restore = ServoHostRestoreContext{};
    restore->token = token;
    restore->requestMask = motorMask & validMotorChannelMask();
    restore->pendingMask = restore->requestMask;
    memcpy(restore->logicalAbs, motorAbs, sizeof(restore->logicalAbs));
    restore->active = true;

    if (!servoCoordinateMaintenanceSafe(sharedData)) {
        restore->rejectedMask = restore->requestMask;
        restore->pendingMask = 0;
        finishHostMultiTurnRestore(sharedData, restore);
    } else if (restore->requestMask == 0) {
        finishHostMultiTurnRestore(sharedData, restore);
    }
}


// P4 持续供电而舵机单独掉电时使用的 RAM 恢复锚点。
// 该状态与 Flash 开关独立：Flash OFF 只禁止跨 P4 掉电持久化，
// 不能丢弃本次 P4 上电会话内已经冻结的可信多圈位置。
struct ServoHotplugPendingSync {
    FeetechServoDriver* driver = nullptr;
    int32_t targetAbs = 0;
    uint32_t onlineSessionGeneration = 0;
    uint32_t trustedTimestampMs = 0;
    uint8_t servoId = 0;
    uint8_t failedRequests = 0;
    bool active = false;
    bool blocked = false;
};

struct ServoHotplugRecoveryAnchors {
    uint32_t validMask;                    // 已冻结可信锚点的舵机位图。
    int32_t logicalAbs[SERVO_TOTAL_NUM];    // 舵机掉线前最后可信的 ABS。
    int16_t raw[SERVO_TOTAL_NUM];           // 与 logicalAbs 同帧的单圈 raw。
    ServoHotplugPendingSync pending[SERVO_TOTAL_NUM]; // 应答未确认时固定首次置数目标。
};

// 热插拔恢复只使用本次 P4 会话内冻结的 RAM 锚点；跨 P4 掉电恢复由桌面 JSON 完成。
enum ServoRecoveryAnchorSource : uint8_t {
    SERVO_RECOVERY_ANCHOR_NONE = 0,
    SERVO_RECOVERY_ANCHOR_RAM_POWER_LOSS = 1,
};

/** 舵机上线恢复跨周期上下文；无论 Flash 是否启用都必须消费 pending。 */
struct ServoOnlineRecoveryContext {
    uint8_t activeChannel; ///< 当前跨周期恢复的 Mxx，0xFF 表示尚未选择。
    uint8_t nextChannel; ///< 下一轮查找 pending 舵机的起点，保证 27 路公平处理。
    bool torqueOffComplete; ///< 当前舵机是否已完成一次关扭矩。
};

/** 用同帧 ABS/raw 锚点计算最近圈目标；检查加法及硬件范围后才允许置数。 */
static bool servoHotplugTargetFromAnchor(int32_t anchorAbs,
                                         int16_t anchorRaw,
                                         int16_t currentRaw,
                                         int32_t* outTargetAbs)
{
    if (!outTargetAbs) return false;
    int32_t delta = (int32_t)normalizeServoRawSingleTurn(currentRaw) -
        (int32_t)normalizeServoRawSingleTurn(anchorRaw);
    if (delta > 2047) delta -= 4096;
    if (delta < -2048) delta += 4096;
    const int64_t target = (int64_t)anchorAbs + (int64_t)delta;
    if (target < kServoHardwareCommandMin || target > kServoHardwareCommandMax) {
        return false;
    }
    *outTargetAbs = (int32_t)target;
    return true;
}

// 为重新上线的单台舵机选择最新可信锚点。刚冻结的 RAM 锚点优先于
// 桌面 JSON 恢复，避免舵机恢复速度快于 JSON 同步时出现 anchor=0。
static bool resolveServoRecoveryAnchor(
    SystemRuntimeContext* sharedData,
    const ServoHotplugRecoveryAnchors* hotplugAnchors,
    uint8_t channel,
    int32_t* logicalAbs,
    int16_t* raw,
    ServoRecoveryAnchorSource* source)
{
    if (!sharedData || !hotplugAnchors ||
        !logicalAbs || !raw || !source || channel >= SERVO_TOTAL_NUM) {
        return false;
    }

    const uint32_t bit = 1UL << channel;
    if ((hotplugAnchors->validMask & bit) != 0) {
        *logicalAbs = hotplugAnchors->logicalAbs[channel];
        *raw = hotplugAnchors->raw[channel];
        *source = SERVO_RECOVERY_ANCHOR_RAM_POWER_LOSS;
        return true;
    }

    (void)sharedData;
    return false;
}

/** 处理离线到在线事务；多圈核验/配置按阶段推进，每周期最多执行一笔恢复写事务。 */
static void processServoOnlineTransitions(
    SystemRuntimeContext* sharedData,
    ServoHotplugRecoveryAnchors* hotplugAnchors,
    ServoOnlineRecoveryContext* recovery,
    uint32_t* sessionRebasedMask,
    bool* hardwareResetAttempted)
{
    if (!sharedData || !hotplugAnchors || !recovery ||
        !sessionRebasedMask || !hardwareResetAttempted) {
        return;
    }
    *hardwareResetAttempted = false;
    const uint32_t nowMs = p4::platform::time::monotonicMillis();

    // 实际置数可能已生效而 ACK 丢失。保持同一次上线的首次目标；新可信
    // 坐标可以结束该事务，路由或上线代次变化则必须拒绝继续使用旧 raw。
    for (uint8_t channel = 0; channel < SERVO_TOTAL_NUM; ++channel) {
        ServoHotplugPendingSync& pending = hotplugAnchors->pending[channel];
        if (!pending.active) continue;
        ServoRoute pendingRoute;
        FeetechServoDriver* pendingBus =
            resolveMotorDriver(channel, &pendingRoute, true);
        if (!pendingBus || pendingBus != pending.driver ||
            pendingRoute.servoId != pending.servoId) {
            pending.blocked = true;
            continue;
        }
        const ServoFeedbackInternal& feedback =
            pendingBus->getFeedback(pendingRoute.servoId);
        if (pendingBus->isAbsoluteReliable(pendingRoute.servoId) ||
            feedback.offlineLastTrustedTimestampMs != pending.trustedTimestampMs) {
            pending = ServoHotplugPendingSync{};
        } else if (!pendingBus->isOnline(pendingRoute.servoId) ||
                   feedback.onlineSessionGeneration != pending.onlineSessionGeneration) {
            pending.blocked = true;
        }
    }

    if (recovery->activeChannel >= SERVO_TOTAL_NUM) {
        for (uint8_t offset = 0; offset < SERVO_TOTAL_NUM; ++offset) {
            const uint8_t candidate = (uint8_t)(
                (recovery->nextChannel + offset) % SERVO_TOTAL_NUM);
            ServoRoute candidateRoute;
            FeetechServoDriver* candidateBus =
                resolveMotorDriver(candidate, &candidateRoute, true);
            if (candidateBus &&
                candidateBus->isRecoveryAttemptDue(candidateRoute.servoId,
                                                    nowMs)) {
                recovery->activeChannel = candidate;
                recovery->nextChannel =
                    (uint8_t)((candidate + 1U) % SERVO_TOTAL_NUM);
                recovery->torqueOffComplete = false;
                break;
            }
        }
    }
    if (recovery->activeChannel >= SERVO_TOTAL_NUM) {
        return;
    }

    const uint8_t ch = recovery->activeChannel;
    const uint32_t bit = 1UL << ch;
    ServoRoute route;
    FeetechServoDriver* bus = resolveMotorDriver(ch, &route, true);
    if (!bus || !bus->isRecoveryAttemptDue(route.servoId, nowMs)) {
        recovery->activeChannel = 0xFF;
        recovery->torqueOffComplete = false;
        return;
    }
    const uint8_t id = route.servoId;

    ServoHotplugPendingSync& pending = hotplugAnchors->pending[ch];
    if (pending.active &&
        (pending.blocked || pending.failedRequests >= kRecoveryMaxAttempts)) {
        sharedAtomicFetchOr(&sharedData->multi_turn_unknown_bitmap, bit);
        bus->finishRecoveryAttempt(id, false, nowMs);
        FW_LOG_WARN_PRINTF(
            "[SERVO HOTPLUG] M%02u bus=%u id=%u pending_abs=%ld result=failed reason=unresolved-sync\r\n",
            (unsigned)ch, (unsigned)route.busIndex, (unsigned)id,
            (long)pending.targetAbs);
        recovery->activeChannel = 0xFF;
        recovery->torqueOffComplete = false;
        return;
    }

    // 先关闭力矩，再核验多圈配置；位置校正入口仍独立检查关力矩应答。
    if (!recovery->torqueOffComplete) {
        if (!bus->setTorqueEnabled(id, false)) {
            bus->finishRecoveryAttempt(id, false, nowMs);
            recovery->activeChannel = 0xFF;
            return;
        }
        recovery->torqueOffComplete = true;
        return;
    }

    bool configured = bus->isMultiTurnConfigured(id);
    if (!configured) {
        const ServoMultiTurnConfigResult configResult =
            bus->stepMultiTurnConfiguration(id, nowMs);
        if (configResult == ServoMultiTurnConfigResult::Pending) {
            return;
        }
        if (configResult == ServoMultiTurnConfigResult::Failed) {
            bus->finishRecoveryAttempt(id, false, nowMs);
            FW_LOG_WARN_PRINTF(
                "[SERVO HOTPLUG] M%02u bus=%u id=%u multi-turn config failed\r\n",
                (unsigned)ch,
                (unsigned)route.busIndex,
                (unsigned)id);
            recovery->activeChannel = 0xFF;
            recovery->torqueOffComplete = false;
            return;
        }
        configured = true;
    }

    bool trusted = bus->isAbsoluteReliable(id);
    int32_t recoveryLogicalAbs = 0;
    int16_t recoveryRaw = 0;
    ServoRecoveryAnchorSource anchorSource = SERVO_RECOVERY_ANCHOR_NONE;
    const bool hasAnchor = resolveServoRecoveryAnchor(sharedData,
                                                       hotplugAnchors,
                                                       ch,
                                                       &recoveryLogicalAbs,
                                                       &recoveryRaw,
                                                       &anchorSource);
    const uint8_t attemptNumber =
        (uint8_t)(bus->recoveryAttemptCount(id) + 1U);
    uint8_t correctionAttempts = 0;
    if (!trusted && configured && hasAnchor) {
        int32_t targetAbs = pending.targetAbs;
        const bool targetValid = pending.active || servoHotplugTargetFromAnchor(
            recoveryLogicalAbs, recoveryRaw, bus->getRawPosition(id), &targetAbs);
        if (targetValid) {
            const uint32_t sessionGeneration =
                bus->getFeedback(id).onlineSessionGeneration;
            const uint32_t trustedTimestamp =
                bus->getFeedback(id).offlineLastTrustedTimestampMs;
            bool correctionAttempted = false;
            int16_t correctedRaw = 0;
            *hardwareResetAttempted = true;
            trusted = bus->calibrateHardwareAbsolutePosition(
                id, targetAbs, &correctedRaw, &correctionAttempts,
                &correctionAttempted);
            if (!trusted && correctionAttempted) {
                if (!pending.active) {
                    pending.driver = bus;
                    pending.servoId = id;
                    pending.targetAbs = targetAbs;
                    pending.onlineSessionGeneration = sessionGeneration;
                    pending.trustedTimestampMs = trustedTimestamp;
                    pending.active = true;
                }
                ++pending.failedRequests;
                if (!bus->isOnline(id) ||
                    bus->getFeedback(id).onlineSessionGeneration != sessionGeneration) {
                    pending.blocked = true;
                }
            }
        } else {
            FW_LOG_WARN_PRINTF(
                "[SERVO HOTPLUG] M%02u anchor_abs=%ld anchor_raw=%d raw=%d result=failed reason=abs-range\r\n",
                (unsigned)ch, (long)recoveryLogicalAbs, (int)recoveryRaw,
                (int)bus->getRawPosition(id));
        }
        configured = bus->isMultiTurnConfigured(id);
    }

    const bool recoveryTransactionComplete = configured &&
        (trusted || !hasAnchor);
    bus->finishRecoveryAttempt(id, recoveryTransactionComplete, nowMs);
    if (configured) {
        if (trusted) {
            sharedAtomicFetchAnd(&sharedData->multi_turn_unknown_bitmap, ~bit);
            hotplugAnchors->validMask &= ~bit;
            pending = ServoHotplugPendingSync{};
            *sessionRebasedMask |= bit;
        } else {
            // 没有可信锚点时保持 Turn Unknown，禁止把“多圈模式正确”误当作“圈数可信”。
            sharedAtomicFetchOr(&sharedData->multi_turn_unknown_bitmap, bit);
        }
    }
    FW_LOG_INFO_PRINTF("[SERVO HOTPLUG] M%02u bus=%u id=%u instruction=0x0B attempt=%u/%u correction_attempts=%u "
                  "configured=%u anchor=%u anchor_src=%u trusted=%u ready=%u "
                  "failed=%u raw=%d unknown=0x%08lX\r\n",
                  (unsigned)ch,
                  (unsigned)route.busIndex,
                  (unsigned)id,
                  (unsigned)attemptNumber,
                  (unsigned)mmhand::config::servo::kRecoveryMaxAttempts,
                  (unsigned)correctionAttempts,
                  configured ? 1U : 0U,
                  hasAnchor ? 1U : 0U,
                  (unsigned)anchorSource,
                  trusted ? 1U : 0U,
                  configured ? 1U : 0U,
                  bus->isRecoveryFailed(id) ? 1U : 0U,
                  (int)bus->getRawPosition(id),
                  (unsigned long)sharedAtomicLoad(
                      &sharedData->multi_turn_unknown_bitmap));
    recovery->activeChannel = 0xFF;
    recovery->torqueOffComplete = false;
}

// 收集各总线刚刚冻结的掉电前可信位置。RAM 锚点仅覆盖本次 P4 上电会话；
// P4 与舵机同时掉电后由上位机 JSON 锚点恢复。
static uint32_t cacheServoPowerLossSnapshots(
    SystemRuntimeContext* sharedData,
    ServoHotplugRecoveryAnchors* hotplugAnchors)
{
    if (!sharedData || !hotplugAnchors) {
        return 0;
    }
    uint32_t capturedMask = 0;

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
        hotplugAnchors->pending[ch] = ServoHotplugPendingSync{};
        hotplugAnchors->validMask |= bit;
        capturedMask |= bit;
        FW_LOG_WARN_PRINTF(
            "[SERVO HOTPLUG ANCHOR] M%02u logical=%ld raw=%d turn=%d ageMs=%lu\r\n",
            (unsigned)ch,
            (long)logicalAbs,
            (int)raw,
            (int)turn,
            (unsigned long)(p4::platform::time::monotonicMillis() - sampleTimestampMs));

    }
    return capturedMask;
}

/**
 * 每个完整反馈轮次最多恢复一台舵机，避免27路位置校正连续阻塞实时反馈。
 * 接收既有 0xDA 请求后，以 0x0B 将主机 ABS 实际写入舵机并读回核验。
 */
static bool stepHostMultiTurnRestore(
    SystemRuntimeContext* sharedData,
    ServoHostRestoreContext* restore,
    uint32_t* sessionRebasedMask,
    ServoHotplugRecoveryAnchors* hotplugAnchors)
{
    if (!sharedData || !restore || !sessionRebasedMask || !hotplugAnchors || !restore->active) {
        return false;
    }
    if (!servoCoordinateMaintenanceSafe(sharedData)) {
        restore->rejectedMask |= restore->pendingMask;
        restore->pendingMask = 0;
        finishHostMultiTurnRestore(sharedData, restore);
        return false;
    }

    uint8_t channel = SERVO_TOTAL_NUM;
    for (uint8_t i = 0; i < SERVO_TOTAL_NUM; ++i) {
        if ((restore->pendingMask & (1UL << i)) != 0) {
            channel = i;
            break;
        }
    }
    if (channel >= SERVO_TOTAL_NUM) {
        finishHostMultiTurnRestore(sharedData, restore);
        return false;
    }

    const uint32_t bit = 1UL << channel;
    uint32_t restoredBit = 0;
    uint32_t rejectedBit = 0;
    if (hotplugAnchors->pending[channel].active) {
        // 热插拔置数应答未确认时，JSON/raw 可能已漂移；不能由主机恢复绕过该锁。
        rejectedBit = bit;
        FW_LOG_WARN_PRINTF(
            "[SERVO RESTORE] M%02u result=failed reason=unresolved-hotplug-sync\r\n",
            (unsigned)channel);
    } else {
        // 主机接管坐标恢复，撤销旧 RAM 锚点，防止失败后另一恢复路径回写旧目标。
        hotplugAnchors->validMask &= ~bit;
        const uint32_t resetMask = (*sessionRebasedMask & bit) == 0 ? bit : 0;
        restoreServoMultiTurnFromHost(sharedData,
                                      bit,
                                      restore->logicalAbs,
                                      resetMask,
                                      true,
                                      &restoredBit,
                                      &rejectedBit);
    }
    restore->pendingMask &= ~bit;
    restore->restoredMask |= restoredBit;
    restore->rejectedMask |= rejectedBit;
    if (restoredBit != 0) {
        *sessionRebasedMask |= bit;
    }
    if (restore->pendingMask == 0) {
        finishHostMultiTurnRestore(sharedData, restore);
    }
    return true;
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
                                ServoHotplugRecoveryAnchors* hotplugAnchors,
                                ServoOnlineRecoveryContext* recovery,
                                uint32_t* sessionRebasedMask)
{
    if (!rebase->active) return false;
    if (!servoCoordinateMaintenanceSafe(sharedData)) {
        // 未尝试的电机保持原坐标；本轮报告为拒绝，不把它们伪装成归零成功。
        rebase->failedMask |= rebase->pendingMask;
        rebase->pendingMask = 0;
        finishServoRebaseZero(sharedData, rebase);
        return false;
    }
    uint8_t channel = 0;
    while (channel < SERVO_TOTAL_NUM &&
           (rebase->pendingMask & (1UL << channel)) == 0) {
        ++channel;
    }
    if (channel >= SERVO_TOTAL_NUM) {
        finishServoRebaseZero(sharedData, rebase);
        return false;
    }
    const uint32_t bit = 1UL << channel;
    ServoRoute route;
    FeetechServoDriver* driver = resolveMotorDriver(channel, &route, true);

    // 即使 0x0B 校正最终失败，也可能已经改写硬件 ABS。先撤销旧 LUT/限位
    // 和掉电锚点，杜绝半成功事务继续使用旧逻辑坐标。
    invalidateAutomaticCalibration(sharedData, jointMaskForMotorMask(bit));
    hotplugAnchors->validMask &= ~bit;
    hotplugAnchors->pending[channel] = ServoHotplugPendingSync{};
    *sessionRebasedMask &= ~bit;
    if (recovery->activeChannel == channel) {
        recovery->activeChannel = 0xFF;
        recovery->torqueOffComplete = false;
    }
    sharedAtomicFetchOr(&sharedData->multi_turn_unknown_bitmap, bit);
    int16_t resetRaw = 0;
    uint8_t attempts = 0;
    const bool ok = driver && driver->calibrateHardwareAbsolutePosition(
        route.servoId, 0, &resetRaw, &attempts);
    if (driver) {
        // 此次显式重定义硬件坐标后不能再恢复校正前的旧锚点。
        int32_t discardedAbs = 0;
        int16_t discardedRaw = 0;
        int16_t discardedTurn = 0;
        uint32_t discardedTimestamp = 0;
        driver->consumeOfflineTrustedSnapshot(
            route.servoId, &discardedAbs, &discardedRaw,
            &discardedTurn, &discardedTimestamp);
    }
    if (ok) {
        rebase->successMask |= bit;
        *sessionRebasedMask |= bit;
        sharedAtomicFetchAnd(&sharedData->multi_turn_unknown_bitmap, ~bit);
        sharedAtomicFetchAnd(&sharedData->servo_turn_overflow_bitmap, ~bit);
    } else {
        rebase->failedMask |= bit;
        sharedAtomicFetchOr(&sharedData->multi_turn_unknown_bitmap, bit);
    }
    rebase->pendingMask &= ~bit;
    FW_LOG_INFO_PRINTF(
        "[SERVO REBASE ZERO] M%02u token=%lu raw=%d attempts=%u success=%u instruction=0x0B\r\n",
        (unsigned)channel, (unsigned long)rebase->token,
        (int)resetRaw,
        (unsigned)attempts, ok ? 1U : 0U);
    if (rebase->pendingMask == 0) {
        finishServoRebaseZero(sharedData, rebase);
    }
    return true;
}

/**
 * 执行 Reset Zero。
 *
 * 硬件 turn0 成功后立即提交 RAM 状态与结果 token；上位机收到结果后会同步
 * 失效对应 Auto/LUT，并把新的 raw/ABS 锚点写入 JSON；处理完成不依赖后台持久化任务。
 */
static void processServoResetZeroRequest(SystemRuntimeContext* sharedData,
                                         uint32_t* lastRequestToken,
                                         ServoHotplugRecoveryAnchors* hotplugAnchors,
                                         ServoOnlineRecoveryContext* recovery,
                                         uint32_t* sessionRebasedMask)
{
    if (!sharedData || !lastRequestToken || !hotplugAnchors ||
        !recovery || !sessionRebasedMask) {
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
    const uint32_t executableMask =
        requestedMask & p4::servo::collectOnlineMotorMask();
    uint32_t successMask = 0;
    uint32_t failedMask = requestedMask & ~executableMask;

    // Reset Zero 的最小安全门禁：控制必须关闭、没有 owner，且系统不在运动态。
    const uint8_t systemState = sharedData->system_state;
    const bool resetStateSafe =
        sharedData->control_enabled == 0 &&
        sharedData->servo_target_owner == SERVO_TARGET_OWNER_NONE &&
        (systemState == SYSTEM_STATE_UNCONTROLLABLE ||
         systemState == SYSTEM_STATE_IDLE ||
         systemState == SYSTEM_STATE_READY ||
         systemState == SYSTEM_STATE_RECOVERY_REQUIRED);
    if (token != 0 && executableMask != 0 && resetStateSafe) {
        for (uint8_t channel = 0; channel < SERVO_TOTAL_NUM; ++channel) {
            const uint32_t bit = 1UL << channel;
            if ((executableMask & bit) == 0) {
                continue;
            }
            ServoRoute route;
            FeetechServoDriver* driver =
                resolveMotorDriver(channel, &route, true);
            int16_t raw = 0;
            uint8_t attempts = 0;
            bool resetOk = false;
            bool anyResetAttempted = false;
            while (driver &&
                   attempts <
                       mmhand::config::servo::kResetZeroTransactionAttempts) {
                ++attempts;
                bool resetAttempted = false;
                resetOk = driver->resetTurnCounter(
                    route.servoId, &raw, &resetAttempted);
                anyResetAttempted |= resetAttempted;
                if (resetOk) {
                    break;
                }
            }
            if (anyResetAttempted) {
                hotplugAnchors->validMask &= ~bit;
                hotplugAnchors->pending[channel] = ServoHotplugPendingSync{};
                *sessionRebasedMask &= ~bit;
                if (recovery->activeChannel == channel) {
                    recovery->activeChannel = 0xFF;
                    recovery->torqueOffComplete = false;
                }
            }
            if (!resetOk) {
                failedMask |= bit;
                if (anyResetAttempted) {
                    sharedAtomicFetchOr(&sharedData->multi_turn_unknown_bitmap, bit);
                }
                continue;
            }
            successMask |= bit;
            FW_LOG_WARN_PRINTF(
                "[SERVO RESET ZERO] RAM-only M%02u raw=%d attempts=%u token=%lu\r\n",
                (unsigned)channel,
                (int)raw,
                (unsigned)attempts,
                (unsigned long)token);
        }
    } else {
        failedMask |= requestedMask;
    }

    if (successMask != 0) {
        sharedAtomicFetchAnd(&sharedData->multi_turn_unknown_bitmap,
                             ~successMask);
        sharedAtomicFetchAnd(&sharedData->servo_turn_overflow_bitmap,
                             ~successMask);
    }
    sharedAtomicStore(&sharedData->servo_reset_zero_success_mask,
                      successMask);
    sharedAtomicStore(&sharedData->servo_reset_zero_failed_mask,
                      failedMask);
    sharedAtomicStore(
        &sharedData->servo_reset_zero_status,
        (uint8_t)((successMask != 0 && failedMask == 0)
                      ? SERVO_RESET_ZERO_STATUS_SUCCESS
                      : SERVO_RESET_ZERO_STATUS_FAILED));
    sharedAtomicStore(&sharedData->servo_reset_zero_result_token, token);
}
/**
 * ServoBusTask 独占的跨周期运行上下文。把路由、恢复与反馈节奏集中保存，
 * 业务 step 不再依赖隐藏在无限循环栈帧中的局部变量。
 */
struct ServoRuntimeContext {
    bool initialized;
    ParallelServoFeedbackCycle feedbackCycle; ///< 四条UART共享轮次的两阶段协调器。
    ServoFeedbackCycleMeta_t completedFeedback; ///< 最近完成轮次，发布时复制到三类快照。
    bool deferredTargetWake; ///< 读事务期间到达的新目标，收集完成后再发送。
    uint32_t lastEmergencyStopToken;
    uint32_t lastRestoreToken;
    uint32_t lastServoResetZeroToken;
    uint32_t lastServoRebaseZeroToken;
    uint32_t feedbackFreshnessGraceUntilMs; ///< Reset Zero 后暂停 stale 判定的截止时刻。
    uint32_t lastLoopStartMs;
    uint32_t queueDropTotal;
    uint32_t lastQueueDropLogMs;
    // 上线恢复不是持久化功能；核验多圈模式后才能让上位机 JSON 提交 logical ABS。
    ServoHotplugRecoveryAnchors hotplugRecoveryAnchors;
    ServoOnlineRecoveryContext onlineRecovery;
    ServoHostRestoreContext hostRestore; ///< 当前上位机 JSON 逐台恢复事务。
    ServoRebaseZeroContext rebaseZero; ///< 当前上位机 logical ABS 归零事务。
    uint32_t sessionRebasedMask; ///< 本次 P4 会话已完成硬件坐标重建的 Mxx。
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
    g_servoRuntime.lastRestoreToken =
        sharedAtomicLoad(&sharedData->restore_multi_turn_request_token);
    g_servoRuntime.lastServoResetZeroToken =
        sharedAtomicLoad(&sharedData->servo_reset_zero_request_token);
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
        g_servoRuntime.hotplugRecoveryAnchors.validMask &= ~changedIds;
        g_servoRuntime.sessionRebasedMask &= ~changedIds;
        for(uint8_t motor=0;motor<SERVO_TOTAL_NUM;++motor) if(changedIds & (1UL<<motor)) {
            g_servoRuntime.hotplugRecoveryAnchors.pending[motor]=ServoHotplugPendingSync{};
            if(g_servoRuntime.onlineRecovery.activeChannel==motor) {
                g_servoRuntime.onlineRecovery.activeChannel=0xFF;
                g_servoRuntime.onlineRecovery.torqueOffComplete=false;
            }
        }
    }
    if (maintenanceHeld) return true;
    ServoRuntimeContext& runtime = g_servoRuntime;
    const uint32_t loopStartMs = nowMs;
    auto& lastEmergencyStopToken = runtime.lastEmergencyStopToken;
    auto& lastRestoreToken = runtime.lastRestoreToken;
    auto& lastServoResetZeroToken = runtime.lastServoResetZeroToken;
    auto& lastServoRebaseZeroToken = runtime.lastServoRebaseZeroToken;
    auto& lastLoopStartMs = runtime.lastLoopStartMs;
    auto& queueDropTotal = runtime.queueDropTotal;
    auto& lastQueueDropLogMs = runtime.lastQueueDropLogMs;
    auto& hotplugRecoveryAnchors = runtime.hotplugRecoveryAnchors;
    auto& onlineRecovery = runtime.onlineRecovery;
    auto& hostRestore = runtime.hostRestore;
    auto& rebaseZero = runtime.rebaseZero;
    auto& sessionRebasedMask = runtime.sessionRebasedMask;
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
            emergencyTorqueOffMappedServos(sharedData);
            taskRuntimeMarkStopTorqueOffComplete();
        }

        if (request.periodicDue) {
            captureHostMultiTurnRestore(sharedData,
                                        &lastRestoreToken,
                                        &hostRestore);
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
        // 必须在撤销动态路由之前消费驱动冻结的最后可信反馈；否则路由失效后
        // 无法再定位原 BUS。这里只保留本次 P4 会话的 RAM 锚点。
        const uint32_t newlyOfflineMask =
            cacheServoPowerLossSnapshots(sharedData,
                                         &hotplugRecoveryAnchors);
        // 同一舵机发生新的掉线边沿后，下一次 RAM/JSON 恢复必须重新执行 RESET。
        sessionRebasedMask &= ~newlyOfflineMask;
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
                const uint8_t offlineBus = route.busIndex;
                servoRouteTable().markMotorOffline(motor);
                servoDiscoveryManager().requestMotorScan(motor, offlineBus);
            }
        }
        publishServoRouteSnapshot(sharedData);

        const bool rebaseZeroStepped =
            stepServoRebaseZero(sharedData, &rebaseZero,
                               &hotplugRecoveryAnchors, &onlineRecovery,
                               &sessionRebasedMask);
        const bool hostRestoreStepped = !rebaseZeroStepped &&
            !rebaseZero.active &&
            stepHostMultiTurnRestore(sharedData,
                                     &hostRestore,
                                     &sessionRebasedMask,
                                     &hotplugRecoveryAnchors);
        if (hostRestoreStepped || rebaseZeroStepped) {
            runtime.feedbackFreshnessGraceUntilMs =
                p4::platform::time::monotonicMillis() +
                mmhand::config::servo::kFeedbackMaintenanceGraceMs;
        } else if (!rebaseZero.active) {
            bool hotplugResetAttempted = false;
            processServoOnlineTransitions(
                sharedData,
                &hotplugRecoveryAnchors,
                &onlineRecovery,
                &sessionRebasedMask,
                &hotplugResetAttempted);
            if (hotplugResetAttempted) {
                runtime.feedbackFreshnessGraceUntilMs =
                    p4::platform::time::monotonicMillis() +
                    mmhand::config::servo::kFeedbackMaintenanceGraceMs;
            }
        }
        p4::servo::publishTurnOverflowState(sharedData);
        const uint32_t resetZeroTokenBefore = lastServoResetZeroToken;
        if (!hostRestoreStepped && !rebaseZeroStepped && !rebaseZero.active) {
            processServoResetZeroRequest(sharedData, &lastServoResetZeroToken,
                                         &hotplugRecoveryAnchors, &onlineRecovery,
                                         &sessionRebasedMask);
        }
        if (lastServoResetZeroToken != resetZeroTokenBefore) {
            // Reset Zero 会暂时阻塞本任务；从事务真正结束的时刻重新计算宽限期。
            // 宽限期只跳过反馈陈旧判定，online/RESET 结果和真实读失败仍照常处理。
            runtime.feedbackFreshnessGraceUntilMs =
                p4::platform::time::monotonicMillis() +
                mmhand::config::servo::kFeedbackMaintenanceGraceMs;
        }
        p4::servo::publishRecoveryState(sharedData);
        p4::servo::publishFeedback(sharedData, runtime.completedFeedback);
        // 路由扫描放在正常同步反馈、掉线失效和恢复事务之后。每轮 step()
        // 仍只探测一个 BUS/ID，但不再因扫描提前 return 而饿死在线反馈。
        if (!hostRestoreStepped && !rebaseZeroStepped && !rebaseZero.active) {
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
