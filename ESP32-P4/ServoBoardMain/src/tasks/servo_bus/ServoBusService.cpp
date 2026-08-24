/**
 * @file ServoBusService.cpp
 * @brief 调度四条舵机总线、应用目标批次并发布统一反馈与恢复状态。
 *
 * 本任务不再直接调用飞特 SDK。底层串口协议由 FeetechServoDriver 封装；
 * 这里负责“何时收发”和跨任务状态同步，后续还会继续拆出恢复管理器。
 */
#include "ServoBusService.h"

#include <string.h>

#include "ServoDiscoveryManager.h"
#include "ServoFeedbackPublisher.h"
#include "ParallelServoFeedbackCycle.h"
#include "ServoRouteTable.h"
#include "ServoBusRouting.h"
#include "../../data/RecoveryTypes.h"
#include "../../modules/actuator/ActuatorCommandGateway.h"
#include "../../modules/runtime/ServiceCyclePolicy.h"
#include "../../config/topology/HardwareMap.h"
#include "../../config/DiagnosticsConfig.h"
#include "../../config/SafetyConfig.h"
#include "../../config/ServoConfig.h"
#include "../../platform/rtos/SharedSafetyState.h"
#include "../../platform/rtos/TaskSignal.h"
#include "../../platform/time/Clock.h"
#include "../../platform/log/FirmwareLog.h"
#include "../../runtime/SystemRuntimeContext.h"
#include "../../runtime/ports/SystemEventPort.h"
#include "../../runtime/metrics/TaskRuntimeMetrics.h"
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

    // 第一遍只解析和裁决，不改驱动缓存。这样第二台失败时第一台不会提前留下目标。
    for (uint8_t i = 0; i < batch.count && i < SERVO_TARGET_BATCH_MAX; i++) {
        const ServoTargetCommand_t& cmd = batch.commands[i];
        FeetechServoDriver* bus =
            resolveMotorDriver(cmd.motorChannel, &commandRoutes[i], true);
        const int32_t logicalOffset = bus
                                          ? bus->getLogicalOffset(commandRoutes[i].servoId)
                                          : 0;
        commandDecisions[i] =
            p4::actuator::evaluateCommand(batch,
                                          i,
                                          logicalOffset,
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

/** 标定事务只在整批同步写完成后回写 APPLIED，结果 token 最后发布。 */
static void reportCalibrationTargetResult(
    SystemRuntimeContext* sharedData,
    const ServoTargetBatch_t& batch,
    const ServoBatchApplyResult& result)
{
    if (!sharedData ||
        batch.purpose != SERVO_TARGET_PURPOSE_CALIBRATION_CONFIRMED) {
        return;
    }
    sharedAtomicStore(&sharedData->calibration_target_requested_mask,
                      result.requestedMask);
    sharedAtomicStore(&sharedData->calibration_target_applied_mask,
                      result.appliedMask);
    sharedAtomicStore(&sharedData->calibration_target_rejected_mask,
                      result.rejectedMask);
    sharedAtomicStore(&sharedData->calibration_target_apply_status,
                      (uint8_t)(result.appliedMask == result.requestedMask &&
                                        result.requestedMask != 0
                                    ? CALIBRATION_TARGET_APPLY_APPLIED
                                    : CALIBRATION_TARGET_APPLY_REJECTED));
    sharedAtomicStore(&sharedData->calibration_target_result_token,
                      batch.transactionToken);
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

/** 消费上位机 JSON 多圈恢复请求，并用 token 发布结果。 */
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
                                  true,
                                  &restoredMask,
                                  &rejectedMask);
    sharedData->restore_multi_turn_restored_mask = restoredMask;
    sharedData->restore_multi_turn_rejected_mask = rejectedMask;
    sharedAtomicStore(&sharedData->restore_multi_turn_result_token, token);
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

// P4 持续供电而舵机单独掉电时使用的 RAM 恢复锚点。
// 该状态与 Flash 开关独立：Flash OFF 只禁止跨 P4 掉电持久化，
// 不能丢弃本次 P4 上电会话内已经冻结的可信多圈位置。
struct ServoHotplugRecoveryAnchors {
    uint32_t validMask;                    // 已冻结可信锚点的舵机位图。
    int32_t logicalAbs[SERVO_TOTAL_NUM];    // 舵机掉线前最后可信的 P4 逻辑 ABS。
    int16_t raw[SERVO_TOTAL_NUM];           // 与 logicalAbs 同帧的单圈 raw。
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
    ServoOnlineRecoveryContext* recovery)
{
    if (!sharedData || !hotplugAnchors || !recovery) {
        return;
    }
    const uint32_t nowMs = p4::platform::time::monotonicMillis();

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

    // 恢复事务先关闭力矩；后续寄存器核验、EEPROM 配置与 offset 恢复都不重复执行该写操作。
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
    if (!trusted && configured && hasAnchor) {
        trusted = bus->restoreAbsolutePositionNearAnchor(
            id, recoveryLogicalAbs, recoveryRaw);
    }

    const uint8_t attemptNumber =
        (uint8_t)(bus->recoveryAttemptCount(id) + 1U);
    const bool recoveryTransactionComplete = configured &&
        (trusted || !hasAnchor);
    bus->finishRecoveryAttempt(id, recoveryTransactionComplete, nowMs);
    if (configured) {
        if (trusted) {
            sharedAtomicFetchAnd(&sharedData->multi_turn_unknown_bitmap, ~bit);
            hotplugAnchors->validMask &= ~bit;
        } else {
            // 没有可信锚点时保持 Turn Unknown，禁止把“多圈模式正确”误当作“圈数可信”。
            sharedAtomicFetchOr(&sharedData->multi_turn_unknown_bitmap, bit);
        }
    }
    FW_LOG_INFO_PRINTF("[SERVO HOTPLUG] M%02u bus=%u id=%u attempt=%u/%u "
                  "configured=%u anchor=%u anchor_src=%u trusted=%u ready=%u "
                  "failed=%u raw=%d unknown=0x%08lX\r\n",
                  (unsigned)ch,
                  (unsigned)route.busIndex,
                  (unsigned)id,
                  (unsigned)attemptNumber,
                  (unsigned)mmhand::config::servo::kRecoveryMaxAttempts,
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
static void cacheServoPowerLossSnapshots(
    SystemRuntimeContext* sharedData,
    ServoHotplugRecoveryAnchors* hotplugAnchors)
{
    if (!sharedData || !hotplugAnchors) {
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

    }
}

/**
 * 执行 Reset Zero。
 *
 * 硬件 turn0 成功后立即提交 RAM 状态与结果 token；上位机收到结果后会同步
 * 失效对应 Auto/LUT，并把新的 raw/ABS 锚点写入 JSON；处理完成不依赖后台持久化任务。
 */
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
            if (!driver || !driver->resetTurnCounter(route.servoId, &raw)) {
                failedMask |= bit;
                continue;
            }
            successMask |= bit;
            FW_LOG_WARN_PRINTF(
                "[SERVO RESET ZERO] RAM-only M%02u raw=%d token=%lu\r\n",
                (unsigned)channel,
                (int)raw,
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
    uint32_t lastCalibrationTrustToken;
    uint32_t lastServoResetZeroToken;
    uint32_t feedbackFreshnessGraceUntilMs; ///< Reset Zero 后暂停 stale 判定的截止时刻。
    bool calibrationMotionWasActive; ///< 用于在标定退出后恢复一次全量反馈宽限期。
    uint32_t lastLoopStartMs;
    uint32_t queueDropTotal;
    uint32_t lastQueueDropLogMs;
    uint32_t lastM19GoalReadbackMs; ///< 最近一次 M19 目标寄存器诊断回读时间。
    uint32_t lastM19GoalReadbackFailureLogMs; ///< 回读失败日志节流时间。
    // 上线恢复不是持久化功能；核验多圈模式后才能让上位机 JSON 提交 logical ABS。
    ServoHotplugRecoveryAnchors hotplugRecoveryAnchors;
    ServoOnlineRecoveryContext onlineRecovery;
};

static ServoRuntimeContext g_servoRuntime;

/**
 * 低频核对 M19 的舵机目标寄存器与实时反馈。
 *
 * 同步写没有单舵机 ACK，因此仅看 PID/command 日志不能证明目标已经进入
 * ST3215 SRAM。本诊断只能在四 BUS 同步读完成后的空档执行；goalLogical
 * 与 actualLogical 可直接比较，goalHw 则用于确认舵机内部真实目标。
 */
static void logM19GoalReadback(SystemRuntimeContext* sharedData,
                               ServoRuntimeContext* runtime,
                               uint32_t nowMs)
{
    constexpr uint8_t kMotorChannel = 19;
    if (!kM19GoalReadbackEnabled || !sharedData || !runtime ||
        sharedAtomicLoad(&sharedData->system_state) !=
            SYSTEM_STATE_RUN_FEEDBACK ||
        (sharedAtomicLoad(&sharedData->active_motor_mask) &
         (1UL << kMotorChannel)) == 0 ||
        (uint32_t)(nowMs - runtime->lastM19GoalReadbackMs) <
            kM19GoalReadbackPeriodMs) {
        return;
    }
    runtime->lastM19GoalReadbackMs = nowMs;

    ServoRoute route;
    FeetechServoDriver* driver =
        resolveMotorDriver(kMotorChannel, &route, true);
    ServoGoalRegisterState goal{};
    if (!driver || !driver->readGoalRegisters(route.servoId, &goal)) {
        if ((uint32_t)(nowMs - runtime->lastM19GoalReadbackFailureLogMs) >=
            kM19GoalReadbackFailureLogPeriodMs) {
            runtime->lastM19GoalReadbackFailureLogMs = nowMs;
            FW_LOG_PRINTLN(FW_LOG_LEVEL_WARN,
                           "[SERVO GOAL M19] read_failed");
        }
        return;
    }

    const ServoFeedbackInternal& feedback =
        driver->getFeedback(route.servoId);
    const int32_t logicalOffset = driver->getLogicalOffset(route.servoId);
    const int32_t goalLogical =
        (int32_t)goal.hardwarePosition + logicalOffset;
    FW_LOG_INFO_PRINTF(
        "[SERVO GOAL M19] bus=%u id=%u goalHw=%d goalLogical=%ld "
        "actualHw=%ld actualLogical=%ld raw=%d presentSpeed=%d load=%d "
        "accReg=%u speedReg=%u ageMs=%lu\r\n",
        (unsigned)route.busIndex,
        (unsigned)route.servoId,
        (int)goal.hardwarePosition,
        (long)goalLogical,
        (long)feedback.absolutePosition,
        (long)(feedback.absolutePosition + logicalOffset),
        (int)feedback.rawPosition,
        (int)feedback.speed,
        (int)feedback.load,
        (unsigned)goal.acceleration,
        (unsigned)goal.speed,
        (unsigned long)(nowMs - feedback.lastUpdate));
}

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
    g_servoRuntime.lastCalibrationTrustToken =
        sharedAtomicLoad(&sharedData->calibration_trust_request_token);
    g_servoRuntime.lastServoResetZeroToken =
        sharedAtomicLoad(&sharedData->servo_reset_zero_request_token);
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
    ServoRuntimeContext& runtime = g_servoRuntime;
    const uint32_t loopStartMs = nowMs;
    auto& lastEmergencyStopToken = runtime.lastEmergencyStopToken;
    auto& lastRestoreToken = runtime.lastRestoreToken;
    auto& lastCalibrationTrustToken = runtime.lastCalibrationTrustToken;
    auto& lastServoResetZeroToken = runtime.lastServoResetZeroToken;
    auto& lastLoopStartMs = runtime.lastLoopStartMs;
    auto& queueDropTotal = runtime.queueDropTotal;
    auto& lastQueueDropLogMs = runtime.lastQueueDropLogMs;
    auto& hotplugRecoveryAnchors = runtime.hotplugRecoveryAnchors;
    auto& onlineRecovery = runtime.onlineRecovery;
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
            currentSystemState == SYSTEM_STATE_CALIBRATION_RUNNING ||
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
            processHostMultiTurnRestore(sharedData, &lastRestoreToken);
            processCalibrationTrustCommit(sharedData, &lastCalibrationTrustToken);
        }
        const bool calibrationMotionActive =
            sharedData->system_state == SYSTEM_STATE_CALIBRATION_RUNNING &&
            sharedData->servo_target_owner == SERVO_TARGET_OWNER_CALIBRATION &&
            sharedAtomicLoad(&sharedData->calibration_motion_started) != 0;
        if (runtime.calibrationMotionWasActive && !calibrationMotionActive) {
            // 标定期间只高频读取 active/aux 电机。退出后的前四组 BUS 轮询内，
            // 其余在线舵机的旧时间戳不能立即触发 offline/Turn Unknown。
            runtime.feedbackFreshnessGraceUntilMs =
                nowMs + mmhand::config::servo::kFeedbackMaintenanceGraceMs;
        }
        runtime.calibrationMotionWasActive = calibrationMotionActive;
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
            reportCalibrationTargetResult(sharedData, batch, applyResult);
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

        const uint32_t calibrationReadMask = calibrationMotionActive
            ? (sharedAtomicLoad(&sharedData->active_motor_mask) |
               sharedAtomicLoad(&sharedData->calibration_aux_motor_mask)) &
                  validMotorChannelMask()
            : 0;
        if (!feedbackCycleCompleted) {
            if (request.periodicDue && !runtime.feedbackCycle.active()) {
                uint8_t readIds[NUM_BUSES][MAX_SERVOS_PER_BUS] = {0};
                uint8_t readCounts[NUM_BUSES] = {0};
                for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
                    if (calibrationMotionActive && calibrationReadMask != 0 &&
                        (calibrationReadMask & (1UL << ch)) == 0) {
                        continue;
                    }
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
        // 标定期间只检查本轮主动读取的电机，避免把刻意降载的其他通道误判掉线。
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
            const uint32_t bit = 1UL << motor;
            if (calibrationMotionActive &&
                calibrationReadMask != 0 &&
                (calibrationReadMask & bit) == 0) {
                continue;
            }
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
        cacheServoPowerLossSnapshots(sharedData, &hotplugRecoveryAnchors);
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

        if (!calibrationMotionActive) {
            processServoOnlineTransitions(
                sharedData,
                &hotplugRecoveryAnchors,
                &onlineRecovery);
        }
        p4::servo::publishTurnOverflowState(sharedData);
        const uint32_t resetZeroTokenBefore = lastServoResetZeroToken;
        processServoResetZeroRequest(sharedData, &lastServoResetZeroToken);
        if (lastServoResetZeroToken != resetZeroTokenBefore) {
            // Reset Zero 会暂时阻塞本任务；从事务真正结束的时刻重新计算宽限期。
            // 宽限期只跳过反馈陈旧判定，online/RESET 结果和真实读失败仍照常处理。
            runtime.feedbackFreshnessGraceUntilMs =
                p4::platform::time::monotonicMillis() +
                mmhand::config::servo::kFeedbackMaintenanceGraceMs;
        }
        p4::servo::publishRecoveryState(sharedData);
        p4::servo::publishFeedback(sharedData, runtime.completedFeedback);
        // 此处四 BUS 同步读已经结束，插入一次 M19 单播回读不会破坏并行事务。
        logM19GoalReadback(sharedData,
                           &runtime,
                           p4::platform::time::monotonicMillis());
        // 路由扫描放在正常同步反馈、掉线失效和恢复事务之后。每轮 step()
        // 仍只探测一个 BUS/ID，但不再因扫描提前 return 而饿死在线反馈。
        servoDiscoveryManager().step(loopStartMs, motionActive);
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
