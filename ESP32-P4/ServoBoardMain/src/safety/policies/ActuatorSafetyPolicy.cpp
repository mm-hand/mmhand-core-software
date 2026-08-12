/**
 * @file ActuatorSafetyPolicy.cpp
 * @brief 实现纯数据驱动的舵机结构检查与发送前最终安全裁决。
 */
#include "ActuatorSafetyPolicy.h"

#include "../../config/safety/ServoPositionRange.h"
#include "../SafetyRuleRegistry.h"
#include "../../data/SystemTypes.h"

#include <limits.h>

namespace p4::actuator {

namespace {

/** 构造完整决策，确保允许与拒绝路径都初始化全部诊断字段。 */
ServoSafetyDecision makeDecision(bool allowed,
                                 ServoRejectReason reason,
                                 int32_t requestedAbs,
                                 int32_t appliedAbs,
                                 int32_t hardwareAbs,
                                 uint8_t motorChannel,
                                 uint8_t commandIndex)
{
    ServoSafetyDecision decision;
    decision.allowed = allowed;
    decision.reason = reason;
    decision.ruleId = p4::safety::ruleForServoRejectReason(reason);
    decision.requestedAbs = requestedAbs;
    decision.appliedAbs = appliedAbs;
    decision.hardwareAbs = hardwareAbs;
    decision.motorChannel = motorChannel;
    decision.commandIndex = commandIndex;
    return decision;
}

/** 拒绝时不暗中修改目标，appliedAbs 保留原请求以便日志复现。 */
ServoSafetyDecision reject(ServoRejectReason reason,
                           int32_t requestedAbs,
                           int32_t hardwareAbs,
                           uint8_t motorChannel,
                           uint8_t commandIndex)
{
    return makeDecision(false,
                        reason,
                        requestedAbs,
                        requestedAbs,
                        hardwareAbs,
                        motorChannel,
                        commandIndex);
}

/** owner 与系统运行态必须成对出现，防止状态切换竞态泄漏旧批次。 */
bool ownerMatchesSystemState(uint8_t owner, uint8_t state)
{
    if (owner == SERVO_TARGET_OWNER_CONTROL) {
        return state == SYSTEM_STATE_RUN_DIRECT ||
               state == SYSTEM_STATE_RUN_FEEDBACK;
    }
    if (owner == SERVO_TARGET_OWNER_CALIBRATION) {
        return state == SYSTEM_STATE_CALIBRATION_RUNNING ||
               state == SYSTEM_STATE_ROPE_INIT_RUNNING;
    }
    return false;
}

} // namespace

ServoSafetyDecision evaluateBatchStructure(const ServoTargetBatch_t& batch)
{
    // 构建器一旦发现容量、拓扑或追加错误，就拒绝整个事务，避免半截批次驱动机械结构。
    if (batch.buildFailed != 0) {
        return reject(ServoRejectReason::InvalidCommand,
                      0,
                      0,
                      0xFF,
                      0xFF);
    }
    // count 是后续遍历 commands[] 的边界，必须先于任何命令解引用检查。
    if (batch.count > SERVO_TARGET_BATCH_MAX) {
        return reject(ServoRejectReason::InvalidCommand,
                      0,
                      0,
                      0xFF,
                      SERVO_TARGET_BATCH_MAX);
    }
    if (batch.purpose != SERVO_TARGET_PURPOSE_NORMAL &&
        batch.purpose != SERVO_TARGET_PURPOSE_HOST_LEASE_HOLD) {
        return reject(ServoRejectReason::InvalidCommand,
                      0,
                      0,
                      0xFF,
                      0xFF);
    }
    if (batch.purpose == SERVO_TARGET_PURPOSE_HOST_LEASE_HOLD &&
        ((batch.source != SERVO_TARGET_OWNER_CONTROL &&
          batch.source != SERVO_TARGET_OWNER_CALIBRATION) ||
         batch.transactionToken == 0 ||
         batch.count == 0)) {
        return reject(ServoRejectReason::InvalidCommand,
                      0,
                      0,
                      0xFF,
                      0xFF);
    }
    for (uint8_t i = 0; i < batch.count; ++i) {
        const ServoTargetCommand_t& command = batch.commands[i];
        if (command.motorChannel >= SERVO_TOTAL_NUM) {
            return reject(ServoRejectReason::InvalidCommand,
                          command.position,
                          0,
                          0xFF,
                          i);
        }
        // 同一事务内一个 Mxx 只能出现一次；重复目标的执行顺序不应由数组顺序决定。
        for (uint8_t previous = 0; previous < i; ++previous) {
            if (batch.commands[previous].motorChannel == command.motorChannel) {
                return reject(ServoRejectReason::InvalidCommand,
                              command.position,
                              0,
                              command.motorChannel,
                              i);
            }
        }
    }
    return makeDecision(true,
                        ServoRejectReason::None,
                        0,
                        0,
                        0,
                        0xFF,
                        0xFF);
}

ServoSafetyDecision evaluateCommand(const ServoTargetBatch_t& batch,
                                    uint8_t commandIndex,
                                    int32_t logicalOffset,
                                    const ServoSafetyContext& context)
{
    const uint8_t motorChannel = commandIndex < batch.count
        ? batch.commands[commandIndex].motorChannel
        : 0xFF;
    if (commandIndex >= batch.count || commandIndex >= SERVO_TARGET_BATCH_MAX ||
        motorChannel >= SERVO_TOTAL_NUM) {
        return reject(ServoRejectReason::InvalidCommand,
                      0,
                      0,
                      motorChannel,
                      commandIndex);
    }

    const ServoTargetCommand_t& command = batch.commands[commandIndex];
    const int32_t requestedAbs = command.position;
    const uint32_t motorBit = 1UL << motorChannel;
    // logicalOffset 可能使减法越过 int32，因此先在 int64 中完成坐标转换。
    const int64_t hardware64 =
        (int64_t)requestedAbs - (int64_t)logicalOffset;
    const int32_t hardwareAbs = hardware64 > INT32_MAX
                                    ? INT32_MAX
                                    : (hardware64 < INT32_MIN
                                           ? INT32_MIN
                                           : (int32_t)hardware64);

    // 动态路由是所有物理访问的前置条件；冲突和超载必须在 online 判断前给出明确原因。
    if ((context.routeConflictMask & motorBit) != 0) {
        return reject(ServoRejectReason::RouteConflict,
                      requestedAbs, hardwareAbs, motorChannel, commandIndex);
    }
    if ((context.routedMotorMask & motorBit) == 0) {
        return reject(ServoRejectReason::RouteUnknown,
                      requestedAbs, hardwareAbs, motorChannel, commandIndex);
    }
    if ((context.controllableRouteMask & motorBit) == 0) {
        return reject(ServoRejectReason::BusOverCapacity,
                      requestedAbs, hardwareAbs, motorChannel, commandIndex);
    }

    // E-STOP 必须先于 owner/state 判断：急停请求一经锁存，任何竞态中的旧批次都不得落到驱动。
    if (context.emergencyStopLatched) {
        return reject(ServoRejectReason::EmergencyStop,
                      requestedAbs,
                      hardwareAbs,
                      motorChannel,
                      commandIndex);
    }
    // owner 与状态不匹配说明批次跨越了模式切换边界，即使 source 恰好相同也必须拒绝。
    if (!ownerMatchesSystemState(context.activeOwner, context.systemState)) {
        return reject(ServoRejectReason::InvalidState,
                      requestedAbs,
                      hardwareAbs,
                      motorChannel,
                      commandIndex);
    }
    // 单调毫秒时钟使用无符号差值支持回绕；陈旧批次不得因队列延迟重新驱动舵机。
    if (context.maxTargetAgeMs != 0 &&
        (uint32_t)(context.nowMs - batch.timestamp) > context.maxTargetAgeMs) {
        return reject(ServoRejectReason::StaleTarget,
                      requestedAbs,
                      hardwareAbs,
                      motorChannel,
                      commandIndex);
    }

    // source 是生产者身份，activeOwner 是状态机授予的当前输出权，两者必须完全一致。
    if (context.activeOwner == SERVO_TARGET_OWNER_NONE ||
        batch.source != context.activeOwner) {
        return reject(ServoRejectReason::InvalidOwner,
                      requestedAbs,
                      hardwareAbs,
                      motorChannel,
                      commandIndex);
    }
    // 在线位图已经经过反馈去抖；未在线时不发送“碰运气”的总线写命令。
    if (context.requireOnline &&
        (context.onlineMotorMask & motorBit) == 0) {
        return reject(ServoRejectReason::Offline,
                      requestedAbs,
                      hardwareAbs,
                      motorChannel,
                      commandIndex);
    }
    // 标定/人工直控可关闭此门禁来重建坐标，负反馈与 Rope Init 必须开启。
    if (context.requireMultiTurnKnown &&
        (context.multiTurnUnknownMask & motorBit) != 0) {
        return reject(ServoRejectReason::MultiTurnUnknown,
                      requestedAbs,
                      hardwareAbs,
                      motorChannel,
                      commandIndex);
    }
    // overflow 表示 hardware ABS 已到 Reset Zero 安全阈值，所有模式均禁止继续推进。
    if ((context.turnOverflowMask & motorBit) != 0) {
        return reject(ServoRejectReason::TurnOverflow,
                      requestedAbs,
                      hardwareAbs,
                      motorChannel,
                      commandIndex);
    }
    // 只有 valid bit 成立才解释范围数组，避免未初始化的 0..0 被误当成有效工作区。
    if ((context.logicalRangeValidMask & motorBit) != 0 &&
        context.logicalMin && context.logicalMax &&
        (requestedAbs < context.logicalMin[motorChannel] ||
         requestedAbs > context.logicalMax[motorChannel])) {
        return reject(ServoRejectReason::LogicalRange,
                      requestedAbs,
                      hardwareAbs,
                      motorChannel,
                      commandIndex);
    }
    // 最终硬边界检查的是换算后的舵机内部坐标，不是平移后的 logical 坐标。
    if (hardware64 < kServoHardwareCommandMin ||
        hardware64 > kServoHardwareCommandMax) {
        return reject(ServoRejectReason::HardwareRange,
                      requestedAbs,
                      hardwareAbs,
                      motorChannel,
                      commandIndex);
    }

    return makeDecision(true,
                        ServoRejectReason::None,
                        requestedAbs,
                        requestedAbs,
                        hardwareAbs,
                        motorChannel,
                        commandIndex);
}

const char* rejectReasonName(ServoRejectReason reason)
{
    // 字符串属于日志/诊断稳定契约，修改名称会影响离线日志检索脚本。
    switch (reason) {
        case ServoRejectReason::None: return "none";
        case ServoRejectReason::InvalidCommand: return "invalid-command";
        case ServoRejectReason::RouteUnknown: return "route-unknown";
        case ServoRejectReason::RouteConflict: return "route-conflict";
        case ServoRejectReason::BusOverCapacity: return "bus-over-capacity";
        case ServoRejectReason::EmergencyStop: return "emergency-stop";
        case ServoRejectReason::InvalidOwner: return "invalid-owner";
        case ServoRejectReason::InvalidState: return "invalid-state";
        case ServoRejectReason::Offline: return "offline";
        case ServoRejectReason::MultiTurnUnknown: return "multiturn-unknown";
        case ServoRejectReason::StaleTarget: return "stale-target";
        case ServoRejectReason::LogicalRange: return "logical-range";
        case ServoRejectReason::HardwareRange: return "hardware-range";
        case ServoRejectReason::TurnOverflow: return "turn-overflow";
        default: return "unknown";
    }
}

} // namespace p4::actuator
