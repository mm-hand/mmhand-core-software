/**
 * @file ActuatorSafetyPolicy.h
 * @brief 集中定义舵机命令从业务层到驱动层必须满足的统一安全规则。
 */
#ifndef P4_ACTUATOR_SAFETY_POLICY_H
#define P4_ACTUATOR_SAFETY_POLICY_H

#include "../../data/ActuatorTypes.h"
#include "../../data/SafetyTypes.h"

namespace p4::actuator {

/**
 * 执行层在一个控制周期内冻结的安全上下文。
 * 位图 bit n 均表示 Mn。
 * 调用方必须保证范围数组在 evaluateCommand() 返回前持续有效。
 */
struct ServoSafetyContext {
    uint8_t activeOwner;              ///< 当前真正拥有输出权的 SERVO_TARGET_OWNER_*。
    uint8_t systemState;              ///< 当前 SYSTEM_STATE_*，用于 owner/state 最终交叉校验。
    bool emergencyStopLatched;        ///< E-STOP 锁存后拒绝全部位置命令，直至 RESET。
    uint32_t nowMs;                   ///< 执行层当前单调时间。
    uint32_t maxTargetAgeMs;          ///< 批次最大允许年龄；0 表示不启用年龄检查。
    uint32_t onlineMotorMask;         ///< 已由反馈去抖确认在线的 Mxx 位图。
    uint32_t routedMotorMask;         ///< 已获得唯一 BUS 路由的 Mxx 位图。
    uint32_t controllableRouteMask;   ///< 唯一路由且 BUS 未超载的 Mxx 位图。
    uint32_t routeConflictMask;       ///< 同 ID 在多条 BUS 响应的 Mxx 位图。
    uint32_t multiTurnUnknownMask;    ///< logical ABS 仍不可信的 Mxx 位图。
    uint32_t turnOverflowMask;        ///< 已越过 Reset Zero 阈值的 Mxx 位图。
    bool requireOnline;               ///< 本次执行是否要求在线反馈已经成立。
    bool requireMultiTurnKnown;       ///< 本次执行是否要求多圈坐标可信。
};

/**
 * 检查整个批次的静态结构。该函数不读取 RTOS/共享状态，后续可直接用于 host 测试。
 * 通过时 reason=None；构建失败、重复 Mxx 或命令字段非法时整批拒绝；
 * 字段非法时 commandIndex 指向首个非法命令，count 越界时指向批次容量。
 */
ServoSafetyDecision evaluateBatchStructure(const ServoTargetBatch_t& batch);

/**
 * 计算目标批次在消费端的有效年龄。
 *
 * 32 位单调时钟正常回绕仍按无符号差值工作；若生产者时间戳比消费端快照更新，
 * 说明批次产生于该快照之后，此时年龄按 0 处理，不能下溢成约 49 天的旧命令。
 */
uint32_t computeTargetAgeMs(uint32_t nowMs, uint32_t targetTimestampMs);

/**
 * 对一条 logical ABS 命令执行最终裁决。
 *
 * 顺序固定为：映射/owner -> online -> Turn -> overflow -> hardware range。
 * 先拒绝坐标不可信，再检查数值边界，可让日志准确指出根因。
 * 驱动层仍保留 hardware range 复核，构成最后一道独立防线。
 */
ServoSafetyDecision evaluateCommand(const ServoTargetBatch_t& batch,
                                    uint8_t commandIndex,
                                    const ServoSafetyContext& context);

/** 返回用于低频日志和 UI 诊断的稳定英文原因名。 */
const char* rejectReasonName(ServoRejectReason reason);

} // namespace p4::actuator

#endif // P4_ACTUATOR_SAFETY_POLICY_H
