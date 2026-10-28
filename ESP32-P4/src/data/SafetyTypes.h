/**
 * @file SafetyTypes.h
 * @brief 定义系统故障位和舵机安全决策的公共类型。
 * @details bit 1、bit 3 为协议保留位，不能压缩或重新编号。
 */
#ifndef MMHAND_SAFETY_TYPES_H
#define MMHAND_SAFETY_TYPES_H

#include <stdint.h>
#include "SafetyRuleTypes.h"

inline constexpr uint32_t SYSTEM_FAULT_CAN_OFFLINE = 1UL << 0; ///< S3/CAN 反馈链路离线。
inline constexpr uint32_t SYSTEM_FAULT_JOINT16_DUAL = 1UL << 1; ///< 协议保留的历史故障位，不得重编号。
inline constexpr uint32_t SYSTEM_FAULT_OVERLOAD = 1UL << 2; ///< 舵机负载硬保护触发。
inline constexpr uint32_t SYSTEM_FAULT_SERVO_OFFLINE = 1UL << 4; ///< active scope 舵机离线。
// 故障位 5 为旧自动标定保留，不再由固件产生；后续故障位保持原编号。
inline constexpr uint32_t SYSTEM_FAULT_MULTITURN_UNKNOWN = 1UL << 6; ///< active scope 多圈不可信。

enum class ServoRejectReason : uint8_t {
    None = 0,        ///< 允许发送。
    InvalidCommand, ///< 批次数量或 Mxx 映射非法。
    RouteUnknown,   ///< Mxx 尚未在四条 BUS 中获得唯一运行时路由。
    RouteConflict,  ///< 同一个舵机 ID 在多条 BUS 上同时响应。
    BusOverCapacity, ///< 所在 BUS 检出超过配置容量的舵机，整 BUS 禁止运动。
    EmergencyStop,  ///< E-STOP 已锁存。
    InvalidOwner,   ///< 生产者不持有当前输出权。
    InvalidState,   ///< 顶层状态不允许运动。
    Offline,        ///< 目标舵机没有可靠反馈。
    MultiTurnUnknown, ///< logical ABS 不可信。
    StaleTarget,    ///< 命令时间戳超过新鲜度窗口。
    // 值 11 永久保留；旧 LogicalRange 判据已退役，后续值不得前移。
    HardwareRange = 12,  ///< 超出 ST3215 hardware ABS 硬边界。
    TurnOverflow,   ///< 舵机已越过 Reset Zero 阈值，必须先重置圈数。
};

struct ServoSafetyDecision {
    bool allowed;             ///< 最终是否允许进入驱动。
    ServoRejectReason reason; ///< 拒绝原因；allowed=true 时必须为 None。
    SafetyRuleId ruleId;      ///< 可跨固件/上位机稳定解释的规则编号。
    int32_t requestedAbs;     ///< 业务层请求的 logical ABS，单位 counts。
    int32_t appliedAbs;       ///< 安全策略允许进入驱动的 logical ABS，单位 counts。
    int32_t hardwareAbs;      ///< 与 requestedAbs 相同的舵机 ABS，单位 counts；拒绝时仍供诊断。
    uint8_t motorChannel;     ///< 对应 Mxx；结构错误且无法映射时为 0xFF。
    uint8_t commandIndex;     ///< 批次内命令下标；无法对应单条命令时为 0xFF。
};

#endif // MMHAND_SAFETY_TYPES_H
