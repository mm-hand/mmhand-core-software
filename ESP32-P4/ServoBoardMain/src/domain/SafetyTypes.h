/**
 * @file SafetyTypes.h
 * @brief 定义系统故障位和舵机安全决策的公共类型。
 * @details bit 1、bit 3 为协议保留位，不能压缩或重新编号。
 */
#ifndef MMHAND_SAFETY_TYPES_H
#define MMHAND_SAFETY_TYPES_H

#include <stdint.h>

inline constexpr uint32_t SYSTEM_FAULT_CAN_OFFLINE = 1UL << 0; ///< S3/CAN 反馈链路离线。
inline constexpr uint32_t SYSTEM_FAULT_JOINT16_DUAL = 1UL << 1; ///< 协议保留的历史故障位，不得重编号。
inline constexpr uint32_t SYSTEM_FAULT_OVERLOAD = 1UL << 2; ///< 舵机负载硬保护触发。
inline constexpr uint32_t SYSTEM_FAULT_SERVO_OFFLINE = 1UL << 4; ///< active scope 舵机离线。
inline constexpr uint32_t SYSTEM_FAULT_CALIBRATION = 1UL << 5; ///< 自动标定失败。
inline constexpr uint32_t SYSTEM_FAULT_MULTITURN_UNKNOWN = 1UL << 6; ///< active scope 多圈不可信。

enum class ServoRejectReason : uint8_t {
    None = 0,        ///< 允许发送。
    EmergencyStop,  ///< E-STOP 已锁存。
    InvalidOwner,   ///< 生产者不持有当前输出权。
    InvalidState,   ///< 顶层状态不允许运动。
    Offline,        ///< 目标舵机没有可靠反馈。
    MultiTurnUnknown, ///< logical ABS 不可信。
    StaleTarget,    ///< 命令时间戳超过新鲜度窗口。
    LogicalRange,   ///< 超出标定/LUT 逻辑边界。
    HardwareRange,  ///< 超出 ST3215 hardware ABS 硬边界。
};

struct ServoSafetyDecision {
    bool allowed;             ///< 最终是否允许进入驱动。
    ServoRejectReason reason; ///< 拒绝或钳位原因。
    int32_t requestedAbs;     ///< 业务层请求的 logical ABS。
    int32_t appliedAbs;       ///< 安全策略实际允许的 logical ABS。
};

#endif // MMHAND_SAFETY_TYPES_H
