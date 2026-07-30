/**
 * @file SafetyTypes.h
 * @brief 定义系统故障位和舵机安全决策的公共类型。
 * @details bit 1、bit 3 为协议保留位，不能压缩或重新编号。
 */
#ifndef MMHAND_SAFETY_TYPES_H
#define MMHAND_SAFETY_TYPES_H

#include <stdint.h>

inline constexpr uint32_t SYSTEM_FAULT_CAN_OFFLINE = 1UL << 0;
inline constexpr uint32_t SYSTEM_FAULT_JOINT16_DUAL = 1UL << 1;
inline constexpr uint32_t SYSTEM_FAULT_OVERLOAD = 1UL << 2;
inline constexpr uint32_t SYSTEM_FAULT_SERVO_OFFLINE = 1UL << 4;
inline constexpr uint32_t SYSTEM_FAULT_CALIBRATION = 1UL << 5;
inline constexpr uint32_t SYSTEM_FAULT_MULTITURN_UNKNOWN = 1UL << 6;

enum class ServoRejectReason : uint8_t {
    None = 0,
    EmergencyStop,
    InvalidOwner,
    InvalidState,
    Offline,
    MultiTurnUnknown,
    StaleTarget,
    LogicalRange,
    HardwareRange,
};

struct ServoSafetyDecision {
    bool allowed;
    ServoRejectReason reason;
    int32_t requestedAbs;
    int32_t appliedAbs;
};

#endif // MMHAND_SAFETY_TYPES_H
