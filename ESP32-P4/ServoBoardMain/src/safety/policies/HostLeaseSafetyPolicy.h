/**
 * @file HostLeaseSafetyPolicy.h
 * @brief 声明上位机控制 owner 的心跳租约纯判断规则。
 * @details 本模块不操作 RTOS 队列；状态机负责发起 Hold 事务，便于后续移植 STM32。
 */
#ifndef P4_HOST_CONTROL_LEASE_POLICY_H
#define P4_HOST_CONTROL_LEASE_POLICY_H

#include <stdint.h>

namespace p4::system {

/** 所有会驱动舵机的运行态都需要上位机租约，包括标定与 Rope Init。 */
bool requiresHostControlLease(uint8_t systemState, uint8_t targetOwner);

/** 判断最近有效上位机帧是否仍在租约窗口内，使用无符号差值兼容单调时钟回绕。 */
bool isHostControlLeaseFresh(uint32_t nowMs, uint32_t lastHeartbeatMs);

} // namespace p4::system

#endif // P4_HOST_CONTROL_LEASE_POLICY_H
