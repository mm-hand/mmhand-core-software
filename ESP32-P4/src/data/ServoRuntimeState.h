/**
 * @file ServoRuntimeState.h
 * @brief 保存舵机在线、动态路由、多圈恢复及会话可达范围。
 * @details ServoBusTask 是主要写入者；状态机、控制和上位机通信仅读取一致快照。
 */
#ifndef P4_SYSTEM_CONTEXT_SERVO_RUNTIME_STATE_H
#define P4_SYSTEM_CONTEXT_SERVO_RUNTIME_STATE_H

#include "../config/Config.h"
#include <stdint.h>

struct ServoRuntimeState {
    volatile uint32_t multi_turn_unknown_bitmap; ///< logical ABS 不可信 Mxx。
    volatile uint32_t servo_turn_overflow_bitmap; ///< hardware ABS 超 Reset Zero 阈值 Mxx。
    volatile uint32_t servo_online_bitmap; ///< 当前确认在线 Mxx。
    volatile uint32_t servo_route_valid_bitmap; ///< 已定位到唯一 BUS 的 Mxx。
    volatile uint32_t servo_route_controllable_bitmap; ///< 可控且 BUS 未超载 Mxx。
    volatile uint32_t servo_route_conflict_bitmap; ///< 同 ID 多 BUS 响应 Mxx。
    volatile uint8_t servo_bus_overload_mask; ///< bit n=BUSn 超过容量。
    volatile uint8_t servo_bus_device_count[NUM_BUSES]; ///< 每条 BUS 检出数量。
    volatile uint8_t servo_route_bus[SERVO_TOTAL_NUM]; ///< 每个 Mxx 当前 BUS。
    volatile uint8_t servo_route_state[SERVO_TOTAL_NUM]; ///< ServoRouteState。
    volatile uint8_t servo_route_scan_state; ///< 0空闲、1全扫、2单 Mxx 重扫。
    volatile uint8_t servo_route_scan_progress; ///< 0..100 扫描进度。
    volatile uint32_t servo_route_generation; ///< 路由重建代际。
    volatile uint32_t servo_recovery_pending_bitmap; ///< 等待热插拔恢复 Mxx。
    volatile uint32_t servo_multiturn_configured_bitmap; ///< 已配置多圈模式 Mxx。
    volatile uint32_t servo_recovery_failed_bitmap; ///< 恢复重试耗尽 Mxx。
    volatile uint32_t servo_logical_range_valid_bitmap; ///< 会话可达范围有效 Mxx。
    int32_t servo_logical_range_min[SERVO_TOTAL_NUM]; ///< 会话最小 logical ABS。
    int32_t servo_logical_range_max[SERVO_TOTAL_NUM]; ///< 会话最大 logical ABS。
    volatile uint32_t trust_multi_turn_request_mask; ///< 上位机对账后认账的 Mxx。
    volatile uint32_t trust_multi_turn_request_token; ///< 认账请求代际。
    volatile uint32_t servo_rebase_zero_request_mask; ///< 当前 logical ABS 归零请求 Mxx。
    volatile uint32_t servo_rebase_zero_request_token; ///< 非零请求代际，与结果不同表示事务未完成。
    volatile uint32_t servo_rebase_zero_result_token; ///< 实际置数/标定失效均完成后发布的结果代际。
    volatile uint32_t servo_rebase_zero_success_mask; ///< 0x0B 校正后 hardware ABS 与 logical ABS 均为0。
    volatile uint32_t servo_rebase_zero_failed_mask; ///< 本次归零失败或执行前安全门禁拒绝的 Mxx。
};

#endif // P4_SYSTEM_CONTEXT_SERVO_RUNTIME_STATE_H
