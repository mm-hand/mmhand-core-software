/**
 * @file ServoRuntimeState.h
 * @brief 保存舵机在线、动态路由、多圈恢复及会话可达范围。
 * @details ServoBusTask 是主要写入者；状态机、控制和上位机通信仅读取一致快照。
 */
#ifndef P4_SYSTEM_CONTEXT_SERVO_RUNTIME_STATE_H
#define P4_SYSTEM_CONTEXT_SERVO_RUNTIME_STATE_H

#include "../../config/Config.h"
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
    int32_t restore_multi_turn_abs[SERVO_TOTAL_NUM]; ///< New Device 候选 logical ABS。
    volatile uint32_t restore_multi_turn_mask; ///< 请求恢复 Mxx。
    volatile uint32_t restore_multi_turn_request_token; ///< 恢复请求代际。
    volatile uint32_t restore_multi_turn_result_token; ///< 恢复完成代际。
    volatile uint32_t restore_multi_turn_restored_mask; ///< 成功恢复 Mxx。
    volatile uint32_t restore_multi_turn_rejected_mask; ///< 拒绝恢复 Mxx。
    volatile uint32_t servo_reset_zero_request_mask; ///< 请求清圈数 Mxx。
    volatile uint32_t servo_reset_zero_request_token; ///< Reset Zero 请求代际。
    volatile uint32_t servo_reset_zero_result_token; ///< Reset Zero 完成代际。
    volatile uint32_t servo_reset_zero_success_mask; ///< 舵机 ACK 且 NVS 更新 Mxx。
    volatile uint32_t servo_reset_zero_failed_mask; ///< Reset Zero 失败 Mxx。
    volatile uint32_t servo_reset_zero_invalidated_joint_mask; ///< 必须重标 Jxx。
    volatile uint8_t servo_reset_zero_status; ///< RESET_ZERO_STATUS_*。
};

#endif // P4_SYSTEM_CONTEXT_SERVO_RUNTIME_STATE_H
