/**
 * @file ControlRuntimeState.h
 * @brief 保存顶层状态机、控制 owner、故障和反馈恢复事务。
 * @details 这些字段由 SystemStateTask 统一仲裁，其他任务只发布请求或读取快照。
 */
#ifndef P4_SYSTEM_CONTEXT_CONTROL_RUNTIME_STATE_H
#define P4_SYSTEM_CONTEXT_CONTROL_RUNTIME_STATE_H

#include "../config/Config.h"
#include "RopeInitTypes.h"
#include <stdint.h>

struct ControlRuntimeState {
    volatile uint8_t control_enabled; ///< MotionControlTask 总使能。
    volatile uint8_t control_mode; ///< CONTROL_MODE_*。
    volatile uint8_t adaptive_kp_enabled; ///< 自适应 Kp 是否启用。
    volatile int16_t adaptive_kp_scale_milli[ENCODER_TOTAL_NUM]; ///< 每关节自适应倍率 ×1000。
    volatile int16_t adaptive_kp_effective_milli[ENCODER_TOTAL_NUM]; ///< 每关节实际 Kp ×1000。
    volatile uint8_t system_state; ///< SYSTEM_STATE_*。
    volatile uint8_t servo_target_owner; ///< 唯一 SERVO_TARGET_OWNER_*。
    volatile uint32_t feedback_admission_sequence; ///< 反馈门禁结果序号。
    volatile uint8_t feedback_admission_status; ///< 反馈命令准入状态。
    volatile uint8_t feedback_reject_reason; ///< 最近反馈拒绝原因。
    volatile uint32_t feedback_collision_armed_bitmap; ///< 正在观察碰撞的 Jxx。
    volatile uint32_t feedback_collision_hold_bitmap; ///< 已冻结目标的 Jxx。
    volatile uint32_t feedback_collision_transition_count; ///< 碰撞状态切换计数。
    volatile uint8_t feedback_recovery_pending; ///< Rope Init 后恢复反馈目标。
    volatile uint32_t feedback_recovery_request_sequence; ///< 待恢复关节命令序号。
    volatile uint32_t emergency_stop_request_token; ///< E-STOP 请求代际。
    volatile uint32_t servo_emergency_stop_token; ///< 舵机断扭矩请求代际。
    volatile uint8_t emergency_stop_latched; ///< E-STOP 最终发送门禁。
    volatile uint8_t host_control_hold_status; ///< HOST_CONTROL_HOLD_*。
    volatile uint32_t host_control_hold_request_token; ///< Hold 请求代际。
    volatile uint32_t host_control_hold_result_token; ///< Hold 完成代际。
    volatile uint32_t host_control_hold_started_ms; ///< Hold 开始时间。
    volatile uint32_t system_fault_bitmap; ///< 当前实时故障位。
    volatile uint32_t system_fault_latched_bitmap; ///< 必须 Clear Fault 的故障位。
    volatile uint32_t state_transition_count; ///< 顶层状态切换计数。
    volatile uint8_t rope_init_status; ///< ROPE_INIT_STATUS_*。
    volatile uint8_t rope_init_reason; ///< ROPE_INIT_REASON_*。
    volatile uint32_t rope_init_sequence; ///< Rope Init 事务序号。
    volatile uint32_t rope_init_joint_mask; ///< 本轮处理 Jxx。
    volatile uint32_t rope_init_timestamp; ///< 最近状态时间 millis()。
    RopeInitRuntimeState ropeInitRuntime; ///< 由 SystemStateTask 独占的非阻塞推进上下文。
    volatile uint32_t active_motor_mask; ///< 当前控制 scope 的 Mxx。
    volatile uint32_t active_joint_mask; ///< 当前控制 scope 的 Jxx。
    volatile uint32_t encoder_online_bitmap; ///< 磁编在线 Jxx。
    volatile uint32_t encoder_warn_bitmap; ///< 磁编非阻断警告 Jxx。
    volatile uint32_t encoder_fault_bitmap; ///< 磁编阻断故障 Jxx。
    volatile uint8_t can_link_online; ///< 最近 CAN 数据是否新鲜。
    volatile uint8_t require_can_feedback; ///< 当前 scope 是否必须有 CAN。
    volatile uint32_t overload_fault_bitmap; ///< 硬过载 Mxx。
    volatile int16_t overload_fault_load[SERVO_TOTAL_NUM]; ///< 触发时有符号 load。
    volatile uint32_t overload_fault_reset_token; ///< Clear Fault 清除代际。
};

#endif // P4_SYSTEM_CONTEXT_CONTROL_RUNTIME_STATE_H
