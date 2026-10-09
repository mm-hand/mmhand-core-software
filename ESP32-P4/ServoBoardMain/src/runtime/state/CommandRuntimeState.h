/**
 * @file CommandRuntimeState.h
 * @brief 保存上位机目标、直控暂存区及命令事务代际。
 * @details 生产者写 pending，状态机完成门禁后再提升为 applied，执行任务只读 applied。
 */
#ifndef P4_SYSTEM_CONTEXT_COMMAND_RUNTIME_STATE_H
#define P4_SYSTEM_CONTEXT_COMMAND_RUNTIME_STATE_H

#include "../../config/Config.h"
#include <stdint.h>

struct CommandRuntimeState {
    volatile uint32_t host_rx_heartbeat_ms; ///< 最近收到有效上位机帧的单调时间，用于控制租约。
    float targetAngles[ENCODER_TOTAL_NUM]; ///< 最新关节目标，单位 deg。
    int32_t motorTargetRaw[SERVO_TOTAL_NUM]; ///< 已应用单点直控 logical ABS。
    int32_t pendingMotorTargetRaw[SERVO_TOTAL_NUM]; ///< 等待门禁的单点目标。
    volatile uint32_t motor_command_token; ///< 单点直控命令代际。
    volatile uint32_t joint_command_token; ///< 关节角命令代际。
    volatile uint32_t joint_command_accepted_ms; ///< P4 完整接收当前目标帧的单调毫秒时间。
    volatile uint32_t feedback_command_joint_mask; ///< 真正参与 PID 的 Jxx。
    volatile uint32_t motor_direct_command_generation; ///< 最近写入的直控 generation。
    volatile uint8_t motor_direct_command_source; ///< 已应用 MOTOR_DIRECT_SOURCE_*。
    volatile uint32_t pending_direct_generation; ///< 待审查 generation。
    volatile uint32_t applied_direct_generation; ///< 已通过门禁 generation。
    volatile uint8_t pending_direct_source; ///< 待审查命令来源。
    volatile uint8_t pending_direct_valid; ///< pending 数组是否完整可读。
    volatile uint32_t pending_direct_motor_mask; ///< pending 依赖的 Mxx。
    volatile uint32_t pending_direct_joint_mask; ///< pending 对应的 Jxx。
    volatile uint8_t pending_direct_require_can; ///< pending 是否依赖磁编 CAN。
    volatile uint8_t direct_reject_reason; ///< 最近一次 DIRECT_REJECT_*。
    uint32_t pending_direct_token;
    uint16_t pending_direct_speed;
    uint8_t pending_direct_acc;
    volatile uint32_t applied_direct_token;
    uint16_t applied_direct_speed;
    uint8_t applied_direct_acc;
    volatile uint32_t applied_direct_motor_mask; ///< 实际输出范围，不改写 active 授权范围。
    uint32_t direct_release_generation; ///< 排队释放的代际，防止旧释放事件覆盖新目标。
    volatile uint32_t direct_target_request_generation;
    volatile uint32_t direct_target_request_token;
    volatile uint32_t direct_target_request_mask;
    volatile uint32_t direct_target_result_generation;
    volatile uint32_t direct_target_result_token;
    volatile uint32_t direct_target_requested_mask;
    volatile uint32_t direct_target_applied_mask;
    volatile uint32_t direct_target_rejected_mask;
    volatile uint8_t direct_target_status;
};

#endif // P4_SYSTEM_CONTEXT_COMMAND_RUNTIME_STATE_H
