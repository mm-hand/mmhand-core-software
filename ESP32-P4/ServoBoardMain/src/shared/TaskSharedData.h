/**
 * @file TaskSharedData.h
 * @brief 汇总当前过渡阶段仍由多个 FreeRTOS 任务共享的运行时状态。
 * @details
 * - 固定配置已迁移到 config，业务数据类型已迁移到 domain。
 * - 本结构将在后续阶段继续拆为命令邮箱、反馈仓库、系统状态和服务句柄。
 * - 现阶段保留字段顺序和类型，避免一次性改变固件行为。
 */
#ifndef TASK_SHARED_DATA_H
#define TASK_SHARED_DATA_H

#include "../config/Config.h"
#include "../domain/ActuatorTypes.h"
#include "../domain/CalibrationTypes.h"
#include "../domain/HardwareTypes.h"
#include "../domain/JointTypes.h"
#include "../domain/RecoveryTypes.h"
#include "../domain/SafetyTypes.h"
#include "../domain/ServoTypes.h"
#include "../domain/SystemTypes.h"
#include "../domain/TelemetryTypes.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

typedef struct {
    // 状态机事件队列：保存顺序事件；E-STOP 另走 token，不依赖该队列容量。
    QueueHandle_t stateEventQueue;
    // 以下容量为 1 的队列均表示最新快照，生产者使用 xQueueOverwrite。
    QueueHandle_t canRxQueue;
    QueueHandle_t servoTargetQueue;
    QueueHandle_t servoFeedbackQueue;
    QueueHandle_t servoAngleQueue;
    QueueHandle_t servoRawQueue;
    QueueHandle_t servoTelemetryQueue;
    QueueHandle_t servoTelemetrySnapshotQueue;
    QueueHandle_t mappedAngleQueue;
    // 调试与触觉允许保留少量待发送帧，由 UpperCommTask 顺序消费。
    QueueHandle_t jointDebugQueue;
    QueueHandle_t tactileQueue;
    // 多圈 Flash 写入使用独立请求/结果队列，避免阻塞舵机通信任务。
    QueueHandle_t multiturnNvsSaveQueue;
    QueueHandle_t multiturnNvsResultQueue;

    float targetAngles[ENCODER_TOTAL_NUM];
    // 所有控制命令、目标角和直控暂存区共用这一把锁，避免读到跨帧组合数据。
    SemaphoreHandle_t commandStateMutex;

    int32_t motorTargetRaw[SERVO_TOTAL_NUM];
    int32_t motorSweepTargetRaw[SERVO_TOTAL_NUM];
    int32_t pendingMotorTargetRaw[SERVO_TOTAL_NUM];
    int32_t pendingMotorSweepTargetRaw[SERVO_TOTAL_NUM];
    volatile uint32_t motor_command_token;
    volatile uint32_t motor_sweep_command_token;
    volatile uint32_t joint_command_token;
    // 当前角度命令真正参与 PID 的关节。它与 active_joint_mask 相交后生效。
    volatile uint32_t feedback_command_joint_mask;
    volatile uint32_t motor_direct_command_generation;
    volatile uint8_t motor_direct_command_source;
    volatile uint32_t pending_direct_generation;
    volatile uint32_t applied_direct_generation;
    volatile uint8_t pending_direct_source;
    volatile uint8_t pending_direct_valid;
    volatile uint32_t pending_direct_motor_mask;
    volatile uint32_t pending_direct_joint_mask;
    volatile uint8_t pending_direct_require_can;
    volatile uint8_t direct_reject_reason;

    volatile uint8_t control_enabled;
    volatile uint8_t control_mode;
    volatile uint8_t system_state;
    volatile uint8_t paused_resume_state;
    volatile uint8_t servo_target_owner;
    volatile uint32_t feedback_admission_sequence;
    volatile uint8_t feedback_admission_status;
    volatile uint8_t feedback_reject_reason;
    // armed 表示关节已进入有效运动；hold 表示已冻结碰撞时的舵机 ABS。
    volatile uint32_t feedback_collision_armed_bitmap;
    volatile uint32_t feedback_collision_hold_bitmap;
    volatile uint32_t feedback_collision_transition_count;
    // 工作区外收到安全向内目标时，先执行 Rope Init，再恢复最新关节目标。
    volatile uint8_t feedback_recovery_pending;
    volatile uint32_t feedback_recovery_request_sequence;
    volatile uint32_t emergency_stop_request_token;
    volatile uint32_t servo_emergency_stop_token;
    volatile uint32_t system_fault_bitmap;
    volatile uint32_t system_fault_latched_bitmap;
    volatile uint32_t state_transition_count;
    volatile uint8_t rope_init_status;
    volatile uint8_t rope_init_reason;
    volatile uint32_t rope_init_sequence;
    volatile uint32_t rope_init_joint_mask;
    volatile uint32_t rope_init_timestamp;
    volatile uint32_t active_motor_mask;
    // 标定临时扩展读取范围，不改变上位机选择的 active scope。
    volatile uint32_t calibration_aux_motor_mask;
    volatile uint32_t active_joint_mask;
    volatile uint32_t multi_turn_unknown_bitmap;
    // hardware ABS 严格越过 Reset Zero 阈值时置位，不使用 logicalOffset。
    volatile uint32_t servo_turn_overflow_bitmap;
    volatile uint32_t servo_online_bitmap;
    // 热插拔恢复：pending=待配置，configured=已配置多圈，failed=恢复失败。
    volatile uint32_t servo_recovery_pending_bitmap;
    volatile uint32_t servo_multiturn_configured_bitmap;
    volatile uint32_t servo_recovery_failed_bitmap;
    // 每台舵机当前上电会话对应的逻辑可达范围。
    volatile uint32_t servo_logical_range_valid_bitmap;
    int32_t servo_logical_range_min[SERVO_TOTAL_NUM];
    int32_t servo_logical_range_max[SERVO_TOTAL_NUM];
    // 最近一次 logical target 转换后超出硬件范围的诊断。
    volatile uint32_t servo_range_reject_sequence;
    volatile uint8_t servo_range_reject_motor;
    int32_t servo_range_reject_logical_target;
    int32_t servo_range_reject_hardware_target;
    int32_t servo_range_reject_logical_min;
    int32_t servo_range_reject_logical_max;
    // 标定真正准备发送首帧运动目标后置 1，通信任务据此暂停后台扫描。
    volatile uint8_t calibration_motion_started;
    // 标定完成后，把当前 power-session ABS 登记为可信的请求/结果闭环。
    volatile uint32_t calibration_trust_request_mask;
    volatile uint32_t calibration_trust_request_token;
    volatile uint32_t calibration_trust_result_token;
    volatile uint32_t calibration_trust_accepted_mask;
    volatile uint32_t encoder_online_bitmap;
    volatile uint32_t encoder_warn_bitmap;
    volatile uint32_t encoder_fault_bitmap;
    volatile uint8_t can_link_online;
    int32_t restore_multi_turn_abs[SERVO_TOTAL_NUM];
    volatile uint32_t restore_multi_turn_mask;
    volatile uint32_t restore_multi_turn_request_token;
    volatile uint32_t restore_multi_turn_result_token;
    volatile uint32_t restore_multi_turn_restored_mask;
    volatile uint32_t restore_multi_turn_rejected_mask;

    // Reset Zero 事务由上位机请求、舵机通信任务执行、NVS 任务确认。
    volatile uint32_t servo_reset_zero_request_mask;
    volatile uint32_t servo_reset_zero_request_token;
    volatile uint32_t servo_reset_zero_result_token;
    volatile uint32_t servo_reset_zero_success_mask;
    volatile uint32_t servo_reset_zero_failed_mask;
    volatile uint32_t servo_reset_zero_invalidated_joint_mask;
    volatile uint8_t servo_reset_zero_status;

    volatile uint32_t multiturn_nvs_save_request_token;
    volatile uint8_t multiturn_nvs_save_reason;
    volatile uint32_t multiturn_nvs_save_result_token;
    volatile uint32_t multiturn_nvs_save_valid_mask;
    volatile uint32_t multiturn_nvs_save_fail_count;
    volatile uint8_t multiturn_nvs_enabled;
    volatile uint8_t multiturn_nvs_snapshot_valid;
    // 无效位的数据不得用于恢复或上位机 Flash raw 比较。
    int16_t multiturn_nvs_snapshot_raw[SERVO_TOTAL_NUM];
    volatile uint32_t host_rx_heartbeat_ms;

    // 从独立 NVS A/B 快照恢复的磁编码器 Manual 端点状态。
    volatile uint32_t manual_reference_valid_bitmap;
    volatile uint32_t manual_reference_crc;
    volatile uint32_t manual_reference_nvs_sequence;
    volatile uint8_t manual_reference_nvs_valid;
    volatile uint8_t manual_reference_save_status;

    volatile uint32_t motor_abs_limit_valid_bitmap;
    volatile uint32_t motor_abs_limit_locked_bitmap;
    int32_t motor_abs_limit_min[SERVO_TOTAL_NUM];
    int32_t motor_abs_limit_max[SERVO_TOTAL_NUM];
    volatile uint8_t require_can_feedback;
    volatile uint8_t calibration_fault_latched;
    volatile uint32_t calibration_warning_bitmap;
    int16_t calibration_auto_min_offset_counts[ENCODER_TOTAL_NUM];
    CalibrationDiagData_t calibration_diag;

    volatile uint32_t overload_fault_bitmap;
    volatile int16_t overload_fault_load[SERVO_TOTAL_NUM];
    volatile uint32_t overload_fault_reset_token;
} TaskSharedData_t;

#endif // TASK_SHARED_DATA_H
