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
    QueueHandle_t stateEventQueue; ///< SystemEvent_t 顺序事件。
    // 以下容量为 1 的队列均表示最新快照，生产者使用 xQueueOverwrite。
    QueueHandle_t canRxQueue;                  ///< RemoteSensorData_t 最新 CAN 快照。
    QueueHandle_t servoTargetQueue;            ///< ServoTargetBatch_t 唯一执行批次。
    QueueHandle_t servoFeedbackQueue;          ///< ServoAngleData_t logical ABS 快照。
    QueueHandle_t servoAngleQueue;             ///< ServoAngleData_t 上行角度快照。
    QueueHandle_t servoRawQueue;               ///< ServoRawData_t 单圈 raw 快照。
    QueueHandle_t servoTelemetryQueue;         ///< ServoTelemetryData_t 上行遥测。
    QueueHandle_t servoTelemetrySnapshotQueue; ///< 控制/标定只读遥测快照。
    QueueHandle_t mappedAngleQueue;            ///< MappedAngleData_t 21 关节角。
    // 调试与触觉允许保留少量待发送帧，由 UpperCommTask 顺序消费。
    QueueHandle_t jointDebugQueue; ///< JointDebugData_t 待发送诊断。
    QueueHandle_t tactileQueue;    ///< TactileData_t 待发送触觉。
    // 多圈 Flash 写入使用独立请求/结果队列，避免阻塞舵机通信任务。
    QueueHandle_t multiturnNvsSaveQueue;   ///< MultiTurnNvsSaveJob_t 请求。
    QueueHandle_t multiturnNvsResultQueue; ///< MultiTurnNvsSaveResult_t 结果。

    float targetAngles[ENCODER_TOTAL_NUM]; ///< 最新关节目标，单位 deg。
    // 所有控制命令、目标角和直控暂存区共用这一把锁，避免读到跨帧组合数据。
    SemaphoreHandle_t commandStateMutex;

    int32_t motorTargetRaw[SERVO_TOTAL_NUM];             ///< 已应用的单点直控 logical ABS。
    int32_t motorSweepTargetRaw[SERVO_TOTAL_NUM];        ///< 已应用的 sweep 直控 logical ABS。
    int32_t pendingMotorTargetRaw[SERVO_TOTAL_NUM];      ///< 等待状态机门禁的单点目标。
    int32_t pendingMotorSweepTargetRaw[SERVO_TOTAL_NUM]; ///< 等待状态机门禁的 sweep 目标。
    volatile uint32_t motor_command_token;               ///< 单点直控命令代际。
    volatile uint32_t motor_sweep_command_token;         ///< sweep 命令代际。
    volatile uint32_t joint_command_token;               ///< 关节角命令代际。
    // 当前角度命令真正参与 PID 的关节。它与 active_joint_mask 相交后生效。
    volatile uint32_t feedback_command_joint_mask;
    volatile uint32_t motor_direct_command_generation; ///< 最近写入的直控 generation。
    volatile uint8_t motor_direct_command_source;      ///< 当前已应用 MOTOR_DIRECT_SOURCE_*。
    volatile uint32_t pending_direct_generation;       ///< 待审查命令 generation。
    volatile uint32_t applied_direct_generation;       ///< 已通过门禁的 generation。
    volatile uint8_t pending_direct_source;            ///< 待审查命令来源。
    volatile uint8_t pending_direct_valid;             ///< pending 数组是否完整可读。
    volatile uint32_t pending_direct_motor_mask;       ///< pending 需要的 Mxx。
    volatile uint32_t pending_direct_joint_mask;       ///< pending 对应的 Jxx。
    volatile uint8_t pending_direct_require_can;       ///< pending 是否依赖磁编 CAN。
    volatile uint8_t direct_reject_reason;             ///< 最近一次 DIRECT_REJECT_*。

    volatile uint8_t control_enabled;              ///< ControlTask 总使能。
    volatile uint8_t control_mode;                 ///< CONTROL_MODE_*。
    volatile uint8_t system_state;                 ///< SYSTEM_STATE_*。
    volatile uint8_t paused_resume_state;          ///< PAUSED 前应恢复的状态。
    volatile uint8_t servo_target_owner;           ///< 当前唯一 SERVO_TARGET_OWNER_*。
    volatile uint32_t feedback_admission_sequence; ///< 反馈控制门禁结果序号。
    volatile uint8_t feedback_admission_status;    ///< 反馈命令等待/接受/拒绝状态。
    volatile uint8_t feedback_reject_reason;       ///< 最近一次反馈门禁拒绝原因。
    // armed 表示关节已进入有效运动；hold 表示已冻结碰撞时的舵机 ABS。
    volatile uint32_t feedback_collision_armed_bitmap; ///< 正在观察碰撞的 Jxx。
    volatile uint32_t feedback_collision_hold_bitmap;  ///< 已冻结目标的 Jxx。
    volatile uint32_t feedback_collision_transition_count; ///< 碰撞子状态切换计数。
    // 工作区外收到安全向内目标时，先执行 Rope Init，再恢复最新关节目标。
    volatile uint8_t feedback_recovery_pending;          ///< 需要 Rope Init 后恢复反馈目标。
    volatile uint32_t feedback_recovery_request_sequence;///< 被恢复的关节命令序号。
    volatile uint32_t emergency_stop_request_token;      ///< 任意任务可递增的 E-STOP 请求。
    volatile uint32_t servo_emergency_stop_token;        ///< 舵机任务已观察的断扭矩请求代际。
    volatile uint8_t emergency_stop_latched;             ///< E-STOP 最终发送门禁，仅 RESET 清除。
    // 上位机失联后先停止普通目标，再将 active 电机锁到最新实际 ABS；Servo 任务确认后才释放 owner。
    volatile uint8_t host_control_hold_status;           ///< HOST_CONTROL_HOLD_*。
    volatile uint32_t host_control_hold_request_token;   ///< 状态机发起的 Hold 事务代次。
    volatile uint32_t host_control_hold_result_token;    ///< Servo 任务最后确认的 Hold 事务代次。
    volatile uint32_t host_control_hold_started_ms;      ///< 本次 Hold 开始时间，用于确认超时。
    volatile uint32_t system_fault_bitmap;               ///< 当前实时故障位。
    volatile uint32_t system_fault_latched_bitmap;       ///< 必须 Clear Fault 的锁存故障位。
    volatile uint32_t state_transition_count;            ///< 系统状态切换计数。
    volatile uint8_t rope_init_status;                   ///< ROPE_INIT_STATUS_*。
    volatile uint8_t rope_init_reason;                   ///< ROPE_INIT_REASON_*。
    volatile uint32_t rope_init_sequence;                ///< Rope Init 事务序号。
    volatile uint32_t rope_init_joint_mask;              ///< 本轮处理的 Jxx。
    volatile uint32_t rope_init_timestamp;               ///< 最近状态更新时间 millis()。
    volatile uint32_t active_motor_mask;                 ///< 当前控制 scope 的 Mxx。
    // 标定临时扩展读取范围，不改变上位机选择的 active scope。
    volatile uint32_t calibration_aux_motor_mask; ///< 工作空间准备额外轮询的 Mxx。
    volatile uint32_t active_joint_mask;       ///< 当前控制 scope 的 Jxx。
    volatile uint32_t multi_turn_unknown_bitmap; ///< logical ABS 不可信的 Mxx。
    // hardware ABS 严格越过 Reset Zero 阈值时置位，不使用 logicalOffset。
    volatile uint32_t servo_turn_overflow_bitmap; ///< hardware ABS 超 Reset Zero 阈值的 Mxx。
    volatile uint32_t servo_online_bitmap; ///< 当前确认在线的 Mxx。
    // 热插拔恢复：pending=待配置，configured=已配置多圈，failed=恢复失败。
    volatile uint32_t servo_recovery_pending_bitmap;    ///< 等待热插拔恢复的 Mxx。
    volatile uint32_t servo_multiturn_configured_bitmap; ///< 本次上电已配置多圈模式的 Mxx。
    volatile uint32_t servo_recovery_failed_bitmap;     ///< 重试耗尽的 Mxx。
    // 每台舵机当前上电会话对应的逻辑可达范围。
    volatile uint32_t servo_logical_range_valid_bitmap; ///< 会话可达范围有效 Mxx。
    int32_t servo_logical_range_min[SERVO_TOTAL_NUM];    ///< 会话最小 logical ABS。
    int32_t servo_logical_range_max[SERVO_TOTAL_NUM];    ///< 会话最大 logical ABS。
    // 最近一次 logical target 转换后超出硬件范围的诊断。
    volatile uint32_t servo_range_reject_sequence; ///< 越界诊断序号。
    volatile uint8_t servo_range_reject_motor;     ///< 被拒绝的 Mxx。
    int32_t servo_range_reject_logical_target;     ///< 请求 logical ABS。
    int32_t servo_range_reject_hardware_target;    ///< 换算后的 hardware ABS。
    int32_t servo_range_reject_logical_min;        ///< 当前会话最小 logical ABS。
    int32_t servo_range_reject_logical_max;        ///< 当前会话最大 logical ABS。
    // 标定真正准备发送首帧运动目标后置 1，通信任务据此暂停后台扫描。
    volatile uint8_t calibration_motion_started; ///< 1 表示标定已真正进入运动阶段。
    // 本轮已经完整提交结果的 Jxx；写 LUT/限位的任务最后置位，通信任务先读此门禁。
    volatile uint32_t calibration_completed_joint_bitmap; ///< 本轮可安全上报的成功 Jxx。
    // 标定完成后，把当前 power-session ABS 登记为可信的请求/结果闭环。
    volatile uint32_t calibration_trust_request_mask;  ///< 请求登记可信的 Mxx。
    volatile uint32_t calibration_trust_request_token; ///< 登记请求代际。
    volatile uint32_t calibration_trust_result_token;  ///< 登记完成代际。
    volatile uint32_t calibration_trust_accepted_mask; ///< 成功登记 Mxx。
    volatile uint32_t encoder_online_bitmap; ///< 磁编在线 Jxx。
    volatile uint32_t encoder_warn_bitmap;   ///< 磁编非阻断警告 Jxx。
    volatile uint32_t encoder_fault_bitmap;  ///< 磁编阻断故障 Jxx。
    volatile uint8_t can_link_online;        ///< 最近 CAN 数据是否新鲜。
    int32_t restore_multi_turn_abs[SERVO_TOTAL_NUM]; ///< New Device 候选 logical ABS。
    volatile uint32_t restore_multi_turn_mask;       ///< 请求恢复的 Mxx。
    volatile uint32_t restore_multi_turn_request_token; ///< 恢复请求代际。
    volatile uint32_t restore_multi_turn_result_token;  ///< 恢复完成代际。
    volatile uint32_t restore_multi_turn_restored_mask; ///< 成功恢复 Mxx。
    volatile uint32_t restore_multi_turn_rejected_mask; ///< 拒绝恢复 Mxx。

    // Reset Zero 事务由上位机请求、舵机通信任务执行、NVS 任务确认。
    volatile uint32_t servo_reset_zero_request_mask;       ///< 请求清圈数的 Mxx。
    volatile uint32_t servo_reset_zero_request_token;      ///< Reset Zero 请求代际。
    volatile uint32_t servo_reset_zero_result_token;       ///< Reset Zero 完成代际。
    volatile uint32_t servo_reset_zero_success_mask;       ///< 舵机 ACK 且 NVS 已更新的 Mxx。
    volatile uint32_t servo_reset_zero_failed_mask;        ///< 任一步骤失败的 Mxx。
    volatile uint32_t servo_reset_zero_invalidated_joint_mask; ///< 必须重标的 Jxx。
    volatile uint8_t servo_reset_zero_status;              ///< RESET_ZERO_STATUS_*。

    volatile uint32_t multiturn_nvs_save_request_token; ///< Flash 保存请求代际。
    volatile uint8_t multiturn_nvs_save_reason;         ///< MULTITURN_NVS_SAVE_REASON_*。
    volatile uint32_t multiturn_nvs_save_result_token;  ///< Flash 保存完成代际。
    volatile uint32_t multiturn_nvs_save_valid_mask;    ///< 最近成功写入的 Mxx。
    volatile uint32_t multiturn_nvs_save_fail_count;    ///< 累计写失败次数。
    volatile uint8_t multiturn_nvs_enabled;             ///< 用户 Flash On/Off 开关。
    volatile uint8_t multiturn_nvs_snapshot_valid;      ///< 启动时是否读到有效快照。
    // 无效位的数据不得用于恢复或上位机 Flash raw 比较。
    int16_t multiturn_nvs_snapshot_raw[SERVO_TOTAL_NUM]; ///< Flash raw 对比基准。
    volatile uint32_t host_rx_heartbeat_ms;              ///< 最近上位机有效帧 millis()。

    // 从独立 NVS A/B 快照恢复的磁编码器 Manual 端点状态。
    volatile uint32_t manual_reference_valid_bitmap; ///< Manual 端点有效 Jxx。
    volatile uint32_t manual_reference_crc;          ///< 21 路 RAM 镜像整体 CRC。
    volatile uint32_t manual_reference_nvs_sequence; ///< 当前 NVS A/B 序号。
    volatile uint8_t manual_reference_nvs_valid;     ///< 启动时是否恢复有效快照。
    volatile uint8_t manual_reference_save_status;   ///< MANUAL_SAVE_STATUS_*。

    volatile uint32_t motor_abs_limit_valid_bitmap;  ///< 已标定 ABS/LUT 边界的 Mxx。
    volatile uint32_t motor_abs_limit_locked_bitmap; ///< UI LOCK 启用硬边界的 Mxx。
    int32_t motor_abs_limit_min[SERVO_TOTAL_NUM];     ///< 每路 logical ABS 下界。
    int32_t motor_abs_limit_max[SERVO_TOTAL_NUM];     ///< 每路 logical ABS 上界。
    volatile uint8_t require_can_feedback;            ///< 当前 scope 是否必须有 CAN。
    volatile uint8_t calibration_fault_latched;       ///< 自动标定失败锁存。
    volatile uint32_t calibration_warning_bitmap;     ///< 非阻断 CALIB WARN Jxx。
    int16_t calibration_auto_min_offset_counts[ENCODER_TOTAL_NUM]; ///< Auto/Manual Min 偏差。
    CalibrationDiagData_t calibration_diag;           ///< 最近结构化标定诊断。

    volatile uint32_t overload_fault_bitmap;            ///< 硬过载 Mxx。
    volatile int16_t overload_fault_load[SERVO_TOTAL_NUM]; ///< 触发时有符号 load。
    volatile uint32_t overload_fault_reset_token;        ///< Clear Fault 清除代际。
} TaskSharedData_t;

#endif // TASK_SHARED_DATA_H
