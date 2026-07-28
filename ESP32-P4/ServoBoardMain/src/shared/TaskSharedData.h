#ifndef TASK_SHARED_DATA_H
#define TASK_SHARED_DATA_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// Shared firmware dimensions.
#define ENCODER_TOTAL_NUM 21
#define SERVO_TOTAL_NUM 27
#define JOINT_COUNT ENCODER_TOTAL_NUM
#define MAX_JOINT_ACTUATORS 2
#define MOTOR_MAP_INVALID_BUS 0xFF
#define NUM_BUSES 4
#define MAX_SERVOS_PER_BUS 8
#define MAX_SERVO_ID 32
#define SERVO_SYNC_READ_TIMEOUT_MS_DEFAULT 3
#define SERVO_TARGET_SPEED_DEFAULT 400
#define SERVO_TARGET_ACC_DEFAULT 200

// Tactile summary: group/sensor/axis flattened into one byte array.
#define TACTILE_GROUP_NUM 5
#define TACTILE_SENSOR_PER_GROUP 3
#define TACTILE_AXIS_NUM 3
#define TACTILE_TIP_TAXELS 25
#define TACTILE_PAD_TAXELS 52
#define TACTILE_SENSOR0_TAXELS TACTILE_PAD_TAXELS
#define TACTILE_SENSOR1_TAXELS TACTILE_TIP_TAXELS
#define TACTILE_SENSOR2_TAXELS TACTILE_TIP_TAXELS
#define TACTILE_SUMMARY_BYTES (TACTILE_GROUP_NUM * TACTILE_SENSOR_PER_GROUP * TACTILE_AXIS_NUM)
#define TACTILE_FULL_BYTES (TACTILE_GROUP_NUM * (TACTILE_SENSOR0_TAXELS + TACTILE_SENSOR1_TAXELS + TACTILE_SENSOR2_TAXELS) * TACTILE_AXIS_NUM)

// Control output modes.
#define CONTROL_MODE_JOINT 0
#define CONTROL_MODE_DIRECT_MOTOR 1

// Direct motor command source.
#define MOTOR_DIRECT_SOURCE_NONE 0
#define MOTOR_DIRECT_SOURCE_TARGET 1
#define MOTOR_DIRECT_SOURCE_SWEEP 2
#define MOTOR_DIRECT_SOURCE_ABSOLUTE 3

// 状态机事件编号属于固件内部 ABI；完成事件通过 SystemEvent_t.token 绑定命令代次。
#define SYSTEM_EVENT_START 1
#define SYSTEM_EVENT_STOP 2
#define SYSTEM_EVENT_RESET 3
#define SYSTEM_EVENT_CALIBRATE 4
#define SYSTEM_EVENT_CALIBRATION_DONE 5
#define SYSTEM_EVENT_CALIBRATION_FAILED 6
#define SYSTEM_EVENT_RUN_DIRECT 7
#define SYSTEM_EVENT_RUN_DIRECT_DONE 8
#define SYSTEM_EVENT_RUN_FEEDBACK 9
#define SYSTEM_EVENT_RUN_FEEDBACK_DONE 10
#define SYSTEM_EVENT_PAUSE 11
#define SYSTEM_EVENT_REEVALUATE_READY 14
#define SYSTEM_EVENT_RUN_FEEDBACK_TIMEOUT 15
#define SYSTEM_EVENT_ROPE_INIT 16
#define SYSTEM_EVENT_ROPE_INIT_DONE 17
#define SYSTEM_EVENT_ROPE_INIT_FAILED 18

#define MULTITURN_NVS_PERSISTENCE_ENABLE 1
#define MULTITURN_NVS_SAVE_REASON_RUN_START 1
#define MULTITURN_NVS_SAVE_REASON_RUN_DONE 2
#define MULTITURN_NVS_SAVE_REASON_HOST_OFFLINE 3
#define MULTITURN_NVS_SAVE_REASON_HOST_RESTORE 4
#define MULTITURN_NVS_SAVE_REASON_CALIB_DONE 5
#define MULTITURN_NVS_SAVE_REASON_OFFLINE_MOVE 6
#define MULTITURN_NVS_SAVE_REASON_OFFLINE_TURN_EDGE 7
// 舵机单独掉电时，保存掉线前最后一次可信反馈；Flash 写入仍由后台任务完成。
#define MULTITURN_NVS_SAVE_REASON_SERVO_POWER_LOSS 8
// Reset Zero 成功后只替换指定舵机的 Flash 圈数，不能覆盖其他通道。
#define MULTITURN_NVS_SAVE_REASON_SERVO_RESET_ZERO 9

#define MULTITURN_NVS_OPERATION_SAVE 0
#define MULTITURN_NVS_OPERATION_RESET_ZERO 1

#define SERVO_RESET_ZERO_STATUS_IDLE 0
#define SERVO_RESET_ZERO_STATUS_RUNNING 1
#define SERVO_RESET_ZERO_STATUS_SUCCESS 2
#define SERVO_RESET_ZERO_STATUS_FAILED 3

// 顶层系统状态：
// UNCONTROLLABLE: 输出关闭/硬件门禁未满足；IDLE: 仅直控条件满足；
// READY: 反馈门禁满足；RECOVERY_REQUIRED: 可通过 Rope Init 回到 READY。
#define SYSTEM_STATE_UNCONTROLLABLE 0
#define SYSTEM_STATE_IDLE 1
#define SYSTEM_STATE_RUN_FEEDBACK 2
#define SYSTEM_STATE_CALIBRATION_RUNNING 4
#define SYSTEM_STATE_READY 5
#define SYSTEM_STATE_RUN_DIRECT 6
#define SYSTEM_STATE_PAUSED 7
#define SYSTEM_STATE_ROPE_INIT_RUNNING 8
#define SYSTEM_STATE_RECOVERY_REQUIRED 9

#define ROPE_INIT_STATUS_IDLE 0
#define ROPE_INIT_STATUS_RUNNING 1
#define ROPE_INIT_STATUS_SUCCESS 2
#define ROPE_INIT_STATUS_FAILED 3

#define ROPE_INIT_REASON_NONE 0
#define ROPE_INIT_REASON_TIMEOUT 1
#define ROPE_INIT_REASON_FEEDBACK_STALE 2
#define ROPE_INIT_REASON_SERVO_OFFLINE 3
#define ROPE_INIT_REASON_MULTITURN_UNKNOWN 4
#define ROPE_INIT_REASON_MANUAL_REQUIRED 5
#define ROPE_INIT_REASON_AUTO_REQUIRED 6
#define ROPE_INIT_REASON_TARGET_PUBLISH_FAILED 7
#define ROPE_INIT_REASON_SYSTEM_FAULT 8
#define ROPE_INIT_REASON_CANCELLED 9
#define ROPE_INIT_REASON_TASK_START_FAILED 10

#define DIRECT_REJECT_NONE 0
#define DIRECT_REJECT_STATE 1
#define DIRECT_REJECT_HARDWARE 2
#define DIRECT_REJECT_MULTITURN 3
#define DIRECT_REJECT_EVENT_QUEUE 4
#define DIRECT_REJECT_STALE 5
#define DIRECT_REJECT_FAULT 6

// Feedback admission result. CMD_RUN_FEEDBACK is asynchronous: the upper
// computer first queues it, then StateMachineTask publishes the final result.
#define FEEDBACK_ADMISSION_IDLE 0
#define FEEDBACK_ADMISSION_PENDING 1
#define FEEDBACK_ADMISSION_ACCEPTED 2
#define FEEDBACK_ADMISSION_REJECTED 3

#define FEEDBACK_REJECT_NONE 0
#define FEEDBACK_REJECT_MANUAL_REFERENCE 1
#define FEEDBACK_REJECT_STATE 2
#define FEEDBACK_REJECT_WORKSPACE 3
#define FEEDBACK_REJECT_MULTITURN 4
#define FEEDBACK_REJECT_ABS_LIMIT 5
#define FEEDBACK_REJECT_MODE_SWITCH 6
#define FEEDBACK_REJECT_EVENT_QUEUE 7

// 上行故障位图。bit 1 保留旧协议编号，当前固件不再产生 J16 专用假故障。
#define SYSTEM_FAULT_CAN_OFFLINE      (1UL << 0)
#define SYSTEM_FAULT_JOINT16_DUAL     (1UL << 1)
#define SYSTEM_FAULT_OVERLOAD         (1UL << 2)
// Bit 3 is reserved to keep the wire-level fault bitmap stable.
#define SYSTEM_FAULT_SERVO_OFFLINE    (1UL << 4)
#define SYSTEM_FAULT_CALIBRATION      (1UL << 5)
#define SYSTEM_FAULT_MULTITURN_UNKNOWN (1UL << 6)

#define CALIB_DIAG_REASON_NONE 0
#define CALIB_DIAG_REASON_ACTIVE_SCOPE_EMPTY 1
#define CALIB_DIAG_REASON_ENCODER_ERROR 5
#define CALIB_DIAG_REASON_SERVO_OFFLINE 6
#define CALIB_DIAG_REASON_OVERLOAD 8
#define CALIB_DIAG_REASON_ZERO_CROSS_INVALID 13
#define CALIB_DIAG_REASON_WRAP_TOO_LARGE 14
#define CALIB_DIAG_REASON_ABS_LIMIT_INVALID 15
#define CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED 16
#define CALIB_DIAG_REASON_FEEDBACK_LOST 17
#define CALIB_DIAG_REASON_TASK_START_FAILED 18
#define CALIB_DIAG_REASON_CONTROL_LOST 20
#define CALIB_DIAG_REASON_HOLD_TARGET_FAILED 21
#define CALIB_DIAG_REASON_JOINT_LIST_INVALID 22
#define CALIB_DIAG_REASON_DUPLICATE_JOINT 23
#define CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED 24
#define CALIB_DIAG_REASON_JOINT_STEP_FAILED 25
#define CALIB_DIAG_REASON_SERVO_RECOVERY_TIMEOUT 26
#define CALIB_DIAG_REASON_RELEASE_OVER_TWO_TURNS 27

#define CALIB_DIAG_PHASE_NONE 0
#define CALIB_DIAG_PHASE_INIT 1
#define CALIB_DIAG_PHASE_FIND_MAX 2
#define CALIB_DIAG_PHASE_RELEASE 3
#define CALIB_DIAG_PHASE_RETURN_ZERO 4
#define CALIB_DIAG_PHASE_DUAL_PRELOAD 5
#define CALIB_DIAG_PHASE_DUAL_EXCHANGE 6
#define CALIB_DIAG_PHASE_SEQUENCE 7

#define SERVO_TARGET_OWNER_NONE 0
#define SERVO_TARGET_OWNER_CONTROL 1
#define SERVO_TARGET_OWNER_CALIBRATION 2

// TWAI(CAN) pins.
#define TWAI_TX_PIN 47
#define TWAI_RX_PIN 48

// FreeRTOS task priorities.
#define TASK_UPPER_COMM_PRIORITY 2
#define TASK_STATE_MACHINE_PRIORITY 5
#define TASK_CAN_COMM_PRIORITY 3
#define TASK_SERVO_COMM_PRIORITY 3
#define TASK_CONTROL_PRIORITY 4
#define TASK_AUTO_CALIB_PRIORITY 2
#define TASK_MONITOR_PRIORITY 1
#define TASK_NVS_PERSIST_PRIORITY 1

// FreeRTOS task stack sizes.
#define UPPER_COMM_TASK_STACK_SIZE 8192
#define CAN_COMM_TASK_STACK_SIZE 4096
#define STATE_MACHINE_TASK_STACK_SIZE 4096
#define SERVO_COMM_TASK_STACK_SIZE 6144
#define CONTROL_TASK_STACK_SIZE 8192
#define AUTO_CALIB_TASK_STACK_SIZE 8192
#define TASK_MONITOR_STACK_SIZE 4096
#define NVS_PERSIST_TASK_STACK_SIZE 4096

#define SERVO_TARGET_BATCH_MAX SERVO_TOTAL_NUM

struct JointMapItem {
    uint8_t busIndex;
    uint8_t servoID;
};

struct MotorMapItem {
    uint8_t busIndex;
    uint8_t servoID;
};

struct JointActuatorItem {
    uint8_t motorChannel;
    int8_t sign;
    int32_t offset;
    int32_t tensionBias;
};

struct JointActuatorGroup {
    uint8_t count;
    JointActuatorItem actuators[MAX_JOINT_ACTUATORS];
};

extern JointMapItem jointMap[ENCODER_TOTAL_NUM];
extern MotorMapItem motorMap[SERVO_TOTAL_NUM];
extern JointActuatorGroup jointActuatorGroups[ENCODER_TOTAL_NUM];

typedef struct {
    uint16_t encoderValues[ENCODER_TOTAL_NUM];
    uint8_t errorFlags[ENCODER_TOTAL_NUM];
    uint32_t errorBitmap;
    uint32_t timestamp;
    bool isValid;
} RemoteSensorData_t;

typedef struct {
    int16_t angleValues[ENCODER_TOTAL_NUM];
    int16_t outsideCounts[ENCODER_TOTAL_NUM];
    uint8_t validFlags[ENCODER_TOTAL_NUM];
    uint32_t timestamp;
    bool isValid;
} MappedAngleData_t;

typedef struct {
    uint8_t event;
    uint32_t timestamp;
    // Runtime completion events carry the joint command generation they belong
    // to. This prevents an old "done" event from releasing a newer command.
    uint32_t token;
} SystemEvent_t;

typedef struct {
    uint8_t busIndex;
    uint8_t servoId;
    // P4 逻辑多圈 ABS。最终写入 ST3215 前，由舵机通信任务减去该电机
    // 自己的 logicalOffset，并校验本次上电会话的硬件行程。
    int32_t position;
    uint16_t speed;
    uint8_t acc;
} ServoTargetCommand_t;

typedef struct {
    uint8_t count;
    uint8_t source;
    ServoTargetCommand_t commands[SERVO_TARGET_BATCH_MAX];
    uint32_t timestamp;
} ServoTargetBatch_t;

typedef struct {
    float targetAngles[ENCODER_TOTAL_NUM];
    int32_t motorTargetRaw[SERVO_TOTAL_NUM];
    int32_t motorSweepTargetRaw[SERVO_TOTAL_NUM];
    uint32_t motorCommandToken;
    uint32_t motorSweepCommandToken;
    uint32_t jointCommandToken;
    uint32_t feedbackCommandJointMask;
    uint32_t motorDirectCommandGeneration;
    uint8_t motorDirectCommandSource;
    uint8_t controlMode;
} ControlCommandSnapshot_t;

typedef struct {
    uint8_t servoId;
    int16_t position;
    int16_t speed;
    int16_t load;
    uint8_t voltage;
    uint8_t temperature;
} ServoStatus_t;

typedef struct {
    int32_t servoAngles[SERVO_TOTAL_NUM];
    int16_t servoRawPositions[SERVO_TOTAL_NUM];
    // 最近一次真正写入 P4 Flash 的单圈 raw，用于上位机判断舵机掉电后是否偏离快照。
    int16_t servoFlashRawPositions[SERVO_TOTAL_NUM];
    uint8_t onlineStatus[SERVO_TOTAL_NUM];
    uint32_t timestamp;
} ServoAngleData_t;

typedef struct {
    int16_t speed[SERVO_TOTAL_NUM];
    int16_t load[SERVO_TOTAL_NUM];
    // ST3215 Present Current 原始有符号读数；物理单位换算由上位机另行定义。
    int16_t current[SERVO_TOTAL_NUM];
    uint8_t voltage[SERVO_TOTAL_NUM];
    uint8_t temperature[SERVO_TOTAL_NUM];
    uint8_t onlineStatus[SERVO_TOTAL_NUM];
    uint32_t timestamp;
} ServoTelemetryData_t;

typedef struct {
    float targetDeg;
    float magActualDeg;
    float loop1Output;
    int16_t cmdTargetPos;
    uint32_t timestamp;
    uint8_t jointIndex;
    uint8_t valid;
    uint8_t cmdValid;
} JointDebugData_t;

typedef struct {
    uint8_t valid;
    uint8_t reason;
    uint8_t phase;
    uint8_t joint;
    uint8_t motor;
    int32_t valueA;
    int32_t valueB;
    int32_t valueC;
    uint32_t bitmap;
    uint32_t timestamp;
    uint32_t sequence;
} CalibrationDiagData_t;

typedef struct {
    uint8_t values[TACTILE_SUMMARY_BYTES];
    uint8_t fullValues[TACTILE_FULL_BYTES];
    uint8_t seq;
    uint8_t fullSeq;
    uint32_t timestamp;
    uint32_t fullTimestamp;
    bool isValid;
    bool fullValid;
} RemoteTactileData_t;

typedef struct {
    uint32_t requestToken;
    uint32_t validMask;
    int32_t motorAbs[SERVO_TOTAL_NUM];
    int16_t motorRaw[SERVO_TOTAL_NUM];
    int16_t motorTurn[SERVO_TOTAL_NUM];
    uint32_t queuedAtMs;
    uint8_t reason;
    uint8_t turnChanged;
    uint8_t edgeChanged;
    // 普通快照保存与 Reset Zero 原子替换共用队列，用 operation 区分语义。
    uint8_t operation;
} MultiTurnNvsSaveJob_t;

typedef struct {
    uint32_t requestToken;
    uint32_t validMask;
    uint32_t sequence;
    uint32_t queuedAtMs;
    int32_t motorAbs[SERVO_TOTAL_NUM];
    int16_t motorRaw[SERVO_TOTAL_NUM];
    int16_t motorTurn[SERVO_TOTAL_NUM];
    uint32_t completedAtMs;
    uint8_t reason;
    uint8_t success;
    uint8_t turnChanged;
    uint8_t edgeChanged;
    uint8_t operation;
} MultiTurnNvsSaveResult_t;

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
    // 磁编运动碰撞检测由 ControlTask 产生并通过状态包上报：armed 表示已确认
    // 关节进入有效运动，hold 表示已冻结触发瞬间的舵机 ABS；状态变化序号用于立即刷新 UI。
    volatile uint32_t feedback_collision_armed_bitmap;
    volatile uint32_t feedback_collision_hold_bitmap;
    volatile uint32_t feedback_collision_transition_count;
    // A safe inward feedback command received while below currentMin starts
    // Rope Init first, then resumes the latest joint target automatically.
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
    // Calibration-only read expansion. It never changes the host-selected
    // active scope and is cleared when the calibration task exits.
    volatile uint32_t calibration_aux_motor_mask;
    volatile uint32_t active_joint_mask;
    volatile uint32_t multi_turn_unknown_bitmap;
    // 当前上电会话的 hardware ABS 严格越过 Reset Zero 阈值 ±30715 时置位。
    // 该位图不使用 logicalOffset，防止软件坐标平移掩盖真实硬件超圈。
    volatile uint32_t servo_turn_overflow_bitmap;
    volatile uint32_t servo_online_bitmap;
    // 舵机热插拔恢复分为三个独立维度：
    // pending 表示仍需执行扭矩关闭/多圈模式配置；configured 表示本次上电会话
    // 已完成多圈模式配置；failed 表示本次上线周期内恢复事务连续失败。
    // 它们不能与 multi_turn_unknown_bitmap 合并：自动标定允许重建未知圈数，
    // 但绝不允许在驱动恢复和多圈模式配置尚未完成时发送运动目标。
    volatile uint32_t servo_recovery_pending_bitmap;
    volatile uint32_t servo_multiturn_configured_bitmap;
    volatile uint32_t servo_recovery_failed_bitmap;
    // 每台舵机本次上电会话对应的逻辑可达范围。范围由该电机自己的
    // logicalOffset 独立计算，单台掉电/恢复不会平移其他电机。
    volatile uint32_t servo_logical_range_valid_bitmap;
    int32_t servo_logical_range_min[SERVO_TOTAL_NUM];
    int32_t servo_logical_range_max[SERVO_TOTAL_NUM];
    // 最近一次因“逻辑目标换算后超出舵机实际硬件行程”而拒绝的诊断。
    volatile uint32_t servo_range_reject_sequence;
    volatile uint8_t servo_range_reject_motor;
    int32_t servo_range_reject_logical_target;
    int32_t servo_range_reject_hardware_target;
    int32_t servo_range_reject_logical_min;
    int32_t servo_range_reject_logical_max;
    // 只有标定预检通过并准备发送首个舵机目标时才置 1。通信任务据此判断
    // 是否需要暂停会占用总线的离线探测和热插拔配置事务。
    volatile uint8_t calibration_motion_started;
    // 标定成功后由舵机通信任务把当前 power-session ABS 正式登记为可信。
    // 自动标定任务必须等待 result_token 与 request_token 一致后才能报告完成。
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

    // Reset Zero 事务由上位机请求、舵机通信任务执行，最终由 NVS 任务确认。
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
    // 与 multiturn_nvs_save_valid_mask 配套；无效位的数据不得用于恢复或界面比较。
    int16_t multiturn_nvs_snapshot_raw[SERVO_TOTAL_NUM];
    volatile uint32_t host_rx_heartbeat_ms;

    // Manual magnetic-encoder endpoint reference restored from the dedicated
    // NVS A/B snapshot. Motion is gated per active joint scope.
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
