#ifndef TASK_SHARED_DATA_H
#define TASK_SHARED_DATA_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// TaskSharedData 模块职责：
// 1) 定义跨任务共享的数据结构与队列句柄；
// 2) 统一维护控制状态、目标缓存与故障状态位图；
// 3) 作为 UpperComm/CanComm/Solver 等任务的数据交换中心。

// 关节/磁编码器通道数。
#define ENCODER_TOTAL_NUM 21
// 固件预留 27 路电机槽位，让灵巧手 1.0 和 2.0 共用同一套协议。
// 1.0 中 M22-M26 在 HardwareMap.cpp 中标记为 invalid，不读、不控、不参与故障。
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
#define TACTILE_GROUP_NUM 5
#define TACTILE_SENSOR_PER_GROUP 3
#define TACTILE_AXIS_NUM 3
// 触觉上报采用压缩汇总数组，按 group/sensor/axis 展平。
#define TACTILE_SUMMARY_BYTES (TACTILE_GROUP_NUM * TACTILE_SENSOR_PER_GROUP * TACTILE_AXIS_NUM)

// 求解器输出控制模式。
#define CONTROL_MODE_JOINT 0
#define CONTROL_MODE_DIRECT_MOTOR 1

// 直控电机命令来源。
#define MOTOR_DIRECT_SOURCE_NONE 0
#define MOTOR_DIRECT_SOURCE_TARGET 1
#define MOTOR_DIRECT_SOURCE_SWEEP 2
// 上位机下发的多圈绝对位置（与 clampServoPos 一致），由 CMD_MOTOR_POS_ABS 写入 motorTargetRaw。
#define MOTOR_DIRECT_SOURCE_ABSOLUTE 3

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

// 多圈 NVS 保存原因码，仅用于日志/诊断；当前快照是否可信仍由舵机通信任务判断。
// 开启后：上位机离线、运行起止、校准完成、离线运动/跨圈边界都会触发本地快照保存。
#define MULTITURN_NVS_PERSISTENCE_ENABLE 1
#define MULTITURN_NVS_SAVE_REASON_RUN_START 1
#define MULTITURN_NVS_SAVE_REASON_RUN_DONE 2
#define MULTITURN_NVS_SAVE_REASON_HOST_OFFLINE 3
#define MULTITURN_NVS_SAVE_REASON_HOST_RESTORE 4
#define MULTITURN_NVS_SAVE_REASON_CALIB_DONE 5
#define MULTITURN_NVS_SAVE_REASON_OFFLINE_MOVE 6
#define MULTITURN_NVS_SAVE_REASON_OFFLINE_TURN_EDGE 7

#define SYSTEM_STATE_UNCONTROLLABLE 0
#define SYSTEM_STATE_IDLE 1
#define SYSTEM_STATE_RUN_FEEDBACK 2
#define SYSTEM_STATE_CALIBRATION_RUNNING 4
#define SYSTEM_STATE_READY 5
#define SYSTEM_STATE_RUN_DIRECT 6
#define SYSTEM_STATE_PAUSED 7

#define SYSTEM_FAULT_CAN_OFFLINE      (1UL << 0)
#define SYSTEM_FAULT_JOINT16_DUAL     (1UL << 1)
#define SYSTEM_FAULT_OVERLOAD         (1UL << 2)
#define SYSTEM_FAULT_RELEASE_GUARD    (1UL << 3)
#define SYSTEM_FAULT_SERVO_OFFLINE    (1UL << 4)
#define SYSTEM_FAULT_CALIBRATION      (1UL << 5)

#define SERVO_TARGET_OWNER_NONE 0
#define SERVO_TARGET_OWNER_CONTROL 1
#define SERVO_TARGET_OWNER_CALIBRATION 2

// TWAI(CAN) 引脚。
#define TWAI_TX_PIN 47
#define TWAI_RX_PIN 48

// FreeRTOS 任务优先级。
#define TASK_UPPER_COMM_PRIORITY 1
#define TASK_STATE_MACHINE_PRIORITY 2
#define TASK_CAN_COMM_PRIORITY 3
#define TASK_SERVO_COMM_PRIORITY 3
#define TASK_CONTROL_PRIORITY 4
#define TASK_SOLVER_PRIORITY TASK_CONTROL_PRIORITY

// FreeRTOS 任务栈大小（字节）。
#define UPPER_COMM_TASK_STACK_SIZE 8192
#define CAN_COMM_TASK_STACK_SIZE 4096
#define STATE_MACHINE_TASK_STACK_SIZE 4096
#define SERVO_COMM_TASK_STACK_SIZE 6144
#define CONTROL_TASK_STACK_SIZE 8192
#define SOLVER_TASK_STACK_SIZE CONTROL_TASK_STACK_SIZE

#define SERVO_TARGET_BATCH_MAX SERVO_TOTAL_NUM

struct JointMapItem {
    // 物理舵机所在总线索引。
    uint8_t busIndex;
    // 飞特舵机 ID。
    uint8_t servoID;
};

struct MotorMapItem {
    // 直控/遥测 motor 通道对应的物理总线索引。
    uint8_t busIndex;
    // 直控/遥测 motor 通道对应的飞特舵机 ID。
    uint8_t servoID;
};

struct JointActuatorItem {
    // Logical motor channel in motorMap[].
    uint8_t motorChannel;
    // +1: actuator pulls in the same direction as the primary actuator.
    // -1: antagonistic actuator; runtime targets must be built explicitly.
    int8_t sign;
    // Static calibration offset applied after sign mapping.
    int32_t offset;
    // Optional calibration field reserved for preload/tension workflows.
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
    // 预留 CAN 下发命令格式，当前主要为后续扩展保留。
    uint8_t cmdID;
    uint8_t payload[8];
    uint8_t len;
} RemoteCommand_t;

typedef struct {
    // 编码器原始计数，数组下标与 jointIndex 一致。
    uint16_t encoderValues[ENCODER_TOTAL_NUM];
    // 各通道错误类型（来自 CAN 错误详情帧，0x00 表示正常）。
    uint8_t errorFlags[ENCODER_TOTAL_NUM];
    // 派生错误位图：bit i = 1 表示 errorFlags[i] 非 0，便于快速判断。
    uint32_t errorBitmap;
    uint32_t timestamp;
    bool isValid;
} RemoteSensorData_t;

typedef struct {
    // 控制层映射后的编码器计数，已处理方向、零偏和断连状态。
    int16_t angleValues[ENCODER_TOTAL_NUM];
    uint8_t validFlags[ENCODER_TOTAL_NUM];
    uint32_t timestamp;
    bool isValid;
} MappedAngleData_t;

typedef struct {
    // 旧的单舵机命令结构，保留给历史队列/扩展接口。
    uint8_t cmdType;
    uint8_t servoId;
    int16_t position;
    uint16_t speed;
    uint8_t busIndex;
} ServoCommand_t;

typedef struct {
    // 系统事件只描述“想发生什么”，具体状态变更由 StateMachineTask 决定。
    uint8_t event;
    uint32_t timestamp;
} SystemEvent_t;

typedef struct {
    // 单条舵机目标命令，按 bus/id 定位，不按数组下标定位。
    uint8_t busIndex;
    uint8_t servoId;
    int16_t position;
    uint16_t speed;
    uint8_t acc;
} ServoTargetCommand_t;

typedef struct {
    // 一个控制周期内要下发的一组舵机目标。
    uint8_t count;
    // 目标来源必须与 servo_target_owner 匹配才会被执行。
    uint8_t source;
    ServoTargetCommand_t commands[SERVO_TARGET_BATCH_MAX];
    uint32_t timestamp;
} ServoTargetBatch_t;

typedef struct {
    // ControlTask 每周期读取一次的完整命令快照。
    float targetAngles[ENCODER_TOTAL_NUM];
    // direct motor 目标，按 motorMap 0..21 对齐。
    int32_t motorTargetRaw[SERVO_TOTAL_NUM];
    // sweep 模式目标，同样按 motorMap 对齐，语义为单圈 raw。
    int32_t motorSweepTargetRaw[SERVO_TOTAL_NUM];
    uint32_t motorCommandToken;
    uint32_t motorSweepCommandToken;
    uint32_t jointCommandToken;
    uint32_t motorDirectCommandGeneration;
    uint8_t motorDirectCommandSource;
    uint8_t controlMode;
    uint8_t tendonGuardEnabled[ENCODER_TOTAL_NUM];
    int8_t tendonGuardSign[ENCODER_TOTAL_NUM];
    int16_t tendonGuardX1Abs[ENCODER_TOTAL_NUM];
} ControlCommandSnapshot_t;

typedef struct {
    // 历史单舵机状态结构，目前主要为 statusQueue 兼容保留。
    uint8_t servoId;
    int16_t position;
    int16_t speed;
    int16_t load;
    uint8_t voltage;
    uint8_t temperature;
} ServoStatus_t;

typedef struct {
    // 多圈舵机位置和单圈 raw 位置，均按 motorMap 通道顺序排列。
    int32_t servoAngles[SERVO_TOTAL_NUM];
    int16_t servoRawPositions[SERVO_TOTAL_NUM];
    uint8_t onlineStatus[SERVO_TOTAL_NUM];
    uint32_t timestamp;
} ServoAngleData_t;

typedef struct {
    // 舵机遥测数据，按 motorMap 通道顺序排列。
    int16_t speed[SERVO_TOTAL_NUM];
    int16_t load[SERVO_TOTAL_NUM];
    uint8_t voltage[SERVO_TOTAL_NUM];
    uint8_t temperature[SERVO_TOTAL_NUM];
    uint8_t onlineStatus[SERVO_TOTAL_NUM];
    uint32_t timestamp;
} ServoTelemetryData_t;

typedef struct {
    // 上位机调试包使用的单关节 PID/命令状态快照。
    float targetDeg;
    float magActualDeg;
    float loop1Output;
    float loop2Actual;
    float loop2Output;
    int16_t cmdTargetPos;
    uint32_t timestamp;
    uint8_t jointIndex;
    uint8_t valid;
    uint8_t cmdValid;
} JointDebugData_t;

typedef struct {
    // 触觉汇总数据，values 按 TACTILE_SUMMARY_BYTES 展平。
    uint8_t values[TACTILE_SUMMARY_BYTES];
    uint8_t seq;
    uint32_t timestamp;
    bool isValid;
} RemoteTactileData_t;

typedef struct {
    // 历史命令/状态队列，部分接口保留兼容。
    QueueHandle_t cmdQueue;
    QueueHandle_t statusQueue;
    QueueHandle_t stateEventQueue;
    QueueHandle_t canTxQueue;
    QueueHandle_t canRxQueue;
    QueueHandle_t servoTargetQueue;
    // 舵机通信层发布的最新位置反馈，控制层使用 peek 读取。
    QueueHandle_t servoFeedbackQueue;
    // 上位机角度包、raw 包、遥测包分别使用的上报队列。
    QueueHandle_t servoAngleQueue;
    QueueHandle_t servoRawQueue;
    QueueHandle_t servoTelemetryQueue;
    QueueHandle_t servoTelemetrySnapshotQueue;
    // 控制层内部映射角度快照；上位机 raw 编码器上报直接读取 canRxQueue。
    QueueHandle_t mappedAngleQueue;
    QueueHandle_t jointDebugQueue;
    QueueHandle_t tactileQueue;

    // joint 模式目标角缓存；新的写入统一通过 commandStateMutex。
    float targetAngles[ENCODER_TOTAL_NUM];
    // targetAnglesMutex 保留兼容，commandStateMutex 是当前控制命令主锁。
    SemaphoreHandle_t targetAnglesMutex;
    SemaphoreHandle_t commandStateMutex;

    // 直控舵机模式目标缓存，按电机通道索引（0..21）对齐，语义为单圈原始位置计数。
    int32_t motorTargetRaw[SERVO_TOTAL_NUM];
    // 滑条扫动专用目标缓存，保持单圈原始位置语义。
    int32_t motorSweepTargetRaw[SERVO_TOTAL_NUM];
    // 上位机每次下发新的直控电机目标时自增，用于解除热插拔保持锁存。
    volatile uint32_t motor_command_token;
    // 上位机每次下发新的滑条扫动命令时自增。
    volatile uint32_t motor_sweep_command_token;
    // 上位机每次下发新的关节角目标时自增，用于解除热插拔保持锁存。
    volatile uint32_t joint_command_token;
    // 最近一次直控命令的统一序号和来源，用于判定当前由哪种直控语义生效。
    volatile uint32_t motor_direct_command_generation;
    volatile uint8_t motor_direct_command_source;

    volatile uint8_t control_enabled;
    // control_enabled 只由状态机维护，控制任务只读取。
    volatile uint8_t control_mode; // CONTROL_MODE_*
    volatile uint8_t system_state; // SYSTEM_STATE_*
    volatile uint8_t paused_resume_state; // RUN_DIRECT or RUN_FEEDBACK before PAUSED.
    // servo_target_owner 决定舵机通信层执行 CONTROL 还是 CALIBRATION batch。
    volatile uint8_t servo_target_owner; // SERVO_TARGET_OWNER_*
    volatile uint32_t servo_emergency_stop_token;
    volatile uint32_t system_fault_bitmap;
    volatile uint32_t system_fault_latched_bitmap;
    volatile uint32_t state_transition_count;
    volatile uint32_t active_motor_mask;
    volatile uint32_t active_joint_mask;
    // bit i = 1 means motor channel i has lost its trusted multi-turn origin
    // after MCU boot/reset. Normal RUN_DIRECT/RUN_FEEDBACK must reject it until
    // firmware calibration or host-assisted restore confirms the absolute turn.
    volatile uint32_t multi_turn_unknown_bitmap;
    int32_t restore_multi_turn_abs[SERVO_TOTAL_NUM];
    volatile uint32_t restore_multi_turn_mask;
    volatile uint32_t restore_multi_turn_request_token;
    volatile uint32_t restore_multi_turn_result_token;
    volatile uint32_t restore_multi_turn_restored_mask;
    volatile uint32_t restore_multi_turn_rejected_mask;
    // 舵机多圈 ABS 的本地 NVS 备份请求。其它任务只递增 token，不直接写 flash。
    volatile uint32_t multiturn_nvs_save_request_token;
    volatile uint8_t multiturn_nvs_save_reason;
    volatile uint32_t multiturn_nvs_save_result_token;
    volatile uint32_t multiturn_nvs_save_valid_mask;
    volatile uint32_t multiturn_nvs_save_fail_count;
    volatile uint8_t multiturn_nvs_enabled;
    volatile uint8_t multiturn_nvs_snapshot_valid;
    // 每个合法上位机帧都会刷新；界面空闲时由 heartbeat 帧维持在线状态。
    volatile uint32_t host_rx_heartbeat_ms;
    // Calibration-derived absolute servo travel limits. bit i in valid_bitmap
    // means motor_abs_limit_min/max[i] is valid; bit i in locked_bitmap means
    // normal control output is currently constrained to that window.
    volatile uint32_t motor_abs_limit_valid_bitmap;
    volatile uint32_t motor_abs_limit_locked_bitmap;
    int32_t motor_abs_limit_min[SERVO_TOTAL_NUM];
    int32_t motor_abs_limit_max[SERVO_TOTAL_NUM];
    volatile uint8_t require_can_feedback;
    volatile uint8_t calibration_fault_latched;
    volatile uint8_t joint16_dual_feedback_fault;
    // 过载锁存故障位图：bit i 对应电机通道 i。
    volatile uint32_t overload_fault_bitmap;
    // START/RESET 时自增，用于通知清除过载锁存故障。
    volatile uint32_t overload_fault_reset_token;
    // 反绕保护故障位图：bit i 对应关节 i（joint16 在求解器内排除）。
    volatile uint32_t reverse_release_fault_bitmap;
    // START/RESET 时自增，用于通知清除反绕故障锁存。
    volatile uint32_t reverse_release_fault_reset_token;
    volatile uint8_t tendon_guard_enabled[ENCODER_TOTAL_NUM];
    volatile int8_t tendon_guard_sign[ENCODER_TOTAL_NUM];
    volatile int16_t tendon_guard_x1_abs[ENCODER_TOTAL_NUM];

    int32_t calib_zero_raw_cache[ENCODER_TOTAL_NUM];
    volatile uint8_t calib_zero_raw_valid;
} TaskSharedData_t;

#endif // TASK_SHARED_DATA_H
