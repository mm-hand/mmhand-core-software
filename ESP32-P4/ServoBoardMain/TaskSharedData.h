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
// 舵机通道数（joint16 使用双舵机）。
#define SERVO_TOTAL_NUM 22
#define JOINT_COUNT ENCODER_TOTAL_NUM
#define NUM_BUSES 4
#define MAX_SERVOS_PER_BUS 8
#define MAX_SERVO_ID 32
#define SERVO_SYNC_READ_TIMEOUT_MS_DEFAULT 3
#define SERVO_TARGET_SPEED_DEFAULT 3000
#define SERVO_TARGET_ACC_DEFAULT 200
#define TACTILE_GROUP_NUM 5
#define TACTILE_SENSOR_PER_GROUP 3
#define TACTILE_AXIS_NUM 3
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

#define SYSTEM_EVENT_NONE 0
#define SYSTEM_EVENT_START 1
#define SYSTEM_EVENT_STOP 2
#define SYSTEM_EVENT_RESET 3
#define SYSTEM_EVENT_CALIBRATE 4
#define SYSTEM_EVENT_CALIBRATION_DONE 5
#define SYSTEM_EVENT_CALIBRATION_FAILED 6

#define SYSTEM_STATE_BOOTING 0
#define SYSTEM_STATE_IDLE 1
#define SYSTEM_STATE_RUNNING 2
#define SYSTEM_STATE_STOPPED 3
#define SYSTEM_STATE_FAULT_HOLD 4
#define SYSTEM_STATE_CALIBRATION_IDLE 5
#define SYSTEM_STATE_CALIBRATION_RUNNING 6
#define SYSTEM_STATE_CALIBRATION_SUCCESS 7
#define SYSTEM_STATE_CALIBRATION_FAILED 8

#define SYSTEM_FAULT_CAN_OFFLINE      (1UL << 0)
#define SYSTEM_FAULT_JOINT16_DUAL     (1UL << 1)
#define SYSTEM_FAULT_OVERLOAD         (1UL << 2)
#define SYSTEM_FAULT_RELEASE_GUARD    (1UL << 3)
#define SYSTEM_FAULT_SERVO_OFFLINE    (1UL << 4)

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
    uint8_t busIndex;
    uint8_t servoID;
};

struct MotorMapItem {
    uint8_t busIndex;
    uint8_t servoID;
};

extern JointMapItem jointMap[ENCODER_TOTAL_NUM];
extern MotorMapItem joint16SecondaryMotor;
extern MotorMapItem motorMap[SERVO_TOTAL_NUM];

typedef struct {
    uint8_t cmdID;
    uint8_t payload[8];
    uint8_t len;
} RemoteCommand_t;

typedef struct {
    uint16_t encoderValues[ENCODER_TOTAL_NUM];
    // 各通道错误类型（来自 CAN 错误详情帧，0x00 表示正常）。
    uint8_t errorFlags[ENCODER_TOTAL_NUM];
    // 派生错误位图：bit i = 1 表示 errorFlags[i] 非 0，便于快速判断。
    uint32_t errorBitmap;
    uint32_t timestamp;
    bool isValid;
} RemoteSensorData_t;

typedef struct {
    int16_t angleValues[ENCODER_TOTAL_NUM];
    uint8_t validFlags[ENCODER_TOTAL_NUM];
    uint32_t timestamp;
    bool isValid;
} MappedAngleData_t;

typedef struct {
    uint8_t cmdType;
    uint8_t servoId;
    int16_t position;
    uint16_t speed;
    uint8_t busIndex;
} ServoCommand_t;

typedef struct {
    uint8_t event;
    uint32_t timestamp;
} SystemEvent_t;

typedef struct {
    uint8_t busIndex;
    uint8_t servoId;
    int16_t position;
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
    uint32_t motorDirectCommandGeneration;
    uint8_t motorDirectCommandSource;
    uint8_t controlMode;
    uint8_t tendonGuardEnabled[ENCODER_TOTAL_NUM];
    int8_t tendonGuardSign[ENCODER_TOTAL_NUM];
    int16_t tendonGuardX1Abs[ENCODER_TOTAL_NUM];
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
    uint8_t onlineStatus[SERVO_TOTAL_NUM];
    uint32_t timestamp;
} ServoAngleData_t;

typedef struct {
    int16_t speed[SERVO_TOTAL_NUM];
    int16_t load[SERVO_TOTAL_NUM];
    uint8_t voltage[SERVO_TOTAL_NUM];
    uint8_t temperature[SERVO_TOTAL_NUM];
    uint8_t onlineStatus[SERVO_TOTAL_NUM];
    uint32_t timestamp;
} ServoTelemetryData_t;

typedef struct {
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
    uint8_t values[TACTILE_SUMMARY_BYTES];
    uint8_t seq;
    uint32_t timestamp;
    bool isValid;
} RemoteTactileData_t;

typedef struct {
    QueueHandle_t cmdQueue;
    QueueHandle_t statusQueue;
    QueueHandle_t stateEventQueue;
    QueueHandle_t canTxQueue;
    QueueHandle_t canRxQueue;
    QueueHandle_t servoTargetQueue;
    QueueHandle_t servoFeedbackQueue;
    QueueHandle_t servoAngleQueue;
    QueueHandle_t servoRawQueue;
    QueueHandle_t servoTelemetryQueue;
    QueueHandle_t servoTelemetrySnapshotQueue;
    QueueHandle_t mappedAngleQueue;
    QueueHandle_t jointDebugQueue;
    QueueHandle_t tactileQueue;

    float targetAngles[ENCODER_TOTAL_NUM];
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
    volatile uint8_t control_mode; // CONTROL_MODE_*
    volatile uint8_t system_state; // SYSTEM_STATE_*
    volatile uint8_t servo_target_owner; // SERVO_TARGET_OWNER_*
    volatile uint32_t system_fault_bitmap;
    volatile uint32_t state_transition_count;
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
