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

// 求解器输出控制模式。
#define CONTROL_MODE_JOINT 0
#define CONTROL_MODE_DIRECT_MOTOR 1

// TWAI(CAN) 引脚。
#define TWAI_TX_PIN 47
#define TWAI_RX_PIN 48

// FreeRTOS 任务优先级。
#define TASK_UPPER_COMM_PRIORITY 1
#define TASK_CAN_COMM_PRIORITY 3
#define TASK_SOLVER_PRIORITY 4

// FreeRTOS 任务栈大小（字节）。
#define UPPER_COMM_TASK_STACK_SIZE 8192
#define CAN_COMM_TASK_STACK_SIZE 4096
#define SOLVER_TASK_STACK_SIZE 8192

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
    uint8_t servoId;
    int16_t position;
    int16_t speed;
    int16_t load;
    uint8_t voltage;
    uint8_t temperature;
} ServoStatus_t;

typedef struct {
    int32_t servoAngles[SERVO_TOTAL_NUM];
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
    QueueHandle_t cmdQueue;
    QueueHandle_t statusQueue;
    QueueHandle_t canTxQueue;
    QueueHandle_t canRxQueue;
    QueueHandle_t servoAngleQueue;
    QueueHandle_t servoTelemetryQueue;
    QueueHandle_t mappedAngleQueue;
    QueueHandle_t jointDebugQueue;

    float targetAngles[ENCODER_TOTAL_NUM];
    SemaphoreHandle_t targetAnglesMutex;

    // 直控舵机模式目标缓存，按电机通道索引（0..21）对齐。
    int32_t motorTargetRaw[SERVO_TOTAL_NUM];

    volatile uint8_t control_enabled;
    volatile uint8_t control_mode; // CONTROL_MODE_*
    volatile uint8_t joint16_dual_feedback_fault;
    // 过载锁存故障位图：bit i 对应电机通道 i。
    volatile uint32_t overload_fault_bitmap;
    // START/RESET 时自增，用于通知清除过载锁存故障。
    volatile uint32_t overload_fault_reset_token;
    // 反绕保护故障位图：bit i 对应关节 i（joint16 在求解器内排除）。
    volatile uint32_t reverse_release_fault_bitmap;
    // START/RESET 时自增，用于通知清除反绕故障锁存。
    volatile uint32_t reverse_release_fault_reset_token;

    int32_t calib_zero_raw_cache[ENCODER_TOTAL_NUM];
    volatile uint8_t calib_zero_raw_valid;
} TaskSharedData_t;

#endif // TASK_SHARED_DATA_H
