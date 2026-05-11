#include "SystemTask.h"
#include "../calibration/CalibrationTask.h"
#include "StateMachineTask.h"
#include "../control/ControlTask.h"
#include "../communication/servo/ServoCommunicationTask.h"
#include "../hardware/HardwareMap.h"

#include <string.h>

// SystemTask 模块职责：
// 1) 完成系统级资源初始化（队列、互斥锁、共享状态）；
// 2) 维护跨任务共享对象 sharedData；
// 3) 创建并拉起上位机通信、状态机、CAN、舵机通信和控制任务。

// 上位机标定 UI 状态的一次性回传标志，由 UpperCommTask 读取并清零。
volatile uint8_t g_calibrationUIStatus = 0;

// 全局共享数据中心，保存任务间队列句柄与控制状态。
TaskSharedData_t sharedData;

// 任务句柄用于系统侧管理（当前主要用于可见性与后续扩展）。
TaskHandle_t upperCommTaskHandle = NULL;
TaskHandle_t stateMachineTaskHandle = NULL;
TaskHandle_t canCommTaskHandle = NULL;
TaskHandle_t servoCommTaskHandle = NULL;
TaskHandle_t controlTaskHandle = NULL;

// 系统启动总入口：按“资源初始化 -> 参数初始化 -> 任务创建”顺序执行。
void System_Init() {
    // 1) 串口初始化：用于上位机通信与启动日志输出。
    Serial.begin(921600);
    while (!Serial) {
        delay(10);
    }
    delay(1000);

    // 2) 创建跨任务消息队列。
    sharedData.cmdQueue = xQueueCreate(5, sizeof(ServoCommand_t));
    sharedData.statusQueue = xQueueCreate(3, sizeof(ServoStatus_t));
    sharedData.stateEventQueue = xQueueCreate(8, sizeof(SystemEvent_t));
    sharedData.canRxQueue = xQueueCreate(1, sizeof(RemoteSensorData_t));
    sharedData.canTxQueue = xQueueCreate(5, sizeof(RemoteCommand_t));
    sharedData.servoTargetQueue = xQueueCreate(1, sizeof(ServoTargetBatch_t));
    sharedData.servoFeedbackQueue = xQueueCreate(1, sizeof(ServoAngleData_t));
    sharedData.servoAngleQueue = xQueueCreate(1, sizeof(ServoAngleData_t));
    sharedData.servoRawQueue = xQueueCreate(1, sizeof(ServoAngleData_t));
    sharedData.servoTelemetryQueue = xQueueCreate(1, sizeof(ServoTelemetryData_t));
    sharedData.servoTelemetrySnapshotQueue = xQueueCreate(1, sizeof(ServoTelemetryData_t));
    sharedData.mappedAngleQueue = xQueueCreate(1, sizeof(MappedAngleData_t));
    sharedData.jointDebugQueue = xQueueCreate(8, sizeof(JointDebugData_t));
    sharedData.tactileQueue = xQueueCreate(1, sizeof(RemoteTactileData_t));

    if (!sharedData.cmdQueue || !sharedData.statusQueue ||
        !sharedData.stateEventQueue ||
        !sharedData.canRxQueue || !sharedData.canTxQueue ||
        !sharedData.servoTargetQueue || !sharedData.servoFeedbackQueue ||
        !sharedData.servoAngleQueue || !sharedData.servoRawQueue ||
        !sharedData.servoTelemetryQueue || !sharedData.servoTelemetrySnapshotQueue ||
        !sharedData.mappedAngleQueue ||
        !sharedData.jointDebugQueue || !sharedData.tactileQueue) {
        // 资源不足时停在此处，避免系统在不完整状态下继续运行。
        while (1) {}
    }

    // 3) 初始化目标角缓存与控制相关状态。
    sharedData.targetAnglesMutex = xSemaphoreCreateMutex();
    sharedData.commandStateMutex = xSemaphoreCreateMutex();
    if (!sharedData.targetAnglesMutex || !sharedData.commandStateMutex) {
        // 互斥锁创建失败同样进入安全停机。
        while (1) {}
    }
    memset(sharedData.targetAngles, 0, sizeof(sharedData.targetAngles));
    memset(sharedData.motorTargetRaw, 0, sizeof(sharedData.motorTargetRaw));
    memset(sharedData.motorSweepTargetRaw, 0, sizeof(sharedData.motorSweepTargetRaw));
    memset(sharedData.calib_zero_raw_cache, 0, sizeof(sharedData.calib_zero_raw_cache));
    sharedData.motor_command_token = 0;
    sharedData.motor_sweep_command_token = 0;
    sharedData.joint_command_token = 0;
    sharedData.motor_direct_command_generation = 0;
    sharedData.motor_direct_command_source = MOTOR_DIRECT_SOURCE_NONE;
    sharedData.control_enabled = 0;
    sharedData.control_mode = CONTROL_MODE_JOINT;
    sharedData.system_state = SYSTEM_STATE_BOOTING;
    sharedData.servo_target_owner = SERVO_TARGET_OWNER_NONE;
    sharedData.servo_emergency_stop_token = 0;
    sharedData.system_fault_bitmap = 0;
    sharedData.state_transition_count = 0;
    sharedData.joint16_dual_feedback_fault = 0;
    sharedData.overload_fault_bitmap = 0;
    sharedData.overload_fault_reset_token = 0;
    sharedData.reverse_release_fault_bitmap = 0;
    sharedData.reverse_release_fault_reset_token = 0;
    memset((void*)sharedData.tendon_guard_enabled, 0, sizeof(sharedData.tendon_guard_enabled));
    memset((void*)sharedData.tendon_guard_x1_abs, 0, sizeof(sharedData.tendon_guard_x1_abs));
    for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
        sharedData.tendon_guard_sign[i] = 1;
    }
    sharedData.calib_zero_raw_valid = 0;


    // 4) 当前测试阶段：禁用自动标定动作，使用手动配置入口。
    initManualCalibrationForTest();

    // 5) 创建上位机通信任务：负责串口协议解析与遥测上报。
    xTaskCreate(
        upperCommunicationTask,
        "upperCommunicationTask",
        UPPER_COMM_TASK_STACK_SIZE,
        &sharedData,
        TASK_UPPER_COMM_PRIORITY,
        &upperCommTaskHandle
    );

    // 6) 创建状态机任务：统一维护 control_enabled、owner、system_state。
    xTaskCreate(
        stateMachineTask,
        "stateMachineTask",
        STATE_MACHINE_TASK_STACK_SIZE,
        &sharedData,
        TASK_STATE_MACHINE_PRIORITY,
        &stateMachineTaskHandle
    );

    // 7) 创建 CAN 通信任务：接收编码器、触觉和错误详情帧。
    xTaskCreate(
        canCommunicationTask,
        "canCommunicationTask",
        CAN_COMM_TASK_STACK_SIZE,
        &sharedData,
        TASK_CAN_COMM_PRIORITY,
        &canCommTaskHandle
    );

    // 8) 创建舵机通信任务：唯一直接访问 FTServo/SMS_STS 与 UART 的任务。
    xTaskCreate(
        servoCommunicationTask,
        "servoCommunicationTask",
        SERVO_COMM_TASK_STACK_SIZE,
        &sharedData,
        TASK_SERVO_COMM_PRIORITY,
        &servoCommTaskHandle
    );

    // 9) 创建实时控制任务：读取快照和反馈，生成舵机目标 batch。
    xTaskCreate(
        controlTask,
        "controlTask",
        CONTROL_TASK_STACK_SIZE,
        &sharedData,
        TASK_CONTROL_PRIORITY,
        &controlTaskHandle
    );

    Serial.println("FreeRTOS tasks created successfully.");
    Serial.println("System ready.");
}

// 系统主循环：仅保留轻量节拍，实际业务由各任务并行执行。
void System_Loop() {
    // 主循环保持轻量占位，主要工作在各 FreeRTOS 任务中执行。
    vTaskDelay(pdMS_TO_TICKS(10));
}
