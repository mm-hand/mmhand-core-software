#include "SystemTask.h"
#include "ServoBusManager.h"
#include "AngleSolver.h"
#include "CalibrationTask.h"

#include <string.h>

// SystemTask 模块职责：
// 1) 完成系统级资源初始化（队列、互斥锁、总线与求解器）；
// 2) 维护跨任务共享对象 sharedData；
// 3) 创建并拉起 UpperComm/CanComm/Solver 三个核心任务。

// 上位机标定 UI 状态的一次性回传标志，由 UpperCommTask 读取并清零。
volatile uint8_t g_calibrationUIStatus = 0;

// 全局共享数据中心，保存任务间队列句柄与控制状态。
TaskSharedData_t sharedData;

// 任务句柄用于系统侧管理（当前主要用于可见性与后续扩展）。
TaskHandle_t taskUpperCommHandle = NULL;
TaskHandle_t taskCanCommHandle = NULL;
TaskHandle_t taskSolverHandle = NULL;

// 4 路舵机总线管理器实例，物理接线与 jointMap 对应。
ServoBusManager servoBus0;
ServoBusManager servoBus1;
ServoBusManager servoBus2;
ServoBusManager servoBus3;

// 角度求解与双环控制实例。
AngleSolver angleSolver;

// 主映射：关节索引(0..20) -> 主舵机总线/ID。
JointMapItem jointMap[ENCODER_TOTAL_NUM] = {
    {0, 1},  {0, 2},  {0, 3},  {0, 4},
    {0, 5},  {0, 6},  {1, 7},  {2, 8},
    {1, 9},  {1, 10}, {1, 11}, {1, 12},
    {2, 13}, {2, 14}, {2, 15}, {2, 16},
    {3, 17}, {3, 19}, {3, 20}, {3, 21}, {3, 22}
};

// // 主映射：关节索引(0..20) -> 主舵机总线/ID。
// JointMapItem jointMap[ENCODER_TOTAL_NUM] = {
//     {0, 1},  {0, 2},  {1, 3},  {2, 4},
//     {0, 5},  {0, 6},  {0, 7},  {0, 8},
//     {1, 9},  {1, 10}, {1, 11}, {1, 12},
//     {2, 13}, {2, 14}, {2, 15}, {2, 16},
//     {3, 17}, {3, 19}, {3, 20}, {3, 21}, {3, 22}
// };

// joint16 的副舵机（拮抗控制用）。
MotorMapItem joint16SecondaryMotor = {3, 18};

// 电机通道映射(0..21)：用于22通道直控与舵机角度/遥测上报。
MotorMapItem motorMap[SERVO_TOTAL_NUM] = {
    {0, 1},  {0, 2},  {0, 3},  {0, 4},     // 小拇指
    {0, 5},  {0, 6},  {1, 7},  {2, 8},     // 无名指
    {1, 9},  {1, 10}, {1, 11}, {1, 12},    //  中指
    {2, 13}, {2, 14}, {2, 15}, {2, 16},    //  食指
    {3, 17}, {3, 18}, {3, 19}, {3, 20},
    {3, 21}, {3, 22}
};  // bus id - motor id


// MotorMapItem motorMap[SERVO_TOTAL_NUM] = {
//     {0, 1},  {0, 2},  {1, 3},  {2, 4},     // 小拇指
//     {0, 5},  {0, 6},  {0, 7},  {0, 8},     // 无名指
//     {1, 9},  {1, 10}, {1, 11}, {1, 12},    //  中指
//     {2, 13}, {2, 14}, {2, 15}, {2, 16},    //  食指
//     {3, 17}, {3, 18}, {3, 19}, {3, 20},
//     {3, 21}, {3, 22}
// };  // bus id - motor id

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
    sharedData.canRxQueue = xQueueCreate(1, sizeof(RemoteSensorData_t));
    sharedData.canTxQueue = xQueueCreate(5, sizeof(RemoteCommand_t));
    sharedData.servoAngleQueue = xQueueCreate(1, sizeof(ServoAngleData_t));
    sharedData.servoRawQueue = xQueueCreate(1, sizeof(ServoAngleData_t));
    sharedData.servoTelemetryQueue = xQueueCreate(1, sizeof(ServoTelemetryData_t));
    sharedData.mappedAngleQueue = xQueueCreate(1, sizeof(MappedAngleData_t));
    sharedData.jointDebugQueue = xQueueCreate(8, sizeof(JointDebugData_t));
    sharedData.tactileQueue = xQueueCreate(1, sizeof(RemoteTactileData_t));

    if (!sharedData.cmdQueue || !sharedData.statusQueue ||
        !sharedData.canRxQueue || !sharedData.canTxQueue ||
        !sharedData.servoAngleQueue || !sharedData.servoRawQueue ||
        !sharedData.servoTelemetryQueue || !sharedData.mappedAngleQueue ||
        !sharedData.jointDebugQueue || !sharedData.tactileQueue) {
        // 资源不足时停在此处，避免系统在不完整状态下继续运行。
        while (1) {}
    }

    // 3) 初始化目标角缓存与控制相关状态。
    sharedData.targetAnglesMutex = xSemaphoreCreateMutex();
    if (!sharedData.targetAnglesMutex) {
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

    // 4) 初始化全部舵机总线，保持与现有接线假设一致。
    servoBus0.begin(0, 21, 20, 1000000);
    servoBus1.begin(1, 23, 22, 1000000);
    servoBus2.begin(2, 27, 26, 1000000);
    servoBus3.begin(3, 33, 32, 1000000);

    // 5) 初始化求解器基础参数（默认零点/比例/方向）。
    int16_t zeros[ENCODER_TOTAL_NUM];
    float ratios[ENCODER_TOTAL_NUM];
    int8_t dirs[ENCODER_TOTAL_NUM];
    for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
        zeros[i] = 2048;
        ratios[i] = 1.0f;
        dirs[i] = 1;
    }
    angleSolver.init(zeros, ratios, dirs);

    // 6) 设置双环 PID 参数。
    float pidConfigs[2][PID_PARAMETER_NUM] = {
        {16.5f, 0.0f, 0.5f, 0.0f, 100.0f, 600.0f},
        {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 30719.0f}
    };
    angleSolver.setPIDParams(pidConfigs);

    // 7) 当前测试阶段：禁用自动标定动作，使用手动配置入口。
    initManualCalibrationForTest();

    // 8) 创建上位机通信任务。
    xTaskCreate(
        taskUpperComm,
        "UpperComm",
        UPPER_COMM_TASK_STACK_SIZE,
        &sharedData,
        TASK_UPPER_COMM_PRIORITY,
        &taskUpperCommHandle
    );

    // 9) 创建 CAN 通信任务。
    xTaskCreate(
        taskCanComm,
        "CanComm",
        CAN_COMM_TASK_STACK_SIZE,
        &sharedData,
        TASK_CAN_COMM_PRIORITY,
        &taskCanCommHandle
    );

    // 10) 创建角度求解与执行任务。
    xTaskCreate(
        taskSolver,
        "Solver",
        SOLVER_TASK_STACK_SIZE,
        &sharedData,
        TASK_SOLVER_PRIORITY,
        &taskSolverHandle
    );

    Serial.println("FreeRTOS tasks created successfully.");
    Serial.println("System ready.");
}

// 系统主循环：仅保留轻量节拍，实际业务由各任务并行执行。
void System_Loop() {
    // 主循环保持轻量占位，主要工作在各 FreeRTOS 任务中执行。
    vTaskDelay(pdMS_TO_TICKS(10));
}
