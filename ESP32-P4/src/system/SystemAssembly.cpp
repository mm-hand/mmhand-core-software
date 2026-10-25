/**
 * @file SystemAssembly.cpp
 * @brief 创建系统共享对象、队列、互斥锁，并装配各 FreeRTOS 任务。
 */
#include "SystemAssembly.h"
#include "AppBootstrap.h"
#include "calibration/CalibrationRuntimeStore.h"
#include "../tasks/s3_can_rx/S3CanRxTask.h"
#include "../tasks/host_comm/HostCommTask.h"
#include "log/FirmwareLog.h"
#include "../tasks/system_state/SystemStateTask.h"
#include "../tasks/runtime_monitor/RuntimeMonitorTask.h"
#include "../tasks/motion_control/MotionControlTask.h"
#include "../tasks/servo_bus/ServoBusTask.h"
#include "../config/topology/HardwareMap.h"
#include "../config/ServoConfig.h"
#include "../data/ServoRouteTypes.h"
#include "calibration/ManualEncoderReferenceStore.h"
#include "../config/protocol/HostProtocol.h"
#include "rtos/TaskFactory.h"
#include "SystemRuntimeContext.h"
#include "../safety/SafetyEventReporter.h"
#include "../safety/policies/RuntimeSafetyPolicy.h"
#include "TaskRuntimeMetrics.h"

#include <string.h>

namespace {

// 所有业务任务共享的唯一运行时状态；静态对象在进入装配函数前已完成全字段零初始化。
SystemRuntimeContext sharedData = {};

// 任务句柄只用于运行时监控注册，不作为跨模块控制接口。
TaskHandle_t hostCommTaskHandle = NULL;
TaskHandle_t systemStateTaskHandle = NULL;
TaskHandle_t canCommTaskHandle = NULL;
TaskHandle_t servoCommTaskHandle = NULL;
TaskHandle_t motionControlTaskHandle = NULL;
TaskHandle_t taskMonitorTaskHandle = NULL;

/** 启动阶段尚无业务 ServiceOutputPort，资源失败直接写入同一安全事件缓存。 */
void recordAssemblyFailure(SafetyRuleId ruleId,
                           int32_t requestedValue,
                           int32_t observedValue)
{
    p4::safety::reportSafetyRuleEvent(&sharedData,
                                      ruleId,
                                      requestedValue,
                                      observedValue);
}

} // namespace

bool assembleAndStartSystem() {
    // 容量为 1 的队列用 Peek/Overwrite 表示“最新快照”；
    // 事件队列保留少量历史，由消费者顺序清空。
    sharedData.stateEventQueue.create(8);
    sharedData.canRxQueue.create(1);
    sharedData.servoTargetQueue.create(1);
    sharedData.servoFeedbackQueue.create(1);
    sharedData.servoTelemetryQueue.create(1);
    sharedData.mappedAngleQueue.create(1);
    sharedData.controlTraceQueue.create(32);
    sharedData.tactileQueue.create(3);

    const bool queuesCreated =
        sharedData.stateEventQueue.valid() &&
        sharedData.canRxQueue.valid() &&
        sharedData.servoTargetQueue.valid() &&
        sharedData.servoFeedbackQueue.valid() &&
        sharedData.servoTelemetryQueue.valid() &&
        sharedData.mappedAngleQueue.valid() &&
        sharedData.controlTraceQueue.valid() &&
        sharedData.tactileQueue.valid();
    const SafetyRuleId queueRule = p4::safety::runtimeResourceCreationRule(
        p4::safety::RuntimeResourceKind::Queue,
        queuesCreated);
    if (queueRule != SafetyRuleId::None) {
        FW_LOG_ERROR_PRINTLN("[SYSTEM] shared queue creation failed");
        recordAssemblyFailure(queueRule, 8, 0);
        return false;
    }

    // 目标角、直控命令和暂存命令必须在同一临界区内成组更新。
    sharedData.commandStateMutex = xSemaphoreCreateMutex();
    const SafetyRuleId mutexRule = p4::safety::runtimeResourceCreationRule(
        p4::safety::RuntimeResourceKind::Mutex,
        sharedData.commandStateMutex != nullptr);
    if (mutexRule != SafetyRuleId::None) {
        FW_LOG_ERROR_PRINTLN("[SYSTEM] command mutex creation failed");
        recordAssemblyFailure(mutexRule, 1, 0);
        return false;
    }
    // 其余命令数组、token、位图和计数器均保持静态零初始化；这里只声明非零或有枚举语义的默认值。
    sharedData.motor_direct_command_source = MOTOR_DIRECT_SOURCE_NONE;
    // 直控命令先写入 pending 区，状态机通过后再原子提升为 applied 命令。
    sharedData.pending_direct_source = MOTOR_DIRECT_SOURCE_NONE;
    sharedData.direct_reject_reason = DIRECT_REJECT_NONE;

    // 上电默认不允许任何任务拥有舵机输出；状态机完成硬件/标定门禁后再进入 IDLE/READY。
    sharedData.control_mode = CONTROL_MODE_JOINT;
    sharedData.system_state = SYSTEM_STATE_UNCONTROLLABLE;
    sharedData.servo_target_owner = SERVO_TARGET_OWNER_NONE;
    sharedData.feedback_admission_status = FEEDBACK_ADMISSION_IDLE;
    sharedData.feedback_reject_reason = FEEDBACK_REJECT_NONE;
    // E-STOP 使用单调 token 绕过普通事件队列，队列满时也不会丢急停请求。
    // 上位机控制租约与 Hold 确认从空闲代次开始；直控、负反馈、自动标定和
    // Rope Init 共用这组事务字段，避免各运动任务维护彼此不一致的看门狗。
    sharedData.host_control_hold_status = HOST_CONTROL_HOLD_IDLE;
    sharedData.rope_init_status = ROPE_INIT_STATUS_IDLE;
    sharedData.rope_init_reason = ROPE_INIT_REASON_NONE;

    // 默认活动范围为 J08-J11；上位机连接后会按 UI 选择覆盖这两个 mask。
    const uint32_t validMotorMask = validMotorChannelMask();
    sharedData.active_motor_mask = 0;
    sharedData.active_joint_mask = 0;
    // 所有有效舵机上电先视为多圈未知，只有桌面 JSON 恢复或重新标定后才能清位。
    sharedData.multi_turn_unknown_bitmap = validMotorMask;
    memset((void*)sharedData.servo_route_bus, SERVO_ROUTE_BUS_UNKNOWN,
           sizeof(sharedData.servo_route_bus));
    // 上电时尚未收到任何舵机反馈，因此没有 pending 事务，也没有已配置电机。
    // 舵机连续上线两帧后，ServoBusTask 会建立 pending 并完成恢复。

    sharedData.host_rx_heartbeat_ms = millis();

    // Manual 磁编参考由桌面 JSON 连接后显式下发；缺失时仅禁止反馈/标定，不禁止直控。
    sharedData.manual_reference_commit_status = MANUAL_SAVE_STATUS_NONE;
    sharedData.require_can_feedback = 1;

    // Auto 标定结果先初始化 RAM；Calib Load 或本次标定会再写入有效结果。
    initializeCalibrationRuntimeState();

    // 每次上电清空 P4 RAM，避免跨设备保留陈旧 Manual；桌面端会从 JSON 重放。
    manualEncoderReferencesClearRam();
    sharedData.manual_reference_committed = 0;
    FW_LOG_INFO_PRINTLN(
        "[MANUAL RAM] waiting for desktop JSON restore");

    // 任务按“通信与采样 -> 状态仲裁 -> 控制 -> 监控”职责拆分。
    bool tasksCreated = p4::platform::rtos::createTask(
        {hostCommTask,
         "hostCommTask",
         HOST_COMM_TASK_STACK_SIZE,
         &sharedData,
         TASK_HOST_COMM_PRIORITY},
        &hostCommTaskHandle);

    tasksCreated = p4::platform::rtos::createTask(
                       {systemStateTask,
                        "systemStateTask",
                        SYSTEM_STATE_TASK_STACK_SIZE,
                        &sharedData,
                        TASK_SYSTEM_STATE_PRIORITY},
                       &systemStateTaskHandle) &&
                   tasksCreated;

    tasksCreated = p4::platform::rtos::createTask(
                       {s3CanRxTask,
                        "s3CanRxTask",
                        S3_CAN_RX_TASK_STACK_SIZE,
                        &sharedData,
                        TASK_S3_CAN_RX_PRIORITY},
                       &canCommTaskHandle) &&
                   tasksCreated;

    tasksCreated = p4::platform::rtos::createTask(
                       {servoBusTask,
                        "servoBusTask",
                        SERVO_BUS_TASK_STACK_SIZE,
                        &sharedData,
                        TASK_SERVO_BUS_PRIORITY},
                       &servoCommTaskHandle) &&
                   tasksCreated;

    tasksCreated = p4::platform::rtos::createTask(
                       {motionControlTask,
                        "motionControlTask",
                        MOTION_CONTROL_TASK_STACK_SIZE,
                        &sharedData,
                        TASK_MOTION_CONTROL_PRIORITY},
                       &motionControlTaskHandle) &&
                   tasksCreated;

    const SafetyRuleId tasksRule = p4::safety::runtimeResourceCreationRule(
        p4::safety::RuntimeResourceKind::Task,
        tasksCreated);
    if (tasksRule != SafetyRuleId::None) {
        FW_LOG_ERROR_PRINTLN("[SYSTEM] critical task creation failed; halt");
        recordAssemblyFailure(tasksRule, 6, 0);
        return false;
    }

    // 注册业务任务后，TaskRuntimeMonitor 才能统计周期、执行时间和栈余量。
    taskRuntimeRegister(TASK_RUNTIME_HOST_COMM, hostCommTaskHandle, HOST_COMM_TASK_STACK_SIZE);
    taskRuntimeRegister(TASK_RUNTIME_SYSTEM_STATE, systemStateTaskHandle, SYSTEM_STATE_TASK_STACK_SIZE);
    taskRuntimeRegister(TASK_RUNTIME_S3_CAN_RX, canCommTaskHandle, S3_CAN_RX_TASK_STACK_SIZE);
    taskRuntimeRegister(TASK_RUNTIME_SERVO_BUS, servoCommTaskHandle, SERVO_BUS_TASK_STACK_SIZE);
    taskRuntimeRegister(TASK_RUNTIME_MOTION_CONTROL, motionControlTaskHandle, MOTION_CONTROL_TASK_STACK_SIZE);

    const bool monitorCreated = p4::platform::rtos::createTask(
        {runtimeMonitorTask,
         "taskRuntimeMonitor",
         TASK_MONITOR_STACK_SIZE,
         &sharedData,
         TASK_MONITOR_PRIORITY},
        &taskMonitorTaskHandle);
    const SafetyRuleId monitorTaskRule =
        p4::safety::runtimeResourceCreationRule(
            p4::safety::RuntimeResourceKind::Task,
            monitorCreated);
    if (monitorTaskRule != SafetyRuleId::None) {
        FW_LOG_ERROR_PRINTLN("[SYSTEM] runtime monitor task creation failed");
        recordAssemblyFailure(monitorTaskRule,
                              TASK_RUNTIME_MONITOR,
                              0);
        return false;
    }
    taskRuntimeRegister(TASK_RUNTIME_MONITOR, taskMonitorTaskHandle, TASK_MONITOR_STACK_SIZE);

    FW_LOG_INFO_PRINTLN("FreeRTOS tasks created successfully.");
    FW_LOG_INFO_PRINTLN("System ready.");
    return true;
}
