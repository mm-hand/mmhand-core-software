#include "SystemTask.h"
#include "../calibration/CalibrationTask.h"
#include "../communication/can/CanCommTask.h"
#include "../communication/upper/UpperCommTask.h"
#include "FirmwareLog.h"
#include "StateMachineTask.h"
#include "TaskRuntimeMonitor.h"
#include "../control/ControlTask.h"
#include "../communication/servo/ServoCommunicationTask.h"
#include "../hardware/HardwareMap.h"
#include "../persistence/MultiTurnNvsPersistenceTask.h"
#include "../persistence/ManualCalibrationNvsStore.h"
#include "../communication/upper/UpperCommProtocol.h"
#include "../shared/TaskSharedData.h"

#include <string.h>

// SystemTask 负责一次性初始化共享资源、恢复持久化数据并创建 FreeRTOS 任务。
// 持续随 SYSTEM_STATUS 上报的标定状态。
volatile uint8_t g_calibrationStatus = CALIB_STATUS_IDLE;
// 需要通过 CALIB_ACK 重发的瞬时标定结果及其剩余重发次数。
volatile uint8_t g_calibrationAckStatus = CALIB_STATUS_IDLE;
volatile uint8_t g_calibrationAckRepeatRemaining = 0;

namespace {

// 所有业务任务共享的唯一运行时状态；只在本文件创建，通过任务参数传入各模块。
TaskSharedData_t sharedData;

// 任务句柄只用于运行时监控注册，不作为跨模块控制接口。
TaskHandle_t upperCommTaskHandle = NULL;
TaskHandle_t stateMachineTaskHandle = NULL;
TaskHandle_t canCommTaskHandle = NULL;
TaskHandle_t servoCommTaskHandle = NULL;
TaskHandle_t controlTaskHandle = NULL;
TaskHandle_t taskMonitorTaskHandle = NULL;
TaskHandle_t nvsPersistTaskHandle = NULL;

} // namespace

void System_Init() {
    // 上位机二进制协议与固件日志共用该串口，波特率必须与 desktop 保持一致。
    Serial.begin(921600);
    while (!Serial) {
        delay(10);
    }
    delay(1000);

    // 容量为 1 的队列用 Peek/Overwrite 表示“最新快照”；
    // 事件与调试队列保留少量历史，由消费者顺序清空。
    sharedData.stateEventQueue = xQueueCreate(8, sizeof(SystemEvent_t));
    sharedData.canRxQueue = xQueueCreate(1, sizeof(RemoteSensorData_t));
    sharedData.servoTargetQueue = xQueueCreate(1, sizeof(ServoTargetBatch_t));
    sharedData.servoFeedbackQueue = xQueueCreate(1, sizeof(ServoAngleData_t));
    sharedData.servoAngleQueue = xQueueCreate(1, sizeof(ServoAngleData_t));
    sharedData.servoRawQueue = xQueueCreate(1, sizeof(ServoAngleData_t));
    sharedData.servoTelemetryQueue = xQueueCreate(1, sizeof(ServoTelemetryData_t));
    sharedData.servoTelemetrySnapshotQueue = xQueueCreate(1, sizeof(ServoTelemetryData_t));
    sharedData.mappedAngleQueue = xQueueCreate(1, sizeof(MappedAngleData_t));
    sharedData.jointDebugQueue = xQueueCreate(8, sizeof(JointDebugData_t));
    sharedData.tactileQueue = xQueueCreate(3, sizeof(RemoteTactileData_t));
    sharedData.multiturnNvsSaveQueue = xQueueCreate(1, sizeof(MultiTurnNvsSaveJob_t));
    sharedData.multiturnNvsResultQueue = xQueueCreate(1, sizeof(MultiTurnNvsSaveResult_t));

    if (!sharedData.stateEventQueue ||
        !sharedData.canRxQueue ||
        !sharedData.servoTargetQueue || !sharedData.servoFeedbackQueue ||
        !sharedData.servoAngleQueue || !sharedData.servoRawQueue ||
        !sharedData.servoTelemetryQueue || !sharedData.servoTelemetrySnapshotQueue ||
        !sharedData.mappedAngleQueue ||
        !sharedData.jointDebugQueue || !sharedData.tactileQueue ||
        !sharedData.multiturnNvsSaveQueue || !sharedData.multiturnNvsResultQueue) {
        while (1) {}
    }

    // 目标角、直控命令和暂存命令必须在同一临界区内成组更新。
    sharedData.commandStateMutex = xSemaphoreCreateMutex();
    if (!sharedData.commandStateMutex) {
        while (1) {}
    }
    // 命令缓冲区先清零；后续 token 递增用于识别新命令并丢弃旧完成事件。
    memset(sharedData.targetAngles, 0, sizeof(sharedData.targetAngles));
    memset(sharedData.motorTargetRaw, 0, sizeof(sharedData.motorTargetRaw));
    memset(sharedData.motorSweepTargetRaw, 0, sizeof(sharedData.motorSweepTargetRaw));
    memset(sharedData.pendingMotorTargetRaw, 0, sizeof(sharedData.pendingMotorTargetRaw));
    memset(sharedData.servo_logical_range_min,
           0,
           sizeof(sharedData.servo_logical_range_min));
    memset(sharedData.servo_logical_range_max,
           0,
           sizeof(sharedData.servo_logical_range_max));
    sharedData.servo_logical_range_valid_bitmap = 0;
    sharedData.servo_range_reject_sequence = 0;
    sharedData.servo_range_reject_motor = 0xFF;
    sharedData.servo_range_reject_logical_target = 0;
    sharedData.servo_range_reject_hardware_target = 0;
    sharedData.servo_range_reject_logical_min = 0;
    sharedData.servo_range_reject_logical_max = 0;
    memset(sharedData.pendingMotorSweepTargetRaw, 0, sizeof(sharedData.pendingMotorSweepTargetRaw));
    memset(sharedData.restore_multi_turn_abs, 0, sizeof(sharedData.restore_multi_turn_abs));

    sharedData.motor_command_token = 0;
    sharedData.motor_sweep_command_token = 0;
    sharedData.joint_command_token = 0;
    sharedData.feedback_command_joint_mask = 0;
    sharedData.motor_direct_command_generation = 0;
    sharedData.motor_direct_command_source = MOTOR_DIRECT_SOURCE_NONE;
    // 直控命令先写入 pending 区，状态机通过后再原子提升为 applied 命令。
    sharedData.pending_direct_generation = 0;
    sharedData.applied_direct_generation = 0;
    sharedData.pending_direct_source = MOTOR_DIRECT_SOURCE_NONE;
    sharedData.pending_direct_valid = 0;
    sharedData.pending_direct_motor_mask = 0;
    sharedData.pending_direct_joint_mask = 0;
    sharedData.pending_direct_require_can = 0;
    sharedData.direct_reject_reason = DIRECT_REJECT_NONE;

    // 上电默认不允许任何任务拥有舵机输出；状态机完成硬件/标定门禁后再进入 IDLE/READY。
    sharedData.control_enabled = 0;
    sharedData.control_mode = CONTROL_MODE_JOINT;
    sharedData.system_state = SYSTEM_STATE_UNCONTROLLABLE;
    sharedData.paused_resume_state = SYSTEM_STATE_IDLE;
    sharedData.servo_target_owner = SERVO_TARGET_OWNER_NONE;
    sharedData.feedback_admission_sequence = 0;
    sharedData.feedback_admission_status = FEEDBACK_ADMISSION_IDLE;
    sharedData.feedback_reject_reason = FEEDBACK_REJECT_NONE;
    sharedData.feedback_collision_armed_bitmap = 0;
    sharedData.feedback_collision_hold_bitmap = 0;
    sharedData.feedback_collision_transition_count = 0;
    sharedData.feedback_recovery_pending = 0;
    sharedData.feedback_recovery_request_sequence = 0;
    // E-STOP 使用单调 token 绕过普通事件队列，队列满时也不会丢急停请求。
    sharedData.emergency_stop_request_token = 0;
    sharedData.servo_emergency_stop_token = 0;
    sharedData.system_fault_bitmap = 0;
    sharedData.system_fault_latched_bitmap = 0;
    sharedData.state_transition_count = 0;
    sharedData.rope_init_status = ROPE_INIT_STATUS_IDLE;
    sharedData.rope_init_reason = ROPE_INIT_REASON_NONE;
    sharedData.rope_init_sequence = 0;
    sharedData.rope_init_joint_mask = 0;
    sharedData.rope_init_timestamp = 0;

    // 默认活动范围为 J08-J11；上位机连接后会按 UI 选择覆盖这两个 mask。
    const uint32_t validMotorMask = validMotorChannelMask();
    sharedData.active_motor_mask = ((1UL << 8) | (1UL << 9) | (1UL << 10) | (1UL << 11)) & validMotorMask;
    sharedData.calibration_aux_motor_mask = 0;
    sharedData.active_joint_mask = ((1UL << 8) | (1UL << 9) | (1UL << 10) | (1UL << 11));
    // 所有有效舵机上电先视为多圈未知，只有 NVS 恢复或重新标定后才能清位。
    sharedData.multi_turn_unknown_bitmap = validMotorMask;
    sharedData.servo_online_bitmap = 0;
    // 上电时尚未收到任何舵机反馈，因此没有 pending 事务，也没有已配置电机。
    // 舵机连续上线两帧后，ServoCommunicationTask 会建立 pending 并完成恢复。
    sharedData.servo_recovery_pending_bitmap = 0;
    sharedData.servo_multiturn_configured_bitmap = 0;
    sharedData.servo_recovery_failed_bitmap = 0;
    sharedData.calibration_motion_started = 0;
    sharedData.calibration_trust_request_mask = 0;
    sharedData.calibration_trust_request_token = 0;
    sharedData.calibration_trust_result_token = 0;
    sharedData.calibration_trust_accepted_mask = 0;
    sharedData.encoder_online_bitmap = 0;
    sharedData.encoder_warn_bitmap = 0;
    sharedData.encoder_fault_bitmap = 0;
    sharedData.can_link_online = 0;
    sharedData.restore_multi_turn_mask = 0;
    sharedData.restore_multi_turn_request_token = 0;
    sharedData.restore_multi_turn_result_token = 0;
    sharedData.restore_multi_turn_restored_mask = 0;
    sharedData.restore_multi_turn_rejected_mask = 0;

    // 多圈 NVS 保存由独立低优先级任务执行，控制任务只递增请求 token。
    sharedData.multiturn_nvs_save_request_token = 0;
    sharedData.multiturn_nvs_save_reason = 0;
    sharedData.multiturn_nvs_save_result_token = 0;
    sharedData.multiturn_nvs_save_valid_mask = 0;
    sharedData.multiturn_nvs_save_fail_count = 0;
    sharedData.multiturn_nvs_enabled = 1;
    sharedData.multiturn_nvs_snapshot_valid = 0;
    memset(sharedData.multiturn_nvs_snapshot_raw,
           0,
           sizeof(sharedData.multiturn_nvs_snapshot_raw));
    sharedData.host_rx_heartbeat_ms = millis();

    // Manual 磁编参考必须从 P4 NVS 恢复；缺失时仅禁止反馈/标定，不禁止直控。
    sharedData.manual_reference_valid_bitmap = 0;
    sharedData.manual_reference_crc = 0;
    sharedData.manual_reference_nvs_sequence = 0;
    sharedData.manual_reference_nvs_valid = 0;
    sharedData.manual_reference_save_status = MANUAL_SAVE_STATUS_NONE;
    sharedData.motor_abs_limit_valid_bitmap = 0;
    sharedData.motor_abs_limit_locked_bitmap = 0;
    memset(sharedData.motor_abs_limit_min, 0, sizeof(sharedData.motor_abs_limit_min));
    memset(sharedData.motor_abs_limit_max, 0, sizeof(sharedData.motor_abs_limit_max));
    sharedData.require_can_feedback = 1;
    sharedData.calibration_fault_latched = 0;
    sharedData.calibration_warning_bitmap = 0;
    memset(sharedData.calibration_auto_min_offset_counts, 0,
           sizeof(sharedData.calibration_auto_min_offset_counts));
    memset(&sharedData.calibration_diag, 0, sizeof(sharedData.calibration_diag));

    // 过载保护保存触发电机及原始负载，reset token 通知 ControlTask 清本地状态。
    sharedData.overload_fault_bitmap = 0;
    for (uint8_t i = 0; i < SERVO_TOTAL_NUM; i++) {
        sharedData.overload_fault_load[i] = 0;
    }
    sharedData.overload_fault_reset_token = 0;

    // Auto 标定结果先初始化 RAM；Calib Load 或本次标定会再写入有效结果。
    initializeCalibrationRuntimeState();

    // Manual Min/Max 只以 P4 NVS 为可信源，上电恢复后同步有效位、CRC 和序号。
    ManualEncoderReference bootManualReferences[ENCODER_TOTAL_NUM];
    uint32_t bootManualSequence = 0;
    uint32_t bootManualCrc = 0;
    if (manualCalibrationNvsLoadLatest(bootManualReferences,
                                      &bootManualSequence,
                                      &bootManualCrc)) {
        manualEncoderReferencesApplyRam(bootManualReferences);
        sharedData.manual_reference_valid_bitmap =
            manualEncoderReferencesValidMask();
        sharedData.manual_reference_crc = bootManualCrc;
        sharedData.manual_reference_nvs_sequence = bootManualSequence;
        sharedData.manual_reference_nvs_valid = 1;
        sharedData.manual_reference_save_status = MANUAL_SAVE_STATUS_OK;
        FW_LOG_INFO_PRINTF(
            "[NVS MANUAL] restored seq=%lu crc=0x%08lX mask=0x%08lX\r\n",
            (unsigned long)bootManualSequence,
            (unsigned long)bootManualCrc,
            (unsigned long)sharedData.manual_reference_valid_bitmap);
    } else {
        manualEncoderReferencesClearRam();
        FW_LOG_INFO_PRINTLN(
            "[NVS MANUAL] no valid snapshot; active scopes without Manual data are gated");
    }

    // 任务按“通信与采样 -> 状态仲裁 -> 控制 -> 持久化/监控”职责拆分。
    xTaskCreate(
        upperCommunicationTask,
        "upperCommunicationTask",
        UPPER_COMM_TASK_STACK_SIZE,
        &sharedData,
        TASK_UPPER_COMM_PRIORITY,
        &upperCommTaskHandle
    );

    xTaskCreate(
        stateMachineTask,
        "stateMachineTask",
        STATE_MACHINE_TASK_STACK_SIZE,
        &sharedData,
        TASK_STATE_MACHINE_PRIORITY,
        &stateMachineTaskHandle
    );

    xTaskCreate(
        canCommunicationTask,
        "canCommunicationTask",
        CAN_COMM_TASK_STACK_SIZE,
        &sharedData,
        TASK_CAN_COMM_PRIORITY,
        &canCommTaskHandle
    );

    xTaskCreate(
        servoCommunicationTask,
        "servoCommunicationTask",
        SERVO_COMM_TASK_STACK_SIZE,
        &sharedData,
        TASK_SERVO_COMM_PRIORITY,
        &servoCommTaskHandle
    );

    xTaskCreate(
        controlTask,
        "controlTask",
        CONTROL_TASK_STACK_SIZE,
        &sharedData,
        TASK_CONTROL_PRIORITY,
        &controlTaskHandle
    );

    xTaskCreate(
        multiTurnNvsPersistenceTask,
        "multiTurnNvsPersist",
        NVS_PERSIST_TASK_STACK_SIZE,
        &sharedData,
        TASK_NVS_PERSIST_PRIORITY,
        &nvsPersistTaskHandle
    );

    // 注册业务任务后，TaskRuntimeMonitor 才能统计周期、执行时间和栈余量。
    taskRuntimeRegister(TASK_RUNTIME_UPPER_COMM, upperCommTaskHandle, UPPER_COMM_TASK_STACK_SIZE);
    taskRuntimeRegister(TASK_RUNTIME_STATE_MACHINE, stateMachineTaskHandle, STATE_MACHINE_TASK_STACK_SIZE);
    taskRuntimeRegister(TASK_RUNTIME_CAN_COMM, canCommTaskHandle, CAN_COMM_TASK_STACK_SIZE);
    taskRuntimeRegister(TASK_RUNTIME_SERVO_COMM, servoCommTaskHandle, SERVO_COMM_TASK_STACK_SIZE);
    taskRuntimeRegister(TASK_RUNTIME_CONTROL, controlTaskHandle, CONTROL_TASK_STACK_SIZE);

    xTaskCreate(
        taskRuntimeMonitorTask,
        "taskRuntimeMonitor",
        TASK_MONITOR_STACK_SIZE,
        &sharedData,
        TASK_MONITOR_PRIORITY,
        &taskMonitorTaskHandle
    );
    taskRuntimeRegister(TASK_RUNTIME_MONITOR, taskMonitorTaskHandle, TASK_MONITOR_STACK_SIZE);

    FW_LOG_INFO_PRINTLN("FreeRTOS tasks created successfully.");
    FW_LOG_INFO_PRINTLN("System ready.");
}

void System_Loop() {
    vTaskDelay(pdMS_TO_TICKS(10));
}
