#include "SystemTask.h"
#include "../calibration/CalibrationTask.h"
#include "FirmwareLog.h"
#include "StateMachineTask.h"
#include "TaskRuntimeMonitor.h"
#include "../control/ControlTask.h"
#include "../communication/servo/ServoCommunicationTask.h"
#include "../hardware/HardwareMap.h"
#include "../persistence/MultiTurnNvsPersistenceTask.h"
#include "../persistence/ManualCalibrationNvsStore.h"
#include "../calibration/RopeInitTask.h"
#include "../communication/upper/UpperCommProtocol.h"

#include <string.h>

// SystemTask 妯″潡鑱岃矗锛?// 1) 瀹屾垚绯荤粺绾ц祫婧愬垵濮嬪寲锛堥槦鍒椼€佷簰鏂ラ攣銆佸叡浜姸鎬侊級锛?// 2) 缁存姢璺ㄤ换鍔″叡浜璞?sharedData锛?// 3) 鍒涘缓骞舵媺璧蜂笂浣嶆満閫氫俊銆佺姸鎬佹満銆丆AN銆佽埖鏈洪€氫俊鍜屾帶鍒朵换鍔°€?
// Persistent calibration state reported in SYSTEM_STATUS.
volatile uint8_t g_calibrationStatus = CALIB_STATUS_IDLE;
// Short-lived calibration event stream reported as repeated CALIB_ACK packets.
volatile uint8_t g_calibrationAckStatus = CALIB_STATUS_IDLE;
volatile uint8_t g_calibrationAckRepeatRemaining = 0;

// Shared task data and queues.
TaskSharedData_t sharedData;

// Task handles used for runtime visibility and future management.
TaskHandle_t upperCommTaskHandle = NULL;
TaskHandle_t stateMachineTaskHandle = NULL;
TaskHandle_t canCommTaskHandle = NULL;
TaskHandle_t servoCommTaskHandle = NULL;
TaskHandle_t controlTaskHandle = NULL;
TaskHandle_t taskMonitorTaskHandle = NULL;
TaskHandle_t nvsPersistTaskHandle = NULL;

void System_Init() {
        Serial.begin(921600);
    while (!Serial) {
        delay(10);
    }
    delay(1000);

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
    sharedData.tactileQueue = xQueueCreate(3, sizeof(RemoteTactileData_t));
    sharedData.multiturnNvsSaveQueue = xQueueCreate(1, sizeof(MultiTurnNvsSaveJob_t));
    sharedData.multiturnNvsResultQueue = xQueueCreate(1, sizeof(MultiTurnNvsSaveResult_t));

    if (!sharedData.cmdQueue || !sharedData.statusQueue ||
        !sharedData.stateEventQueue ||
        !sharedData.canRxQueue || !sharedData.canTxQueue ||
        !sharedData.servoTargetQueue || !sharedData.servoFeedbackQueue ||
        !sharedData.servoAngleQueue || !sharedData.servoRawQueue ||
        !sharedData.servoTelemetryQueue || !sharedData.servoTelemetrySnapshotQueue ||
        !sharedData.mappedAngleQueue ||
        !sharedData.jointDebugQueue || !sharedData.tactileQueue ||
        !sharedData.multiturnNvsSaveQueue || !sharedData.multiturnNvsResultQueue) {
        while (1) {}
    }

    sharedData.targetAnglesMutex = xSemaphoreCreateMutex();
    sharedData.commandStateMutex = xSemaphoreCreateMutex();
    if (!sharedData.targetAnglesMutex || !sharedData.commandStateMutex) {
        while (1) {}
    }
    memset(sharedData.targetAngles, 0, sizeof(sharedData.targetAngles));
    memset(sharedData.motorTargetRaw, 0, sizeof(sharedData.motorTargetRaw));
    memset(sharedData.motorSweepTargetRaw, 0, sizeof(sharedData.motorSweepTargetRaw));
    memset(sharedData.pendingMotorTargetRaw, 0, sizeof(sharedData.pendingMotorTargetRaw));
    memset(sharedData.pendingMotorSweepTargetRaw, 0, sizeof(sharedData.pendingMotorSweepTargetRaw));
    memset(sharedData.restore_multi_turn_abs, 0, sizeof(sharedData.restore_multi_turn_abs));

    sharedData.motor_command_token = 0;
    sharedData.motor_sweep_command_token = 0;
    sharedData.joint_command_token = 0;
    sharedData.motor_direct_command_generation = 0;
    sharedData.motor_direct_command_source = MOTOR_DIRECT_SOURCE_NONE;
    sharedData.pending_direct_generation = 0;
    sharedData.applied_direct_generation = 0;
    sharedData.pending_direct_source = MOTOR_DIRECT_SOURCE_NONE;
    sharedData.pending_direct_valid = 0;
    sharedData.pending_direct_motor_mask = 0;
    sharedData.pending_direct_joint_mask = 0;
    sharedData.pending_direct_require_can = 0;
    sharedData.direct_reject_reason = DIRECT_REJECT_NONE;
    sharedData.sine_test_enabled = 0;
    sharedData.sine_test_request_token = 0;
    sharedData.sine_test_start_ms = 0;
    sharedData.sine_test_joint_mask = 0;
    sharedData.sine_test_hold_ms = 0;
    memset(sharedData.sine_test_center_deg, 0, sizeof(sharedData.sine_test_center_deg));
    memset(sharedData.sine_test_amp_deg, 0, sizeof(sharedData.sine_test_amp_deg));
    memset(sharedData.sine_test_freq_hz, 0, sizeof(sharedData.sine_test_freq_hz));
    memset(sharedData.sine_test_phase_rad, 0, sizeof(sharedData.sine_test_phase_rad));
    sharedData.control_enabled = 0;
    sharedData.control_mode = CONTROL_MODE_JOINT;
    sharedData.system_state = SYSTEM_STATE_UNCONTROLLABLE;
    sharedData.paused_resume_state = SYSTEM_STATE_IDLE;
    sharedData.servo_target_owner = SERVO_TARGET_OWNER_NONE;
    sharedData.feedback_admission_sequence = 0;
    sharedData.feedback_admission_status = FEEDBACK_ADMISSION_IDLE;
    sharedData.feedback_reject_reason = FEEDBACK_REJECT_NONE;
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
    const uint32_t validMotorMask = validMotorChannelMask();
    sharedData.active_motor_mask = ((1UL << 8) | (1UL << 9) | (1UL << 10) | (1UL << 11)) & validMotorMask;
    sharedData.calibration_aux_motor_mask = 0;
    sharedData.active_joint_mask = ((1UL << 8) | (1UL << 9) | (1UL << 10) | (1UL << 11));
    sharedData.multi_turn_unknown_bitmap = validMotorMask;
    sharedData.servo_online_bitmap = 0;
    sharedData.encoder_online_bitmap = 0;
    sharedData.encoder_warn_bitmap = 0;
    sharedData.encoder_fault_bitmap = 0;
    sharedData.can_link_online = 0;
    sharedData.restore_multi_turn_mask = 0;
    sharedData.restore_multi_turn_request_token = 0;
    sharedData.restore_multi_turn_result_token = 0;
    sharedData.restore_multi_turn_restored_mask = 0;
    sharedData.restore_multi_turn_rejected_mask = 0;
    sharedData.multiturn_nvs_save_request_token = 0;
    sharedData.multiturn_nvs_save_reason = 0;
    sharedData.multiturn_nvs_save_result_token = 0;
    sharedData.multiturn_nvs_save_valid_mask = 0;
    sharedData.multiturn_nvs_save_fail_count = 0;
    sharedData.multiturn_nvs_enabled = 1;
    sharedData.multiturn_nvs_snapshot_valid = 0;
    sharedData.host_rx_heartbeat_ms = millis();
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
    sharedData.joint16_dual_feedback_fault = 0;
    memset(&sharedData.calibration_diag, 0, sizeof(sharedData.calibration_diag));
    sharedData.overload_fault_bitmap = 0;
    for (uint8_t i = 0; i < SERVO_TOTAL_NUM; i++) {
        sharedData.overload_fault_load[i] = 0;
    }
    sharedData.overload_fault_reset_token = 0;



        initializeCalibrationRuntimeState();

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
