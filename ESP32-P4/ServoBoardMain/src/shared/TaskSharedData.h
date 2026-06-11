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

// TWAI(CAN) pins.
#define TWAI_TX_PIN 47
#define TWAI_RX_PIN 48

// FreeRTOS task priorities.
#define TASK_UPPER_COMM_PRIORITY 1
#define TASK_STATE_MACHINE_PRIORITY 2
#define TASK_CAN_COMM_PRIORITY 3
#define TASK_SERVO_COMM_PRIORITY 3
#define TASK_CONTROL_PRIORITY 4
#define TASK_SOLVER_PRIORITY TASK_CONTROL_PRIORITY

// FreeRTOS task stack sizes.
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

struct JointActuatorItem {
    uint8_t motorChannel;
    // +1: actuator pulls in the same direction as the primary actuator.
    // -1: antagonistic actuator; runtime targets must be built explicitly.
    int8_t sign;
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
    uint8_t cmdID;
    uint8_t payload[8];
    uint8_t len;
} RemoteCommand_t;

typedef struct {
    uint16_t encoderValues[ENCODER_TOTAL_NUM];
    uint8_t errorFlags[ENCODER_TOTAL_NUM];
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
    uint8_t sineTestEnabled;
    uint32_t sineTestRequestToken;
    uint32_t sineTestStartMs;
    uint32_t sineTestJointMask;
    uint32_t sineTestHoldMs;
    float sineTestCenterDeg[ENCODER_TOTAL_NUM];
    float sineTestAmpDeg[ENCODER_TOTAL_NUM];
    float sineTestFreqHz[ENCODER_TOTAL_NUM];
    float sineTestPhaseRad[ENCODER_TOTAL_NUM];
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
    uint8_t fullValues[TACTILE_FULL_BYTES];
    uint8_t seq;
    uint8_t fullSeq;
    uint32_t timestamp;
    uint32_t fullTimestamp;
    bool isValid;
    bool fullValid;
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

    int32_t motorTargetRaw[SERVO_TOTAL_NUM];
    int32_t motorSweepTargetRaw[SERVO_TOTAL_NUM];
    volatile uint32_t motor_command_token;
    volatile uint32_t motor_sweep_command_token;
    volatile uint32_t joint_command_token;
    volatile uint32_t motor_direct_command_generation;
    volatile uint8_t motor_direct_command_source;

    volatile uint8_t control_enabled;
    volatile uint8_t control_mode;
    volatile uint8_t system_state;
    volatile uint8_t paused_resume_state;
    volatile uint8_t servo_target_owner;
    volatile uint32_t servo_emergency_stop_token;
    volatile uint32_t system_fault_bitmap;
    volatile uint32_t system_fault_latched_bitmap;
    volatile uint32_t state_transition_count;
    volatile uint32_t active_motor_mask;
    volatile uint32_t active_joint_mask;
    volatile uint8_t sine_test_enabled;
    volatile uint32_t sine_test_request_token;
    volatile uint32_t sine_test_start_ms;
    volatile uint32_t sine_test_joint_mask;
    volatile uint32_t sine_test_hold_ms;
    float sine_test_center_deg[ENCODER_TOTAL_NUM];
    float sine_test_amp_deg[ENCODER_TOTAL_NUM];
    float sine_test_freq_hz[ENCODER_TOTAL_NUM];
    float sine_test_phase_rad[ENCODER_TOTAL_NUM];
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

    volatile uint32_t multiturn_nvs_save_request_token;
    volatile uint8_t multiturn_nvs_save_reason;
    volatile uint32_t multiturn_nvs_save_result_token;
    volatile uint32_t multiturn_nvs_save_valid_mask;
    volatile uint32_t multiturn_nvs_save_fail_count;
    volatile uint8_t multiturn_nvs_enabled;
    volatile uint8_t multiturn_nvs_snapshot_valid;
    volatile uint32_t host_rx_heartbeat_ms;

    volatile uint32_t motor_abs_limit_valid_bitmap;
    volatile uint32_t motor_abs_limit_locked_bitmap;
    int32_t motor_abs_limit_min[SERVO_TOTAL_NUM];
    int32_t motor_abs_limit_max[SERVO_TOTAL_NUM];
    volatile uint8_t require_can_feedback;
    volatile uint8_t calibration_fault_latched;
    volatile uint8_t joint16_dual_feedback_fault;

    volatile uint32_t overload_fault_bitmap;
    volatile int16_t overload_fault_load[SERVO_TOTAL_NUM];
    // START/RESET 时自增，用于通知清除过载锁存故障。
    volatile uint32_t overload_fault_reset_token;
    volatile uint32_t reverse_release_fault_bitmap;
    volatile uint32_t reverse_release_fault_reset_token;
    volatile uint8_t tendon_guard_enabled[ENCODER_TOTAL_NUM];
    volatile int8_t tendon_guard_sign[ENCODER_TOTAL_NUM];
    volatile int16_t tendon_guard_x1_abs[ENCODER_TOTAL_NUM];

    int32_t calib_zero_raw_cache[ENCODER_TOTAL_NUM];
    volatile uint8_t calib_zero_raw_valid;
} TaskSharedData_t;

#endif // TASK_SHARED_DATA_H
