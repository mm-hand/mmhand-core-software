#ifndef TASK_SHARED_DATA_H
#define TASK_SHARED_DATA_H

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#define ENCODER_TOTAL_NUM 21
#define SERVO_TOTAL_NUM 27
#define JOINT_COUNT ENCODER_TOTAL_NUM
#define MAX_JOINT_ACTUATORS 2
#define MOTOR_MAP_INVALID_BUS 0xFF
#define NUM_BUSES 3
#define MAX_SERVOS_PER_BUS 9
#define MAX_SERVO_ID 32
#define SERVO_SYNC_READ_TIMEOUT_MS_DEFAULT 3
#define SERVO_TARGET_SPEED_DEFAULT 400
#define SERVO_TARGET_ACC_DEFAULT 200
#define TACTILE_GROUP_NUM 5
#define TACTILE_SENSOR_PER_GROUP 3
#define TACTILE_AXIS_NUM 3
#define TACTILE_SUMMARY_BYTES (TACTILE_GROUP_NUM * TACTILE_SENSOR_PER_GROUP * TACTILE_AXIS_NUM)

#define CONTROL_MODE_JOINT 0
#define CONTROL_MODE_DIRECT_MOTOR 1

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

#define SYSTEM_STATE_UNCONTROLLABLE 0
#define SYSTEM_STATE_IDLE 1
#define SYSTEM_STATE_RUN_FEEDBACK 2
#define SYSTEM_STATE_STOPPED 3
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

#define MOTOR_CHANNEL_MASK_ALL ((1UL << SERVO_TOTAL_NUM) - 1UL)
#define JOINT_CHANNEL_MASK_ALL ((1UL << ENCODER_TOTAL_NUM) - 1UL)

#define TASK_UPPER_COMM_PRIORITY 1
#define TASK_CAN_COMM_PRIORITY 3
#define TASK_SERVO_COMM_PRIORITY 3
#define CAN_COMM_TASK_STACK_SIZE 4096U
#define SERVO_COMM_TASK_STACK_SIZE 6144U
#define TASK_USB_COMM_PRIORITY 2
#define USB_COMM_TASK_STACK_SIZE 2048U
#define UPPER_COMM_TASK_STACK_SIZE 8192U
#define USB_COMM_MAX_DATA_SIZE 256U
#define USB_COMM_RX_QUEUE_LENGTH 16U
#define USB_COMM_TX_QUEUE_LENGTH 16U
#define UPPER_COMM_RX_BUFFER_SIZE 512U
#define CAN_COMM_MAX_DATA_SIZE 64U
#define SERVO_TARGET_BATCH_MAX SERVO_TOTAL_NUM

typedef struct {
    uint8_t busIndex;
    uint8_t servoID;
} JointMapItem;

typedef struct {
    uint8_t busIndex;
    uint8_t servoID;
} MotorMapItem;

typedef struct {
    uint8_t motorChannel;
    int8_t sign;
    int32_t offset;
    int32_t tensionBias;
} JointActuatorItem;

typedef struct {
    uint8_t count;
    JointActuatorItem actuators[MAX_JOINT_ACTUATORS];
} JointActuatorGroup;

extern JointMapItem jointMap[ENCODER_TOTAL_NUM];
extern MotorMapItem motorMap[SERVO_TOTAL_NUM];
extern JointActuatorGroup jointActuatorGroups[ENCODER_TOTAL_NUM];

typedef struct {
    uint8_t cmdID;
    uint8_t payload[CAN_COMM_MAX_DATA_SIZE];
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
    uint8_t values[TACTILE_SUMMARY_BYTES];
    uint8_t seq;
    uint32_t timestamp;
    bool isValid;
} RemoteTactileData_t;

typedef struct {
    uint8_t event;
    uint32_t timestamp;
} SystemEvent_t;

typedef struct {
    uint8_t servoId;
    int16_t position;
    int16_t speed;
    int16_t load;
    uint8_t voltage;
    uint8_t temperature;
} ServoStatus_t;

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
    uint8_t payload[USB_COMM_MAX_DATA_SIZE];
    uint16_t len;
    uint32_t timestamp;
} UsbPacket_t;

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
    QueueHandle_t usbRxQueue;
    QueueHandle_t usbTxQueue;

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
    volatile uint32_t multi_turn_unknown_bitmap;
    int32_t restore_multi_turn_abs[SERVO_TOTAL_NUM];
    volatile uint32_t restore_multi_turn_mask;
    volatile uint32_t restore_multi_turn_request_token;
    volatile uint32_t restore_multi_turn_result_token;
    volatile uint32_t restore_multi_turn_restored_mask;
    volatile uint32_t restore_multi_turn_rejected_mask;
    volatile uint32_t motor_abs_limit_valid_bitmap;
    volatile uint32_t motor_abs_limit_locked_bitmap;
    int32_t motor_abs_limit_min[SERVO_TOTAL_NUM];
    int32_t motor_abs_limit_max[SERVO_TOTAL_NUM];
    volatile uint8_t require_can_feedback;
    volatile uint8_t calibration_fault_latched;
    volatile uint8_t joint16_dual_feedback_fault;
    volatile uint32_t overload_fault_bitmap;
    volatile uint32_t overload_fault_reset_token;
    volatile uint32_t reverse_release_fault_bitmap;
    volatile uint32_t reverse_release_fault_reset_token;
    volatile uint8_t tendon_guard_enabled[ENCODER_TOTAL_NUM];
    volatile int8_t tendon_guard_sign[ENCODER_TOTAL_NUM];
    volatile int16_t tendon_guard_x1_abs[ENCODER_TOTAL_NUM];

    int32_t calib_zero_raw_cache[ENCODER_TOTAL_NUM];
    volatile uint8_t calib_zero_raw_valid;
} TaskSharedData_t;

#ifdef __cplusplus
extern "C" {
#endif

extern TaskSharedData_t sharedData;
void postSystemEvent(TaskSharedData_t *sharedData, uint8_t event);
uint32_t validMotorChannelMask(void);

#ifdef __cplusplus
}
#endif

#endif /* TASK_SHARED_DATA_H */
