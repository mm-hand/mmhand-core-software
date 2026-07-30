/**
 * @file ActuatorTypes.h
 * @brief 定义业务层提交给舵机执行层的命令数据结构。
 * @details position 始终表示 P4 logical ABS；驱动层负责转换为 hardware ABS。
 */
#ifndef MMHAND_ACTUATOR_TYPES_H
#define MMHAND_ACTUATOR_TYPES_H

#include "../config/DimensionsConfig.h"
#include <stdint.h>

inline constexpr uint8_t SERVO_TARGET_OWNER_NONE = 0;
inline constexpr uint8_t SERVO_TARGET_OWNER_CONTROL = 1;
inline constexpr uint8_t SERVO_TARGET_OWNER_CALIBRATION = 2;
inline constexpr uint8_t SERVO_TARGET_BATCH_MAX = SERVO_TOTAL_NUM;

struct ServoTargetCommand_t {
    uint8_t busIndex;
    uint8_t servoId;
    int32_t position;
    uint16_t speed;
    uint8_t acc;
};

struct ServoTargetBatch_t {
    uint8_t count;
    uint8_t source;
    ServoTargetCommand_t commands[SERVO_TARGET_BATCH_MAX];
    uint32_t timestamp;
};

struct ControlCommandSnapshot_t {
    float targetAngles[ENCODER_TOTAL_NUM];
    int32_t motorTargetRaw[SERVO_TOTAL_NUM];
    int32_t motorSweepTargetRaw[SERVO_TOTAL_NUM];
    uint32_t motorCommandToken;
    uint32_t motorSweepCommandToken;
    uint32_t jointCommandToken;
    uint32_t feedbackCommandJointMask;
    uint32_t motorDirectCommandGeneration;
    uint8_t motorDirectCommandSource;
    uint8_t controlMode;
};

#endif // MMHAND_ACTUATOR_TYPES_H
