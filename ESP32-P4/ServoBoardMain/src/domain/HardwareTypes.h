/**
 * @file HardwareTypes.h
 * @brief 定义关节、舵机、总线和执行器映射所使用的纯数据类型。
 */
#ifndef MMHAND_HARDWARE_TYPES_H
#define MMHAND_HARDWARE_TYPES_H

#include "../config/DimensionsConfig.h"
#include <stdint.h>

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
    int8_t sign;
    int32_t offset;
    int32_t tensionBias;
};

struct JointActuatorGroup {
    uint8_t count;
    JointActuatorItem actuators[MAX_JOINT_ACTUATORS];
};

extern JointMapItem jointMap[ENCODER_TOTAL_NUM];
extern MotorMapItem motorMap[SERVO_TOTAL_NUM];
extern JointActuatorGroup jointActuatorGroups[ENCODER_TOTAL_NUM];

#endif // MMHAND_HARDWARE_TYPES_H
