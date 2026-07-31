/**
 * @file HardwareTypes.h
 * @brief 定义关节、舵机、总线和执行器映射所使用的纯数据类型。
 */
#ifndef MMHAND_HARDWARE_TYPES_H
#define MMHAND_HARDWARE_TYPES_H

#include "../config/DimensionsConfig.h"
#include <stdint.h>

struct JointMapItem {
    uint8_t busIndex; ///< 兼容的关节主舵机总线索引。
    uint8_t servoID;  ///< 兼容的关节主舵机物理 ID。
};

struct MotorMapItem {
    uint8_t busIndex; ///< 逻辑 Mxx 对应的 UART 总线索引。
    uint8_t servoID;  ///< 逻辑 Mxx 对应的飞特物理 ID。
};

struct JointActuatorItem {
    uint8_t motorChannel; ///< 逻辑舵机通道 M00..M26。
    int8_t sign;          ///< 关节正方向对应的腱绳/舵机方向符号。
    int32_t offset;       ///< 模型坐标偏移，单位 counts。
    int32_t tensionBias;  ///< 预紧偏置，单位 counts。
};

struct JointActuatorGroup {
    uint8_t count; ///< 当前关节实际执行器数量，1 为单腱、2 为双腱。
    JointActuatorItem actuators[MAX_JOINT_ACTUATORS]; ///< 按主/副顺序保存的执行器。
};

extern JointMapItem jointMap[ENCODER_TOTAL_NUM]; ///< 兼容旧调用的关节主映射表。
extern MotorMapItem motorMap[SERVO_TOTAL_NUM]; ///< Mxx 到总线/ID 的唯一映射表。
extern JointActuatorGroup jointActuatorGroups[ENCODER_TOTAL_NUM]; ///< 关节到执行器组映射。

#endif // MMHAND_HARDWARE_TYPES_H
