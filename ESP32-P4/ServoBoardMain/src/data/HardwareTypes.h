/**
 * @file HardwareTypes.h
 * @brief 定义关节、舵机、总线和执行器映射所使用的纯数据类型。
 */
#ifndef MMHAND_HARDWARE_TYPES_H
#define MMHAND_HARDWARE_TYPES_H

#include "../config/DimensionsConfig.h"
#include <stdint.h>

struct JointMapItem {
    uint8_t motorChannel; ///< 关节主执行器 Mxx；物理 BUS 由运行时路由表决定。
};

struct MotorMapItem {
    uint8_t servoID; ///< 逻辑 Mxx 对应的全手唯一飞特物理 ID；BUS 由运行时扫描决定。
};

struct JointActuatorItem {
    uint8_t motorChannel; ///< 逻辑舵机通道 M00..M26。
    int8_t sign;          ///< 关节正方向对应的腱绳/舵机方向符号。
};

struct JointActuatorGroup {
    uint8_t count; ///< 当前关节实际执行器数量，1 为单腱、2 为双腱。
    JointActuatorItem actuators[MAX_JOINT_ACTUATORS]; ///< 按主/副顺序保存的执行器。
};

extern JointMapItem jointMap[ENCODER_TOTAL_NUM]; ///< Jxx 到主执行器 Mxx 的机械拓扑。
extern MotorMapItem motorMap[SERVO_TOTAL_NUM]; ///< Mxx 到全手唯一舵机 ID 的身份表，不保存物理 BUS。
extern JointActuatorGroup jointActuatorGroups[ENCODER_TOTAL_NUM]; ///< 关节到执行器组映射。

#endif // MMHAND_HARDWARE_TYPES_H
