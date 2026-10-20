/**
 * @file ServoTargetBuilder.h
 * @brief 声明舵机目标批次构建接口。
 */
#ifndef CONTROL_OUTPUT_BUILDER_H
#define CONTROL_OUTPUT_BUILDER_H

#include <Arduino.h>
#include "../../system/SystemRuntimeContext.h"

// 一个逻辑关节命令展开到单个物理电机后的运行时目标。
// 拮抗双腱关节由 DualTendonControl 显式给出两个电机目标，
// 不能简单镜像生成第二个电机目标。
struct JointActuatorRuntimeTarget {
    uint8_t motorChannel; ///< 逻辑 Mxx。
    int8_t sign;          ///< 腱绳方向符号。
    int32_t targetAbs;    ///< P4 logical ABS。
};

// ServoTargetBuilder 负责把控制器输出转换成舵机总线目标：
// - 关节执行器辅助函数向 PID 隐藏单腱/双腱硬件拓扑差异。
// 如果固定大小 batch 仍有空间，则追加一条 Mxx 命令；BUS 在执行时解析。
bool appendServoTarget(ServoTargetBatch_t* batch,
                       uint8_t motorChannel,
                       int32_t position,
                       uint16_t speed,
                       uint8_t acc);

// 普通单电机或同向多电机关节的便捷封装。
uint8_t appendJointActuatorTargets(ServoTargetBatch_t* batch,
                                   uint8_t jointIndex,
                                   int32_t primaryTargetAbs,
                                   uint16_t speed,
                                   uint8_t acc);

// 追加已经展开好的运行时目标，用于双腱保持/控制。
uint8_t appendJointActuatorRuntimeTargets(ServoTargetBatch_t* batch,
                                          const JointActuatorRuntimeTarget* targets,
                                          uint8_t targetCount,
                                          uint16_t speed,
                                          uint8_t acc);

#endif // CONTROL_OUTPUT_BUILDER_H
