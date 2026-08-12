/**
 * @file ServoTargetBuilder.h
 * @brief 声明舵机目标批次构建接口。
 */
#ifndef CONTROL_OUTPUT_BUILDER_H
#define CONTROL_OUTPUT_BUILDER_H

#include <Arduino.h>
#include "../../runtime/SystemRuntimeContext.h"

// 一个逻辑关节命令展开到单个物理电机后的运行时目标。
// 拮抗双腱关节由 DualTendonControl 显式给出两个电机目标，
// 不能简单镜像生成第二个电机目标。
struct JointActuatorRuntimeTarget {
    uint8_t motorChannel; ///< 逻辑 Mxx。
    int8_t sign;          ///< 腱绳方向符号。
    int32_t targetAbs;    ///< P4 logical ABS。
};

// ServoTargetBuilder 负责把控制器输出转换成舵机总线目标：
// - 单圈 raw 目标可围绕当前绝对位置锚点展开；
// - 关节执行器辅助函数向 PID 隐藏单腱/双腱硬件拓扑差异。
// 只有电机限位处于锁定状态时，才应用标定得到的 ABS 最小/最大值。
int32_t clampServoPosByMotorAbsLimit(SystemRuntimeContext* sharedData,
                                    uint8_t motorChannel,
                                    int32_t value);
// 在展开为多圈 ABS 前，先把单圈 raw 命令限制到合法范围。
int16_t clampServoSingleTurnRaw(int32_t value);
// 在当前姿态附近选择距离最近的等价多圈位置。
int32_t expandSingleTurnTargetNearCurrent(int32_t currentAbsPos, int16_t singleTurnRaw);
// 将 0..4095 raw 差值转换为最短有符号单圈位移。
int32_t wrapServoRawDelta(int32_t delta);

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
