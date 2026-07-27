#ifndef CONTROL_OUTPUT_BUILDER_H
#define CONTROL_OUTPUT_BUILDER_H

#include <Arduino.h>
#include "../shared/TaskSharedData.h"

// 一个逻辑关节命令展开到单个物理电机后的运行时目标。
// 拮抗双腱关节由 DualTendonControl 显式给出两个电机目标，
// 不能简单镜像生成第二个电机目标。
struct JointActuatorRuntimeTarget {
    uint8_t motorChannel;
    uint8_t busIndex;
    uint8_t servoId;
    int8_t sign;
    int32_t targetAbs;
};

// ControlOutputBuilder 负责把控制器输出转换成舵机总线目标：
// - 所有绝对位置都会限制在舵机多圈命令范围内；
// - 单圈 raw 目标可围绕当前绝对位置锚点展开；
// - 关节执行器辅助函数向 PID 隐藏 V1/V2 单电机/双电机差异。
int32_t clampServoPos(int32_t value);
// 只有电机限位处于锁定状态时，才应用标定得到的 ABS 最小/最大值。
int32_t clampServoPosByMotorAbsLimit(TaskSharedData_t* sharedData,
                                    uint8_t motorChannel,
                                    int32_t value);
// 在展开为多圈 ABS 前，先把单圈 raw 命令限制到合法范围。
int16_t clampServoSingleTurnRaw(int32_t value);
// 在当前姿态附近选择距离最近的等价多圈位置。
int32_t expandSingleTurnTargetNearCurrent(int32_t currentAbsPos, int16_t singleTurnRaw);
// 用当前绝对姿态和首个 raw 目标初始化 sweep 模式锚点。
int32_t anchorSingleTurnSweepToCurrent(int32_t currentAbsPos, int16_t singleTurnRaw);
// 将 0..4095 raw 差值转换为最短有符号单圈位移。
int32_t wrapServoRawDelta(int32_t delta);

// 将一个逻辑关节命令展开成一个或多个物理电机命令。
uint8_t buildJointActuatorRuntimeTargets(uint8_t jointIndex,
                                         int32_t primaryTargetAbs,
                                         JointActuatorRuntimeTarget* outTargets,
                                         uint8_t outCapacity);

// 如果固定大小 batch 仍有空间，则追加一条 bus/id 命令。
bool appendServoTarget(ServoTargetBatch_t* batch,
                       uint8_t busIndex,
                       uint8_t servoId,
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
