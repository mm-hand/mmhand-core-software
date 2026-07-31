/**
 * @file ControlRuntimePolicy.h
 * @brief 定义控制任务的命令快照、过载作用域和动作完成判定。
 *
 * 这些函数不运行 PID，也不发布舵机目标；它们把原本散落在 ControlTask
 * 中的运行准入规则集中为可审查、可测试的策略接口。
 */
#ifndef P4_CONTROL_RUNTIME_POLICY_H
#define P4_CONTROL_RUNTIME_POLICY_H

#include <stdint.h>

#include "../shared/TaskSharedData.h"

namespace p4::control {

/** 在同一互斥区内复制一次完整上位机控制命令，避免同周期读到半更新数组。 */
bool readCommandSnapshot(TaskSharedData_t* sharedData,
                         ControlCommandSnapshot_t* snapshot);

/** 返回当前控制模式真正主动驱动的舵机 mask，供硬过载检查使用。 */
uint32_t activeOverloadMotorMask(
    TaskSharedData_t* sharedData,
    const ControlCommandSnapshot_t& commandSnapshot);

/** 判断 Jxx 任一执行器是否出现在已锁存的舵机过载位图中。 */
bool jointHasOverloadFault(uint8_t jointIndex,
                           uint32_t overloadFaultBitmap);

/** 检查 active motor 的实际 ABS 是否全部进入直控完成误差窗。 */
bool directTargetsReached(
    TaskSharedData_t* sharedData,
    const ServoAngleData_t& servoData,
    const ControlCommandSnapshot_t& commandSnapshot);

/** 检查本次 commandJointMask 内的关节角是否全部进入反馈完成误差窗。 */
bool feedbackTargetsReached(
    TaskSharedData_t* sharedData,
    const MappedAngleData_t& mappedData,
    const float* targetDegs,
    const float* actualDegs,
    uint32_t commandJointMask);

} // namespace p4::control

#endif // P4_CONTROL_RUNTIME_POLICY_H
