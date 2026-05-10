#ifndef UPPER_COMM_COMMAND_ROUTER_H
#define UPPER_COMM_COMMAND_ROUTER_H

#include <Arduino.h>
#include "../../shared/TaskSharedData.h"

// UpperCommCommandRouter 是串口命令进入共享控制状态的唯一写入口：
// - 所有控制目标写入都在 commandStateMutex 下完成；
// - 写入目标时同步更新 token，通知 ControlTask 解除热插拔保持/刷新快照；
// - 本模块不处理串口字节流，也不直接下发舵机目标。

// 将上位机绝对舵机目标限制到多圈安全范围。
int16_t clampServoAbsCommand(int32_t value);
// 写入 21 个关节角目标，并切换到 joint PID 控制模式。
void upperApplyTargetAngles(TaskSharedData_t* sharedData, const float* angles, uint8_t count);
// 清零关节目标，保留为历史单字节 'b' 命令兼容入口。
void upperClearTargetAngles(TaskSharedData_t* sharedData);
// 写入 22 个舵机直控目标，source 表示 TARGET/ABSOLUTE 等直控语义。
void upperApplyMotorTargets(TaskSharedData_t* sharedData,
                            const int32_t* targets,
                            uint8_t count,
                            uint8_t source);
// 写入滑条 sweep 目标；ControlTask 会基于当前位置把单圈 raw 连续展开。
void upperApplyMotorSweepTargets(TaskSharedData_t* sharedData, const int32_t* targets, uint8_t count);
// 写入 tendon guard 配置，用于阻止指定关节继续释放到危险边界。
void upperApplyTendonGuardConfig(TaskSharedData_t* sharedData,
                                 const uint8_t* enabled,
                                 const int8_t* sign,
                                 const int16_t* x1Abs,
                                 uint8_t count);
// 缓存上位机下发的校准零点 raw 数据，供后续校准流程消费。
void upperCacheCalibZeroRaw(TaskSharedData_t* sharedData, const float* values, uint8_t count);

#endif // UPPER_COMM_COMMAND_ROUTER_H
