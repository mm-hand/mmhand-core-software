/**
 * @file HostCommandRouter.h
 * @brief 声明上位机命令到内部控制接口的路由函数。
 */
#ifndef UPPER_COMM_COMMAND_ROUTER_H
#define UPPER_COMM_COMMAND_ROUTER_H

#include <Arduino.h>
#include "../../runtime/SystemRuntimeContext.h"

// HostCommandRouter 是串口命令进入共享控制状态的唯一写入口：
// - 所有控制目标写入都在 commandStateMutex 下完成；
// - 写入目标时同步更新 token，通知 MotionControlTask 解除热插拔保持/刷新快照；
// - 本模块不处理串口字节流，也不直接下发舵机目标。

// 写入 21 个关节角目标，并切换到 joint PID 控制模式。
void upperApplyTargetAngles(SystemRuntimeContext* sharedData, const float* angles, uint8_t count);
// 写入角度并限定本次负反馈实际参与 PID 的关节；未置位 active joint 保持当前 ABS。
void upperApplyTargetAnglesMasked(SystemRuntimeContext* sharedData,
                                  const float* angles,
                                  uint8_t count,
                                  uint32_t jointMask);
// 暂存最多 27 个逻辑多圈 ABS 直控目标，source 区分普通目标和绝对位置命令。
// 此处不做统一数值钳位；发送前由舵机层结合每台电机 logicalOffset 做最终硬限制。
bool upperApplyMotorTargets(SystemRuntimeContext* sharedData,
                            const int32_t* targets,
                            uint8_t count,
                            uint8_t source);
// 写入滑条 sweep 目标；MotionControlTask 会基于当前位置把单圈 raw 连续展开。
bool upperApplyMotorSweepTargets(SystemRuntimeContext* sharedData, const int32_t* targets, uint8_t count);
// 在静止态原子更新 active motor/joint scope；requireCanFeedback 决定是否依赖磁编反馈。
bool upperApplyActiveScope(SystemRuntimeContext* sharedData,
                           uint32_t activeMotorMask,
                           uint32_t activeJointMask,
                           uint8_t requireCanFeedback);

// 暂存一台舵机的逻辑多圈 ABS 目标，并请求状态机进入 RUN_DIRECT。
bool upperApplySingleMotorTarget(SystemRuntimeContext* sharedData,
                                 uint8_t motorIndex,
                                 int32_t target,
                                 uint8_t source);

#endif // UPPER_COMM_COMMAND_ROUTER_H
