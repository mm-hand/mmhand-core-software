#ifndef UPPER_COMM_COMMAND_ROUTER_H
#define UPPER_COMM_COMMAND_ROUTER_H

#include "TaskSharedData.h"

#ifdef __cplusplus

#include <stdint.h>

// 将上位机绝对舵机目标限制到固件允许的多圈范围。
int16_t clampServoAbsCommand(int32_t value);

// 写入 21 个关节角目标，并切换到 joint PID 控制语义。
void upperApplyTargetAngles(TaskSharedData_t *sharedData, const float *angles, uint8_t count);

// 清零关节目标，保留给历史单字节 'b' 命令。
void upperClearTargetAngles(TaskSharedData_t *sharedData);

// 写入 27 个电机直控目标，source 区分 target/sweep/absolute 语义。
void upperApplyMotorTargets(TaskSharedData_t *sharedData,
                            const int32_t *targets,
                            uint8_t count,
                            uint8_t source);

// 写入滑条 sweep 目标，后续控制层按单圈 raw 语义解释。
void upperApplyMotorSweepTargets(TaskSharedData_t *sharedData, const int32_t *targets, uint8_t count);

// 写入 tendon guard 配置，用于阻止关节继续释放到危险边界。
void upperApplyTendonGuardConfig(TaskSharedData_t *sharedData,
                                 const uint8_t *enabled,
                                 const int8_t *sign,
                                 const int16_t *x1Abs,
                                 uint8_t count);

// 缓存上位机下发的校准零点 raw 数据，供后续校准流程消费。
void upperCacheCalibZeroRaw(TaskSharedData_t *sharedData, const float *values, uint8_t count);

// 设置本次上位机控制的电机/关节范围和是否强制需要 CAN 反馈。
void upperApplyActiveScope(TaskSharedData_t *sharedData,
                           uint32_t activeMotorMask,
                           uint32_t activeJointMask,
                           uint8_t requireCanFeedback);

#endif

#endif /* UPPER_COMM_COMMAND_ROUTER_H */
