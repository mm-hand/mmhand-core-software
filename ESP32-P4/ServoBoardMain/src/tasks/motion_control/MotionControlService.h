/**
 * @file MotionControlService.h
 * @brief 声明 100 Hz 关节负反馈和舵机直控编排服务。
 * @details 服务读取一致性反馈快照并生成批量舵机目标，不直接访问 UART 或飞特 SDK。
 */
#ifndef P4_TASKS_MOTION_CONTROL_MOTION_CONTROL_SERVICE_H
#define P4_TASKS_MOTION_CONTROL_MOTION_CONTROL_SERVICE_H

#include "../../runtime/ports/ServiceOutputPort.h"
#include "../../data/ServiceSnapshots.h"

struct SystemRuntimeContext;

/** @brief 运行实时控制主循环；该函数必须由唯一的 MotionControlTask 调用且不会主动返回。 */
/** 初始化控制服务的单实例运行上下文；任务启动后只调用一次。 */
bool initMotionControlService(SystemRuntimeContext* sharedData, uint32_t nowMs);

/**
 * @brief 执行一个控制周期。
 * @param sharedData 系统装配层提供的兼容上下文；后续由 Control 端口继续缩窄。
 * @param nowMs 任务层采样的单调毫秒时间，同一周期内所有判断使用同一时间基准。
 * @return 本周期是否完成了有效快照读取和控制计算。
 */
bool stepMotionControlService(SystemRuntimeContext* sharedData,
                        const ControlServiceInputSnapshot& input,
                        const ServiceOutputPort& output);

#endif // P4_TASKS_MOTION_CONTROL_MOTION_CONTROL_SERVICE_H
