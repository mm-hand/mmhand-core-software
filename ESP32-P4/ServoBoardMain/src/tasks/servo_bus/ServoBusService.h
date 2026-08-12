/**
 * @file ServoBusService.h
 * @brief 声明舵机总线收发、反馈发布和热插拔恢复服务。
 * @details 该服务是四条飞特总线的运行时所有者，其他模块只能通过命令网关提交目标。
 */
#ifndef P4_TASKS_SERVO_BUS_SERVO_BUS_SERVICE_H
#define P4_TASKS_SERVO_BUS_SERVO_BUS_SERVICE_H

#include "../../runtime/ports/ServiceOutputPort.h"
#include "../../data/ServiceSnapshots.h"

struct SystemRuntimeContext;

/** 初始化舵机服务和四条总线；任务启动后只调用一次。 */
bool initServoBusService(SystemRuntimeContext* sharedData, uint32_t nowMs);

/** 执行一次舵机命令、反馈、热插拔与持久化事务推进。 */
bool stepServoBusService(SystemRuntimeContext* sharedData,
                             const ServoServiceInputSnapshot& input,
                             const ServiceOutputPort& output);

#endif // P4_TASKS_SERVO_BUS_SERVO_BUS_SERVICE_H
