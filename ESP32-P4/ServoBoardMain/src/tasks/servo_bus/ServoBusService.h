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

/** 描述本轮为何执行；事件快速路径不能推进固定周期反馈相位。 */
struct ServoBusStepRequest {
    uint32_t wakeReasons; ///< TaskSignal 合并后的 kServo* 原因位。
    bool periodicDue;     ///< true 时才允许同步读、失效检查、恢复和反馈发布。
};

/** 初始化舵机服务和四条总线；任务启动后只调用一次。 */
bool initServoBusService(SystemRuntimeContext* sharedData, uint32_t nowMs);

/** 执行一次舵机命令、反馈、热插拔与持久化事务推进。 */
bool stepServoBusService(SystemRuntimeContext* sharedData,
                         const ServoServiceInputSnapshot& input,
                         const ServiceOutputPort& output,
                         const ServoBusStepRequest& request);

#endif // P4_TASKS_SERVO_BUS_SERVO_BUS_SERVICE_H
