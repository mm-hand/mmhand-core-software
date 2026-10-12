/**
 * @file ServoBusService.h
 * @brief 声明舵机总线收发、反馈发布和热插拔恢复服务。
 * @details 该服务是四条飞特总线的运行时所有者，其他模块只能通过命令网关提交目标。
 */
#ifndef P4_TASKS_SERVO_BUS_SERVO_BUS_SERVICE_H
#define P4_TASKS_SERVO_BUS_SERVO_BUS_SERVICE_H

#include "../../system/ports/ServiceOutputPort.h"
#include "../../data/ServiceSnapshots.h"

struct SystemRuntimeContext;

/** 描述本轮为何执行；事件快速路径不能推进固定周期反馈相位。 */
struct ServoBusStepRequest {
    uint32_t wakeReasons; ///< TaskSignal 合并后的 kServo* 原因位。
    bool periodicDue;     ///< true 时才允许同步读、失效检查、恢复和反馈发布。
    bool feedbackCollectDue; ///< 四路UART到达本轮非阻塞收集时点。
};

/** 初始化舵机服务和四条总线；任务启动后只调用一次。 */
bool initServoBusService(SystemRuntimeContext* sharedData, uint32_t nowMs);

/** 执行一次舵机命令、反馈、热插拔与持久化事务推进。 */
bool stepServoBusService(SystemRuntimeContext* sharedData,
                         const ServoServiceInputSnapshot& input,
                         const ServiceOutputPort& output,
                         const ServoBusStepRequest& request);

/** 返回当前是否存在已到收集时点的四 BUS 反馈事务。 */
bool servoBusFeedbackCollectDue(uint64_t nowUs);

/** 返回到下一次反馈收集的向上取整毫秒数；0表示当前无事务或已经到期。 */
uint32_t servoBusFeedbackWaitMs(uint64_t nowUs);

#endif // P4_TASKS_SERVO_BUS_SERVO_BUS_SERVICE_H
