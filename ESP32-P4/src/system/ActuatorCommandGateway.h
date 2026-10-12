/**
 * @file ActuatorCommandGateway.h
 * @brief 提供所有业务任务提交、清除和消费舵机目标批次的唯一入口。
 *
 * 当前底层仍使用容量为 1 的“最新快照”队列，以保持原有实时行为。后续的
 * owner 仲裁和统一安全策略只需接入本模块，不再修改每一个控制生产者。
 */
#ifndef P4_ACTUATOR_COMMAND_GATEWAY_H
#define P4_ACTUATOR_COMMAND_GATEWAY_H

#include "../data/ActuatorTypes.h"
#include "SystemRuntimeContext.h"
#include "rtos/FreeRtosKernel.h"

namespace p4::actuator {

/** 发布最新目标快照；空批次视为无操作并返回 true。 */
bool publishLatest(SystemRuntimeContext* sharedData,
                   const ServoTargetBatch_t& batch);

/** 清除尚未被执行层消费的目标快照。 */
void clearPending(SystemRuntimeContext* sharedData);

/** 非阻塞读取一条最新目标，供唯一执行者 ServoBusTask 使用。 */
bool receiveLatest(SystemRuntimeContext* sharedData,
                   ServoTargetBatch_t* outBatch);

/** 返回当前待消费快照数；容量为 1，因此结果只会是 0 或 1。 */
UBaseType_t pendingCount(const SystemRuntimeContext* sharedData);

} // namespace p4::actuator

#endif // P4_ACTUATOR_COMMAND_GATEWAY_H
