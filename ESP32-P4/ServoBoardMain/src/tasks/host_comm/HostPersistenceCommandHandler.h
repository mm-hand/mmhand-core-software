/**
 * @file HostPersistenceCommandHandler.h
 * @brief 声明上位机多圈 NVS 控制命令的异步投递与结果提交接口。
 * @details HostCommTask 只操作任务间队列和 RAM 镜像，不直接访问 Flash。
 */
#ifndef P4_TASKS_HOST_COMM_HOST_PERSISTENCE_COMMAND_HANDLER_H
#define P4_TASKS_HOST_COMM_HOST_PERSISTENCE_COMMAND_HANDLER_H

#include <stdint.h>

struct SystemRuntimeContext;

/** 非阻塞投递 0=关闭并清空、1=启用；返回协议 ACK 状态。 */
uint8_t enqueueMultiTurnNvsControl(SystemRuntimeContext* sharedData,
                                   uint8_t action,
                                   uint32_t nowMs);

/** 消费 NVS 任务结果并提交共享 RAM；消费到本代结果时返回 true。 */
bool processMultiTurnNvsControlResult(SystemRuntimeContext* sharedData);

#endif // P4_TASKS_HOST_COMM_HOST_PERSISTENCE_COMMAND_HANDLER_H
