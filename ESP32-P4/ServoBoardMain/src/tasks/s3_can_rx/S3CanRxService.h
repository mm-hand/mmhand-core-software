/**
 * @file S3CanRxService.h
 * @brief 声明 S3 CAN 传感器接入服务。
 * @details 服务拥有 CAN 分帧重组和传感器健康状态；Task 层仅负责把 FreeRTOS 入口转交给本服务。
 */
#ifndef P4_TASKS_S3_CAN_RX_S3_CAN_RX_SERVICE_H
#define P4_TASKS_S3_CAN_RX_S3_CAN_RX_SERVICE_H

#include "../../runtime/SystemRuntimeContext.h"

/** 初始化 TWAI 和所有分帧重组状态。 */
bool initS3CanRxService(SystemRuntimeContext* sharedData, uint32_t nowMs);

/** 消费当前已到达的 CAN 帧并推进一次健康状态。 */
bool stepS3CanRxService(SystemRuntimeContext* sharedData, uint32_t nowMs);

#endif // P4_TASKS_S3_CAN_RX_S3_CAN_RX_SERVICE_H
