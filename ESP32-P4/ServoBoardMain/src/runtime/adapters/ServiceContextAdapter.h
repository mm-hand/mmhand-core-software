/**
 * @file ServiceContextAdapter.h
 * @brief 把 SystemRuntimeContext 原子字段采集为纯值快照，并创建默认输出端口。
 * @details 本适配器是旧共享上下文与新服务契约之间唯一允许的转换位置。
 */
#ifndef P4_RUNTIME_ADAPTERS_SERVICE_CONTEXT_ADAPTER_H
#define P4_RUNTIME_ADAPTERS_SERVICE_CONTEXT_ADAPTER_H

#include "../ports/ServiceOutputPort.h"
#include "../../data/ServiceSnapshots.h"

struct SystemRuntimeContext;

ControlServiceInputSnapshot captureControlServiceInput(
    SystemRuntimeContext* sharedData, uint32_t nowMs);
ServoServiceInputSnapshot captureServoServiceInput(
    SystemRuntimeContext* sharedData, uint32_t nowMs);
CalibrationServiceInputSnapshot captureCalibrationServiceInput(
    SystemRuntimeContext* sharedData, uint32_t nowMs);
StateServiceInputSnapshot captureStateServiceInput(
    SystemRuntimeContext* sharedData, uint32_t nowMs);
ServiceOutputPort makeSystemServiceOutputPort(SystemRuntimeContext* sharedData);

#endif // P4_RUNTIME_ADAPTERS_SERVICE_CONTEXT_ADAPTER_H
