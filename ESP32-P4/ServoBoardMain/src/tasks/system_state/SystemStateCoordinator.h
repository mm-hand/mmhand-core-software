/**
 * @file SystemStateCoordinator.h
 * @brief 声明顶层系统状态编排器运行入口。
 * @details 状态迁移、owner 仲裁、READY 门禁和租约恢复均在该编排器内推进。
 */
#ifndef P4_MODULES_STATE_SYSTEM_STATE_COORDINATOR_H
#define P4_MODULES_STATE_SYSTEM_STATE_COORDINATOR_H

#include <stdint.h>
#include "../../runtime/ports/ServiceOutputPort.h"
#include "../../data/ServiceSnapshots.h"

struct SystemRuntimeContext;

/** 顶层状态服务的私有跨周期状态；由 SystemStateTask 独占。 */
struct SystemStateCoordinatorContext {
    uint32_t handledEmergencyStopToken;
    bool initialized;
};

/** 初始化状态服务；不创建任务、不进入循环。 */
bool initSystemStateCoordinator(SystemStateCoordinatorContext* context,
                                SystemRuntimeContext* sharedData);

/** 使用任务层提供的统一时间推进一个 10 ms 状态周期。 */
bool stepSystemStateCoordinator(SystemStateCoordinatorContext* context,
                                SystemRuntimeContext* sharedData,
                                const StateServiceInputSnapshot& input,
                                const ServiceOutputPort& output);

#endif // P4_MODULES_STATE_SYSTEM_STATE_COORDINATOR_H
