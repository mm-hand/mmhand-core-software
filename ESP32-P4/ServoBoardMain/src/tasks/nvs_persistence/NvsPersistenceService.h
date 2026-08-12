/**
 * @file NvsPersistenceService.h
 * @brief 声明多圈快照的非阻塞持久化服务。
 * @details
 * - 线程：仅由 NvsPersistenceTask 调用。
 * - 输入：深度 1 的最新保存请求、当前运动状态和任务层单调时间。
 * - 输出：NVS A/B 快照、保存结果邮箱和 Reset Zero 事务结果。
 * - 状态所有权：NvsPersistenceServiceContext 由任务栈独占，禁止使用静态可变状态。
 * - 禁止依赖：本服务不得创建任务、永久循环或主动延时。
 */
#ifndef P4_TASKS_NVS_PERSISTENCE_NVS_PERSISTENCE_SERVICE_H
#define P4_TASKS_NVS_PERSISTENCE_NVS_PERSISTENCE_SERVICE_H

#include <stdint.h>

#include "../../data/RecoveryTypes.h"
#include "../../data/PersistenceTypes.h"
#include "../../runtime/ports/ServiceOutputPort.h"

struct SystemRuntimeContext;

/** 持久化服务跨周期上下文；等待运动停止时持续合并最新请求。 */
struct NvsPersistenceServiceContext {
    MultiTurnNvsSaveJob_t pendingJob; ///< 当前等待提交的最高优先级事务。
    ManualNvsSaveJob pendingManualJob; ///< 当前等待提交的最新 Manual 事务。
    NvsControlJob pendingControlJob;    ///< 当前等待提交的最新 NVS 控制事务。
    uint32_t motionStoppedAtMs;       ///< 首次观察到退出运动态的时间。
    bool pending;                     ///< pendingJob 是否有效。
    bool manualPending;               ///< pendingManualJob 是否有效。
    bool controlPending;              ///< pendingControlJob 是否有效。
    bool initialized;                 ///< init 是否成功完成。
};

/** 初始化持久化服务，不访问 Flash。 */
bool initNvsPersistenceService(NvsPersistenceServiceContext* context,
                            SystemRuntimeContext* sharedData);

/**
 * @brief 推进一次持久化状态机。
 * @return 本周期是否实际执行了一次 NVS 事务；等待期间返回 false。
 */
bool stepNvsPersistenceService(NvsPersistenceServiceContext* context,
                            SystemRuntimeContext* sharedData,
                            uint32_t nowMs,
                            const ServiceOutputPort& output);

#endif // P4_TASKS_NVS_PERSISTENCE_NVS_PERSISTENCE_SERVICE_H
