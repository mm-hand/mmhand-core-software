/**
 * @file HostPersistenceCommandHandler.cpp
 * @brief 实现多圈 NVS 控制事务的 Host 侧异步边界。
 */
#include "HostPersistenceCommandHandler.h"

#include <string.h>

#include "../../config/Config.h"
#include "../../modules/protocol/HostProtocol.h"
#include "../../platform/log/FirmwareLog.h"
#include "../../platform/rtos/AtomicValue.h"
#include "../../runtime/SystemRuntimeContext.h"

uint8_t enqueueMultiTurnNvsControl(SystemRuntimeContext* sharedData,
                                   uint8_t action,
                                   uint32_t nowMs)
{
#if MULTITURN_NVS_PERSISTENCE_ENABLE
    if (!sharedData || action > 1U || !sharedData->nvsControlQueue.valid()) {
        return PROTO_ACK_STATUS_UNSUPPORTED_MODE;
    }
    NvsControlJob job = {};
    job.requestToken = p4::platform::rtos::atomicLoad(
                           &sharedData->nvs_control_request_token) + 1U;
    job.queuedAtMs = nowMs;
    job.action = action;
    // 控制命令不可覆盖已 ACK 的请求；队列忙时要求上位机稍后重试。
    if (!sharedData->nvsControlQueue.send(job)) {
        return PROTO_ACK_STATUS_NVS_FAILURE;
    }
    p4::platform::rtos::atomicStore(&sharedData->nvs_control_request_token,
                                    job.requestToken);
    return PROTO_ACK_STATUS_QUEUED;
#else
    (void)sharedData;
    (void)action;
    (void)nowMs;
    return PROTO_ACK_STATUS_UNSUPPORTED_MODE;
#endif
}

bool processMultiTurnNvsControlResult(SystemRuntimeContext* sharedData)
{
    if (!sharedData || !sharedData->nvsControlResultQueue.valid()) {
        return false;
    }
    NvsControlResult result = {};
    bool received = false;
    while (sharedData->nvsControlResultQueue.receive(&result)) {
        received = true;
    }
    if (!received || result.requestToken != p4::platform::rtos::atomicLoad(
            &sharedData->nvs_control_request_token)) {
        return false;
    }

    // 先提交数组和位图，最后提交 snapshot_valid，避免其他任务读到半更新快照。
    sharedData->multiturn_nvs_snapshot_valid = 0;
    sharedData->multiturn_nvs_save_valid_mask = result.snapshotValidMask;
    memcpy(sharedData->multiturn_nvs_snapshot_raw,
           result.snapshotRaw,
           sizeof(sharedData->multiturn_nvs_snapshot_raw));
    sharedData->multiturn_nvs_enabled = result.enabled;
    sharedData->multiturn_nvs_snapshot_valid = result.snapshotValid;

    FW_LOG_INFO_PRINTF(
        "[NVS MULTITURN] control action=%u success=%u enabled=%u snapshot=%u mask=0x%08lX\r\n",
        (unsigned)result.action,
        (unsigned)result.success,
        (unsigned)result.enabled,
        (unsigned)result.snapshotValid,
        (unsigned long)result.snapshotValidMask);
    return true;
}
