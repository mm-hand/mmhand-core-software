/**
 * @file HostMultiTurnPublisher.cpp
 * @brief 实现多圈恢复最终结果和 Flash Raw 快照的可靠事件上报。
 */
#include "HostMultiTurnPublisher.h"

#include <string.h>

#include "HostTelemetryPublisher.h"
#include "../../platform/rtos/AtomicValue.h"
#include "../../platform/rtos/SharedSafetyState.h"

namespace p4::host {
namespace {

uint32_t s_lastRestoreResultTokenSent = 0;
uint32_t s_lastSnapshotResultTokenSent = 0xFFFFFFFFUL;

} // namespace

void initHostMultiTurnPublisher()
{
    s_lastRestoreResultTokenSent = 0;
    // 强制启动后发送一次当前 Flash 快照；重连时还可由 STATE_SYNC 再次请求。
    s_lastSnapshotResultTokenSent = 0xFFFFFFFFUL;
}

bool stepHostMultiTurnPublisher(SystemRuntimeContext* sharedData,
                                     bool syncRequested)
{
    if (!sharedData) {
        return false;
    }

    const uint32_t restoreTokenHint = sharedAtomicLoad(
        &sharedData->restore_multi_turn_result_token);
    const uint32_t snapshotTokenHint = sharedAtomicLoad(
        &sharedData->multiturn_nvs_save_result_token);
    const bool needRestore =
        (restoreTokenHint != 0 &&
         restoreTokenHint != s_lastRestoreResultTokenSent) ||
        syncRequested;
    const bool needSnapshot =
        snapshotTokenHint != s_lastSnapshotResultTokenSent || syncRequested;
    if (!needRestore && !needSnapshot) {
        return true;
    }

    uint32_t restoreToken = restoreTokenHint;
    uint32_t restoredMask = 0;
    uint32_t rejectedMask = 0;
    uint32_t snapshotToken = 0;
    uint32_t snapshotValidMask = 0;
    int16_t snapshotRaw[SERVO_TOTAL_NUM] = {0};
    {
        SharedSafetyGuard guard;
        if (needRestore) {
            restoreToken = sharedAtomicLoad(
                &sharedData->restore_multi_turn_result_token);
            restoredMask = sharedAtomicLoad(
                &sharedData->restore_multi_turn_restored_mask);
            rejectedMask = sharedAtomicLoad(
                &sharedData->restore_multi_turn_rejected_mask);
        }
        if (needSnapshot) {
            snapshotToken = sharedAtomicLoad(
                &sharedData->multiturn_nvs_save_result_token);
            snapshotValidMask = sharedAtomicLoad(
                &sharedData->multiturn_nvs_save_valid_mask);
            memcpy(snapshotRaw,
                   sharedData->multiturn_nvs_snapshot_raw,
                   sizeof(snapshotRaw));
        }
    }

    bool restoreSent = true;
    if (needRestore) {
        restoreSent = sendMultiTurnRestoreResultPacket(
            restoreToken, restoredMask, rejectedMask);
        if (restoreSent) {
            s_lastRestoreResultTokenSent = restoreToken;
        }
    }

    bool snapshotSent = true;
    if (needSnapshot) {
        snapshotSent = sendMultiTurnSnapshotRawPacket(
            snapshotToken, snapshotValidMask, snapshotRaw);
        if (snapshotSent) {
            s_lastSnapshotResultTokenSent = snapshotToken;
        }
    }
    return !syncRequested || (restoreSent && snapshotSent);
}

} // namespace p4::host
