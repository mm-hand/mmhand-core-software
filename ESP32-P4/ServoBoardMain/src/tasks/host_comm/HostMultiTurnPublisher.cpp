/**
 * @file HostMultiTurnPublisher.cpp
 * @brief 实现桌面 JSON 多圈恢复最终结果的可靠事件上报。
 */
#include "HostMultiTurnPublisher.h"

#include "HostTelemetryPublisher.h"
#include "../../platform/rtos/AtomicValue.h"
#include "../../platform/rtos/SharedSafetyState.h"

namespace p4::host {
namespace {

uint32_t s_lastRestoreResultTokenSent = 0;

} // namespace

void initHostMultiTurnPublisher()
{
    s_lastRestoreResultTokenSent = 0;
}

bool stepHostMultiTurnPublisher(SystemRuntimeContext* sharedData,
                                     bool syncRequested)
{
    if (!sharedData) {
        return false;
    }

    const uint32_t restoreTokenHint = sharedAtomicLoad(
        &sharedData->restore_multi_turn_result_token);
    const bool needRestore =
        (restoreTokenHint != 0 &&
         restoreTokenHint != s_lastRestoreResultTokenSent) ||
        syncRequested;
    if (!needRestore) {
        return true;
    }

    uint32_t restoreToken = restoreTokenHint;
    uint32_t restoredMask = 0;
    uint32_t rejectedMask = 0;
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
    }

    bool restoreSent = true;
    if (needRestore) {
        restoreSent = sendMultiTurnRestoreResultPacket(
            restoreToken, restoredMask, rejectedMask);
        if (restoreSent) {
            s_lastRestoreResultTokenSent = restoreToken;
        }
    }

    return !syncRequested || restoreSent;
}

} // namespace p4::host
