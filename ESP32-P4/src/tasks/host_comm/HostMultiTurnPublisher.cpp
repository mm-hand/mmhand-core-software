/**
 * @file HostMultiTurnPublisher.cpp
 * @brief 实现桌面 JSON 多圈恢复和 logical ABS 设零最终结果的可靠事件上报。
 */
#include "HostMultiTurnPublisher.h"

#include "HostTelemetryPublisher.h"
#include "HostCommService.h"
#include "../../config/protocol/HostProtocol.h"
#include "../../system/rtos/AtomicValue.h"
#include "../../system/rtos/SharedSafetyState.h"

namespace p4::host {
namespace {

uint32_t s_lastRebaseZeroResultTokenSent = 0;
uint32_t s_lastDirectResultGenerationSent = 0;

} // namespace

void initHostMultiTurnPublisher()
{
    s_lastRebaseZeroResultTokenSent = 0;
    s_lastDirectResultGenerationSent = 0;
}

bool stepHostMultiTurnPublisher(SystemRuntimeContext* sharedData,
                                     bool syncRequested)
{
    if (!sharedData) {
        return false;
    }

    const uint32_t rebaseTokenHint = sharedAtomicLoad(
        &sharedData->servo_rebase_zero_result_token);
    const bool needRebase =
        (rebaseTokenHint != 0 &&
         rebaseTokenHint != s_lastRebaseZeroResultTokenSent) ||
        syncRequested;
    const uint32_t directGenerationHint = sharedAtomicLoad(
        &sharedData->direct_target_result_generation);
    const bool needDirectResult = syncRequested ||
        (directGenerationHint != 0 &&
         directGenerationHint != s_lastDirectResultGenerationSent);
    if (!needRebase && !needDirectResult) {
        return true;
    }

    uint32_t rebaseToken = rebaseTokenHint;
    uint32_t rebaseSuccessMask = 0;
    uint32_t rebaseFailedMask = 0;
    uint32_t directGeneration = directGenerationHint;
    uint32_t directToken = 0;
    uint32_t directRequestedMask = 0;
    uint32_t directAppliedMask = 0;
    uint32_t directRejectedMask = 0;
    uint8_t directApplyStatus = 0;
    {
        SharedSafetyGuard guard;
        if (needRebase) {
            rebaseToken = sharedAtomicLoad(
                &sharedData->servo_rebase_zero_result_token);
            rebaseSuccessMask = sharedAtomicLoad(
                &sharedData->servo_rebase_zero_success_mask);
            rebaseFailedMask = sharedAtomicLoad(
                &sharedData->servo_rebase_zero_failed_mask);
        }
        if (needDirectResult) {
            directGeneration = sharedData->direct_target_result_generation;
            directToken = sharedData->direct_target_result_token;
            directRequestedMask = sharedData->direct_target_requested_mask;
            directAppliedMask = sharedData->direct_target_applied_mask;
            directRejectedMask = sharedData->direct_target_rejected_mask;
            directApplyStatus = sharedData->direct_target_status;
        }
    }

    bool rebaseSent = true;
    if (needRebase) {
        if (rebaseToken != 0 &&
            rebaseToken != s_lastRebaseZeroResultTokenSent) {
            // HostComm 独占 pendingStateSyncMask；归零后补发新的静态坐标契约。
            requestHostStateSync(STATE_SYNC_MOTOR_ABS_LIMIT |
                                 STATE_SYNC_SERVO_LOGICAL_RANGE |
                                 STATE_SYNC_JOINT_CALIBRATION);
        }
        rebaseSent = sendServoRebaseZeroResultPacket(
            rebaseToken, rebaseSuccessMask, rebaseFailedMask);
        if (rebaseSent) {
            s_lastRebaseZeroResultTokenSent = rebaseToken;
        }
    }
    bool directResultSent = true;
    if (needDirectResult) {
        directResultSent = sendDirectTargetResultPacket(
            directToken, directRequestedMask, directAppliedMask,
            directRejectedMask, directApplyStatus);
        if (directResultSent) {
            s_lastDirectResultGenerationSent = directGeneration;
        }
    }
    return !syncRequested || (rebaseSent && directResultSent);
}

} // namespace p4::host
