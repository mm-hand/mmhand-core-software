/** @file DirectTargetResult.h
 * @brief 提交普通直控的总线写入结果；代际最后发布以保持快照一致。
 */
#ifndef P4_DIRECT_TARGET_RESULT_H
#define P4_DIRECT_TARGET_RESULT_H

#include "../SystemRuntimeContext.h"
#include "../../platform/rtos/SharedSafetyState.h"
#include "../../platform/rtos/AtomicValue.h"

inline void commitDirectTargetResult(SystemRuntimeContext* runtime,
                                      uint32_t generation,
                                      uint32_t token,
                                      uint32_t requestedMask,
                                      uint32_t appliedMask,
                                      uint32_t rejectedMask)
{
    if (!runtime || generation == 0) return;
    SharedSafetyGuard guard;
    if (runtime->direct_target_request_generation != generation ||
        runtime->direct_target_request_token != token ||
        runtime->direct_target_result_generation == generation) return;
    runtime->direct_target_result_token = token;
    runtime->direct_target_requested_mask = requestedMask;
    runtime->direct_target_applied_mask = appliedMask;
    runtime->direct_target_rejected_mask = rejectedMask;
    runtime->direct_target_status = requestedMask != 0 && appliedMask == requestedMask
        ? DIRECT_TARGET_APPLY_APPLIED : DIRECT_TARGET_APPLY_REJECTED;
    sharedAtomicStore(&runtime->direct_target_result_generation, generation);
}

inline void rejectOutstandingDirectTarget(SystemRuntimeContext* runtime)
{
    if (!runtime) return;
    commitDirectTargetResult(runtime,
        runtime->direct_target_request_generation,
        runtime->direct_target_request_token,
        runtime->direct_target_request_mask, 0,
        runtime->direct_target_request_mask);
}

#endif // P4_DIRECT_TARGET_RESULT_H
