/**
 * @file NvsPersistenceService.cpp
 * @brief 非阻塞合并多圈 NVS 请求，并在运动结束后提交一次 Flash 事务。
 */
#include "NvsPersistenceService.h"

#include <string.h>

#include "../../modules/persistence/ManualCalibrationNvsStore.h"
#include "../../modules/persistence/MultiTurnNvsStore.h"
#include "../../config/topology/HardwareMap.h"
#include "../../platform/rtos/AtomicValue.h"
#include "../../platform/rtos/SharedSafetyState.h"
#include "../../runtime/SystemRuntimeContext.h"
#include "../../safety/SafetyEventReporter.h"
#include "../../safety/policies/PersistenceSafetyPolicy.h"

namespace {

constexpr uint32_t kFeedbackSettleAfterMotionMs = 20; ///< 退出运动后等待最终反馈帧。

bool flashSaveBlockedByMotion(const SystemRuntimeContext* sharedData)
{
    if (!sharedData) return false;
    const uint8_t state = sharedData->system_state;
    return state == SYSTEM_STATE_RUN_FEEDBACK ||
           state == SYSTEM_STATE_RUN_DIRECT ||
           state == SYSTEM_STATE_CALIBRATION_RUNNING;
}

/** Reset Zero 已改变硬件坐标，普通周期快照不得覆盖这项高优先级事务。 */
void keepHigherPriorityJob(MultiTurnNvsSaveJob_t* current,
                           const MultiTurnNvsSaveJob_t& candidate)
{
    if (!current) return;
    if (current->operation == MULTITURN_NVS_OPERATION_RESET_ZERO &&
        candidate.operation != MULTITURN_NVS_OPERATION_RESET_ZERO) {
        return;
    }
    *current = candidate;
}

/** 非阻塞吸收邮箱中的全部请求，深度 1 配置下通常只执行一次。 */
void collectLatestJob(NvsPersistenceServiceContext* context,
                      SystemRuntimeContext* sharedData)
{
    if (!context || !sharedData) {
        return;
    }
    MultiTurnNvsSaveJob_t candidate = {};
    while (sharedData->multiturnNvsSaveQueue.receive(&candidate)) {
        if (!context->pending) {
            context->pendingJob = candidate;
            context->pending = true;
        } else {
            keepHigherPriorityJob(&context->pendingJob, candidate);
        }
    }
}

/** 吸收最新 Manual 保存请求；邮箱覆盖语义保证过时编辑不会排队写入 Flash。 */
void collectLatestManualJob(NvsPersistenceServiceContext* context,
                            SystemRuntimeContext* sharedData)
{
    if (!context || !sharedData) {
        return;
    }
    ManualNvsSaveJob candidate = {};
    while (sharedData->manualNvsSaveQueue.receive(&candidate)) {
        context->pendingManualJob = candidate;
        context->manualPending = true;
    }
}

/** 吸收多圈 NVS 开关/清空请求；控制事务不允许被普通快照覆盖。 */
void collectLatestControlJob(NvsPersistenceServiceContext* context,
                             SystemRuntimeContext* sharedData)
{
    if (!context || !sharedData) {
        return;
    }
    NvsControlJob candidate = {};
    while (sharedData->nvsControlQueue.receive(&candidate)) {
        context->pendingControlJob = candidate;
        context->controlPending = true;
    }
}

void publishNvsSaveResult(SystemRuntimeContext* sharedData,
                          const MultiTurnNvsSaveResult_t& result)
{
    if (!sharedData) return;
    const bool ordinarySave = result.operation == MULTITURN_NVS_OPERATION_SAVE;
    if (ordinarySave && result.requestToken != 0) {
        sharedData->multiturn_nvs_save_result_token = result.requestToken;
    }
    if (result.success) {
        sharedData->multiturn_nvs_save_valid_mask = 0;
        for (uint8_t channel = 0; channel < SERVO_TOTAL_NUM; ++channel) {
            if ((result.validMask & (1UL << channel)) != 0) {
                sharedData->multiturn_nvs_snapshot_raw[channel] =
                    result.motorRaw[channel];
            }
        }
        sharedData->multiturn_nvs_save_valid_mask = result.validMask;
        sharedData->multiturn_nvs_snapshot_valid = 1;
    } else if (ordinarySave) {
        p4::platform::rtos::atomicAddFetch(
            &sharedData->multiturn_nvs_save_fail_count, (uint32_t)1);
    }
    sharedData->multiturnNvsResultQueue.overwrite(result);
}

void executePendingJob(SystemRuntimeContext* sharedData,
                       const MultiTurnNvsSaveJob_t& job,
                       uint32_t nowMs,
                       const ServiceOutputPort& output)
{
    MultiTurnNvsSaveResult_t result = {};
    result.requestToken = job.requestToken;
    result.validMask = job.validMask;
    result.reason = job.reason;
    result.turnChanged = job.turnChanged;
    result.edgeChanged = job.edgeChanged;
    result.operation = job.operation;
    result.queuedAtMs = job.queuedAtMs;
    memcpy(result.motorAbs, job.motorAbs, sizeof(result.motorAbs));
    memcpy(result.motorRaw, job.motorRaw, sizeof(result.motorRaw));
    memcpy(result.motorTurn, job.motorTurn, sizeof(result.motorTurn));

    uint32_t sequence = 0;
    uint32_t storedValidMask = job.validMask;
    MultiTurnNvsSnapshot storedSnapshot = {};
    if (job.operation == MULTITURN_NVS_OPERATION_RESET_ZERO) {
        result.success = multiTurnNvsReplaceChannelsWithTurnZero(
            job.validMask, job.motorRaw, &sequence, &storedValidMask,
            &storedSnapshot) ? 1U : 0U;
    } else {
        result.success = multiTurnNvsSaveSnapshot(
            job.validMask, job.motorAbs, job.motorRaw, &sequence,
            &storedValidMask, &storedSnapshot) ? 1U : 0U;
    }
    result.sequence = sequence;
    if (result.success) {
        result.validMask = storedValidMask;
        for (uint8_t channel = 0; channel < SERVO_TOTAL_NUM; ++channel) {
            if ((storedValidMask & (1UL << channel)) == 0) continue;
            result.motorRaw[channel] = storedSnapshot.raw[channel];
            result.motorTurn[channel] = storedSnapshot.turns[channel];
            result.motorAbs[channel] =
                multiTurnSnapshotAbsForChannel(storedSnapshot, channel);
        }
    }
    result.completedAtMs = nowMs;

    const SafetyRuleId writeFailureRule =
        p4::safety::persistenceMultiTurnWriteFailureRule(
            result.success != 0,
            job.operation == MULTITURN_NVS_OPERATION_RESET_ZERO);
    if (writeFailureRule != SafetyRuleId::None) {
        p4::safety::reportSafetyRuleEvent(
            sharedData,
            writeFailureRule,
            (int32_t)job.validMask,
            (int32_t)job.operation);
    }

    if (job.operation == MULTITURN_NVS_OPERATION_RESET_ZERO) {
        sharedData->servo_reset_zero_success_mask = job.validMask;
        if (result.success) {
            sharedData->servo_reset_zero_failed_mask &= ~job.validMask;
            sharedData->servo_reset_zero_status = SERVO_RESET_ZERO_STATUS_SUCCESS;
        } else {
            sharedData->servo_reset_zero_failed_mask |= job.validMask;
            sharedData->servo_reset_zero_status = SERVO_RESET_ZERO_STATUS_FAILED;
            sharedAtomicFetchOr(&sharedData->multi_turn_unknown_bitmap,
                                job.validMask);
        }
        // token 最后发布，作为上位机读取整组结果的提交点。
        sharedAtomicStore(&sharedData->servo_reset_zero_result_token,
                          job.requestToken);
    }
    publishNvsSaveResult(sharedData, result);
}

/** 执行一次 Manual A/B 槽保存，并把读回校验结果发回 Host 任务提交 RAM。 */
void executePendingManualJob(SystemRuntimeContext* sharedData,
                             const ManualNvsSaveJob& job,
                             uint32_t nowMs,
                             const ServiceOutputPort& output)
{
    ManualEncoderReference references[ENCODER_TOTAL_NUM] = {};
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; ++joint) {
        references[joint].valid =
            (job.validMask & (1UL << joint)) != 0U ? 1U : 0U;
        references[joint].minRaw = job.minRaw[joint];
        references[joint].maxRaw = job.maxRaw[joint];
    }

    ManualNvsSaveResult result = {};
    result.requestToken = job.requestToken;
    result.validMask = job.validMask;
    memcpy(result.minRaw, job.minRaw, sizeof(result.minRaw));
    memcpy(result.maxRaw, job.maxRaw, sizeof(result.maxRaw));
    result.success = manualCalibrationNvsSave(
        references,
        &result.sequence,
        &result.referenceCrc) ? 1U : 0U;
    result.completedAtMs = nowMs;

    const SafetyRuleId manualFailureRule =
        p4::safety::persistenceManualWriteFailureRule(result.success != 0);
    if (manualFailureRule != SafetyRuleId::None) {
        p4::safety::reportSafetyRuleEvent(
            sharedData,
            manualFailureRule,
            static_cast<int32_t>(job.validMask),
            0);
    }
    sharedData->manualNvsResultQueue.overwrite(result);
}

/** 执行用户显式发起的多圈 NVS 开关事务，并读回实际 Flash 状态。 */
void executePendingControlJob(SystemRuntimeContext* sharedData,
                              const NvsControlJob& job,
                              uint32_t nowMs,
                              const ServiceOutputPort& output)
{
    NvsControlResult result = {};
    result.requestToken = job.requestToken;
    result.action = job.action;

    bool operationOk = false;
    if (job.action == 0U) {
        const bool disableOk = multiTurnNvsSetEnabled(false);
        const bool clearOk = disableOk && multiTurnNvsClearSnapshots();
        operationOk = disableOk && clearOk;
    } else if (job.action == 1U) {
        operationOk = multiTurnNvsSetEnabled(true);
    }

    bool enabled = false;
    const bool enabledReadOk = multiTurnNvsLoadEnabled(&enabled);
    MultiTurnNvsSnapshot snapshot = {};
    const bool snapshotValid = enabledReadOk && enabled &&
                               multiTurnNvsLoadLatest(&snapshot);
    result.success = operationOk && enabledReadOk ? 1U : 0U;
    result.enabled = enabledReadOk && enabled ? 1U : 0U;
    result.snapshotValid = snapshotValid ? 1U : 0U;
    if (snapshotValid) {
        result.snapshotValidMask = snapshot.validMask & validMotorChannelMask();
        memcpy(result.snapshotRaw, snapshot.raw, sizeof(result.snapshotRaw));
    }
    result.completedAtMs = nowMs;

    const SafetyRuleId controlFailureRule =
        p4::safety::persistenceControlFailureRule(result.success != 0);
    if (controlFailureRule != SafetyRuleId::None) {
        p4::safety::reportSafetyRuleEvent(
            sharedData,
            controlFailureRule,
            static_cast<int32_t>(job.action),
            enabledReadOk ? static_cast<int32_t>(enabled) : -1);
    }
    sharedData->nvsControlResultQueue.overwrite(result);
}

} // namespace

bool initNvsPersistenceService(NvsPersistenceServiceContext* context,
                            SystemRuntimeContext* sharedData)
{
    if (!context || !sharedData ||
        !sharedData->multiturnNvsSaveQueue.valid() ||
        !sharedData->manualNvsSaveQueue.valid() ||
        !sharedData->nvsControlQueue.valid() ||
        !sharedData->nvsControlResultQueue.valid()) {
        return false;
    }
    memset(context, 0, sizeof(*context));
    context->initialized = true;
    return true;
}

bool stepNvsPersistenceService(NvsPersistenceServiceContext* context,
                            SystemRuntimeContext* sharedData,
                            uint32_t nowMs,
                            const ServiceOutputPort& output)
{
    if (!context || !context->initialized || !sharedData) return false;
    collectLatestJob(context, sharedData);
    collectLatestManualJob(context, sharedData);
    collectLatestControlJob(context, sharedData);
    if (!context->pending && !context->manualPending &&
        !context->controlPending) return false;

    if (flashSaveBlockedByMotion(sharedData)) {
        context->motionStoppedAtMs = 0;
        return false;
    }
    if (context->motionStoppedAtMs == 0) {
        context->motionStoppedAtMs = nowMs;
        return false;
    }
    if ((uint32_t)(nowMs - context->motionStoppedAtMs) <
        kFeedbackSettleAfterMotionMs) {
        return false;
    }

    context->motionStoppedAtMs = 0;
    // 用户显式控制 > Manual 维护 > 可合并的多圈周期快照。
    if (context->controlPending) {
        const NvsControlJob job = context->pendingControlJob;
        context->controlPending = false;
        executePendingControlJob(sharedData, job, nowMs, output);
    } else if (context->manualPending) {
        const ManualNvsSaveJob job = context->pendingManualJob;
        context->manualPending = false;
        executePendingManualJob(sharedData, job, nowMs, output);
    } else {
        const MultiTurnNvsSaveJob_t job = context->pendingJob;
        context->pending = false;
        executePendingJob(sharedData, job, nowMs, output);
    }
    return true;
}
