#include "MultiTurnNvsPersistenceTask.h"

#include "MultiTurnNvsStore.h"
#include "../shared/SharedSafetyState.h"

#include <string.h>

static bool flashSaveBlockedByMotion(const TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return false;
    }
    const uint8_t state = sharedData->system_state;
    return state == SYSTEM_STATE_RUN_FEEDBACK ||
           state == SYSTEM_STATE_RUN_DIRECT ||
           state == SYSTEM_STATE_CALIBRATION_RUNNING;
}

static void keepHigherPriorityJob(MultiTurnNvsSaveJob_t* current,
                                  const MultiTurnNvsSaveJob_t& candidate)
{
    if (!current) {
        return;
    }
    // Reset Zero 已改变舵机硬件坐标，必须优先提交新的 turn0。
    // 普通周期快照不能在等待期间覆盖这项事务。
    if (current->operation == MULTITURN_NVS_OPERATION_RESET_ZERO &&
        candidate.operation != MULTITURN_NVS_OPERATION_RESET_ZERO) {
        return;
    }
    *current = candidate;
}

static void waitForMotionCompleteAndKeepLatestJob(
    TaskSharedData_t* sharedData,
    MultiTurnNvsSaveJob_t* job)
{
    if (!sharedData || !job || !sharedData->multiturnNvsSaveQueue) {
        return;
    }

    MultiTurnNvsSaveJob_t newerJob;
    for (;;) {
        while (flashSaveBlockedByMotion(sharedData)) {
            if (xQueueReceive(sharedData->multiturnNvsSaveQueue,
                              &newerJob,
                              pdMS_TO_TICKS(20)) == pdTRUE) {
                keepHigherPriorityJob(job, newerJob);
            }
        }

        // 退出运动状态后再等待一帧，让舵机任务发布最终反馈快照。
        vTaskDelay(pdMS_TO_TICKS(20));
        if (flashSaveBlockedByMotion(sharedData)) {
            continue;
        }
        while (xQueueReceive(sharedData->multiturnNvsSaveQueue,
                             &newerJob,
                             0) == pdTRUE) {
            keepHigherPriorityJob(job, newerJob);
        }
        return;
    }
}

static void publishNvsSaveResult(TaskSharedData_t* sharedData,
                                 const MultiTurnNvsSaveResult_t* result)
{
    if (!sharedData || !result) {
        return;
    }

    const bool ordinarySave =
        result->operation == MULTITURN_NVS_OPERATION_SAVE;
    if (ordinarySave && result->requestToken != 0) {
        sharedData->multiturn_nvs_save_result_token = result->requestToken;
    }
    if (result->success) {
        // 先撤销有效位，再更新 raw，最后一次性发布新 mask。
        sharedData->multiturn_nvs_save_valid_mask = 0;
        for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
            if ((result->validMask & (1UL << ch)) != 0) {
                sharedData->multiturn_nvs_snapshot_raw[ch] =
                    result->motorRaw[ch];
            }
        }
        sharedData->multiturn_nvs_save_valid_mask = result->validMask;
        sharedData->multiturn_nvs_snapshot_valid = 1;
    } else if (ordinarySave) {
        // Reset Zero 使用独立状态和失败掩码，不污染普通保存失败计数。
        sharedData->multiturn_nvs_save_fail_count++;
    }

    if (sharedData->multiturnNvsResultQueue) {
        xQueueOverwrite(sharedData->multiturnNvsResultQueue, result);
    }
}

void multiTurnNvsPersistenceTask(void* parameter)
{
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    if (!sharedData || !sharedData->multiturnNvsSaveQueue) {
        vTaskDelete(NULL);
        return;
    }

    MultiTurnNvsSaveJob_t job;
    MultiTurnNvsSaveResult_t result;
    for (;;) {
        if (xQueueReceive(sharedData->multiturnNvsSaveQueue,
                          &job,
                          portMAX_DELAY) != pdTRUE) {
            continue;
        }

        waitForMotionCompleteAndKeepLatestJob(sharedData, &job);

        memset(&result, 0, sizeof(result));
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
        MultiTurnNvsSnapshot storedSnapshot;
        memset(&storedSnapshot, 0, sizeof(storedSnapshot));
        if (job.operation == MULTITURN_NVS_OPERATION_RESET_ZERO) {
            result.success = multiTurnNvsReplaceChannelsWithTurnZero(
                job.validMask,
                job.motorRaw,
                &sequence,
                &storedValidMask,
                &storedSnapshot) ? 1U : 0U;
        } else {
            result.success = multiTurnNvsSaveSnapshot(
                job.validMask,
                job.motorAbs,
                job.motorRaw,
                &sequence,
                &storedValidMask,
                &storedSnapshot) ? 1U : 0U;
        }
        result.sequence = sequence;
        if (result.success) {
            result.validMask = storedValidMask;
            for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
                if ((storedValidMask & (1UL << ch)) == 0) {
                    continue;
                }
                result.motorRaw[ch] = storedSnapshot.raw[ch];
                result.motorTurn[ch] = storedSnapshot.turns[ch];
                result.motorAbs[ch] =
                    multiTurnSnapshotAbsForChannel(storedSnapshot, ch);
            }
        }
        result.completedAtMs = millis();

        if (job.operation == MULTITURN_NVS_OPERATION_RESET_ZERO) {
            // 硬件 RESET 已执行，因此无论 Flash 成败都保留 success mask，
            // 供上层失效对应关节标定；failed mask 表示持久化是否闭环。
            sharedData->servo_reset_zero_success_mask = job.validMask;
            if (result.success) {
                sharedData->servo_reset_zero_failed_mask &= ~job.validMask;
                sharedData->servo_reset_zero_status =
                    SERVO_RESET_ZERO_STATUS_SUCCESS;
            } else {
                sharedData->servo_reset_zero_failed_mask |= job.validMask;
                sharedData->servo_reset_zero_status =
                    SERVO_RESET_ZERO_STATUS_FAILED;
                sharedAtomicFetchOr(
                    &sharedData->multi_turn_unknown_bitmap, job.validMask);
            }
            // 最终 token 必须最后发布，作为上位机读取整组结果的提交点。
            sharedAtomicStore(
                &sharedData->servo_reset_zero_result_token,
                job.requestToken);
        }
        publishNvsSaveResult(sharedData, &result);
    }
}
