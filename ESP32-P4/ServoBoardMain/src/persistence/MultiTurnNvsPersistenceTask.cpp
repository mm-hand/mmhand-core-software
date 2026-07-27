#include "MultiTurnNvsPersistenceTask.h"

#include "MultiTurnNvsStore.h"

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

static void waitForMotionCompleteAndKeepLatestJob(TaskSharedData_t* sharedData,
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
                *job = newerJob;
            }
        }

        // Let the servo task publish the completion snapshot after the state
        // transition, then collapse all waiting requests to the newest one.
        vTaskDelay(pdMS_TO_TICKS(20));
        if (flashSaveBlockedByMotion(sharedData)) {
            continue;
        }
        while (xQueueReceive(sharedData->multiturnNvsSaveQueue,
                             &newerJob,
                             0) == pdTRUE) {
            *job = newerJob;
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

    if (result->requestToken != 0) {
        sharedData->multiturn_nvs_save_result_token = result->requestToken;
    }
    if (result->success) {
        // 复制期间先撤销有效位，最后一次性发布新 mask，避免上位机看到半新半旧的一帧。
        sharedData->multiturn_nvs_save_valid_mask = 0;
        // 先更新各通道 raw，最后发布 valid mask，避免上位机把尚未复制完成的数据当成有效快照。
        for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
            if ((result->validMask & (1UL << ch)) != 0) {
                sharedData->multiturn_nvs_snapshot_raw[ch] =
                    result->motorRaw[ch];
            }
        }
        sharedData->multiturn_nvs_save_valid_mask = result->validMask;
        sharedData->multiturn_nvs_snapshot_valid = 1;
    } else {
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
        if (xQueueReceive(sharedData->multiturnNvsSaveQueue, &job, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        waitForMotionCompleteAndKeepLatestJob(sharedData, &job);

        memset(&result, 0, sizeof(result));
        result.requestToken = job.requestToken;
        result.validMask = job.validMask;
        result.reason = job.reason;
        result.turnChanged = job.turnChanged;
        result.edgeChanged = job.edgeChanged;
        result.queuedAtMs = job.queuedAtMs;
        memcpy(result.motorAbs, job.motorAbs, sizeof(result.motorAbs));
        memcpy(result.motorRaw, job.motorRaw, sizeof(result.motorRaw));
        memcpy(result.motorTurn, job.motorTurn, sizeof(result.motorTurn));

        uint32_t sequence = 0;
        uint32_t storedValidMask = job.validMask;
        MultiTurnNvsSnapshot storedSnapshot;
        memset(&storedSnapshot, 0, sizeof(storedSnapshot));
        result.success = multiTurnNvsSaveSnapshot(job.validMask,
                                                  job.motorAbs,
                                                  job.motorRaw,
                                                  &sequence,
                                                  &storedValidMask,
                                                  &storedSnapshot) ? 1U : 0U;
        result.sequence = sequence;
        if (result.success) {
            result.validMask = storedValidMask;
            for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
                if ((storedValidMask & (1UL << ch)) == 0) {
                    continue;
                }
                result.motorRaw[ch] = storedSnapshot.raw[ch];
                result.motorTurn[ch] = storedSnapshot.turns[ch];
                result.motorAbs[ch] = multiTurnSnapshotAbsForChannel(storedSnapshot, ch);
            }
        }
        result.completedAtMs = millis();
        publishNvsSaveResult(sharedData, &result);
    }
}
