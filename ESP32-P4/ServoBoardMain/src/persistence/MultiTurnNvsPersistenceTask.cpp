#include "MultiTurnNvsPersistenceTask.h"

#include "MultiTurnNvsStore.h"

#include <string.h>

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
        result.success = multiTurnNvsSaveSnapshot(job.validMask, job.motorAbs, job.motorRaw, &sequence) ? 1U : 0U;
        result.sequence = sequence;
        result.completedAtMs = millis();
        publishNvsSaveResult(sharedData, &result);
    }
}
