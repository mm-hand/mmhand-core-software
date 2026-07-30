/**
 * @file ActuatorCommandGateway.cpp
 * @brief 实现舵机目标单槽邮箱，并集中检查批次数量和队列有效性。
 */
#include "ActuatorCommandGateway.h"

#include "ActuatorCommandArbiter.h"
#include "ServoSafetyPolicy.h"
#include "../shared/TaskSharedData.h"

namespace p4::actuator {

bool publishLatest(TaskSharedData_t* sharedData,
                   const ServoTargetBatch_t& batch)
{
    if (!sharedData || !batchStructureIsSafe(batch)) {
        return false;
    }
    if (batch.count == 0) {
        return true;
    }
    if (!producerOwnsOutput(sharedData, batch.source)) {
        return false;
    }
    return sharedData->servoTargetQueue &&
           xQueueOverwrite(sharedData->servoTargetQueue, &batch) == pdTRUE;
}

void clearPending(TaskSharedData_t* sharedData)
{
    if (sharedData && sharedData->servoTargetQueue) {
        xQueueReset(sharedData->servoTargetQueue);
    }
}

bool receiveLatest(TaskSharedData_t* sharedData,
                   ServoTargetBatch_t* outBatch)
{
    return sharedData && outBatch && sharedData->servoTargetQueue &&
           xQueueReceive(sharedData->servoTargetQueue, outBatch, 0) == pdTRUE;
}

UBaseType_t pendingCount(const TaskSharedData_t* sharedData)
{
    return sharedData && sharedData->servoTargetQueue
               ? uxQueueMessagesWaiting(sharedData->servoTargetQueue)
               : 0;
}

} // namespace p4::actuator
