/**
 * @file ActuatorCommandGateway.cpp
 * @brief 实现舵机目标单槽邮箱，并集中检查批次数量和队列有效性。
 */
#include "ActuatorCommandGateway.h"

#include "ActuatorCommandArbiter.h"
#include "ServoSafetyPolicy.h"
#include "../shared/SharedSafetyState.h"
#include "../shared/TaskSharedData.h"

namespace p4::actuator {

/** 发布最新目标；单槽邮箱会覆盖尚未消费的旧批次，保证控制端只执行最新意图。 */
bool publishLatest(TaskSharedData_t* sharedData,
                   const ServoTargetBatch_t& batch)
{
    const ServoSafetyDecision structureDecision =
        evaluateBatchStructure(batch);
    if (!sharedData || !structureDecision.allowed) {
        return false;
    }
    if (batch.count == 0) {
        return true;
    }
    // 租约失联 Hold 一旦开始，普通控制/标定生产者不得再覆盖单槽邮箱中的
    // 当前位置保持批次。Hold 本身仍需通过 owner 仲裁和执行层二次校验。
    if (batch.purpose == SERVO_TARGET_PURPOSE_NORMAL &&
        sharedAtomicLoad(&sharedData->host_control_hold_status) ==
            HOST_CONTROL_HOLD_PENDING) {
        return false;
    }
    if (!producerOwnsOutput(sharedData, batch.source)) {
        return false;
    }
    return sharedData->servoTargetQueue &&
           xQueueOverwrite(sharedData->servoTargetQueue, &batch) == pdTRUE;
}

/** 模式切换、E-STOP 或 owner 释放时清空尚未执行的旧目标。 */
void clearPending(TaskSharedData_t* sharedData)
{
    if (sharedData && sharedData->servoTargetQueue) {
        xQueueReset(sharedData->servoTargetQueue);
    }
}

/** 仅供 ServoCommunicationTask 单消费者非阻塞取出最新批次。 */
bool receiveLatest(TaskSharedData_t* sharedData,
                   ServoTargetBatch_t* outBatch)
{
    return sharedData && outBatch && sharedData->servoTargetQueue &&
           xQueueReceive(sharedData->servoTargetQueue, outBatch, 0) == pdTRUE;
}

/** 返回邮箱中等待批次数，仅用于任务诊断，不参与控制决策。 */
UBaseType_t pendingCount(const TaskSharedData_t* sharedData)
{
    return sharedData && sharedData->servoTargetQueue
               ? uxQueueMessagesWaiting(sharedData->servoTargetQueue)
               : 0;
}

} // namespace p4::actuator
