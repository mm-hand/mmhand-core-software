/**
 * @file ActuatorCommandGateway.cpp
 * @brief 实现舵机目标单槽邮箱，并集中检查批次数量和队列有效性。
 */
#include "ActuatorCommandGateway.h"

#include "ActuatorCommandArbiter.h"
#include "../../safety/policies/ActuatorSafetyPolicy.h"
#include "../../safety/SafetyEventReporter.h"
#include "../../platform/rtos/SharedSafetyState.h"
#include "../../runtime/SystemRuntimeContext.h"

namespace p4::actuator {

/** 发布最新目标；单槽邮箱会覆盖尚未消费的旧批次，保证控制端只执行最新意图。 */
bool publishLatest(SystemRuntimeContext* sharedData,
                   const ServoTargetBatch_t& batch)
{
    if (!sharedData) {
        return false;
    }
    const uint8_t systemState = sharedAtomicLoad(&sharedData->system_state);
    const uint8_t owner = sharedAtomicLoad(&sharedData->servo_target_owner);
    const ServoSafetyDecision structureDecision =
        p4::actuator::evaluateBatchStructure(batch);
    if (!structureDecision.allowed) {
        p4::safety::reportServoSafetyDecision(structureDecision,
                                              systemState,
                                              owner);
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
    return sharedData->servoTargetQueue.overwrite(batch);
}

/** 模式切换、E-STOP 或 owner 释放时清空尚未执行的旧目标。 */
void clearPending(SystemRuntimeContext* sharedData)
{
    if (sharedData) {
        sharedData->servoTargetQueue.reset();
    }
}

/** 仅供 ServoBusTask 单消费者非阻塞取出最新批次。 */
bool receiveLatest(SystemRuntimeContext* sharedData,
                   ServoTargetBatch_t* outBatch)
{
    return sharedData && outBatch &&
           sharedData->servoTargetQueue.receive(outBatch);
}

/** 返回邮箱中等待批次数，仅用于任务诊断，不参与控制决策。 */
UBaseType_t pendingCount(const SystemRuntimeContext* sharedData)
{
    return sharedData
               ? (UBaseType_t)sharedData->servoTargetQueue.pending()
               : 0;
}

} // namespace p4::actuator
