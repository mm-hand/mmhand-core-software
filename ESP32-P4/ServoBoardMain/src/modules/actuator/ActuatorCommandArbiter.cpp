/**
 * @file ActuatorCommandArbiter.cpp
 * @brief 实现基于状态机 owner 的舵机命令生产者仲裁。
 */
#include "ActuatorCommandArbiter.h"

#include "../../data/ActuatorTypes.h"
#include "../../platform/rtos/SharedSafetyState.h"
#include "../../runtime/SystemRuntimeContext.h"

namespace p4::actuator {

bool producerOwnsOutput(const SystemRuntimeContext* sharedData,
                        uint8_t producerOwner)
{
    if (!sharedData ||
        (producerOwner != SERVO_TARGET_OWNER_CONTROL &&
         producerOwner != SERVO_TARGET_OWNER_CALIBRATION)) {
        return false;
    }
    return sharedAtomicLoad(&sharedData->servo_target_owner) == producerOwner;
}

} // namespace p4::actuator
