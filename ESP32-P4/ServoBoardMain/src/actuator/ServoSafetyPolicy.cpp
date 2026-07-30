/**
 * @file ServoSafetyPolicy.cpp
 * @brief 实现与实时硬件状态无关的舵机目标批次安全校验。
 */
#include "ServoSafetyPolicy.h"

namespace p4::actuator {

bool batchStructureIsSafe(const ServoTargetBatch_t& batch)
{
    if (batch.count > SERVO_TARGET_BATCH_MAX) {
        return false;
    }
    for (uint8_t i = 0; i < batch.count; ++i) {
        const ServoTargetCommand_t& command = batch.commands[i];
        if (command.busIndex >= NUM_BUSES || command.servoId > MAX_SERVO_ID) {
            return false;
        }
    }
    return true;
}

} // namespace p4::actuator
