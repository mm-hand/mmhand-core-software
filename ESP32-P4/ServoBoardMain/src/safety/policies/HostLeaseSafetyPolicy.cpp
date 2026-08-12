/**
 * @file HostLeaseSafetyPolicy.cpp
 * @brief 实现不依赖硬件和 FreeRTOS 的上位机控制租约判断。
 */
#include "HostLeaseSafetyPolicy.h"

#include "../../config/SafetyConfig.h"
#include "../../data/ActuatorTypes.h"
#include "../../data/SystemTypes.h"

namespace p4::system {

bool requiresHostControlLease(uint8_t systemState, uint8_t targetOwner)
{
    if (targetOwner == SERVO_TARGET_OWNER_CONTROL) {
        return systemState == SYSTEM_STATE_RUN_DIRECT ||
               systemState == SYSTEM_STATE_RUN_FEEDBACK;
    }
    if (targetOwner == SERVO_TARGET_OWNER_CALIBRATION) {
        return systemState == SYSTEM_STATE_CALIBRATION_RUNNING ||
               systemState == SYSTEM_STATE_ROPE_INIT_RUNNING;
    }
    return false;
}

bool isHostControlLeaseFresh(uint32_t nowMs, uint32_t lastHeartbeatMs)
{
    return lastHeartbeatMs != 0 &&
           (uint32_t)(nowMs - lastHeartbeatMs) <=
               mmhand::config::safety::kHostControlLeaseTimeoutMs;
}

} // namespace p4::system
