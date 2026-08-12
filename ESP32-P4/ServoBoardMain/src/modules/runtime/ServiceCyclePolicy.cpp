/**
 * @file ServiceCyclePolicy.cpp
 * @brief 实现无需 TaskSharedData、Queue 或 RTOS 的周期快照基础校验。
 */
#include "ServiceCyclePolicy.h"

#include "../../config/DimensionsConfig.h"

namespace {
constexpr uint32_t kValidMotorMask = (1UL << SERVO_TOTAL_NUM) - 1UL;
constexpr uint32_t kValidJointMask = (1UL << ENCODER_TOTAL_NUM) - 1UL;
}

bool controlSnapshotHasActiveScope(const ControlServiceInputSnapshot& input)
{
    return ((input.activeMotorMask & kValidMotorMask) != 0) ||
           ((input.activeJointMask & kValidJointMask) != 0);
}

bool servoSnapshotIsConsistent(const ServoServiceInputSnapshot& input)
{
    return (input.activeMotorMask & ~kValidMotorMask) == 0 &&
           (input.calibrationAuxMotorMask & ~kValidMotorMask) == 0;
}

bool calibrationSnapshotHasSelection(
    const CalibrationServiceInputSnapshot& input)
{
    return (input.activeJointMask & kValidJointMask) != 0 &&
           (input.activeMotorMask & kValidMotorMask) != 0;
}

bool stateSnapshotIsConsistent(const StateServiceInputSnapshot& input)
{
    return (input.activeMotorMask & ~kValidMotorMask) == 0 &&
           (input.activeJointMask & ~kValidJointMask) == 0;
}
