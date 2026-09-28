/**
 * @file CalibrationSafetyPolicy.cpp
 * @brief 实现自动标定关节列表、释放行程、命令范围和负载保护判断。
 */
#include "CalibrationSafetyPolicy.h"

namespace p4::safety {

CalibrationSequenceDecision validateCalibrationSequence(
    bool contextAvailable,
    const uint8_t* joints,
    uint8_t jointCount,
    uint8_t maximumJointCount)
{
    CalibrationSequenceDecision result = {
        false, CalibrationSequenceReject::NullContext, 0, 0xFF, 0};
    if (!contextAvailable || !joints) return result;
    if (jointCount == 0 || jointCount > maximumJointCount) {
        result.reason = CalibrationSequenceReject::EmptyOrOversized;
        return result;
    }

    result.seenJointMask = 0;
    for (uint8_t index = 0; index < jointCount; ++index) {
        const uint8_t joint = joints[index];
        result.itemIndex = index;
        result.jointIndex = joint;
        if (joint >= maximumJointCount) {
            result.reason = CalibrationSequenceReject::JointOutOfRange;
            return result;
        }
        const uint32_t bit = 1UL << joint;
        if ((result.seenJointMask & bit) != 0) {
            result.reason = CalibrationSequenceReject::DuplicateJoint;
            return result;
        }
        result.seenJointMask |= bit;
    }

    result.allowed = true;
    result.reason = CalibrationSequenceReject::None;
    result.itemIndex = 0xFF;
    result.jointIndex = 0xFF;
    return result;
}

bool calibrationReleaseExceeded(int32_t releaseCounts,
                                int32_t maximumReleaseCounts)
{
    return maximumReleaseCounts < 0 || releaseCounts > maximumReleaseCounts;
}

bool calibrationCommandInsideRange(bool rangeValid,
                                   int32_t commandAbs,
                                   int32_t rangeMinimum,
                                   int32_t rangeMaximum)
{
    return rangeValid && rangeMinimum <= rangeMaximum &&
           commandAbs >= rangeMinimum && commandAbs <= rangeMaximum;
}

bool calibrationLoadOverloaded(int16_t loadMagnitude,
                               int16_t overloadThreshold)
{
    return overloadThreshold >= 0 && loadMagnitude >= overloadThreshold;
}

} // namespace p4::safety
