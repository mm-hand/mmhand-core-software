/**
 * @file CalibrationValidity.h
 * @brief 统一定义同一份 Manual/LUT 快照的版本与端点一致性判断。
 */
#pragma once
#include "CalibrationRuntimeStore.h"
#include "ManualEncoderReferenceStore.h"
#include "../../generated/UrdfJointLimits.generated.h"

inline bool calibrationMatchesManual(const JointCalibrationRuntimeSnapshot& snapshot, uint32_t manualCrc)
{
    const auto& result = snapshot.result;
    return snapshot.manual.valid && result.success &&
        result.mappingVersion == kManualScaledUrdfFeedbackMappingVersion &&
        result.urdfConfigCrc == kManualScaledUrdfConfigCrc &&
        result.manualReferenceCrc == manualCrc;
}

inline bool calibrationMatchesManual(uint8_t joint, const JointCalibrationRuntimeSnapshot& snapshot)
{
    return calibrationMatchesManual(snapshot, manualEncoderReferenceEntryCrc(joint, snapshot.manual));
}
