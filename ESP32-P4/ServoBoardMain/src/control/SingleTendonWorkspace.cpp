/**
 * @file SingleTendonWorkspace.cpp
 * @brief 实现单腱关节基于实测标定 LUT 的最终 ABS 安全钳位。
 */
#include "SingleTendonWorkspace.h"

#include "../calibration/CalibrationTask.h"
#include "ControlOutputBuilder.h"
#include "TendonCalibrationLut.h"

namespace p4::control {

bool clampSingleTendonTargetByLut(uint8_t jointIndex,
                                  uint8_t motorChannel,
                                  int32_t requestedAbs,
                                  int32_t* appliedAbs,
                                  int32_t* lutMinAbs,
                                  int32_t* lutMaxAbs)
{
    if (!appliedAbs || !lutMinAbs || !lutMaxAbs ||
        jointIndex >= ENCODER_TOTAL_NUM ||
        motorChannel >= SERVO_TOTAL_NUM) {
        return false;
    }

    const JointCalibrationResult& calibration =
        g_jointCalibResult[jointIndex];
    if (calibration.tendonLutMotor[0] != motorChannel ||
        !tendonCalibrationMotorAbsRange(
            calibration, 0, lutMinAbs, lutMaxAbs)) {
        return false;
    }

    int32_t clamped = requestedAbs;
    if (clamped < *lutMinAbs) {
        clamped = *lutMinAbs;
    }
    if (clamped > *lutMaxAbs) {
        clamped = *lutMaxAbs;
    }
    *appliedAbs = clampServoPos(clamped);
    return true;
}

} // namespace p4::control
