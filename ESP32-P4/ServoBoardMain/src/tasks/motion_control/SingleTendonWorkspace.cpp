/**
 * @file SingleTendonWorkspace.cpp
 * @brief 实现单腱关节基于实测标定 LUT 的最终 ABS 安全钳位。
 */
#include "SingleTendonWorkspace.h"

#include "ServoTargetBuilder.h"
#include "../../modules/calibration/CalibrationRuntimeStore.h"
#include "../../modules/calibration/TendonCalibrationLut.h"
#include "../../safety/policies/ControlBoundarySafetyPolicy.h"

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

    bool boundaryValid = false;
    bool targetClamped = false;
    const int32_t clamped = p4::safety::clampAbsToBoundary(
        requestedAbs,
        *lutMinAbs,
        *lutMaxAbs,
        &boundaryValid,
        &targetClamped);
    (void)targetClamped;
    if (!boundaryValid) return false;
    *appliedAbs = clamped;
    return true;
}

} // namespace p4::control
