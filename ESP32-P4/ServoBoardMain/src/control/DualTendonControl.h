#ifndef DUAL_TENDON_CONTROL_H
#define DUAL_TENDON_CONTROL_H

#include <Arduino.h>
#include "../shared/TaskSharedData.h"

struct DualTendonMotorTargets {
    uint8_t motorChannel[2];
    int32_t targetAbs[2];
    float motorRef[2];
    float motorFbModel[2];
    float motorErr[2];
    float pidCorrection[2];
    uint8_t count;
};

bool isDualTendonJoint(uint8_t jointIndex);
void resetDualTendonController(uint8_t jointIndex);

// Block only commands that continue releasing after actual feedback reaches
// the calibrated release allowance. Tightening commands always pass.
int32_t clampDualTendonReleaseFromCalib(uint8_t jointIndex,
                                       uint8_t motorChannel,
                                       int32_t calibratedAnchorRawAbs,
                                       int32_t currentRawAbs,
                                       int32_t targetRawAbs);

bool computeDualTendonMotorTargets(uint8_t jointIndex,
                                   float qRefDeg,
                                   float qFbDeg,
                                   const ServoAngleData_t* servoData,
                                   uint32_t commandToken,
                                   DualTendonMotorTargets* outTargets);

#endif // DUAL_TENDON_CONTROL_H
