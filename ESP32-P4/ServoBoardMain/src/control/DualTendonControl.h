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
    float refVelocity[2];
    float fbVelocity[2];
    float velocityError[2];
    float filteredVelocityError[2];
    float pTerm[2];
    float iTerm[2];
    float dTerm[2];
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
                                   DualTendonMotorTargets* outTargets);

#endif // DUAL_TENDON_CONTROL_H
