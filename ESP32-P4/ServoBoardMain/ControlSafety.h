#ifndef CONTROL_SAFETY_H
#define CONTROL_SAFETY_H

#include <Arduino.h>
#include "TaskSharedData.h"

// joint16 双舵机拮抗参数。
static const uint8_t kJoint16Index = 16;
static const int32_t kJoint16SecondaryOffset = 0;
static const int32_t kJoint16TensionBias = 0;
static const int32_t kJoint16DiffThresholdCounts = 180;
static const uint8_t kJoint16DiffFaultCycles = 3;

typedef struct
{
    uint8_t initialized;
    uint8_t faultActive;
    uint8_t catastrophicFault;
    uint8_t releaseWindowActive;
    int32_t releaseServoAccumCounts;
    float actualReturnAccumDeg;
    float filteredActualDeg;
    uint8_t forwardAnomalyCycles;
    uint8_t recoverStableCycles;
    float prevTargetDeg;
    float prevActualDeg;
    int32_t prevServoPos;
} ReleaseGuardState;

bool shouldEmergencyStop(bool canBusOnline,
                         const RemoteSensorData_t& sensorData,
                         const MappedAngleData_t& mappedData,
                         uint8_t jointIndex);
void resetAllReleaseGuardStates(ReleaseGuardState guards[ENCODER_TOTAL_NUM]);
bool updateReleaseGuardState(ReleaseGuardState* guard,
                             float targetDeg,
                             float actualDeg,
                             int32_t servoPos,
                             bool inputsValid);
uint32_t buildReleaseFaultBitmap(const ReleaseGuardState guards[ENCODER_TOTAL_NUM]);
bool shouldBlockReleaseByTendonGuard(const ControlCommandSnapshot_t* commandSnapshot,
                                     uint8_t jointIndex,
                                     float targetDeg,
                                     float actualDeg,
                                     int32_t absNow,
                                     bool inputsValid);

#endif // CONTROL_SAFETY_H
