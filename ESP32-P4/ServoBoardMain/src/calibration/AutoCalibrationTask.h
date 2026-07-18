#ifndef AUTO_CALIBRATION_TASK_H
#define AUTO_CALIBRATION_TASK_H

#include <Arduino.h>
#include "../shared/TaskSharedData.h"

struct AutoCalibrationParams {
    int32_t tightenSpeedCountsPerSec;
    uint16_t servoSpeed;
    uint8_t servoAcc;
    uint32_t loopMs;
    uint32_t windowMs;
    uint32_t traceMs;
    int32_t windowServoDeltaCounts;
    int32_t zeroReachAbsTolerance;
    float stallActualDeg;
    int16_t startLoadDelta;
    int16_t tightLoadHigh;
    uint8_t startLoadStableSamples;
};

extern const uint8_t kDefaultAutoCalibrationJoints[];
extern const uint8_t kDefaultAutoCalibrationJointsCount;

AutoCalibrationParams defaultAutoCalibrationParams(void);

bool runAutoCalibrationJoint(TaskSharedData_t* sharedData,
                             uint8_t jointIndex,
                             const AutoCalibrationParams& params);

bool runAutoCalibrationSequence(TaskSharedData_t* sharedData,
                                const uint8_t* jointList,
                                uint8_t jointCount,
                                const AutoCalibrationParams& params);

bool startAutoCalibrationTask(TaskSharedData_t* sharedData);
void autoCalibrationTask(void* parameter);

#endif // AUTO_CALIBRATION_TASK_H
