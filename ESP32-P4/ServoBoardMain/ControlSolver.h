#ifndef CONTROL_SOLVER_H
#define CONTROL_SOLVER_H

#include <Arduino.h>
#include "TaskSharedData.h"
#include "pid.h"

class ControlSolver {
public:
    ControlSolver();

    void begin();
    void init(int16_t* zeroOffsets, float* gearRatios, int8_t* directions);
    void setPIDParams(float pidParams[][PID_PARAMETER_NUM]);
    bool compute(float* targetDegs,
                 float* magActualDegs,
                 const int32_t* absolutePosition,
                 int16_t* outServoPulses);
    float getPidOutput(uint8_t jointIndex, uint8_t loopIndex) const;
    void resetAll();

private:
    int16_t _zeroOffsets[JOINT_COUNT];
    float _gearRatios[JOINT_COUNT];
    int8_t _directions[JOINT_COUNT];
    PID_Info_TypeDef _pids[JOINT_COUNT][2];
    bool _initialized;
};

#endif // CONTROL_SOLVER_H
