#include "ControlSolver.h"

#include <string.h>

ControlSolver::ControlSolver() : _initialized(false)
{
    memset(_zeroOffsets, 0, sizeof(_zeroOffsets));
    memset(_gearRatios, 0, sizeof(_gearRatios));
    memset(_directions, 0, sizeof(_directions));
    memset(_pids, 0, sizeof(_pids));
}

void ControlSolver::begin()
{
    int16_t zeros[JOINT_COUNT];
    float ratios[JOINT_COUNT];
    int8_t dirs[JOINT_COUNT];
    for (int i = 0; i < JOINT_COUNT; i++) {
        zeros[i] = 2048;
        ratios[i] = 1.0f;
        dirs[i] = 1;
    }
    init(zeros, ratios, dirs);

    float pidConfigs[2][PID_PARAMETER_NUM] = {
        {16.5f, 0.0f, 0.5f, 0.0f, 100.0f, 600.0f},
        {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 30719.0f}
    };
    setPIDParams(pidConfigs);
}

void ControlSolver::init(int16_t* zeroOffsets, float* gearRatios, int8_t* directions)
{
    for (int i = 0; i < JOINT_COUNT; i++) {
        _zeroOffsets[i] = zeroOffsets[i];
        _gearRatios[i] = gearRatios[i];
        _directions[i] = directions[i];
    }
    _initialized = true;
}

void ControlSolver::setPIDParams(float pidParams[][PID_PARAMETER_NUM])
{
    for (int i = 0; i < JOINT_COUNT; i++) {
        PID_Init(&_pids[i][0], PID_POSITION, pidParams[0]);
        PID_Init(&_pids[i][1], PID_POSITION, pidParams[1]);
    }
}

bool ControlSolver::compute(float* targetDegs,
                            float* magActualDegs,
                            const int32_t* absolutePosition,
                            int16_t* outServoPulses)
{
    if (!targetDegs || !magActualDegs || !absolutePosition || !outServoPulses) {
        return false;
    }

    for (int i = 0; i < JOINT_COUNT; i++) {
        f_PID_Calculate(&_pids[i][0], targetDegs[i], magActualDegs[i]);

        const float loop2Target = _pids[i][0].Output + (float)absolutePosition[i];
        const float loop2Actual = (float)absolutePosition[i];
        f_PID_Calculate(&_pids[i][1], loop2Target, loop2Actual);

        outServoPulses[i] = (int16_t)_pids[i][1].Output + absolutePosition[i];
    }
    return true;
}

float ControlSolver::getPidOutput(uint8_t jointIndex, uint8_t loopIndex) const
{
    if (jointIndex >= JOINT_COUNT || loopIndex > 1) {
        return 0.0f;
    }
    return _pids[jointIndex][loopIndex].Output;
}

void ControlSolver::resetAll()
{
    memset(_pids, 0, sizeof(_pids));
}
