#include "ControlSolver.h"

#include <string.h>

// 构造阶段只清空缓存，不依赖 Arduino/FreeRTOS 资源，保证静态对象可安全初始化。
ControlSolver::ControlSolver() : _initialized(false)
{
    memset(_zeroOffsets, 0, sizeof(_zeroOffsets));
    memset(_gearRatios, 0, sizeof(_gearRatios));
    memset(_directions, 0, sizeof(_directions));
    memset(_pids, 0, sizeof(_pids));
}

void ControlSolver::begin()
{
    // 当前版本使用保守默认值：机械零点 2048、传动比 1、方向为正。
    // 这些值后续可替换为持久化配置，而不影响 ControlTask 的调度结构。
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
        {5.5f, 0.0f, 0.7f, 0.0f, 100.0f, 600.0f},
        {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 30719.0f}
    };
    setPIDParams(pidConfigs);
}

void ControlSolver::init(int16_t* zeroOffsets, float* gearRatios, int8_t* directions)
{
    // 参数按 jointIndex 顺序拷贝到内部缓存，避免运行中依赖外部数组生命周期。
    for (int i = 0; i < JOINT_COUNT; i++) {
        _zeroOffsets[i] = zeroOffsets[i];
        _gearRatios[i] = gearRatios[i];
        _directions[i] = directions[i];
    }
    _initialized = true;
}

void ControlSolver::setPIDParams(float pidParams[][PID_PARAMETER_NUM])
{
    // 每个关节拥有两套 PID 状态，互不共享积分和历史误差。
    for (int i = 0; i < JOINT_COUNT; i++) {
        PID_Init(&_pids[i][0], PID_POSITION, pidParams[0]);
        PID_Init(&_pids[i][1], PID_POSITION, pidParams[1]);
    }
}

bool ControlSolver::compute(float* targetDegs,
                            float* magActualDegs,
                            const int32_t* absolutePosition,
                            int32_t* outServoPulses)
{
    if (!targetDegs || !magActualDegs || !absolutePosition || !outServoPulses) {
        return false;
    }

    for (int i = 0; i < JOINT_COUNT; i++) {
        // 外环把角度误差转成舵机侧位置修正量。
        f_PID_Calculate(&_pids[i][0], targetDegs[i], magActualDegs[i]);

        // 内环以当前舵机绝对位置为基准，输出最终舵机目标位置。
        const float loop2Target = _pids[i][0].Output + (float)absolutePosition[i];
        const float loop2Actual = (float)absolutePosition[i];
        f_PID_Calculate(&_pids[i][1], loop2Target, loop2Actual);

        outServoPulses[i] = (int32_t)_pids[i][1].Output + absolutePosition[i];
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
