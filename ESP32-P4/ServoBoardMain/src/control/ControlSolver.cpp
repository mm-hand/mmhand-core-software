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

    // 2026-07-19: 弹簧加载的单电机关节在正向紧绳负反馈时实测存在约 1.5 deg 静差。
    // 当前先保持 Ki/Kd 为 0，待阶跃数据确认后再按标准离散 PID 参数调节。
    float pidConfigs[PID_PARAMETER_NUM] = {
        5.7f, 0.0f, 0.0f, 0.0f, 100.0f, 600.0f
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

void ControlSolver::setPIDParams(float pidParams[PID_PARAMETER_NUM])
{
    // 每个关节拥有独立 PID 状态，互不共享积分和历史误差。
    for (int i = 0; i < JOINT_COUNT; i++) {
        PID_Init(&_pids[i], pidParams);
    }
}

bool ControlSolver::compute(float* targetDegs,
                            float* magActualDegs,
                            const int32_t* absolutePosition,
                            int32_t* outServoPulses,
                            float dtSeconds)
{
    // 本周期所有缓冲区都由 ControlTask 持有；求解器只读输入并写
    // outServoPulses，因此自身不做 FreeRTOS 同步。
    if (!targetDegs || !magActualDegs || !absolutePosition || !outServoPulses ||
        !isfinite(dtSeconds) || dtSeconds <= 0.0f) {
        return false;
    }

    for (int i = 0; i < JOINT_COUNT; i++) {
        // 把角度误差转成舵机侧位置修正量，舵机自身完成位置闭环。
        f_PID_Calculate(&_pids[i], targetDegs[i], magActualDegs[i], dtSeconds);
        outServoPulses[i] = absolutePosition[i] + (int32_t)_pids[i].Output;
    }
    return true;
}

float ControlSolver::getPidOutput(uint8_t jointIndex) const
{
    // UI 选择变化时调试端可能请求非法索引；返回无害的 0，
    // 避免暴露原始数组访问。
    if (jointIndex >= JOINT_COUNT) {
        return 0.0f;
    }
    return _pids[jointIndex].Output;
}

void ControlSolver::resetJoint(uint8_t jointIndex)
{       //重置特定关节角
    if (jointIndex >= JOINT_COUNT) {
        return;
    }
    PID_Calc_Clear(&_pids[jointIndex]);
}

void ControlSolver::resetAll()
{
    // 只重置动态 PID 状态；已配置的增益和机械变换保持不变，
    // 这样下一个控制周期可以立即重新启动。
    for (int i = 0; i < JOINT_COUNT; i++) {
        PID_Calc_Clear(&_pids[i]);
    }
}
