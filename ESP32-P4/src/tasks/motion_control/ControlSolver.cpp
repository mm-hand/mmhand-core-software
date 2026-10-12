/**
 * @file ControlSolver.cpp
 * @brief 实现单腱关节的位置 PID 和反馈速度阻尼，输出舵机 ABS 增量。
 */
#include "ControlSolver.h"

#include <string.h>

#include "../../config/ControlConfig.h"

// 构造阶段只清空 PID，不依赖 Arduino/FreeRTOS 资源，保证静态对象可安全初始化。
ControlSolver::ControlSolver()
{
    memset(_pids, 0, sizeof(_pids));
}

void ControlSolver::begin()
{
    using namespace mmhand::config::control;
    const float pidConfigs[PID_PARAMETER_NUM] = {
        kSingleTendonKp,
        kSingleTendonKi,
        kSingleTendonKd,
        kSingleTendonDeadbandDeg,
        kSingleTendonIntegralLimit,
        kSingleTendonVelocityFilterTauSec,
        kSingleTendonDTermLimitCounts
    };
    setPIDParams(pidConfigs);
}

void ControlSolver::setPIDParams(const float pidParams[PID_PARAMETER_NUM])
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
                            const float* kpByJoint,
                            float dtSeconds)
{
    // 本周期所有缓冲区都由 MotionControlTask 持有；求解器只读输入并写
    // outServoPulses，因此自身不做 FreeRTOS 同步。
    if (!targetDegs || !magActualDegs || !absolutePosition || !outServoPulses ||
        !isfinite(dtSeconds) || dtSeconds <= 0.0f) {
        return false;
    }

    for (int i = 0; i < JOINT_COUNT; i++) {
        // 增益调度只改变本周期比例系数，不重置积分和误差历史。
        _pids[i].param.kp = kpByJoint && isfinite(kpByJoint[i]) &&
                kpByJoint[i] > 0.0f
            ? kpByJoint[i]
            : mmhand::config::control::kSingleTendonKp;
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
{
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
