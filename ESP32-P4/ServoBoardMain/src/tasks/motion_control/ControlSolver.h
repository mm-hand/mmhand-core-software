/**
 * @file ControlSolver.h
 * @brief 声明单腱关节位置 PID 求解器。
 */
#ifndef CONTROL_SOLVER_H
#define CONTROL_SOLVER_H

#include <Arduino.h>
#include "../../runtime/SystemRuntimeContext.h"
#include "pid.h"

// ControlSolver 是控制层内部的关节角度 PID 求解器：
// - 关节目标角度 -> 舵机侧位置增量；
// - 舵机自身负责根据位置命令到达目标；
// - 不直接访问队列、状态机或硬件总线，只做纯计算。
class ControlSolver {
public:
    ControlSolver();

    // 使用 ControlConfig 中的 PID 参数初始化 21 个独立控制器。
    void begin();
    // 执行一次 21 关节角度 PID 计算，输出每个关节主舵机目标位置。
    bool compute(float* targetDegs,
                 float* magActualDegs,
                 const int32_t* absolutePosition,
                 int32_t* outServoPulses,
                 float dtSeconds);
    // 提供给调试上报使用，读取指定关节的 PID 输出。
    float getPidOutput(uint8_t jointIndex) const;
    // 只清除指定关节的积分、误差历史和输出。
    void resetJoint(uint8_t jointIndex);
    // 清空 PID 状态，后续状态机需要在模式切换时重置积分项可调用此接口。
    void resetAll();

private:
    // 用同一组参数初始化全部关节，但每个关节保有独立动态状态。
    void setPIDParams(const float pidParams[PID_PARAMETER_NUM]);
    // 每个关节一套 PID：将角度误差映射为电机侧位置修正。
    PID_Info_TypeDef _pids[JOINT_COUNT];
};

#endif // CONTROL_SOLVER_H
