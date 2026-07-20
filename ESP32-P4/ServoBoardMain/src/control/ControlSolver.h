#ifndef CONTROL_SOLVER_H
#define CONTROL_SOLVER_H

#include <Arduino.h>
#include "../shared/TaskSharedData.h"
#include "pid.h"

// ControlSolver 是控制层内部的关节角度 PID 求解器：
// - 关节目标角度 -> 舵机侧位置增量；
// - 舵机自身负责根据位置命令到达目标；
// - 不直接访问队列、状态机或硬件总线，只做纯计算。
class ControlSolver {
public:
    ControlSolver();

    // 使用当前固件默认零点、传动比、方向和 PID 参数初始化求解器。
    void begin();
    // 低层初始化入口：保留给后续从配置/EEPROM 装载参数时复用。
    void init(int16_t* zeroOffsets, float* gearRatios, int8_t* directions);
    // 批量设置关节角度 PID 参数。
    void setPIDParams(float pidParams[PID_PARAMETER_NUM]);
    // 执行一次 21 关节角度 PID 计算，输出每个关节主舵机目标位置。
    bool compute(float* targetDegs,
                 float* magActualDegs,
                 const int32_t* absolutePosition,
                 int32_t* outServoPulses,
                 float dtSeconds);
    // 提供给调试上报使用，读取指定关节的 PID 输出。
    float getPidOutput(uint8_t jointIndex) const;
    // 清空 PID 状态，后续状态机需要在模式切换时重置积分项可调用此接口。
    void resetAll();

private:
    // 每个关节的机械变换参数。当前由固件默认值初始化，
    // 保留这些字段是为了兼容后续标定/配置数据。
    int16_t _zeroOffsets[JOINT_COUNT];
    float _gearRatios[JOINT_COUNT];
    int8_t _directions[JOINT_COUNT];
    // 每个关节一套 PID：将角度误差映射为电机侧位置修正。
    PID_Info_TypeDef _pids[JOINT_COUNT];
    // init() 拷贝变换数组后置位，compute() 预期在该状态后调用。
    bool _initialized;
};

#endif // CONTROL_SOLVER_H
