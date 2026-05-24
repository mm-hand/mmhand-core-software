#ifndef CONTROL_SOLVER_H
#define CONTROL_SOLVER_H

#include <Arduino.h>
#include "../shared/TaskSharedData.h"
#include "pid.h"

// ControlSolver 是控制层内部的双环 PID 求解器：
// - 外环：关节目标角度 -> 舵机侧位置增量；
// - 内环：舵机侧目标位置 -> 舵机命令脉冲；
// - 不直接访问队列、状态机或硬件总线，只做纯计算。
class ControlSolver {
public:
    ControlSolver();

    // 使用当前固件默认零点、传动比、方向和 PID 参数初始化求解器。
    void begin();
    // 低层初始化入口：保留给后续从配置/EEPROM 装载参数时复用。
    void init(int16_t* zeroOffsets, float* gearRatios, int8_t* directions);
    // 批量设置两级 PID 参数，pidParams[0] 为角度外环，pidParams[1] 为位置内环。
    void setPIDParams(float pidParams[][PID_PARAMETER_NUM]);
    // 执行一次 21 关节双环计算，输出每个关节主舵机目标位置。
    bool compute(float* targetDegs,
                 float* magActualDegs,
                 const int32_t* absolutePosition,
                 int32_t* outServoPulses);
    // 提供给调试上报使用，读取指定关节/环路的 PID 输出。
    float getPidOutput(uint8_t jointIndex, uint8_t loopIndex) const;
    // 清空 PID 状态，后续状态机需要在模式切换时重置积分项可调用此接口。
    void resetAll();

private:
    int16_t _zeroOffsets[JOINT_COUNT];
    float _gearRatios[JOINT_COUNT];
    int8_t _directions[JOINT_COUNT];
    PID_Info_TypeDef _pids[JOINT_COUNT][2];
    bool _initialized;
};

#endif // CONTROL_SOLVER_H
