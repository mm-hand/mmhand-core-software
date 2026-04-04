#ifndef ANGLE_SOLVER_H
#define ANGLE_SOLVER_H

#include <math.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
#include "pid.h"
#ifdef __cplusplus
}
#endif

#include "ServoBusManager.h"
#include "TaskSharedData.h"

// AngleSolver 头文件职责：
// 1) 定义关节与电机映射数据结构；
// 2) 声明双环 PID 解算器接口；
// 3) 暴露求解任务入口与全局映射对象。

// 关节数量与编码器通道数量保持一致（当前为 21）。
#define JOINT_COUNT ENCODER_TOTAL_NUM
// 舵机内部步进与角度换算系数（保留给历史逻辑/扩展使用）。
#define STS_STEPS_PER_DEG 11.3777f

// 关节主舵机映射项：关节索引 -> 总线号 + 舵机ID。
struct JointMapItem {
    uint8_t busIndex;
    uint8_t servoID;
};

// 电机通道映射项：电机通道 -> 总线号 + 舵机ID。
struct MotorMapItem {
    uint8_t busIndex;
    uint8_t servoID;
};

// 角度解算器：
// - 维护每个关节双环 PID 参数与状态；
// - 将目标角与反馈角解算为舵机目标位置。
class AngleSolver {
public:
    // 构造：清零内部缓存并置未初始化状态。
    AngleSolver();

    // 初始化解算器基础参数（零点/传动比/方向）。
    void init(int16_t* zeroOffsets, float* gearRatios, int8_t* directions);
    // 设置双环 PID 参数（pidParams[0] 为外环，pidParams[1] 为内环）。
    void setPIDParams(float pidParams[][PID_PARAMETER_NUM]);

    // 双环解算：
    // 输入目标角、编码器侧角度、舵机绝对位置，输出舵机目标脉冲。
    bool compute(float* targetDegs,
                 float* magActualDegs,
                 const int32_t* absolutePosition,
                 int16_t* outServoPulses);

    // 读取指定关节/环路的 PID 输出，便于上位机调试显示。
    float getPidOutput(uint8_t jointIndex, uint8_t loopIndex) const;
    void resetAll();

private:
    // 关节基础参数缓存。
    int16_t _zeroOffsets[JOINT_COUNT];
    float _gearRatios[JOINT_COUNT];
    int8_t _directions[JOINT_COUNT];
    // 双环 PID：_pids[joint][0]=外环，_pids[joint][1]=内环。
    PID_Info_TypeDef _pids[JOINT_COUNT][2];
    bool _initialized;
};

extern AngleSolver angleSolver;

// 主映射：关节索引（0..20） -> 主舵机总线/ID。
extern JointMapItem jointMap[ENCODER_TOTAL_NUM];
// 副映射：joint16 双舵机拮抗控制使用的副舵机。
extern MotorMapItem joint16SecondaryMotor;
// 电机通道映射：通道（0..21） -> 舵机总线/ID。
extern MotorMapItem motorMap[SERVO_TOTAL_NUM];

// Solver 任务主循环入口（FreeRTOS 任务函数）。
void taskSolver(void* parameter);

#endif
