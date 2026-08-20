/**
 * @file DualTendonControl.h
 * @brief 声明双腱 LUT 位置环控制器及运行目标结构。
 */
#ifndef DUAL_TENDON_CONTROL_H
#define DUAL_TENDON_CONTROL_H

#include <Arduino.h>
#include "../../runtime/SystemRuntimeContext.h"

// 单个拮抗双腱关节每个控制周期的命令结果。控制器始终填充两个电机槽位：
// 参考/模型位置使用 raw ABS 计数，速度和 PID 项用于诊断，单位为计数或计数/秒。
struct DualTendonMotorTargets {
    // 该关节使用的物理电机通道，顺序与标定 LUT 一致。
    uint8_t motorChannel[2];
    // 经过双腱 LUT 扩展边界钳位后的最终绝对电机命令。
    int32_t targetAbs[2];
    // PID 原始目标，用于区分模型输出与实际应用的边界目标。
    int32_t rawTargetAbs[2];
    // 由标定 LUT 预测的目标和反馈电机位置。
    float motorRef[2];     ///< qRef 经 LUT 插值得到的模型舵机 ABS。
    float motorFbModel[2]; ///< qFb 经同一 LUT 插值得到的模型舵机 ABS。
    // 单电机稳定器使用的位置误差和速度项。
    float motorErr[2];     ///< motorRef-motorFbModel，单位 counts。
    float refVelocity[2];  ///< 模型目标变化率，单位 counts/s。
    float fbVelocity[2];   ///< 模型反馈变化率，单位 counts/s。
    float velocityError[2]; ///< refVelocity-fbVelocity。
    float filteredVelocityError[2]; ///< 一阶低通后的速度误差。
    float pTerm[2];        ///< Kp*motorErr。
    float iTerm[2];        ///< Ki*误差时间积分。
    float dTerm[2];        ///< Kd*filteredVelocityError。
    float pidCorrection[2]; ///< 最终叠加到实际舵机 ABS 的 PID 修正量。
    // 每台电机独立使用自己的 LUT 最小/最大值并向两端扩展 500 counts。
    uint8_t lutTargetClamped[2]; ///< 原始 PID 目标是否被双腱 LUT 边界截断。
    int32_t lutMinAbs[2];        ///< 加入裕量后的运行下边界。
    int32_t lutMaxAbs[2];        ///< 加入裕量后的运行上边界。
    // 上述数组中有效的电机条目数量。
    uint8_t count;
};

// 只有关节已配置且硬件映射为异号双执行器时才返回 true。
bool isDualTendonJoint(uint8_t jointIndex);
// 清空指定关节的 PID 历史；输出关闭或反馈无效时调用。
void resetDualTendonController(uint8_t jointIndex);

/** 用 qRef/qFb 经同一标定 LUT 生成两台舵机的增量 PID 目标。 */
bool computeDualTendonMotorTargets(uint8_t jointIndex,
                                   float qRefDeg,
                                   float qFbDeg,
                                   float dtSeconds,
                                   const ServoAngleData_t* servoData,
                                   DualTendonMotorTargets* outTargets);

#endif // DUAL_TENDON_CONTROL_H
