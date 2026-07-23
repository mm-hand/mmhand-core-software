#ifndef DUAL_TENDON_CONTROL_H
#define DUAL_TENDON_CONTROL_H

#include <Arduino.h>
#include "../shared/TaskSharedData.h"

// 单个拮抗双腱关节每个控制周期的命令结果。控制器始终填充两个电机槽位：
// 参考/模型位置使用 raw ABS 计数，速度和 PID 项用于诊断，单位为计数或计数/秒。
struct DualTendonMotorTargets {
    // 该关节使用的物理电机通道，顺序与标定 LUT 一致。
    uint8_t motorChannel[2];
    // 经过模型 PID 修正和限幅后的最终绝对电机命令。
    int32_t targetAbs[2];
    // 尚未经过 LUT 工作空间钳位的 PID 原始目标，用于诊断和判断实际是否抵达边界。
    int32_t rawTargetAbs[2];
    // 由标定 LUT 预测的目标和反馈电机位置。
    float motorRef[2];
    float motorFbModel[2];
    // 单电机稳定器使用的位置误差和速度项。
    float motorErr[2];
    float refVelocity[2];
    float fbVelocity[2];
    float velocityError[2];
    float filteredVelocityError[2];
    float pTerm[2];
    float iTerm[2];
    float dTerm[2];
    float pidCorrection[2];
    // PID 原始目标是否被自动标定 LUT 边界截断。该标志只用于诊断，
    // 不能直接作为碰撞检测的否决条件。
    uint8_t lutTargetClamped[2];
    int32_t lutMinAbs[2];
    int32_t lutMaxAbs[2];
    // 上述数组中有效的电机条目数量。
    uint8_t count;
};

// 只有关节已配置且硬件映射为异号双执行器时才返回 true。
bool isDualTendonJoint(uint8_t jointIndex);
// 清空指定关节的 PID 历史；输出关闭或反馈无效时调用。
void resetDualTendonController(uint8_t jointIndex);

bool computeDualTendonMotorTargets(uint8_t jointIndex,
                                   float qRefDeg,
                                   float qFbDeg,
                                   const ServoAngleData_t* servoData,
                                   DualTendonMotorTargets* outTargets);

#endif // DUAL_TENDON_CONTROL_H
