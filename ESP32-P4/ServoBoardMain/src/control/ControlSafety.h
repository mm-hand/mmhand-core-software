#ifndef CONTROL_SAFETY_H
#define CONTROL_SAFETY_H

#include <Arduino.h>
#include "../shared/TaskSharedData.h"

// ControlSafety 封装控制层安全判定：
// - CAN/编码器有效性急停；
// - joint16 双舵机反馈一致性参数；
// - 反绕释放保护状态；
// - 上位机配置的 tendon guard 释放阻断。

// joint16 双舵机拮抗参数。
static const uint8_t kJoint16Index = 16;
static const int32_t kJoint16SecondaryOffset = 0;
static const int32_t kJoint16TensionBias = 0;
static const int32_t kJoint16DiffThresholdCounts = 180;
static const uint8_t kJoint16DiffFaultCycles = 3;

typedef struct
{
    // 是否已建立上一周期参考值。
    uint8_t initialized;
    // 普通故障可在满足恢复条件后清除。
    uint8_t faultActive;
    // 灾难性故障需要 START/RESET 令牌重置。
    uint8_t catastrophicFault;
    // 释放窗口表示目标正在向“放松/回退”方向运动。
    uint8_t releaseWindowActive;
    // 释放窗口内舵机累计运动量，用于发现空转/反绕。
    int32_t releaseServoAccumCounts;
    float actualReturnAccumDeg;
    float filteredActualDeg;
    uint8_t forwardAnomalyCycles;
    uint8_t recoverStableCycles;
    float prevTargetDeg;
    float prevActualDeg;
    int32_t prevServoPos;
} ReleaseGuardState;

// 判断当前关节反馈是否足以继续闭环控制。
bool shouldEmergencyStop(bool canBusOnline,
                         const RemoteSensorData_t& sensorData,
                         const MappedAngleData_t& mappedData,
                         uint8_t jointIndex);
// 清空全部反绕保护状态，通常由 START/RESET 令牌触发。
void resetAllReleaseGuardStates(ReleaseGuardState guards[ENCODER_TOTAL_NUM]);
// 更新单关节反绕保护状态，返回当前是否需要保持/阻断该关节。
bool updateReleaseGuardState(ReleaseGuardState* guard,
                             float targetDeg,
                             float actualDeg,
                             int32_t servoPos,
                             bool inputsValid);
// 将每个关节的 faultActive 压缩成上报用 bitmap。
uint32_t buildReleaseFaultBitmap(const ReleaseGuardState guards[ENCODER_TOTAL_NUM]);
// 根据上位机下发的 tendon guard 配置判断是否禁止继续释放。
bool shouldBlockReleaseByTendonGuard(const ControlCommandSnapshot_t* commandSnapshot,
                                     uint8_t jointIndex,
                                     float targetDeg,
                                     float actualDeg,
                                     int32_t absNow,
                                     bool inputsValid);

#endif // CONTROL_SAFETY_H
