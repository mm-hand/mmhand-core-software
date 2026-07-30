/**
 * @file HardwareMap.h
 * @brief 声明硬件映射表及关节到执行器查询接口。
 */
#ifndef HARDWARE_MAP_H
#define HARDWARE_MAP_H

#include "../shared/TaskSharedData.h"

#define HAND_PROFILE_V1 1
#define HAND_PROFILE_V2 2

#ifndef HAND_PROFILE
#define HAND_PROFILE HAND_PROFILE_V2
#endif

// Shared topology index for the thumb base dual-tendon joint (J16).
static constexpr uint8_t kThumbBaseJointIndex = 16;
static_assert(kThumbBaseJointIndex < ENCODER_TOTAL_NUM,
              "thumb base joint index must be within the encoder map");

// HardwareMap 模块职责：
// 1) 集中定义关节/电机到物理总线与 ID 的映射；
// 2) 避免 SystemTask 承担硬件拓扑配置职责。
// 映射变量本身在 TaskSharedData.h 中声明为 extern，本头文件用于让调用方显式依赖硬件拓扑模块。

bool isMotorMapItemValid(const MotorMapItem& item);
bool isMotorChannelValid(uint8_t motorChannel);
uint32_t validMotorChannelMask();
int8_t getMotorTendonShortenAbsSign(uint8_t motorChannel);
int32_t motorRawAbsToTendonAbs(uint8_t motorChannel, int32_t rawAbs);
int32_t motorTendonAbsToRawAbs(uint8_t motorChannel, int32_t tendonAbs);
int32_t motorRawDeltaFromTendonDelta(uint8_t motorChannel, int32_t tendonDelta);
int32_t motorApplyTendonDelta(uint8_t motorChannel, int32_t rawAbs, int32_t tendonDelta);
uint8_t getJointActuatorCount(uint8_t jointIndex);
bool getJointActuator(uint8_t jointIndex, uint8_t actuatorIndex, JointActuatorItem* out);
bool getJointActuatorGroup(uint8_t jointIndex, JointActuatorGroup* out);
int findPrimaryMotorChannelForJoint(uint8_t jointIndex);
int findMotorChannelByBusId(uint8_t busIndex, uint8_t servoID);

#endif // HARDWARE_MAP_H
