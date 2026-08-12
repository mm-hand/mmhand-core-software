/**
 * @file HardwareMap.h
 * @brief 声明硬件映射表及关节到执行器查询接口。
 */
#ifndef HARDWARE_MAP_H
#define HARDWARE_MAP_H

#include "../../data/HardwareTypes.h"

// 拇指根部双腱关节在统一 21 关节拓扑中的固定索引。
static constexpr uint8_t kThumbBaseJointIndex = 16;
static_assert(kThumbBaseJointIndex < ENCODER_TOTAL_NUM,
              "thumb base joint index must be within the encoder map");

// HardwareMap 模块职责：
// 1) 集中定义关节/逻辑电机/唯一舵机 ID 的机械身份映射；
// 2) 避免 AppBootstrap 承担硬件拓扑配置职责。
// 映射数据类型来自 data/HardwareTypes.h，HAL 不得依赖 RTOS SystemContext。

bool isMotorMapItemValid(const MotorMapItem& item); ///< 检查唯一舵机 ID 是否在产品范围内。
bool isMotorChannelValid(uint8_t motorChannel); ///< 检查 Mxx 是否存在有效硬件映射。
uint32_t validMotorChannelMask(); ///< 返回全部有效 Mxx 的位图。
int8_t getMotorTendonShortenAbsSign(uint8_t motorChannel); ///< ABS 增大收绳为 +1，反之为 -1。
int32_t motorRawAbsToTendonAbs(uint8_t motorChannel, int32_t rawAbs); ///< 舵机坐标转腱绳收紧坐标。
int32_t motorTendonAbsToRawAbs(uint8_t motorChannel, int32_t tendonAbs); ///< 腱绳坐标转舵机坐标。
int32_t motorRawDeltaFromTendonDelta(uint8_t motorChannel, int32_t tendonDelta); ///< 转换增量方向。
int32_t motorApplyTendonDelta(uint8_t motorChannel, int32_t rawAbs, int32_t tendonDelta); ///< 在 raw 上施加腱绳增量。
uint8_t getJointActuatorCount(uint8_t jointIndex); ///< 返回关节执行器数量，非法索引返回 0。
bool getJointActuator(uint8_t jointIndex, uint8_t actuatorIndex, JointActuatorItem* out); ///< 查询单个执行器。
bool getJointActuatorGroup(uint8_t jointIndex, JointActuatorGroup* out); ///< 复制完整执行器组。
int findPrimaryMotorChannelForJoint(uint8_t jointIndex); ///< 返回主执行器 Mxx，失败返回 -1。
int findMotorChannelByServoId(uint8_t servoID); ///< 按全手唯一舵机 ID 反查逻辑 Mxx。

#endif // HARDWARE_MAP_H
