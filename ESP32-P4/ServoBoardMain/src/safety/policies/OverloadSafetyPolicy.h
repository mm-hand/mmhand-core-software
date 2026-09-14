/**
 * @file OverloadSafetyPolicy.h
 * @brief 集中定义控制输出的舵机过载作用域、连续帧判定和关节故障映射。
 *
 * Policy 只计算判定结果，不写故障位图、不打印日志、不发送舵机命令。
 */
#ifndef P4_SAFETY_POLICIES_OVERLOAD_SAFETY_POLICY_H
#define P4_SAFETY_POLICIES_OVERLOAD_SAFETY_POLICY_H

#include <stdint.h>

#include "../../runtime/SystemRuntimeContext.h"

namespace p4::safety {

struct OverloadSampleDecision {
    uint8_t nextHighCount; ///< 本帧处理后的连续超阈值计数。
    bool thresholdExceeded; ///< 本帧负载绝对值是否越过模式阈值。
    bool shouldLatch;       ///< 连续帧达到要求，业务层应锁存故障。
    int16_t threshold;      ///< 本模式实际采用的负载阈值。
    uint8_t latchCycles;    ///< 本模式要求的连续帧数。
};

/** 返回当前控制模式真正主动驱动的舵机集合。 */
uint32_t overloadActiveMotorMask(SystemRuntimeContext* runtime,
                                 const ControlCommandSnapshot_t& command);

/** 纯函数：根据控制模式、负载和旧计数给出本帧过载判定。 */
OverloadSampleDecision evaluateOverloadSample(uint8_t controlMode,
                                              int16_t signedLoad,
                                              uint8_t previousHighCount);

/** 判断指定关节任一执行器是否已存在过载故障。 */
bool jointHasOverloadFault(uint8_t jointIndex, uint32_t overloadFaultBitmap);

} // namespace p4::safety

#endif // P4_SAFETY_POLICIES_OVERLOAD_SAFETY_POLICY_H
