/**
 * @file SystemFaultPolicy.h
 * @brief 集中定义系统故障位图的汇总和按状态裁剪规则。
 *
 * 本 Policy 只返回故障位图，不锁存、不清除、不迁移状态；业务动作仍由
 * SystemStateCoordinator 执行。
 */
#ifndef P4_SAFETY_POLICIES_SYSTEM_FAULT_POLICY_H
#define P4_SAFETY_POLICIES_SYSTEM_FAULT_POLICY_H

#include <stdint.h>

#include "../../system/SystemRuntimeContext.h"

namespace p4::safety {

/** 同一次状态评估的输入与结果；只在调用栈中使用，不跨周期缓存。 */
struct SystemHealthSnapshot {
    uint32_t activeJointMask;
    uint32_t encoderOnlineMask;
    uint32_t encoderFaultMask;
    uint8_t canOnline;
    bool sensorFresh;
    uint32_t faults;
};
SystemHealthSnapshot readSystemHealth(SystemRuntimeContext* runtime, uint32_t nowMs);

/** 汇总当前 CAN、舵机、过载、标定和多圈观测故障。 */
uint32_t collectSystemFaults(SystemRuntimeContext* runtime);

/** 根据当前运行状态裁剪可忽略故障，返回真正阻断运动的位图。 */
uint32_t blockingFaultsForState(SystemRuntimeContext* runtime,
                                uint8_t state,
                                uint32_t faults);

} // namespace p4::safety

#endif // P4_SAFETY_POLICIES_SYSTEM_FAULT_POLICY_H
