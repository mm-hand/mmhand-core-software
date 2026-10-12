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

/** 汇总当前 CAN、舵机、过载、标定和多圈观测故障。 */
uint32_t collectSystemFaults(SystemRuntimeContext* runtime);

/** 根据当前运行状态裁剪可忽略故障，返回真正阻断运动的位图。 */
uint32_t blockingFaultsForState(SystemRuntimeContext* runtime,
                                uint8_t state,
                                uint32_t faults);

} // namespace p4::safety

#endif // P4_SAFETY_POLICIES_SYSTEM_FAULT_POLICY_H
