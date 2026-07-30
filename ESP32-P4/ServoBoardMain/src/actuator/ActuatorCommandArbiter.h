/**
 * @file ActuatorCommandArbiter.h
 * @brief 统一判断哪个业务生产者当前拥有舵机输出权。
 */
#ifndef P4_ACTUATOR_COMMAND_ARBITER_H
#define P4_ACTUATOR_COMMAND_ARBITER_H

#include "../shared/TaskSharedData.h"
#include <stdint.h>

namespace p4::actuator {

/**
 * 检查 producerOwner 是否等于状态机授予的 owner。
 *
 * 此检查位于发布端；执行端仍会再次核对 owner，以覆盖检查后发生状态切换的
 * 竞态窗口。
 */
bool producerOwnsOutput(const TaskSharedData_t* sharedData,
                        uint8_t producerOwner);

} // namespace p4::actuator

#endif // P4_ACTUATOR_COMMAND_ARBITER_H
