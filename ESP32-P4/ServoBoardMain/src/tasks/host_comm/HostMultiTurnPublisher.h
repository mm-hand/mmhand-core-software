/**
 * @file HostMultiTurnPublisher.h
 * @brief 声明多圈异步结果与 Flash Raw 快照的变化触发上报器。
 */
#ifndef P4_MODULES_COMMUNICATION_UPPER_COMM_MULTI_TURN_PUBLISHER_H
#define P4_MODULES_COMMUNICATION_UPPER_COMM_MULTI_TURN_PUBLISHER_H

#include "../../runtime/SystemRuntimeContext.h"

namespace p4::host {

void initHostMultiTurnPublisher();

/**
 * 发送尚未成功提交的多圈状态。
 * @return 非同步请求始终返回 true；同步请求仅在两个状态包均发送成功后返回 true。
 */
bool stepHostMultiTurnPublisher(SystemRuntimeContext* sharedData,
                                     bool syncRequested);

} // namespace p4::host

#endif // P4_MODULES_COMMUNICATION_UPPER_COMM_MULTI_TURN_PUBLISHER_H
