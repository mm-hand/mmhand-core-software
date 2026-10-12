/**
 * @file StateAdmissionPolicy.h
 * @brief 定义不依赖硬件快照的顶层状态迁移准入规则。
 */
#ifndef P4_SYSTEM_STATE_POLICY_H
#define P4_SYSTEM_STATE_POLICY_H

#include <stdint.h>

namespace p4::system {

bool canAcceptStart(uint8_t state);
bool canAcceptRunDirect(uint8_t state);
bool canAcceptRunFeedback(uint8_t state);

} // namespace p4::system

#endif // P4_SYSTEM_STATE_POLICY_H
