/**
 * @file RopeInitSafetyPolicy.h
 * @brief 集中定义 Rope Init 的准入、会话存活和反馈有效性规则。
 *
 * Policy 不发送舵机目标、不结束任务，也不写 fault；Controller 根据返回原因执行动作。
 */
#ifndef P4_SAFETY_POLICIES_ROPE_INIT_SAFETY_POLICY_H
#define P4_SAFETY_POLICIES_ROPE_INIT_SAFETY_POLICY_H

#include <stdint.h>
#include "../../runtime/SystemRuntimeContext.h"

namespace p4::safety {

struct RopeInitSessionDecision {
    bool active;      ///< 会话 owner 和状态仍有效。
    uint8_t reason;   ///< ROPE_INIT_REASON_*；NONE 表示继续。
};

uint8_t evaluateRopeInitActiveScope(SystemRuntimeContext* runtime);
RopeInitSessionDecision evaluateRopeInitSession(SystemRuntimeContext* runtime,
                                                uint32_t nowMs,
                                                uint32_t beginMs);
uint8_t readRopeInitFeedback(SystemRuntimeContext* runtime,
                             uint32_t nowMs,
                             MappedAngleData_t* mapped,
                             ServoAngleData_t* servo);
bool ropeInitCalibrationMatchesManual(uint8_t jointIndex);

} // namespace p4::safety

#endif // P4_SAFETY_POLICIES_ROPE_INIT_SAFETY_POLICY_H
