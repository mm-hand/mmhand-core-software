/**
 * @file EncoderHealthSafetyPolicy.h
 * @brief 定义磁编码器连续好帧、坏帧和反馈超时的去抖规则。
 *
 * Policy 只修改调用者持有的单路健康状态，不写共享位图、不设置 fault。
 */
#ifndef P4_SAFETY_POLICIES_ENCODER_HEALTH_SAFETY_POLICY_H
#define P4_SAFETY_POLICIES_ENCODER_HEALTH_SAFETY_POLICY_H

#include <stdint.h>

namespace p4::safety {

struct EncoderHealthState {
    uint8_t goodCount;       ///< 连续正确快照数，用于确认上线。
    uint8_t badCount;        ///< 连续错误快照数，用于确认故障。
    uint8_t online;          ///< 去抖后的在线状态。
    uint8_t fault;           ///< 去抖后的阻断故障状态。
    uint32_t lastGoodMs;     ///< 最近正确快照的单调时间。
    uint32_t warnUntilMs;    ///< 瞬时错误警告保持到该时刻。
};

void acceptEncoderSample(EncoderHealthState* state, uint32_t nowMs);
void rejectEncoderSample(EncoderHealthState* state,
                         uint32_t nowMs,
                         bool countTowardFault);
bool applyEncoderFeedbackTimeout(EncoderHealthState* state, uint32_t nowMs);
void resetEncoderHealthState(EncoderHealthState* state);

} // namespace p4::safety

#endif // P4_SAFETY_POLICIES_ENCODER_HEALTH_SAFETY_POLICY_H
