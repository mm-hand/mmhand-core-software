/**
 * @file EncoderHealthSafetyPolicy.cpp
 * @brief 实现磁编码器健康去抖和超时升级规则。
 */
#include "EncoderHealthSafetyPolicy.h"

#include <string.h>

#include "../../config/SafetyConfig.h"

namespace p4::safety {
namespace {
constexpr uint8_t kOnlineGoodFrames = 3; ///< 连续三帧正确后才确认上线。
constexpr uint8_t kFaultBadFrames = 5;   ///< 连续五帧错误后才升级为故障。
constexpr uint32_t kWarnHoldMs = 800;    ///< 瞬时错误的告警保持时间。
using mmhand::config::safety::kEncoderFaultTimeoutMs;
} // namespace

void acceptEncoderSample(EncoderHealthState* state, uint32_t nowMs)
{
    if (!state) return;
    if (state->goodCount < kOnlineGoodFrames) ++state->goodCount;
    state->badCount = 0;
    state->lastGoodMs = nowMs;
    if (state->goodCount >= kOnlineGoodFrames) {
        state->online = 1;
        state->fault = 0;
    }
}

void rejectEncoderSample(EncoderHealthState* state,
                         uint32_t nowMs,
                         bool countTowardFault)
{
    if (!state) return;
    state->goodCount = 0;
    state->warnUntilMs = nowMs + kWarnHoldMs;
    if (countTowardFault && state->badCount < kFaultBadFrames) {
        ++state->badCount;
    }
    if (state->badCount >= kFaultBadFrames) {
        state->online = 0;
        state->fault = 1;
    }
}

bool applyEncoderFeedbackTimeout(EncoderHealthState* state, uint32_t nowMs)
{
    if (!state || state->lastGoodMs == 0 ||
        static_cast<uint32_t>(nowMs - state->lastGoodMs) <=
            kEncoderFaultTimeoutMs) {
        return false;
    }
    if (!state->online && state->fault) return false;

    state->online = 0;
    state->fault = 1;
    state->goodCount = 0;
    state->badCount = kFaultBadFrames;
    state->warnUntilMs = nowMs + kWarnHoldMs;
    return true;
}

void resetEncoderHealthState(EncoderHealthState* state)
{
    if (state) memset(state, 0, sizeof(*state));
}

} // namespace p4::safety
