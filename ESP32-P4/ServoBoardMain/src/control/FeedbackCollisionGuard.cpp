#include "FeedbackCollisionGuard.h"

#include <math.h>
#include <string.h>

namespace {

// 小于该误差时视为接近到位，不再继承之前的碰撞武装状态。
static const float kCollisionMinErrorDeg = 4.0f;
// 单帧目标跳变超过 1 度时，按一条新的运动指令重建时间窗。
static const float kCollisionTargetResetDeg = 1.0f;
// 200 ms 速度窗只负责确认关节确实正在向目标运动。
static const uint32_t kCollisionVelocityWindowMs = 200;
static const float kCollisionArmVelocityDegPerSec = 8.0f;
// 武装后在 400 ms 内进展不到 0.4 度，才认为可能受阻。
static const uint32_t kCollisionProgressWindowMs = 400;
static const float kCollisionMinProgressDeg = 0.4f;
// HOLD 后只允许“减小原碰撞目标推进量”的新目标解除保护。
// 解除基准必须是触发碰撞时的目标，而不是 HOLD 瞬间的反馈角。
static const float kCollisionTargetRetreatReleaseDeg = 1.0f;

static float directionOf(float value)
{
    return value >= 0.0f ? 1.0f : -1.0f;
}

} // namespace

FeedbackCollisionGuard::FeedbackCollisionGuard()
{
    resetAll();
}

void FeedbackCollisionGuard::clearMonitoring(JointState& state)
{
    state.candidateActive = false;
    state.armed = false;
    clearHistory(state);
}

void FeedbackCollisionGuard::startCandidate(JointState& state,
                                            uint32_t timestampMs,
                                            float feedbackDeg,
                                            float direction)
{
    // 新运动段必须从未武装状态开始，禁止复用旧目标的速度与进展样本。
    clearMonitoring(state);
    state.candidateActive = true;
    state.candidateStartMs = timestampMs;
    state.candidateDirection = direction;
    appendSample(state, timestampMs, feedbackDeg);
}

void FeedbackCollisionGuard::clearHistory(JointState& state)
{
    state.historyWriteIndex = 0;
    state.historyCount = 0;
}

void FeedbackCollisionGuard::appendSample(JointState& state,
                                          uint32_t timestampMs,
                                          float angleDeg)
{
    state.history[state.historyWriteIndex].timestampMs = timestampMs;
    state.history[state.historyWriteIndex].angleDeg = angleDeg;
    state.historyWriteIndex =
        (uint8_t)((state.historyWriteIndex + 1U) % kHistoryCapacity);
    if (state.historyCount < kHistoryCapacity) {
        state.historyCount++;
    }
}

bool FeedbackCollisionGuard::findWindowStart(const JointState& state,
                                             uint32_t nowMs,
                                             uint32_t windowMs,
                                             uint32_t candidateStartMs,
                                             AngleSample* outSample) const
{
    if (!outSample || state.historyCount == 0) {
        return false;
    }

    // 从最新样本向前查找覆盖指定时间窗的起始点，
    // 不假设磁编帧或控制周期严格固定。
    for (uint8_t offset = 1; offset <= state.historyCount; offset++) {
        const uint8_t index = (uint8_t)(
            (state.historyWriteIndex + kHistoryCapacity - offset) %
            kHistoryCapacity);
        const AngleSample& sample = state.history[index];
        if ((int32_t)(sample.timestampMs - candidateStartMs) < 0) {
            break;
        }
        if ((uint32_t)(nowMs - sample.timestampMs) >= windowMs) {
            *outSample = sample;
            return true;
        }
    }
    return false;
}

FeedbackCollisionResult FeedbackCollisionGuard::makeResult(
    const JointState& state) const
{
    FeedbackCollisionResult result;
    memset(&result, 0, sizeof(result));
    result.armed = state.armed;
    result.holdActive = state.holdActive;
    result.holdAngleDeg = state.holdAngleDeg;
    result.holdTargetDeg = state.holdTargetDeg;
    result.holdDirection = state.holdDirection;
    return result;
}

FeedbackCollisionResult FeedbackCollisionGuard::resetJoint(
    uint8_t jointIndex,
    uint8_t releaseReason)
{
    FeedbackCollisionResult result;
    memset(&result, 0, sizeof(result));
    if (jointIndex >= ENCODER_TOTAL_NUM) {
        return result;
    }

    JointState& state = joints_[jointIndex];
    const bool wasArmed = state.armed;
    const bool wasHold = state.holdActive;
    const float holdAngleDeg = state.holdAngleDeg;
    const float holdTargetDeg = state.holdTargetDeg;
    memset(&state, 0, sizeof(state));
    result.armedChanged = wasArmed;
    result.holdReleased = wasHold;
    result.releaseReason = wasHold ? releaseReason : FEEDBACK_COLLISION_RELEASE_NONE;
    result.holdAngleDeg = holdAngleDeg;
    result.holdTargetDeg = holdTargetDeg;
    return result;
}

void FeedbackCollisionGuard::resetAll()
{
    memset(joints_, 0, sizeof(joints_));
}

bool FeedbackCollisionGuard::isArmed(uint8_t jointIndex) const
{
    return jointIndex < ENCODER_TOTAL_NUM && joints_[jointIndex].armed;
}

bool FeedbackCollisionGuard::isHoldActive(uint8_t jointIndex) const
{
    return jointIndex < ENCODER_TOTAL_NUM && joints_[jointIndex].holdActive;
}

float FeedbackCollisionGuard::holdAngleDeg(uint8_t jointIndex) const
{
    return jointIndex < ENCODER_TOTAL_NUM
        ? joints_[jointIndex].holdAngleDeg
        : 0.0f;
}

FeedbackCollisionResult FeedbackCollisionGuard::update(
    uint8_t jointIndex,
    const FeedbackCollisionInput& input)
{
    FeedbackCollisionResult empty;
    memset(&empty, 0, sizeof(empty));
    if (jointIndex >= ENCODER_TOTAL_NUM) {
        return empty;
    }

    if (!input.contextActive) {
        return resetJoint(jointIndex, FEEDBACK_COLLISION_RELEASE_CONTEXT);
    }
    if (!input.feedbackValid || !isfinite(input.targetDeg) ||
        !isfinite(input.feedbackDeg)) {
        return resetJoint(jointIndex,
                          FEEDBACK_COLLISION_RELEASE_INVALID_FEEDBACK);
    }

    JointState& state = joints_[jointIndex];
    if (!state.contextActive) {
        state.contextActive = true;
        state.targetInitialized = true;
        state.lastTargetDeg = input.targetDeg;
        clearMonitoring(state);
    }

    const float targetDeltaDeg = state.targetInitialized
        ? input.targetDeg - state.lastTargetDeg
        : 0.0f;
    const bool targetChanged =
        state.targetInitialized &&
        fabsf(targetDeltaDeg) >= kCollisionTargetResetDeg;
    state.targetInitialized = true;
    state.lastTargetDeg = input.targetDeg;

    FeedbackCollisionResult result = makeResult(state);
    result.targetDeltaDeg = targetDeltaDeg;

    if (state.holdActive) {
        const float targetRetreatDeg = -state.holdDirection *
            (input.targetDeg - state.holdTargetDeg);
        if (targetRetreatDeg >= kCollisionTargetRetreatReleaseDeg) {
            // 用户相对触发碰撞的原目标回撤至少 1 度即可解除 HOLD。
            // 重复原目标或继续向障碍物深入仍保持锁定；解除后必须重新通过
            // 速度窗口，才允许再次进入碰撞监测。
            const bool wasArmed = state.armed;
            state.holdActive = false;
            clearMonitoring(state);
            appendSample(state, input.timestampMs, input.feedbackDeg);
            result = makeResult(state);
            result.armedChanged = wasArmed;
            result.holdReleased = true;
            result.monitorReset = true;
            result.releaseReason = FEEDBACK_COLLISION_RELEASE_TARGET_RETREAT;
            result.monitorResetReason =
                FEEDBACK_COLLISION_MONITOR_RESET_HOLD_RELEASED;
            result.targetDeltaDeg = targetDeltaDeg;
            result.targetRetreatDeg = targetRetreatDeg;
        } else {
            result = makeResult(state);
            result.targetDeltaDeg = targetDeltaDeg;
            result.targetRetreatDeg = targetRetreatDeg;
        }
        return result;
    }

    const float errorDeg = input.targetDeg - input.feedbackDeg;
    if (fabsf(errorDeg) <= kCollisionMinErrorDeg) {
        // 关节进入到位区即结束当前监测段，避免到位后的弹簧回弹被判为碰撞。
        const bool hadMonitoring = state.candidateActive || state.armed;
        const bool wasArmed = state.armed;
        clearMonitoring(state);
        appendSample(state, input.timestampMs, input.feedbackDeg);
        result = makeResult(state);
        result.armedChanged = wasArmed;
        result.monitorReset = hadMonitoring;
        result.monitorResetReason = hadMonitoring
            ? FEEDBACK_COLLISION_MONITOR_RESET_ERROR_DEADBAND
            : FEEDBACK_COLLISION_MONITOR_RESET_NONE;
        result.targetDeltaDeg = targetDeltaDeg;
        return result;
    }

    const float direction = directionOf(errorDeg);
    const bool directionChanged =
        state.candidateActive && state.candidateDirection != direction;
    if (targetChanged || directionChanged || !state.candidateActive) {
        const bool hadMonitoring = state.candidateActive || state.armed;
        const bool wasArmed = state.armed;
        startCandidate(state,
                       input.timestampMs,
                       input.feedbackDeg,
                       direction);
        result = makeResult(state);
        result.armedChanged = wasArmed;
        result.monitorReset = hadMonitoring &&
            (targetChanged || directionChanged);
        result.monitorResetReason = targetChanged
            ? FEEDBACK_COLLISION_MONITOR_RESET_TARGET_CHANGED
            : (directionChanged
                ? FEEDBACK_COLLISION_MONITOR_RESET_DIRECTION_CHANGED
                : FEEDBACK_COLLISION_MONITOR_RESET_NONE);
        result.targetDeltaDeg = targetDeltaDeg;
        return result;
    }

    appendSample(state, input.timestampMs, input.feedbackDeg);

    AngleSample velocityStart;
    const bool velocityWindowReady = findWindowStart(
        state,
        input.timestampMs,
        kCollisionVelocityWindowMs,
        state.candidateStartMs,
        &velocityStart);
    float directionVelocity = 0.0f;
    if (velocityWindowReady) {
        const uint32_t elapsedMs = input.timestampMs - velocityStart.timestampMs;
        if (elapsedMs > 0) {
            directionVelocity = direction *
                (input.feedbackDeg - velocityStart.angleDeg) *
                1000.0f / (float)elapsedMs;
        }
    }

    // 禁止再用“累计移动超过固定角度”旁路武装；
    // 只有最近 200 ms 方向平均速度达标，才证明本运动段曾正常运动。
    if (!state.armed && velocityWindowReady &&
        directionVelocity >= kCollisionArmVelocityDegPerSec) {
        state.armed = true;
        result.armedChanged = true;
    }

    AngleSample progressStart;
    const bool progressWindowReady = findWindowStart(
        state,
        input.timestampMs,
        kCollisionProgressWindowMs,
        state.candidateStartMs,
        &progressStart);
    float directionProgress = 0.0f;
    if (progressWindowReady) {
        directionProgress = direction *
            (input.feedbackDeg - progressStart.angleDeg);
    }

    // 200 ms 速度窗用于确认关节曾正常运动；武装后再用
    // 400 ms 进展窗判断是否停滞。只有 PID 仍在推进，并且执行器实际 ABS
    // 尚未抵达推进方向的 LUT 边界时，方可进入碰撞保持。
    if (state.armed && progressWindowReady &&
        directionProgress < kCollisionMinProgressDeg &&
        input.pidPushActive && !input.workspaceBoundaryReached) {
        state.holdActive = true;
        state.holdAngleDeg = input.feedbackDeg;
        state.holdTargetDeg = input.targetDeg;
        state.holdDirection = direction;
        result.holdEntered = true;
    }

    result.armed = state.armed;
    result.holdActive = state.holdActive;
    result.candidateAgeMs = input.timestampMs - state.candidateStartMs;
    result.targetDeltaDeg = targetDeltaDeg;
    result.directionVelocityDegPerSec = directionVelocity;
    result.directionProgressDeg = directionProgress;
    result.holdAngleDeg = state.holdAngleDeg;
    result.holdTargetDeg = state.holdTargetDeg;
    result.holdDirection = state.holdDirection;
    return result;
}
