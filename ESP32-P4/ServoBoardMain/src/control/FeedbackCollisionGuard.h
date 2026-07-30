/**
 * @file FeedbackCollisionGuard.h
 * @brief 声明基于磁编进展窗口的碰撞保持小状态机。
 */
#ifndef FEEDBACK_COLLISION_GUARD_H
#define FEEDBACK_COLLISION_GUARD_H

#include <Arduino.h>
#include "../shared/TaskSharedData.h"

enum FeedbackCollisionReleaseReason : uint8_t {
    FEEDBACK_COLLISION_RELEASE_NONE = 0,
    FEEDBACK_COLLISION_RELEASE_CONTEXT = 1,
    FEEDBACK_COLLISION_RELEASE_INVALID_FEEDBACK = 2,
    FEEDBACK_COLLISION_RELEASE_TARGET_RETREAT = 3,
};

// 记录碰撞监测运动段被重新创建的原因，便于实机日志定位误判。
enum FeedbackCollisionMonitorResetReason : uint8_t {
    FEEDBACK_COLLISION_MONITOR_RESET_NONE = 0,
    FEEDBACK_COLLISION_MONITOR_RESET_TARGET_CHANGED = 1,
    FEEDBACK_COLLISION_MONITOR_RESET_DIRECTION_CHANGED = 2,
    FEEDBACK_COLLISION_MONITOR_RESET_ERROR_DEADBAND = 3,
    FEEDBACK_COLLISION_MONITOR_RESET_HOLD_RELEASED = 4,
};

struct FeedbackCollisionInput {
    uint32_t timestampMs;
    bool contextActive;
    bool feedbackValid;
    float targetDeg;
    float feedbackDeg;
    bool pidPushActive;
    // 只有执行器实际位置已抵达当前推进方向的 LUT 边界时才置位。
    bool workspaceBoundaryReached;
};

struct FeedbackCollisionResult {
    bool armed;
    bool holdActive;
    bool armedChanged;
    bool holdEntered;
    bool holdReleased;
    bool monitorReset;
    uint8_t releaseReason;
    uint8_t monitorResetReason;
    uint32_t candidateAgeMs;
    float targetDeltaDeg;
    float directionVelocityDegPerSec;
    float directionProgressDeg;
    float holdAngleDeg;
    float holdTargetDeg;
    float holdDirection;
    float targetRetreatDeg;
};

class FeedbackCollisionGuard {
public:
    FeedbackCollisionGuard();

    FeedbackCollisionResult update(uint8_t jointIndex,
                                   const FeedbackCollisionInput& input);
    FeedbackCollisionResult resetJoint(
        uint8_t jointIndex,
        uint8_t releaseReason = FEEDBACK_COLLISION_RELEASE_CONTEXT);
    void resetAll();

    bool isArmed(uint8_t jointIndex) const;
    bool isHoldActive(uint8_t jointIndex) const;
    float holdAngleDeg(uint8_t jointIndex) const;

private:
    static const uint8_t kHistoryCapacity = 64;

    struct AngleSample {
        uint32_t timestampMs;
        float angleDeg;
    };

    struct JointState {
        AngleSample history[kHistoryCapacity];
        uint8_t historyWriteIndex;
        uint8_t historyCount;
        bool contextActive;
        bool candidateActive;
        bool armed;
        bool holdActive;
        bool targetInitialized;
        uint32_t candidateStartMs;
        float candidateDirection;
        float lastTargetDeg;
        float holdAngleDeg;
        float holdTargetDeg;
        float holdDirection;
    };

    JointState joints_[ENCODER_TOTAL_NUM];

    // 清除当前运动段的武装与采样状态，不改变 HOLD 快照。
    void clearMonitoring(JointState& state);
    // 从当前磁编角度创建新运动段，后续速度和进展窗都以此为起点。
    void startCandidate(JointState& state,
                        uint32_t timestampMs,
                        float feedbackDeg,
                        float direction);
    void clearHistory(JointState& state);
    void appendSample(JointState& state, uint32_t timestampMs, float angleDeg);
    bool findWindowStart(const JointState& state,
                         uint32_t nowMs,
                         uint32_t windowMs,
                         uint32_t candidateStartMs,
                         AngleSample* outSample) const;
    FeedbackCollisionResult makeResult(const JointState& state) const;
};

#endif // FEEDBACK_COLLISION_GUARD_H
