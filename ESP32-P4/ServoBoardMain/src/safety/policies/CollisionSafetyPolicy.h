/**
 * @file CollisionSafetyPolicy.h
 * @brief 声明基于磁编进展窗口的碰撞保持小状态机。
 */
#ifndef P4_COLLISION_SAFETY_POLICY_H
#define P4_COLLISION_SAFETY_POLICY_H

#include <Arduino.h>
#include "../../runtime/SystemRuntimeContext.h"

enum FeedbackCollisionReleaseReason : uint8_t {
    FEEDBACK_COLLISION_RELEASE_NONE = 0,             ///< 尚未退出 HOLD。
    FEEDBACK_COLLISION_RELEASE_CONTEXT = 1,          ///< 控制模式或 active scope 已改变。
    FEEDBACK_COLLISION_RELEASE_INVALID_FEEDBACK = 2, ///< 磁编反馈失效。
    FEEDBACK_COLLISION_RELEASE_TARGET_RETREAT = 3,   ///< 新目标沿碰撞反方向退让。
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
    uint32_t timestampMs; ///< 当前控制周期单调时间。
    bool contextActive;   ///< 当前关节是否处于反馈控制作用域。
    bool feedbackValid;   ///< 本周期磁编角是否可信。
    float targetDeg;      ///< 当前钳位后的关节目标角。
    float feedbackDeg;    ///< 当前磁编实际角。
    bool pidPushActive;   ///< PID 是否仍沿目标方向产生有效推进。
    // 只有执行器实际位置已抵达当前推进方向的 LUT 边界时才置位。
    bool workspaceBoundaryReached;
};

struct FeedbackCollisionResult {
    bool armed;             ///< 运动速度已足够，正在监视低进展。
    bool holdActive;        ///< 已锁存碰撞，调用方应持续保持 ABS 快照。
    bool armedChanged;      ///< 本周期 armed 是否发生变化。
    bool holdEntered;       ///< 本周期首次进入 HOLD。
    bool holdReleased;      ///< 本周期退出 HOLD。
    bool monitorReset;      ///< 目标/方向变化导致监视窗口重建。
    uint8_t releaseReason;  ///< FeedbackCollisionReleaseReason。
    uint8_t monitorResetReason; ///< FeedbackCollisionMonitorResetReason。
    uint32_t candidateAgeMs; ///< 当前运动候选段持续时间。
    float targetDeltaDeg;   ///< 目标相对反馈的带符号误差。
    float directionVelocityDegPerSec; ///< 200 ms 窗口沿目标方向平均速度。
    float directionProgressDeg; ///< 400 ms 窗口沿目标方向角度进展。
    float holdAngleDeg;     ///< 触发 HOLD 时的磁编角。
    float holdTargetDeg;    ///< 触发 HOLD 时的目标角。
    float holdDirection;    ///< 触发时推进方向，仅为 -1 或 +1。
    float targetRetreatDeg; ///< 新目标沿 holdDirection 反向退让量。
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
        uint32_t timestampMs; ///< 样本时间。
        float angleDeg;       ///< 磁编反馈角。
    };

    struct JointState {
        AngleSample history[kHistoryCapacity]; ///< 约 640 ms 的 10 ms 角度环形缓存。
        uint8_t historyWriteIndex; ///< 下一样本写入位置。
        uint8_t historyCount;      ///< 当前有效样本数。
        bool contextActive;        ///< 上周期控制上下文是否有效。
        bool candidateActive;      ///< 是否已有运动候选段。
        bool armed;                ///< 是否进入低进展监视阶段。
        bool holdActive;           ///< 是否锁存碰撞保持。
        bool targetInitialized;    ///< lastTargetDeg 是否有效。
        uint32_t candidateStartMs; ///< 当前候选运动段起点。
        float candidateDirection;  ///< 当前推进方向。
        float lastTargetDeg;       ///< 上周期目标，用于识别目标跳变。
        float holdAngleDeg;        ///< 碰撞时磁编角快照。
        float holdTargetDeg;       ///< 碰撞时目标角快照。
        float holdDirection;       ///< 碰撞时推进方向。
    };

    JointState joints_[ENCODER_TOTAL_NUM];

    // 清除当前运动段的武装与采样状态，不改变 HOLD 快照。
    void clearMonitoring(JointState& state);
    // 从当前磁编角度创建新运动段，后续速度和进展窗都以此为起点。
    void startCandidate(JointState& state,
                        uint32_t timestampMs,
                        float feedbackDeg,
                        float direction);
    // 清空角度窗口，不改变关节控制上下文。
    void clearHistory(JointState& state);
    // 向固定容量环形缓存追加一个磁编样本。
    void appendSample(JointState& state, uint32_t timestampMs, float angleDeg);
    // 查找不早于候选起点、且覆盖指定时间窗的最旧可用样本。
    bool findWindowStart(const JointState& state,
                         uint32_t nowMs,
                         uint32_t windowMs,
                         uint32_t candidateStartMs,
                         AngleSample* outSample) const;
    // 将内部状态转换为不暴露缓存实现的只读结果。
    FeedbackCollisionResult makeResult(const JointState& state) const;
};

#endif // P4_COLLISION_SAFETY_POLICY_H
