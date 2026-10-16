/** @file JointFeedbackController.cpp
 * @brief 计算关节反馈控制、配对恢复、换向与碰撞保护状态。
 */
#include "JointFeedbackController.h"
#include "../config/topology/HardwareMap.h"
#include "../system/calibration/TendonCalibrationLut.h"
#include "../tasks/motion_control/ServoTargetBuilder.h"
#include "../system/rtos/SharedSafetyState.h"
#include <math.h>

namespace mmhand::control {
namespace {
float clip(float value, float low, float high) { return fmaxf(low, fminf(high, value)); }
int8_t sign(float value) { return value > 0.0001f ? 1 : value < -0.0001f ? -1 : 0; }
bool slopeAt(const JointCalibrationResult& lut, uint8_t axis, float q, float* slope, float* model) {
    float lower, upper;
    if (!interpolateTendonCalibrationMotorAbs(lut, axis, q, model) ||
        !interpolateTendonCalibrationMotorAbs(lut, axis, q - 0.01f, &lower) ||
        !interpolateTendonCalibrationMotorAbs(lut, axis, q + 0.01f, &upper)) return false;
    *slope = (upper - lower) / 0.02f;
    return isfinite(*slope) && fabsf(*slope) > 0.001f;
}
}
void JointFeedbackController::reset(uint8_t joint) { if (joint < ENCODER_TOTAL_NUM) states[joint] = {}; }
void JointFeedbackController::suspend(uint8_t joint, const ServoAngleData_t& servo, uint32_t now) {
    if (joint >= ENCODER_TOTAL_NUM) return;
    auto& state = states[joint];
    state.phase = Tracking; state.candidate = state.direction = 0;
    state.stableSince = state.errorSince = now;
    for (uint8_t axis = 0; axis < getJointActuatorCount(joint) && axis < MAX_JOINT_ACTUATORS; ++axis) {
        JointActuatorItem actuator;
        if (getJointActuator(joint, axis, &actuator)) state.lastCommand[axis] = servo.servoAngles[actuator.motorChannel];
    }
}
void JointFeedbackController::resetAll() { for (auto& state : states) state = {}; }

JointControlResult JointFeedbackController::compute(uint8_t joint, float target, const FeedbackEstimate& fb,
    uint32_t now, float dt, const ServoAngleData_t& servo, const JointCalibrationResult& lut,
    uint32_t motorMask, SystemRuntimeContext* system, const Parameters& p) {
    JointControlResult out;
    if (joint >= ENCODER_TOTAL_NUM || !p.configured || !fb.ready || !isfinite(target) ||
        !isfinite(dt) || dt <= 0 || dt > 0.1f) return out;
    const uint8_t count = getJointActuatorCount(joint);
    if (!count || count > MAX_JOINT_ACTUATORS || count > TENDON_LUT_ACTUATOR_COUNT) return out;
    float slope[MAX_JOINT_ACTUATORS] = {}, model[MAX_JOINT_ACTUATORS] = {};
    float measured[MAX_JOINT_ACTUATORS] = {}, low[MAX_JOINT_ACTUATORS] = {}, high[MAX_JOINT_ACTUATORS] = {};
    for (uint8_t i = 0; i < count; ++i) {
        JointActuatorItem actuator;
        if (!getJointActuator(joint, i, &actuator) || actuator.motorChannel >= SERVO_TOTAL_NUM) return out;
        const auto motor = actuator.motorChannel;
        if (!(motorMask & (1UL << motor)) || !servo.onlineStatus[motor] || lut.tendonLutMotor[i] != motor ||
            !slopeAt(lut, i, fb.fast, &slope[i], &model[i])) return out;
        int32_t lo, hi;
        const bool bounded = count == 1 ? singleTendonCalibrationMotorAbsRange(lut, i, motor,
            p[OutputLutReleaseMarginCounts], p[OutputLutTightMarginCounts], &lo, &hi) :
            tendonCalibrationMotorAbsRangeWithMargins(lut, i, motor,
            p[OutputLutReleaseMarginCounts], p[OutputLutTightMarginCounts], &lo, &hi);
        if (!bounded) return out;
        out.motors[i] = motor;
        measured[i] = servo.servoAngles[motor];
        low[i] = clampServoPosToSessionRange(system, motor, lo);
        high[i] = clampServoPosToSessionRange(system, motor, hi);
        // Auto-calibrated ABS limits remain mandatory and cannot be enlarged by runtime settings.
        const uint32_t bit = 1UL << motor;
        {
            SharedSafetyGuard guard;
            if (!(system->motor_abs_limit_valid_bitmap & bit)) return out;
            low[i] = fmaxf(low[i], system->motor_abs_limit_min[motor]);
            high[i] = fminf(high[i], system->motor_abs_limit_max[motor]);
        }
        if (low[i] > high[i]) return out;
    }
    auto& state = states[joint];
    if (!state.initialized || state.revision != p.revision) {
        state = {};
        state.initialized = true; state.revision = p.revision;
        state.lastTarget = state.stableTarget = target; state.stableSince = now;
        for (uint8_t i = 0; i < count; ++i) state.lastCommand[i] = measured[i];
    }
    const float delta = target - state.lastTarget;
    const int8_t targetDirection = sign(delta);
    if (fabsf(target - state.stableTarget) > p[FeedbackIntegralNoiseDeg]) {
        state.stableTarget = target; state.stableSince = now;
    }
    if (targetDirection) {
        if (!state.direction) state.direction = targetDirection;
        if (targetDirection == state.direction) {
            if (state.phase == Pending) state.phase = Tracking;
            state.candidate = 0;
        } else if (state.candidate != targetDirection) {
            state.candidate = targetDirection;
            state.candidateAnchor = state.lastTarget;
            state.candidateSince = now;
            if (state.phase != Failed) state.phase = Pending;
        }
    }
    state.lastTarget = target;
    if (state.candidate && (target - state.candidateAnchor) * state.candidate >= p[ReversalConfirmDeg] &&
        now - state.candidateSince >= p[ReversalConfirmMs] && int32_t(now - state.cooldownUntil) >= 0) {
        state.direction = state.candidate; state.candidate = 0;
        state.phase = p[ReversalMaxCounts] > 0 ? Takeup : Tracking;
        state.phaseSince = now; state.responseAnchor = fb.fast;
        for (uint8_t i = 0; i < count; ++i) state.motorAnchor[i] = measured[i];
    }
    if (state.phase == Takeup) {
        if (state.direction * (fb.fast - state.responseAnchor) >= p[ReversalExitTravelDeg]) {
            state.phase = Settling; state.phaseSince = now;
        } else if (now - state.phaseSince >= p[ReversalMaxDurationMs]) {
            state.phase = Failed;
            for (uint8_t i = 0; i < count; ++i) state.hold[i] = measured[i];
        }
    }
    if (state.phase == Settling && now - state.phaseSince >= p[ReversalSettleMs]) {
        state.phase = Tracking; state.cooldownUntil = now + uint32_t(p[ReversalCooldownMs]);
    }
    out.count = count;
    if (state.phase == Failed) {
        out.reversalFailed = true;
        for (uint8_t i = 0; i < count; ++i) out.targets[i] = lroundf(state.hold[i]);
        return out;
    }
    const float kp = p[count == 1 ? SingleTendonKp : DualTendonKp];
    const float ki = p[count == 1 ? SingleTendonKiPerS : DualTendonKiPerS];
    const float iLimit = p[count == 1 ? SingleTendonIntegralLimitCounts : DualTendonIntegralLimitCounts];
    const float error = target - fb.fast, slowError = target - fb.slow;
    const float integralError = sign(error) == sign(slowError) ?
        sign(error) * fmaxf(0, fabsf(slowError) - p[FeedbackIntegralNoiseDeg]) : 0;
    if (fb.fresh && state.errorSign != sign(integralError)) {
        state.errorSign = sign(integralError); state.errorSince = now;
    }
    bool following = true;
    for (uint8_t i = 0; i < count; ++i)
        if (fabsf(state.lastCommand[i] - measured[i]) > p[OutputMaxStepCounts]) following = false;
    const bool integrate = fb.fresh && fb.sampleDt > 0 && following && !state.saturated && state.phase == Tracking &&
        now - state.stableSince >= p[FeedbackIntegralConfirmMs] &&
        state.errorSign != 0 && now - state.errorSince >= p[FeedbackIntegralConfirmMs];
    float raw[MAX_JOINT_ACTUATORS] = {}, previousIntegral[MAX_JOINT_ACTUATORS] = {};
    for (uint8_t i = 0; i < count; ++i) {
        previousIntegral[i] = state.integral[i];
        if (integrate) state.integral[i] = clip(state.integral[i] + ki * slope[i] * integralError * fb.sampleDt, -iLimit, iLimit);
        out.proportional[i] = kp * slope[i] * error;
        out.integral[i] = state.integral[i];
        raw[i] = measured[i] + out.proportional[i] + state.integral[i];
        if (state.phase == Takeup) {
            const int8_t motorDirection = sign(slope[i]) * state.direction;
            const float amount = fminf(p[ReversalMaxCounts], p[ReversalInitialCounts] +
                p[ReversalGrowthCountsPerS] * (now - state.phaseSince) * 0.001f);
            const float reference = state.motorAnchor[i] + motorDirection * amount;
            out.reversal[i] = motorDirection * fmaxf(0, motorDirection * (reference - raw[i]));
            raw[i] += out.reversal[i];
        }
    }
    if (count == 2) {
        const float z0 = model[0] - measured[0], z1 = model[1] - measured[1];
        const float along = (slope[0] * z0 + slope[1] * z1) / (slope[0] * slope[0] + slope[1] * slope[1]);
        out.recovery[0] = p[DualTendonPairRecoveryGain] * (z0 - slope[0] * along);
        out.recovery[1] = p[DualTendonPairRecoveryGain] * (z1 - slope[1] * along);
    }
    const float rateStep = fminf(p[OutputMaxStepCounts], p[OutputMaxRateCountsPerS] * dt);
    float allowedLow[MAX_JOINT_ACTUATORS], allowedHigh[MAX_JOINT_ACTUATORS];
    bool recoveryAllowed = count == 2;
    for (uint8_t i = 0; i < count; ++i) {
        allowedLow[i] = fmaxf(low[i], state.lastCommand[i] - rateStep);
        allowedHigh[i] = fminf(high[i], state.lastCommand[i] + rateStep);
        if (allowedLow[i] > allowedHigh[i]) { allowedLow[i] = low[i]; allowedHigh[i] = high[i]; }
        if (raw[i] + out.recovery[i] < allowedLow[i] || raw[i] + out.recovery[i] > allowedHigh[i]) recoveryAllowed = false;
    }
    state.saturated = false;
    for (uint8_t i = 0; i < count; ++i) {
        if (!recoveryAllowed) out.recovery[i] = 0;
        const float candidate = raw[i] + out.recovery[i];
        out.rawTargets[i] = candidate;
        const float applied = clip(candidate, allowedLow[i], allowedHigh[i]);
        if (fabsf(candidate - applied) > 0.5f) {
            state.saturated = true;
            if ((state.integral[i] - previousIntegral[i]) * (candidate - applied) > 0)
                state.integral[i] = previousIntegral[i];
        }
        out.targets[i] = lroundf(applied); state.lastCommand[i] = out.targets[i];
        if ((applied - measured[i]) * slope[i] * error > 0 && fabsf(applied - measured[i]) >= 1) out.pushing = true;
    }
    out.saturated = state.saturated;
    // Rate limiting is normal, but a mechanical boundary cannot be collision evidence.
    for (uint8_t i = 0; i < count; ++i)
        if (raw[i] < low[i] || raw[i] > high[i]) { out.pushing = false; out.boundaryClamped = true; }
    return out;
}
}
