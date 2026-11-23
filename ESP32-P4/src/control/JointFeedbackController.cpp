/** @file JointFeedbackController.cpp
 * @brief 计算关节反馈控制、拉绳、配对恢复与碰撞推进状态。
 */
#include "JointFeedbackController.h"
#include "../config/AdaptiveControlConfig.h"
#include "../config/topology/HardwareMap.h"
#include "../config/ControlConfig.h"
#include "../system/calibration/TendonCalibrationLut.h"
#include "../system/rtos/SharedSafetyState.h"
#ifdef MMHAND_ADAPTIVE_KP_LOG_JOINT
#include "../system/log/FirmwareLog.h"
#endif
#include <math.h>

namespace mmhand::control {
using namespace mmhand::config::control;
namespace {
float clip(float value, float low, float high) { return fmaxf(low, fminf(high, value)); }
int8_t sign(float value) { return value > 0.0001f ? 1 : value < -0.0001f ? -1 : 0; }
Parameter jointKpParameter(uint8_t joint) {
    // J00..J19: four joints per finger in AA/MCP/PIP/DIP order; J20: thumb CMC.
    if (joint + 1 == ENCODER_TOTAL_NUM) return JointKpCmc;
    switch (joint % 4) {
        case 0: return JointKpAa;
        case 1: return JointKpMcp;
        case 2: return JointKpPip;
        default: return JointKpDip;
    }
}
Parameter jointKiParameter(uint8_t joint) {
    if (joint + 1 == ENCODER_TOTAL_NUM) return JointKiCmc;
    switch (joint % 4) {
        case 0: return JointKiAa;
        case 1: return JointKiMcp;
        case 2: return JointKiPip;
        default: return JointKiDip;
    }
}
Parameter jointKdParameter(uint8_t joint) {
    if (joint + 1 == ENCODER_TOTAL_NUM) return JointKdCmc;
    switch (joint % 4) {
        case 0: return JointKdAa;
        case 1: return JointKdMcp;
        case 2: return JointKdPip;
        default: return JointKdDip;
    }
}
Parameter typeParameter(uint8_t joint, Parameter aa, Parameter mcp, Parameter pip,
                        Parameter dip, Parameter cmc) {
    if (joint + 1 == ENCODER_TOTAL_NUM) return cmc;
    switch (joint % 4) {
        case 0: return aa;
        case 1: return mcp;
        case 2: return pip;
        default: return dip;
    }
}
Parameter jointIntegralLimitParameter(uint8_t joint) {
    return typeParameter(joint, JointKiIntegralLimitCountsAa, JointKiIntegralLimitCountsMcp,
                         JointKiIntegralLimitCountsPip, JointKiIntegralLimitCountsDip,
                         JointKiIntegralLimitCountsCmc);
}
Parameter jointConfirmDegParameter(uint8_t joint) {
    return typeParameter(joint, ReversalConfirmDegAa, ReversalConfirmDegMcp,
                         ReversalConfirmDegPip, ReversalConfirmDegDip,
                         ReversalConfirmDegCmc);
}
Parameter jointTakeupMaxParameter(uint8_t joint) {
    return typeParameter(joint, ReversalTakeupMaxAa, ReversalTakeupMaxMcp,
                         ReversalTakeupMaxPip, ReversalTakeupMaxDip,
                         ReversalTakeupMaxCmc);
}
Parameter jointExitTravelParameter(uint8_t joint) {
    return typeParameter(joint, ReversalExitTravelDegAa, ReversalExitTravelDegMcp,
                         ReversalExitTravelDegPip, ReversalExitTravelDegDip,
                         ReversalExitTravelDegCmc);
}
RampParameter rampTypeParameter(uint8_t joint, RampParameter aa, RampParameter mcp,
                                RampParameter pip, RampParameter dip, RampParameter cmc) {
    if (joint + 1 == ENCODER_TOTAL_NUM) return cmc;
    switch (joint % 4) {
        case 0: return aa;
        case 1: return mcp;
        case 2: return pip;
        default: return dip;
    }
}
RampParameter jointB0Parameter(uint8_t joint) {
    return rampTypeParameter(joint, ReversalRampB0Aa, ReversalRampB0Mcp,
                             ReversalRampB0Pip, ReversalRampB0Dip, ReversalRampB0Cmc);
}
RampParameter jointVbParameter(uint8_t joint) {
    return rampTypeParameter(joint, ReversalRampVbAa, ReversalRampVbMcp,
                             ReversalRampVbPip, ReversalRampVbDip, ReversalRampVbCmc);
}
}
JointFeedbackController::JointFeedbackController() {
    for (auto& scale : kpScale) scale = 1.0f;
}
float JointFeedbackController::kpScaleOf(uint8_t joint) const {
    return joint < ENCODER_TOTAL_NUM ? kpScale[joint] : 1.0f;
}
float JointFeedbackController::effectiveKpOf(uint8_t joint, const Parameters& parameters) const {
    if (joint >= ENCODER_TOTAL_NUM) return 0.0f;
    const float base = parameters[jointKpParameter(joint)];
    return parameters[AdaptiveKpEnabled] > 0.5f ? base * kpScale[joint] : base;
}
void JointFeedbackController::reset(uint8_t joint) { if (joint < ENCODER_TOTAL_NUM) states[joint] = {}; }
void JointFeedbackController::suspend(uint8_t joint, uint32_t now) {
    if (joint >= ENCODER_TOTAL_NUM) return;
    auto& state = states[joint];
    state.phase = Tracking; state.candidate = state.direction = 0;
    state.errorSince = now;
    // A collision hold must not keep a stale velocity for the next tracking segment.
    state.lastFastValid = false;
    state.lastFast = 0;
    state.lastFastMs = 0;
    state.derivativeFiltered = 0;
    // Cancels any half-finished adaptive event; adaptation resumes on fresh data only.
    state.suspended = true;
    state.adaptive = {};
    state.targetVelocity = 0;
}
void JointFeedbackController::resetAll() {
    // STOP/revision only clears runtime state; learned Kp scales survive until reboot.
    for (auto& state : states) state = {};
}

JointControlResult JointFeedbackController::compute(uint8_t joint, float target, const FeedbackEstimate& fb,
    uint32_t now, const ServoAngleData_t& servo,
    const JointCalibrationResult& lut, uint32_t motorMask, const Parameters& p,
    const RampParameters& ramp) {
    JointControlResult out;
    if (joint >= ENCODER_TOTAL_NUM) return out;
    if (!p.configured || !fb.ready || !isfinite(target) || !isfinite(fb.fast)) {
        states[joint].adaptive = {};
        return out;
    }
    const uint8_t count = getJointActuatorCount(joint);
    if (!count || count > MAX_JOINT_ACTUATORS || count > TENDON_LUT_ACTUATOR_COUNT) return out;
    float slope[MAX_JOINT_ACTUATORS] = {}, model[MAX_JOINT_ACTUATORS] = {};
    float measured[MAX_JOINT_ACTUATORS] = {}, low[MAX_JOINT_ACTUATORS] = {}, high[MAX_JOINT_ACTUATORS] = {};
    int32_t releaseMargin, tightMargin;
    lutMarginsForJoint(joint, p, &releaseMargin, &tightMargin);
    for (uint8_t i = 0; i < count; ++i) {
        JointActuatorItem actuator;
        if (!getJointActuator(joint, i, &actuator) || actuator.motorChannel >= SERVO_TOTAL_NUM) return out;
        const auto motor = actuator.motorChannel;
        // Local LUT interpolation feeds the model/pair recovery; the PID gains always
        // use the constant middle-table slope so the loop gain cannot explode at the
        // distorted tendon ends.
        if (!(motorMask & (1UL << motor)) || !servo.onlineStatus[motor] || lut.tendonLutMotor[i] != motor ||
            !interpolateTendonCalibrationMotorAbs(lut, i, fb.fast, &model[i]) ||
            !tendonCalibrationMiddleSlope(lut, i, &slope[i])) return out;
        // The LUT working window (calibrated span widened by the configurable
        // release/tight margins) is the only command boundary; nothing widens it.
        int32_t lo, hi;
        const bool bounded = tendonCalibrationMotorAbsRangeWithMargins(lut, i, motor,
            releaseMargin, tightMargin, &lo, &hi);
        if (!bounded) return out;
        out.motors[i] = motor;
        measured[i] = servo.servoAngles[motor];
        low[i] = (float)lo;
        high[i] = (float)hi;
        if (low[i] > high[i]) return out;
    }
    auto& state = states[joint];
    if (!state.initialized || state.revision != p.revision) {
        state = {};
        state.initialized = true; state.revision = p.revision;
        state.lastTarget = target;
        state.lastTargetMs = now;
    }
    const bool suspendedThisCycle = state.suspended;
    if (suspendedThisCycle) {
        // A collision/release gap must not look like a huge target move.
        state.lastTarget = target;
        state.lastTargetMs = now;
        state.targetVelocity = 0;
        state.adaptive = {};
        state.suspended = false;
    }
    const uint32_t targetDtMs = now - state.lastTargetMs;
    float rawTargetVelocity = 0.0f;
    if (targetDtMs > 0 && targetDtMs <= 100) {
        rawTargetVelocity = (target - state.lastTarget) * 1000.0f / (float)targetDtMs;
    } else {
        state.targetVelocity = 0;
    }
    const float velocityAlpha = 1.0f - expf(-(float)(targetDtMs > 100 ? 100 : targetDtMs) * 0.001f / 0.1f);
    state.targetVelocity += velocityAlpha * (rawTargetVelocity - state.targetVelocity);
    state.lastTargetMs = now;
    // Differentiate tracking error at fresh feedback samples. Limit and filter the
    // resulting rate, including target steps, before applying the D gain.
    float derivativeRate = 0.0f;
    const bool derivativeFresh = fb.fresh && fb.sampleDt > 0.0f;
    if (derivativeFresh) {
        if (state.lastFastValid) {
            const uint32_t elapsed = now - state.lastFastMs;
            if (elapsed >= 1 && elapsed <= 100) {
                float rate = ((target - state.lastDerivativeTarget) -
                              (fb.fast - state.lastFast)) * 1000.0f / (float)elapsed;
                rate = clip(rate, -kFeedbackDerivativeLimitDegPerS, kFeedbackDerivativeLimitDegPerS);
                const float alpha = (float)elapsed / (kFeedbackDerivativeFilterMs + (float)elapsed);
                state.derivativeFiltered += alpha * (rate - state.derivativeFiltered);
                derivativeRate = state.derivativeFiltered;
            } else {
                state.derivativeFiltered = 0.0f;
            }
        }
        state.lastFast = fb.fast;
        state.lastDerivativeTarget = target;
        state.lastFastMs = now;
        state.lastFastValid = true;
    } else {
        state.derivativeFiltered = 0.0f;
    }
    const float delta = target - state.lastTarget;
    const int8_t targetDirection = sign(delta);
    if (targetDirection) {
        if (!state.direction) state.direction = targetDirection;
        if (targetDirection == state.direction) {
            if (state.phase == Pending) state.phase = Tracking;
            state.candidate = 0;
        } else if (state.candidate != targetDirection) {
            state.candidate = targetDirection;
            state.candidateAnchor = state.lastTarget;
            state.candidateSince = now;
            state.phase = Pending;
        }
    }
    state.lastTarget = target;
    if (state.candidate && (target - state.candidateAnchor) * state.candidate >= p[jointConfirmDegParameter(joint)] &&
        now - state.candidateSince >= p[ReversalConfirmMs]) {
        state.direction = state.candidate; state.candidate = 0;
        // The old loading direction must not carry integral bias into take-up.
        for (uint8_t i = 0; i < count; ++i) state.integral[i] = 0;
        state.errorSign = 0;
        state.errorSince = now;
        state.phase = p[jointTakeupMaxParameter(joint)] > 0 ? Takeup : Tracking;
        state.responseAnchor = fb.fast;
        state.takeupStartMs = now;
    }
    if (state.phase == Takeup &&
        state.direction * (fb.fast - state.responseAnchor) >= p[jointExitTravelParameter(joint)]) {
        // The joint answered; hand back to plain tracking and let the lift die.
        state.phase = Tracking;
    }
    out.count = count;
    const float kp = p[jointKpParameter(joint)] *
        (p[AdaptiveKpEnabled] > 0.5f ? kpScale[joint] : 1.0f);
    const float ki = p[jointKiParameter(joint)];
    const float kd = p[jointKdParameter(joint)];
    const float iLimit = p[jointIntegralLimitParameter(joint)];
    const float error = target - fb.fast, slowError = target - fb.slow;
    const float integralError = sign(error) == sign(slowError) ?
        sign(error) * fmaxf(0, fabsf(slowError) - p[JointKiIntegralNoiseDeg]) : 0;
    if (fb.fresh && state.errorSign != sign(integralError)) {
        state.errorSign = sign(integralError); state.errorSince = now;
    }
    const bool integrate = fb.fresh && fb.sampleDt > 0 && !state.saturated && state.phase == Tracking &&
        state.errorSign != 0 && now - state.errorSince >= p[JointKiIntegralConfirmMs];
    float takeupB = 0.0f;
    if (state.phase == Takeup) {
        const float elapsedMs = (float)(now - state.takeupStartMs);
        const float bmax = p[jointTakeupMaxParameter(joint)];
        const float b0 = ramp[jointB0Parameter(joint)];
        const float timeoutMs = ramp[ReversalRampTakeupTimeoutMs];
        if (timeoutMs > 0.0f && elapsedMs >= timeoutMs) {
            state.phase = Tracking;
        } else {
            takeupB = fminf(bmax, b0 + ramp[jointVbParameter(joint)] * elapsedMs * 0.001f);
        }
    }
    float raw[MAX_JOINT_ACTUATORS] = {};
    for (uint8_t i = 0; i < count; ++i) {
        if (integrate) state.integral[i] = clip(state.integral[i] + ki * slope[i] * integralError * fb.sampleDt, -iLimit, iLimit);
        out.proportional[i] = kp * slope[i] * error;
        out.integral[i] = state.integral[i];
        // D acts only during plain tracking; take-up and pending reversal must not
        // be biased by a stale or target-induced velocity.
        out.derivative[i] = (state.phase == Tracking && derivativeRate != 0.0f)
            ? kd * slope[i] * derivativeRate : 0.0f;
        raw[i] = measured[i] + out.proportional[i] + state.integral[i] + out.derivative[i];
        if (state.phase == Takeup && takeupB > 0.0f) {
            // PDF Eq.(7): b_i is an additive winding bias on top of PI, growing
            // from B0 to Bmax until the joint responds or take-up times out.
            const int8_t motorDirection = sign(slope[i]) * state.direction;
            out.reversal[i] = motorDirection * takeupB;
            raw[i] += out.reversal[i];
        }
    }
    if (state.phase == Takeup && takeupB <= 0.0f) {
        state.phase = Tracking;
    }
    if (count == 2) {
        const float z0 = model[0] - measured[0], z1 = model[1] - measured[1];
        const float along = (slope[0] * z0 + slope[1] * z1) / (slope[0] * slope[0] + slope[1] * slope[1]);
        out.recovery[0] = p[DualTendonPairRecoveryGain] * (z0 - slope[0] * along);
        out.recovery[1] = p[DualTendonPairRecoveryGain] * (z1 - slope[1] * along);
    }
    bool recoveryAllowed = count == 2;
    for (uint8_t i = 0; i < count; ++i)
        if (raw[i] + out.recovery[i] < low[i] || raw[i] + out.recovery[i] > high[i]) recoveryAllowed = false;
    state.saturated = false;
    for (uint8_t i = 0; i < count; ++i) {
        if (!recoveryAllowed) out.recovery[i] = 0;
        const float candidate = raw[i] + out.recovery[i];
        out.rawTargets[i] = candidate;
        const float applied = clip(candidate, low[i], high[i]);
        if (fabsf(candidate - applied) > 0.5f) state.saturated = true;
        out.targets[i] = lroundf(applied);
        if ((applied - measured[i]) * slope[i] * error > 0 && fabsf(applied - measured[i]) >= 1) out.pushing = true;
    }
    out.saturated = state.saturated;
    // A mechanical boundary clamp is normal operation, not collision evidence.
    for (uint8_t i = 0; i < count; ++i)
        if (raw[i] < low[i] || raw[i] > high[i]) { out.pushing = false; out.boundaryClamped = true; }
    state.boundaryClamped = out.boundaryClamped;
    if (!suspendedThisCycle) updateAdaptiveKp(joint, target, fb, now, p);
    return out;
}

bool JointFeedbackController::AdaptiveEvent::qualifies(bool active, uint32_t now, uint32_t dwell) {
    if (!active) {
        phase = Idle;
    } else if (phase == Idle) {
        since = now;
        phase = Timing;
    } else if (phase == Timing && now - since >= dwell) {
        phase = Counted;
        return true;
    }
    return false;
}

void JointFeedbackController::updateAdaptiveKp(uint8_t joint, float target, const FeedbackEstimate& fb,
                                               uint32_t now, const Parameters& p) {
    auto& state = states[joint];
    auto& evidence = state.adaptive;
    if (!(p[AdaptiveKpEnabled] > 0.5f) || now - evidence.feedbackMs > 100) {
        evidence = {};
    }
    if (!(p[AdaptiveKpEnabled] > 0.5f) || !fb.fresh || !(fb.sampleDt > 0)) return;
    evidence.feedbackMs = now;
    if (!evidence.started) {
        evidence.windowStartMs = now;
        evidence.started = true;
    }

    const bool tracking = state.phase == Tracking && state.candidate == 0 &&
        !state.saturated && !state.boundaryClamped;
    const int8_t direction = state.direction;
    const float position = direction * fb.fast;
    if (direction != evidence.direction) {
        evidence.direction = direction;
        evidence.travelAnchor = position;
        evidence.armed = false;
        evidence.overshoot = {};
    }
    if (!evidence.armed && direction) {
        evidence.travelAnchor = fminf(evidence.travelAnchor, position);
        evidence.armed = position - evidence.travelAnchor >= MMHAND_ADAPTIVE_OVERSHOOT_ARM_DEG;
    }
    const float error = target - fb.fast;
    const bool overshooting = tracking && evidence.armed && -direction * error > p[AdaptiveKpOvershootDeg];
    constexpr uint32_t windowMs = 10000;
    if (now - evidence.windowStartMs >= windowMs) {
        const bool eligible = tracking && !overshooting;
#ifdef MMHAND_ADAPTIVE_KP_LOG_JOINT
        const float before = effectiveKpOf(joint, p);
#endif
        if (eligible && evidence.overshoots >= MMHAND_ADAPTIVE_OVERSHOOT_EVENTS) {
            kpScale[joint] = fmaxf(MMHAND_ADAPTIVE_KP_MIN, kpScale[joint] * 0.9f);
        } else if (eligible && !evidence.overshoots && evidence.lags >= MMHAND_ADAPTIVE_LAG_EVENTS) {
            kpScale[joint] = fminf(MMHAND_ADAPTIVE_KP_MAX, kpScale[joint] * MMHAND_ADAPTIVE_KP_STEP);
        }
#ifdef MMHAND_ADAPTIVE_KP_LOG_JOINT
        if (joint == MMHAND_ADAPTIVE_KP_LOG_JOINT) {
            FW_LOG_INFO_PRINTF("[ADAPTIVE_KP_WINDOW] J%02u start_ms=%lu end_ms=%lu overshoots=%u lags=%u before=%.3f after=%.3f eligible=%u\r\n",
                (unsigned)joint, (unsigned long)evidence.windowStartMs, (unsigned long)(evidence.windowStartMs + windowMs),
                (unsigned)evidence.overshoots, (unsigned)evidence.lags, before, effectiveKpOf(joint, p), (unsigned)eligible);
        }
#endif
        evidence.windowStartMs += windowMs;
        evidence.overshoots = evidence.lags = 0;
    }
    const bool lagging = tracking && !overshooting && fabsf(state.targetVelocity) >= 3.0f &&
        error / state.targetVelocity > p[AdaptiveKpLagLimitS];
    evidence.overshoots += evidence.overshoot.qualifies(overshooting, now, MMHAND_ADAPTIVE_OVERSHOOT_DWELL_MS);
    evidence.lags += evidence.lag.qualifies(lagging, now, MMHAND_ADAPTIVE_LAG_MS);
}

}
