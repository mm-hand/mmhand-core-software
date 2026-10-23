/** @file JointFeedbackController.cpp
 * @brief 计算关节反馈控制、拉绳、配对恢复与碰撞推进状态。
 */
#include "JointFeedbackController.h"
#include "../config/topology/HardwareMap.h"
#include "../config/ControlConfig.h"
#include "../system/calibration/TendonCalibrationLut.h"
#include "../system/rtos/SharedSafetyState.h"
#include "../hal/servo/ServoValue.h"
#include <math.h>

namespace mmhand::control {
using namespace mmhand::config::control;
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
RampParameter jointLoadLimitParameter(uint8_t joint) {
    return rampTypeParameter(joint, ReversalRampLoadLimitAa, ReversalRampLoadLimitMcp,
                             ReversalRampLoadLimitPip, ReversalRampLoadLimitDip,
                             ReversalRampLoadLimitCmc);
}
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
}
void JointFeedbackController::resetAll() { for (auto& state : states) state = {}; }

JointControlResult JointFeedbackController::compute(uint8_t joint, float target, const FeedbackEstimate& fb,
    uint32_t now, const ServoAngleData_t& servo, const ServoTelemetryData_t& telemetry,
    const JointCalibrationResult& lut, uint32_t motorMask, const Parameters& p,
    const RampParameters& ramp) {
    JointControlResult out;
    if (joint >= ENCODER_TOTAL_NUM || !p.configured || !fb.ready || !isfinite(target)) return out;
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
        // The LUT working window (calibrated span widened by the configurable
        // release/tight margins) is the only command boundary; nothing widens it.
        int32_t lo, hi;
        const bool bounded = tendonCalibrationMotorAbsRangeWithMargins(lut, i, motor,
            p[OutputLutReleaseMarginCounts], p[OutputLutTightMarginCounts], &lo, &hi);
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
    }
    // D term uses the measured joint velocity only; a target step must not create
    // a derivative kick. The 20 ms fast mean already rejects most encoder jitter;
    // this first-order low-pass plus hard rate limit rejects the rest.
    float derivativeRate = 0.0f;
    const bool derivativeFresh = fb.fresh && fb.sampleDt > 0.0f;
    if (derivativeFresh) {
        if (state.lastFastValid) {
            const uint32_t elapsed = now - state.lastFastMs;
            if (elapsed >= 1 && elapsed <= 100) {
                float rate = (fb.fast - state.lastFast) * 1000.0f / (float)elapsed;
                rate = clip(rate, -kFeedbackDerivativeLimitDegPerS, kFeedbackDerivativeLimitDegPerS);
                const float alpha = (float)elapsed / (kFeedbackDerivativeFilterMs + (float)elapsed);
                state.derivativeFiltered += alpha * (rate - state.derivativeFiltered);
                derivativeRate = state.derivativeFiltered;
            } else {
                state.derivativeFiltered = 0.0f;
            }
        }
        state.lastFast = fb.fast;
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
        state.takeupMagnitude = ramp[jointB0Parameter(joint)];
        state.takeupFrozen = false;
        for (uint8_t i = 0; i < count; ++i) state.motorAnchor[i] = measured[i];
    }
    if (state.phase == Takeup &&
        state.direction * (fb.fast - state.responseAnchor) >= p[jointExitTravelParameter(joint)]) {
        // The joint answered; hand back to plain tracking and let the lift die.
        state.phase = Tracking;
    }
    out.count = count;
    const float kp = p[jointKpParameter(joint)];
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
        const float vb = ramp[jointVbParameter(joint)];
        const float loadLimit = ramp[jointLoadLimitParameter(joint)];
        float b = fminf(bmax, b0 + vb * elapsedMs * 0.001f);
        bool overload = false;
        bool hardOverload = false;
        for (uint8_t i = 0; i < count; ++i) {
            const int16_t load = mmhand::servo::loadMagnitude(telemetry.load[out.motors[i]]);
            overload = overload || (float)load >= loadLimit;
            hardOverload = hardOverload || load >= 1000;
        }
        if (overload) state.takeupFrozen = true;
        if (state.takeupFrozen) b = fminf(b, state.takeupMagnitude);
        else state.takeupMagnitude = b;
        takeupB = b;
        const float timeoutMs = ramp[ReversalRampTakeupTimeoutMs];
        if ((timeoutMs > 0.0f && elapsedMs >= timeoutMs) || hardOverload) {
            state.phase = Tracking;
            takeupB = 0.0f;
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
            ? -kd * slope[i] * derivativeRate : 0.0f;
        raw[i] = measured[i] + out.proportional[i] + state.integral[i] + out.derivative[i];
        if (state.phase == Takeup && takeupB > 0.0f) {
            // PDF Eq.(7): b_i is an additive winding bias on top of PI, growing
            // from B0 to Bmax and frozen when the load threshold is reached.
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
    return out;
}
}
