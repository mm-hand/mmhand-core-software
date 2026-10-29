/** @file JointFeedbackController.h
 * @brief 声明单绳和双绳关节控制器及其逐关节运行状态。
 */
#pragma once
#include "ControlParameters.h"
#include "RampParameters.h"
#include "FeedbackFilter.h"
#include "../data/ServoTypes.h"
#include "../system/SystemRuntimeContext.h"
#include "../system/calibration/CalibrationRuntimeStore.h"

namespace mmhand::control {
struct JointControlResult {
    uint8_t count = 0;
    uint8_t motors[MAX_JOINT_ACTUATORS] = {};
    int32_t targets[MAX_JOINT_ACTUATORS] = {};
    float proportional[MAX_JOINT_ACTUATORS] = {};
    float integral[MAX_JOINT_ACTUATORS] = {};
    float derivative[MAX_JOINT_ACTUATORS] = {};
    float reversal[MAX_JOINT_ACTUATORS] = {};
    float recovery[MAX_JOINT_ACTUATORS] = {};
    float rawTargets[MAX_JOINT_ACTUATORS] = {};
    bool saturated = false;
    bool boundaryClamped = false;
    bool pushing = false;
};
class JointFeedbackController {
public:
    JointControlResult compute(uint8_t joint, float target, const FeedbackEstimate& feedback,
        uint32_t now, const ServoAngleData_t& servo, const ServoTelemetryData_t& telemetry,
        const JointCalibrationResult& calibration, uint32_t motorMask,
        const Parameters& parameters, const RampParameters& ramp);
    void reset(uint8_t joint);
    void suspend(uint8_t joint, uint32_t now);
    void resetAll();
    float kpScaleOf(uint8_t joint) const;
    float effectiveKpOf(uint8_t joint, const Parameters& parameters) const;
    JointFeedbackController();
private:
    void updateAdaptiveKp(uint8_t joint, float target, const FeedbackEstimate& fb,
                          uint32_t now, const Parameters& parameters);
    enum Phase : uint8_t { Tracking, Pending, Takeup };
    struct State {
        bool initialized = false, saturated = false;
        uint32_t revision = 0, candidateSince = 0, errorSince = 0;
        int8_t errorSign = 0;
        int8_t direction = 0, candidate = 0;
        Phase phase = Tracking;
        float lastTarget = 0, candidateAnchor = 0, responseAnchor = 0;
        uint32_t takeupStartMs = 0;
        float takeupMagnitude = 0;
        bool takeupFrozen = false;
        float integral[MAX_JOINT_ACTUATORS] = {};
        float motorAnchor[MAX_JOINT_ACTUATORS] = {};
        // D-term history: filtered derivative of target minus measured angle.
        bool lastFastValid = false;
        uint32_t lastFastMs = 0;
        float lastFast = 0;
        float lastDerivativeTarget = 0;
        float derivativeFiltered = 0;
        // Adaptive Kp: slow closed-loop evidence, never a fast control loop.
        float lagScore = 0, overScore = 0;
        uint32_t lastLagMs = 0, lastEventMs = 0, noIncreaseUntilMs = 0;
        bool lagFresh = false, overFresh = false;
        float targetVelocity = 0, stableTarget = 0, lastHoldTarget = 0;
        float holdTarget = 0, holdPeak = 0, holdAmplitude = 0;
        uint32_t lastTargetMs = 0, lastTargetChangeMs = 0, holdStartMs = 0;
        int8_t moveDir = 0;
        bool holdActive = false, suspended = false, boundaryClamped = false;
    } states[ENCODER_TOTAL_NUM];
    float kpScale[ENCODER_TOTAL_NUM];
    uint32_t scaleRevision = 0;
    uint32_t lastAdaptiveCheckMs = 0;
};
}
