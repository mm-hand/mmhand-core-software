/** @file JointFeedbackController.h
 * @brief 声明单绳和双绳关节控制器及其逐关节运行状态。
 */
#pragma once
#include "ControlParameters.h"
#include "FeedbackFilter.h"
#include "../system/SystemRuntimeContext.h"
#include "../system/calibration/CalibrationRuntimeStore.h"

namespace mmhand::control {
struct JointControlResult {
    uint8_t count = 0;
    uint8_t motors[MAX_JOINT_ACTUATORS] = {};
    int32_t targets[MAX_JOINT_ACTUATORS] = {};
    float proportional[MAX_JOINT_ACTUATORS] = {};
    float integral[MAX_JOINT_ACTUATORS] = {};
    float reversal[MAX_JOINT_ACTUATORS] = {};
    float recovery[MAX_JOINT_ACTUATORS] = {};
    float rawTargets[MAX_JOINT_ACTUATORS] = {};
    bool saturated = false;
    bool boundaryClamped = false;
    bool pushing = false;
    bool reversalFailed = false;
};
class JointFeedbackController {
public:
    JointControlResult compute(uint8_t joint, float target, const FeedbackEstimate& feedback,
        uint32_t now, float dt, const ServoAngleData_t& servo,
        const JointCalibrationResult& calibration, uint32_t motorMask,
        SystemRuntimeContext* system, const Parameters& parameters);
    void reset(uint8_t joint);
    void suspend(uint8_t joint, const ServoAngleData_t& servo, uint32_t now);
    void resetAll();
private:
    enum Phase : uint8_t { Tracking, Pending, Takeup, Settling, Failed };
    struct State {
        bool initialized = false, saturated = false;
        uint32_t revision = 0, stableSince = 0, candidateSince = 0, phaseSince = 0, cooldownUntil = 0;
        uint32_t errorSince = 0;
        int8_t errorSign = 0;
        int8_t direction = 0, candidate = 0;
        Phase phase = Tracking;
        float lastTarget = 0, stableTarget = 0, candidateAnchor = 0, responseAnchor = 0;
        float integral[MAX_JOINT_ACTUATORS] = {};
        float motorAnchor[MAX_JOINT_ACTUATORS] = {};
        float lastCommand[MAX_JOINT_ACTUATORS] = {};
        float hold[MAX_JOINT_ACTUATORS] = {};
    } states[ENCODER_TOTAL_NUM];
};
}
