#include "DualTendonControl.h"

#include <math.h>
#include <string.h>
#include "ControlOutputBuilder.h"
#include "ControlSafety.h"
#include "../hardware/HardwareMap.h"

namespace {

struct Point2D {
    float x;
    float y;
};

struct DualTendonJointConfig {
    uint8_t enabled;
    uint8_t model;
    float motorZero[2];
    float countsPerMm[2];
    float kp[2];
    float ki[2];
    float kd[2];
    float integralLimit[2];
};

struct DualTendonJointState {
    uint32_t lastToken;
    float integral[2];
    float prevFbModel[2];
    uint8_t prevValid;
};

static DualTendonJointState g_dualTendonState[ENCODER_TOTAL_NUM];

static const float kControlPeriodSec = 0.010f;
static const float kDegToRad = 3.14159265358979323846f / 180.0f;

static const uint8_t kDualTendonModelSharedCadGeometry = 1;

static const float kDefaultDualTendonCountsPerMm = 40.0f;
static const float kDefaultDualTendonKp = 1.0f;
static const float kDefaultDualTendonKi = 0.0f;
static const float kDefaultDualTendonKd = 0.03f;
static const float kDefaultDualTendonIntegralLimit = 0.0f;
static const int32_t kDualTendonMaxReleaseFromCalibCounts = 450;

static const Point2D kJoint16Origin = {0.0f, 0.0f};
static const Point2D kJoint16A1_0 = {-5.741894f, 0.244188f};
static const Point2D kJoint16B1_0 = {-1.808043f, 4.317810f};
static const Point2D kJoint16C1 = {6.900f, 3.50f};
static const float kJoint16L1_AB = 5.663f;
static const float kJoint16L1_0 = 14.4094f;
static const float kJoint16ThetaSwitch1 = 52.87f * kDegToRad;

static const Point2D kJoint16A2_0 = {1.232425f, -4.847660f};
static const Point2D kJoint16B2_0 = {4.245159f, -1.727883f};
static const Point2D kJoint16C2 = {6.900f, -2.502f};
static const float kJoint16L2_AB = 4.337f;
static const float kJoint16L2_0 = 6.1338f;
static const float kJoint16ThetaSwitch2 = 30.51f * kDegToRad;

static bool getDualTendonConfig(uint8_t jointIndex, DualTendonJointConfig* out)
{
    if (!out) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->enabled = 1;

    out->countsPerMm[0] = kDefaultDualTendonCountsPerMm;
    out->countsPerMm[1] = kDefaultDualTendonCountsPerMm;
    out->kp[0] = kDefaultDualTendonKp;
    out->kp[1] = kDefaultDualTendonKp;
    out->ki[0] = kDefaultDualTendonKi;
    out->ki[1] = kDefaultDualTendonKi;
    out->kd[0] = kDefaultDualTendonKd;
    out->kd[1] = kDefaultDualTendonKd;
    out->integralLimit[0] = kDefaultDualTendonIntegralLimit;
    out->integralLimit[1] = kDefaultDualTendonIntegralLimit;

    switch (jointIndex) {
        case 0:
        case 4:
        case 8:
        case 12:
        case kJoint16Index:
        case 20:
            out->model = kDualTendonModelSharedCadGeometry;
            out->motorZero[0] = 2557.0f;
            out->motorZero[1] = 358.0f;
            return true;
        default:
            out->enabled = 0;
            return false;
    }
}

static float constrainFloat(float value, float minValue, float maxValue)
{
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

static Point2D rotateAroundCW(Point2D p, Point2D origin, float thetaRad)
{
    const float dx = p.x - origin.x;
    const float dy = p.y - origin.y;
    const float c = cosf(thetaRad);
    const float s = sinf(thetaRad);
    Point2D out;
    out.x = origin.x + dx * c + dy * s;
    out.y = origin.y - dx * s + dy * c;
    return out;
}

static float dist2D(Point2D a, Point2D b)
{
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    return sqrtf(dx * dx + dy * dy);
}

static float joint16Tendon1LengthChange(float qRad)
{
    const Point2D a = rotateAroundCW(kJoint16A1_0, kJoint16Origin, qRad);
    const Point2D b = rotateAroundCW(kJoint16B1_0, kJoint16Origin, qRad);
    const float lWrap = kJoint16L1_AB + dist2D(b, kJoint16C1);
    const float lDirect = dist2D(a, kJoint16C1);
    const float length = (qRad > kJoint16ThetaSwitch1) ? lDirect : lWrap;
    return length - kJoint16L1_0;
}

static float joint16Tendon2LengthChange(float qRad)
{
    const Point2D a = rotateAroundCW(kJoint16A2_0, kJoint16Origin, qRad);
    const Point2D b = rotateAroundCW(kJoint16B2_0, kJoint16Origin, qRad);
    const float lWrap = kJoint16L2_AB + dist2D(b, kJoint16C2);
    const float lDirect = dist2D(a, kJoint16C2);
    const float length = (qRad < kJoint16ThetaSwitch2) ? lDirect : lWrap;
    return length - kJoint16L2_0;
}

static bool tendonLengthChange(const DualTendonJointConfig& cfg,
                               uint8_t jointIndex,
                               uint8_t actuatorIndex,
                               float qDeg,
                               float* outMm)
{
    if (!outMm) {
        return false;
    }
    switch (cfg.model) {
        case kDualTendonModelSharedCadGeometry: {
            (void)jointIndex;
            const float qRad = qDeg * kDegToRad;
            *outMm = (actuatorIndex == 0)
                ? joint16Tendon1LengthChange(qRad)
                : joint16Tendon2LengthChange(qRad);
            return true;
        }
        default:
            return false;
    }
}

static bool getDualTendonActuators(uint8_t jointIndex, JointActuatorItem outActuators[2])
{
    if (!outActuators || getJointActuatorCount(jointIndex) < 2) {
        return false;
    }
    if (!getJointActuator(jointIndex, 0, &outActuators[0]) ||
        !getJointActuator(jointIndex, 1, &outActuators[1])) {
        return false;
    }
    return outActuators[0].sign != outActuators[1].sign;
}

static int32_t clampDualTendonReleaseFromCalib(uint8_t motorChannel,
                                               int32_t calibratedZeroRawAbs,
                                               int32_t targetRawAbs)
{
    const int32_t zeroTendonAbs =
        motorRawAbsToTendonAbs(motorChannel, calibratedZeroRawAbs);
    const int32_t minReleaseTendonAbs =
        zeroTendonAbs - kDualTendonMaxReleaseFromCalibCounts;
    int32_t targetTendonAbs = motorRawAbsToTendonAbs(motorChannel, targetRawAbs);
    if (targetTendonAbs < minReleaseTendonAbs) {
        targetTendonAbs = minReleaseTendonAbs;
    }
    return clampServoPos(motorTendonAbsToRawAbs(motorChannel, targetTendonAbs));
}

} // namespace

bool isDualTendonJoint(uint8_t jointIndex)
{
    DualTendonJointConfig cfg;
    if (!getDualTendonConfig(jointIndex, &cfg) || !cfg.enabled) {
        return false;
    }
    JointActuatorItem actuators[2];
    return getDualTendonActuators(jointIndex, actuators);
}

void resetDualTendonController(uint8_t jointIndex)
{
    if (jointIndex >= ENCODER_TOTAL_NUM) {
        return;
    }
    g_dualTendonState[jointIndex] = DualTendonJointState();
}

bool computeDualTendonMotorTargets(uint8_t jointIndex,
                                   float qRefDeg,
                                   float qFbDeg,
                                   const ServoAngleData_t* servoData,
                                   uint32_t commandToken,
                                   DualTendonMotorTargets* outTargets)
{
    if (!servoData || !outTargets || jointIndex >= ENCODER_TOTAL_NUM) {
        return false;
    }

    DualTendonJointConfig cfg;
    if (!getDualTendonConfig(jointIndex, &cfg) || !cfg.enabled) {
        return false;
    }

    JointActuatorItem actuators[2];
    if (!getDualTendonActuators(jointIndex, actuators)) {
        return false;
    }

    float xRefMm[2] = {0.0f, 0.0f};
    float xFbMm[2] = {0.0f, 0.0f};
    for (uint8_t i = 0; i < 2; i++) {
        if (!tendonLengthChange(cfg, jointIndex, i, qRefDeg, &xRefMm[i]) ||
            !tendonLengthChange(cfg, jointIndex, i, qFbDeg, &xFbMm[i])) {
            return false;
        }
    }

    DualTendonJointState& state = g_dualTendonState[jointIndex];
    outTargets->count = 2;
    for (uint8_t i = 0; i < 2; i++) {
        const uint8_t motorChannel = actuators[i].motorChannel;
        if (motorChannel >= SERVO_TOTAL_NUM ||
            !isMotorChannelValid(motorChannel) ||
            servoData->onlineStatus[motorChannel] == 0) {
            return false;
        }

        outTargets->motorChannel[i] = motorChannel;
        const float zeroTendon =
            (float)motorRawAbsToTendonAbs(motorChannel, (int32_t)cfg.motorZero[i]);
        const float motorRefTendon = zeroTendon - cfg.countsPerMm[i] * xRefMm[i];
        const float motorFbTendon = zeroTendon - cfg.countsPerMm[i] * xFbMm[i];
        outTargets->motorRef[i] =
            (float)motorTendonAbsToRawAbs(motorChannel, (int32_t)motorRefTendon);
        outTargets->motorFbModel[i] =
            (float)motorTendonAbsToRawAbs(motorChannel, (int32_t)motorFbTendon);
        outTargets->motorErr[i] = outTargets->motorRef[i] - outTargets->motorFbModel[i];
        outTargets->pidCorrection[i] = 0.0f;
    }

    const bool resetState =
        !state.prevValid || (state.lastToken != commandToken);
    if (resetState) {
        state.integral[0] = 0.0f;
        state.integral[1] = 0.0f;
        state.prevFbModel[0] = outTargets->motorFbModel[0];
        state.prevFbModel[1] = outTargets->motorFbModel[1];
        state.prevValid = 1;
        state.lastToken = commandToken;
    }

    for (uint8_t i = 0; i < 2; i++) {
        const uint8_t motorChannel = outTargets->motorChannel[i];
        const float motorCurrent = (float)servoData->servoAngles[motorChannel];
        const float dFb = resetState
            ? 0.0f
            : (outTargets->motorFbModel[i] - state.prevFbModel[i]) / kControlPeriodSec;

        state.integral[i] += outTargets->motorErr[i] * kControlPeriodSec;
        state.integral[i] = constrainFloat(state.integral[i],
                                           -cfg.integralLimit[i],
                                           cfg.integralLimit[i]);

        const float correction =
            cfg.kp[i] * outTargets->motorErr[i] +
            cfg.ki[i] * state.integral[i] -
            cfg.kd[i] * dFb;
        outTargets->pidCorrection[i] = correction;
        const int32_t requestedTarget =
            clampServoPos((int32_t)(motorCurrent + correction));
        outTargets->targetAbs[i] = clampDualTendonReleaseFromCalib(
            motorChannel,
            (int32_t)cfg.motorZero[i],
            requestedTarget
        );
        state.prevFbModel[i] = outTargets->motorFbModel[i];
    }

    return true;
}
