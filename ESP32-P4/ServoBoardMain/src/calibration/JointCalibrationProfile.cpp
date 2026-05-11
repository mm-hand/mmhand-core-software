#include "JointCalibrationProfile.h"

// Field order:
// angleScopeDeg, bottomReservedDeg, topReservedDeg, angleReservedDeg,
// loadThreshold, tightenStep, tightenSpeed, tightenAcc, settleMs, maxSearchMs

static const JointCalibrationConfig kDefaultJointCalib = {
    90.0f, 3.0f, 3.0f, 6.0f,
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint0Calib = {
    70.0f, 3.0f, 3.0f, 45.0f,
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint1Calib = {
    80.0f, 3.0f, 3.0f, 40.0f,
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint2Calib = {
    95.0f, 5.0f, 2.0f, 45.0f,
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint3Calib = {
    100.0f, 5.0f, 2.0f, 50.0f,
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint4Calib = {
    70.0f, 3.0f, 2.0f, 45.0f,
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint5Calib = {
    80.0f, 3.0f, 2.0f, 40.0f,
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint6Calib = {
    95.0f, 5.0f, 2.0f, 45.0f,
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint7Calib = {
    100.0f, 5.0f, 2.0f, 50.0f,
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint8Calib = {
    70.0f, 3.0f, 2.0f, 45.0f,
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint9Calib = {
    80.0f, 3.0f, 2.0f, 40.0f,
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint10Calib = {
    95.0f, 5.0f, 2.0f, 45.0f,
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint11Calib = {
    100.0f, 5.0f, 2.0f, 50.0f,
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint12Calib = {
    70.0f, 3.0f, 2.0f, 45.0f,
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint13Calib = {
    80.0f, 3.0f, 2.0f, 40.0f,
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint14Calib = {
    95.0f, 5.0f, 2.0f, 45.0f,
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint15Calib = {
    100.0f, 5.0f, 2.0f, 50.0f,
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint16Calib = {
    70.0f, 3.0f, 2.0f, 45.0f,
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint17Calib = {
    70.0f, 3.0f, 2.0f, 45.0f,
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint18Calib = {
    80.0f, 3.0f, 2.0f, 40.0f,
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint19Calib = {
    95.0f, 5.0f, 2.0f, 45.0f,
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint20Calib = {
    100.0f, 5.0f, 2.0f, 50.0f,
    180, 10, 150, 10, 20, 5000
};

const JointCalibrationConfig kJointCalibrationProfile[ENCODER_TOTAL_NUM] = {
    kJoint0Calib,
    kJoint1Calib,
    kJoint2Calib,
    kJoint3Calib,
    kJoint4Calib,
    kJoint5Calib,
    kJoint6Calib,
    kJoint7Calib,
    kJoint8Calib,
    kJoint9Calib,
    kJoint10Calib,
    kJoint11Calib,
    kJoint12Calib,
    kJoint13Calib,
    kJoint14Calib,
    kJoint15Calib,
    kJoint16Calib,
    kJoint17Calib,
    kJoint18Calib,
    kJoint19Calib,
    kJoint20Calib
};

void loadJointCalibrationProfile(JointCalibrationConfig* dst, uint8_t count)
{
    if (!dst) return;

    uint8_t n = count;
    if (n > ENCODER_TOTAL_NUM) n = ENCODER_TOTAL_NUM;

    for (uint8_t i = 0; i < n; i++) {
        dst[i] = kJointCalibrationProfile[i];
    }
}
