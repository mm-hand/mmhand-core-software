/**
 * @file UrdfJointLimits.generated.h
 * @brief 活动 URDF 的 21 关节静态角度上下限和映射 CRC（自动生成）。
 *
 * 由 tools/generate_robot_model.py 生成，禁止手工编辑。
 */
#ifndef URDF_JOINT_LIMITS_GENERATED_H
#define URDF_JOINT_LIMITS_GENERATED_H

#include <Arduino.h>

#include "../shared/TaskSharedData.h"

struct UrdfJointLimitDeg {
    float minDeg;
    float maxDeg;
};

static const uint8_t kUrdfFeedbackMappingVersion = 2;
static const uint32_t kUrdfJointLimitConfigCrc = 0xA3BF080FUL;
static const uint8_t kManualScaledUrdfFeedbackMappingVersion = 3;
static const uint32_t kManualScaledUrdfConfigCrc = 0x7C0F9E73UL;
static const UrdfJointLimitDeg kUrdfJointLimits[ENCODER_TOTAL_NUM] = {
    {0.000000f, 54.000000f},  // J00
    {-15.000000f, 90.000000f},  // J01
    {-9.000000f, 89.000000f},  // J02
    {-13.000000f, 101.000000f},  // J03
    {0.000000f, 61.000000f},  // J04
    {-15.000000f, 90.000000f},  // J05
    {-9.000000f, 89.000000f},  // J06
    {-13.000000f, 101.000000f},  // J07
    {0.000000f, 59.000000f},  // J08
    {-14.000000f, 90.000000f},  // J09
    {-9.000000f, 89.000000f},  // J10
    {-13.000000f, 101.000000f},  // J11
    {0.000000f, 58.000000f},  // J12
    {-14.000000f, 90.000000f},  // J13
    {-9.000000f, 89.000000f},  // J14
    {-13.000000f, 101.000000f},  // J15
    {0.000000f, 57.000000f},  // J16
    {-19.000000f, 90.000000f},  // J17
    {-9.000000f, 89.000000f},  // J18
    {-13.000000f, 101.000000f},  // J19
    {0.000000f, 77.000000f},  // J20
};

static_assert(
    sizeof(kUrdfJointLimits) / sizeof(kUrdfJointLimits[0]) == ENCODER_TOTAL_NUM,
    "URDF joint limit table must match ENCODER_TOTAL_NUM");

#endif // URDF_JOINT_LIMITS_GENERATED_H
