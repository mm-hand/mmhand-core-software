/**
 * @file UrdfJointLimits.generated.h
 * @brief 活动 URDF 的 21 关节静态角度上下限和 Manual 映射 CRC（自动生成）。
 *
 * 由 tools/generate_robot_model.py 生成，禁止手工编辑。
 */
#ifndef URDF_JOINT_LIMITS_GENERATED_H
#define URDF_JOINT_LIMITS_GENERATED_H

#include <Arduino.h>

#include "../system/SystemRuntimeContext.h"

struct UrdfJointLimitDeg {
    float minDeg;
    float maxDeg;
};

static const uint8_t kUrdfFeedbackMappingVersion = 3;
static const uint8_t kManualScaledUrdfFeedbackMappingVersion = 4;
static const uint32_t kManualScaledUrdfConfigCrc = 0xC490BA68UL;
static const UrdfJointLimitDeg kUrdfJointLimits[ENCODER_TOTAL_NUM] = {
    {-75.000000f, 16.000000f},  // J00
    {-14.000000f, 90.000000f},  // J01
    {-9.000000f, 89.000000f},  // J02
    {-13.000000f, 101.000000f},  // J03
    {-46.000000f, 26.000000f},  // J04
    {-14.000000f, 90.000000f},  // J05
    {-9.000000f, 89.000000f},  // J06
    {-13.000000f, 101.000000f},  // J07
    {-32.000000f, 30.000000f},  // J08
    {-14.000000f, 90.000000f},  // J09
    {-9.000000f, 89.000000f},  // J10
    {-13.000000f, 101.000000f},  // J11
    {-28.000000f, 36.000000f},  // J12
    {-14.000000f, 90.000000f},  // J13
    {-9.000000f, 89.000000f},  // J14
    {-13.000000f, 101.000000f},  // J15
    {-75.000000f, 25.000000f},  // J16
    {-19.000000f, 90.000000f},  // J17
    {-9.000000f, 89.000000f},  // J18
    {-13.000000f, 101.000000f},  // J19
    {0.000000f, 54.000000f},  // J20
};

static_assert(
    sizeof(kUrdfJointLimits) / sizeof(kUrdfJointLimits[0]) == ENCODER_TOTAL_NUM,
    "URDF joint limit table must match ENCODER_TOTAL_NUM");

#endif // URDF_JOINT_LIMITS_GENERATED_H
