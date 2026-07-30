/**
 * @file DimensionsConfig.h
 * @brief 定义灵巧手固件在编译期固定的数据规模。
 * @details
 * - 本文件只保存数组维度和协议固定容量，不保存运行时状态。
 * - 修改关节数、舵机数或触觉规模会改变上下位机协议，必须同步评审。
 */
#ifndef MMHAND_DIMENSIONS_CONFIG_H
#define MMHAND_DIMENSIONS_CONFIG_H

#include <stdint.h>

inline constexpr uint8_t ENCODER_TOTAL_NUM = 21;
inline constexpr uint8_t SERVO_TOTAL_NUM = 27;
inline constexpr uint8_t JOINT_COUNT = ENCODER_TOTAL_NUM;
inline constexpr uint8_t MAX_JOINT_ACTUATORS = 2;
inline constexpr uint8_t NUM_BUSES = 4;
inline constexpr uint8_t MAX_SERVOS_PER_BUS = 8;
inline constexpr uint8_t MAX_SERVO_ID = 32;
inline constexpr uint8_t MOTOR_MAP_INVALID_BUS = 0xFF;

inline constexpr uint8_t TACTILE_GROUP_NUM = 5;
inline constexpr uint8_t TACTILE_SENSOR_PER_GROUP = 3;
inline constexpr uint8_t TACTILE_AXIS_NUM = 3;
inline constexpr uint8_t TACTILE_TIP_TAXELS = 25;
inline constexpr uint8_t TACTILE_PAD_TAXELS = 52;
inline constexpr uint8_t TACTILE_SENSOR0_TAXELS = TACTILE_PAD_TAXELS;
inline constexpr uint8_t TACTILE_SENSOR1_TAXELS = TACTILE_TIP_TAXELS;
inline constexpr uint8_t TACTILE_SENSOR2_TAXELS = TACTILE_TIP_TAXELS;
inline constexpr uint16_t TACTILE_SUMMARY_BYTES =
    TACTILE_GROUP_NUM * TACTILE_SENSOR_PER_GROUP * TACTILE_AXIS_NUM;
inline constexpr uint16_t TACTILE_FULL_BYTES =
    TACTILE_GROUP_NUM *
    (TACTILE_SENSOR0_TAXELS + TACTILE_SENSOR1_TAXELS +
     TACTILE_SENSOR2_TAXELS) *
    TACTILE_AXIS_NUM;

static_assert(SERVO_TOTAL_NUM <= 32, "Motor bitmap uses uint32_t");
static_assert(ENCODER_TOTAL_NUM <= 32, "Joint bitmap uses uint32_t");

#endif // MMHAND_DIMENSIONS_CONFIG_H
