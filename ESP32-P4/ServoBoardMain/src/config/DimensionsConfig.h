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

inline constexpr uint8_t ENCODER_TOTAL_NUM = 21;       ///< 磁编码器及逻辑关节总数。
inline constexpr uint8_t SERVO_TOTAL_NUM = 27;         ///< 飞特舵机通道总数。
inline constexpr uint8_t JOINT_COUNT = ENCODER_TOTAL_NUM; ///< 供控制域使用的关节数别名。
inline constexpr uint8_t MAX_JOINT_ACTUATORS = 2;      ///< 一个关节最多关联两台拮抗舵机。
inline constexpr uint8_t NUM_BUSES = 4;                ///< 独立飞特 UART 总线数量。
inline constexpr uint8_t MAX_SERVOS_PER_BUS = 8;       ///< 单总线静态缓存容量。
inline constexpr uint8_t MAX_SERVO_ID = 32;            ///< 允许出现在硬件映射中的最大舵机 ID。
inline constexpr uint8_t MOTOR_MAP_INVALID_BUS = 0xFF; ///< 未映射电机使用的总线哨兵值。

inline constexpr uint8_t TACTILE_GROUP_NUM = 5;        ///< 五根手指对应的触觉组数。
inline constexpr uint8_t TACTILE_SENSOR_PER_GROUP = 3; ///< 每根手指的触觉模组数。
inline constexpr uint8_t TACTILE_AXIS_NUM = 3;         ///< 每个 taxel 的三轴力分量数。
inline constexpr uint8_t TACTILE_TIP_TAXELS = 25;      ///< 单个指尖模组 taxel 数。
inline constexpr uint8_t TACTILE_PAD_TAXELS = 52;      ///< 单个指腹模组 taxel 数。
inline constexpr uint8_t TACTILE_SENSOR0_TAXELS = TACTILE_PAD_TAXELS; ///< 组内 0 号模组规模。
inline constexpr uint8_t TACTILE_SENSOR1_TAXELS = TACTILE_TIP_TAXELS; ///< 组内 1 号模组规模。
inline constexpr uint8_t TACTILE_SENSOR2_TAXELS = TACTILE_TIP_TAXELS; ///< 组内 2 号模组规模。
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
