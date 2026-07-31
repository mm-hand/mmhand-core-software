/**
 * @file ServoTypes.h
 * @brief 定义舵机位置、在线状态及遥测反馈快照。
 * @details 区分 logical ABS、单圈 raw 和 Flash raw，禁止混用三个坐标系。
 */
#ifndef MMHAND_SERVO_TYPES_H
#define MMHAND_SERVO_TYPES_H

#include "../config/DimensionsConfig.h"
#include <stdint.h>

struct ServoStatus_t {
    uint8_t servoId;    ///< 物理舵机 ID。
    int16_t position;   ///< 单圈 Present Position 原始值。
    int16_t speed;      ///< Present Speed 有符号原始值。
    int16_t load;       ///< Present Load 有符号原始值。
    uint8_t voltage;    ///< Present Voltage 原始值。
    uint8_t temperature; ///< Present Temperature，单位摄氏度。
};

struct ServoAngleData_t {
    // P4 逻辑多圈位置，控制与标定统一使用该坐标。
    int32_t servoAngles[SERVO_TOTAL_NUM];
    // 舵机当前 0..4095 单圈位置。
    int16_t servoRawPositions[SERVO_TOTAL_NUM];
    // 最近一次真正落入 P4 Flash 的单圈位置。
    int16_t servoFlashRawPositions[SERVO_TOTAL_NUM];
    uint8_t onlineStatus[SERVO_TOTAL_NUM]; ///< 逻辑 Mxx 在线标志。
    uint32_t timestamp; ///< 完整位置快照的发布时间。
};

struct ServoTelemetryData_t {
    int16_t speed[SERVO_TOTAL_NUM]; ///< 飞特 Present Speed 原始有符号值。
    int16_t load[SERVO_TOTAL_NUM];  ///< 飞特 Present Load 原始有符号值。
    // ST3215 Present Current 原始有符号读数，物理单位由上位机解释。
    int16_t current[SERVO_TOTAL_NUM];
    uint8_t voltage[SERVO_TOTAL_NUM];     ///< Present Voltage 原始值。
    uint8_t temperature[SERVO_TOTAL_NUM]; ///< Present Temperature，单位摄氏度。
    uint8_t onlineStatus[SERVO_TOTAL_NUM]; ///< 遥测字段对应舵机在线标志。
    uint32_t timestamp;                    ///< 完整遥测快照发布时间。
};

#endif // MMHAND_SERVO_TYPES_H
