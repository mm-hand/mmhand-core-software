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
    uint8_t servoId;
    int16_t position;
    int16_t speed;
    int16_t load;
    uint8_t voltage;
    uint8_t temperature;
};

struct ServoAngleData_t {
    // P4 逻辑多圈位置，控制与标定统一使用该坐标。
    int32_t servoAngles[SERVO_TOTAL_NUM];
    // 舵机当前 0..4095 单圈位置。
    int16_t servoRawPositions[SERVO_TOTAL_NUM];
    // 最近一次真正落入 P4 Flash 的单圈位置。
    int16_t servoFlashRawPositions[SERVO_TOTAL_NUM];
    uint8_t onlineStatus[SERVO_TOTAL_NUM];
    uint32_t timestamp;
};

struct ServoTelemetryData_t {
    int16_t speed[SERVO_TOTAL_NUM];
    int16_t load[SERVO_TOTAL_NUM];
    // ST3215 Present Current 原始有符号读数，物理单位由上位机解释。
    int16_t current[SERVO_TOTAL_NUM];
    uint8_t voltage[SERVO_TOTAL_NUM];
    uint8_t temperature[SERVO_TOTAL_NUM];
    uint8_t onlineStatus[SERVO_TOTAL_NUM];
    uint32_t timestamp;
};

#endif // MMHAND_SERVO_TYPES_H
