/**
 * @file JointTypes.h
 * @brief 定义磁编码器、映射角度和关节控制诊断数据。
 */
#ifndef MMHAND_JOINT_TYPES_H
#define MMHAND_JOINT_TYPES_H

#include "../config/DimensionsConfig.h"
#include <stdint.h>

struct RemoteSensorData_t {
    uint16_t encoderValues[ENCODER_TOTAL_NUM];
    uint8_t errorFlags[ENCODER_TOTAL_NUM];
    uint32_t errorBitmap;
    uint32_t timestamp;
    bool isValid;
};

struct MappedAngleData_t {
    int16_t angleValues[ENCODER_TOTAL_NUM];
    int16_t outsideCounts[ENCODER_TOTAL_NUM];
    uint8_t validFlags[ENCODER_TOTAL_NUM];
    uint32_t timestamp;
    bool isValid;
};

struct JointDebugData_t {
    float targetDeg;
    float magActualDeg;
    float loop1Output;
    int16_t cmdTargetPos;
    uint32_t timestamp;
    uint8_t jointIndex;
    uint8_t valid;
    uint8_t cmdValid;
};

#endif // MMHAND_JOINT_TYPES_H
