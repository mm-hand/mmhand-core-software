/**
 * @file CalibrationSequencePlanner.h
 * @brief 声明自动标定 active scope 解析、机械顺序排序和 J/M 位图规划接口。
 * @details 本模块只生成计划，不读取反馈、不发送舵机命令、不修改标定结果。
 */
#ifndef P4_MODULES_CALIBRATION_SEQUENCE_PLANNER_H
#define P4_MODULES_CALIBRATION_SEQUENCE_PLANNER_H

#include <stdint.h>

#include "../../runtime/SystemRuntimeContext.h"

uint8_t autoCalibrationSequenceRank(uint8_t jointIndex);
uint32_t calibrationJointMotorMask(uint8_t jointIndex);
uint32_t buildCalibrationJointMask(const uint8_t* joints, uint8_t jointCount);
uint32_t buildCalibrationMotorMask(const uint8_t* joints, uint8_t jointCount);
uint8_t buildAutoCalibrationJointListFromActiveScope(SystemRuntimeContext* sharedData,
                                                     uint8_t* outJoints,
                                                     uint8_t maxCount);

#endif // P4_MODULES_CALIBRATION_SEQUENCE_PLANNER_H
