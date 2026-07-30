/**
 * @file CalibrationFeedbackReader.h
 * @brief 为自动标定流程提供只读、带有效性校验的传感器反馈接口。
 *
 * 调用方不再直接窥视 CAN、舵机反馈或遥测队列。所有读取都先验证索引、
 * 在线状态和必要的新鲜度，再返回一个一致的快照值。
 */
#ifndef MMHAND_CALIBRATION_FEEDBACK_READER_H
#define MMHAND_CALIBRATION_FEEDBACK_READER_H

#include <stdint.h>

#include "../shared/TaskSharedData.h"

namespace mmhand::calibration {

bool readLatestEncoderSample(TaskSharedData_t* sharedData,
                             uint8_t jointIndex,
                             uint16_t* outRaw,
                             int32_t* outOriented);

bool readLatestEncoderOriented(TaskSharedData_t* sharedData,
                               uint8_t jointIndex,
                               int32_t* outEncoder);

bool readLatestServoAbs(TaskSharedData_t* sharedData,
                        uint8_t bus,
                        uint8_t id,
                        int32_t* outAbs);

bool readLatestServoRaw(TaskSharedData_t* sharedData,
                        uint8_t bus,
                        uint8_t id,
                        int16_t* outRaw);

bool readLatestServoLoad(TaskSharedData_t* sharedData,
                         uint8_t bus,
                         uint8_t id,
                         int16_t* outLoad);

bool readLatestMotorServoAbs(TaskSharedData_t* sharedData,
                             uint8_t motorChannel,
                             int32_t* outAbs);

bool readLatestMotorServoLoad(TaskSharedData_t* sharedData,
                              uint8_t motorChannel,
                              int16_t* outLoad);

/** 初始反馈等待失败时输出队列、在线位和样本年龄诊断。 */
void logInitialFeedbackDiagnostic(TaskSharedData_t* sharedData,
                                  uint8_t jointIndex,
                                  uint8_t bus,
                                  uint8_t id,
                                  bool encoderOk,
                                  bool absOk,
                                  bool rawOk);

} // namespace mmhand::calibration

#endif // MMHAND_CALIBRATION_FEEDBACK_READER_H
