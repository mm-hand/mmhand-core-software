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

/** 读取最新磁编 raw 和按安装方向归一化后的环形坐标；反馈过期、离线或故障时返回 false。 */
bool readLatestEncoderSample(TaskSharedData_t* sharedData,
                             uint8_t jointIndex,
                             uint16_t* outRaw,
                             int32_t* outOriented);

/** 仅读取方向归一化磁编坐标，校验规则与 readLatestEncoderSample 相同。 */
bool readLatestEncoderOriented(TaskSharedData_t* sharedData,
                               uint8_t jointIndex,
                               int32_t* outEncoder);

/** 按扁平 Mxx 通道读取 P4 logical ABS，避免调用方重复做 bus/id 映射。 */
bool readLatestMotorServoAbs(TaskSharedData_t* sharedData,
                             uint8_t motorChannel,
                             int32_t* outAbs);

/** 按扁平 Mxx 通道读取飞特有符号 load 原值。 */
bool readLatestMotorServoLoad(TaskSharedData_t* sharedData,
                              uint8_t motorChannel,
                              int16_t* outLoad);

/** 按 Mxx 读取归一化到 0..4095 的舵机单圈 raw。 */
bool readLatestMotorServoRaw(TaskSharedData_t* sharedData,
                             uint8_t motorChannel,
                             int16_t* outRaw);

/** 初始反馈等待失败时输出队列、在线位和样本年龄诊断。 */
void logInitialFeedbackDiagnostic(TaskSharedData_t* sharedData,
                                  uint8_t jointIndex,
                                  uint8_t motorChannel,
                                  bool encoderOk,
                                  bool absOk,
                                  bool rawOk);

} // namespace mmhand::calibration

#endif // MMHAND_CALIBRATION_FEEDBACK_READER_H
