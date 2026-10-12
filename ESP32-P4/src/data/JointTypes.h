/**
 * @file JointTypes.h
 * @brief 定义磁编码器与映射角度快照。
 */
#ifndef MMHAND_JOINT_TYPES_H
#define MMHAND_JOINT_TYPES_H

#include "../config/DimensionsConfig.h"
#include <stdint.h>

struct RemoteSensorData_t {
    uint16_t encoderValues[ENCODER_TOTAL_NUM]; ///< S3 上报的 14 bit 磁编码器 raw。
    uint8_t errorFlags[ENCODER_TOTAL_NUM];     ///< 每关节原始传感器错误标志。
    uint32_t errorBitmap;                     ///< errorFlags 的快速位图汇总。
    uint32_t timestamp;                       ///< P4 接收完整磁编帧的毫秒时间。
    bool isValid;                             ///< 至少收到过一帧结构有效的数据。
};

struct MappedAngleData_t {
    int16_t angleValues[ENCODER_TOTAL_NUM];   ///< 映射后角度，协议单位 0.01 degree。
    int16_t outsideCounts[ENCODER_TOTAL_NUM]; ///< 超出 Auto 工作区的带符号 raw 距离。
    uint8_t validFlags[ENCODER_TOTAL_NUM];    ///< 每关节 Manual/Auto/反馈综合有效标志。
    uint32_t timestamp;                      ///< 映射快照生成时间。
    bool isValid;                            ///< 快照整体是否可供控制读取。
};

#endif // MMHAND_JOINT_TYPES_H
