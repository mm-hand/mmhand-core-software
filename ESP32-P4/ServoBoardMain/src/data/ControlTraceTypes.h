/**
 * @file ControlTraceTypes.h
 * @brief 定义使用同一 P4 单调时钟采集的关节控制跟踪快照。
 * @details 控制任务只覆盖最新快照；HostComm 按需读取，不得反向阻塞 10 ms 控制周期。
 */
#ifndef MMHAND_CONTROL_TRACE_TYPES_H
#define MMHAND_CONTROL_TRACE_TYPES_H

#include "../config/DimensionsConfig.h"
#include <stdint.h>

struct ControlTraceData_t {
    uint32_t sequence;                 ///< 控制跟踪帧序号，用于识别上行丢帧。
    uint32_t targetToken;              ///< 本周期实际读取的关节目标代次。
    uint32_t targetAcceptedTimestamp;  ///< P4 完整接收该目标帧的时间。
    uint32_t encoderTimestamp;         ///< 本周期角度所用磁编快照的采样时间。
    uint32_t activeJointMask;          ///< 本周期真正参与 PID 的关节位图。
    uint32_t validJointMask;           ///< actualDegCenti 中有效关节位图。
    int16_t targetDegCenti[ENCODER_TOTAL_NUM]; ///< PID 实际采用的目标角，0.01 degree。
    int16_t actualDegCenti[ENCODER_TOTAL_NUM]; ///< 同周期磁编反馈角，0.01 degree。
    uint32_t timestamp;                ///< 当前控制周期的 P4 单调毫秒时间。
};

#endif // MMHAND_CONTROL_TRACE_TYPES_H
