/**
 * @file ControlTraceTypes.h
 * @brief 定义使用同一 P4 单调时钟采集的关节控制跟踪快照。
 * @details 订阅时覆盖最新快照，HostComm 以固定周期采样；不得反向阻塞控制周期。
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
    uint32_t motorCommandMask;         ///< 本周期成功发布到 ServoBus 邮箱的 Mxx 位图。
    int16_t targetDegCenti[ENCODER_TOTAL_NUM]; ///< PID 实际采用的目标角，0.01 degree。
    int16_t actualDegCenti[ENCODER_TOTAL_NUM]; ///< 同周期磁编反馈角，0.01 degree。
    int32_t motorCommandAbs[SERVO_TOTAL_NUM];  ///< 最终舵机 logical ABS CMD，单位 counts。
    uint32_t timestamp;                ///< 当前控制周期的 P4 单调毫秒时间。
    uint32_t servoTimestamp;
    uint32_t dtMicros;
    uint32_t servoValidMask;
    uint32_t recoveryAppliedMask;
    uint32_t droppedFrames;
    int32_t motorActualAbs[SERVO_TOTAL_NUM];
    int16_t pidCounts[SERVO_TOTAL_NUM]; ///< 四舍五入并饱和到 int16 的双腱 PID 修正。
    int16_t recoveryCounts[SERVO_TOTAL_NUM]; ///< 实际叠加的整数恢复量。
};

#endif // MMHAND_CONTROL_TRACE_TYPES_H
