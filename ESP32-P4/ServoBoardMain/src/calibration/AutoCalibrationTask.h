#ifndef AUTO_CALIBRATION_TASK_H
#define AUTO_CALIBRATION_TASK_H

#include <Arduino.h>
#include "../shared/TaskSharedData.h"

// 自动标定的统一运动与判定参数。
struct AutoCalibrationParams {
    int32_t tightenSpeedCountsPerSec; // P4 每秒推进的舵机 ABS counts。
    uint16_t servoSpeed;              // 写入飞特舵机命令的速度参数。
    uint8_t servoAcc;                 // 写入飞特舵机命令的加速度参数。
    uint32_t loopMs;                  // 标定状态机调度周期。
    uint32_t windowMs;                // 机械停稳/行程判断窗口。
    uint32_t traceMs;                 // 低频诊断日志周期。
    int32_t windowServoDeltaCounts;   // 窗口内舵机 ABS 最小有效变化。
    int32_t zeroReachAbsTolerance;    // 目标与实际舵机 ABS 的到位容差。
    float stallActualDeg;             // 窗口内磁编角变化小于该值视为机械停稳。
    int16_t startLoadDelta;            // 相对基线负载的起始变化阈值。
    int16_t tightLoadHigh;             // 单电机紧端负载阈值。
    uint8_t startLoadStableSamples;    // 负载阈值需连续满足的样本数。
};

// 默认全手标定顺序；实际任务会再与 active_joint_mask 求交集。
extern const uint8_t kDefaultAutoCalibrationJoints[];
extern const uint8_t kDefaultAutoCalibrationJointsCount;

// 返回当前固件默认的运动和判定参数。
AutoCalibrationParams defaultAutoCalibrationParams(void);

// 标定单个关节的同步包装接口。
bool runAutoCalibrationJoint(TaskSharedData_t* sharedData,
                             uint8_t jointIndex,
                             const AutoCalibrationParams& params);

// 按机械安全顺序串行标定指定关节列表，并保留已成功关节结果。
bool runAutoCalibrationSequence(TaskSharedData_t* sharedData,
                                const uint8_t* jointList,
                                uint8_t jointCount,
                                const AutoCalibrationParams& params);

// 创建自动标定 FreeRTOS 任务；重复启动会复用当前任务。
bool startAutoCalibrationTask(TaskSharedData_t* sharedData);
// FreeRTOS 任务入口，仅供 startAutoCalibrationTask 使用。
void autoCalibrationTask(void* parameter);

#endif // AUTO_CALIBRATION_TASK_H
