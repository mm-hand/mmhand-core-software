#ifndef JOINT_CALIBRATION_PROFILE_H
#define JOINT_CALIBRATION_PROFILE_H

#include <Arduino.h>
#include "CalibrationTask.h"

// JointCalibrationProfile 头文件职责：
// 1) 暴露只读标定参数总表；
// 2) 提供按数量拷贝 profile 的辅助接口；
// 3) 保证标定参数索引与 jointIndex 一致。

// 关节标定参数总表（只读）：
// - 下标与 jointIndex 一一对应；
// - 每项的字段语义见 JointCalibrationConfig；
// - 表内容由本模块维护，调用方按索引读取，不应在运行时修改。
extern const JointCalibrationConfig kJointCalibrationProfile[ENCODER_TOTAL_NUM];

// 将标定参数表拷贝到调用方缓存：
// - dst 为空时直接返回；
// - count 会截断到 ENCODER_TOTAL_NUM；
// - 按索引顺序拷贝 [0, count) 项。
void loadJointCalibrationProfile(JointCalibrationConfig* dst, uint8_t count);

#endif // JOINT_CALIBRATION_PROFILE_H
