/**
 * @file CalibrationResultCommitter.h
 * @brief 声明单关节标定结果的原子提交入口。
 * @details
 * 结果、装配告警和控制目标必须按固定顺序提交，避免通信任务读到半成品。
 * 本模块不计算机械端点、不生成 LUT，也不修改舵机多圈可信状态。
 */
#ifndef P4_MODULES_CALIBRATION_CALIBRATION_RESULT_COMMITTER_H
#define P4_MODULES_CALIBRATION_CALIBRATION_RESULT_COMMITTER_H

#include <stdint.h>

#include "../../modules/calibration/CalibrationRuntimeStore.h"

struct SystemRuntimeContext;

/** 保存 Jxx 完整结果，并把控制目标同步到标定结束时的实测角。 */
bool commitCalibrationResult(SystemRuntimeContext* sharedData,
                             uint8_t jointIndex,
                             const JointCalibrationResult& result,
                             float finalTargetDeg);

#endif // P4_MODULES_CALIBRATION_CALIBRATION_RESULT_COMMITTER_H
