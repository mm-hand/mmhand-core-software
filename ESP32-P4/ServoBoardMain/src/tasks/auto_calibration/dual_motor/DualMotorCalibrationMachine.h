/**
 * @file DualMotorCalibrationMachine.h
 * @brief 声明双腱关节及四指工作空间准备状态机。
 */
#ifndef P4_MODULES_CALIBRATION_DUAL_MOTOR_CALIBRATION_MACHINE_H
#define P4_MODULES_CALIBRATION_DUAL_MOTOR_CALIBRATION_MACHINE_H

#include "../AutoCalibrationCoordinator.h"

enum DualMotorCalibrationRunResult : uint8_t {
    DUAL_MOTOR_CALIBRATION_COMPLETED = 0,
    DUAL_MOTOR_CALIBRATION_FAILED,
    DUAL_MOTOR_CALIBRATION_PREFLIGHT_FAILED,
};

/** 清空仅在本轮序列内可信的 A/B 工作空间姿态缓存。 */
void resetDualWorkspaceCalibrationCache();

/** 完成工作空间准备、双电机机械端搜索、LUT/限位提交。 */
DualMotorCalibrationRunResult runDualMotorCalibrationMachine(
    SystemRuntimeContext* sharedData,
    uint8_t jointIndex,
    uint8_t sequenceIndex,
    uint8_t jointCount,
    uint32_t selectedJointMask,
    const AutoCalibrationParams& params);

#endif // P4_MODULES_CALIBRATION_DUAL_MOTOR_CALIBRATION_MACHINE_H
