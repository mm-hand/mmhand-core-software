/**
 * @file SingleMotorCalibrationMachine.h
 * @brief 声明单电机关节自动标定显式状态机。
 */
#ifndef P4_MODULES_CALIBRATION_SINGLE_MOTOR_CALIBRATION_MACHINE_H
#define P4_MODULES_CALIBRATION_SINGLE_MOTOR_CALIBRATION_MACHINE_H

#include "../AutoCalibrationCoordinator.h"

enum SingleMotorCalibrationRunResult : uint8_t {
    SINGLE_MOTOR_CALIBRATION_COMPLETED = 0,
    SINGLE_MOTOR_CALIBRATION_FAILED,
    SINGLE_MOTOR_CALIBRATION_ABORTED,
};

/** 串行运行一个单电机关节；非活动关节在每个周期保持当前 ABS。 */
SingleMotorCalibrationRunResult runSingleMotorCalibrationMachine(
    SystemRuntimeContext* sharedData,
    uint8_t jointIndex,
    const uint8_t* orderedJoints,
    uint8_t jointCount,
    uint32_t failedJointMask,
    uint8_t sequenceIndex,
    uint32_t selectedJointMask,
    const AutoCalibrationParams& params);

#endif // P4_MODULES_CALIBRATION_SINGLE_MOTOR_CALIBRATION_MACHINE_H
