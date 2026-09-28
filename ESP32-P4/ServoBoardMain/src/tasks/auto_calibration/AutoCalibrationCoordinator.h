/**
 * @file AutoCalibrationCoordinator.h
 * @brief 声明自动标定业务编排器的运行入口。
 * @details Task 层只负责 FreeRTOS 入口，单/双腱流程、工作空间和结果提交在本模块推进。
 */
#ifndef P4_MODULES_CALIBRATION_AUTO_CALIBRATION_COORDINATOR_H
#define P4_MODULES_CALIBRATION_AUTO_CALIBRATION_COORDINATOR_H

#include "../../runtime/SystemRuntimeContext.h"
#include "CalibrationSequencePlanner.h"

struct AutoCalibrationParams {
    int32_t tightenSpeedCountsPerSec; ///< P4 每秒推进舵机 ABS counts。
    uint16_t servoSpeed; ///< 飞特舵机速度参数。
    uint8_t servoAcc; ///< 飞特舵机加速度参数。
    uint32_t loopMs; ///< 标定子状态机调度周期。
    uint32_t windowMs; ///< 机械停稳/行程判断窗口。
    uint32_t traceMs; ///< 低频诊断日志周期。
    int32_t windowServoDeltaCounts; ///< 窗口内舵机 ABS 最小有效变化。
    int32_t zeroReachAbsTolerance; ///< 目标与实际 ABS 到位容差。
    float stallActualDeg; ///< 窗口内磁编角变化低于该值视为停稳。
    int16_t tightLoadHigh; ///< 单电机紧端负载阈值。
};

AutoCalibrationParams defaultAutoCalibrationParams(void);
bool runAutoCalibrationSequence(SystemRuntimeContext* sharedData,
                                const uint8_t* jointList,
                                uint8_t jointCount,
                                const AutoCalibrationParams& params);
#endif // P4_MODULES_CALIBRATION_AUTO_CALIBRATION_COORDINATOR_H
