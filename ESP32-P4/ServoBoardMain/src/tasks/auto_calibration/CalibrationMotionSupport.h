/**
 * @file CalibrationMotionSupport.h
 * @brief 声明单/双电机自动标定共享的反馈、目标发布、范围和多圈提交原语。
 * @details 这里只提供机械无关原语，不保存标定子状态，不决定关节执行顺序。
 */
#ifndef P4_MODULES_CALIBRATION_CALIBRATION_MOTION_SUPPORT_H
#define P4_MODULES_CALIBRATION_CALIBRATION_MOTION_SUPPORT_H

#include "AutoCalibrationCoordinator.h"

// 本文件只通过 const 引用读取标定结果，前置声明可避免引入完整运行时存储实现。
struct JointCalibrationResult;

int32_t calibrationAbs32(int32_t value);
int32_t singleTendonReleaseCounts(uint8_t motorChannel,
                                  int32_t tightAbs,
                                  int32_t currentAbs);
int32_t clampCalibrationServoAbs(int32_t value);
void setCalibratedMotorAbsLimit(SystemRuntimeContext* sharedData,
                                uint8_t motorChannel,
                                int32_t tightAbs);
void setSingleTendonLutAbsLimit(SystemRuntimeContext* sharedData,
                               uint8_t motorChannel,
                               const JointCalibrationResult& calibration);
void clearCalibratedMotorAbsLimitsForJoint(SystemRuntimeContext* sharedData,
                                           uint8_t jointIndex);
int32_t servoTurnFromAbs(int32_t absValue);
bool autoCalibrationStillOwnsControl(SystemRuntimeContext* sharedData);
bool getServoLogicalCommandRange(SystemRuntimeContext* sharedData,
                                 uint8_t motorChannel,
                                 int32_t* outMin,
                                 int32_t* outMax);
bool isServoCommandInsideLimit(SystemRuntimeContext* sharedData,
                               uint8_t motorChannel,
                               int32_t targetAbs);
bool isJointPrimaryServoMultiTurnKnown(SystemRuntimeContext* sharedData,
                                       uint8_t jointIndex);
bool waitForCalibrationEncoderRecovery(SystemRuntimeContext* sharedData,
                                       uint32_t selectedJointMask);
bool waitForCalibrationServoRecovery(SystemRuntimeContext* sharedData,
                                     uint32_t selectedMotorMask);
bool commitJointServoMultiTurnTrust(SystemRuntimeContext* sharedData,
                                    uint8_t jointIndex);
bool appendCalibrationServoTarget(ServoTargetBatch_t* batch,
                                  uint8_t motorChannel,
                                  int32_t position,
                                  uint16_t speed,
                                  uint8_t acc);
bool appendJointCalibrationTarget(ServoTargetBatch_t* batch,
                                  uint8_t jointIndex,
                                  int32_t primaryTarget,
                                  uint16_t speed,
                                  uint8_t acc);
bool publishCalibrationBatch(SystemRuntimeContext* sharedData,
                             ServoTargetBatch_t* batch);
bool appendCalibrationHoldTargetsExcept(SystemRuntimeContext* sharedData,
                                        ServoTargetBatch_t* batch,
                                        const uint8_t* jointList,
                                        uint8_t jointCount,
                                        uint8_t activeJoint,
                                        uint32_t skipJointMask,
                                        const AutoCalibrationParams& params);
bool getDualCalibrationMotors(uint8_t jointIndex,
                              uint8_t* primaryMotor,
                              uint8_t* secondaryMotor);
bool publishMotorCalibrationTarget(SystemRuntimeContext* sharedData,
                                   uint8_t motorChannel,
                                   int32_t targetAbs,
                                   const AutoCalibrationParams& params);
bool publishMotorCalibrationTargetDirect(SystemRuntimeContext* sharedData,
                                         uint8_t motorChannel,
                                         int32_t targetAbs,
                                         const AutoCalibrationParams& params);
bool publishDualMotorCalibrationTargets(SystemRuntimeContext* sharedData,
                                        uint8_t movingMotor,
                                        int32_t movingTargetAbs,
                                        uint8_t holdMotor,
                                        int32_t holdTargetAbs,
                                        const AutoCalibrationParams& params);

#endif // P4_MODULES_CALIBRATION_CALIBRATION_MOTION_SUPPORT_H
