/**
 * @file CalibrationCoordinatorPort.h
 * @brief 声明标定会话控制器调用机械协调器所需的最小内部端口。
 *
 * 本文件不是 UI/协议接口。它只隔离“会话预检”与“机械状态机”，避免任务入口
 * 反向访问协调器私有变量；后续单/双电机状态机迁出时继续收窄此端口。
 */
#ifndef P4_MODULES_CALIBRATION_CALIBRATION_COORDINATOR_PORT_H
#define P4_MODULES_CALIBRATION_CALIBRATION_COORDINATOR_PORT_H

#include <stdint.h>

#include "../../runtime/SystemRuntimeContext.h"

bool waitForCalibrationEncoderRecovery(SystemRuntimeContext* sharedData,
                                       uint32_t requiredJointMask);
bool waitForCalibrationServoRecovery(SystemRuntimeContext* sharedData,
                                     uint32_t requiredMotorMask);
bool isWorkspaceDualJoint(uint8_t jointIndex);
bool buildDynamicWorkspaceMotorMask(SystemRuntimeContext* sharedData,
                                    const uint8_t* selectedJoints,
                                    uint8_t selectedJointCount,
                                    uint32_t* workspaceMotorMask);
void setCalibrationWorkspaceReadMask(SystemRuntimeContext* sharedData,
                                     uint32_t motorMask);

#endif // P4_MODULES_CALIBRATION_CALIBRATION_COORDINATOR_PORT_H
