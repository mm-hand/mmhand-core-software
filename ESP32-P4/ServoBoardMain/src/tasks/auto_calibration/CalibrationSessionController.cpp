/**
 * @file CalibrationSessionController.cpp
 * @brief 编排自动标定会话的 scope、反馈预检、机械序列和完成事件。
 *
 * 输入：SystemRuntimeContext 中的 active joint/motor scope 和实时反馈。
 * 输出：calibration_motion_started、诊断快照及 CALIBRATION_DONE/FAILED 事件。
 * 禁止依赖：飞特 SDK、串口协议组包、持久化原始 API 和 FreeRTOS 任务创建。
 */
#include "CalibrationSessionController.h"

#include <stdio.h>

#include "AutoCalibrationCoordinator.h"
#include "CalibrationCoordinatorPort.h"
#include "CalibrationDiagnostics.h"
#include "CalibrationSequencePlanner.h"
#include "../../modules/runtime/ServiceCyclePolicy.h"
#include "../../config/topology/HardwareMap.h"
#include "../../platform/rtos/SharedSafetyState.h"
#include "../../platform/log/FirmwareLog.h"
#include "../../runtime/SystemRuntimeContext.h"
#include "../../runtime/ports/SystemEventPort.h"

namespace {

/** 输出本轮关节顺序；固定缓冲区避免在实时任务中使用动态内存。 */
void logActiveScope(const uint8_t* joints, uint8_t jointCount)
{
    char line[128] = {0};
    size_t used = (size_t)snprintf(line, sizeof(line),
                                   "[AUTO CALIB] ACTIVE_SCOPE joints=");
    if (used >= sizeof(line)) {
        used = sizeof(line) - 1;
    }
    for (uint8_t i = 0; i < jointCount && used < sizeof(line) - 1; ++i) {
        const int written = snprintf(line + used,
                                     sizeof(line) - used,
                                     "%sJ%02u",
                                     i == 0 ? "" : ",",
                                     (unsigned)joints[i]);
        if (written <= 0) {
            break;
        }
        used += (size_t)written;
        if (used >= sizeof(line)) {
            used = sizeof(line) - 1;
            break;
        }
    }
    line[used] = '\0';
    FW_LOG_INFO_PRINTLN(line);
}

/** 判断本轮是否需要四指侧摆工作空间辅助电机。 */
bool requiresWorkspaceHelpers(const uint8_t* joints, uint8_t jointCount)
{
    for (uint8_t i = 0; i < jointCount; ++i) {
        if (isWorkspaceDualJoint(joints[i])) {
            return true;
        }
    }
    return false;
}

} // namespace

void runAutoCalibrationSession(SystemRuntimeContext* sharedData,
                               const CalibrationServiceInputSnapshot& input,
                               const ServiceOutputPort& output)
{
    const AutoCalibrationParams params = defaultAutoCalibrationParams();
    if (sharedData) {
        // 预检阶段没有机械命令；通信任务仍可完成热插拔恢复和路由刷新。
        sharedAtomicStore(&sharedData->calibration_motion_started, (uint8_t)0);
        sharedAtomicStore(&sharedData->calibration_completed_joint_bitmap,
                          (uint32_t)0);
    }

    uint8_t activeJoints[ENCODER_TOTAL_NUM] = {0};
    const uint8_t jointCount = calibrationSnapshotHasSelection(input)
        ? buildAutoCalibrationJointListFromActiveScope(
              sharedData, activeJoints, ENCODER_TOTAL_NUM)
        : 0;
    if (jointCount == 0) {
        FW_LOG_INFO_PRINTLN("[AUTO CALIB] FAIL active scope has no joints");
        setCalibrationDiag(
            sharedData,
            CALIB_DIAG_REASON_ACTIVE_SCOPE_EMPTY,
            CALIB_DIAG_PHASE_SEQUENCE,
            0xFF,
            0xFF,
            sharedData
                ? (int32_t)sharedAtomicLoad(&sharedData->active_joint_mask) : 0,
            sharedData
                ? (int32_t)sharedAtomicLoad(&sharedData->active_motor_mask) : 0,
            0,
            0);
    } else {
        logActiveScope(activeJoints, jointCount);
    }

    const uint32_t selectedMotorMask =
        buildCalibrationMotorMask(activeJoints, jointCount);
    const uint32_t selectedJointMask =
        buildCalibrationJointMask(activeJoints, jointCount);
    const bool selectedServoReady =
        jointCount > 0 && selectedMotorMask != 0 &&
        waitForCalibrationServoRecovery(sharedData, selectedMotorMask);
    const bool selectedEncoderReady =
        selectedServoReady && selectedJointMask != 0 &&
        waitForCalibrationEncoderRecovery(sharedData, selectedJointMask);

    uint32_t workspaceMotorMask = 0;
    const bool workspaceReady =
        selectedEncoderReady &&
        (!requiresWorkspaceHelpers(activeJoints, jointCount) ||
         buildDynamicWorkspaceMotorMask(sharedData,
                                        activeJoints,
                                        jointCount,
                                        &workspaceMotorMask));
    setCalibrationWorkspaceReadMask(sharedData,
                                    workspaceReady ? workspaceMotorMask : 0);

    const uint32_t recoveryMask =
        (selectedMotorMask | workspaceMotorMask) & validMotorChannelMask();
    const bool allFeedbackReady =
        workspaceReady &&
        waitForCalibrationServoRecovery(sharedData, recoveryMask);
    if (allFeedbackReady && sharedData) {
        // 只有预检全部完成后才宣布机械运动开始；掉线从此恢复为运动故障。
        sharedAtomicStore(&sharedData->calibration_motion_started, (uint8_t)1);
    }

    const bool ok = jointCount > 0 && allFeedbackReady &&
                    runAutoCalibrationSequence(sharedData,
                                               activeJoints,
                                               jointCount,
                                               params);
    if (sharedData) {
        sharedAtomicStore(&sharedData->calibration_motion_started, (uint8_t)0);
    }
    setCalibrationWorkspaceReadMask(sharedData, 0);

    if (!ok && sharedData) {
        setCalibrationDiagIfEmpty(sharedData,
                                  CALIB_DIAG_REASON_JOINT_STEP_FAILED,
                                  CALIB_DIAG_PHASE_SEQUENCE,
                                  0xFF,
                                  0xFF,
                                  0,
                                  0,
                                  0,
                                  0);
    }
    servicePostEvent(output,
                     ok ? SYSTEM_EVENT_CALIBRATION_DONE
                        : SYSTEM_EVENT_CALIBRATION_FAILED);
}
