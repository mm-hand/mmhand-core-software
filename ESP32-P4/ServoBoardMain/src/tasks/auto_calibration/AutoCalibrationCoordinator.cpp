/**
 * @file AutoCalibrationCoordinator.cpp
 * @brief 按机械顺序串行调度单/双电机关节标定，并汇总每关节结果。
 */
#include "AutoCalibrationCoordinator.h"
#include "CalibrationDiagnostics.h"
#include "CalibrationMotionSupport.h"
#include "CalibrationSequencePlanner.h"
#include "dual_motor/DualMotorCalibrationMachine.h"
#include "single_motor/SingleMotorCalibrationMachine.h"
#include "../../platform/rtos/AtomicValue.h"
#include "../../platform/log/FirmwareLog.h"
#include "../../safety/policies/CalibrationSafetyPolicy.h"

// 构造默认标定参数；所有单/双电机状态机共享这组周期和停稳判据。
AutoCalibrationParams defaultAutoCalibrationParams(void)
{
    AutoCalibrationParams params;
    params.tightenSpeedCountsPerSec = 400;
    params.servoSpeed = 400;
    params.servoAcc = 20;
    params.loopMs = 20;
    params.windowMs = 200;
    params.traceMs = 200;
    params.windowServoDeltaCounts = 50;
    params.zeroReachAbsTolerance = 200;
    params.stallActualDeg = 0.75f;
    params.tightLoadHigh = 250;
    return params;
}

// 自动标定总调度：
// 1. 校验并按机械规则重排关节；
// 2. 双腱关节执行工作空间规划和专用流程；
// 3. 单电机关节逐帧运行非阻塞状态机；
// 4. 成功结果逐关节保留，失败位图用于下次只重标失败关节。
bool runAutoCalibrationSequence(SystemRuntimeContext* sharedData,
                                const uint8_t* jointList,
                                uint8_t jointCount,
                                const AutoCalibrationParams& params)
{
    const p4::safety::CalibrationSequenceDecision sequenceDecision =
        p4::safety::validateCalibrationSequence(
            sharedData != nullptr,
            jointList,
            jointCount,
            ENCODER_TOTAL_NUM);
    if (!sequenceDecision.allowed &&
        sequenceDecision.reason !=
            p4::safety::CalibrationSequenceReject::DuplicateJoint &&
        sequenceDecision.reason !=
            p4::safety::CalibrationSequenceReject::JointOutOfRange) {
        setCalibrationDiag(sharedData,
                           CALIB_DIAG_REASON_JOINT_LIST_INVALID,
                           CALIB_DIAG_PHASE_SEQUENCE,
                           0xFF,
                           0xFF,
                           jointCount,
                           ENCODER_TOTAL_NUM,
                           jointList ? 1 : 0,
                           0);
        return false;
    }

    resetDualWorkspaceCalibrationCache();

    uint32_t seenJoints = 0;
    uint8_t orderedJoints[ENCODER_TOTAL_NUM] = {0};
    for (uint8_t i = 0; i < jointCount; i++) {
        if (!sequenceDecision.allowed &&
            sequenceDecision.itemIndex == i &&
            sequenceDecision.reason ==
                p4::safety::CalibrationSequenceReject::JointOutOfRange) {
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_JOINT_LIST_INVALID,
                               CALIB_DIAG_PHASE_SEQUENCE,
                               jointList[i],
                               0xFF,
                               jointList[i],
                               ENCODER_TOTAL_NUM,
                               i,
                               seenJoints);
            return false;
        }
        const uint32_t bit = (1UL << jointList[i]);
        if (!sequenceDecision.allowed &&
            sequenceDecision.itemIndex == i &&
            sequenceDecision.reason ==
                p4::safety::CalibrationSequenceReject::DuplicateJoint) {
            FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL duplicate joint J%02u in sequence\r\n",
                          jointList[i]);
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_DUPLICATE_JOINT,
                               CALIB_DIAG_PHASE_SEQUENCE,
                               jointList[i],
                               0xFF,
                               i,
                               jointCount,
                               0,
                               seenJoints);
            return false;
        }
        seenJoints |= bit;
        orderedJoints[i] = jointList[i];
    }

    for (uint8_t i = 0; i < jointCount; i++) {
        for (uint8_t j = (uint8_t)(i + 1); j < jointCount; j++) {
            if (autoCalibrationSequenceRank(orderedJoints[j]) <
                autoCalibrationSequenceRank(orderedJoints[i])) {
                const uint8_t tmp = orderedJoints[i];
                orderedJoints[i] = orderedJoints[j];
                orderedJoints[j] = tmp;
            }
        }
    }

    // 标定刻意采用串行方式：收紧一根肌腱会改变相邻关节的机械负载和磁编角度，
    // 并行标定可能使检测到的零点/紧端产生偏差。单关节内部仍使用非阻塞状态机，
    // 但必须等当前关节完整结束后再启动下一个关节。
    uint32_t completedJointMask = 0;
    uint32_t failedJointMask = 0;
    uint8_t firstFailedJoint = 0xFF;

    for (uint8_t i = 0; i < jointCount; ++i) {
        const uint8_t jointIndex = orderedJoints[i];
        const uint32_t jointBit = (1UL << jointIndex);
        uint8_t primaryMotor = 0;
        uint8_t secondaryMotor = 0;

        // 双腱归属只查询 HardwareMap 的统一入口，避免标定状态机维护第二份判断表。
        if (getDualCalibrationMotors(jointIndex,
                                     &primaryMotor,
                                     &secondaryMotor)) {
            const DualMotorCalibrationRunResult result =
                runDualMotorCalibrationMachine(sharedData,
                                               jointIndex,
                                               i,
                                               jointCount,
                                               seenJoints,
                                               params);
            if (result == DUAL_MOTOR_CALIBRATION_PREFLIGHT_FAILED) {
                failedJointMask |= jointBit;
                if (firstFailedJoint == 0xFF) firstFailedJoint = jointIndex;
                break;
            }
            if (result == DUAL_MOTOR_CALIBRATION_FAILED) {
                failedJointMask |= jointBit;
                if (firstFailedJoint == 0xFF) firstFailedJoint = jointIndex;
                continue;
            }
            completedJointMask |= jointBit;
            continue;
        }

        const SingleMotorCalibrationRunResult result =
            runSingleMotorCalibrationMachine(sharedData,
                                             jointIndex,
                                             orderedJoints,
                                             jointCount,
                                             failedJointMask,
                                             i,
                                             seenJoints,
                                             params);
        if (result == SINGLE_MOTOR_CALIBRATION_ABORTED) {
            failedJointMask |= jointBit;
            if (firstFailedJoint == 0xFF) firstFailedJoint = jointIndex;
            break;
        }
        if (result == SINGLE_MOTOR_CALIBRATION_FAILED) {
            failedJointMask |= jointBit;
            if (firstFailedJoint == 0xFF) firstFailedJoint = jointIndex;
            continue;
        }
        completedJointMask |= jointBit;
    }
    if (failedJointMask != 0) {
        FW_LOG_INFO_PRINTF("[AUTO CALIB] SEQUENCE_SERIAL partial failed=0x%08lX completed=0x%08lX selected=0x%08lX\r\n",
                      (unsigned long)failedJointMask,
                      (unsigned long)completedJointMask,
                      (unsigned long)seenJoints);
        // 具体状态机已经写入真实根因时，调度层不得再用“部分关节失败”覆盖它。
        // 成功/失败汇总改用独立字段，供上位机合并本轮局部标定快照。
        if (!hasCalibrationDiag(sharedData)) {
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_JOINT_STEP_FAILED,
                               CALIB_DIAG_PHASE_SEQUENCE,
                               firstFailedJoint,
                               0xFF,
                               0,
                               0,
                               jointCount,
                               failedJointMask);
        }
        setCalibrationDiagResultMasks(sharedData,
                                      completedJointMask,
                                      failedJointMask);
        return false;
    }

    return true;
}
