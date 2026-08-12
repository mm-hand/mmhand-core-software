/**
 * @file CalibrationDiagnostics.cpp
 * @brief 实现自动标定诊断快照和统一安全事件登记。
 * @details 该模块只保存“发生了什么”，不决定标定状态机如何恢复。
 */
#include "CalibrationDiagnostics.h"

#include <string.h>

#include "../../data/CalibrationTypes.h"
#include "../../platform/rtos/AtomicValue.h"
#include "../../platform/rtos/SharedSafetyState.h"
#include "../../platform/time/Clock.h"
#include "../../safety/SafetyEventReporter.h"
#include "../../safety/SafetyRuleRegistry.h"
#include "../../runtime/SystemRuntimeContext.h"

void setCalibrationDiag(SystemRuntimeContext* sharedData,
                        uint8_t reason,
                        uint8_t phase,
                        uint8_t joint,
                        uint8_t motor,
                        int32_t valueA,
                        int32_t valueB,
                        int32_t valueC,
                        uint32_t bitmap)
{
    if (!sharedData || reason == CALIB_DIAG_REASON_NONE) {
        return;
    }

    CalibrationDiagData_t diag = {};
    diag.valid = 1;
    diag.reason = reason;
    diag.phase = phase;
    diag.joint = joint;
    diag.motor = motor;
    diag.valueA = valueA;
    diag.valueB = valueB;
    diag.valueC = valueC;
    diag.bitmap = bitmap;
    diag.timestamp = p4::platform::time::monotonicMillis();
    {
        // sequence 与快照必须在同一临界区更新，上位机才能识别完整的新诊断。
        SharedSafetyGuard guard;
        diag.sequence = sharedData->calibration_diag.sequence + 1;
        sharedData->calibration_diag = diag;
    }

    const SafetyRuleId ruleId =
        p4::safety::ruleForCalibrationDiagReason(reason);
    if (ruleId == SafetyRuleId::None) {
        return;
    }
    p4::safety::reportSafetyRuleEvent(sharedData,
                                      ruleId,
                                      valueA,
                                      valueB,
                                      joint,
                                      motor);
}

void clearCalibrationDiag(SystemRuntimeContext* sharedData)
{
    if (!sharedData) {
        return;
    }
    SharedSafetyGuard guard;
    const uint32_t nextSequence = sharedData->calibration_diag.sequence + 1;
    memset(&sharedData->calibration_diag, 0,
           sizeof(sharedData->calibration_diag));
    sharedData->calibration_diag.sequence = nextSequence;
}

void setCalibrationDiagIfEmpty(SystemRuntimeContext* sharedData,
                               uint8_t reason,
                               uint8_t phase,
                               uint8_t joint,
                               uint8_t motor,
                               int32_t valueA,
                               int32_t valueB,
                               int32_t valueC,
                               uint32_t bitmap)
{
    if (hasCalibrationDiag(sharedData)) {
        return;
    }
    setCalibrationDiag(sharedData,
                       reason,
                       phase,
                       joint,
                       motor,
                       valueA,
                       valueB,
                       valueC,
                       bitmap);
}

bool hasCalibrationDiag(SystemRuntimeContext* sharedData)
{
    if (!sharedData) {
        return false;
    }
    SharedSafetyGuard guard;
    return sharedData->calibration_diag.valid != 0;
}

uint8_t calibrationDiagReason(SystemRuntimeContext* sharedData)
{
    if (!sharedData) {
        return CALIB_DIAG_REASON_NONE;
    }
    SharedSafetyGuard guard;
    return sharedData->calibration_diag.valid
        ? sharedData->calibration_diag.reason
        : CALIB_DIAG_REASON_NONE;
}

void setCalibrationDiagResultMasks(SystemRuntimeContext* sharedData,
                                   uint32_t completedJointMask,
                                   uint32_t failedJointMask)
{
    if (!sharedData) {
        return;
    }
    SharedSafetyGuard guard;
    sharedData->calibration_diag.completedJointMask = completedJointMask;
    sharedData->calibration_diag.failedJointMask = failedJointMask;
    sharedData->calibration_diag.sequence++;
}
