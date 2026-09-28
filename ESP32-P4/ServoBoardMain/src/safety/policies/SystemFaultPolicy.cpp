/**
 * @file SystemFaultAggregator.cpp
 * @brief 汇总系统当前故障，并按运动态计算阻断故障。
 *
 * 输入：CAN/磁编、舵机反馈、标定与多圈原子状态。
 * 输出：无副作用 fault bitmap；禁止在这里执行 E-STOP、owner 释放或状态迁移。
 */
#include "SystemFaultPolicy.h"

#include "SystemReadinessPolicy.h"
#include "../../config/SafetyConfig.h"
#include "../../data/SafetyTypes.h"
#include "../../platform/rtos/SharedSafetyState.h"
#include "../../platform/time/Clock.h"

using namespace mmhand::config::safety;
using p4::platform::time::monotonicMillis;

namespace p4 {
namespace safety {

uint32_t collectSystemFaults(SystemRuntimeContext* sharedData)
{
    if (!sharedData) {
        return 0;
    }

    const uint32_t nowMs = monotonicMillis();
    uint32_t faults = 0;
    const uint32_t activeJointMask =
        sharedAtomicLoad(&sharedData->active_joint_mask) &
        ((1UL << ENCODER_TOTAL_NUM) - 1UL);
    const uint32_t encoderOnlineMask =
        sharedAtomicLoad(&sharedData->encoder_online_bitmap);
    const uint32_t encoderFaultMask =
        sharedAtomicLoad(&sharedData->encoder_fault_bitmap);
    const uint8_t canLinkOnline =
        sharedAtomicLoad(&sharedData->can_link_online);

    RemoteSensorData_t sensorData;
    const bool motionRequiresCan =
        p4::state::requiresCan(sharedData) ||
        sharedAtomicLoad(&sharedData->system_state) ==
            SYSTEM_STATE_ROPE_INIT_RUNNING;
    if (motionRequiresCan &&
        (!sharedData->canRxQueue.peek(&sensorData) ||
         !sensorData.isValid ||
         (uint32_t)(nowMs - sensorData.timestamp) >
             kStateCanOfflineTimeoutMs ||
         canLinkOnline == 0 ||
         (activeJointMask & ~encoderOnlineMask) != 0 ||
         (activeJointMask & encoderFaultMask) != 0)) {
        faults |= SYSTEM_FAULT_CAN_OFFLINE;
    }

    ServoAngleData_t servoSnapshot;
    const bool servoFresh =
        sharedData->servoFeedbackQueue.peek(&servoSnapshot) &&
        (uint32_t)(nowMs - servoSnapshot.timestamp) <=
            kStateServoFeedbackTimeoutMs;
    if (!servoFresh ||
        !p4::state::requiredServosOnline(sharedData, servoSnapshot)) {
        faults |= SYSTEM_FAULT_SERVO_OFFLINE;
    }
    if (sharedAtomicLoad(&sharedData->overload_fault_bitmap) != 0) {
        faults |= SYSTEM_FAULT_OVERLOAD;
    }
    if (sharedAtomicLoad(&sharedData->calibration_fault_latched) != 0) {
        faults |= SYSTEM_FAULT_CALIBRATION;
    }
    if ((sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap) &
         p4::state::requiredMotorMask(sharedData)) != 0) {
        faults |= SYSTEM_FAULT_MULTITURN_UNKNOWN;
    }
    return faults |
           sharedAtomicLoad(&sharedData->system_fault_latched_bitmap);
}

uint32_t blockingFaultsForState(SystemRuntimeContext* sharedData,
                                uint8_t state,
                                uint32_t faults)
{
    switch (state) {
        case SYSTEM_STATE_RUN_DIRECT:
            return faults & ~SYSTEM_FAULT_CAN_OFFLINE;

        case SYSTEM_STATE_RUN_FEEDBACK:
        case SYSTEM_STATE_ROPE_INIT_RUNNING:
            return faults;

        case SYSTEM_STATE_CALIBRATION_RUNNING:
            // 标定会重建多圈；预检阶段还允许通信任务完成舵机/CAN 恢复。
            if (sharedData &&
                sharedAtomicLoad(&sharedData->calibration_motion_started) == 0) {
                return faults &
                       ~(SYSTEM_FAULT_CAN_OFFLINE |
                         SYSTEM_FAULT_MULTITURN_UNKNOWN |
                         SYSTEM_FAULT_SERVO_OFFLINE);
            }
            return faults & ~SYSTEM_FAULT_MULTITURN_UNKNOWN;

        default:
            return 0;
    }
}

} // namespace safety
} // namespace p4
