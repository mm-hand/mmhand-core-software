/**
 * @file ServoFeedbackPublisher.cpp
 * @brief 实现从驱动缓存到共享反馈快照的转换及恢复/越界状态发布。
 */
#include "ServoFeedbackPublisher.h"

#include <string.h>

#include "../../actuator/ActuatorCommandGateway.h"
#include "../../hardware/HardwareMap.h"
#include "../../hardware/servo/FeetechServoDriver.h"
#include "../../hardware/servo/ServoBusRegistry.h"
#include "../../shared/SharedSafetyState.h"
#include "../../system/FirmwareLog.h"
#include "../../system/StateMachineTask.h"
#include "ServoPositionRange.h"

namespace p4::servo {
namespace {

FeetechServoDriver* driverForMotor(uint8_t motorChannel)
{
    if (motorChannel >= SERVO_TOTAL_NUM) {
        return nullptr;
    }
    return servoBusRegistry().driver(motorMap[motorChannel].busIndex);
}

uint32_t collectTurnOverflowMask()
{
    uint32_t overflowMask = 0;
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        FeetechServoDriver* driver = driverForMotor(ch);
        const uint8_t id = motorMap[ch].servoID;
        if (!driver || !driver->isOnline(id)) {
            continue;
        }

        // 这里只检查舵机本次上电会话的 hardware ABS；logicalOffset 不参与判断。
        const int32_t hardwareAbs =
            driver->getHardwareAbsolutePosition(id);
        if (hardwareAbs < kServoResetZeroTriggerMin ||
            hardwareAbs > kServoResetZeroTriggerMax) {
            overflowMask |= (1UL << ch);
            driver->setTorqueEnabled(id, false);
        }
    }
    return overflowMask & validMotorChannelMask();
}

} // namespace

uint32_t collectOnlineMotorMask()
{
    uint32_t mask = 0;
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        FeetechServoDriver* driver = driverForMotor(ch);
        if (driver && driver->isOnline(motorMap[ch].servoID)) {
            mask |= (1UL << ch);
        }
    }
    return mask & validMotorChannelMask();
}

void publishRecoveryState(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return;
    }

    uint32_t pendingMask = 0;
    uint32_t configuredMask = 0;
    uint32_t failedMask = 0;
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        FeetechServoDriver* driver = driverForMotor(ch);
        if (!driver) {
            continue;
        }
        const uint8_t id = motorMap[ch].servoID;
        const uint32_t bit = 1UL << ch;
        if (driver->hasOnlineTransitionPending(id)) {
            pendingMask |= bit;
        }
        if (driver->isMultiTurnConfigured(id)) {
            configuredMask |= bit;
        }
        if (driver->isRecoveryFailed(id)) {
            failedMask |= bit;
        }
    }

    sharedAtomicStore(&sharedData->servo_recovery_pending_bitmap,
                      pendingMask & validMotorChannelMask());
    sharedAtomicStore(&sharedData->servo_multiturn_configured_bitmap,
                      configuredMask & validMotorChannelMask());
    sharedAtomicStore(&sharedData->servo_recovery_failed_bitmap,
                      failedMask & validMotorChannelMask());
}

void publishTurnOverflowState(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return;
    }

    const uint32_t previousMask =
        sharedAtomicLoad(&sharedData->servo_turn_overflow_bitmap);
    const uint32_t currentMask = collectTurnOverflowMask();
    sharedAtomicStore(&sharedData->servo_turn_overflow_bitmap,
                      currentMask);

    const uint32_t enteredMask = currentMask & ~previousMask;
    const uint32_t clearedMask = previousMask & ~currentMask;
    if (enteredMask != 0) {
        // 清空尚未消费的命令，并退出运动态，避免下一帧重新使能力矩。
        p4::actuator::clearPending(sharedData);
        postSystemEvent(sharedData, SYSTEM_EVENT_STOP);
        FW_LOG_WARN_PRINTF(
            "[SERVO TURN OVERFLOW] entered=0x%08lX hardwareRange=[%ld,%ld]\r\n",
            (unsigned long)enteredMask,
            (long)kServoResetZeroTriggerMin,
            (long)kServoResetZeroTriggerMax);
    }
    if (clearedMask != 0) {
        FW_LOG_INFO_PRINTF(
            "[SERVO TURN OVERFLOW] cleared=0x%08lX remaining=0x%08lX\r\n",
            (unsigned long)clearedMask,
            (unsigned long)currentMask);
    }
}

void publishFeedback(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return;
    }

    ServoAngleData_t servoData = {};
    ServoAngleData_t servoRawData = {};
    ServoTelemetryData_t telemetryData = {};
    servoData.timestamp = millis();
    servoRawData.timestamp = servoData.timestamp;
    telemetryData.timestamp = servoData.timestamp;

    uint32_t onlineMask = 0;
    uint32_t rangeValidMask = 0;
    int32_t logicalRangeMin[SERVO_TOTAL_NUM] = {0};
    int32_t logicalRangeMax[SERVO_TOTAL_NUM] = {0};

    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        // Flash raw 表示最近一次成功落盘的恢复锚点，与当前在线状态无关。
        servoRawData.servoFlashRawPositions[ch] =
            sharedData->multiturn_nvs_snapshot_raw[ch];

        const uint8_t busIndex = motorMap[ch].busIndex;
        const uint8_t id = motorMap[ch].servoID;
        FeetechServoDriver* driver = driverForMotor(ch);
        const uint32_t bit = 1UL << ch;
        if (driver && driver->isOnline(id)) {
            const ServoFeedbackInternal& feedback =
                driver->getFeedback(id);
            servoData.servoAngles[ch] =
                driver->getAbsolutePosition(id);
            servoData.servoRawPositions[ch] =
                driver->getRawPosition(id);
            servoRawData.servoRawPositions[ch] =
                driver->getRawPosition(id);
            servoData.onlineStatus[ch] = 1;
            servoRawData.onlineStatus[ch] = 1;
            onlineMask |= bit;

            telemetryData.speed[ch] = feedback.speed;
            telemetryData.load[ch] = feedback.load;
            telemetryData.current[ch] = feedback.current;
            telemetryData.voltage[ch] = feedback.voltage;
            telemetryData.temperature[ch] = feedback.temperature;
            telemetryData.onlineStatus[ch] = 1;

            if (driver->isMultiTurnConfigured(id) &&
                !driver->hasOnlineTransitionPending(id) &&
                !driver->isRecoveryFailed(id)) {
                const ServoLogicalRange range =
                    driver->getLogicalRange(id);
                logicalRangeMin[ch] = range.minAbs;
                logicalRangeMax[ch] = range.maxAbs;
                rangeValidMask |= bit;
            }

            if (driver->isAbsoluteReliable(id) &&
                (sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap) &
                 bit) != 0) {
                sharedAtomicFetchAnd(
                    &sharedData->multi_turn_unknown_bitmap, ~bit);
                FW_LOG_INFO_PRINTF(
                    "[SERVO RESTORE] M%02u absolute feedback trusted "
                    "abs=%ld raw=%d unknown=0x%08lX\r\n",
                    (unsigned)ch,
                    (long)servoData.servoAngles[ch],
                    (int)servoData.servoRawPositions[ch],
                    (unsigned long)sharedAtomicLoad(
                        &sharedData->multi_turn_unknown_bitmap));
            }
        } else if (!driver || driver->isOfflineInvalidated(id)) {
            if ((sharedAtomicLoad(
                     &sharedData->multi_turn_unknown_bitmap) &
                 bit) == 0) {
                sharedAtomicFetchOr(
                    &sharedData->multi_turn_unknown_bitmap, bit);
                FW_LOG_INFO_PRINTF(
                    "[SERVO OFFLINE] M%02u bus=%u id=%u marked "
                    "Turn UNKNOWN unknown=0x%08lX\r\n",
                    (unsigned)ch,
                    (unsigned)busIndex,
                    (unsigned)id,
                    (unsigned long)sharedAtomicLoad(
                        &sharedData->multi_turn_unknown_bitmap));
            }
        }
    }

    sharedAtomicStore(&sharedData->servo_online_bitmap,
                      onlineMask & validMotorChannelMask());
    {
        SharedSafetyGuard guard;
        memcpy(sharedData->servo_logical_range_min,
               logicalRangeMin,
               sizeof(logicalRangeMin));
        memcpy(sharedData->servo_logical_range_max,
               logicalRangeMax,
               sizeof(logicalRangeMax));
        sharedAtomicStore(
            &sharedData->servo_logical_range_valid_bitmap,
            rangeValidMask & validMotorChannelMask());
    }

    if (sharedData->servoFeedbackQueue) {
        xQueueOverwrite(sharedData->servoFeedbackQueue, &servoData);
    }
    if (sharedData->servoAngleQueue) {
        xQueueOverwrite(sharedData->servoAngleQueue, &servoData);
    }
    if (sharedData->servoRawQueue) {
        xQueueOverwrite(sharedData->servoRawQueue, &servoRawData);
    }
    if (sharedData->servoTelemetryQueue) {
        xQueueOverwrite(sharedData->servoTelemetryQueue,
                        &telemetryData);
    }
    if (sharedData->servoTelemetrySnapshotQueue) {
        xQueueOverwrite(sharedData->servoTelemetrySnapshotQueue,
                        &telemetryData);
    }
}

} // namespace p4::servo
