/**
 * @file ServoFeedbackPublisher.cpp
 * @brief 实现从驱动缓存到共享反馈快照的转换及恢复/越界状态发布。
 */
#include "ServoFeedbackPublisher.h"
#include "../../system/time/Clock.h"

#include <string.h>

#include "../../system/ActuatorCommandGateway.h"
#include "../../config/topology/HardwareMap.h"
#include "../../hal/servo/FeetechServoDriver.h"
#include "../../hal/servo/ServoBusRegistry.h"
#include "ServoRouteTable.h"
#include "../../system/rtos/SharedSafetyState.h"
#include "../../system/log/FirmwareLog.h"
#include "../../system/ports/SystemEventPort.h"
#include "../../config/safety/ServoPositionRange.h"

namespace p4::servo {
namespace {

/** 由 Mxx 映射到其总线驱动；无效通道返回 nullptr。 */
FeetechServoDriver* driverForMotor(uint8_t motorChannel,
                                  ServoRoute* outRoute = nullptr)
{
    ServoRoute route;
    route.busIndex = SERVO_ROUTE_BUS_UNKNOWN;
    route.servoId = motorChannel < SERVO_TOTAL_NUM
        ? motorMap[motorChannel].servoID
        : 0xFF;
    route.state = ServoRouteState::Unknown;
    if (outRoute) {
        *outRoute = route;
    }
    if (!servoRouteTable().resolveDetected(motorChannel, &route)) {
        return nullptr;
    }
    if (outRoute) {
        *outRoute = route;
    }
    return servoBusRegistry().driver(route.busIndex);
}

/** 检查全部在线舵机 hardware ABS，并立即关闭越界通道力矩。 */
uint32_t collectTurnOverflowMask()
{
    uint32_t overflowMask = 0;
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        ServoRoute route;
        FeetechServoDriver* driver = driverForMotor(ch, &route);
        const uint8_t id = route.servoId;
        if (!driver || !driver->isOnline(id)) {
            continue;
        }

        // 业务反馈与舵机使用相同 ABS，越界时立即关闭力矩。
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

// 在线位图直接来自各总线驱动缓存，只统计 HardwareMap 中实际存在的 Mxx。
uint32_t collectOnlineMotorMask()
{
    uint32_t mask = 0;
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        ServoRoute route;
        FeetechServoDriver* driver = driverForMotor(ch, &route);
        if (driver && driver->isOnline(route.servoId)) {
            mask |= (1UL << ch);
        }
    }
    return mask & validMotorChannelMask();
}

// 恢复 pending/success/failed 三个位图来自驱动事务，供状态机和上位机显示。
void publishRecoveryState(SystemRuntimeContext* sharedData)
{
    if (!sharedData) {
        return;
    }

    uint32_t pendingMask = 0;
    uint32_t configuredMask = 0;
    uint32_t failedMask = 0;
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        ServoRoute route;
        FeetechServoDriver* driver = driverForMotor(ch, &route);
        if (!driver) {
            continue;
        }
        const uint8_t id = route.servoId;
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

// 只在越界位图发生变化时记录/停机；持续越界不会每周期重复触发 STOP。
void publishTurnOverflowState(SystemRuntimeContext* sharedData)
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

// 每周期生成同一时刻的逻辑位置、物理位置和负载视图。
void publishFeedback(SystemRuntimeContext* sharedData,
                     const ServoFeedbackCycleMeta_t& cycleMeta)
{
    if (!sharedData) {
        return;
    }

    ServoAngleData_t servoData = {};
    ServoAngleData_t servoRawData = {};
    ServoTelemetryData_t telemetryData = {};
    servoData.timestamp = p4::platform::time::monotonicMillis();
    servoRawData.timestamp = servoData.timestamp;
    telemetryData.timestamp = servoData.timestamp;
    servoData.cycle = cycleMeta;
    servoRawData.cycle = cycleMeta;
    telemetryData.cycle = cycleMeta;

    uint32_t onlineMask = 0;
    uint32_t rangeValidMask = 0;
    int32_t logicalRangeMin[SERVO_TOTAL_NUM] = {0};
    int32_t logicalRangeMax[SERVO_TOTAL_NUM] = {0};

    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        ServoRoute route;
        FeetechServoDriver* driver = driverForMotor(ch, &route);
        const uint8_t busIndex = route.busIndex;
        const uint8_t id = route.servoId;
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

    // 三类数据各保留一个“最新值邮箱”；多个消费者使用 peek() 共读，不复制队列。
    sharedData->servoFeedbackQueue.overwrite(servoData);
    sharedData->servoRawQueue.overwrite(servoRawData);
    sharedData->servoTelemetryQueue.overwrite(telemetryData);
}

} // namespace p4::servo
