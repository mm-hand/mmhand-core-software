/**
 * @file ServoDiscoveryManager.cpp
 * @brief 实现低阻塞 BUS 扫描、重复 ID 检测和掉线后的定向重定位。
 */
#include "ServoDiscoveryManager.h"

#include "ServoBusRegistry.h"
#include "ServoRouteTable.h"
#include "../HardwareMap.h"
#include "../../config/ServoConfig.h"
#include "../../system/FirmwareLog.h"

using mmhand::config::servo::kDiscoveryProbeTimeoutMs;
using mmhand::config::servo::kOfflineRouteRescanIntervalMs;

ServoDiscoveryManager& servoDiscoveryManager()
{
    static ServoDiscoveryManager manager;
    return manager;
}

void ServoDiscoveryManager::begin()
{
    servoRouteTable().begin();
    fullScanPending_ = true;
    startPendingScan(0);
}

void ServoDiscoveryManager::requestMotorScan(uint8_t motorChannel)
{
    if (motorChannel < SERVO_TOTAL_NUM) {
        pendingMotorMask_ |= 1UL << motorChannel;
        hotplugWatchMask_ |= 1UL << motorChannel;
    }
}

void ServoDiscoveryManager::startPendingScan(uint32_t nowMs)
{
    if (servoRouteTable().scanActive()) {
        return;
    }
    scanBus_ = 0;
    if (fullScanPending_) {
        fullScanPending_ = false;
        // 全量扫描已经覆盖所有 Mxx，旧的定向重扫请求不应在其结束后再次执行。
        pendingMotorMask_ = 0;
        targetedMotor_ = 0xFF;
        scanMotor_ = 0;
        servoRouteTable().beginFullScan();
        FW_LOG_INFO_PRINTF("[SERVO ROUTE] full scan started\r\n");
        return;
    }
    // 即时请求处理完后，低频轮询曾在线后掉线的 Mxx。每次只排入一台，
    // 扫描本身仍按 4 个控制周期分片，且运动状态下不会启动。
    if (pendingMotorMask_ == 0 && hotplugWatchMask_ != 0 &&
        (uint32_t)(nowMs - lastHotplugRescanMs_) >=
            kOfflineRouteRescanIntervalMs) {
        for (uint8_t offset = 0; offset < SERVO_TOTAL_NUM; ++offset) {
            const uint8_t motor = static_cast<uint8_t>(
                (nextHotplugMotor_ + offset) % SERVO_TOTAL_NUM);
            if ((hotplugWatchMask_ & (1UL << motor)) != 0) {
                pendingMotorMask_ |= 1UL << motor;
                nextHotplugMotor_ = static_cast<uint8_t>(
                    (motor + 1U) % SERVO_TOTAL_NUM);
                lastHotplugRescanMs_ = nowMs;
                break;
            }
        }
    }
    if (pendingMotorMask_ != 0) {
        targetedMotor_ = 0;
        while (targetedMotor_ < SERVO_TOTAL_NUM &&
               (pendingMotorMask_ & (1UL << targetedMotor_)) == 0) {
            ++targetedMotor_;
        }
        pendingMotorMask_ &= ~(1UL << targetedMotor_);
        scanMotor_ = targetedMotor_;
        servoRouteTable().beginMotorScan(targetedMotor_);
        FW_LOG_INFO_PRINTF("[SERVO ROUTE] targeted scan M%02u started\r\n",
                           (unsigned)targetedMotor_);
    }
}

void ServoDiscoveryManager::finishCurrentScan(uint32_t nowMs)
{
    servoRouteTable().finishScan(nowMs);
    const ServoRouteSnapshot routes = servoRouteTable().snapshot();
    // 唯一且未超容量的路由已经恢复，不再继续低频探测对应 Mxx。
    hotplugWatchMask_ &= ~routes.controllableMotorMask;
    FW_LOG_INFO_PRINTF(
        "[SERVO ROUTE] scan done valid=0x%08lx controllable=0x%08lx conflict=0x%08lx overload=0x%02x\r\n",
        (unsigned long)routes.validMotorMask,
        (unsigned long)routes.controllableMotorMask,
        (unsigned long)routes.conflictMotorMask,
        (unsigned)routes.busOverloadMask);
    for (uint8_t bus = 0; bus < NUM_BUSES; ++bus) {
        if ((routes.busOverloadMask & (1U << bus)) != 0) {
            FW_LOG_WARN_PRINTF(
                "[SERVO ROUTE] BUS%u has %u responding IDs; maximum controllable count is %u, move servos to another BUS\r\n",
                (unsigned)bus,
                (unsigned)routes.busDeviceCount[bus],
                (unsigned)mmhand::config::servo::kMaxControllableServosPerBus);
        }
    }
    targetedMotor_ = 0xFF;
}

bool ServoDiscoveryManager::step(uint32_t nowMs, bool motionActive)
{
    // 运行中禁止开始新扫描；已经开始的上电扫描必须在进入运动状态前完成。
    if (!servoRouteTable().scanActive()) {
        if (motionActive) {
            return false;
        }
        startPendingScan(nowMs);
    }
    if (!servoRouteTable().scanActive()) {
        return false;
    }

    FeetechServoDriver* driver = servoBusRegistry().driver(scanBus_);
    const uint8_t servoId = motorMap[scanMotor_].servoID;
    const bool present = driver &&
        driver->probeFeedback(servoId, kDiscoveryProbeTimeoutMs);
    servoRouteTable().recordProbe(scanBus_, servoId, present, nowMs);

    ++scanBus_;
    if (scanBus_ >= NUM_BUSES) {
        scanBus_ = 0;
        if (targetedMotor_ < SERVO_TOTAL_NUM) {
            finishCurrentScan(nowMs);
        } else {
            ++scanMotor_;
            if (scanMotor_ >= SERVO_TOTAL_NUM) {
                finishCurrentScan(nowMs);
            }
        }
    }
    const uint16_t completed =
        targetedMotor_ < SERVO_TOTAL_NUM
            ? scanBus_
            : static_cast<uint16_t>(scanMotor_) * NUM_BUSES + scanBus_;
    const uint16_t total =
        targetedMotor_ < SERVO_TOTAL_NUM
            ? NUM_BUSES
            : static_cast<uint16_t>(SERVO_TOTAL_NUM) * NUM_BUSES;
    servoRouteTable().setScanProgress(
        static_cast<uint8_t>((completed * 100U) / total));
    return true;
}

bool ServoDiscoveryManager::active() const
{
    return servoRouteTable().scanActive();
}
