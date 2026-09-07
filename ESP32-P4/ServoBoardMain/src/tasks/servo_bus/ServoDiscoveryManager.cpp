/**
 * @file ServoDiscoveryManager.cpp
 * @brief 实现低阻塞 BUS 扫描、重复 ID 检测和掉线后的定向重定位。
 */
#include "ServoDiscoveryManager.h"

#include "../../hal/servo/ServoBusRegistry.h"
#include "ServoRouteTable.h"
#include "../../config/topology/HardwareMap.h"
#include "../../config/ServoConfig.h"
#include "../../platform/log/FirmwareLog.h"
#include <string.h>

using mmhand::config::servo::kDiscoveryProbeTimeoutMs;
using mmhand::config::servo::kOfflineRouteRescanIntervalMs;
using mmhand::config::servo::kRecoveryProbeTimeoutMs;

ServoDiscoveryManager& servoDiscoveryManager()
{
    static ServoDiscoveryManager manager;
    return manager;
}

void ServoDiscoveryManager::begin()
{
    memset(lastKnownBus_, SERVO_ROUTE_BUS_UNKNOWN, sizeof(lastKnownBus_));
    servoRouteTable().begin();
    fullScanPending_ = true;
    startPendingScan(0);
}

void ServoDiscoveryManager::requestMotorScan(uint8_t motorChannel,
                                             uint8_t lastKnownBus)
{
    if (motorChannel < SERVO_TOTAL_NUM) {
        if (lastKnownBus < NUM_BUSES) {
            lastKnownBus_[motorChannel] = lastKnownBus;
        }
        pendingMotorMask_ |= 1UL << motorChannel;
        hotplugWatchMask_ |= 1UL << motorChannel;
    }
}

void ServoDiscoveryManager::startPendingScan(uint32_t nowMs)
{
    if (servoRouteTable().scanActive()) {
        return;
    }
    if (fullScanPending_) {
        fullScanPending_ = false;
        // 全量扫描已经覆盖所有 Mxx，旧的定向重扫请求不应在其结束后再次执行。
        pendingMotorMask_ = 0;
        targetedMotor_ = 0xFF;
        targetedPreferredBus_ = SERVO_ROUTE_BUS_UNKNOWN;
        targetedProbeCount_ = 0;
        scanBus_ = 0;
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
        // 一轮登记全部离线 Mxx，随后仍由 step() 每周期只探测一个 BUS/ID。
        // 这样整组舵机重新上电时无需等待 27 个 500 ms 轮询周期。
        pendingMotorMask_ |= hotplugWatchMask_;
        lastHotplugRescanMs_ = nowMs;
    }
    if (pendingMotorMask_ != 0) {
        targetedMotor_ = 0;
        while (targetedMotor_ < SERVO_TOTAL_NUM &&
               (pendingMotorMask_ & (1UL << targetedMotor_)) == 0) {
            ++targetedMotor_;
        }
        pendingMotorMask_ &= ~(1UL << targetedMotor_);
        scanMotor_ = targetedMotor_;
        targetedPreferredBus_ = lastKnownBus_[targetedMotor_];
        targetedProbeCount_ = 0;
        scanBus_ = targetedBusForProbe(0);
        servoRouteTable().beginMotorScan(targetedMotor_);
        FW_LOG_INFO_PRINTF(
            "[SERVO ROUTE] targeted scan M%02u started preferred_bus=%d\r\n",
            (unsigned)targetedMotor_,
            targetedPreferredBus_ < NUM_BUSES
                ? (int)targetedPreferredBus_
                : -1);
    }
}

/**
 * 生成定向恢复的 BUS 探测顺序：先探测掉电前 BUS，再按编号探测其余 BUS。
 * 全量启动扫描不使用本函数，因此不会改变重复 ID 和 BUS 容量检查的覆盖范围。
 */
uint8_t ServoDiscoveryManager::targetedBusForProbe(uint8_t probeIndex) const
{
    if (targetedPreferredBus_ >= NUM_BUSES) {
        return probeIndex < NUM_BUSES ? probeIndex : 0;
    }
    if (probeIndex == 0) {
        return targetedPreferredBus_;
    }

    uint8_t remainingIndex = (uint8_t)(probeIndex - 1U);
    for (uint8_t bus = 0; bus < NUM_BUSES; ++bus) {
        if (bus == targetedPreferredBus_) {
            continue;
        }
        if (remainingIndex == 0) {
            return bus;
        }
        --remainingIndex;
    }
    return targetedPreferredBus_;
}

void ServoDiscoveryManager::finishCurrentScan()
{
    const uint8_t completedMotor = targetedMotor_;
    servoRouteTable().finishScan();
    const ServoRouteSnapshot routes = servoRouteTable().snapshot();
    for (uint8_t motor = 0; motor < SERVO_TOTAL_NUM; ++motor) {
        if (routes.routes[motor].busIndex < NUM_BUSES &&
            routes.routes[motor].state == ServoRouteState::Online) {
            lastKnownBus_[motor] = routes.routes[motor].busIndex;
        }
    }
    if (completedMotor >= SERVO_TOTAL_NUM) {
        // P4 启动时舵机电源可能尚未开启。旧逻辑只监视“曾在线后掉线”的 Mxx，
        // 导致首次全量扫描缺失的舵机永远不再探测，只能依靠 P4 Reset 恢复。
        // 全量扫描结束后先把全部缺失通道安排一次快速补扫；补扫结束仍未上线的
        // 通道继续进入低频热插拔轮询，真正收到反馈后会从 watch mask 中移除。
        const uint32_t missingMask =
            validMotorChannelMask() & ~routes.validMotorMask;
        pendingMotorMask_ |= missingMask;
        hotplugWatchMask_ |= missingMask;
        if (missingMask != 0) {
            FW_LOG_INFO_PRINTF(
                "[SERVO ROUTE] initial missing motors queued for hotplug scan mask=0x%08lx\r\n",
                (unsigned long)missingMask);
        }
    }
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
    targetedPreferredBus_ = SERVO_ROUTE_BUS_UNKNOWN;
    targetedProbeCount_ = 0;
    scanBus_ = 0;
}

bool ServoDiscoveryManager::step(uint32_t nowMs, bool motionActive)
{
    // 定向扫描不阻断其他 active scope；若其间开始运动，暂停探测以免占用 UART。
    if (motionActive) {
        return false;
    }
    if (!servoRouteTable().scanActive()) {
        startPendingScan(nowMs);
    }
    if (!servoRouteTable().scanActive()) {
        return false;
    }

    const uint8_t probedBus = scanBus_;
    const bool targetedScan = targetedMotor_ < SERVO_TOTAL_NUM;
    FeetechServoDriver* driver = servoBusRegistry().driver(probedBus);
    const uint8_t servoId = motorMap[scanMotor_].servoID;
    const bool present = driver && driver->probeFeedback(
        servoId,
        targetedScan ? kRecoveryProbeTimeoutMs : kDiscoveryProbeTimeoutMs);
    servoRouteTable().recordProbe(probedBus, servoId, present);

    bool scanFinished = false;
    if (targetedScan) {
        // 第一周期优先探测上次 BUS，但仍分周期核验其余 BUS；这样既缩短首次响应等待，
        // 又保留热插拔后重复 ID 检测，不能因旧 BUS 命中就静默接受冲突路由。
        ++targetedProbeCount_;
        if (targetedProbeCount_ >= NUM_BUSES) {
            finishCurrentScan();
            scanFinished = true;
        } else {
            scanBus_ = targetedBusForProbe(targetedProbeCount_);
        }
    } else {
        ++scanBus_;
        if (scanBus_ >= NUM_BUSES) {
            scanBus_ = 0;
            ++scanMotor_;
            if (scanMotor_ >= SERVO_TOTAL_NUM) {
                finishCurrentScan();
                scanFinished = true;
            }
        }
    }
    const uint16_t completed =
        scanFinished
            ? (targetedScan
                   ? NUM_BUSES
                   : static_cast<uint16_t>(SERVO_TOTAL_NUM) * NUM_BUSES)
            : (targetedScan
                   ? targetedProbeCount_
                   : static_cast<uint16_t>(scanMotor_) * NUM_BUSES + scanBus_);
    const uint16_t total =
        targetedScan
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
