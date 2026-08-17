/**
 * @file ServoDiscoveryManager.h
 * @brief 在舵机通信任务内分时扫描四条 BUS，并生成 Mxx 到物理 BUS 的运行时路由。
 *
 * 本模块不创建独立任务，也不直接拥有 UART。调用方必须是 ServoBusTask，
 * 从而保证扫描、同步反馈和控制写入不会并发访问同一条飞特总线。
 */
#ifndef P4_SERVO_DISCOVERY_MANAGER_H
#define P4_SERVO_DISCOVERY_MANAGER_H

#include <stdint.h>

#include "../../config/DimensionsConfig.h"
#include "../../data/ServoRouteTypes.h"

class ServoDiscoveryManager {
public:
    /** 上电后启动一次全量扫描；扫描期间不允许提交运动命令。 */
    void begin();

    /** 请求重新定位单个 Mxx；用于掉线重连，不扫描无关舵机。 */
    void requestMotorScan(uint8_t motorChannel, uint8_t lastKnownBus);

    /** 每次只探测一个 BUS/ID，返回本周期是否实际访问过 UART。 */
    bool step(uint32_t nowMs, bool motionActive);

    bool active() const;

private:
    void startPendingScan(uint32_t nowMs);
    void finishCurrentScan(uint32_t nowMs, bool motionActive);
    bool stepRestartProbe(uint32_t nowMs, bool motionActive);
    uint8_t targetedBusForProbe(uint8_t probeIndex) const;

    uint8_t scanBus_ = 0;
    uint8_t scanMotor_ = 0;
    uint8_t targetedMotor_ = 0xFF;
    uint8_t targetedPreferredBus_ = SERVO_ROUTE_BUS_UNKNOWN;
    uint8_t targetedProbeCount_ = 0;
    // 多台舵机可能在同一反馈周期掉线，位图可避免后来的请求覆盖先前请求。
    uint32_t pendingMotorMask_ = 0;
    // 保存当前仍离线的 Mxx。首次扫描缺失和运行中掉线都会进入该位图；
    // 一旦收到真实反馈并恢复可控，立即从位图移除，避免在线通道被重复扫描。
    uint32_t hotplugWatchMask_ = 0;
    uint32_t lastHotplugRescanMs_ = 0;
    // 路由表在掉线时会清成 UNKNOWN；这里保留掉线前 BUS，供定向 UART 恢复使用。
    uint8_t lastKnownBus_[SERVO_TOTAL_NUM] = {0};
    uint8_t failedScanCount_[SERVO_TOTAL_NUM] = {0};
    uint8_t restartProbeMotor_ = 0xFF;
    uint8_t restartProbeBus_ = 0xFF;
    uint32_t restartProbeNotBeforeMs_ = 0;
    uint8_t unknownBusRestartCursor_ = 0; ///< 首次缺失无历史 BUS 时轮流恢复四条 UART。
    bool fullScanPending_ = false;
};

ServoDiscoveryManager& servoDiscoveryManager();

#endif // P4_SERVO_DISCOVERY_MANAGER_H
