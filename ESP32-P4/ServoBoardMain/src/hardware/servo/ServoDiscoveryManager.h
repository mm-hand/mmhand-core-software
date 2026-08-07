/**
 * @file ServoDiscoveryManager.h
 * @brief 在舵机通信任务内分时扫描四条 BUS，并生成 Mxx 到物理 BUS 的运行时路由。
 *
 * 本模块不创建独立任务，也不直接拥有 UART。调用方必须是 ServoCommunicationTask，
 * 从而保证扫描、同步反馈和控制写入不会并发访问同一条飞特总线。
 */
#ifndef P4_SERVO_DISCOVERY_MANAGER_H
#define P4_SERVO_DISCOVERY_MANAGER_H

#include <stdint.h>

class ServoDiscoveryManager {
public:
    /** 上电后启动一次全量扫描；扫描期间不允许提交运动命令。 */
    void begin();

    /** 请求重新定位单个 Mxx；用于掉线重连，不扫描无关舵机。 */
    void requestMotorScan(uint8_t motorChannel);

    /** 每次只探测一个 BUS/ID，返回本周期是否实际访问过 UART。 */
    bool step(uint32_t nowMs, bool motionActive);

    bool active() const;

private:
    void startPendingScan(uint32_t nowMs);
    void finishCurrentScan(uint32_t nowMs);

    uint8_t scanBus_ = 0;
    uint8_t scanMotor_ = 0;
    uint8_t targetedMotor_ = 0xFF;
    // 多台舵机可能在同一反馈周期掉线，位图可避免后来的请求覆盖先前请求。
    uint32_t pendingMotorMask_ = 0;
    // 只观察“本次会话曾在线后掉线”的 Mxx，避免部分装机时反复扫描从未连接的通道。
    uint32_t hotplugWatchMask_ = 0;
    uint32_t lastHotplugRescanMs_ = 0;
    uint8_t nextHotplugMotor_ = 0;
    bool fullScanPending_ = false;
};

ServoDiscoveryManager& servoDiscoveryManager();

#endif // P4_SERVO_DISCOVERY_MANAGER_H
