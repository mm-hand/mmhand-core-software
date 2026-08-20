/**
 * @file ServoBusRegistry.h
 * @brief 集中持有四条飞特总线驱动，只负责 UART 和驱动实例生命周期。
 */
#ifndef P4_SERVO_BUS_REGISTRY_H
#define P4_SERVO_BUS_REGISTRY_H

#include "FeetechServoDriver.h"

class ServoBusRegistry {
public:
    /** 按物理 BUS 索引获取驱动；Mxx 到 BUS 的关系由 ServoRouteTable 提供。 */
    FeetechServoDriver* driver(uint8_t busIndex);

    /** 使用 BoardConfig 的引脚和波特率初始化全部 UART，仅执行一次。 */
    void begin();

    /**
     * 受控重启一条物理 BUS。调用方必须传入当前运动状态，本层同时执行
     * 1000 ms 限频兜底；不会重启其他 BUS。重启 BUS 上的旧在线和 logical
     * ABS 信任会失效，必须由后续真实反馈及恢复事务重新建立。
     */
    bool restartBus(uint8_t busIndex, uint32_t nowMs, bool motionActive);

private:
    FeetechServoDriver buses_[NUM_BUSES];
    uint32_t lastRestartMs_[NUM_BUSES] = {0}; ///< 每条 BUS 最近一次成功重启时间。
    bool hasRestarted_[NUM_BUSES] = {false}; ///< 区分“从未重启”和时间戳恰为 0。
    bool initialized_ = false;
};

/** 返回进程内唯一总线注册表。 */
ServoBusRegistry& servoBusRegistry();

#endif // P4_SERVO_BUS_REGISTRY_H
