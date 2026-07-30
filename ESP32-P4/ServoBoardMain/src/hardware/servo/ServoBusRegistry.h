/**
 * @file ServoBusRegistry.h
 * @brief 集中持有四条飞特总线驱动，并负责一次性初始化和离线探测。
 */
#ifndef P4_SERVO_BUS_REGISTRY_H
#define P4_SERVO_BUS_REGISTRY_H

#include "FeetechServoDriver.h"

class ServoBusRegistry {
public:
    /** 按 HardwareMap.busIndex 获取驱动；越界返回 nullptr。 */
    FeetechServoDriver* driver(uint8_t busIndex);

    /** 使用 BoardConfig 的引脚和波特率初始化全部 UART，仅执行一次。 */
    void begin();

    /** 将 HardwareMap 中出现的舵机配置为多圈模式，仅执行一次。 */
    void configureMappedServosForMultiTurn();

    /** 每个探测周期只查询一台离线舵机，避免故障设备拖慢实时总线。 */
    void probeOneOfflineMappedServo(uint32_t nowMs);

private:
    FeetechServoDriver buses_[NUM_BUSES];
    bool initialized_ = false;
    bool configuredOnce_ = false;
    uint32_t lastOfflineProbeMs_ = 0;
    uint8_t nextOfflineProbeChannel_ = 0;
};

/** 返回进程内唯一总线注册表。 */
ServoBusRegistry& servoBusRegistry();

#endif // P4_SERVO_BUS_REGISTRY_H
