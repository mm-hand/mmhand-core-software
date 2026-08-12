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

private:
    FeetechServoDriver buses_[NUM_BUSES];
    bool initialized_ = false;
};

/** 返回进程内唯一总线注册表。 */
ServoBusRegistry& servoBusRegistry();

#endif // P4_SERVO_BUS_REGISTRY_H
