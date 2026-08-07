/**
 * @file ServoBusRegistry.cpp
 * @brief 实现飞特总线实例管理、启动配置和低频离线探测。
 */
#include "ServoBusRegistry.h"

#include "../../config/BoardConfig.h"
#include "ServoRouteTable.h"

ServoBusRegistry& servoBusRegistry()
{
    static ServoBusRegistry registry;
    return registry;
}

FeetechServoDriver* ServoBusRegistry::driver(uint8_t busIndex)
{
    return busIndex < NUM_BUSES ? &buses_[busIndex] : nullptr;
}

void ServoBusRegistry::begin()
{
    if (initialized_) {
        return;
    }
    for (uint8_t bus = 0; bus < NUM_BUSES; ++bus) {
        buses_[bus].begin(bus,
                          kServoBusRxPins[bus],
                          kServoBusTxPins[bus],
                          kServoBusBaud);
    }
    initialized_ = true;
}

void ServoBusRegistry::configureRoutedServosForMultiTurn()
{
#if SERVO_CONFIGURE_MULTI_TURN_ON_BOOT
    for (uint8_t channel = 0; channel < SERVO_TOTAL_NUM; ++channel) {
        ServoRoute route;
        if (!servoRouteTable().resolveControllable(channel, &route)) {
            continue;
        }
        FeetechServoDriver* bus = driver(route.busIndex);
        if (bus && !bus->isMultiTurnConfigured(route.servoId)) {
            bus->configureMultiTurnMode(route.servoId);
        }
    }
#endif
}
