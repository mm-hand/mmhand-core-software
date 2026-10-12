/**
 * @file ServoBusRegistry.cpp
 * @brief 实现飞特总线实例管理和 UART 启动配置。
 */
#include "ServoBusRegistry.h"

#include "../../config/BoardConfig.h"

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
