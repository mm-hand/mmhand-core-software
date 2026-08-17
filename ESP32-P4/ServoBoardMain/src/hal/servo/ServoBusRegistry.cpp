/**
 * @file ServoBusRegistry.cpp
 * @brief 实现飞特总线实例管理和 UART 启动配置。
 */
#include "ServoBusRegistry.h"

#include "../../config/BoardConfig.h"
#include "../../config/ServoConfig.h"

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

bool ServoBusRegistry::restartBus(uint8_t busIndex,
                                  uint32_t nowMs,
                                  bool motionActive)
{
    if (!initialized_ || motionActive || busIndex >= NUM_BUSES) {
        return false;
    }
    if (hasRestarted_[busIndex] &&
        (uint32_t)(nowMs - lastRestartMs_[busIndex]) <
            mmhand::config::servo::kTransportRestartMinIntervalMs) {
        return false;
    }
    // 从发起重启时刻开始限频；即使底层重启失败，也不能在下一扫描周期连续冲击 BUS。
    lastRestartMs_[busIndex] = nowMs;
    hasRestarted_[busIndex] = true;
    if (!buses_[busIndex].restartTransport()) {
        return false;
    }
    return true;
}
