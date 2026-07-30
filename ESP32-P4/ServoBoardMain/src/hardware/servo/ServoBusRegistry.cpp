/**
 * @file ServoBusRegistry.cpp
 * @brief 实现飞特总线实例管理、启动配置和低频离线探测。
 */
#include "ServoBusRegistry.h"

#include "../../config/BoardConfig.h"
#include "../HardwareMap.h"

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

void ServoBusRegistry::configureMappedServosForMultiTurn()
{
#if SERVO_CONFIGURE_MULTI_TURN_ON_BOOT
    if (configuredOnce_) {
        return;
    }

    uint8_t configuredIds[NUM_BUSES][MAX_SERVOS_PER_BUS] = {0};
    uint8_t configuredCounts[NUM_BUSES] = {0};
    for (uint8_t channel = 0; channel < SERVO_TOTAL_NUM; ++channel) {
        const uint8_t busIndex = motorMap[channel].busIndex;
        const uint8_t id = motorMap[channel].servoID;
        if (busIndex >= NUM_BUSES || id > MAX_SERVO_ID) {
            continue;
        }

        bool alreadyConfigured = false;
        for (uint8_t i = 0; i < configuredCounts[busIndex]; ++i) {
            if (configuredIds[busIndex][i] == id) {
                alreadyConfigured = true;
                break;
            }
        }
        if (alreadyConfigured ||
            configuredCounts[busIndex] >= MAX_SERVOS_PER_BUS) {
            continue;
        }

        FeetechServoDriver* bus = driver(busIndex);
        if (bus && bus->configureMultiTurnMode(id)) {
            configuredIds[busIndex][configuredCounts[busIndex]++] = id;
        }
    }
    configuredOnce_ = true;
#endif
}

void ServoBusRegistry::probeOneOfflineMappedServo(uint32_t nowMs)
{
    if (static_cast<uint32_t>(nowMs - lastOfflineProbeMs_) <
        kServoOfflineProbeIntervalMs) {
        return;
    }
    lastOfflineProbeMs_ = nowMs;

    for (uint8_t offset = 0; offset < SERVO_TOTAL_NUM; ++offset) {
        const uint8_t channel =
            static_cast<uint8_t>((nextOfflineProbeChannel_ + offset) %
                                 SERVO_TOTAL_NUM);
        const uint8_t busIndex = motorMap[channel].busIndex;
        const uint8_t id = motorMap[channel].servoID;
        FeetechServoDriver* bus = driver(busIndex);
        if (!bus || id > MAX_SERVO_ID || bus->isOnline(id)) {
            continue;
        }
        nextOfflineProbeChannel_ =
            static_cast<uint8_t>((channel + 1) % SERVO_TOTAL_NUM);
        bus->probeFeedback(id);
        return;
    }
}
