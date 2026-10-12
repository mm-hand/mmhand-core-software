/**
 * @file ServoRouteTable.cpp
 * @brief 实现动态路由提交、重复 ID 棭测和 BUS 容量硬门禁。
 */
#include "ServoRouteTable.h"

#include "../../config/ServoConfig.h"
#include "../../config/topology/HardwareMap.h"
#include <string.h>

using mmhand::config::servo::kMaxControllableServosPerBus;

namespace {

uint8_t popcount27(uint32_t value)
{
    uint8_t count = 0;
    value &= validMotorChannelMask();
    while (value != 0) {
        count += static_cast<uint8_t>(value & 1U);
        value >>= 1U;
    }
    return count;
}

} // namespace

ServoRouteTable& servoRouteTable()
{
    static ServoRouteTable table;
    return table;
}

void ServoRouteTable::begin()
{
    memset(&snapshot_, 0, sizeof(snapshot_));
    for (uint8_t motor = 0; motor < SERVO_TOTAL_NUM; ++motor) {
        snapshot_.routes[motor].busIndex = SERVO_ROUTE_BUS_UNKNOWN;
        snapshot_.routes[motor].servoId = motorMap[motor].servoID;
        snapshot_.routes[motor].state = ServoRouteState::Unknown;
    }
}

void ServoRouteTable::beginFullScan()
{
    memset(snapshot_.detectedIdMask, 0, sizeof(snapshot_.detectedIdMask));
    snapshot_.validMotorMask = 0;
    snapshot_.controllableMotorMask = 0;
    snapshot_.conflictMotorMask = 0;
    snapshot_.busOverloadMask = 0;
    snapshot_.scanState = 1;
    snapshot_.scanProgress = 0;
    for (uint8_t motor = 0; motor < SERVO_TOTAL_NUM; ++motor) {
        // 扫描期间旧 BUS 路由立即失效，禁止控制链继续向旧物理位置发送命令。
        snapshot_.routes[motor].busIndex = SERVO_ROUTE_BUS_UNKNOWN;
        snapshot_.routes[motor].state = ServoRouteState::Probing;
    }
    ++snapshot_.generation;
}

void ServoRouteTable::beginMotorScan(uint8_t motorChannel)
{
    if (motorChannel >= SERVO_TOTAL_NUM || scanActive()) {
        return;
    }
    const uint32_t idBit = 1UL << motorMap[motorChannel].servoID;
    for (uint8_t bus = 0; bus < NUM_BUSES; ++bus) {
        snapshot_.detectedIdMask[bus] &= ~idBit;
    }
    snapshot_.validMotorMask &= ~(1UL << motorChannel);
    snapshot_.controllableMotorMask &= ~(1UL << motorChannel);
    snapshot_.conflictMotorMask &= ~(1UL << motorChannel);
    snapshot_.routes[motorChannel].busIndex = SERVO_ROUTE_BUS_UNKNOWN;
    snapshot_.routes[motorChannel].state = ServoRouteState::Probing;
    snapshot_.scanState = 2;
    snapshot_.scanProgress = 0;
    ++snapshot_.generation;
}

void ServoRouteTable::recordProbe(uint8_t busIndex,
                                  uint8_t servoId,
                                  bool present)
{
    if (busIndex >= NUM_BUSES || servoId > MAX_SERVO_ID) {
        return;
    }
    const uint32_t bit = 1UL << servoId;
    const int motor = findMotorChannelByServoId(servoId);
    if (present) {
        snapshot_.detectedIdMask[busIndex] |= bit;
        if (motor >= 0) {
        }
    } else {
        snapshot_.detectedIdMask[busIndex] &= ~bit;
    }
}

void ServoRouteTable::rebuildDerivedState()
{
    snapshot_.validMotorMask = 0;
    snapshot_.controllableMotorMask = 0;
    snapshot_.conflictMotorMask = 0;
    snapshot_.busOverloadMask = 0;
    for (uint8_t bus = 0; bus < NUM_BUSES; ++bus) {
        snapshot_.busDeviceCount[bus] = popcount27(snapshot_.detectedIdMask[bus]);
        if (snapshot_.busDeviceCount[bus] > kMaxControllableServosPerBus) {
            snapshot_.busOverloadMask |= static_cast<uint8_t>(1U << bus);
        }
    }

    for (uint8_t motor = 0; motor < SERVO_TOTAL_NUM; ++motor) {
        const uint8_t id = motorMap[motor].servoID;
        uint8_t foundBus = SERVO_ROUTE_BUS_UNKNOWN;
        uint8_t foundCount = 0;
        for (uint8_t bus = 0; bus < NUM_BUSES; ++bus) {
            if ((snapshot_.detectedIdMask[bus] & (1UL << id)) != 0) {
                foundBus = bus;
                ++foundCount;
            }
        }

        ServoRoute& route = snapshot_.routes[motor];
        route.servoId = id;
        route.busIndex = foundCount == 1 ? foundBus : SERVO_ROUTE_BUS_UNKNOWN;
        if (foundCount == 0) {
            route.state = ServoRouteState::Offline;
        } else if (foundCount > 1) {
            route.state = ServoRouteState::Conflict;
            snapshot_.conflictMotorMask |= 1UL << motor;
        } else {
            snapshot_.validMotorMask |= 1UL << motor;
            if ((snapshot_.busOverloadMask & (1U << foundBus)) != 0) {
                route.state = ServoRouteState::BusOverloaded;
            } else {
                route.state = ServoRouteState::Online;
                snapshot_.controllableMotorMask |= 1UL << motor;
            }
        }
    }
    ++snapshot_.generation;
}

void ServoRouteTable::finishScan()
{
    rebuildDerivedState();
    snapshot_.scanState = 0;
    snapshot_.scanProgress = 100;
}

void ServoRouteTable::setScanProgress(uint8_t progress)
{
    if (scanActive()) {
        snapshot_.scanProgress = progress > 99U ? 99U : progress;
    }
}

void ServoRouteTable::markMotorOffline(uint8_t motorChannel)
{
    if (motorChannel >= SERVO_TOTAL_NUM) {
        return;
    }
    ServoRoute& route = snapshot_.routes[motorChannel];
    if (route.busIndex < NUM_BUSES) {
        snapshot_.detectedIdMask[route.busIndex] &= ~(1UL << route.servoId);
    }
    route.busIndex = SERVO_ROUTE_BUS_UNKNOWN;
    route.state = ServoRouteState::Offline;
    snapshot_.validMotorMask &= ~(1UL << motorChannel);
    snapshot_.controllableMotorMask &= ~(1UL << motorChannel);
    ++snapshot_.generation;
}

bool ServoRouteTable::resolveDetected(uint8_t motorChannel, ServoRoute* out) const
{
    if (!out || motorChannel >= SERVO_TOTAL_NUM) {
        return false;
    }
    const ServoRoute& route = snapshot_.routes[motorChannel];
    if (route.busIndex >= NUM_BUSES ||
        (route.state != ServoRouteState::Online &&
         route.state != ServoRouteState::BusOverloaded)) {
        return false;
    }
    *out = route;
    return true;
}

bool ServoRouteTable::resolveControllable(uint8_t motorChannel, ServoRoute* out) const
{
    if (!out || motorChannel >= SERVO_TOTAL_NUM ||
        (snapshot_.controllableMotorMask & (1UL << motorChannel)) == 0) {
        return false;
    }
    *out = snapshot_.routes[motorChannel];
    return true;
}

ServoRouteSnapshot ServoRouteTable::snapshot() const
{
    return snapshot_;
}

bool ServoRouteTable::scanActive() const
{
    return snapshot_.scanState != 0;
}

uint8_t ServoRouteTable::scanState() const
{
    return snapshot_.scanState;
}
