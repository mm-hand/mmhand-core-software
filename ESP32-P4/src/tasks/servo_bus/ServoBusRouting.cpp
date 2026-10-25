/**
 * @file ServoBusRouting.cpp
 * @brief 实现 Mxx 到物理 BUS/ID 的唯一解析及跨任务路由快照发布。
 */
#include "ServoBusRouting.h"

#include "../../config/topology/HardwareMap.h"
#include "../../hal/servo/FeetechServoDriver.h"
#include "../../hal/servo/ServoBusRegistry.h"
#include "../../system/log/FirmwareLog.h"
#include "../../system/rtos/SharedSafetyState.h"
#include "../../system/time/Clock.h"
#include "../../system/SystemRuntimeContext.h"

FeetechServoDriver* resolveMotorDriver(uint8_t motorChannel,
                                      ServoRoute* outRoute,
                                      bool controllable)
{
    ServoRoute route;
    const bool resolved = controllable
        ? servoRouteTable().resolveControllable(motorChannel, &route)
        : servoRouteTable().resolveDetected(motorChannel, &route);
    if (!resolved) return nullptr;
    FeetechServoDriver* driver = servoBusRegistry().driver(route.busIndex);
    if (!driver) return nullptr;
    if (outRoute) *outRoute = route;
    return driver;
}

void publishServoRouteSnapshot(SystemRuntimeContext* sharedData)
{
    if (!sharedData) return;
    const ServoRouteSnapshot snapshot = servoRouteTable().snapshot();
    SharedSafetyGuard guard;
    sharedAtomicStore(&sharedData->servo_route_valid_bitmap,
                      snapshot.validMotorMask);
    sharedAtomicStore(&sharedData->servo_route_controllable_bitmap,
                      snapshot.controllableMotorMask);
    sharedAtomicStore(&sharedData->servo_route_conflict_bitmap,
                      snapshot.conflictMotorMask);
    sharedAtomicStore(&sharedData->servo_bus_overload_mask,
                      snapshot.busOverloadMask);
    sharedAtomicStore(&sharedData->servo_route_scan_state,
                      snapshot.scanState);
    sharedAtomicStore(&sharedData->servo_route_scan_progress,
                      snapshot.scanProgress);
    sharedAtomicStore(&sharedData->servo_route_generation,
                      snapshot.generation);
    for (uint8_t bus = 0; bus < NUM_BUSES; ++bus) {
        sharedData->servo_bus_device_count[bus] = snapshot.busDeviceCount[bus];
    }
    for (uint8_t motor = 0; motor < SERVO_TOTAL_NUM; ++motor) {
        sharedData->servo_route_bus[motor] = snapshot.routes[motor].busIndex;
        sharedData->servo_route_state[motor] =
            static_cast<uint8_t>(snapshot.routes[motor].state);
    }
}

bool addServoReadTarget(uint8_t busIds[NUM_BUSES][MAX_SERVOS_PER_BUS],
                        uint8_t busCounts[NUM_BUSES],
                        uint8_t bus,
                        uint8_t id)
{
    if (bus >= NUM_BUSES) return false;
    for (uint8_t i = 0; i < busCounts[bus]; ++i) {
        if (busIds[bus][i] == id) return true;
    }
    if (busCounts[bus] >= MAX_SERVOS_PER_BUS) return false;
    busIds[bus][busCounts[bus]++] = id;
    return true;
}

void beginServoBuses()
{
    servoBusRegistry().begin();
}
