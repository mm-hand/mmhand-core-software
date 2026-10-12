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

bool restoreServoMultiTurnFromHost(SystemRuntimeContext* sharedData,
                                   uint32_t motorMask,
                                   const int32_t* motorAbs,
                                   uint32_t resetZeroMask,
                                   bool clearRecoveryTransaction,
                                   uint32_t* restoredMask,
                                   uint32_t* rejectedMask)
{
    uint32_t restored = 0;
    uint32_t rejected = 0;
    if (!sharedData || !motorAbs) {
        if (restoredMask) *restoredMask = 0;
        if (rejectedMask) *rejectedMask = motorMask;
        return false;
    }
    // 旧 JSON 的 savedRaw 可能不在当前 ABS 坐标系。置数已生效但应答丢失时，
    // 主机按旧 raw 重算的下一次 ABS 会漂移；有限续接必须保持首次写入目标。
    struct PendingHardwareSync {
        FeetechServoDriver* driver = nullptr;
        int32_t absolutePosition = 0;
        uint32_t trustedTimestampMs = 0;
        uint32_t onlineSessionGeneration = 0;
        uint8_t servoId = 0;
        uint8_t failedRequests = 0;
        bool active = false;
        bool blocked = false;
    };
    static PendingHardwareSync pending[SERVO_TOTAL_NUM];
    static SystemRuntimeContext* pendingOwner = nullptr;
    constexpr uint8_t kMaxFailedSyncRequests = 3;
    if (pendingOwner != sharedData) {
        pendingOwner = sharedData;
        for (auto& item : pending) item = PendingHardwareSync{};
    }
    // 保留既有调用接口；主机坐标同步始终以 0x0B 改写实际 ABS，
    // 不再依据本次 P4 会话是否清过圈数而仅修改 RAM 坐标。
    (void)resetZeroMask;
    const uint32_t requestMask = motorMask & validMotorChannelMask();
    for (uint8_t channel = 0; channel < SERVO_TOTAL_NUM; ++channel) {
        const uint32_t bit = 1UL << channel;
        if ((requestMask & bit) == 0) continue;
        ServoRoute route;
        FeetechServoDriver* driver =
            resolveMotorDriver(channel, &route, true);
        PendingHardwareSync& retry = pending[channel];
        if (retry.active) {
            if (!driver || driver != retry.driver || route.servoId != retry.servoId) {
                retry.blocked = true;
            } else {
                const ServoFeedbackInternal& fb = driver->getFeedback(route.servoId);
                if (driver->isAbsoluteReliable(route.servoId) ||
                    fb.offlineLastTrustedTimestampMs != retry.trustedTimestampMs) {
                    // 同一舵机已经建立新的可信坐标（例如显式 F0），旧事务结束。
                    retry = PendingHardwareSync{};
                } else if (!driver->isOnline(route.servoId) ||
                           fb.onlineSessionGeneration != retry.onlineSessionGeneration ||
                           driver->hasOnlineTransitionPending(route.servoId)) {
                    retry.blocked = true;
                }
            }
        }
        if (motorAbs[channel] < kServoHardwareCommandMin ||
            motorAbs[channel] > kServoHardwareCommandMax) {
            rejected |= bit;
            FW_LOG_WARN_PRINTF(
                "[SERVO SESSION REBASE] M%02u requested_abs=%ld result=failed reason=abs-range\r\n",
                (unsigned)channel, (long)motorAbs[channel]);
            continue;
        }
        if (retry.active && (retry.blocked ||
                             retry.failedRequests >= kMaxFailedSyncRequests)) {
            // 无法确认还是同一次坐标改写时保持 Unknown，禁止使用漂移后的请求值。
            rejected |= bit;
            FW_LOG_WARN_PRINTF(
                "[SERVO SESSION REBASE] M%02u requested_abs=%ld pending_abs=%ld result=failed reason=unresolved-sync\r\n",
                (unsigned)channel, (long)motorAbs[channel],
                (long)retry.absolutePosition);
            continue;
        }
        const int32_t targetAbs = retry.active
            ? retry.absolutePosition : motorAbs[channel];
        uint8_t attempts = 0;
        int16_t calibratedRaw = 0;
        bool correctionAttempted = false;
        const uint32_t sessionGeneration = driver
            ? driver->getFeedback(route.servoId).onlineSessionGeneration : 0;
        const uint32_t trustedTimestamp = driver
            ? driver->getFeedback(route.servoId).offlineLastTrustedTimestampMs : 0;
        const bool restoredOk = driver &&
            driver->calibrateHardwareAbsolutePosition(
                route.servoId, targetAbs, &calibratedRaw, &attempts,
                &correctionAttempted);
        if (!restoredOk) {
            if (correctionAttempted) {
                if (!retry.active) {
                    retry.driver = driver;
                    retry.servoId = route.servoId;
                    retry.absolutePosition = targetAbs;
                    retry.trustedTimestampMs = trustedTimestamp;
                    retry.onlineSessionGeneration = sessionGeneration;
                    retry.active = true;
                }
                if (!driver->isOnline(route.servoId) ||
                    driver->getFeedback(route.servoId).onlineSessionGeneration !=
                        sessionGeneration) {
                    retry.blocked = true;
                }
                ++retry.failedRequests;
            }
            rejected |= bit;
            FW_LOG_WARN_PRINTF(
                "[SERVO SESSION REBASE] M%02u instruction=0x0B requested_abs=%ld target_abs=%ld attempts=%u result=failed\r\n",
                (unsigned)channel,
                (long)motorAbs[channel],
                (long)targetAbs,
                (unsigned)attempts);
            continue;
        }
        retry = PendingHardwareSync{};
        if (clearRecoveryTransaction) {
            driver->finishRecoveryAttempt(
                route.servoId, true,
                p4::platform::time::monotonicMillis());
        }
        restored |= bit;
        FW_LOG_INFO_PRINTF(
            "[SERVO SESSION REBASE] M%02u bus=%u id=%u instruction=0x0B requested_abs=%ld target_abs=%ld attempts=%u "
            "raw=%d abs=%ld result=success\r\n",
            (unsigned)channel,
            (unsigned)route.busIndex,
            (unsigned)route.servoId,
            (long)motorAbs[channel],
            (long)targetAbs,
            (unsigned)attempts,
            (int)driver->getRawPosition(route.servoId),
            (long)driver->getAbsolutePosition(route.servoId));
    }
    if (restored != 0) {
        sharedAtomicFetchAnd(&sharedData->multi_turn_unknown_bitmap,
                             ~restored);
    }
    if (restoredMask) *restoredMask = restored;
    if (rejectedMask) *rejectedMask = rejected;
    FW_LOG_INFO_PRINTF(
        "[SERVO RESTORE] request=0x%08lX restored=0x%08lX rejected=0x%08lX unknown=0x%08lX mode=hardware-abs\r\n",
        (unsigned long)requestMask,
        (unsigned long)restored,
        (unsigned long)rejected,
        (unsigned long)sharedAtomicLoad(
            &sharedData->multi_turn_unknown_bitmap));
    return requestMask != 0 && (requestMask & ~restored) == 0;
}

void beginServoBuses()
{
    servoBusRegistry().begin();
}
