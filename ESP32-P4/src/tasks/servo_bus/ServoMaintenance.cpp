/** @file ServoMaintenance.cpp
 * @brief 在舵机总线任务中执行维护会话、探测、改号和参数验证。
 */
#include "ServoMaintenance.h"
#include "ServoDiscoveryManager.h"
#include "../../system/SystemRuntimeContext.h"
#include "../../system/rtos/SharedSafetyState.h"
#include "../../hal/servo/ServoBusRegistry.h"
#include "../../hal/serial/HostSerialPort.h"
#include "../../config/protocol/HostProtocol.h"
#include "../../system/log/FirmwareLog.h"
#include "../../config/topology/HardwareMap.h"
#include "../host_comm/HostCalibrationCommandHandler.h"
#include <Arduino.h>
#include <string.h>

namespace {
portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
uint8_t request[12], result[16];
bool queued = false, executing = false, ready = false, sending = false, active = false, cancel = false;
uint32_t session = 0, lastRequestMs = 0;
uint32_t changedMask=0;
// At most 253 final IDs * two moves + begin/end. Probes must not evict writes.
uint8_t cachedRequests[512][12], cachedResults[512][16];
uint16_t cachedCount=0, cachedNext=0;
uint32_t cachedSession=0;
uint32_t readMaintenanceU32(const uint8_t* b) { return (uint32_t(b[0])<<24)|(uint32_t(b[1])<<16)|(uint32_t(b[2])<<8)|b[3]; }
}

bool submitServoMaintenance(const uint8_t* payload) {
    portENTER_CRITICAL(&lock);
    if (queued || executing || ready || sending) { portEXIT_CRITICAL(&lock); return false; }
    memcpy(request, payload, sizeof(request)); queued = true;
    portEXIT_CRITICAL(&lock); return true;
}
bool servoMaintenanceBlocksCommands() {
    portENTER_CRITICAL(&lock);
    const bool blocked = active || queued || executing;
    portEXIT_CRITICAL(&lock); return blocked;
}
void cancelServoMaintenance() {
    portENTER_CRITICAL(&lock); cancel = true; portEXIT_CRITICAL(&lock);
}
uint32_t takeServoMaintenanceChangedMask() {
    portENTER_CRITICAL(&lock); const uint32_t mask=changedMask; changedMask=0; portEXIT_CRITICAL(&lock); return mask;
}
bool stepServoMaintenance(SystemRuntimeContext* context, uint32_t nowMs) {
    uint8_t local[12]; bool hasRequest = false, wasActive;
    portENTER_CRITICAL(&lock);
    wasActive = active;
    if (cancel || (active && uint32_t(nowMs - lastRequestMs) > 3000)) {
        active = false; session = 0; cancel = false;
    }
    if (queued) { memcpy(local, request, 12); queued = false; executing = true; hasRequest = true; }
    const bool held = active;
    portEXIT_CRITICAL(&lock);
    if (wasActive && !held) servoDiscoveryManager().begin();
    if (!hasRequest) return held;
    uint8_t response[16] = {}; memcpy(response, local, 12); response[14] = 3;
    const uint8_t op=local[8], bus=local[9], oldId=local[10], newId=local[11];
    const uint32_t requestedSession = readMaintenanceU32(local+4);
    uint8_t status = 0;
    const bool stopped = !sharedAtomicLoad(&context->control_enabled) &&
        !sharedAtomicLoad(&context->servo_target_owner) &&
        sharedAtomicLoad(&context->system_state) != SYSTEM_STATE_ROPE_INIT_RUNNING &&
        sharedAtomicLoad(&context->system_state) != SYSTEM_STATE_RUN_DIRECT &&
        sharedAtomicLoad(&context->system_state) != SYSTEM_STATE_RUN_FEEDBACK &&
        sharedAtomicLoad(&context->pending_direct_valid) == 0 &&
        sharedAtomicLoad(&context->direct_target_request_generation) == sharedAtomicLoad(&context->direct_target_result_generation) &&
        sharedAtomicLoad(&context->restore_multi_turn_request_token) == sharedAtomicLoad(&context->restore_multi_turn_result_token) &&
        sharedAtomicLoad(&context->servo_rebase_zero_request_token) == sharedAtomicLoad(&context->servo_rebase_zero_result_token);
    bool duplicate=false;
    for(uint16_t i=0;i<cachedCount;++i) if(readMaintenanceU32(cachedRequests[i])==readMaintenanceU32(local) &&
        readMaintenanceU32(cachedRequests[i]+4)==requestedSession) {
        duplicate=true;
        if(!memcmp(cachedRequests[i],local,12)) {
            portENTER_CRITICAL(&lock);
            memcpy(result,cachedResults[i],16); ready=true; executing=false;
            portEXIT_CRITICAL(&lock); return held;
        }
    }
    if (duplicate || !readMaintenanceU32(local) || op > 4) status = 3;
    else if (op == 0) response[13] = 1;
    else if (!stopped) status = 2;
    else if (op == 1) {
        if (!requestedSession || (held && session != requestedSession)) status = 2;
        else {
            if(cachedSession!=requestedSession) { cachedCount=0; cachedNext=0; cachedSession=requestedSession; }
            portENTER_CRITICAL(&lock); session=requestedSession; active=true; lastRequestMs=nowMs; portEXIT_CRITICAL(&lock);
        }
    } else if (op==2 && !held) { /* idempotent cleanup after STOP/lease expiry */ }
    else if (!held || requestedSession != session) status = 6;
    else if (op == 2) {
        portENTER_CRITICAL(&lock); active=false; session=0; portEXIT_CRITICAL(&lock);
        servoDiscoveryManager().begin();
    } else if (op > 4 || bus >= NUM_BUSES || oldId >= 254 || (op == 4 && newId >= 254)) status = 3;
    else {
        lastRequestMs=nowMs;
        auto* driver = servoBusRegistry().driver(bus);
        if (!driver) status = 3;
        else if (op == 3) {
            response[13] = driver->probeMaintenanceId(oldId);
            if(response[13]) response[15]=driver->maintenanceIdLocked(oldId);
        }
        else {
            uint32_t affected = 0;
            if (oldId < SERVO_TOTAL_NUM) affected |= 1UL << oldId;
            if (newId < SERVO_TOTAL_NUM) affected |= 1UL << newId;
            invalidateAutomaticCalibration(context, jointMaskForMotorMask(affected));
            sharedAtomicFetchOr(&context->multi_turn_unknown_bitmap, affected);
            portENTER_CRITICAL(&lock); changedMask |= affected; portEXIT_CRITICAL(&lock);
            for(uint8_t b=0;b<NUM_BUSES;++b) {
                auto* affectedDriver=servoBusRegistry().driver(b);
                if(affectedDriver) { affectedDriver->discardMaintenanceState(oldId); affectedDriver->discardMaintenanceState(newId); }
            }
            status = driver->renameMaintenanceId(oldId, newId, response+15);
        }
    }
    response[12] = status;
    if(op==1 || op==2 || op==4) {
        memcpy(cachedRequests[cachedNext],local,12); memcpy(cachedResults[cachedNext],response,16);
        if(cachedCount<512) ++cachedCount;
        cachedNext=(cachedNext+1)%512;
    }
    portENTER_CRITICAL(&lock);
    memcpy(result,response,16); ready=true; executing=false;
    const bool stillActive=active;
    portEXIT_CRITICAL(&lock);
    return stillActive || hasRequest;
}
void publishServoMaintenance() {
    uint8_t frame[20] = {PROTOCOL_HEADER,18,PACKET_TYPE_SERVO_MAINTENANCE};
    portENTER_CRITICAL(&lock);
    if (!ready) { portEXIT_CRITICAL(&lock); return; }
    memcpy(frame+3,result,16); sending=true;
    portEXIT_CRITICAL(&lock);
    frame[19]=PROTOCOL_TAIL;
    const bool sent=FW_SERIAL_WRITE(frame,sizeof(frame))==sizeof(frame);
    portENTER_CRITICAL(&lock); if(sent) ready=false; sending=false; portEXIT_CRITICAL(&lock);
}
