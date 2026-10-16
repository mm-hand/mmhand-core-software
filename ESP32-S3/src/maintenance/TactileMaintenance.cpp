#include "TactileMaintenance.h"
#include "../hal/HalTactile.h"
#include "../hal/Crc16.h"

#include <Arduino.h>
#include <cstring>

namespace {
constexpr uint8_t kHeader = 0xFE, kTail = 0xFF;
constexpr uint8_t kRequestType = 0x31, kResultType = 0x32;
constexpr uint8_t kRequestSize = 12, kResultSize = 22;
uint8_t frame[32] = {};
uint8_t frameSize = 0;
uint8_t lastRequest[kRequestSize] = {};
uint8_t lastResult[kResultSize] = {};
bool haveLast = false;

uint32_t readU32(const uint8_t* data) {
    return (uint32_t(data[0]) << 24) | (uint32_t(data[1]) << 16) |
           (uint32_t(data[2]) << 8) | data[3];
}
void putU32(uint8_t* data, uint32_t value) {
    data[0] = uint8_t(value >> 24); data[1] = uint8_t(value >> 16);
    data[2] = uint8_t(value >> 8); data[3] = uint8_t(value);
}
void publish(const uint8_t* result) {
    uint8_t packet[kResultSize + 4] = {kHeader, kResultSize + 2, kResultType};
    memcpy(packet + 3, result, kResultSize);
    packet[sizeof(packet) - 1] = kTail;
    Serial.write(packet, sizeof(packet));
}

void execute(const uint8_t* request) {
    if (haveLast && !memcmp(lastRequest, request, kRequestSize)) {
        publish(lastResult); // A repeated token never repeats an EEPROM write.
        return;
    }
    uint8_t result[kResultSize] = {};
    result[0] = 2;
    result[1] = request[1];
    memcpy(result + 2, request + 2, 4);
    result[6] = 0;             // stage
    result[7] = 4;             // INVALID_TARGET until validated
    result[8] = 0xFF;          // old physical selector
    result[9] = 0xFF;          // new physical selector
    result[10] = 0xFF;         // model
    result[11] = 0;            // point count
    const uint8_t op = request[1];
    const uint8_t physical = request[6];
    // Reconcile a known interrupted transaction without writing sensor EEPROM.
    // The host supplies the preserved installation model and semantic slot.
    if (op == 2) {
        const uint8_t knownModel = request[7], semanticSlot = request[8];
        if (physical < 15 && semanticSlot < 3 && request[9] == 0 &&
            (knownModel == 1 || knownModel == 2) &&
            tactile.probePhysical(physical) && tactile.verifyPhysicalModel(physical, knownModel)) {
            const uint8_t semanticIndex = (physical / 3) * 3 + semanticSlot;
            result[8] = result[9] = physical;
            result[10] = knownModel; result[11] = knownModel == 1 ? 52 : 25;
            result[13] = 1; result[15] = semanticIndex;
            result[7] = tactile.remapSensor(semanticIndex, physical) ? 0 : 8;
            result[6] = 4;
        }
        putU32(result + 16, millis());
        const uint16_t checksum = crc16Ccitt(result, 20);
        result[20] = uint8_t(checksum >> 8); result[21] = uint8_t(checksum);
        memcpy(lastRequest, request, kRequestSize); memcpy(lastResult, result, kResultSize);
        haveLast = true; publish(result); return;
    }
    uint8_t semantic = 0xFF, model = 0xFF;
    const uint8_t found = physical < 15 ? tactile.inspectSensor(physical, &semantic, &model) : 0;
    if (found == 0) result[7] = 1;              // NONE
    else if (found == 0xFF) result[7] = 3;      // UNKNOWN_MODEL
    else {
        result[7] = 0;                         // FOUND_ONE
        result[8] = physical;
        result[9] = physical;
        result[10] = model;
        result[11] = model == 1 ? 52 : 25;
        result[12] = 1;                         // old CRC-valid
        result[15] = semantic;
        if (op == 1) {
            const uint8_t wanted = request[8];
            const uint8_t target = request[6];
            if (wanted > 2 || target != physical || request[7] != model || request[9] != 0) {
                result[7] = 4;
            } else {
                const uint8_t newPhysical = (physical / 3) * 3 + wanted;
                result[9] = newPhysical;
                if (newPhysical != physical && tactile.probePhysical(newPhysical)) {
                    result[7] = 2;
                } else if (newPhysical == physical) {
                    result[13] = 1;
                } else {
                    bool sent = false;
                    result[6] = 1;
                    if (!tactile.writeSensorId(physical, wanted + 1, &sent)) {
                        result[14] = sent ? 1 : 0;
                        result[7] = sent ? 6 : 8;
                    } else {
                        result[14] = 1;
                        result[6] = 3;
                        bool newValid = false, oldValid = true;
                        const uint32_t verifyStart = millis();
                        do {
                            delay(25);
                            newValid = tactile.probePhysical(newPhysical) && tactile.verifyPhysicalModel(newPhysical, model);
                            oldValid = tactile.probePhysical(physical);
                            if (newValid && !oldValid) break;
                        } while (uint32_t(millis() - verifyStart) < 1000);
                        // Probe repeatedly, but never resend unlock or the ID write.
                        result[12] = oldValid ? 1 : 0;
                        result[13] = newValid ? 1 : 0;
                        if (!newValid || oldValid) result[7] = 6;
                        else if (!tactile.remapSensor(semantic, newPhysical)) result[7] = 8;
                        else { result[6] = 4; result[7] = 0; }
                    }
                }
            }
        } else if (op != 0) {
            result[7] = 4;
        }
    }
    putU32(result + 16, millis());
    const uint16_t crc = crc16Ccitt(result, 20);
    result[20] = uint8_t(crc >> 8); result[21] = uint8_t(crc);
    memcpy(lastRequest, request, kRequestSize);
    memcpy(lastResult, result, kResultSize);
    haveLast = true;
    publish(result);
}
} // namespace

bool pollTactileMaintenance() {
    bool processed = false;
    while (Serial.available() > 0) {
        const int incoming = Serial.read();
        if (incoming < 0) break;
        const uint8_t value = uint8_t(incoming);
        if (!frameSize) {
            if (value == kHeader) frame[frameSize++] = value;
            continue;
        }
        if (frameSize >= sizeof(frame)) { frameSize = 0; continue; }
        frame[frameSize++] = value;
        if (frameSize == 2 && value != kRequestSize + 2) { frameSize = 0; continue; }
        if (frameSize < kRequestSize + 4) continue;
        if (frameSize == kRequestSize + 4 && frame[2] == kRequestType &&
            frame[frameSize - 1] == kTail && frame[3] == 2) {
            const uint8_t* body = frame + 3;
            const uint16_t expected = crc16Ccitt(body, 10);
            if (body[10] == uint8_t(expected >> 8) && body[11] == uint8_t(expected)) {
                execute(body);
                processed = true;
            }
        }
        frameSize = 0;
    }
    return processed;
}
