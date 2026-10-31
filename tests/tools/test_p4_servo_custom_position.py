"""Exercise production STS correction, HAL and host restore routing with fake UARTs."""

from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path

import pytest

from tools.precommit_checks import compiler_environment


REPO_ROOT = Path(__file__).resolve().parents[2]
P4_SRC = REPO_ROOT / "ESP32-P4/src"
SDK_SRC = REPO_ROOT / "ESP32-P4/src/hal/servo/FTServo_Arduino/src"


ARDUINO_STUB = r'''
#pragma once
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <stdexcept>
#include <vector>

inline unsigned long fakeMilliseconds = 100;
inline unsigned long millis() { return ++fakeMilliseconds; }
inline void delay(unsigned long value) { fakeMilliseconds += value; }
inline constexpr int SERIAL_8N1 = 0;

class HardwareSerial {
public:
    std::deque<unsigned char> received;
    std::vector<unsigned char> transmitted;
    std::vector<unsigned char> pending;
    std::function<void(const std::vector<unsigned char>&)> onPacket;
    int transmitBudget = -1;

    void setRxBufferSize(std::size_t) {}
    void begin(std::uint32_t, int, int, int) {}
    void end() {}
    void flush(bool = true) {}
    int available() const { return int(received.size()); }
    int read() {
        if (received.empty()) return -1;
        const int value = received.front();
        received.pop_front();
        return value;
    }
    std::size_t write(const unsigned char* data, std::size_t count) {
        const auto accepted = transmitBudget < 0 ? count
            : std::min(count, std::size_t(transmitBudget));
        if (transmitBudget >= 0) transmitBudget -= int(accepted);
        transmitted.insert(transmitted.end(), data, data + accepted);
        pending.insert(pending.end(), data, data + accepted);
        while (pending.size() >= 4) {
            if (pending[0] != 0xff || pending[1] != 0xff)
                throw std::runtime_error("production UART emitted an invalid header");
            const auto frameSize = std::size_t(pending[3]) + 4;
            if (pending.size() < frameSize) break;
            std::vector<unsigned char> frame(pending.begin(), pending.begin() + frameSize);
            pending.erase(pending.begin(), pending.begin() + frameSize);
            if (onPacket) onPacket(frame);
        }
        return accepted;
    }
};

inline HardwareSerial Serial1, Serial2, Serial3, Serial4;
'''

# FeetechServoDriver.h now includes FirmwareLog.h, which pulls in the FreeRTOS
# kernel headers. The native harness only needs the types and calls those
# headers name; no scheduler runs, so the guard short-circuits every lock.
# Only FreeRTOS.h defines anything: the sibling headers name each other's
# symbols, so repeating the body in all four would define it four times.
FREERTOS_STUB = r'''
#pragma once
#include <cstddef>
#include <cstdint>

using TickType_t = uint32_t;
using BaseType_t = int;
using UBaseType_t = unsigned;
using QueueHandle_t = void*;
using SemaphoreHandle_t = void*;
using TaskHandle_t = void*;

#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define portMAX_DELAY 0xFFFFFFFFu
#define configMAX_PRIORITIES 32

// Scheduler states: the harness never starts one, which is the case
// FirmwareLog.h treats as "do not lock".
#define taskSCHEDULER_NOT_STARTED 0
#define taskSCHEDULER_RUNNING 1
#define taskSCHEDULER_SUSPENDED 2

inline BaseType_t xTaskGetSchedulerState() { return taskSCHEDULER_NOT_STARTED; }
inline BaseType_t xSemaphoreTake(SemaphoreHandle_t, TickType_t) { return pdTRUE; }
inline BaseType_t xSemaphoreGive(SemaphoreHandle_t) { return pdTRUE; }
#define pdMS_TO_TICKS(ms) (static_cast<TickType_t>(ms))
'''

FREERTOS_SHELL = r'''
#pragma once
#include "FreeRTOS.h"
'''

FREERTOS_HEADERS = ("FreeRTOS.h", "task.h", "queue.h", "semphr.h")


NATIVE_SOURCE = r'''
#include "hal/servo/FeetechServoDriver.h"
#include "tasks/servo_bus/ServoAbsoluteRestore.h"
#include "hal/serial/HostSerialPort.h"
#include <climits>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>

// FirmwareLog.h routes its output through HostSerialPort, whose implementation
// talks to the ESP32 USB-CDC peripheral. The harness only needs the two text
// sinks it declares, so provide them here instead of compiling the hardware file.
// No serial mutex exists in the harness, so the take/give guards fall through.
void HostSerialPort::print(const char* text) { (void)text; }
void HostSerialPort::println(const char* text) { (void)text; }
SemaphoreHandle_t fwSerialMutex() { return nullptr; }

using Bytes = std::vector<unsigned char>;

static void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}

static std::vector<std::string> diagnosticLogs;
namespace p4::platform::time {
    static uint32_t diagnosticNowMs = 0;
    static uint32_t monotonicMillis() { return diagnosticNowMs; }
}
static void captureDiagnosticLog(const char* format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    const int size = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    require(size >= 0 && size < int(sizeof(buffer)), "diagnostic text was truncated");
    diagnosticLogs.emplace_back(buffer);
}
#define FW_LOG_WARN_PRINTF(...) captureDiagnosticLog(__VA_ARGS__)
/* PRODUCTION_READ_DIAGNOSTICS */

static unsigned char checksum(const Bytes& bytes, std::size_t end) {
    unsigned sum = 0;
    for (std::size_t i = 2; i < end; ++i) sum += bytes[i];
    return static_cast<unsigned char>(~sum);
}

static unsigned encodePosition(int position) {
    return position < 0 ? unsigned(-position) | 0x8000U : unsigned(position);
}

enum class Fault {
    None, AckMissing, AckWrongId, AckWrongLength, AckChecksum, AckStatus, AckTruncated,
    ReadMissing, ReadWrongId, ReadWrongLength, ReadChecksum, ReadStatus, ReadTruncated,
    ReadMismatch, ReadOneAfterOffline, ReadModulo,
    TorqueStatus, TorqueMissing, LimitsMismatch, LimitsMissing, LimitsStatus,
    ResetAckMissing, ResetAckStatus
};

struct Device {
    Bytes config = {1,0,0,0,0,0,0,0,0,0,0x95};
    Bytes version = {3,10};
    Bytes profile = {254,254,0};
    // Registers 34..36 hold the servo's own overload protection: protection
    // time ×10 ms, torque percent, and the load unload condition. The driver
    // writes 200/95 and enables unload on connect, so the matching defaults
    // describe an already configured servo.
    Bytes overload = {0,200,95};
    unsigned char unloadCondition = 0x20;
    bool locked = true;
    bool ignoreProfileWrites = false, configAckMissing = false;
    bool versionMissing = false, lockFails = false;
    int hardwarePosition = 12488;
    int previousHardwarePosition = 12488;
    int resetReadbackPosition = INT_MIN;
    Fault fault = Fault::None;
    bool corrected = false;
    bool torqueOff = false;
    bool requireTorqueOff = false;
    unsigned corrections = 0;
    unsigned resets = 0;
    unsigned positionReads = 0;
    unsigned positionReadsAfterCorrection = 0;
    unsigned validPositionResponsesAfterCorrection = 0;
    std::vector<Bytes> frames;

    static void enqueue(HardwareSerial& serial, const Bytes& response) {
        serial.received.insert(serial.received.end(), response.begin(), response.end());
    }

    static Bytes response(unsigned char id, unsigned char status, const Bytes& payload = {}) {
        Bytes bytes{0xff, 0xff, id, static_cast<unsigned char>(payload.size() + 2), status};
        bytes.insert(bytes.end(), payload.begin(), payload.end());
        bytes.push_back(checksum(bytes, bytes.size()));
        return bytes;
    }

    void ack(HardwareSerial& serial, unsigned char id, bool correction, bool torque,
             bool reset = false) {
        if (id == 0xfe || (configAckMissing && !correction)) return;
        if ((correction && fault == Fault::AckMissing) ||
            (torque && fault == Fault::TorqueMissing) ||
            (reset && fault == Fault::ResetAckMissing)) return;
        const unsigned char status =
            ((correction && fault == Fault::AckStatus) ||
             (torque && fault == Fault::TorqueStatus) ||
             (reset && fault == Fault::ResetAckStatus)) ? 4 : 0;
        auto bytes = response(id, status);
        if (correction && fault == Fault::AckWrongId) {
            bytes[2] ^= 1;
            bytes.back() = checksum(bytes, bytes.size() - 1);
        }
        if (correction && fault == Fault::AckWrongLength) {
            bytes[3] = 3;
            bytes.back() = checksum(bytes, bytes.size() - 1);
        }
        if (correction && fault == Fault::AckChecksum) bytes.back() ^= 1;
        if (correction && fault == Fault::AckTruncated) bytes.pop_back();
        enqueue(serial, bytes);
    }

    void receive(HardwareSerial& serial, const Bytes& frame) {
        require(frame.size() == std::size_t(frame[3]) + 4, "incorrect request frame length");
        require(frame.back() == checksum(frame, frame.size() - 1), "incorrect request checksum");
        frames.push_back(frame);
        const auto id = frame[2];
        const auto instruction = frame[4];
        if (instruction == 0x0b && frame[3] == 4) {
            require(frame.size() == 8, "custom correction must contain exactly two parameters");
            if (requireTorqueOff) require(torqueOff, "HAL corrected position with torque enabled");
            const unsigned encoded = unsigned(frame[5]) | unsigned(frame[6]) << 8;
            hardwarePosition = encoded & 0x8000U ? -int(encoded & 0x7fffU) : int(encoded);
            // A missing response may follow a completed write. Failures must remain untrusted.
            corrected = true;
            ++corrections;
            ack(serial, id, true, false);
            return;
        }
        if (instruction == 0x0a) {
            ++resets;
            hardwarePosition = resetReadbackPosition == INT_MIN
                ? ((hardwarePosition % 4096) + 4096) % 4096 : resetReadbackPosition;
            ack(serial, id, false, false, true);
            return;
        }
        if (instruction == 0x02) {
            require(frame.size() == 8, "unexpected read request");
            const auto address = frame[5];
            const auto size = frame[6];
            const bool positionRead = address == SMS_STS_PRESENT_POSITION_L;
            const bool limitsRead = address == SMS_STS_MIN_ANGLE_LIMIT_L;
            if (positionRead) ++positionReads;
            if (positionRead && corrected) ++positionReadsAfterCorrection;
            if (positionRead && corrected && fault == Fault::ReadOneAfterOffline) {
                const unsigned failuresBeforeOneGoodFrame =
                    3 * (3 - 1);
                require(failuresBeforeOneGoodFrame >=
                        mmhand::config::servo::kOfflineInvalidateMisses,
                        "read fault schedule must invalidate online before its one good frame");
                // Fail the earlier complete attempts, allow one frame in the last
                // attempt, then fail again. One frame cannot satisfy online debounce.
                if (positionReadsAfterCorrection != failuresBeforeOneGoodFrame + 1)
                    return;
            }
            if ((positionRead && corrected && fault == Fault::ReadMissing) ||
                (limitsRead && fault == Fault::LimitsMissing)) return;
            Bytes payload(size, 0);
            if (address == 0) { if (versionMissing) return; payload = version; }
            if (address == 8) payload = config;
            if (address == 84) payload = profile;
            if (address == 34) payload = overload;
            if (address == 19) payload[0] = unloadCondition;
            if (address == SMS_STS_LOCK) payload[0] = locked;
            if (address == SMS_STS_TORQUE_ENABLE) payload[0] = !torqueOff;
            if (positionRead) {
                require(size >= 2, "position read omitted a position byte");
                const int position = corrected && fault == Fault::ReadMismatch
                    ? previousHardwarePosition : hardwarePosition;
                const unsigned encoded = encodePosition(corrected && fault == Fault::ReadModulo ? ((position % 4096) + 4096) % 4096 : position);
                payload[0] = static_cast<unsigned char>(encoded);
                payload[1] = static_cast<unsigned char>(encoded >> 8);
            }
            if (limitsRead && fault == Fault::LimitsMismatch) payload[0] = 1;
            const unsigned char status =
                ((positionRead && corrected && fault == Fault::ReadStatus) ||
                 (limitsRead && fault == Fault::LimitsStatus)) ? 4 : 0;
            auto bytes = response(id, status, payload);
            if (positionRead && corrected && fault == Fault::ReadWrongId) {
                bytes[2] ^= 1;
                bytes.back() = checksum(bytes, bytes.size() - 1);
            }
            if (positionRead && corrected && fault == Fault::ReadWrongLength) {
                bytes[3] = static_cast<unsigned char>(bytes[3] + 1);
                bytes.back() = checksum(bytes, bytes.size() - 1);
            }
            if (positionRead && corrected && fault == Fault::ReadChecksum) bytes.back() ^= 1;
            if (positionRead && corrected && fault == Fault::ReadTruncated) bytes.pop_back();
            if (positionRead && corrected && fault == Fault::ReadOneAfterOffline)
                ++validPositionResponsesAfterCorrection;
            enqueue(serial, bytes);
            return;
        }
        if (instruction == 0x03 && frame.size() >= 8 && frame[5] >= 8 && frame[5] <= 18) {
            for (std::size_t n = 6; n + 1 < frame.size() && frame[5] + n - 6 <= 18; ++n)
                config[frame[5] + n - 6 - 8] = frame[n];
        }
        if (instruction == 0x03 && frame.size() >= 8) {
            if (frame[5] == SMS_STS_LOCK) locked = frame[6] && !lockFails;
            if (frame[5] >= 84 && frame[5] <= 86 && !ignoreProfileWrites) profile[frame[5]-84] = frame[6];
            // The driver writes the overload registers while configuring.
            if (frame[5] >= 34 && frame[5] <= 36) overload[frame[5]-34] = frame[6];
            if (frame[5] == 19) unloadCondition = frame[6];
        }
        const bool torque = instruction == 0x03 && frame.size() >= 8 &&
            frame[5] == SMS_STS_TORQUE_ENABLE;
        if (torque) torqueOff = frame[6] == 0;
        ack(serial, id, false, torque);
    }
};

static Fault namedFault(const std::string& suffix) {
    if (suffix == "ack_missing") return Fault::AckMissing;
    if (suffix == "ack_wrong_id") return Fault::AckWrongId;
    if (suffix == "ack_wrong_length") return Fault::AckWrongLength;
    if (suffix == "ack_checksum") return Fault::AckChecksum;
    if (suffix == "ack_status") return Fault::AckStatus;
    if (suffix == "ack_truncated") return Fault::AckTruncated;
    if (suffix == "read_missing") return Fault::ReadMissing;
    if (suffix == "read_wrong_id") return Fault::ReadWrongId;
    if (suffix == "read_wrong_length") return Fault::ReadWrongLength;
    if (suffix == "read_checksum") return Fault::ReadChecksum;
    if (suffix == "read_status") return Fault::ReadStatus;
    if (suffix == "read_truncated") return Fault::ReadTruncated;
    if (suffix == "read_mismatch") return Fault::ReadMismatch;
    if (suffix == "read_one_after_offline") return Fault::ReadOneAfterOffline;
    if (suffix == "torque_status") return Fault::TorqueStatus;
    if (suffix == "torque_missing") return Fault::TorqueMissing;
    if (suffix == "limits_mismatch") return Fault::LimitsMismatch;
    if (suffix == "limits_missing") return Fault::LimitsMissing;
    if (suffix == "limits_status") return Fault::LimitsStatus;
    throw std::runtime_error("unknown fault scenario");
}

static void sdkScenario(const std::string& name) {
    HardwareSerial serial;
    Device device;
    serial.onPacket = [&](const Bytes& bytes) { device.receive(serial, bytes); };
    SMS_STS sdk;
    sdk.pSerial = &serial;
    if (name == "example" || name == "zero" || name == "negative" ||
        name == "signed_limits" || name == "broadcast" || name == "level_zero") {
        int position = name == "zero" ? 0 : name == "negative" ? -200 : 200;
        const unsigned char id = name == "broadcast" ? 0xfe : 1;
        if (name == "level_zero") sdk.Level = 0;
        require(sdk.CalibratePosition(id, static_cast<short>(position)) == 1, "SDK rejected valid correction");
        const Bytes expected = name == "negative"
            ? Bytes{0xff, 0xff, 1, 4, 0x0b, 0xc8, 0x80, 0xa7}
            : name == "zero" ? Bytes{0xff, 0xff, 1, 4, 0x0b, 0, 0, 0xef}
            : name == "broadcast" ? Bytes{0xff, 0xff, 0xfe, 4, 0x0b, 0xc8, 0, 0x2a}
            : Bytes{0xff, 0xff, 1, 4, 0x0b, 0xc8, 0, 0x27};
        require(device.frames.size() == 1 && device.frames[0] == expected, "SDK wire bytes differ from the STS specification");
        if (name == "level_zero") require(sdk.Level == 0, "correction changed the normal reply Level");
        if (name == "signed_limits") {
            require(sdk.CalibratePosition(1, SHRT_MAX) == 1, "positive signed limit rejected");
            require(device.frames.back()[5] == 0xff && device.frames.back()[6] == 0x7f, "positive sign-magnitude encoding changed");
            require(sdk.CalibratePosition(1, -SHRT_MAX) == 1, "negative signed limit rejected");
            require(device.frames.back()[5] == 0xff && device.frames.back()[6] == 0xff, "negative sign-magnitude encoding changed");
        }
        return;
    }
    if (name == "invalid_id" || name == "minimum_invalid") {
        const int result = name == "invalid_id" ? sdk.CalibratePosition(0xff, 0)
            : sdk.CalibratePosition(1, SHRT_MIN);
        require(result == 0 && serial.transmitted.empty(), "invalid SDK request reached the UART");
        return;
    }
    if (name == "legacy_recal") {
        require(sdk.Recal(1) == 1, "legacy no-parameter correction failed");
        require(device.frames[0] == Bytes({0xff, 0xff, 1, 2, 0x0b, 0xf1}), "legacy Recal wire format changed");
        return;
    }
    if (name == "short_tx") {
        serial.transmitBudget = 7;
    } else if (name == "level_zero_missing") {
        sdk.Level = 0;
        device.fault = Fault::AckMissing;
    } else {
        device.fault = namedFault(name);
    }
    require(sdk.CalibratePosition(1, 200) == 0, "SDK accepted an unconfirmed or partial transaction");
    if (name == "level_zero_missing") require(sdk.Level == 0, "failed correction changed reply Level");
}


static void halScenario(const std::string& name) {
    Serial1 = HardwareSerial{};
    Device device;
    FeetechServoDriver driver;
    driver.begin(0, 1, 2, 1000000);
    Serial1.onPacket = [&](const Bytes& frame) { device.receive(Serial1, frame); };
    for (int i=0;i<3;++i) require(driver.probeFeedback(1), "initial feedback missing");
    if (name == "configuration") { device.config[0] = 0; device.config[10] = 0x85; device.config[3] = 0xff; }
    if (name == "firmware_old") device.version = {3,9};
    if (name == "firmware_new") device.version = {4,0};
    if (name == "firmware_minor") device.version = {3,11};
    if (name == "version_missing") device.versionMissing = true;
    if (name == "profile" || name == "profile_ignored" || name == "config_ack_missing") device.profile = {1,2,3};
    if (name == "profile_ignored") device.ignoreProfileWrites = true;
    if (name == "config_ack_missing") device.configAckMissing = true;
    if (name == "lock_failure") { device.profile = {1,2,3}; device.lockFails = true; }
    if (name == "firmware_old" || name == "version_missing" || name == "lock_failure" || name == "profile_ignored") {
        require(!driver.checkOnlineConfiguration(1), "invalid motor passed online check");
        const auto expected = name == "firmware_old" ? ServoOnlineCheck::UnsupportedFirmware :
            name == "version_missing" ? ServoOnlineCheck::VersionReadFailed : ServoOnlineCheck::ConfigurationFailed;
        require(driver.getFeedback(1).onlineCheck == expected, "online failure classified incorrectly");
        require(!driver.isMultiTurnConfigured(1), "failed motor configured");
        if (name != "lock_failure" && name != "profile_ignored") for (const auto& f : device.frames)
            require(f[4] != 3, "incompatible or unreadable version changed EEPROM");
        return;
    }
    const auto before = device.frames.size();
    require(driver.checkOnlineConfiguration(1), "hardware configuration failed");
    require(device.config[0] == 1 && device.config[10] == 0x95 && device.config[3] == 0,
            "configuration lost direction bits or did not enable full-angle feedback");
    require(device.profile == Bytes({254,254,0}), "speed/acceleration profile differs");
    require(driver.getFeedback(1).onlineCheck == ServoOnlineCheck::Ready, "online ready not published");
    if (name == "configuration" || name == "profile" || name == "firmware_new" || name == "firmware_minor" || name == "config_ack_missing") return;
    for (size_t i = before; i < device.frames.size(); ++i)
        require(device.frames[i][4] != 3, "verified configuration rewrote EEPROM");
    const auto cached = device.frames.size();
    require(driver.checkOnlineConfiguration(1) && device.frames.size() == cached, "online cache rechecked hardware");
    if (name == "session_recheck") {
        const auto generation = driver.getFeedback(1).onlineSessionGeneration;
        require(driver.invalidateIfFeedbackStale(1, millis() + 1000, 80), "offline transition missing");
        device.version = {3,9};
        for (int i=0; i<3; ++i) require(driver.probeFeedback(1), "reconnect read failed");
        require(driver.getFeedback(1).onlineSessionGeneration == generation + 1, "online generation did not change");
        require(!driver.checkOnlineConfiguration(1) && driver.getFeedback(1).onlineCheck == ServoOnlineCheck::UnsupportedFirmware,
                "reconnect reused cached version/configuration");
        return;
    }
    if (name == "torque_enable_missing") {
        require(driver.setTorqueEnabled(1, false), "torque off failed");
        device.fault = Fault::TorqueMissing;
        require(!driver.queueTarget(1, 100, 400, 20), "unacknowledged torque enable accepted a position target");
        device.fault = Fault::None;
        require(driver.queueTarget(1, 100, 400, 20), "torque enable failure poisoned its retry cache");
        return;
    }
    if (name == "native") {
        for (int position : {8292, 100, 4100, 4090, -73, -8392, 30719, -30719}) {
            device.hardwarePosition = position;
            require(driver.probeFeedback(1), "native sample missing");
            require(driver.getAbsolutePosition(1) == position, "native feedback was unwrapped or masked");
        }
        return;
    }
    p4::servo::AbsoluteRestore writer;
    using Result = p4::servo::RestoreResult;
    if(name=="ack_missing")device.fault=Fault::AckMissing;
    if(name=="ack_status")device.fault=Fault::AckStatus;
    if(name=="torque_missing")device.fault=Fault::TorqueMissing;
    auto first=writer.step(&driver,1,1000);
    if(name=="ack_status" || name=="torque_missing") {
        require(first==Result::Failed,"rejected write accepted");return;
    }
    require(first==Result::Pending && !driver.isAbsoluteReliable(1),"cached sample trusted");
    require(driver.zeroCommandDiagnostic().correctionAck == (name=="ack_missing" ? 0 : 1), "correction ACK evidence lost");
    require(device.corrections==1 && device.resets==0,"wrong command order");
    require(device.hardwarePosition==2048, "incorrect calibration target");
    if(name=="cancel") {writer.cancel(); require(!writer.active(),"cancel failed");return;}
    if(name=="disconnect_pending") {
        driver.invalidateIfFeedbackStale(1,driver.getFeedback(1).lastUpdate+100,80);
        require(writer.step(&driver,1,1250)==Result::Failed && !driver.isAbsoluteReliable(1), "disconnect accepted");
        require(device.corrections==1 && device.resets==0, "disconnect retried a command");
        return;
    }
    if(name=="stale") {
        require(writer.step(&driver,1,1750)==Result::Failed,"missing new feedback did not time out");return;
    }
    if(name=="read_missing")device.fault=Fault::ReadMissing;
    if(name=="read_mismatch")device.fault=Fault::ReadMismatch;
    if(name=="read_status")device.fault=Fault::ReadStatus;
    if(name=="edge_low")device.hardwarePosition=2044;
    if(name=="edge_high")device.hardwarePosition=2052;
    if(name=="outside_low")device.hardwarePosition=2043;
    if(name=="outside_high")device.hardwarePosition=2053;
    if(name=="wrong_turn_positive")device.hardwarePosition=6144;
    if(name=="wrong_turn_negative")device.hardwarePosition=-2048;
    driver.probeFeedback(1);
    require(writer.step(&driver,1,1249)==Result::Pending,"position accepted before settle");
    const bool bad=name=="read_missing" || name=="read_mismatch" ||
        name=="outside_low" || name=="outside_high" || name=="wrong_turn_positive" || name=="wrong_turn_negative";
    // A non-zero status byte means the servo reports its own fault, not a
    // failed read: the position it returned is still authoritative, so
    // read_status keeps its coordinate trust and stays a successful step.
    const auto status=writer.step(&driver,1,bad?1750:1250);
    require(status==(bad?Result::Failed:Result::Success),"full-coordinate verification incorrect");
    require(driver.isAbsoluteReliable(1)==!bad,"incorrect coordinate trust");
    require(device.corrections==1 && device.resets==0,"commands repeated or reset sent");
    if(name=="disconnect") {
        driver.invalidateIfFeedbackStale(1,driver.getFeedback(1).lastUpdate+100,80);
        require(!driver.isAbsoluteReliable(1),"offline retained trust");
    }

}

static void diagnosticScenario(const std::string& name) {
    FeetechServoDriver driver;
    driver.begin(2, 1, 2, 1000000);
    const bool sync = name.rfind("sync_", 0) == 0;
    const auto fault = name.substr(sync ? 5 : 6);
    Serial3.onPacket = [&](const Bytes& frame) {
        if (fault == "timeout") return;
        const auto reply = [&](unsigned char id, unsigned size) {
            if (fault == "stale_status" && id == 6) return;
            auto bytes = Device::response(id, fault == "status" || fault == "stale_status" ? 4 : 0,
                                          Bytes(size, 0));
            if (fault == "checksum") bytes.back() ^= 1;
            if (fault == "length") { bytes[3]++; bytes.back() = checksum(bytes, bytes.size()-1); }
            if (fault == "wrong_id") { bytes[2]++; bytes.back() = checksum(bytes, bytes.size()-1); }
            if (fault == "partial" || (fault == "partial_batch" && id == 6)) bytes.resize(7); // Header plus two payload bytes.
            Device::enqueue(Serial3, bytes);
        };
        if (sync) {
            require(frame[4] == INST_SYNC_READ, "diagnostics generated an extra UART command");
            for (std::size_t i=7; i+1<frame.size(); ++i) reply(frame[i], frame[6]);
        } else {
            require(frame[4] == INST_READ, "probe diagnostics generated an extra UART command");
            reply(frame[2], frame[6]);
        }
    };
    if (sync) {
        const uint8_t ids[]{5,6};
        const uint8_t count = fault == "stale_status" || fault == "partial_batch" ? 2 : 1;
        require(driver.beginSyncRead(ids, count, 123, 1000), "sync begin failed");
        const auto result = driver.collectSyncRead(6000);
        require(result.successCount ==
                    (fault == "good" || fault == "status" || fault == "partial_batch" ||
                     fault == "stale_status" ? 1 : 0),
                "sync admitted failed feedback");
    } else {
        require(driver.probeFeedback(5) == (fault == "good" || fault == "status"),
                "probe admitted failed feedback");
    }
    const uint8_t id = fault == "stale_status" || fault == "partial_batch" ? 6 : 5;
    const auto& diagnostic = driver.getReadDiagnostic(id);
    const auto expectedError = fault == "checksum" ? ERR_CRC_CMP :
        fault == "length" ? ERR_BUFF_LEN : fault == "wrong_id" ? ERR_SLAVE_ID :
        fault == "timeout" || fault == "partial" || fault == "partial_batch" || fault == "stale_status" ? ERR_NO_REPLY : 0;
    require(diagnostic.communicationError == expectedError, "read failure cause lost");
    require(diagnostic.failed == (fault != "good" && fault != "status") && diagnostic.sync == sync,
            "read failure mode lost");
    require(diagnostic.reply.statusValid == (fault == "status" || fault == "good"),
            "unverified reply supplied a trusted status byte");
    if (fault == "status") require(diagnostic.motorStatus == 4, "raw motor status lost");
    if (fault == "stale_status") require(diagnostic.motorStatus == 0,
        "missing sync reply inherited another motor's fault status");
    if (fault == "partial" || fault == "partial_batch") require(diagnostic.reply.payloadBytes == 2,
        "partial reply byte count lost");
    if (sync) require(diagnostic.busExpectedBytes == (fault == "stale_status" || fault == "partial_batch" ? 42 : 21) &&
                      diagnostic.sequence == 123, "sync batch evidence lost");
    logServoReadFailure(&driver, 2, id);
    if (fault == "good" || fault == "status") {
        // A non-zero status byte means the servo reports its own fault while
        // answering: the read succeeded, so no read-failure text is emitted.
        require(diagnosticLogs.empty(), "readable feedback generated failure text");
        return;
    }
    require(diagnosticLogs.size() == 1 && diagnosticLogs[0].find("bus=2 id=" + std::to_string(id)) != std::string::npos,
            "failure text lost physical route");
    const std::string reason = fault == "status" ? "motor_status" :
        fault == "partial" || fault == "partial_batch" || fault == "stale_status" ? "timeout" : fault;
    require(diagnosticLogs[0].find("reason=" + reason) != std::string::npos,
            "failure text lost failure reason");
    logServoReadFailure(&driver, 2, id);
    p4::platform::time::diagnosticNowMs = 4999;
    logServoReadFailure(&driver, 2, id);
    require(diagnosticLogs.size() == 1, "repeated failure flooded text logs");
    p4::platform::time::diagnosticNowMs = 5000;
    logServoReadFailure(&driver, 2, id);
    require(diagnosticLogs.size() == 2 && diagnosticLogs.back().find("suppressed=2") != std::string::npos,
            "failure did not resume after throttle interval");
    logServoReadFailure(&driver, 1, id);
    require(diagnosticLogs.size() == 3, "one BUS suppressed another BUS's evidence");
    p4::platform::time::diagnosticNowMs = UINT32_MAX - 100;
    logServoReadFailure(&driver, 2, id);
    p4::platform::time::diagnosticNowMs = 4900;
    logServoReadFailure(&driver, 2, id);
    require(diagnosticLogs.size() == 5, "millis rollover suppressed diagnostics permanently");
}

int main(int argc, char** argv) {
    try {
        require(argc == 2, "scenario required");
        const std::string name(argv[1]);
        if (name.rfind("diag_",0) == 0) diagnosticScenario(name.substr(5));
        else if (name.rfind("sdk_",0) == 0) sdkScenario(name.substr(4));
        else halScenario(name);
        return 0;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}

'''


def _msvc_sdk_source(source: str) -> str:
    """MSVC lacks VLA support; replace only the two unrelated sync-motion bodies.

    The production correction, signed encoding, framing, ACK handling, serial
    transport and feedback methods compile unchanged. The sentinels abort if a
    correction unexpectedly attempts a sync-motion write.
    """
    for method in ("SyncWritePosEx", "SyncWriteSpe"):
        start = source.index(f"void SMS_STS::{method}(")
        opening = source.index("{", start)
        depth = 1
        closing = opening + 1
        while depth:
            depth += (source[closing] == "{") - (source[closing] == "}")
            closing += 1
        source = source[:opening] + "{ std::abort(); }" + source[closing:]
    return "#include <cstdlib>\n" + source


@pytest.fixture(scope="module")
def native_harness(tmp_path_factory: pytest.TempPathFactory) -> tuple[Path, dict[str, str]]:
    env = compiler_environment()
    compiler = next(
        (path for name in ("g++", "clang++", "cl")
         if (path := shutil.which(name, path=env.get("PATH")))),
        None,
    )
    assert compiler is not None, "A native C++ compiler is required; this test may not skip"
    directory = tmp_path_factory.mktemp("servo-custom-position-native")
    include = directory / "include"
    include.mkdir()
    (include / "Arduino.h").write_text(ARDUINO_STUB, encoding="utf-8")
    # FirmwareLog.h reaches the FreeRTOS kernel headers through the servo driver.
    freertos = include / "freertos"
    freertos.mkdir()
    for header in FREERTOS_HEADERS:
        (freertos / header).write_text(
            FREERTOS_STUB if header == "FreeRTOS.h" else FREERTOS_SHELL,
            encoding="utf-8")
    source = directory / "servo_custom_position_test.cpp"
    diagnostics = (P4_SRC / "tasks/servo_bus/ServoReadDiagnostics.cpp").read_text(encoding="utf-8")
    diagnostics = re.sub(r'^#include[^\n]*\n', '', diagnostics, flags=re.MULTILINE)
    source.write_text(NATIVE_SOURCE.replace("/* PRODUCTION_READ_DIAGNOSTICS */", diagnostics), encoding="utf-8")
    executable = directory / "servo_custom_position_test.exe"
    sms_source = SDK_SRC / "SMS_STS.cpp"
    msvc = Path(compiler).name.lower() in {"cl", "cl.exe"}
    if msvc:
        sms_source = directory / "SMS_STS_native.cpp"
        sms_source.write_text(
            _msvc_sdk_source((SDK_SRC / "SMS_STS.cpp").read_text(encoding="utf-8")),
            encoding="utf-8",
        )
    sources = [str(source), str(SDK_SRC / "SCS.cpp"),
               str(SDK_SRC / "SCSerial.cpp"), str(sms_source),
               str(P4_SRC / "hal/servo/FeetechServoDriver.cpp")]
    if msvc:
        command = [compiler, "/std:c++17", "/EHsc", "/utf-8", "/DARDUINO=10819",
                   f"/I{include}", f"/I{P4_SRC}", f"/I{SDK_SRC}",
                   *sources, f"/Fo{directory.resolve()}\\", f"/Fe{executable}"]
    else:
        command = [compiler, "-std=c++17", "-DARDUINO=10819",
                   f"-I{include}", f"-I{P4_SRC}", f"-I{SDK_SRC}",
                   *sources, "-o", str(executable)]
    subprocess.run(command, check=True, cwd=directory, env=env, timeout=120)
    return executable, env


SCENARIOS = (
    "edge_low", "edge_high", "outside_low", "outside_high", "wrong_turn_positive", "wrong_turn_negative", "disconnect_pending",
    "sdk_example", "sdk_zero", "sdk_negative", "sdk_signed_limits",
    "sdk_minimum_invalid", "sdk_invalid_id", "sdk_broadcast", "sdk_level_zero",
    "sdk_level_zero_missing", "sdk_short_tx", "sdk_legacy_recal", "sdk_ack_missing",
    "sdk_ack_wrong_id", "sdk_ack_wrong_length", "sdk_ack_checksum", "sdk_ack_status", "sdk_ack_truncated",
    "firmware_old", "firmware_new", "firmware_minor", "session_recheck", "torque_enable_missing", "version_missing", "profile", "profile_ignored", "config_ack_missing", "lock_failure",
    "configuration", "native", "zero", "ack_missing", "ack_status",
    "torque_missing", "read_missing", "read_mismatch", "read_status", "stale", "cancel", "disconnect",
)

@pytest.mark.parametrize("scenario", SCENARIOS)
def test_production_custom_position_transaction(native_harness, scenario: str) -> None:
    executable, env = native_harness
    subprocess.run([str(executable), scenario], check=True,
                   cwd=executable.parent, env=env, timeout=10)


@pytest.mark.parametrize("scenario", (
    "probe_good", "probe_timeout", "probe_checksum", "probe_wrong_id", "probe_length",
    "probe_status", "probe_partial", "sync_good", "sync_timeout", "sync_checksum",
    "sync_length", "sync_status", "sync_partial", "sync_partial_batch", "sync_stale_status",
))
def test_feedback_failure_evidence_and_log_throttle(native_harness, scenario):
    executable, env = native_harness
    subprocess.run([str(executable), "diag_" + scenario], check=True,
                   cwd=executable.parent, env=env, timeout=10)
