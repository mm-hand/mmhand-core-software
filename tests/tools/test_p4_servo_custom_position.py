"""Exercise production STS correction, HAL and host restore routing with fake UARTs."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

from tools.precommit_checks import compiler_environment


REPO_ROOT = Path(__file__).resolve().parents[2]
P4_SRC = REPO_ROOT / "ESP32-P4/ServoBoardMain/src"
SDK_SRC = REPO_ROOT / "ESP32-P4/libraries/FTServo_Arduino/src"


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


NATIVE_SOURCE = r'''
#include "hal/servo/FeetechServoDriver.h"
#include <climits>
#include <cstdlib>
#include <iostream>
#include <string>

using Bytes = std::vector<unsigned char>;

static void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}

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
    ReadMismatch, ReadOneAfterOffline,
    TorqueStatus, TorqueMissing, LimitsMismatch, LimitsMissing, LimitsStatus,
    ResetAckMissing, ResetAckStatus
};

struct Device {
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
        if (id == 0xfe) return;
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
                    mmhand::config::servo::kResetZeroFeedbackAttempts *
                    (mmhand::config::servo::kResetZeroTransactionAttempts - 1);
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
            if (positionRead) {
                require(size >= 2, "position read omitted a position byte");
                const int position = corrected && fault == Fault::ReadMismatch
                    ? previousHardwarePosition : hardwarePosition;
                const unsigned encoded = encodePosition(position);
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

static void establishNativePosition(FeetechServoDriver& driver, Device& device,
                                    uint8_t id, int position, bool configured) {
    short raw = -1;
    require(driver.calibrateHardwareAbsolutePosition(id, position, &raw) &&
            driver.probeFeedback(id), "fixture could not establish verified native ABS");
    require(device.hardwarePosition == position && driver.getAbsolutePosition(id) == position &&
            driver.getHardwareAbsolutePosition(id) == position && driver.isAbsoluteReliable(id),
            "fixture substituted a cached coordinate for native position correction");
    if (!configured) {
        // A new online session clears the HAL mode cache. Prepare this through
        // actual public session transitions, without exposing private HAL fields.
        require(driver.invalidateIfFeedbackStale(id, driver.getFeedback(id).lastUpdate + 2, 1),
                "fixture could not invalidate its configured session");
        for (unsigned read = 0; read < mmhand::config::servo::kOnlineConfirmReads; ++read)
            require(driver.probeFeedback(id), "fixture unconfigured reconnect failed");
        require(driver.isOnline(id) && !driver.isMultiTurnConfigured(id),
                "fixture reconnect retained its old mode cache");
    }
    driver.finishRecoveryAttempt(id, true, static_cast<uint32_t>(millis()));
    // Setup instructions must not count as writes made by the scenario itself.
    device.corrected = false;
    device.corrections = 0;
    device.resets = 0;
    device.positionReads = 0;
    device.positionReadsAfterCorrection = 0;
    device.validPositionResponsesAfterCorrection = 0;
    device.frames.clear();
}

struct HalFixture {
    FeetechServoDriver driver;
    Device device;
    explicit HalFixture(bool configured = true, int hardware = 12488) {
        Serial1 = HardwareSerial{};
        device.hardwarePosition = device.previousHardwarePosition = hardware;
        device.requireTorqueOff = true;
        Serial1.onPacket = [&](const Bytes& bytes) { device.receive(Serial1, bytes); };
        driver.begin(0, 21, 20, 1000000);
        for (unsigned i = 0; i < mmhand::config::servo::kOnlineConfirmReads; ++i)
            require(driver.probeFeedback(1), "fixture could not establish production feedback");
        require(driver.isOnline(1), "fixture did not establish online state");
        establishNativePosition(driver, device, 1, hardware, configured);
        Serial1.transmitted.clear();
    }
};

static void halScenario(const std::string& name) {
    if (name == "unbound") {
        FeetechServoDriver driver;
        short raw = -123;
        require(!driver.calibrateHardwareAbsolutePosition(1, 0, &raw), "unbound HAL accepted correction");
        return;
    }
    if (name == "reset_uncertain_ack") {
        for (Fault fault : {Fault::ResetAckMissing, Fault::ResetAckStatus}) {
            HalFixture fixture;
            auto& driver = fixture.driver;
            auto& device = fixture.device;
            require(driver.setTarget(1, 12500, 400, 20),
                    "RESET fixture could not queue an old-coordinate target");
            device.fault = fault;
            short raw = -123;
            bool attempted = false;
            require(!driver.resetTurnCounter(1, &raw, &attempted) && attempted,
                    "HAL accepted an unconfirmed RESET or lost its attempted flag");
            require(device.resets == 1 && device.corrections == 0 &&
                    device.hardwarePosition == 200,
                    "uncertain RESET did not include an actual hardware coordinate change");
            require(raw == -123 && !driver.isAbsoluteReliable(1) &&
                    !driver.isMultiTurnConfigured(1) &&
                    !driver.getFeedback(1).offlineTrustedSnapshotPending,
                    "uncertain RESET retained a trusted old coordinate or snapshot");
            require(!driver.setTarget(1, 200, 400, 20),
                    "unconfirmed RESET allowed a new motor target");
            const auto frameCount = device.frames.size();
            const auto transmittedCount = Serial1.transmitted.size();
            driver.syncWriteAll();
            require(device.frames.size() == frameCount &&
                    Serial1.transmitted.size() == transmittedCount,
                    "unconfirmed RESET emitted the queued old-coordinate target");
        }
        return;
    }
    if (name == "reset_torque_failure") {
        for (Fault fault : {Fault::TorqueMissing, Fault::TorqueStatus}) {
            HalFixture fixture;
            auto& driver = fixture.driver;
            auto& device = fixture.device;
            device.fault = fault;
            short raw = -123;
            bool attempted = true;
            require(!driver.resetTurnCounter(1, &raw, &attempted) && !attempted,
                    "failed torque precondition attempted a RESET");
            require(raw == -123 && device.resets == 0 && device.corrections == 0 &&
                    device.hardwarePosition == 12488 &&
                    driver.getAbsolutePosition(1) == 12488 &&
                    driver.getHardwareAbsolutePosition(1) == 12488 &&
                    driver.isAbsoluteReliable(1) && driver.isMultiTurnConfigured(1),
                    "pre-RESET torque failure invalidated an unchanged native coordinate");
            require(driver.setTarget(1, 12500, 400, 20),
                    "unchanged trusted coordinate stopped accepting valid targets");
        }
        return;
    }
    HalFixture fixture(name.rfind("limits_", 0) != 0 && name != "unconfigured");
    auto& driver = fixture.driver;
    auto& device = fixture.device;
    short raw = -123;
    unsigned char attempts = 99;
    if (name == "invalid_id" || name == "range" || name == "null_output") {
        if (name == "invalid_id") {
            for (unsigned char id : {static_cast<unsigned char>(27), static_cast<unsigned char>(254), static_cast<unsigned char>(255)})
                require(!driver.calibrateHardwareAbsolutePosition(id, 0, &raw, &attempts), "HAL accepted unsupported or broadcast ID");
        } else if (name == "range") {
            for (int value : {30720, -30720, INT_MAX, INT_MIN})
                require(!driver.calibrateHardwareAbsolutePosition(1, value, &raw, &attempts), "HAL truncated an out-of-range position");
        } else {
            require(!driver.calibrateHardwareAbsolutePosition(1, 0, nullptr, &attempts), "HAL accepted missing readback output");
        }
        require(device.frames.empty(), "invalid HAL request accessed the physical UART");
        return;
    }
    if (name == "legacy_restore" || name == "reset_multiturn_readback" ||
        name == "reset_negative_readback") {
        const int expected = name == "reset_multiturn_readback" ? 8392
            : name == "reset_negative_readback" ? -8392 : 200;
        if (name != "legacy_restore") device.resetReadbackPosition = expected;
        require(driver.resetTurnCounter(1, &raw), "native RESET transaction failed");
        require(device.resets == 1 && device.corrections == 0,
                "native RESET substituted custom position correction");
        require(device.positionReads > 0 && device.hardwarePosition == expected &&
                driver.getHardwareAbsolutePosition(1) == expected &&
                driver.getAbsolutePosition(1) == expected,
                "RESET replaced actual fresh ABS feedback with a normalized cached position");
        require(raw == ((expected % 4096) + 4096) % 4096 &&
                driver.getRawPosition(1) == raw && driver.isAbsoluteReliable(1),
                "RESET did not establish the actual ABS and single-turn raw together");
        return;
    }
    if (name == "zero" || name == "positive" || name == "negative" ||
        name == "hardware_limits" || name == "unconfigured") {
        const int position = name == "positive" ? 200 : name == "negative" ? -200 : 0;
        require(driver.calibrateHardwareAbsolutePosition(1, position, &raw, &attempts), "HAL correction failed");
        require(device.corrections == 1 && device.resets == 0, "HAL substituted RESET for custom correction");
        require(attempts == 1 && device.positionReadsAfterCorrection > 0, "HAL did not verify a fresh correction readback");
        require(driver.getHardwareAbsolutePosition(1) == position, "HAL reused the previous hardware unwrap anchor");
        require(driver.getAbsolutePosition(1) == device.hardwarePosition,
                "HAL ABS differs from the actual corrected servo position");
        require(raw == ((position % 4096) + 4096) % 4096, "HAL returned stale raw");
        require(driver.isAbsoluteReliable(1) && driver.isMultiTurnConfigured(1), "verified correction did not become trusted");
        const auto& feedback = driver.getFeedback(1);
        require(feedback.offlineLastTrustedAbs == position &&
                feedback.offlineLastTrustedRaw == raw &&
                feedback.offlineLastTrustedTimestampMs == feedback.lastUpdate,
                "verified correction retained the old hotplug anchor");
        device.hardwarePosition = position + 1;
        require(driver.probeFeedback(1) && driver.getHardwareAbsolutePosition(1) == position + 1, "next feedback reused the pre-correction unwrap");
        if (name == "hardware_limits") {
            for (int boundary : {-30719, 30719}) {
                require(driver.calibrateHardwareAbsolutePosition(1, boundary, &raw, &attempts), "valid hardware boundary rejected");
                require(driver.getHardwareAbsolutePosition(1) == boundary, "boundary correction did not reach hardware feedback");
            }
        }
        return;
    }
    if (name == "cached_zero") {
        // Cached equality alone cannot establish that the instruction was executed.
        HalFixture cached(true, 0);
        cached.device.fault = Fault::ReadMissing;
        require(!cached.driver.calibrateHardwareAbsolutePosition(1, 0, &raw, &attempts), "old cached zero replaced fresh readback verification");
        require(!cached.driver.isAbsoluteReliable(1), "unverified cached zero remained trusted");
        require(!cached.driver.setTarget(1, 0, 400, 20), "failed correction accepted a target");
        require(cached.device.resets == 0, "failed correction silently fell back to RESET");
        return;
    }
    if (name == "pending_targets") {
        require(driver.setTarget(1, 12500, 400, 20), "fixture failed to queue an old-coordinate target");
        device.fault = Fault::AckMissing;
    } else {
        device.fault = namedFault(name);
    }
    require(!driver.calibrateHardwareAbsolutePosition(1, 0, &raw, &attempts), "HAL accepted an unverified coordinate write");
    require(!driver.isAbsoluteReliable(1), "partially completed correction retained coordinate trust");
    require(!driver.getFeedback(1).offlineTrustedSnapshotPending, "failed correction retained an old hotplug snapshot");
    require(raw == -123, "failed correction returned an unverified raw");
    require(!driver.setTarget(1, 0, 400, 20), "failed correction accepted a motor target");
    require(device.resets == 0, "failed correction silently fell back to RESET");
    if (name == "pending_targets") {
        const auto frameCount = device.frames.size();
        driver.syncWriteAll();
        require(device.frames.size() == frameCount, "failed correction emitted an old-coordinate target");
    }
    require(attempts > 0 && attempts <= mmhand::config::servo::kResetZeroTransactionAttempts, "HAL retries were not bounded");
    if (name == "read_one_after_offline") {
        require(!driver.isOnline(1), "one feedback frame incorrectly restored online state");
        require(device.validPositionResponsesAfterCorrection == 1,
                "online regression did not deliver exactly one valid post-offline frame");
        require(device.positionReadsAfterCorrection ==
                mmhand::config::servo::kResetZeroFeedbackAttempts *
                mmhand::config::servo::kResetZeroTransactionAttempts,
                "HAL committed before the offline readback sequence finished");
    }
    if (name.rfind("torque_", 0) == 0 || name.rfind("limits_", 0) == 0)
        require(device.corrections == 0, "HAL corrected before torque/mode verification succeeded");
    else
        require(device.corrections > 0, "failure scenario did not exercise the custom instruction");
}

// Only the task/platform boundary is stubbed. The complete production 0xDA
// execution function is inserted unchanged below and calls the actual HAL/SDK.
struct SystemRuntimeContext {
    volatile uint32_t multi_turn_unknown_bitmap = 0;
};

struct ServoRoute {
    uint8_t busIndex = 0;
    uint8_t servoId = 0;
};

static FeetechServoDriver* restoreDrivers[2] = {};
static uint8_t restoreServoIds[2] = {4, 9};
static uint8_t restoreBusIndices[2] = {0, 1};
static uint32_t routeLookups = 0;
static unsigned recoveryClockCalls = 0;

uint32_t validMotorChannelMask() {
    return (uint32_t(1) << SERVO_TOTAL_NUM) - 1;
}

FeetechServoDriver* resolveMotorDriver(uint8_t channel, ServoRoute* route,
                                     bool controllable) {
    require(controllable, "restore bypassed the controllable route gate");
    routeLookups |= uint32_t(1) << channel;
    if (channel > 1) return nullptr;
    route->busIndex = restoreBusIndices[channel];
    // Physical IDs deliberately differ from host channel indices.
    route->servoId = restoreServoIds[channel];
    return restoreDrivers[channel];
}

static uint32_t sharedAtomicFetchAnd(volatile uint32_t* value, uint32_t mask) {
    const uint32_t previous = *value;
    *value &= mask;
    return previous;
}

static uint32_t sharedAtomicLoad(const volatile uint32_t* value) {
    return *value;
}

namespace p4::platform::time {
uint32_t monotonicMillis() {
    ++recoveryClockCalls;
    return static_cast<uint32_t>(millis());
}
}

#define FW_LOG_WARN_PRINTF(...) ((void)0)
#define FW_LOG_INFO_PRINTF(...) ((void)0)

/* PRODUCTION_HOST_RESTORE_FUNCTION */

struct RoutingFixture {
    FeetechServoDriver drivers[2];
    Device devices[2];
    SystemRuntimeContext context;
    int32_t saved[SERVO_TOTAL_NUM] = {};

    explicit RoutingFixture(bool configured = true) {
        Serial1 = HardwareSerial{};
        Serial2 = HardwareSerial{};
        HardwareSerial* serials[] = {&Serial1, &Serial2};
        for (unsigned channel = 0; channel < 2; ++channel) {
            const uint8_t id = channel == 0 ? 4 : 9;
            auto& driver = drivers[channel];
            auto& device = devices[channel];
            auto* serial = serials[channel];
            device.hardwarePosition = device.previousHardwarePosition =
                channel == 0 ? 12488 : -8300;
            device.requireTorqueOff = true;
            serial->onPacket = [this, channel, serial](const Bytes& frame) {
                devices[channel].receive(*serial, frame);
            };
            driver.begin(static_cast<uint8_t>(channel), 21, 20, 1000000);
            for (unsigned read = 0; read < mmhand::config::servo::kOnlineConfirmReads; ++read)
                require(driver.probeFeedback(id), "routing fixture feedback failed");
            require(driver.isOnline(id), "routing fixture is not online");
            establishNativePosition(driver, device, id, device.hardwarePosition, configured);
            restoreDrivers[channel] = &driver;
            restoreServoIds[channel] = id;
            restoreBusIndices[channel] = static_cast<uint8_t>(channel);
            device.frames.clear();
            serial->transmitted.clear();
        }
        // The third channel is deliberately unselected and has no usable route.
        context.multi_turn_unknown_bitmap = 7;
        saved[0] = 200;
        saved[1] = -200;
        saved[2] = INT_MAX;
        routeLookups = 0;
        recoveryClockCalls = 0;
    }

    void verifySuccess(unsigned channel, int32_t position) const {
        const uint8_t id = channel == 0 ? 4 : 9;
        const auto& driver = drivers[channel];
        const auto& device = devices[channel];
        require(device.corrections == 1 && device.resets == 0,
                "host restore substituted RESET or RAM-only restore for 0x0B");
        require(device.hardwarePosition == position &&
                driver.getHardwareAbsolutePosition(id) == position &&
                driver.getAbsolutePosition(id) == position,
                "host restore did not establish the requested hardware coordinate");
        require(driver.isAbsoluteReliable(id), "verified host restore remained unknown");
        require(device.positionReadsAfterCorrection > 0,
                "host restore accepted cached feedback without a fresh read");
        bool checkedCorrection = false;
        for (const auto& frame : device.frames) {
            require(frame[2] == id, "host channel was used as the physical servo ID");
            if (frame[4] != 0x0b) continue;
            const unsigned encoded = encodePosition(position);
            require(frame[3] == 4 && frame[5] == (encoded & 0xff) &&
                    frame[6] == (encoded >> 8),
                    "host restore changed the saved position before its protocol write");
            checkedCorrection = true;
        }
        require(checkedCorrection, "host restore emitted no custom correction frame");
    }

    void verifyUnselectedSecond() const {
        require(devices[1].frames.empty() && devices[1].corrections == 0 &&
                devices[1].resets == 0 && drivers[1].getAbsolutePosition(9) == -8300 &&
                drivers[1].getHardwareAbsolutePosition(9) == devices[1].hardwarePosition,
                "host restore touched an unselected channel");
    }
};

static void verifyCorrectionFrames(const Device& device, std::size_t start,
                                   int32_t expectedPosition) {
    const unsigned encoded = encodePosition(expectedPosition);
    unsigned corrections = 0;
    for (std::size_t index = start; index < device.frames.size(); ++index) {
        const auto& frame = device.frames[index];
        require(frame[2] == 4, "retry used a different physical servo");
        if (frame[4] != 0x0b) continue;
        require(frame[3] == 4 && frame[5] == (encoded & 0xff) &&
                frame[6] == (encoded >> 8),
                "retry replaced the original hardware write with a drifted payload");
        ++corrections;
    }
    require(corrections > 0, "retry emitted no custom position correction");
}

static void routingRetryScenario(const std::string& name) {
    RoutingFixture fixture(name != "retry_mode_new_payload");
    auto& driver = fixture.drivers[0];
    auto& device = fixture.devices[0];
    // The external legacy JSON has saved ABS=0 and saved raw=200. The current
    // device reports native ABS=12488, raw=200. A completed 0x0B write changes
    // actual raw to zero; recomputing from the old JSON may yield payload=-200.
    require(driver.probeFeedback(4), "legacy retry fixture could not refresh native feedback");
    require(driver.getAbsolutePosition(4) == 12488 && driver.getRawPosition(4) == 200 &&
            !driver.hasOnlineTransitionPending(4),
            "legacy retry fixture does not represent a native ABS with an old JSON anchor");
    device.frames.clear();
    Serial1.transmitted.clear();
    fixture.saved[0] = 0;

    auto request = [&](bool expectedSuccess) {
        uint32_t restored = 0xdeadbeef;
        uint32_t rejected = 0xdeadbeef;
        fixture.context.multi_turn_unknown_bitmap |= 1;
        const bool success = restoreServoMultiTurnFromHost(
            &fixture.context, 1, fixture.saved, 0, false, &restored, &rejected);
        require(success == expectedSuccess && restored == (expectedSuccess ? 1U : 0U) &&
                rejected == (expectedSuccess ? 0U : 1U),
                "retry result masks do not describe its verified hardware outcome");
        require(fixture.context.multi_turn_unknown_bitmap == (expectedSuccess ? 6U : 7U),
                "retry cleared Unknown before the coordinate was confirmed");
        if (!expectedSuccess)
            require(!driver.isAbsoluteReliable(4) && !driver.setTarget(4, 0, 400, 20),
                    "failed retry retained coordinate trust or accepted motion");
        fixture.verifyUnselectedSecond();
    };

    if (name == "success_new_payload") {
        request(true);
        const auto start = device.frames.size();
        fixture.saved[0] = -200;
        request(true);
        verifyCorrectionFrames(device, start, -200);
        require(device.hardwarePosition == -200 && driver.getAbsolutePosition(4) == -200 &&
                driver.getHardwareAbsolutePosition(4) == -200,
                "successful transaction froze the next independent host target");
        return;
    }

    if (name == "retry_torque_new_payload" || name == "retry_mode_new_payload") {
        device.fault = name == "retry_torque_new_payload"
            ? Fault::TorqueMissing : Fault::LimitsMismatch;
        request(false);
        require(device.corrections == 0 && device.hardwarePosition == 12488,
                "precondition failure unexpectedly attempted hardware correction");
        const auto start = device.frames.size();
        device.fault = Fault::None;
        fixture.saved[0] = -200;
        request(true);
        verifyCorrectionFrames(device, start, -200);
        fixture.verifySuccess(0, -200);
        return;
    }

    const bool bounded = name == "retry_limit" || name == "retry_new_trusted" ||
        name == "retry_new_trusted_timestamp";
    const bool boundary = name == "retry_route_changed" || name == "retry_route_missing" ||
        name == "retry_session_changed" || name == "retry_payload_out_of_range";
    require(bounded || name == "retry_ack_changed_payload" ||
            name == "retry_read_changed_payload" || boundary,
            "unknown hardware-sync retry scenario");
    device.fault = name == "retry_read_changed_payload"
        ? Fault::ReadMismatch : Fault::AckMissing;
    const unsigned failedRequests = bounded ? 3 : 1;
    for (unsigned index = 0; index < failedRequests; ++index) {
        const auto start = device.frames.size();
        request(false);
        verifyCorrectionFrames(device, start, 0);
        require(device.hardwarePosition == 0,
                "retry fault did not follow a completed hardware coordinate write");
        fixture.saved[0] = -200;
    }

    if (boundary) {
        device.fault = Fault::None;
        if (name == "retry_payload_out_of_range") {
            const auto start = device.frames.size();
            const auto txStart = Serial1.transmitted.size();
            for (int invalid : {30720, -30720, INT_MAX, INT_MIN}) {
                fixture.saved[0] = invalid;
                request(false);
                require(device.frames.size() == start && Serial1.transmitted.size() == txStart,
                        "active retry cache bypassed the incoming payload range guard");
            }
            fixture.saved[0] = -200;
            request(true);
            verifyCorrectionFrames(device, start, 0);
            require(device.hardwarePosition == 0 && driver.getAbsolutePosition(4) == 0,
                    "rejected out-of-range payload erased the original pending coordinate");
            return;
        }
        if (name == "retry_route_changed" || name == "retry_route_missing") {
            const auto framesBefore = device.frames.size();
            const auto txBefore = Serial1.transmitted.size();
            if (name == "retry_route_changed") {
                // The replacement route already has a different trusted basis.
                // Route identity must be checked before that basis can clear cache.
                restoreDrivers[0] = &fixture.drivers[1];
                restoreServoIds[0] = 9;
                restoreBusIndices[0] = 1;
            } else restoreDrivers[0] = nullptr;
            request(false);
            require(device.frames.size() == framesBefore && Serial1.transmitted.size() == txBefore,
                    "changed or missing route issued a pending hardware write");
            restoreDrivers[0] = &driver;
            restoreServoIds[0] = 4;
            restoreBusIndices[0] = 0;
            request(false);
            require(device.frames.size() == framesBefore && Serial1.transmitted.size() == txBefore,
                    "returning old route restarted a previously blocked transaction");
            return;
        }
        const uint32_t previousGeneration = driver.getFeedback(4).onlineSessionGeneration;
        const uint32_t previousTrustedTimestamp = driver.getFeedback(4).offlineLastTrustedTimestampMs;
        require(driver.invalidateIfFeedbackStale(4, driver.getFeedback(4).lastUpdate + 2, 1),
                "session regression could not invalidate the old online session");
        for (unsigned read = 0; read < mmhand::config::servo::kOnlineConfirmReads; ++read)
            require(driver.probeFeedback(4), "session regression could not reconnect");
        require(driver.isOnline(4) && driver.hasOnlineTransitionPending(4) &&
                driver.getFeedback(4).onlineSessionGeneration != previousGeneration,
                "reconnection did not establish a new online generation");
        driver.finishRecoveryAttempt(4, true, static_cast<uint32_t>(millis()));
        require(!driver.hasOnlineTransitionPending(4) && !driver.isAbsoluteReliable(4) &&
                driver.getFeedback(4).offlineLastTrustedTimestampMs == previousTrustedTimestamp,
                "session regression did not preserve an unresolved coordinate basis");
        const auto framesBefore = device.frames.size();
        const auto txBefore = Serial1.transmitted.size();
        request(false);
        require(device.frames.size() == framesBefore && Serial1.transmitted.size() == txBefore,
                "consumed online event allowed a retry across device sessions");
        return;
    }

    if (!bounded) {
        const auto start = device.frames.size();
        device.fault = Fault::None;
        request(true);
        verifyCorrectionFrames(device, start, 0);
        require(device.hardwarePosition == 0 && driver.getHardwareAbsolutePosition(4) == 0 &&
                driver.getAbsolutePosition(4) == 0,
                "legacy JSON retry drifted the established hardware zero");
        require(device.resets == 0, "hardware-sync retry fell back to RESET");
        return;
    }

    const auto lockedFrameCount = device.frames.size();
    const auto lockedTransmitCount = Serial1.transmitted.size();
    device.fault = Fault::None;
    for (unsigned index = 0; index < 2; ++index) {
        fixture.saved[0] = index == 0 ? -200 : 400;
        request(false);
        require(device.frames.size() == lockedFrameCount &&
                Serial1.transmitted.size() == lockedTransmitCount,
                "exhausted host retry budget emitted another UART transaction");
    }
    if (name == "retry_limit") return;

    const uint32_t previousTrustedTimestamp = driver.getFeedback(4).offlineLastTrustedTimestampMs;
    short raw = -1;
    bool correctionAttempted = false;
    require(driver.calibrateHardwareAbsolutePosition(4, 50, &raw, nullptr,
                                                     &correctionAttempted) &&
            correctionAttempted && driver.getFeedback(4).offlineLastTrustedTimestampMs !=
                previousTrustedTimestamp,
            "explicit coordinate rebase did not establish a fresh trusted generation");
    if (name == "retry_new_trusted_timestamp") {
        // Isolate the timestamp invalidation path: the explicit rebase is trusted,
        // then a failed torque precondition removes current reliability without
        // replacing that newer trusted generation or writing another position.
        device.fault = Fault::TorqueMissing;
        require(!driver.calibrateHardwareAbsolutePosition(4, 60, &raw, nullptr,
                                                          &correctionAttempted) &&
                !correctionAttempted && !driver.isAbsoluteReliable(4),
                "pre-write failure did not isolate the new trusted timestamp");
    }
    const auto start = device.frames.size();
    device.fault = Fault::None;
    fixture.saved[0] = -200;
    request(true);
    verifyCorrectionFrames(device, start, -200);
    require(device.hardwarePosition == -200 && driver.getAbsolutePosition(4) == -200 &&
            driver.getHardwareAbsolutePosition(4) == -200,
            "fresh trusted coordinates did not release the exhausted retry cache");
}

static void routingScenario(const std::string& name) {
    if (name.rfind("retry_", 0) == 0 || name == "success_new_payload") {
        routingRetryScenario(name);
        return;
    }
    RoutingFixture fixture;
    uint32_t restored = 0xdeadbeef;
    uint32_t rejected = 0xdeadbeef;
    uint32_t mask = 1;
    uint32_t resetZeroMask = 0;
    const bool clearRecovery = name == "clear_recovery" || name == "two_routes";
    if (name == "negative") fixture.saved[0] = -200;
    else if (name == "zero") fixture.saved[0] = 0;
    else if (name == "multi_turn") fixture.saved[0] = 8392;
    else if (name == "negative_multi_turn") fixture.saved[0] = -8392;
    else if (name == "reset_mask_set") resetZeroMask = 1;
    else if (name == "two_routes") {
        mask = 3;
        fixture.saved[0] = 8392;
        fixture.saved[1] = -8392;
    } else if (name == "ack_missing" || name == "read_mismatch") {
        fixture.devices[0].fault = namedFault(name);
        require(fixture.drivers[0].setTarget(4, 12500, 400, 20),
                "routing fixture could not queue an old-coordinate target");
    } else if (name == "out_of_range") fixture.saved[0] = 30720;
    else if (name == "partial_ack_missing" || name == "partial_out_of_range") {
        mask = 3;
        if (name == "partial_ack_missing") fixture.devices[1].fault = Fault::AckMissing;
        else fixture.saved[1] = -30720;
    } else if (name == "missing_route") mask = 4;
    else if (name == "empty_mask") mask = 0;
    else require(name == "positive" || name == "reset_mask_zero" ||
                 name == "clear_recovery" || name == "keep_recovery",
                 "unknown host restore scenario");

    const bool success = restoreServoMultiTurnFromHost(
        &fixture.context, mask, fixture.saved, resetZeroMask,
        clearRecovery, &restored, &rejected);
    const bool failedFirst = name == "ack_missing" || name == "read_mismatch" ||
        name == "out_of_range" || name == "missing_route";
    const bool failedSecond = name == "partial_ack_missing" || name == "partial_out_of_range";
    const uint32_t expectedRestored = failedFirst || mask == 0 ? 0
        : failedSecond ? 1 : mask;
    const uint32_t expectedRejected = failedFirst ? mask : failedSecond ? 2 : 0;
    require(success == (mask != 0 && expectedRejected == 0),
            "host restore returned success for a partial or empty transaction");
    require(restored == expectedRestored && rejected == expectedRejected,
            "host restore success/rejected masks do not match physical outcomes");
    require(fixture.context.multi_turn_unknown_bitmap == (7U & ~expectedRestored),
            "host restore cleared unknown for an unverified or unselected channel");
    require(routeLookups == mask, "host restore resolved an unselected channel");
    require(recoveryClockCalls == (clearRecovery ? (mask == 3 ? 2U : 1U) : 0U),
            "clearRecoveryTransaction did not control recovery completion calls");
    if ((expectedRestored & 1) != 0) fixture.verifySuccess(0, fixture.saved[0]);
    if ((expectedRestored & 2) != 0) fixture.verifySuccess(1, fixture.saved[1]);
    if ((mask & 2) == 0) fixture.verifyUnselectedSecond();
    if (clearRecovery) {
        const auto& feedback = fixture.drivers[0].getFeedback(4);
        require(!fixture.drivers[0].hasOnlineTransitionPending(4) &&
                !fixture.drivers[0].isRecoveryFailed(4) &&
                feedback.recoveryAttemptCount == 0 && feedback.nextRecoveryAttemptMs == 0,
                "successful restore did not finish the recovery transaction");
    }
    if (name == "out_of_range" || name == "partial_out_of_range") {
        const unsigned channel = name == "out_of_range" ? 0 : 1;
        require(fixture.devices[channel].frames.empty(),
                "out-of-range saved ABS reached the physical UART");
    }
    if (name == "ack_missing" || name == "read_mismatch" || name == "partial_ack_missing") {
        const unsigned channel = name == "partial_ack_missing" ? 1 : 0;
        const uint8_t id = channel == 0 ? 4 : 9;
        auto& driver = fixture.drivers[channel];
        auto& device = fixture.devices[channel];
        require(device.hardwarePosition == fixture.saved[channel] && device.corrections > 0,
                "failure did not include a physically completed position write");
        require(!driver.isAbsoluteReliable(id) && !driver.setTarget(id, 0, 400, 20),
                "unconfirmed host restore remained eligible for motion");
        const auto framesBeforeFlush = device.frames.size();
        driver.syncWriteAll();
        require(device.frames.size() == framesBeforeFlush && device.resets == 0,
                "failed host restore emitted a queued target or RESET fallback");
    }
}

int main(int argc, char** argv) {
    try {
        require(argc == 2, "expected one scenario name");
        const std::string scenario(argv[1]);
        if (scenario.rfind("sdk_", 0) == 0) sdkScenario(scenario.substr(4));
        else if (scenario.rfind("hal_", 0) == 0) halScenario(scenario.substr(4));
        else if (scenario.rfind("routing_", 0) == 0) routingScenario(scenario.substr(8));
        else throw std::runtime_error("unknown scenario prefix");
        return 0;
    } catch (const std::exception& error) {
        std::cerr << (argc > 1 ? argv[1] : "harness") << ": " << error.what() << '\n';
        return 1;
    }
}
'''


def _production_restore_function(source: str) -> str:
    """Extract the whole current routing function, without rewriting its body."""
    start = source.index("bool restoreServoMultiTurnFromHost(")
    opening = source.index("{", start)
    depth = 1
    closing = opening + 1
    while depth:
        depth += (source[closing] == "{") - (source[closing] == "}")
        closing += 1
    return source[start:closing]


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
    source = directory / "servo_custom_position_test.cpp"
    routing = (P4_SRC / "tasks/servo_bus/ServoBusRouting.cpp").read_text(encoding="utf-8")
    source.write_text(
        NATIVE_SOURCE.replace("/* PRODUCTION_HOST_RESTORE_FUNCTION */",
                              _production_restore_function(routing)),
        encoding="utf-8",
    )
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
               str(SDK_SRC / "SCSerial.cpp"), str(sms_source)]
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
    "sdk_example", "sdk_zero", "sdk_negative", "sdk_signed_limits",
    "sdk_minimum_invalid", "sdk_invalid_id", "sdk_broadcast",
    "sdk_level_zero", "sdk_level_zero_missing", "sdk_short_tx", "sdk_legacy_recal",
    "sdk_ack_missing", "sdk_ack_wrong_id", "sdk_ack_wrong_length",
    "sdk_ack_checksum", "sdk_ack_status", "sdk_ack_truncated",
    "hal_zero", "hal_positive", "hal_negative", "hal_hardware_limits",
    "hal_unconfigured", "hal_invalid_id", "hal_range", "hal_null_output", "hal_unbound",
    "hal_legacy_restore", "hal_cached_zero", "hal_pending_targets",
    "hal_reset_multiturn_readback", "hal_reset_negative_readback",
    "hal_reset_uncertain_ack", "hal_reset_torque_failure",
    "hal_ack_missing", "hal_ack_wrong_id", "hal_ack_wrong_length",
    "hal_ack_checksum", "hal_ack_status", "hal_ack_truncated",
    "hal_read_missing", "hal_read_wrong_id", "hal_read_wrong_length",
    "hal_read_checksum", "hal_read_status", "hal_read_truncated", "hal_read_mismatch",
    "hal_read_one_after_offline",
    "hal_torque_status", "hal_torque_missing",
    "hal_limits_mismatch", "hal_limits_missing", "hal_limits_status",
    "routing_positive", "routing_negative", "routing_zero",
    "routing_multi_turn", "routing_negative_multi_turn",
    "routing_reset_mask_zero", "routing_reset_mask_set", "routing_two_routes",
    "routing_ack_missing", "routing_read_mismatch", "routing_out_of_range",
    "routing_partial_ack_missing", "routing_partial_out_of_range",
    "routing_missing_route", "routing_empty_mask",
    "routing_clear_recovery", "routing_keep_recovery",
    "routing_retry_ack_changed_payload", "routing_retry_read_changed_payload",
    "routing_retry_torque_new_payload", "routing_retry_mode_new_payload",
    "routing_success_new_payload", "routing_retry_limit",
    "routing_retry_new_trusted", "routing_retry_new_trusted_timestamp",
    "routing_retry_route_changed", "routing_retry_route_missing",
    "routing_retry_session_changed", "routing_retry_payload_out_of_range",
)


@pytest.mark.parametrize("scenario", SCENARIOS)
def test_production_custom_position_transaction(native_harness, scenario: str) -> None:
    executable, env = native_harness
    subprocess.run([str(executable), scenario], check=True,
                   cwd=executable.parent, env=env, timeout=10)
