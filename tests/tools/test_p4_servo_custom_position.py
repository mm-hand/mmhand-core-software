"""Exercise production STS correction, HAL and host restore routing with fake UARTs."""

from __future__ import annotations

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


NATIVE_SOURCE = r'''
#include "hal/servo/FeetechServoDriver.h"
#include "tasks/servo_bus/ServoAbsoluteRestore.h"
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
    ReadMismatch, ReadOneAfterOffline, ReadModulo,
    TorqueStatus, TorqueMissing, LimitsMismatch, LimitsMissing, LimitsStatus,
    ResetAckMissing, ResetAckStatus
};

struct Device {
    Bytes config = {1,0,0,0,0,0,0,0,0,0,0x95};
    Bytes version = {3,10};
    Bytes profile = {254,254,0};
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
    const int target = name == "negative" ? -8392 : name == "zero" ? 0 : 8292;
    if (name == "ack_missing") device.fault = Fault::AckMissing;
    if (name == "ack_status") device.fault = Fault::AckStatus;
    if (name == "read_missing") device.fault = Fault::ReadMissing;
    if (name == "read_status") device.fault = Fault::ReadStatus;
    if (name == "read_mismatch") device.fault = Fault::ReadMismatch;
    if (name == "single_turn_rejected") device.fault = Fault::ReadModulo;
    if (name == "torque_missing") device.fault = Fault::TorqueMissing;
    const auto first = writer.step(&driver, 1, target, 1000);
    if (name == "ack_status" || name == "torque_missing") {
        require(first == Result::Failed && !driver.isAbsoluteReliable(1), "rejected command became trusted");
        return;
    }
    require(first == Result::Pending && !driver.isAbsoluteReliable(1), "write trusted a cached sample");
    require(device.corrections == 1, "one request did not issue exactly one correction");
    require(writer.step(&driver, 1, target, 1001) == Result::Pending, "old feedback accepted");
    if (name == "cancel") {
        writer.cancel();
        require(!writer.active() && !driver.isAbsoluteReliable(1), "cancel restored motion authority");
        return;
    }
    if (name == "stale") {
        require(writer.step(&driver, 1, target, 1500) == Result::Failed, "stale sample accepted");
        return;
    }
    const bool read = driver.probeFeedback(1);
    if (name == "read_missing") require(!read, "missing read was accepted");
    if (name == "read_missing" || name == "read_mismatch" || name == "single_turn_rejected" || name == "read_status") {
        require(writer.step(&driver, 1, target, 1500) == Result::Failed, "invalid full ABS accepted");
        require(!driver.isAbsoluteReliable(1), "failed restore became trusted");
        // A later request must retain the first uncertain target and never rewrite from shifted raw.
        require(writer.step(&driver, 1, -200, 1600) == Result::Pending, "pending restore not retained");
        require(device.corrections == 1, "uncertain write was repeated");
        device.fault = Fault::None;
        require(driver.probeFeedback(1), "late native feedback missing");
        require(writer.step(&driver, 1, -200, 1601) == Result::Success, "late exact result not recovered");
        require(driver.getAbsolutePosition(1) == target, "late request changed first target");
        return;
    }
    require(read && writer.step(&driver, 1, target, 1002) == Result::Success, "exact native ABS not confirmed");
    require(driver.getAbsolutePosition(1) == target && driver.isAbsoluteReliable(1), "native ABS differs from target");
    require(device.corrections == 1 && device.resets == 0, "restore rewrote coordinates or reset turns");
    if (name == "disconnect") {
        require(driver.invalidateIfFeedbackStale(1, driver.getFeedback(1).lastUpdate+100, 80), "offline not detected");
        int32_t saved = 0;
        require(driver.consumeOfflineTrustedSnapshot(1, &saved) && saved == target,
                "offline lost native checkpoint");
        require(!driver.isAbsoluteReliable(1), "offline retained coordinate authority");
    }
}

int main(int argc, char** argv) {
    try {
        require(argc == 2, "scenario required");
        const std::string name(argv[1]);
        if (name.rfind("sdk_",0) == 0) sdkScenario(name.substr(4));
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
    source = directory / "servo_custom_position_test.cpp"
    source.write_text(NATIVE_SOURCE, encoding="utf-8")
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
    "sdk_example", "sdk_zero", "sdk_negative", "sdk_signed_limits",
    "sdk_minimum_invalid", "sdk_invalid_id", "sdk_broadcast", "sdk_level_zero",
    "sdk_level_zero_missing", "sdk_short_tx", "sdk_legacy_recal", "sdk_ack_missing",
    "sdk_ack_wrong_id", "sdk_ack_wrong_length", "sdk_ack_checksum", "sdk_ack_status", "sdk_ack_truncated",
    "firmware_old", "firmware_new", "firmware_minor", "session_recheck", "torque_enable_missing", "version_missing", "profile", "profile_ignored", "config_ack_missing", "lock_failure",
    "configuration", "native", "positive", "negative", "zero", "ack_missing", "ack_status",
    "torque_missing", "read_missing", "read_mismatch", "read_status", "single_turn_rejected", "stale", "cancel", "disconnect",
)

@pytest.mark.parametrize("scenario", SCENARIOS)
def test_production_custom_position_transaction(native_harness, scenario: str) -> None:
    executable, env = native_harness
    subprocess.run([str(executable), scenario], check=True,
                   cwd=executable.parent, env=env, timeout=10)
