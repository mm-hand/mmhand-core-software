"""Run the production Direct ABS zero dispatcher and service with fake hardware."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

from tools.precommit_checks import compiler_environment


P4_SRC = Path(__file__).resolve().parents[2] / "ESP32-P4/src"


NATIVE_SOURCE = r'''
#include "generated/UpperProtocol.generated.h"
#include "data/ActuatorTypes.h"
#include "data/RecoveryTypes.h"
#include "data/SystemTypes.h"
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

static void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
template<class T> static T sharedAtomicLoad(const T* value) { return *value; }
template<class T> static void sharedAtomicStore(T* value, T next) { *value = next; }
static void sharedAtomicFetchOr(uint32_t* value, uint32_t bits) { *value |= bits; }
static void sharedAtomicFetchAnd(uint32_t* value, uint32_t bits) { *value &= bits; }
struct SharedSafetyGuard {};
using SemaphoreHandle_t = void*;
static bool mutexAllowed = true;
static unsigned mutexTakes = 0, mutexGives = 0;
inline constexpr int pdTRUE = 1;
#define pdMS_TO_TICKS(value) (value)
static int xSemaphoreTake(SemaphoreHandle_t, int) { ++mutexTakes; return mutexAllowed; }
static void xSemaphoreGive(SemaphoreHandle_t) { ++mutexGives; }
#define FW_LOG_INFO_PRINTF(...) ((void)0)

struct SystemRuntimeContext {
    uint8_t host_protocol_compatible = 1;
    uint8_t system_state = SYSTEM_STATE_IDLE, control_enabled = 0;
    uint8_t servo_target_owner = SERVO_TARGET_OWNER_NONE;
    uint8_t servo_reset_zero_status = SERVO_RESET_ZERO_STATUS_IDLE;
    uint32_t servo_online_bitmap = (1UL << SERVO_TOTAL_NUM) - 1;
    uint32_t servo_route_controllable_bitmap = (1UL << SERVO_TOTAL_NUM) - 1;
    uint32_t servo_recovery_pending_bitmap = 0, servo_recovery_failed_bitmap = 0;
    uint32_t servo_rebase_zero_request_mask = 0;
    uint32_t servo_rebase_zero_request_token = 9, servo_rebase_zero_result_token = 9;
    uint32_t servo_rebase_zero_success_mask = 0, servo_rebase_zero_failed_mask = 0;
    uint32_t restore_multi_turn_request_token = 0, restore_multi_turn_result_token = 0;
    uint32_t direct_target_request_generation = 0, direct_target_result_generation = 0;
    uint32_t multi_turn_unknown_bitmap = 0, servo_turn_overflow_bitmap = 0;
    uint8_t pending_direct_valid = 1, pending_direct_source = MOTOR_DIRECT_SOURCE_ABSOLUTE;
    uint32_t pending_direct_motor_mask = 3, pending_direct_joint_mask = 3;
    uint8_t pending_direct_require_can = 1, direct_reject_reason = DIRECT_REJECT_NONE;
    uint8_t motor_direct_command_source = MOTOR_DIRECT_SOURCE_ABSOLUTE;
    uint32_t pending_direct_token = 25, direct_release_generation = 2;
    uint32_t applied_direct_token = 25, applied_direct_motor_mask = 3;
    SemaphoreHandle_t commandStateMutex = reinterpret_cast<void*>(1);
};
static uint32_t validMotorChannelMask() { return (1UL << SERVO_TOTAL_NUM) - 1; }
static uint32_t readU32BigEndian(const uint8_t* bytes) {
    return (uint32_t(bytes[0]) << 24) | (uint32_t(bytes[1]) << 16) |
           (uint32_t(bytes[2]) << 8) | bytes[3];
}
struct Ack { uint8_t command, state, status; };
static std::vector<Ack> acks;
static void sendProtoAckPacket(uint8_t command, uint8_t state, uint8_t status) {
    acks.push_back({command, state, status});
}

/* PRODUCTION_PENDING_FUNCTIONS */

static void dispatchZero(SystemRuntimeContext* sharedData, const uint8_t* frame,
                         size_t frameLen) {
    const uint8_t cmd = frame[0];
    /* PRODUCTION_DISPATCH_GATES */
    /* PRODUCTION_ZERO_BRANCH */
}

class FeetechServoDriver {
public:
    bool succeeds = true;
    int32_t actualAbs = 12488;
    unsigned calibrationCalls = 0, consumedSnapshots = 0;
    uint8_t calibratedId = 255;
    bool calibrateHardwareAbsolutePosition(uint8_t id, int32_t target,
                                            int16_t* raw, uint8_t* attempts) {
        require(target == 0, "Direct zero sent a non-zero hardware correction");
        ++calibrationCalls;
        calibratedId = id;
        actualAbs = 0; // An uncertain ACK may follow a successful physical write.
        *raw = 0;
        *attempts = 1;
        return succeeds;
    }
    bool consumeOfflineTrustedSnapshot(uint8_t, int32_t*, int16_t*, int16_t*, uint32_t*) {
        ++consumedSnapshots;
        return false;
    }
};
struct ServoRoute { uint8_t busIndex = 0, servoId = 0; };
static FeetechServoDriver* routes[SERVO_TOTAL_NUM]{};
static FeetechServoDriver* resolveMotorDriver(uint8_t motor, ServoRoute* route, bool executable) {
    require(executable, "zero allowed a non-executable physical route");
    route->busIndex = motor % NUM_BUSES;
    route->servoId = motor;
    return routes[motor];
}

/* PRODUCTION_ZERO_CONTEXT */
/* PRODUCTION_HOTPLUG_CONTEXT */
/* PRODUCTION_MAINTENANCE_GATE */

static uint32_t invalidatedJoints = 0;
static unsigned readyEvents = 0, clearedQueues = 0;
static uint32_t jointMaskForMotorMask(uint32_t mask) {
    uint32_t joints = 0;
    for (unsigned m = 0; m != SERVO_TOTAL_NUM; ++m)
        if (mask & (1UL << m)) joints |= 1UL << ((m == 7 || m == 8) ? 5 : m % ENCODER_TOTAL_NUM);
    return joints;
}
static void invalidateAutomaticCalibration(SystemRuntimeContext*, uint32_t mask) {
    invalidatedJoints |= mask;
}
static bool postSystemEvent(SystemRuntimeContext*, uint8_t event) {
    require(event == SYSTEM_EVENT_REEVALUATE_READY, "zero scheduled an unexpected motion event");
    ++readyEvents;
    return true;
}
namespace p4::actuator {
static void clearPending(SystemRuntimeContext*) { ++clearedQueues; }
}

/* PRODUCTION_ZERO_SERVICE */

struct Fixture {
    SystemRuntimeContext runtime;
    FeetechServoDriver drivers[SERVO_TOTAL_NUM];
    ServoHotplugRecoveryAnchors anchors{};
    ServoOnlineRecoveryContext recovery{};
    ServoRebaseZeroContext zero{};
    uint32_t lastToken = 9, sessionMask = validMotorChannelMask();
    Fixture() {
        acks.clear();
        mutexAllowed = true;
        mutexTakes = mutexGives = 0;
        invalidatedJoints = readyEvents = clearedQueues = 0;
        recovery.activeChannel = 7;
        recovery.torqueOffComplete = true;
        anchors.validMask = validMotorChannelMask();
        for (unsigned m = 0; m != SERVO_TOTAL_NUM; ++m) {
            routes[m] = &drivers[m];
            anchors.pending[m].active = true;
        }
    }
    void dispatch(uint8_t cmd, uint32_t mask, size_t length = 5) {
        const uint8_t frame[]{cmd, uint8_t(mask >> 24), uint8_t(mask >> 16),
                              uint8_t(mask >> 8), uint8_t(mask)};
        dispatchZero(&runtime, frame, length);
    }
    void capture() { captureServoRebaseZero(&runtime, &lastToken, &zero); }
    bool step() {
        return stepServoRebaseZero(&runtime, &zero, &anchors, &recovery, &sessionMask);
    }
    void verifyAck(uint8_t cmd, uint8_t status) const {
        require(acks.size() == 1 && acks[0].command == cmd && acks[0].status == status,
                "dispatcher ACK did not match the received command and outcome");
    }
    void verifyUntouched() const {
        for (const auto& driver : drivers)
            require(driver.calibrationCalls == 0 && driver.actualAbs == 12488,
                    "rejected zero changed a hardware coordinate");
    }
};

static void run(const std::string& name) {
    const auto f1 = CMD_MOTOR_ABS_ZERO;
    require(f1 == 0xF1 && generatedCommandPayloadLength(f1) == 4 &&
            generatedCommandRequiresHandshake(f1), "Direct zero contract drift");
    Fixture f;
    if (name.rfind("reject_", 0) == 0) {
        uint32_t mask = 1UL << 7;
        size_t length = 5;
        uint8_t status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        const auto reason = name.substr(7);
        if (reason == "handshake") {
            f.runtime.host_protocol_compatible = 0;
            status = PROTO_ACK_STATUS_CONTRACT_MISMATCH;
        } else if (reason == "direct_pending") f.runtime.direct_target_request_generation = 1;
        else if (reason == "motion_enabled") f.runtime.control_enabled = 1;
        else if (reason == "owner") f.runtime.servo_target_owner = SERVO_TARGET_OWNER_CONTROL;
        else if (reason == "motion_state") f.runtime.system_state = SYSTEM_STATE_RUN_DIRECT;
        else if (reason == "retired_state") f.runtime.system_state = 7;
        else if (reason == "zero_pending") ++f.runtime.servo_rebase_zero_request_token;
        else if (reason == "restore_pending") ++f.runtime.restore_multi_turn_request_token;
        else if (reason == "reset_pending") f.runtime.servo_reset_zero_status = SERVO_RESET_ZERO_STATUS_RUNNING;
        else if (reason == "recovery_busy") f.runtime.servo_recovery_pending_bitmap = mask;
        else if (reason == "mutex") mutexAllowed = false;
        else {
            status = PROTO_ACK_STATUS_INVALID_DATA;
            if (reason == "empty") mask = 0;
            else if (reason == "mask_range") mask |= 1UL << SERVO_TOTAL_NUM;
            else if (reason == "offline") f.runtime.servo_online_bitmap &= ~mask;
            else if (reason == "route") f.runtime.servo_route_controllable_bitmap &= ~mask;
            else if (reason == "short") length = 4;
            else if (reason == "long") length = 6;
            else require(false, "unknown rejection scenario");
        }
        const auto initialToken = f.runtime.servo_rebase_zero_request_token;
        f.dispatch(f1, mask, length);
        f.verifyAck(f1, status);
        require(f.runtime.servo_rebase_zero_request_token == initialToken &&
                f.runtime.servo_rebase_zero_request_mask == 0 &&
                f.runtime.pending_direct_valid == 1, "rejection committed a maintenance transaction");
        f.verifyUntouched();
        return;
    }
    if (name == "pending_blocks_motion") {
        ++f.runtime.servo_rebase_zero_request_token;
        for (uint8_t command : {CMD_START, CMD_RUN_FEEDBACK, CMD_ROPE_INIT,
                               CMD_MOTOR_POS_ABS, CMD_ANGLE_CTRL,
                               CMD_CALIB_LOAD_JOINT, CMD_CALIB_LOAD_LIMITS}) {
            acks.clear();
            f.dispatch(command, 1UL << 7);
            f.verifyAck(command, PROTO_ACK_STATUS_UNSUPPORTED_MODE);
        }
        f.verifyUntouched();
        return;
    }
    const auto cmd = f1;
    uint32_t mask = 1UL << 7;
    if (name == "partial_failure") mask |= 1UL << 26;
    if (name == "token_wrap") {
        f.runtime.servo_rebase_zero_request_token = std::numeric_limits<uint32_t>::max();
        f.runtime.servo_rebase_zero_result_token = f.runtime.servo_rebase_zero_request_token;
        f.lastToken = f.runtime.servo_rebase_zero_result_token;
    }
    if (name == "recovery_failed") {
        f.runtime.servo_recovery_pending_bitmap = mask;
        f.runtime.servo_recovery_failed_bitmap = mask;
    }
    if (name == "partial_failure") f.drivers[26].succeeds = false;
    if (name == "route_lost") routes[7] = nullptr;
    f.dispatch(cmd, mask);
    f.verifyAck(cmd, PROTO_ACK_STATUS_QUEUED);
    require(f.runtime.servo_rebase_zero_request_mask == mask &&
            f.runtime.servo_rebase_zero_request_token != 0 &&
            f.runtime.servo_rebase_zero_request_token != f.runtime.servo_rebase_zero_result_token,
            "accepted zero did not publish the exact selected mask and generation");
    require(!f.runtime.pending_direct_valid && !f.runtime.pending_direct_motor_mask &&
            !f.runtime.applied_direct_motor_mask && !f.runtime.pending_direct_token &&
            !f.runtime.direct_release_generation, "accepted zero retained an old direct target");
    require(mutexTakes == 1 && mutexGives == 1, "zero failed to release the command mutex");
    const auto token = f.runtime.servo_rebase_zero_request_token;
    if (name == "duplicate") {
        acks.clear();
        f.dispatch(f1, mask);
        f.verifyAck(f1, PROTO_ACK_STATUS_UNSUPPORTED_MODE);
        require(f.runtime.servo_rebase_zero_request_token == token,
                "duplicate zero created another generation");
    }
    if (name == "capture_unsafe") f.runtime.control_enabled = 1;
    f.capture();
    if (name == "step_unsafe") f.runtime.servo_target_owner = SERVO_TARGET_OWNER_CONTROL;
    while (f.zero.active) f.step();
    require(f.runtime.servo_rebase_zero_result_token == token && readyEvents == 1,
            "zero did not finalize the matching transaction");
    if (name == "capture_unsafe" || name == "step_unsafe") {
        require(f.runtime.servo_rebase_zero_failed_mask == mask &&
                f.runtime.servo_rebase_zero_success_mask == 0 && invalidatedJoints == 0,
                "execution safety rejection altered calibration or reported success");
        f.verifyUntouched();
        return;
    }
    const uint32_t success = name == "route_lost" ? 0 : (1UL << 7);
    require(f.runtime.servo_rebase_zero_success_mask == success &&
            f.runtime.servo_rebase_zero_failed_mask == (mask & ~success),
            "final zero result misreported selected motor success/failure");
    require((f.runtime.multi_turn_unknown_bitmap & mask) == (mask & ~success),
            "failed correction trusted uncertain hardware ABS");
    require((f.anchors.validMask & mask) == 0 && !f.anchors.pending[7].active &&
            f.anchors.pending[8].active, "zero changed an unselected motor's recovery anchor");
    require(f.recovery.activeChannel == 0xFF && !f.recovery.torqueOffComplete,
            "zero retained the selected motor's old online recovery phase");
    require(invalidatedJoints == jointMaskForMotorMask(mask), "zero invalidated unrelated joint calibration");
    for (unsigned m = 0; m != SERVO_TOTAL_NUM; ++m) {
        const bool expected = (mask & (1UL << m)) && routes[m];
        require(f.drivers[m].calibrationCalls == unsigned(expected),
                "zero hardware calls expanded beyond selected motors or retried a coordinate write");
        if (expected) require(f.drivers[m].actualAbs == 0 &&
                              f.drivers[m].calibratedId == m &&
                              f.drivers[m].consumedSnapshots == 1,
                              "zero used the wrong physical motor or retained a pre-zero snapshot");
        else require(f.drivers[m].actualAbs == 12488, "zero changed an unselected hardware ABS");
    }
}

int main(int argc, char** argv) {
    try {
        require(argc == 2, "scenario required");
        run(argv[1]);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
'''


def _braced(source: str, signature: str) -> str:
    start = source.index(signature)
    opening = source.index("{", start)
    depth, closing = 1, opening + 1
    while depth:
        depth += (source[closing] == "{") - (source[closing] == "}")
        closing += 1
    return source[start:closing]


@pytest.fixture(scope="module")
def direct_zero_harness(tmp_path_factory: pytest.TempPathFactory) -> tuple[Path, dict[str, str]]:
    env = compiler_environment()
    compiler = next((path for name in ("g++", "clang++", "cl")
                     if (path := shutil.which(name, path=env.get("PATH")))), None)
    assert compiler is not None, "A native C++ compiler is required; this test may not skip"
    directory = tmp_path_factory.mktemp("direct-zero-native")
    dispatcher = (P4_SRC / "tasks/host_comm/HostCommandDispatcher.cpp").read_text(encoding="utf-8")
    service = (P4_SRC / "tasks/servo_bus/ServoBusService.cpp").read_text(encoding="utf-8")
    gates_start = dispatcher.index("    if (generatedCommandRequiresHandshake(cmd)")
    handshake_end = dispatcher.index("    if (cmd == CMD_PARAMETERS_QUERY)", gates_start)
    pending_start = dispatcher.index("    if (servoRebaseZeroPending(sharedData)", handshake_end)
    pending_end = dispatcher.index("    if (cmd == CMD_HOST_HEARTBEAT)", pending_start)
    contexts = "\n".join(_braced(service, "struct " + name + " {") + ";"
                         for name in ("ServoHotplugPendingSync", "ServoHotplugRecoveryAnchors",
                                      "ServoOnlineRecoveryContext"))
    source = directory / "direct_zero_test.cpp"
    source.write_text(
        NATIVE_SOURCE
        .replace("/* PRODUCTION_PENDING_FUNCTIONS */",
                 _braced(dispatcher, "bool servoRebaseZeroPending(") + "\n" +
                 _braced(dispatcher, "bool commandConflictsWithServoRebaseZero("))
        .replace("/* PRODUCTION_DISPATCH_GATES */", dispatcher[gates_start:handshake_end] +
                 dispatcher[pending_start:pending_end])
        .replace("/* PRODUCTION_ZERO_BRANCH */",
                 _braced(dispatcher, "if (cmd == CMD_MOTOR_ABS_ZERO)"))
        .replace("/* PRODUCTION_ZERO_CONTEXT */",
                 _braced(service, "struct ServoRebaseZeroContext {") + ";")
        .replace("/* PRODUCTION_HOTPLUG_CONTEXT */", contexts)
        .replace("/* PRODUCTION_MAINTENANCE_GATE */",
                 _braced(service, "static bool servoCoordinateMaintenanceSafe("))
        .replace("/* PRODUCTION_ZERO_SERVICE */", "\n".join(
            _braced(service, "static " + signature) for signature in
            ("void finishServoRebaseZero(", "void captureServoRebaseZero(",
             "bool stepServoRebaseZero("))),
        encoding="utf-8",
    )
    executable = directory / "direct_zero_test.exe"
    if Path(compiler).name.lower() in {"cl", "cl.exe"}:
        command = [compiler, "/std:c++17", "/EHsc", "/utf-8", f"/I{P4_SRC}", str(source),
                   f"/Fo{directory.resolve()}\\", f"/Fe{executable}"]
    else:
        command = [compiler, "-std=c++17", f"-I{P4_SRC}", str(source), "-o", str(executable)]
    subprocess.run(command, check=True, cwd=directory, env=env, timeout=120)
    return executable, env


SCENARIOS = (
    "reject_handshake", "reject_direct_pending", "reject_motion_enabled", "reject_owner",
    "reject_motion_state", "reject_retired_state", "reject_zero_pending", "reject_restore_pending",
    "reject_reset_pending", "reject_recovery_busy", "reject_mutex", "reject_empty",
    "reject_mask_range", "reject_offline", "reject_route", "reject_short", "reject_long",
    "pending_blocks_motion", "selected_motor", "f0_compatible", "partial_failure", "route_lost",
    "token_wrap", "recovery_failed", "duplicate", "capture_unsafe", "step_unsafe",
)


@pytest.mark.parametrize("scenario", SCENARIOS)
def test_production_direct_abs_zero(
    direct_zero_harness: tuple[Path, dict[str, str]], scenario: str,
) -> None:
    executable, env = direct_zero_harness
    subprocess.run([str(executable), scenario], check=True, cwd=executable.parent,
                   env=env, timeout=10)
