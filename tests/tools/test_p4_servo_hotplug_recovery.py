"""Compile the production ABS safety policy and complete RAM recovery handler."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

from tools.precommit_checks import compiler_environment


ROOT = Path(__file__).resolve().parents[2]
P4_SRC = ROOT / "ESP32-P4/src"


NATIVE_SOURCE = r'''
#include "safety/policies/ActuatorSafetyPolicy.h"
#include "safety/SafetyRuleRegistry.h"
#include "config/safety/ServoPositionRange.h"
#include "config/ServoConfig.h"
#include "generated/HardwareBindings.generated.h"
#include "data/SystemTypes.h"
#include "hal/servo/ServoValue.h"

#include <climits>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using mmhand::config::servo::kRecoveryMaxAttempts;
static void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
namespace p4::safety {
/* PRODUCTION_RULE_MAPPING */
}

static int16_t normalizeServoRawSingleTurn(int32_t value) {
    return mmhand::servo::normalizeRawSingleTurn(value);
}
struct SystemRuntimeContext {
    uint32_t multi_turn_unknown_bitmap = 3;
    uint32_t restore_multi_turn_restored_mask = 0;
    uint32_t restore_multi_turn_rejected_mask = 0;
    uint32_t restore_multi_turn_result_token = 0;
};
static void sharedAtomicFetchOr(uint32_t* value, uint32_t mask) { *value |= mask; }
static void sharedAtomicFetchAnd(uint32_t* value, uint32_t mask) { *value &= mask; }
static uint32_t sharedAtomicLoad(const uint32_t* value) { return *value; }
static void sharedAtomicStore(uint32_t* value, uint32_t data) { *value = data; }
#define FW_LOG_WARN_PRINTF(...) ((void)0)
#define FW_LOG_INFO_PRINTF(...) ((void)0)
namespace p4::platform::time {
static uint32_t clockMs = 1000;
static uint32_t monotonicMillis() { return clockMs; }
}
struct ServoFeedbackInternal {
    uint32_t onlineSessionGeneration = 7;
    uint32_t offlineLastTrustedTimestampMs = 50;
};
enum class ServoMultiTurnConfigResult { Pending, Complete, Failed };
enum class Fault { None, AckLost, BeforeWrite, ReconnectDuringWrite };
class FeetechServoDriver {
public:
    ServoFeedbackInternal feedback;
    bool online = true, reliable = false, configured = true;
    bool recoveryPending = true, failed = false, torqueOk = true;
    uint8_t attemptCount = 0, expectedId = 4;
    uint32_t nextAttempt = 0;
    int16_t raw = 200;
    int32_t hardwareAbs = 200;
    Fault fault = Fault::None;
    unsigned torqueCalls = 0, calibrationCalls = 0, configCalls = 0;
    bool snapshotPending = false;
    int32_t snapshotAbs = 0;
    int16_t snapshotRaw = 0;
    std::vector<int32_t> writtenTargets;

    bool isOnline(uint8_t) const { return online; }
    bool isAbsoluteReliable(uint8_t) const { return reliable; }
    const ServoFeedbackInternal& getFeedback(uint8_t) const { return feedback; }
    bool isRecoveryAttemptDue(uint8_t, uint32_t now) const {
        return online && recoveryPending && !failed && now >= nextAttempt;
    }
    bool setTorqueEnabled(uint8_t id, bool enabled) {
        require(id == expectedId && !enabled, "recovery accessed wrong servo or enabled torque");
        ++torqueCalls;
        return torqueOk;
    }
    bool isMultiTurnConfigured(uint8_t) const { return configured; }
    ServoMultiTurnConfigResult stepMultiTurnConfiguration(uint8_t id, uint32_t) {
        require(id == expectedId, "configuration used wrong servo");
        ++configCalls;
        configured = true;
        return ServoMultiTurnConfigResult::Complete;
    }
    int16_t getRawPosition(uint8_t) const { return raw; }
    bool calibrateHardwareAbsolutePosition(uint8_t id, int32_t target,
                                           int16_t* outRaw, uint8_t* attempts,
                                           bool* attempted) {
        require(id == expectedId, "correction used wrong physical servo");
        require(torqueCalls != 0 && configured, "correction skipped torque or mode preparation");
        ++calibrationCalls;
        *attempts = 1;
        *attempted = false;
        reliable = false;
        if (fault == Fault::BeforeWrite) return false;
        *attempted = true;
        writtenTargets.push_back(target);
        hardwareAbs = target;
        raw = normalizeServoRawSingleTurn(target);
        if (fault == Fault::ReconnectDuringWrite) {
            ++feedback.onlineSessionGeneration;
            online = false;
            return false;
        }
        if (fault == Fault::AckLost) return false;
        *outRaw = raw;
        reliable = true;
        recoveryPending = false;
        attemptCount = 0;
        feedback.offlineLastTrustedTimestampMs = p4::platform::time::clockMs;
        return true;
    }
    uint8_t recoveryAttemptCount(uint8_t) const { return attemptCount; }
    bool isRecoveryFailed(uint8_t) const { return failed; }
    void finishRecoveryAttempt(uint8_t id, bool success, uint32_t now) {
        require(id == expectedId, "recovery completion used wrong physical servo");
        if (success) {
            recoveryPending = false;
            attemptCount = 0;
            failed = false;
            nextAttempt = 0;
        } else {
            ++attemptCount;
            failed = attemptCount >= kRecoveryMaxAttempts;
            nextAttempt = now + mmhand::config::servo::kRecoveryRetryIntervalMs;
        }
    }
    bool consumeOfflineTrustedSnapshot(uint8_t, int32_t* abs, int16_t* rawOut,
                                        int16_t* turn, uint32_t* timestamp) {
        if (!snapshotPending) return false;
        *abs = snapshotAbs;
        *rawOut = snapshotRaw;
        *turn = (int16_t)((snapshotAbs - snapshotRaw) / 4096);
        *timestamp = feedback.offlineLastTrustedTimestampMs;
        snapshotPending = false;
        return true;
    }
};
struct ServoRoute { uint8_t busIndex = 0, servoId = 4; };
static FeetechServoDriver* routedDrivers[2] = {};
static uint8_t routedIds[2] = {4, 9};
static FeetechServoDriver* resolveMotorDriver(uint8_t channel, ServoRoute* route, bool) {
    if (channel >= 2 || !routedDrivers[channel]) return nullptr;
    route->busIndex = channel;
    route->servoId = routedIds[channel];
    return routedDrivers[channel];
}

/* PRODUCTION_HOTPLUG_HANDLER */

/* PRODUCTION_HOST_RESTORE_CONTEXT */
static bool maintenanceSafe = true;
static bool lowerHostSuccess = true;
static unsigned lowerHostCalls = 0;
static ServoHotplugRecoveryAnchors* observedAnchors = nullptr;
static std::vector<int32_t> lowerHostTargets;
static bool servoCoordinateMaintenanceSafe(SystemRuntimeContext*) { return maintenanceSafe; }
static bool restoreServoMultiTurnFromHost(SystemRuntimeContext* runtime, uint32_t mask,
                                         const int32_t* positions, uint32_t,
                                         bool clearRecovery, uint32_t* restored,
                                         uint32_t* rejected) {
    require(clearRecovery && mask == 1, "host restore used an unexpected physical request");
    require(observedAnchors && (observedAnchors->validMask & mask) == 0,
            "host restore wrote hardware before retiring its old RAM anchor");
    ++lowerHostCalls;
    lowerHostTargets.push_back(positions[0]);
    auto* driver = routedDrivers[0];
    driver->hardwareAbs = positions[0];
    driver->raw = normalizeServoRawSingleTurn(positions[0]);
    driver->reliable = lowerHostSuccess;
    if (lowerHostSuccess) {
        driver->recoveryPending = false;
        runtime->multi_turn_unknown_bitmap &= ~mask;
    }
    *restored = lowerHostSuccess ? mask : 0;
    *rejected = lowerHostSuccess ? 0 : mask;
    return lowerHostSuccess;
}
/* PRODUCTION_HOST_FINISH */
/* PRODUCTION_HOST_STEP */

struct Fixture {
    FeetechServoDriver driver;
    SystemRuntimeContext runtime;
    ServoHotplugRecoveryAnchors anchors{};
    ServoOnlineRecoveryContext recovery{0xFF, 0, false};
    uint32_t sessionRebasedMask = 0;
    bool attempted = false;
    Fixture() {
        routedDrivers[0] = &driver;
        routedDrivers[1] = nullptr;
        routedIds[0] = 4;
        anchors.validMask = 1;
        anchors.logicalAbs[0] = 0;
        anchors.raw[0] = 200;
        observedAnchors = &anchors;
    }
    void step() {
        p4::platform::time::clockMs += 1000;
        processServoOnlineTransitions(&runtime, &anchors, &recovery,
                                       &sessionRebasedMask, &attempted);
    }
    void attempt() {
        const unsigned calls = driver.calibrationCalls;
        const uint8_t previous = driver.attemptCount;
        for (unsigned n = 0; n != 10; ++n) {
            step();
            if (driver.calibrationCalls != calls || driver.attemptCount != previous ||
                !driver.recoveryPending) return;
        }
    }
    void verifyUnknown() const {
        require((runtime.multi_turn_unknown_bitmap & 1) != 0 && !driver.reliable,
                "failed recovery trusted unverified ABS");
    }
};

static void crosspathScenario(const std::string& name) {
    Fixture f;
    ServoHostRestoreContext request{};
    request.active = true;
    request.token = 77;
    request.requestMask = request.pendingMask = 1;
    request.logicalAbs[0] = 8392;
    if (name == "pending_blocks_host") {
        f.driver.fault = Fault::AckLost;
        f.attempt();
        request.logicalAbs[0] = -200;
        const auto calls = f.driver.calibrationCalls;
        require(stepHostMultiTurnRestore(&f.runtime, &request, &f.sessionRebasedMask, &f.anchors),
                "pending rejection did not complete a host restore step");
        require(lowerHostCalls == 0 && f.driver.calibrationCalls == calls &&
                f.driver.writtenTargets == std::vector<int32_t>{0} &&
                f.anchors.validMask == 1 && f.anchors.pending[0].active &&
                f.runtime.restore_multi_turn_result_token == 77 &&
                f.runtime.restore_multi_turn_restored_mask == 0 &&
                f.runtime.restore_multi_turn_rejected_mask == 1 && !request.active,
                "host bypassed unresolved RAM sync or lost its anchor/result");
        f.verifyUnknown();
    } else if (name == "host_failure_retires_ram") {
        lowerHostSuccess = false;
        stepHostMultiTurnRestore(&f.runtime, &request, &f.sessionRebasedMask, &f.anchors);
        require(lowerHostCalls == 1 && lowerHostTargets == std::vector<int32_t>{8392} &&
                f.anchors.validMask == 0 && f.runtime.restore_multi_turn_result_token == 77 &&
                f.runtime.restore_multi_turn_rejected_mask == 1,
                "failed host restore left an old RAM anchor usable");
        f.driver.raw = 0; // A changed read after an unconfirmed host write cannot cause RAM replay.
        f.attempt();
        require(f.driver.calibrationCalls == 0 && f.driver.writtenTargets.empty(),
                "RAM recovery rewrote hardware after host took ownership of its coordinate");
        f.verifyUnknown();
    } else if (name == "host_success") {
        require(stepHostMultiTurnRestore(&f.runtime, &request, &f.sessionRebasedMask, &f.anchors),
                "normal host restore did not execute");
        require(lowerHostCalls == 1 && lowerHostTargets == std::vector<int32_t>{8392} &&
                f.anchors.validMask == 0 && f.sessionRebasedMask == 1 && f.driver.reliable &&
                f.runtime.restore_multi_turn_result_token == 77 &&
                f.runtime.restore_multi_turn_restored_mask == 1 &&
                f.runtime.restore_multi_turn_rejected_mask == 0 && !request.active,
                "normal host restore did not commit the ABS/result");
    } else if (name == "unsafe_rejected") {
        maintenanceSafe = false;
        require(!stepHostMultiTurnRestore(&f.runtime, &request, &f.sessionRebasedMask, &f.anchors),
                "unsafe maintenance was executed");
        require(lowerHostCalls == 0 && f.anchors.validMask == 1 &&
                f.runtime.restore_multi_turn_result_token == 77 &&
                f.runtime.restore_multi_turn_rejected_mask == 1 && !request.active,
                "unsafe host request touched a RAM anchor or missed its rejected result");
        f.verifyUnknown();
    } else require(false, "unknown crosspath scenario");
}

static void safetyScenario(const std::string& name) {
    if (name == "checked_conversion") {
        int16_t output = 123;
        const auto range = servoPositionRange();
        require(range.minAbs == -30719 && range.maxAbs == 30719, "range is not fixed hardware ABS");
        for (int value : {-30719, -8392, -1, 0, 1, 8392, 30719}) {
            require(servoHardwareCommandFromAbs(value, &output) && output == value,
                    "checked ABS conversion shifted or rejected valid target");
        }
        for (int value : {INT_MIN, -30720, 30720, INT_MAX}) {
            output = 123;
            require(!servoHardwareCommandFromAbs(value, &output) && output == 123,
                    "invalid ABS was truncated or overwrote output");
        }
        require(!servoHardwareCommandFromAbs(0, nullptr), "null conversion output accepted");
        return;
    }
    ServoTargetBatch_t batch{};
    batch.count = 1;
    batch.source = SERVO_TARGET_OWNER_CONTROL;
    batch.timestamp = 1000;
    batch.commands[0].motorChannel = 0;
    batch.commands[0].position = 8392;
    int32_t mins[SERVO_TOTAL_NUM] = {}, maxs[SERVO_TOTAL_NUM] = {};
    mins[0] = -30719;
    maxs[0] = 30719;
    p4::actuator::ServoSafetyContext context{};
    context.activeOwner = SERVO_TARGET_OWNER_CONTROL;
    context.systemState = SYSTEM_STATE_RUN_DIRECT;
    context.nowMs = 1000;
    context.maxTargetAgeMs = 500;
    context.onlineMotorMask = context.routedMotorMask = context.controllableRouteMask = 1;
    context.logicalRangeValidMask = 1;
    context.logicalMin = mins;
    context.logicalMax = maxs;
    context.requireOnline = context.requireMultiTurnKnown = true;
    ServoRejectReason expected = ServoRejectReason::None;
    if (name == "identity") {
        for (int value : {-30719, -8392, 0, 8392, 30719}) {
            batch.commands[0].position = value;
            const auto result = p4::actuator::evaluateCommand(batch, 0, context);
            require(result.allowed && result.requestedAbs == value && result.appliedAbs == value &&
                    result.hardwareAbs == value, "safety shifted a valid ABS target");
        }
        return;
    }
    if (name == "hard_boundary") {
        context.logicalRangeValidMask = 0;
        for (int value : {INT_MIN, -30720, 30720, INT_MAX}) {
            batch.commands[0].position = value;
            const auto result = p4::actuator::evaluateCommand(batch, 0, context);
            require(!result.allowed && result.reason == ServoRejectReason::HardwareRange &&
                    result.hardwareAbs == value, "hardware boundary accepted or truncated invalid ABS");
        }
        return;
    }
    if (name == "unknown_before_range") {
        batch.commands[0].position = INT_MAX;
        context.multiTurnUnknownMask = 1;
        expected = ServoRejectReason::MultiTurnUnknown;
    } else if (name == "overflow") {
        context.turnOverflowMask = 1;
        expected = ServoRejectReason::TurnOverflow;
    } else if (name == "logical_boundary") {
        maxs[0] = 8000;
        expected = ServoRejectReason::LogicalRange;
    } else require(false, "unknown safety scenario");
    const auto result = p4::actuator::evaluateCommand(batch, 0, context);
    require(!result.allowed && result.reason == expected &&
            result.hardwareAbs == batch.commands[0].position, "ABS safety gate changed priority or coordinate");
}

static void hotplugScenario(const std::string& name) {
    Fixture f;
    if (name == "nearest_turn") {
        int32_t output = 100;
        require(servoHotplugTargetFromAnchor(8392, 200, 210, &output) && output == 8402,
                "same-turn restoration failed");
        require(servoHotplugTargetFromAnchor(12283, 4091, 3, &output) && output == 12291,
                "forward wrap restoration failed");
        require(servoHotplugTargetFromAnchor(-8189, 3, 4091, &output) && output == -8197,
                "negative backward wrap restoration failed");
        require(servoHotplugTargetFromAnchor(0, 0, 2048, &output) && output == -2048,
                "half-turn tie is inconsistent with host restore");
        require(!servoHotplugTargetFromAnchor(INT_MAX, 0, 1, &output) &&
                !servoHotplugTargetFromAnchor(INT_MIN, 0, 4095, &output) &&
                !servoHotplugTargetFromAnchor(30719, 100, 101, &output) &&
                !servoHotplugTargetFromAnchor(0, 0, 0, nullptr), "invalid anchor accepted");
        return;
    }
    if (name == "no_anchor") {
        f.anchors.validMask = 0;
        f.attempt();
        require(f.driver.calibrationCalls == 0 && !f.driver.recoveryPending,
                "missing anchor triggered correction or blocked host restore");
        f.verifyUnknown();
        return;
    }
    if (name == "range_rejected") {
        f.anchors.logicalAbs[0] = INT_MAX;
        for (unsigned n = 0; n != 3; ++n) f.attempt();
        require(f.driver.writtenTargets.empty() && f.driver.failed,
                "out-of-range anchor reached hardware or retried indefinitely");
        f.verifyUnknown();
        return;
    }
    if (name == "success") {
        f.anchors.logicalAbs[0] = 8392;
        f.driver.raw = 210;
        f.driver.configured = false;
        f.attempt();
        require(f.driver.writtenTargets == std::vector<int32_t>{8402} && f.driver.reliable &&
                (f.runtime.multi_turn_unknown_bitmap & 1) == 0 && f.anchors.validMask == 0 &&
                f.sessionRebasedMask == 1 && !f.anchors.pending[0].active && f.driver.configCalls == 1,
                "hardware ABS restore failed to commit or skipped configuration");
        return;
    }
    if (name == "negative_success") {
        f.anchors.logicalAbs[0] = -8392;
        f.anchors.raw[0] = normalizeServoRawSingleTurn(-8392);
        f.driver.raw = normalizeServoRawSingleTurn(-8391);
        f.attempt();
        require(f.driver.writtenTargets == std::vector<int32_t>{-8391} && f.driver.reliable,
                "negative hardware ABS was shifted or truncated");
        return;
    }
    if (name == "partial_two_motors") {
        FeetechServoDriver second;
        second.expectedId = 9;
        second.fault = Fault::AckLost;
        routedDrivers[1] = &second;
        f.anchors.validMask = 3;
        f.anchors.logicalAbs[1] = -8392;
        f.anchors.raw[1] = second.raw;
        f.attempt();
        for (unsigned n = 0; n != 12; ++n) f.step();
        require(f.driver.writtenTargets == std::vector<int32_t>{0} && f.driver.reliable &&
                (f.runtime.multi_turn_unknown_bitmap & 1) == 0 &&
                (f.runtime.multi_turn_unknown_bitmap & 2) != 0 && !second.reliable &&
                second.writtenTargets == std::vector<int32_t>{-8392, -8392, -8392} &&
                f.anchors.pending[1].active && f.anchors.pending[1].failedRequests == 3,
                "partial recovery trusted a failed motor or drifted its target");
        return;
    }
    if (name == "reconnect_during_write") {
        f.driver.fault = Fault::ReconnectDuringWrite;
        f.attempt();
        require(f.anchors.pending[0].active && f.anchors.pending[0].blocked &&
                f.anchors.pending[0].onlineSessionGeneration == 7,
                "mid-write reconnect did not bind pending to its original session");
        f.driver.online = true;
        f.driver.fault = Fault::None;
        f.attempt();
        require(f.driver.writtenTargets == std::vector<int32_t>{0},
                "mid-write reconnect resumed an ambiguous correction");
        f.verifyUnknown();
        return;
    }
    if (name == "before_write_retry") {
        f.driver.fault = Fault::BeforeWrite;
        f.attempt();
        require(!f.anchors.pending[0].active && f.driver.writtenTargets.empty(),
                "pre-write failure froze a target");
        f.driver.fault = Fault::None;
        f.driver.raw = 201;
        f.attempt();
        require(f.driver.writtenTargets == std::vector<int32_t>{1},
                "pre-write retry reused a stale target");
        return;
    }
    f.driver.fault = Fault::AckLost;
    f.attempt();
    require(f.driver.hardwareAbs == 0 && f.anchors.pending[0].active &&
            f.anchors.pending[0].targetAbs == 0 && f.anchors.pending[0].failedRequests == 1,
            "unconfirmed hardware correction did not freeze its target");
    f.verifyUnknown();
    if (name == "ack_lost_retry") {
        f.driver.fault = Fault::None;
        f.attempt();
        require(f.driver.writtenTargets == std::vector<int32_t>{0, 0} && f.driver.reliable &&
                !f.anchors.pending[0].active, "ACK loss changed the retry target from zero to minus 200");
    } else if (name == "retry_limit") {
        f.attempt();
        f.attempt();
        require(f.driver.failed && f.anchors.pending[0].failedRequests == 3,
                "recovery failure limit not latched");
        const auto writes = f.driver.writtenTargets.size();
        for (unsigned n = 0; n != 5; ++n) f.step();
        require(f.driver.writtenTargets.size() == writes, "failed recovery continued to write");
    } else if (name == "new_generation" || name == "route_change") {
        if (name == "new_generation") ++f.driver.feedback.onlineSessionGeneration;
        else {
            routedIds[0] = 9;
            f.driver.expectedId = 9;
        }
        f.attempt();
        require(f.anchors.pending[0].blocked && f.driver.writtenTargets.size() == 1,
                "changed route or session continued old correction");
        f.verifyUnknown();
    } else if (name == "new_trusted_basis") {
        f.driver.reliable = true;
        f.driver.recoveryPending = false;
        ++f.driver.feedback.offlineLastTrustedTimestampMs;
        f.step();
        require(!f.anchors.pending[0].active && f.driver.writtenTargets.size() == 1,
                "new trusted ABS did not retire pending correction");
    } else if (name == "new_snapshot") {
        f.driver.snapshotPending = true;
        f.driver.snapshotAbs = 8392;
        f.driver.snapshotRaw = 200;
        ++f.driver.feedback.offlineLastTrustedTimestampMs;
        require(cacheServoPowerLossSnapshots(&f.runtime, &f.anchors) == 1 &&
                f.anchors.logicalAbs[0] == 8392 && f.anchors.raw[0] == 200 &&
                !f.anchors.pending[0].active, "new trusted snapshot retained stale correction");
    } else require(false, "unknown hotplug scenario");
}

int main(int argc, char** argv) {
    try {
        require(argc == 2, "scenario required");
        const std::string scenario(argv[1]);
        if (scenario.rfind("safety_", 0) == 0) safetyScenario(scenario.substr(7));
        else if (scenario.rfind("hotplug_", 0) == 0) hotplugScenario(scenario.substr(8));
        else if (scenario.rfind("crosspath_", 0) == 0) crosspathScenario(scenario.substr(10));
        else require(false, "unknown scenario prefix");
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
'''


def _production_function(source: str, signature: str) -> str:
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 1
    closing = opening + 1
    while depth:
        depth += (source[closing] == "{") - (source[closing] == "}")
        closing += 1
    return source[start:closing]


@pytest.fixture(scope="module")
def hotplug_harness(tmp_path_factory: pytest.TempPathFactory) -> tuple[Path, dict[str, str]]:
    env = compiler_environment()
    compiler = next((path for name in ("g++", "clang++", "cl")
                     if (path := shutil.which(name, path=env.get("PATH")))), None)
    assert compiler is not None, "A native C++ compiler is required; this test may not skip"
    directory = tmp_path_factory.mktemp("servo-hotplug-native")
    service = (P4_SRC / "tasks/servo_bus/ServoBusService.cpp").read_text(encoding="utf-8")
    start = service.index("struct ServoHotplugPendingSync {")
    cache = _production_function(service, "static uint32_t cacheServoPowerLossSnapshots(")
    end = service.index(cache) + len(cache)
    mapping = _production_function(
        (P4_SRC / "safety/SafetyRuleRegistry.cpp").read_text(encoding="utf-8"),
        "SafetyRuleId ruleForServoRejectReason(",
    )
    context_start = service.index("struct ServoHostRestoreContext {")
    context_end = service.index("};", context_start) + 2
    finish = _production_function(service, "static void finishHostMultiTurnRestore(")
    host_step = _production_function(service, "static bool stepHostMultiTurnRestore(")
    source = directory / "servo_hotplug_test.cpp"
    source.write_text(
        NATIVE_SOURCE.replace("/* PRODUCTION_HOTPLUG_HANDLER */", service[start:end])
        .replace("/* PRODUCTION_RULE_MAPPING */", mapping)
        .replace("/* PRODUCTION_HOST_RESTORE_CONTEXT */", service[context_start:context_end])
        .replace("/* PRODUCTION_HOST_FINISH */", finish)
        .replace("/* PRODUCTION_HOST_STEP */", host_step), encoding="utf-8",
    )
    executable = directory / "servo_hotplug_test.exe"
    production_policy = P4_SRC / "safety/policies/ActuatorSafetyPolicy.cpp"
    if Path(compiler).name.lower() in {"cl", "cl.exe"}:
        command = [compiler, "/std:c++17", "/EHsc", "/utf-8", f"/I{P4_SRC}",
                   str(source), str(production_policy),
                   f"/Fo{directory.resolve()}\\", f"/Fe{executable}"]
    else:
        command = [compiler, "-std=c++17", f"-I{P4_SRC}", str(source),
                   str(production_policy), "-o", str(executable)]
    subprocess.run(command, check=True, cwd=directory, env=env, timeout=120)
    return executable, env


SCENARIOS = (
    "safety_checked_conversion", "safety_identity", "safety_hard_boundary",
    "safety_unknown_before_range", "safety_overflow", "safety_logical_boundary",
    "hotplug_nearest_turn", "hotplug_no_anchor", "hotplug_range_rejected",
    "hotplug_success", "hotplug_negative_success", "hotplug_partial_two_motors",
    "hotplug_reconnect_during_write", "hotplug_before_write_retry", "hotplug_ack_lost_retry",
    "hotplug_retry_limit", "hotplug_new_generation", "hotplug_route_change",
    "hotplug_new_trusted_basis", "hotplug_new_snapshot",
    "crosspath_pending_blocks_host", "crosspath_host_failure_retires_ram",
    "crosspath_host_success", "crosspath_unsafe_rejected",
)


@pytest.mark.parametrize("scenario", SCENARIOS)
def test_abs_safety_and_hotplug_recovery(
    hotplug_harness: tuple[Path, dict[str, str]], scenario: str,
) -> None:
    executable, env = hotplug_harness
    subprocess.run([str(executable), scenario], check=True, cwd=executable.parent,
                   env=env, timeout=10)
