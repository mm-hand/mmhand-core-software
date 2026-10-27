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
static void require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
namespace p4::safety { /* PRODUCTION_RULE_MAPPING */ }
static int16_t normalizeServoRawSingleTurn(int32_t value) { return mmhand::servo::normalizeRawSingleTurn(value); }
struct SystemRuntimeContext {
    uint32_t multi_turn_unknown_bitmap = 3;
};
static void sharedAtomicFetchAnd(uint32_t* value, uint32_t mask) { *value &= mask; }
static void sharedAtomicStore(uint32_t* value, uint32_t data) { *value = data; }
static bool safe = true;
static bool servoCoordinateMaintenanceSafe(SystemRuntimeContext*) { return safe; }
#define FW_LOG_INFO_PRINTF(...) ((void)0)
namespace p4::platform::time { static uint32_t clockMs = 1000; static uint32_t monotonicMillis() { return clockMs; } }
class FeetechServoDriver {
public:
    bool online=true, pending=true, configured=true, fail=false;
    int32_t native=200, saved=8392;
    unsigned writes=0;
    bool snapshot=false;
    bool isOnline(uint8_t) const { return online; }
    bool isRecoveryAttemptDue(uint8_t, uint32_t) const { return pending; }
    bool checkOnlineConfiguration(uint8_t) { return configured; }
    int32_t getAbsolutePosition(uint8_t) const { return native; }
    void finishRecoveryAttempt(uint8_t, bool, uint32_t) { pending=false; }
    bool consumeOfflineTrustedSnapshot(uint8_t, int32_t* absolute) {
        if (!snapshot) return false;
        snapshot=false; *absolute=saved; return true;
    }
};
struct ServoRoute { uint8_t busIndex=0, servoId=1; };
static FeetechServoDriver* routeDriver=nullptr;
static FeetechServoDriver* resolveMotorDriver(uint8_t channel, ServoRoute*, bool) { return channel==0 ? routeDriver : nullptr; }
namespace p4::servo {
enum class RestoreResult { Pending, Success, Failed };
class AbsoluteRestore {
public:
    int32_t target=0;
    bool defer=false;
    RestoreResult step(FeetechServoDriver* driver, uint8_t, int32_t value, uint32_t) {
        if (!driver) return RestoreResult::Failed;
        target=value;
        if (defer) return RestoreResult::Pending;
        ++driver->writes;
        if (driver->fail || value < -30719 || value > 30719) return RestoreResult::Failed;
        driver->native=value;
        return RestoreResult::Success;
    }
    void cancel() {}
};
}
/* PRODUCTION_SERVICE */
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
    p4::actuator::ServoSafetyContext context{};
    context.activeOwner = SERVO_TARGET_OWNER_CONTROL;
    context.systemState = SYSTEM_STATE_RUN_DIRECT;
    context.nowMs = 1000;
    context.maxTargetAgeMs = 500;
    context.onlineMotorMask = context.routedMotorMask = context.controllableRouteMask = 1;
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
    } else require(false, "unknown safety scenario");
    const auto result = p4::actuator::evaluateCommand(batch, 0, context);
    require(!result.allowed && result.reason == expected &&
            result.hardwareAbs == batch.commands[0].position, "ABS safety gate changed priority or coordinate");
}

static void recoveryScenario(const std::string& name) {
    SystemRuntimeContext runtime;
    FeetechServoDriver driver; routeDriver=&driver;
    ServoHotplugRecoveryAnchors anchors{}; anchors.validMask=1; anchors.logicalAbs[0]=8392;
    ServoOnlineRecoveryContext online{0xFF,0};
    p4::servo::AbsoluteRestore writer;
    if (name == "nearest") {
        require(restoreTargetFromAnchor(8190, 5)==8197, "positive wrap lost a turn");
        require(restoreTargetFromAnchor(8197,4090)==8186, "reverse wrap lost a turn");
        require(restoreTargetFromAnchor(-73,4023)==-73, "negative anchor lost its sign");
        return;
    }
    if (name == "freeze") {
        driver.snapshot=true;
        require(cacheServoPowerLossSnapshots(&runtime,&anchors)==1 && anchors.logicalAbs[0]==8392,
                "native checkpoint not frozen");
        require(cacheServoPowerLossSnapshots(&runtime,&anchors)==0, "checkpoint consumed twice"); return;
    }
    if (name == "no_anchor") anchors.validMask=0;
    if (name == "unsafe") safe=false;
    if (name == "failure") driver.fail=true;
    if (name == "pending") writer.defer=true;
    processServoOnlineTransitions(&runtime,&anchors,&online,&writer);
    if (name=="no_anchor" || name=="unsafe") require(driver.writes==0 && runtime.multi_turn_unknown_bitmap==3,
        "missing basis or unsafe state changed hardware");
    else if (name=="failure") require(driver.writes==1 && anchors.validMask==1 && runtime.multi_turn_unknown_bitmap==3,
        "failed hotplug discarded basis or granted trust");
    else if (name=="pending") require(driver.writes==0 && online.activeChannel==0 && runtime.multi_turn_unknown_bitmap==3,
        "pending hotplug prematurely finished");
    else require(driver.writes==1 && driver.native==8392 && anchors.validMask==0 && runtime.multi_turn_unknown_bitmap==2,
        "hotplug did not use the shared native restore");
}
int main(int argc,char** argv) {
    try { const std::string name(argv[1]);
        if (name.rfind("safety_",0)==0) safetyScenario(name.substr(7)); else recoveryScenario(name);
        return 0;
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
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
    names = ("struct ServoHotplugRecoveryAnchors {",
             "struct ServoOnlineRecoveryContext {",
             "static int32_t restoreTargetFromAnchor(", "static void processServoOnlineTransitions(",
             "static uint32_t cacheServoPowerLossSnapshots(")
    code = "\n".join(_production_function(service, name) + (";" if name.startswith("struct") else "") for name in names)
    mapping = _production_function((P4_SRC / "safety/SafetyRuleRegistry.cpp").read_text(), "SafetyRuleId ruleForServoRejectReason(")
    source = directory / "servo_hotplug_test.cpp"
    source.write_text(NATIVE_SOURCE.replace("/* PRODUCTION_SERVICE */", code)
                      .replace("/* PRODUCTION_RULE_MAPPING */", mapping), encoding="utf-8")
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


SCENARIOS = ("safety_checked_conversion", "safety_identity", "safety_hard_boundary",
    "safety_unknown_before_range", "safety_overflow", "nearest", "freeze", "success",
    "no_anchor", "unsafe", "failure", "pending")

@pytest.mark.parametrize("scenario", SCENARIOS)
def test_abs_safety_and_hotplug_recovery(
    hotplug_harness: tuple[Path, dict[str, str]], scenario: str,
) -> None:
    executable, env = hotplug_harness
    subprocess.run([str(executable), scenario], check=True, cwd=executable.parent,
                   env=env, timeout=10)
