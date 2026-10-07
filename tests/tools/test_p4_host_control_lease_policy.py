"""Host-control lease policy regression tests compiled against production C++."""

from __future__ import annotations

import shutil
import os
import subprocess
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
P4_SRC = REPO_ROOT / "ESP32-P4" / "ServoBoardMain" / "src"


def test_host_control_lease_policy_covers_every_servo_motion_state(tmp_path: Path) -> None:
    compiler = os.environ.get("MMHAND_CXX") or shutil.which("g++") or shutil.which("clang++")
    if compiler is None:
        pytest.fail("A native C++ compiler is required; set MMHAND_CXX to its executable")

    source = tmp_path / "host_control_lease_policy_test.cpp"
    source.write_text(
        r'''
#include "safety/policies/HostLeaseSafetyPolicy.h"
#include "safety/policies/StateAdmissionPolicy.h"
#include "config/SafetyConfig.h"
#include "data/ActuatorTypes.h"
#include "data/SystemTypes.h"

#include <cassert>
#include <cstdint>

int main()
{
    using p4::system::isHostControlLeaseFresh;
    using p4::system::requiresHostControlLease;

    assert(requiresHostControlLease(SYSTEM_STATE_RUN_DIRECT,
                                    SERVO_TARGET_OWNER_CONTROL));
    assert(requiresHostControlLease(SYSTEM_STATE_RUN_FEEDBACK,
                                    SERVO_TARGET_OWNER_CONTROL));
    assert(requiresHostControlLease(SYSTEM_STATE_CALIBRATION_RUNNING,
                                    SERVO_TARGET_OWNER_CALIBRATION));
    assert(requiresHostControlLease(SYSTEM_STATE_ROPE_INIT_RUNNING,
                                    SERVO_TARGET_OWNER_CALIBRATION));

    assert(!requiresHostControlLease(SYSTEM_STATE_IDLE,
                                     SERVO_TARGET_OWNER_NONE));
    assert(!requiresHostControlLease(SYSTEM_STATE_RUN_DIRECT,
                                     SERVO_TARGET_OWNER_CALIBRATION));
    assert(!requiresHostControlLease(SYSTEM_STATE_CALIBRATION_RUNNING,
                                     SERVO_TARGET_OWNER_CONTROL));

    constexpr std::uint32_t last = 100;
    assert(isHostControlLeaseFresh(last, last));
    assert(isHostControlLeaseFresh(
        last + mmhand::config::safety::kHostControlLeaseTimeoutMs, last));
    assert(!isHostControlLeaseFresh(
        last + mmhand::config::safety::kHostControlLeaseTimeoutMs + 1, last));
    assert(!isHostControlLeaseFresh(100, 0));

    // Unsigned subtraction keeps the watchdog correct across millis() wrap.
    assert(isHostControlLeaseFresh(5, UINT32_MAX - 5));
    // Retired PAUSED state 7 cannot grant execution through any admission path.
    assert(!p4::system::canAcceptStart(7));
    assert(!p4::system::canAcceptRunDirect(7));
    assert(!p4::system::canAcceptRunFeedback(7));
    assert(!p4::system::canAcceptCalibration(7));
    assert(p4::system::canAcceptRunFeedback(SYSTEM_STATE_READY));
    assert(!p4::system::canAcceptRunFeedback(SYSTEM_STATE_RECOVERY_REQUIRED));
    return 0;
}
''',
        encoding="utf-8",
    )
    executable = tmp_path / "host_control_lease_policy_test.exe"
    subprocess.run(
        [
            compiler,
            *(["c++"] if Path(compiler).stem == "zig" else []),
            "-std=c++17",
            f"-I{P4_SRC}",
            str(source),
            str(P4_SRC / "safety" / "policies" / "HostLeaseSafetyPolicy.cpp"),
            str(P4_SRC / "safety" / "policies" / "StateAdmissionPolicy.cpp"),
            "-o",
            str(executable),
        ],
        check=True,
        cwd=REPO_ROOT,
    )
    subprocess.run([str(executable)], check=True, cwd=REPO_ROOT)
