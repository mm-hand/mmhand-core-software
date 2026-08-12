"""Host-control lease policy regression tests compiled against production C++."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
P4_SRC = REPO_ROOT / "ESP32-P4" / "ServoBoardMain" / "src"


def test_host_control_lease_policy_covers_every_servo_motion_state(tmp_path: Path) -> None:
    compiler = shutil.which("g++") or shutil.which("clang++")
    if compiler is None:
        pytest.skip("C++ compiler is unavailable")

    source = tmp_path / "host_control_lease_policy_test.cpp"
    source.write_text(
        r'''
#include "safety/HostControlLeasePolicy.h"
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
    return 0;
}
''',
        encoding="utf-8",
    )
    executable = tmp_path / "host_control_lease_policy_test.exe"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            f"-I{P4_SRC}",
            str(source),
            str(P4_SRC / "safety" / "HostControlLeasePolicy.cpp"),
            "-o",
            str(executable),
        ],
        check=True,
        cwd=REPO_ROOT,
    )
    subprocess.run([str(executable)], check=True, cwd=REPO_ROOT)
