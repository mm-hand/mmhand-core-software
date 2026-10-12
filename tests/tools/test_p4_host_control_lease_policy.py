"""Host-control lease policy regression tests compiled against production C++."""

from __future__ import annotations

import shutil
import os
import subprocess
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
P4_SRC = REPO_ROOT / "ESP32-P4" / "src"


def test_host_control_lease_policy_covers_every_servo_motion_state(tmp_path: Path) -> None:
    compiler = shutil.which("g++") or shutil.which("clang++") or shutil.which("cl")
    if compiler is None:
        pytest.fail("A native C++ compiler is required; install MSVC, GCC or Clang")

    assert "canAcceptCalibration" not in (P4_SRC / "safety/policies/StateAdmissionPolicy.h").read_text(encoding="utf-8")
    source = tmp_path / "host_control_lease_policy_test.cpp"
    source.write_text(
        r'''
#include "safety/policies/HostLeaseSafetyPolicy.h"
#include "safety/policies/StateAdmissionPolicy.h"
#include "safety/policies/ProtocolContractPolicy.h"
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
    // Wire state 4 is retired. Neither old owner pairing may admit motion.
    assert(!requiresHostControlLease(4, SERVO_TARGET_OWNER_CALIBRATION));
    assert(requiresHostControlLease(SYSTEM_STATE_ROPE_INIT_RUNNING,
                                    SERVO_TARGET_OWNER_CALIBRATION));

    assert(!requiresHostControlLease(SYSTEM_STATE_IDLE,
                                     SERVO_TARGET_OWNER_NONE));
    assert(!requiresHostControlLease(SYSTEM_STATE_RUN_DIRECT,
                                     SERVO_TARGET_OWNER_CALIBRATION));
    assert(!requiresHostControlLease(4, SERVO_TARGET_OWNER_CONTROL));
    assert(!p4::system::canAcceptStart(4));
    assert(!p4::system::canAcceptRunDirect(4));
    assert(!p4::system::canAcceptRunFeedback(4));

    constexpr std::uint32_t last = 100;
    assert(isHostControlLeaseFresh(last, last));
    assert(isHostControlLeaseFresh(
        last + mmhand::config::safety::kHostControlLeaseTimeoutMs, last));
    assert(!isHostControlLeaseFresh(
        last + mmhand::config::safety::kHostControlLeaseTimeoutMs + 1, last));
    assert(!isHostControlLeaseFresh(100, 0));

    // Unsigned subtraction keeps the watchdog correct across millis() wrap.
    assert(isHostControlLeaseFresh(5, UINT32_MAX - 5));
    // PAUSE may resume only through START; direct and feedback entry remain rejected.
    assert(p4::system::canAcceptStart(SYSTEM_STATE_PAUSED));
    assert(!p4::system::canAcceptRunDirect(7));
    assert(!p4::system::canAcceptRunFeedback(7));
    // Firmware calibration has no admission API; it runs exclusively in the host.
    assert(p4::system::canAcceptRunFeedback(SYSTEM_STATE_READY));
    assert(!p4::system::canAcceptRunFeedback(SYSTEM_STATE_RECOVERY_REQUIRED));
    assert(p4::system::protocolContractMatches(2, 31, 42, 2, 31, 42));
    assert(!p4::system::protocolContractMatches(1, 31, 42, 2, 31, 42));
    assert(!p4::system::protocolContractMatches(2, 30, 42, 2, 31, 42));
    assert(!p4::system::protocolContractMatches(2, 31, 43, 2, 31, 42));
    return 0;
}
''',
        encoding="utf-8",
    )
    executable = tmp_path / "host_control_lease_policy_test.exe"
    production_sources = [
        str(P4_SRC / "safety" / "policies" / "HostLeaseSafetyPolicy.cpp"),
        str(P4_SRC / "safety" / "policies" / "StateAdmissionPolicy.cpp"),
    ]
    if Path(compiler).name.lower() in {"cl", "cl.exe"}:
        compile_command = [
            compiler, "/std:c++17", "/EHsc", "/utf-8", f"/I{P4_SRC}", str(source),
            *production_sources, f"/Fo{tmp_path.resolve()}\\", f"/Fe{executable}",
        ]
    else:
        compile_command = [
            compiler, "-std=c++17", f"-I{P4_SRC}", str(source),
            *production_sources, "-o", str(executable),
        ]
    subprocess.run(compile_command, check=True, cwd=REPO_ROOT)
    subprocess.run([str(executable)], check=True, cwd=REPO_ROOT)
