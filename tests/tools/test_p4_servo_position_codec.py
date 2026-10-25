"""Compile the production codec to verify the native ABS wire contract."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
P4_SRC = REPO_ROOT / "ESP32-P4/src"


def test_native_abs_preserves_wire_frames_and_atomic_capacity(tmp_path: Path) -> None:
    compiler = shutil.which("g++") or shutil.which("clang++") or shutil.which("cl")
    if compiler is None:
        pytest.skip("C++ compiler is unavailable")
    source = tmp_path / "servo_position_codec_test.cpp"
    source.write_text(
        r'''
#include "tasks/host_comm/HostProtocolCodec.h"
#include <cassert>
#include <climits>
#include <cstring>

static unsigned read32(const unsigned char* p)
{
    return (unsigned(p[0]) << 24) | (unsigned(p[1]) << 16) |
           (unsigned(p[2]) << 8) | unsigned(p[3]);
}

int main()
{
    ServoAngleData_t position = {};
    position.timestamp = 0xFEDCBA98U;
    for (unsigned i = 0; i < 27; ++i) {
        position.servoAngles[i] = -70000 + int(i) * 5007;
    }
    position.servoAngles[0] = INT_MIN;
    position.servoAngles[26] = INT_MAX;
    unsigned char buffer[180];
    std::memset(buffer, 0xA5, sizeof(buffer));
    assert(p4::host::encodeServoPositionPacket(buffer, 115, position) == 0);
    for (auto value : buffer) assert(value == 0xA5);
    assert(p4::host::encodeServoPositionPacket(nullptr, sizeof(buffer), position) == 0);
    assert(p4::host::encodeServoPositionPacket(buffer, sizeof(buffer), position) == 116);
    // One frame: FE/length/03/27 native ABS/time/FF.
    assert(buffer[0] == 0xFE && buffer[1] == 114 && buffer[2] == 0x03);
    assert(buffer[115] == 0xFF && buffer[116] == 0xA5 && buffer[117] == 0xA5);
    for (unsigned i = 0; i < 27; ++i) {
        assert(read32(buffer + 3 + i * 4) == static_cast<unsigned>(position.servoAngles[i]));
    }
    assert(read32(buffer + 111) == position.timestamp);
    ServoTelemetryData_t telemetry = {};
    telemetry.timestamp = 0xABCDEF12;
    for (unsigned i = 0; i < 27; ++i) {
        telemetry.onlineCheck.generation[i] = 0x11223300 + i;
        telemetry.onlineCheck.major[i] = 3;
        telemetry.onlineCheck.minor[i] = i;
        telemetry.onlineCheck.status[i] = i % 5;
    }
    unsigned char online[200];
    std::memset(online, 0xA5, sizeof(online));
    assert(p4::host::encodeServoOnlineStatusPacket(online, 196, telemetry) == 0);
    for (auto value : online) assert(value == 0xA5);
    assert(p4::host::encodeServoOnlineStatusPacket(online, sizeof(online), telemetry) == 197);
    assert(online[0] == 0xFE && online[1] == 195 && online[2] == 0x20 && online[196] == 0xFF);
    for (unsigned i = 0; i < 27; ++i) {
        assert(read32(online + 3 + 7 * i) == 0x11223300 + i);
        assert(online[7 + 7 * i] == 3 && online[8 + 7 * i] == i && online[9 + 7 * i] == i % 5);
    }
    assert(read32(online + 192) == 0xABCDEF12 && online[197] == 0xA5);
    return 0;
}
''', encoding="utf-8",
    )
    executable = tmp_path / "servo_position_codec_test.exe"
    production_source = P4_SRC / "tasks/host_comm/HostProtocolCodec.cpp"
    if Path(compiler).name.lower() in {"cl", "cl.exe"}:
        compile_command = [
            compiler, "/std:c++17", "/EHsc", "/utf-8", f"/I{P4_SRC}", str(source),
            str(production_source), f"/Fo{tmp_path.resolve()}\\",
            f"/Fe{executable}",
        ]
    else:
        compile_command = [
            compiler, "-std=c++17", f"-I{P4_SRC}", str(source),
            str(production_source), "-o", str(executable),
        ]
    subprocess.run(compile_command, check=True, cwd=REPO_ROOT)
    subprocess.run([str(executable)], check=True, cwd=REPO_ROOT)
