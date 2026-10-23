"""Compile the production codec to verify the existing RAW/ABS wire contract."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
P4_SRC = REPO_ROOT / "ESP32-P4/src"


def test_position_pair_preserves_wire_frames_and_atomic_capacity(tmp_path: Path) -> None:
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
        position.servoRawPositions[i] = short(i * 157);
        position.servoAngles[i] = -70000 + int(i) * 5007;
    }
    position.servoRawPositions[0] = SHRT_MIN;
    position.servoRawPositions[1] = SHRT_MAX;
    position.servoRawPositions[2] = -1;
    position.servoRawPositions[26] = 4095;
    position.servoAngles[0] = INT_MIN;
    position.servoAngles[26] = INT_MAX;
    unsigned char buffer[180];
    std::memset(buffer, 0xA5, sizeof(buffer));
    assert(p4::host::encodeServoPositionPacket(buffer, 169, position) == 0);
    for (auto value : buffer) assert(value == 0xA5);
    assert(p4::host::encodeServoPositionPacket(nullptr, sizeof(buffer), position) == 0);
    assert(p4::host::encodeServoPositionPacket(buffer, sizeof(buffer), position) == 170);
    // One frame: FE/length/03/27 raw/time/27 ABS/time/FF.
    assert(buffer[0] == 0xFE && buffer[1] == 168 && buffer[2] == 0x03);
    assert(buffer[169] == 0xFF && buffer[170] == 0xA5 && buffer[171] == 0xA5);
    for (unsigned i = 0; i < 27; ++i) {
        const unsigned char* raw = buffer + 3 + i * 2;
        const unsigned rawBits = (unsigned(raw[0]) << 8) | raw[1];
        assert(rawBits == static_cast<unsigned short>(position.servoRawPositions[i]));
        assert(read32(buffer + 57 + i * 4) == static_cast<unsigned>(position.servoAngles[i]));
    }
    assert(read32(buffer + 165) == position.timestamp);
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
