import struct

import pytest

from desktop.core.protocol import (
    CMD_CLEAR_CALIBRATION_SCOPE,
    build_clear_calibration_scope_cmd,
    parse_servo_logical_range_packet,
)


def test_scoped_calibration_clear_encodes_motor_mask():
    frame = build_clear_calibration_scope_cmd((1 << 3) | (1 << 6))

    assert frame[0] == 0xFE
    assert frame[2] == CMD_CLEAR_CALIBRATION_SCOPE
    assert struct.unpack(">I", frame[3:7])[0] == ((1 << 3) | (1 << 6))
    assert frame[-1] == 0xFF


def test_scoped_calibration_clear_rejects_empty_mask():
    with pytest.raises(ValueError):
        build_clear_calibration_scope_cmd(0)


def test_servo_logical_ranges_are_independent_per_motor():
    valid_mask = (1 << 3) | (1 << 16)
    payload = bytearray(struct.pack(">I", valid_mask))
    for motor in range(27):
        payload.extend(struct.pack(">ii", motor * 1000 - 30719, motor * 1000 + 30719))

    parsed = parse_servo_logical_range_packet(bytes(payload))

    assert parsed is not None
    parsed_mask, mins, maxs = parsed
    assert parsed_mask == valid_mask
    assert (mins[3], maxs[3]) == (-27719, 33719)
    assert (mins[16], maxs[16]) == (-14719, 46719)
