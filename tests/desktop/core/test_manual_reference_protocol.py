import struct

import pytest

from desktop.core.protocol import (
    CMD_MANUAL_ENCODER_REFERENCE,
    CMD_SERVO_RESET_ZERO,
    ENCODER_COUNT,
    PROTOCOL_HEADER,
    PROTOCOL_TAIL,
    build_clear_manual_encoder_reference_cmd,
    build_manual_encoder_reference_cmd,
    build_servo_reset_zero_cmd,
    parse_manual_encoder_reference_packet,
    parse_system_status_packet,
)


def test_manual_reference_command_contains_all_21_big_endian_pairs():
    mins = [100 + i for i in range(ENCODER_COUNT)]
    maxs = [16000 - i for i in range(ENCODER_COUNT)]
    frame = build_manual_encoder_reference_cmd(mins, maxs)

    assert frame[0] == PROTOCOL_HEADER
    assert frame[2] == CMD_MANUAL_ENCODER_REFERENCE
    assert frame[-1] == PROTOCOL_TAIL
    payload = frame[3:-1]
    assert payload[0] == 1
    for joint in range(ENCODER_COUNT):
        offset = 1 + joint * 4
        assert struct.unpack(">HH", payload[offset:offset + 4]) == (
            mins[joint],
            maxs[joint],
        )


def test_manual_reference_command_rejects_partial_or_out_of_range_data():
    with pytest.raises(ValueError):
        build_manual_encoder_reference_cmd([0] * (ENCODER_COUNT - 1), [1] * ENCODER_COUNT)
    with pytest.raises(ValueError):
        build_manual_encoder_reference_cmd([0] * ENCODER_COUNT, [16384] * ENCODER_COUNT)


def test_clear_manual_reference_uses_same_fixed_wire_length():
    frame = build_clear_manual_encoder_reference_cmd()
    payload = frame[3:-1]
    assert frame[2] == CMD_MANUAL_ENCODER_REFERENCE
    assert len(payload) == 1 + ENCODER_COUNT * 4
    assert payload == bytes(len(payload))


def test_system_status_requires_current_length_and_parses_manual_fields():
    legacy = bytearray(43)
    parsed_legacy = parse_system_status_packet(bytes(legacy))
    assert parsed_legacy is None

    extended = legacy + struct.pack(">III", 0x001FFFFF, 0x12345678, 7) + bytes([1])
    parsed = parse_system_status_packet(bytes(extended).ljust(173, b"\x00"))
    assert parsed is not None
    assert parsed[19:24] == (
        0x001FFFFF,
        0x12345678,
        7,
        1,
        True,
    )


def test_system_status_parses_calibration_assembly_warning_extension():
    payload = bytearray(84)
    payload[4] = 4
    warning_mask = (1 << 12) | (1 << 15)
    offsets = [0] * ENCODER_COUNT
    offsets[12] = 350
    offsets[15] = 427
    payload.extend(struct.pack(">I", warning_mask))
    for offset in offsets:
        payload.extend(struct.pack(">h", offset))

    parsed = parse_system_status_packet(bytes(payload).ljust(173, b"\x00"))

    assert parsed is not None
    assert parsed[4] == 4
    assert parsed[-3] == warning_mask
    assert parsed[-2] == offsets
    assert parsed[-1] is True


def test_system_status_parses_feedback_admission_extension():
    payload = bytearray(130)
    payload.extend(struct.pack(">I", 42))
    payload.extend(bytes([3, 4]))

    parsed = parse_system_status_packet(bytes(payload).ljust(173, b"\x00"))

    assert parsed is not None
    # The calibration warning fields deliberately remain at the tuple tail for
    # older desktop callers; the admission fields precede them.
    assert parsed[-11:-7] == (42, 3, 4, True)


def test_system_status_parses_feedback_collision_extension():
    payload = bytearray(136)
    dual_hold_mask = (1 << 0) | (1 << 12) | (1 << 20)
    armed_mask = dual_hold_mask | (1 << 3)
    payload.extend(struct.pack(">II", dual_hold_mask, 9))
    payload.extend(struct.pack(">I", armed_mask))

    parsed = parse_system_status_packet(bytes(payload).ljust(173, b"\x00"))

    assert parsed is not None
    assert parsed[-7:-3] == (
        armed_mask,
        dual_hold_mask,
        9,
        True,
    )


def test_system_status_parses_servo_reset_zero_extension():
    payload = bytearray(173)
    overflow_mask = (1 << 2) | (1 << 24)
    success_mask = 1 << 24
    failed_mask = 1 << 2
    invalidated_joint_mask = (1 << 19) | (1 << 20)
    payload[148:152] = struct.pack(">I", overflow_mask)
    payload[152:156] = struct.pack(">I", 7)
    payload[156:160] = struct.pack(">I", 6)
    payload[160:164] = struct.pack(">I", success_mask)
    payload[164:168] = struct.pack(">I", failed_mask)
    payload[168:172] = struct.pack(">I", invalidated_joint_mask)
    payload[172] = 3

    parsed = parse_system_status_packet(bytes(payload).ljust(173, b"\x00"))

    assert parsed is not None
    assert parsed[36:44] == (
        overflow_mask,
        7,
        6,
        success_mask,
        failed_mask,
        invalidated_joint_mask,
        3,
        True,
    )


def test_servo_reset_zero_command_contains_big_endian_motor_mask():
    motor_mask = (1 << 0) | (1 << 16) | (1 << 26)
    frame = build_servo_reset_zero_cmd(motor_mask)

    assert frame[0] == PROTOCOL_HEADER
    assert frame[2] == CMD_SERVO_RESET_ZERO
    assert frame[-1] == PROTOCOL_TAIL
    assert struct.unpack(">I", frame[3:-1])[0] == motor_mask


def test_manual_reference_snapshot_parses_all_21_pairs():
    mins = [100 + i for i in range(ENCODER_COUNT)]
    maxs = [1000 + i for i in range(ENCODER_COUNT)]
    payload = bytearray([4])
    payload.extend(struct.pack(">III", (1 << ENCODER_COUNT) - 1, 0x12345678, 9))
    payload.append(1)
    payload.extend(struct.pack(">I", 0))
    payload.append(1)
    for min_raw, max_raw in zip(mins, maxs):
        payload.extend(struct.pack(">HH", min_raw, max_raw))

    parsed = parse_manual_encoder_reference_packet(bytes(payload))

    assert parsed is not None
    assert parsed["version"] == 4
    assert parsed["valid_bitmap"] == (1 << ENCODER_COUNT) - 1
    assert parsed["reference_crc"] == 0x12345678
    assert parsed["revision"] == 9
    assert parsed["committed"] is True
    assert parsed["min_raw"] == mins
    assert parsed["max_raw"] == maxs


def test_manual_reference_snapshot_rejects_wrong_version_or_short_payload():
    assert parse_manual_encoder_reference_packet(b"\x01") is None
    assert parse_manual_encoder_reference_packet(b"\x02" + bytes(97)) is None
