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
    directed_encoder_delta,
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


def test_system_status_parses_manual_nvs_extension_and_old_payload():
    legacy = bytearray(61)
    parsed_legacy = parse_system_status_packet(bytes(legacy))
    assert parsed_legacy is not None
    assert parsed_legacy[-1] is False

    extended = legacy + struct.pack(">III", 0x001FFFFF, 0x12345678, 7) + bytes([1])
    parsed = parse_system_status_packet(bytes(extended))
    assert parsed is not None
    assert parsed[26:31] == (
        0x001FFFFF,
        0x12345678,
        7,
        1,
        True,
    )


def test_system_status_parses_calibration_assembly_warning_extension():
    payload = bytearray(102)
    payload[4] = 4
    warning_mask = (1 << 12) | (1 << 15)
    offsets = [0] * ENCODER_COUNT
    offsets[12] = 350
    offsets[15] = 427
    payload.extend(struct.pack(">I", warning_mask))
    for offset in offsets:
        payload.extend(struct.pack(">h", offset))

    parsed = parse_system_status_packet(bytes(payload))

    assert parsed is not None
    assert parsed[4] == 4
    assert parsed[-3] == warning_mask
    assert parsed[-2] == offsets
    assert parsed[-1] is True


def test_system_status_parses_feedback_admission_extension():
    payload = bytearray(148)
    payload.extend(struct.pack(">I", 42))
    payload.extend(bytes([3, 4]))

    parsed = parse_system_status_packet(bytes(payload))

    assert parsed is not None
    # The calibration warning fields deliberately remain at the tuple tail for
    # older desktop callers; the admission fields precede them.
    assert parsed[-11:-7] == (42, 3, 4, True)


def test_system_status_parses_feedback_collision_extension():
    payload = bytearray(154)
    dual_hold_mask = (1 << 0) | (1 << 12) | (1 << 20)
    armed_mask = dual_hold_mask | (1 << 3)
    payload.extend(struct.pack(">II", dual_hold_mask, 9))
    payload.extend(struct.pack(">I", armed_mask))

    parsed = parse_system_status_packet(bytes(payload))

    assert parsed is not None
    assert parsed[-7:-3] == (
        armed_mask,
        dual_hold_mask,
        9,
        True,
    )


def test_system_status_parses_servo_reset_zero_extension():
    payload = bytearray(191)
    overflow_mask = (1 << 2) | (1 << 24)
    success_mask = 1 << 24
    failed_mask = 1 << 2
    invalidated_joint_mask = (1 << 19) | (1 << 20)
    payload[166:170] = struct.pack(">I", overflow_mask)
    payload[170:174] = struct.pack(">I", 7)
    payload[174:178] = struct.pack(">I", 6)
    payload[178:182] = struct.pack(">I", success_mask)
    payload[182:186] = struct.pack(">I", failed_mask)
    payload[186:190] = struct.pack(">I", invalidated_joint_mask)
    payload[190] = 3

    parsed = parse_system_status_packet(bytes(payload))

    assert parsed is not None
    assert parsed[43:51] == (
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
    payload = bytearray([1])
    payload.extend(struct.pack(">III", (1 << ENCODER_COUNT) - 1, 0x12345678, 9))
    payload.append(1)
    for min_raw, max_raw in zip(mins, maxs):
        payload.extend(struct.pack(">HH", min_raw, max_raw))

    parsed = parse_manual_encoder_reference_packet(bytes(payload))

    assert parsed is not None
    assert parsed["valid_bitmap"] == (1 << ENCODER_COUNT) - 1
    assert parsed["reference_crc"] == 0x12345678
    assert parsed["nvs_sequence"] == 9
    assert parsed["nvs_valid"] is True
    assert parsed["min_raw"] == mins
    assert parsed["max_raw"] == maxs
    assert parsed["has_encoder_directions"] is False
    assert parsed["encoder_directions"] == [0] * ENCODER_COUNT


def test_manual_reference_v3_parses_p4_encoder_directions():
    mins = [16000] * ENCODER_COUNT
    maxs = [200] * ENCODER_COUNT
    directions = [-1 if index % 2 == 0 else 1 for index in range(ENCODER_COUNT)]
    payload = bytearray([3])
    payload.extend(struct.pack(">III", (1 << ENCODER_COUNT) - 1, 0x87654321, 12))
    payload.append(1)
    payload.extend(struct.pack(">I", 0x00000003))
    payload.append(1)
    for min_raw, max_raw in zip(mins, maxs):
        payload.extend(struct.pack(">HH", min_raw, max_raw))
    for direction in directions:
        payload.extend(struct.pack(">b", direction))

    parsed = parse_manual_encoder_reference_packet(bytes(payload))

    assert parsed is not None
    assert parsed["version"] == 3
    assert parsed["auto_valid_bitmap"] == 0x00000003
    assert parsed["save_status"] == 1
    assert parsed["has_encoder_directions"] is True
    assert parsed["encoder_directions"] == directions


def test_directed_encoder_delta_matches_p4_cross_wrap_rules():
    assert directed_encoder_delta(16000, 200, 1) == 584
    assert directed_encoder_delta(200, 16000, -1) == 584
    assert directed_encoder_delta(16000, 200, -1) == 15800
    assert directed_encoder_delta(100, 200, 0) is None


def test_manual_reference_snapshot_rejects_wrong_version_or_short_payload():
    assert parse_manual_encoder_reference_packet(b"\x01") is None
    assert parse_manual_encoder_reference_packet(b"\x02" + bytes(97)) is None
