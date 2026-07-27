import struct

import pytest

import desktop.core.comm_layer as comm_layer
from desktop.core.comm_layer import LowerComputerComm
from desktop.core.data_models import HandModel, RAW_TO_DEG
from desktop.core.protocol import (
    DISCONNECT_SENTINEL,
    ENCODER_COUNT,
    PACKET_TYPE_JOINT_CALIB_SNAPSHOT,
    PACKET_TYPE_SENSOR,
    parse_servo_raw_packet,
    parse_servo_telem_packet,
)


def _sensor_payload(joint_idx: int, value: int) -> bytes:
    values = [DISCONNECT_SENTINEL] * ENCODER_COUNT
    values[joint_idx] = value
    return b"".join(struct.pack(">h", item) for item in values) + struct.pack(">I", 1234)


def _calibrated_model(joint_idx: int = 6, angle_max: float = 64.104) -> HandModel:
    model = HandModel()
    model.joint_calib_results[joint_idx] = {
        "joint": joint_idx,
        "success": True,
        "angleMax": angle_max,
    }
    return model


def test_servo_telemetry_packet_includes_signed_current() -> None:
    payload = b"".join(
        (
            struct.pack(">hhhBBB", 120, -230, 45, 120, 36, 1),
            struct.pack(">hhhBBB", -80, 700, -61, 118, 40, 0),
        )
    )

    parsed = parse_servo_telem_packet(payload)

    assert parsed == (
        [120, -80],
        [-230, 700],
        [45, -61],
        [120, 118],
        [36, 40],
        [True, False],
    )


def test_servo_raw_packet_contains_current_and_flash_raw() -> None:
    payload = (
        struct.pack(">hh", 12, 4090)
        + struct.pack(">hh", 4000, 8)
        + bytes((1, 0))
    )

    parsed = parse_servo_raw_packet(payload)

    assert parsed == ([12, 4090], [4000, 8], [True, False])


def test_servo_raw_packet_rejects_removed_legacy_layouts() -> None:
    assert parse_servo_raw_packet(bytes(2 * 3)) is None
    assert parse_servo_raw_packet(bytes(2 * 7)) is None


def test_valid_angle_outside_workspace_updates_instead_of_freezing():
    joint_idx = 6
    comm = LowerComputerComm("")
    model = _calibrated_model(joint_idx)
    model.encoders[joint_idx].raw = 624
    model.encoders[joint_idx].angle_error = False
    comm._last_model = model

    updated = comm._process_packets([
        (PACKET_TYPE_SENSOR, _sensor_payload(joint_idx, -1573)),
    ])

    assert updated is not None
    assert updated.encoders[joint_idx].raw == -1573
    assert updated.angles[joint_idx] == pytest.approx(-1573 * RAW_TO_DEG)
    assert updated.encoders[joint_idx].angle_error is False
    assert updated.encoders[joint_idx].angle_out_of_workspace is True


def test_next_in_workspace_angle_clears_workspace_warning():
    joint_idx = 6
    comm = LowerComputerComm("")
    model = _calibrated_model(joint_idx)
    model.encoders[joint_idx].angle_out_of_workspace = True
    comm._last_model = model

    updated = comm._process_packets([
        (PACKET_TYPE_SENSOR, _sensor_payload(joint_idx, 900)),
    ])

    assert updated is not None
    assert updated.encoders[joint_idx].raw == 900
    assert updated.encoders[joint_idx].angle_out_of_workspace is False


def test_changed_calibration_snapshot_invalidates_only_old_mapped_cache(monkeypatch):
    joint_idx = 6
    comm = LowerComputerComm("")
    model = _calibrated_model(joint_idx, angle_max=60.0)
    model.encoders[joint_idx].raw = 777
    model.encoders[joint_idx].angle_error = False
    model.encoders[joint_idx].angle_out_of_workspace = True
    comm._last_model = model

    new_result = {"joint": joint_idx, "success": True, "angleMax": 70.0}
    monkeypatch.setattr(
        comm_layer,
        "parse_joint_calib_snapshot_packet",
        lambda _payload: dict(new_result),
    )

    updated = comm._process_packets([
        (PACKET_TYPE_JOINT_CALIB_SNAPSHOT, struct.pack(">I", 2234)),
    ])

    assert updated is not None
    assert updated.joint_calib_results[joint_idx] == new_result
    assert updated.encoders[joint_idx].raw == 0
    assert updated.encoders[joint_idx].angle_error is True
    assert updated.encoders[joint_idx].angle_out_of_workspace is False


def test_repeated_identical_snapshot_does_not_invalidate_fresh_angle(monkeypatch):
    joint_idx = 6
    comm = LowerComputerComm("")
    model = _calibrated_model(joint_idx, angle_max=70.0)
    model.encoders[joint_idx].raw = 321
    model.encoders[joint_idx].angle_error = False
    comm._last_model = model

    monkeypatch.setattr(
        comm_layer,
        "parse_joint_calib_snapshot_packet",
        lambda _payload: dict(model.joint_calib_results[joint_idx]),
    )

    updated = comm._process_packets([
        (PACKET_TYPE_JOINT_CALIB_SNAPSHOT, struct.pack(">I", 3234)),
    ])

    assert updated is not None
    assert updated.encoders[joint_idx].raw == 321
    assert updated.encoders[joint_idx].angle_error is False
