import struct

import pytest

import desktop.core.comm_layer as comm_layer
import desktop.core.communication.state_reducer as state_reducer
from desktop.core.comm_layer import LowerComputerComm
from desktop.core.data_models import HandModel, RAW_TO_DEG
from desktop.core.protocol import (
    DISCONNECT_SENTINEL,
    ENCODER_COUNT,
    MOTOR_COUNT,
    PACKET_TYPE_JOINT_CALIB_SNAPSHOT,
    PACKET_TYPE_MULTITURN_RESTORE_RESULT,
    PACKET_TYPE_SENSOR,
    parse_multiturn_restore_result_packet,
    parse_servo_position_packet,
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


def test_servo_telemetry_packet_includes_signed_current_without_online_array() -> None:
    rows = [(120, -230, 45, 120, 36), (-80, 700, -61, 118, 40)]
    rows.extend([(0, 0, 0, 0, 0)] * 25)
    payload = b"".join(struct.pack(">hhhBB", *row) for row in rows)

    parsed = parse_servo_telem_packet(payload)

    assert parsed == (
        [120, -80] + [0] * 25,
        [-230, 700] + [0] * 25,
        [45, -61] + [0] * 25,
        [120, 118] + [0] * 25,
        [36, 40] + [0] * 25,
    )


def test_servo_position_packet_contains_raw_and_abs_in_one_frame() -> None:
    raw = [12, 4090] + [0] * 25
    angles = list(range(MOTOR_COUNT))
    payload = b"".join(struct.pack(">h", value) for value in raw)
    payload += b"".join(struct.pack(">i", value) for value in angles)
    assert parse_servo_position_packet(payload) == (raw, angles)
    assert parse_servo_position_packet(payload[:-1]) is None


def test_multiturn_restore_result_packet() -> None:
    assert parse_multiturn_restore_result_packet(
        struct.pack(">III", 7, 0x15, 0x20)
    ) == (7, 0x15, 0x20)


def test_multiturn_restore_result_updates_hand_model_end_to_end() -> None:
    comm = LowerComputerComm("")
    comm._state_reducer.last_model = HandModel()
    packets = [
        (
            PACKET_TYPE_MULTITURN_RESTORE_RESULT,
            struct.pack(">IIII", 11, 0x03, 0x04, 1200),
        ),
    ]

    updated = comm._process_packets(packets)

    assert updated is not None
    assert updated.multiturn_restore_result_token == 11
    assert updated.multiturn_restore_restored_mask == 0x03
    assert updated.multiturn_restore_rejected_mask == 0x04


def test_valid_angle_outside_workspace_updates_instead_of_freezing():
    joint_idx = 6
    comm = LowerComputerComm("")
    model = _calibrated_model(joint_idx)
    model.encoders[joint_idx].raw = 624
    model.encoders[joint_idx].angle_error = False
    comm._state_reducer.last_model = model

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
    comm._state_reducer.last_model = model

    updated = comm._process_packets([
        (PACKET_TYPE_SENSOR, _sensor_payload(joint_idx, 900)),
    ])

    assert updated is not None
    assert updated.encoders[joint_idx].raw == 900
    assert updated.encoders[joint_idx].angle_out_of_workspace is False


def test_changed_calibration_snapshot_keeps_last_angle_until_p4_refresh(monkeypatch):
    joint_idx = 6
    comm = LowerComputerComm("")
    model = _calibrated_model(joint_idx, angle_max=60.0)
    model.encoders[joint_idx].raw = 777
    model.encoders[joint_idx].angle_error = False
    model.encoders[joint_idx].angle_out_of_workspace = True
    comm._state_reducer.last_model = model

    new_result = {"joint": joint_idx, "success": True, "angleMax": 70.0}
    monkeypatch.setattr(
        state_reducer,
        "parse_joint_calib_snapshot_packet",
        lambda _payload: dict(new_result),
    )

    updated = comm._process_packets([
        (PACKET_TYPE_JOINT_CALIB_SNAPSHOT, struct.pack(">I", 2234)),
    ])

    assert updated is not None
    assert updated.joint_calib_results[joint_idx] == new_result
    # 快照与映射角属于两条独立上行流。不能先把 Current 人为清零；P4 会在
    # 快照后主动补发同一标定代次的新映射角。
    assert updated.encoders[joint_idx].raw == 777
    assert updated.encoders[joint_idx].angle_error is False
    assert updated.encoders[joint_idx].angle_out_of_workspace is True


def test_repeated_identical_snapshot_does_not_invalidate_fresh_angle(monkeypatch):
    joint_idx = 6
    comm = LowerComputerComm("")
    model = _calibrated_model(joint_idx, angle_max=70.0)
    model.encoders[joint_idx].raw = 321
    model.encoders[joint_idx].angle_error = False
    comm._state_reducer.last_model = model

    monkeypatch.setattr(
        state_reducer,
        "parse_joint_calib_snapshot_packet",
        lambda _payload: dict(model.joint_calib_results[joint_idx]),
    )

    updated = comm._process_packets([
        (PACKET_TYPE_JOINT_CALIB_SNAPSHOT, struct.pack(">I", 3234)),
    ])

    assert updated is not None
    assert updated.encoders[joint_idx].raw == 321
    assert updated.encoders[joint_idx].angle_error is False
