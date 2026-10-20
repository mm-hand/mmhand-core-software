"""所有上位机直控共用的字节契约、执行确认和独立反馈时钟。"""

import json
from pathlib import Path
import struct

import pytest

import desktop.core.communication.state_reducer as state_reducer_module
from desktop.core.comm_layer import LowerComputerComm
from desktop.core.data_models import HandModel
from desktop.core.protocol import (
    CMD_MOTOR_POS_ABS,
    CMD_MOTOR_DIRECT_RELEASE,
    CMD_SERVO_REBASE_ZERO,
    MOTOR_COUNT,
    PACKET_TYPE_ENCODER_RAW,
    PACKET_TYPE_DIRECT_TARGET_RESULT,
    PACKET_TYPE_PROTO_ACK,
    PACKET_TYPE_SERVO_ANGLE,
    PACKET_TYPE_SERVO_RAW,
    PACKET_TYPE_SERVO_TELEM,
    PACKET_TYPE_SERVO_LOGICAL_RANGE,
    build_motor_pos_abs_cmd,
    build_motor_direct_release_cmd,
    parse_direct_target_result_packet,
    split_p4_timestamp,
)


def test_targets_preserve_signed_abs_speed_acc_and_transaction_token():
    targets = [-2147483648, 2147483647, -64] + [0] * (MOTOR_COUNT - 3)
    moving_mask = 1 << 26
    frame = build_motor_pos_abs_cmd(
        targets, token=0xF0123456, motor_mask=moving_mask, speed=400, acc=20)
    assert frame[:3] == bytes((0xFE, 121, CMD_MOTOR_POS_ABS))
    assert len(frame) == 123 and frame[-1] == 0xFF
    assert struct.unpack(">II27iHB", frame[3:-1]) == (0xF0123456, moving_mask, *targets, 400, 20)
    comm = LowerComputerComm("")
    assert comm._encode_command(
        ("motor_pos_abs", 0xF0123456, moving_mask, targets, 400, 20)
    ) == frame


@pytest.mark.parametrize("token,targets,speed,acc", [
    (-1, [0] * MOTOR_COUNT, 400, 20),
    (1 << 32, [0] * MOTOR_COUNT, 400, 20),
    (1, [0] * (MOTOR_COUNT - 1), 400, 20),
    (1, [1 << 31] + [0] * (MOTOR_COUNT - 1), 400, 20),
    (1, [0] * MOTOR_COUNT, -1, 20),
    (1, [0] * MOTOR_COUNT, 1 << 16, 20),
    (1, [0] * MOTOR_COUNT, 400, -1),
    (1, [0] * MOTOR_COUNT, 400, 256),
])
def test_builder_rejects_outside_wire_contract(token, targets, speed, acc):
    with pytest.raises(ValueError):
        build_motor_pos_abs_cmd(targets, token=token, motor_mask=1, speed=speed, acc=acc)


@pytest.mark.parametrize("mask", [-1, 1 << 27, 1 << 32])
def test_builder_rejects_outside_requested_mask(mask):
    with pytest.raises(ValueError):
        build_motor_pos_abs_cmd([0] * MOTOR_COUNT, token=1, motor_mask=mask, speed=400, acc=20)


def test_ordinary_batch_uses_same_direct_wire_format_and_current_scope_defaults():
    targets = [-100, 200] + [0] * (MOTOR_COUNT - 2)
    frame = build_motor_pos_abs_cmd(targets)
    assert frame[:3] == bytes((0xFE, 121, CMD_MOTOR_POS_ABS))
    assert struct.unpack(">II27iHB", frame[3:-1]) == (0, 0, *targets, 400, 254)
    comm = LowerComputerComm("")
    assert comm._encode_command(("motor_pos_abs", targets)) == frame
    assert comm._encode_command(("motor_pos_abs", 0, 0, targets, 400, 254)) == frame


def test_zero_mask_can_track_current_scope_execution_without_a_calibration_mode():
    targets = [0] * MOTOR_COUNT
    frame = build_motor_pos_abs_cmd(targets, token=12)
    assert struct.unpack(">II27iHB", frame[3:-1]) == (12, 0, *targets, 400, 254)


def test_protocol_has_one_direct_entry_and_no_host_calibration_motion_protocol():
    from desktop.core import protocol

    schema_path = Path(__file__).resolve().parents[3] / "desktop/resources/config/upper_protocol_schema.json"
    schema = json.loads(schema_path.read_text(encoding="utf-8"))
    commands = {item["name"]: item for item in schema["commands"]}
    assert commands["CMD_MOTOR_POS_ABS"]["payload_bytes"] == 119
    assert commands["CMD_MOTOR_DIRECT_RELEASE"]["payload_bytes"] == 4
    assert commands["CMD_MOTOR_ABS_ZERO"]["value"] == 0xF1
    assert commands["CMD_MOTOR_ABS_ZERO"]["payload_bytes"] == 4
    assert commands["CMD_MOTOR_ABS_ZERO"]["requires_handshake"] is True
    assert not any("HOST_CALIBRATION" in item["name"] for item in schema["commands"] + schema["packets"])
    assert not hasattr(protocol, "build_host_calibration_targets_cmd")
    assert not hasattr(protocol, "build_host_calibration_release_cmd")


def test_release_routes_a_generic_completed_transaction_token():
    frame = build_motor_direct_release_cmd(12)
    assert frame == bytes((0xFE, 6, CMD_MOTOR_DIRECT_RELEASE)) + struct.pack(">I", 12) + b"\xFF"
    comm = LowerComputerComm("")
    assert comm._encode_command(("motor_direct_release", 12)) == frame
    for token in (-1, 1 << 32):
        with pytest.raises(ValueError):
            build_motor_direct_release_cmd(token)


def test_release_zero_finishes_current_direct_generation_for_ordinary_control():
    frame = build_motor_direct_release_cmd()
    assert frame == bytes((0xFE, 6, CMD_MOTOR_DIRECT_RELEASE)) + bytes(4) + b"\xFF"
    comm = LowerComputerComm("")
    assert comm._encode_command(("motor_direct_release", 0)) == frame


def test_retired_session_drops_motion_but_keeps_rebase_and_result_queries():
    comm = LowerComputerComm('')
    target = ('motor_pos_abs', 12, 1, [0] * MOTOR_COUNT, 400, 20)
    comm.tx_queue.put_nowait(target)
    comm.tx_queue.put_nowait(('motor_direct_release', 12))
    comm.tx_queue.put_nowait(('servo_rebase_zero', 1))
    comm.tx_queue.put_nowait(('motor_abs_zero', 1 << 9))
    comm.tx_queue.put_nowait(('state_sync', 64))
    assert comm.invalidate_pending_direct_commands() == 2
    assert list(comm.tx_queue.queue) == [('servo_rebase_zero', 1), ('motor_abs_zero', 1 << 9), ('state_sync', 64)]


def test_result_requires_whole_batch_applied_or_rejected():
    assert parse_direct_target_result_packet(
        struct.pack(">IIIIB", 12, 0x04000001, 0x04000001, 0, 2)
    ) == (12, 0x04000001, 0x04000001, 0, 2)
    assert parse_direct_target_result_packet(
        struct.pack(">IIIIB", 12, 3, 0, 3, 3)
    ) == (12, 3, 0, 3, 3)
    assert parse_direct_target_result_packet(
        struct.pack(">IIIIB", 0, 3, 3, 0, 2)
    ) == (0, 3, 3, 0, 2)
    # 跨 BUS 写入可能部分成功；真实 REJECTED 掩码不能被伪装为整批未写入。
    assert parse_direct_target_result_packet(
        struct.pack(">IIIIB", 12, 3, 1, 2, 3)
    ) == (12, 3, 1, 2, 3)
    assert parse_direct_target_result_packet(bytes(17)) == (0, 0, 0, 0, 0)
    for fields in [
        (12, 3, 1, 2, 2), (12, 3, 3, 3, 2),
        (12, 1 << 27, 1 << 27, 0, 2), (12, 3, 3, 0, 1),
        (12, 0, 0, 0, 2), (12, 3, 3, 0, 3),
    ]:
        assert parse_direct_target_result_packet(
            struct.pack(">IIIIB", *fields)
        ) is None
    assert parse_direct_target_result_packet(bytes(16)) is None
    assert parse_direct_target_result_packet(bytes(18)) is None


def test_final_result_routes_independently_from_ack_and_rebase_result():
    comm = LowerComputerComm("")
    payload = struct.pack(">IIIIB", 12, 3, 3, 0, 2)
    assert split_p4_timestamp(
        PACKET_TYPE_DIRECT_TARGET_RESULT, payload + struct.pack(">I", 500)
    ) == (payload, 500)
    model = comm._process_packets([(
        PACKET_TYPE_DIRECT_TARGET_RESULT, payload + struct.pack(">I", 500),
    )])
    assert model.direct_target_token == 12
    assert model.direct_target_requested_mask == 3
    assert model.direct_target_applied_mask == 3
    assert model.direct_target_rejected_mask == 0
    assert model.direct_target_status == 2
    assert model.direct_target_result_rx_ts > 0
    assert model.has_direct_target_result
    assert model.servo_rebase_zero_result_token == 0
    after_ack = comm._process_packets([(
        PACKET_TYPE_PROTO_ACK,
        bytes((CMD_SERVO_REBASE_ZERO, 1, 2)) + struct.pack(">I", 501),
    )])
    assert after_ack.direct_target_token == 12
    assert after_ack.direct_target_status == 2


def test_ordinary_untracked_direct_execution_result_is_still_reduced():
    comm = LowerComputerComm("")
    targets = [0] * MOTOR_COUNT
    assert comm._encode_command(("motor_pos_abs", targets))[2] == CMD_MOTOR_POS_ABS
    model = comm._process_packets([(
        PACKET_TYPE_DIRECT_TARGET_RESULT,
        struct.pack(">IIIIBI", 0, 3, 3, 0, 2, 500),
    )])
    assert model.has_direct_target_result
    assert model.direct_target_token == 0
    assert model.direct_target_status == 2
    assert model.direct_target_requested_mask == model.direct_target_applied_mask == 3
    assert model.direct_target_rejected_mask == 0


def test_partial_bus_write_reports_rejection_with_actual_written_mask():
    comm = LowerComputerComm("")
    model = comm._process_packets([(
        PACKET_TYPE_DIRECT_TARGET_RESULT,
        struct.pack(">IIIIBI", 12, 3, 1, 2, 3, 500),
    )])
    assert model.has_direct_target_result
    assert model.direct_target_status == 3
    assert model.direct_target_applied_mask == 1
    assert model.direct_target_rejected_mask == 2
    assert model.direct_target_applied_mask != model.direct_target_requested_mask


def test_each_feedback_source_keeps_its_own_timestamp_and_copy():
    comm = LowerComputerComm("")
    model = comm._process_packets([
        (PACKET_TYPE_ENCODER_RAW, struct.pack(">21HI", *range(21), 90)),
        (PACKET_TYPE_SERVO_ANGLE, struct.pack(">27iI", *range(27), 91)),
        (PACKET_TYPE_SERVO_TELEM,
         b"".join(struct.pack(">hhhBB", 1, -2, 3, 12, 24) for _ in range(27))
         + struct.pack(">I", 92)),
        (PACKET_TYPE_DIRECT_TARGET_RESULT,
         struct.pack(">IIIIBI", 12, 3, 3, 0, 2, 93)),
        (PACKET_TYPE_SERVO_LOGICAL_RANGE,
         struct.pack(">I", 3) + b"".join(struct.pack(">ii", -30000, 30000) for _ in range(27))
         + struct.pack(">I", 94)),
    ])
    assert model.encoder_raw_p4_time_ms == 90
    assert model.servo_feedback_p4_time_ms == 91
    assert model.servo_telem_p4_time_ms == 92
    assert model.encoder_raw_last_rx_ts > 0 and model.servo_telem_last_rx_ts > 0
    assert model.servo_logical_range_p4_time_ms == 94
    assert model.servo_logical_range_last_rx_ts > 0
    copied = HandModel()
    copied.copy_state_from(model)
    assert copied.encoder_raw_p4_time_ms == 90
    assert copied.servo_telem_p4_time_ms == 92
    assert copied.servo_telem_last_rx_ts == model.servo_telem_last_rx_ts
    assert copied.direct_target_token == 12
    assert copied.direct_target_applied_mask == 3
    assert copied.direct_target_result_rx_ts > 0
    assert copied.has_direct_target_result
    assert copied.servo_logical_range_p4_time_ms == 94
    assert copied.servo_logical_range_last_rx_ts == model.servo_logical_range_last_rx_ts


def test_paired_position_packets_can_arrive_in_separate_rx_batches_without_losing_pair(
    monkeypatch,
):
    # 每批都要拿到快照：关闭发布节流（默认 20 ms 只约束高频遥测，不改变配对语义）。
    monkeypatch.setattr(state_reducer_module, "STATE_SNAPSHOT_MIN_INTERVAL_S", 0.0)
    comm = LowerComputerComm('')
    previous = comm._process_packets([
        (PACKET_TYPE_SERVO_RAW, struct.pack('>27hI', *([100] * 27), 1000)),
        (PACKET_TYPE_SERVO_ANGLE, struct.pack('>27iI', *([0] * 27), 1000)),
    ])
    assert previous.servo_feedback_p4_time_ms == previous.servo_raw_p4_time_ms == 1000
    raw = comm._process_packets([
        (PACKET_TYPE_SERVO_RAW, struct.pack('>27hI', *([101] * 27), 1020)),
    ])
    assert raw.servo_raw_p4_time_ms == 1020 and raw.servo_feedback_p4_time_ms == 1000
    paired = comm._process_packets([
        (PACKET_TYPE_SERVO_ANGLE, struct.pack('>27iI', *([1] * 27), 1020)),
    ])
    copied = HandModel()
    copied.copy_state_from(paired)
    assert copied.servo_feedback_p4_time_ms == copied.servo_raw_p4_time_ms == 1020
    assert copied.servo_angles[2] == 1 and copied.servo_raw_positions[2] == 101


def test_invalid_result_does_not_replace_the_last_confirmed_transaction():
    comm = LowerComputerComm("")
    comm._process_packets([(
        PACKET_TYPE_DIRECT_TARGET_RESULT,
        struct.pack(">IIIIBI", 12, 3, 3, 0, 2, 93),
    )])
    assert comm._process_packets([(
        PACKET_TYPE_DIRECT_TARGET_RESULT,
        struct.pack(">IIIIBI", 13, 3, 1, 2, 2, 94),
    )]) is None
    assert comm._state_reducer.last_model.direct_target_token == 12
