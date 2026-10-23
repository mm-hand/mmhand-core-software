"""P1 通信组件拆分后的职责边界与兼容回归测试。

职责：验证传输、解帧、路由、状态归并和 ACK 跟踪可以脱离 UI 独立工作。
线程：pytest 主线程执行；ACK 用例以同步发布模拟 RX 线程结果。
输入：假串口、分片协议帧、路由包和最小 HandModel 更新函数。
输出：组件行为断言及 LowerComputerComm 兼容属性断言。
状态所有权：每个测试独占组件实例，不共享串口或模型状态。
禁止依赖：禁止启动真实串口、NiceGUI 事件循环、ROS2 或 P4 硬件。
"""

from __future__ import annotations

import struct
from types import SimpleNamespace

import pytest

from desktop.core.comm_layer import LowerComputerComm
from desktop.core.communication import (
    AckTracker,
    EventChannel,
    FrameDecoder,
    P4PacketRouter,
    SerialTransport,
    StateReducer,
)
from desktop.core.data_models import HandModel
from desktop.core.hand_backend import HandBackend
from desktop.core.communication.feedback_history import FeedbackHistoryError
from desktop.core.protocol import (
    CMD_RESET,
    DISCONNECT_SENTINEL,
    ENCODER_COUNT,
    MOTOR_COUNT,
    PACKET_TYPE_ENCODER_RAW,
    PACKET_TYPE_SERVO_POSITION,
    PACKET_TYPE_SERVO_REBASE_ZERO_RESULT,
    PACKET_TYPE_SERVO_TELEM,
    PACKET_TYPE_SYSTEM_STATUS,
    PACKET_TYPE_TASK_DIAG,
    PROTOCOL_HEADER,
    PROTOCOL_TAIL,
    PROTO_ACK_STATUS_OK,
    TASK_RUNTIME_NAMES,
    split_p4_timestamp,
)


class _FakeSerial:
    """模拟 pyserial 构造式打开及默认启用的控制线。"""

    def __init__(self, port=None, baudrate=None, timeout=None) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.dtr = True
        self.rts = True
        self.is_open = port is not None
        self.in_waiting = 0
        self.writes: list[bytes] = []

    def open(self) -> None:
        self.lines_at_open = (self.dtr, self.rts)
        self.is_open = True

    def close(self) -> None:
        self.is_open = False

    def write(self, data: bytes) -> int:
        self.writes.append(bytes(data))
        return len(data)

    def read(self, _count: int) -> bytes:
        return b""


def test_serial_transport_owns_open_write_and_close() -> None:
    transport = SerialTransport("COM_TEST", 1_000_000, 0.02, _FakeSerial)

    assert transport.open() is True
    assert transport.is_open is True
    assert transport.write(b"abc") == 3
    assert transport.serial.writes == [b"abc"]

    transport.close()
    assert transport.is_open is False
    assert transport.serial is None


def test_native_usb_transport_sets_inactive_reset_lines_before_open() -> None:
    transport = SerialTransport(
        "COM_TOUCH",
        921_600,
        0.02,
        _FakeSerial,
    )

    assert transport.open() is True
    assert transport.serial.lines_at_open == (False, False)
    assert transport.serial.port == 'COM_TOUCH'
    assert transport.serial.write_timeout == 0.5


def test_frame_decoder_handles_log_bytes_fragmentation_and_sticky_frames() -> None:
    log_bytes: list[int] = []
    flush_count = 0

    def _flush() -> None:
        nonlocal flush_count
        flush_count += 1

    decoder = FrameDecoder(
        PROTOCOL_HEADER,
        PROTOCOL_TAIL,
        log_byte_callback=log_bytes.append,
        flush_log_callback=_flush,
    )
    frame_a = bytes((PROTOCOL_HEADER, 4, 0x31, 0xAA, 0xBB, PROTOCOL_TAIL))
    frame_b = bytes((PROTOCOL_HEADER, 3, 0x32, 0xCC, PROTOCOL_TAIL))

    assert decoder.feed(b"FW\n" + frame_a[:3]) == []
    assert decoder.feed(frame_a[3:] + frame_b) == [
        (0x31, b"\xAA\xBB"),
        (0x32, b"\xCC"),
    ]
    assert bytes(log_bytes) == b"FW\n"
    assert flush_count == 2
    assert decoder.buffer == bytearray()


def test_packet_router_strips_p4_timestamp_and_assigns_semantic_kind() -> None:
    router = P4PacketRouter({0x21: "example"}, split_p4_timestamp)
    payload = b"\x10\x20" + struct.pack(">I", 4321)

    packet = router.route(0x21, payload)

    # 未登记为带时间戳的数据包不会被协议层误剥离，路由器遵守 splitter 的判断。
    assert packet.kind == "example"
    assert packet.payload == payload
    assert packet.p4_time_ms is None


def test_ack_survives_unrelated_packets_in_one_rx_batch_and_is_cleared_on_reset():
    tracker = AckTracker()
    tracker.publish(0xE8, 0, 0, received_at=10.0)
    tracker.publish(0xED, 0, 0, received_at=10.001)
    tracker.publish(0xF3, 0, 0, received_at=10.002)
    assert tracker.wait_for(0xE8, timeout_s=0, received_after=9.9) == (0xE8,0,0,10.0)
    assert tracker.wait_for(0xE8, timeout_s=0, received_after=10.1) is None
    tracker.reset()
    assert tracker.wait_for(0xE8, timeout_s=0) is None


def test_ack_tracker_rejects_stale_ack_and_exposes_monotonic_sequence() -> None:
    tracker = AckTracker()
    first_sequence = tracker.publish(0x10, 2, 0, received_at=10.0, p4_time_ms=100)
    assert first_sequence == 1
    assert tracker.wait_for(0x10, timeout_s=0.0, received_after=11.0) is None

    second_sequence = tracker.publish(0x10, 3, 0, received_at=12.0, p4_time_ms=120)
    assert second_sequence == 2
    ack = tracker.wait_for(0x10, timeout_s=0.0, received_after=11.0)
    assert ack == (0x10, 3, 0, 12.0)
    events = tracker.drain_events()
    assert all(event.pop('received_monotonic') > 0 for event in events)
    assert events == [
        {
            "sequence": 1,
            "command": 0x10,
            "mode": 2,
            "status": 0,
            "received_at": 10.0,
            "p4_time_ms": 100,
        },
        {
            "sequence": 2,
            "command": 0x10,
            "mode": 3,
            "status": 0,
            "received_at": 12.0,
            "p4_time_ms": 120,
        },
    ]

    tracker.reset()
    assert tracker.drain_events() == []


def test_new_command_cannot_reuse_ack_with_same_wall_clock_timestamp() -> None:
    comm = LowerComputerComm("")
    comm._transport.attach(_FakeSerial("COM_TEST"))
    comm.running = True
    comm._protocol_compatible = True
    tracker = comm._ack_tracker
    tracker.publish(CMD_RESET, 0, 0, received_at=10.0)
    tracker.publish(0xED, 0, 0, received_at=10.0)

    assert comm.send_command("reset")
    assert comm.wait_for_ack(CMD_RESET, 0, received_after=10.0) is None
    # An unrelated transaction's result survives.
    assert tracker.wait_for(0xED, 0, received_after=10.0) == (0xED, 0, 0, 10.0)
    tracker.publish(CMD_RESET, 0, 0, received_at=10.0)
    assert comm.wait_for_ack(CMD_RESET, 0, received_after=10.0) == (CMD_RESET, 0, 0, 10.0)


def test_event_channel_preserves_order_until_explicit_drain() -> None:
    channel: EventChannel[dict] = EventChannel()
    channel.publish({"sequence": 1})
    channel.publish({"sequence": 2})

    assert channel.drain() == [{"sequence": 1}, {"sequence": 2}]
    assert channel.drain() == []


def test_state_reducer_carries_previous_snapshot_and_commits_once() -> None:
    reducer = StateReducer()
    previous = HandModel()
    previous.calib_status = "SUCCESS"
    reducer.last_model = previous

    routed = P4PacketRouter(
        {0x41: "value"},
        lambda packet_type, payload: (payload, 77),
    ).route(0x41, b"\x05")

    def _apply(model: HandModel, packet, _host_time: float) -> bool:
        model.proto_ack_status = int(packet.payload[0])
        return True

    finalized: list[HandModel] = []
    model, host_received_at = reducer.begin()
    model.p4_time_ms = int(routed.p4_time_ms or 0)
    model.has_p4_time = routed.p4_time_ms is not None
    emitted = _apply(model, routed, host_received_at)
    model = reducer.commit(model, emitted, finalized.append)

    assert model is not None
    assert model is reducer.last_model
    assert model.calib_status == "SUCCESS"
    assert model.p4_time_ms == 77
    assert model.proto_ack_status == 5
    assert finalized == [model]


def test_lower_computer_comm_uses_split_state_reducer() -> None:
    comm = LowerComputerComm("")
    model = HandModel()

    comm._state_reducer.last_model = model

    assert comm._state_reducer.last_model is model


def test_lower_computer_comm_rx_queue_is_latest_value_mailbox() -> None:
    comm = LowerComputerComm("")
    older = HandModel()
    older.host_rx_time = 1.0
    latest = HandModel()
    latest.host_rx_time = 2.0

    comm._push_rx_model(older)
    comm._push_rx_model(latest)

    assert comm.rx_queue.qsize() == 1
    assert comm.rx_queue.get_nowait() is latest


def test_hand_backend_connection_uses_comm_health_api_without_legacy_serial_property() -> None:
    """UI connection state must not depend on removed ``LowerComputerComm.serial``."""
    backend = HandBackend.__new__(HandBackend)
    backend._controller = SimpleNamespace(
        comm=SimpleNamespace(is_connected=lambda: True)
    )

    assert backend.is_comm_connected() is True


def _timestamped_packet(packet_type: int, payload: bytes, timestamp_ms: int) -> tuple:
    return packet_type, payload + struct.pack(">I", timestamp_ms)


def _encoder_packet(timestamp_ms: int, first: int, error_index: int = -1) -> tuple:
    values = [first + index for index in range(ENCODER_COUNT)]
    if error_index >= 0:
        values[error_index] = DISCONNECT_SENTINEL
    return _timestamped_packet(
        PACKET_TYPE_ENCODER_RAW, struct.pack(f">{ENCODER_COUNT}h", *values), timestamp_ms
    )


def _abs_packet(timestamp_ms: int, first: int) -> tuple:
    values = [first + index for index in range(MOTOR_COUNT)]
    payload = struct.pack(f">{MOTOR_COUNT}h", *values)
    payload += struct.pack(f">{MOTOR_COUNT}i", *values)
    return _timestamped_packet(PACKET_TYPE_SERVO_POSITION, payload, timestamp_ms)


def _connectivity_packet(
    timestamp_ms: int, *, servo_mask: int = (1 << MOTOR_COUNT) - 1,
    encoder_mask: int = (1 << ENCODER_COUNT) - 1, unknown_mask: int = 0,
    encoder_fault_mask: int = 0, can_online: bool = True,
) -> tuple:
    payload = bytearray(173)
    struct.pack_into(">I", payload, 13, unknown_mask)
    struct.pack_into(">I", payload, 26, servo_mask)
    struct.pack_into(">I", payload, 30, encoder_mask)
    payload[34] = int(can_online)
    struct.pack_into(">I", payload, 39, encoder_fault_mask)
    return _timestamped_packet(PACKET_TYPE_SYSTEM_STATUS, bytes(payload), timestamp_ms)


def test_feedback_history_preserves_every_packet_and_original_device_time_in_one_batch() -> None:
    comm = LowerComputerComm("")
    cursor = comm.feedback_history.mark()

    result = comm._process_packets([
        _connectivity_packet(120),
        _encoder_packet(0, 100),
        _abs_packet(3, -50),
        _encoder_packet(10, 130),
        _abs_packet(13, -40),
        _encoder_packet(20, 160),
        _abs_packet(23, -30),
    ])

    frames = comm.feedback_history.read(cursor)
    assert [(frame.kind, frame.p4_time_ms) for frame in frames] == [
        ("encoder_raw", 0), ("servo_abs", 3),
        ("encoder_raw", 10), ("servo_abs", 13),
        ("encoder_raw", 20), ("servo_abs", 23),
    ]
    assert [frame.values[0] for frame in frames] == [100, -50, 130, -40, 160, -30]
    assert frames[0].valid_mask == (1 << ENCODER_COUNT) - 1
    assert frames[1].valid_mask == (1 << MOTOR_COUNT) - 1
    assert isinstance(frames[0].values, tuple)
    assert frames[0].received_at > 0.0
    assert result.encoders[0].raw_sensor == 160
    assert result.servo_angles[0] == -30
    assert comm.feedback_history is comm._state_reducer.feedback_history


def test_feedback_history_uses_raw_error_online_fault_and_turn_unknown_masks() -> None:
    comm = LowerComputerComm("")
    cursor = comm.feedback_history.mark()

    comm._process_packets([
        _connectivity_packet(
            90, servo_mask=0b111, encoder_mask=0b111,
            unknown_mask=0b010, encoder_fault_mask=0b001,
        ),
        _encoder_packet(100, 700, error_index=2),
        _abs_packet(101, 50),
    ])

    raw, absolute = comm.feedback_history.read(cursor)
    assert raw.valid_mask == 0b010
    assert raw.values[2] == 0
    assert absolute.valid_mask == 0b101


def test_feedback_history_keeps_untrusted_frames_before_online_status() -> None:
    comm = LowerComputerComm("")
    cursor = comm.feedback_history.mark()

    comm._process_packets([_encoder_packet(100, 700, error_index=2), _abs_packet(101, 50)])

    raw, absolute = comm.feedback_history.read(cursor)
    assert raw.valid_mask == 0
    assert absolute.valid_mask == 0


def test_feedback_history_keeps_can_offline_raw_as_an_invalid_break() -> None:
    comm = LowerComputerComm("")
    cursor = comm.feedback_history.mark()

    comm._process_packets([
        _connectivity_packet(90, can_online=False), _encoder_packet(100, 700)
    ])

    frames = comm.feedback_history.read(cursor)
    assert len(frames) == 1
    assert frames[0].kind == "encoder_raw"
    assert frames[0].valid_mask == 0


def test_feedback_history_does_not_apply_later_status_to_earlier_frames() -> None:
    comm = LowerComputerComm("")
    cursor = comm.feedback_history.mark()

    comm._process_packets([
        _encoder_packet(100, 700), _abs_packet(101, 50),
        _connectivity_packet(102), _encoder_packet(110, 710), _abs_packet(111, 60),
    ])

    frames = comm.feedback_history.read(cursor)
    assert [frame.valid_mask for frame in frames] == [
        0, 0, (1 << ENCODER_COUNT) - 1, (1 << MOTOR_COUNT) - 1
    ]


def test_servo_telemetry_cannot_supply_abs_history() -> None:
    comm = LowerComputerComm("")
    cursor = comm.feedback_history.mark()
    payload = b"".join(
        struct.pack(">hhhBB", 1, 2, 3, 12, 29) for _ in range(MOTOR_COUNT)
    )

    result = comm._process_packets([
        _timestamped_packet(PACKET_TYPE_SERVO_TELEM, payload, 55)
    ])

    assert comm.feedback_history.read(cursor) == ()
    assert result.servo_temperature == [29] * MOTOR_COUNT


def test_feedback_history_never_substitutes_host_time_for_missing_p4_timestamp() -> None:
    comm = LowerComputerComm("")
    cursor = comm.feedback_history.mark()
    comm._packet_router._timestamp_splitter = lambda _kind, payload: (payload, None)

    raw_packet = _encoder_packet(100, 700)
    absolute_packet = _abs_packet(101, 50)
    comm._process_packets([
        (raw_packet[0], raw_packet[1][:-4]),
        (absolute_packet[0], absolute_packet[1][:-4]),
    ])

    assert comm.feedback_history.read(cursor) == ()


def test_successful_f0_result_breaks_history_before_new_logical_origin_samples() -> None:
    comm = LowerComputerComm("")
    comm._process_packets([_connectivity_packet(1), _encoder_packet(2, 100), _abs_packet(3, 900)])
    old_cursor = comm.feedback_history.mark()
    payload = struct.pack(">III", 42, 0b1, 0) + bytes(MOTOR_COUNT * 4)

    comm._process_packets([
        _timestamped_packet(PACKET_TYPE_SERVO_REBASE_ZERO_RESULT, payload, 10),
        _encoder_packet(11, 110), _abs_packet(12, 0),
    ])

    with pytest.raises(FeedbackHistoryError):
        comm.feedback_history.read(old_cursor)
    frames = comm.feedback_history.read(comm.feedback_history.mark())
    assert [(frame.kind, frame.p4_time_ms) for frame in frames] == [
        ("encoder_raw", 11), ("servo_abs", 12)
    ]
    assert frames[-1].values[0] == 0


def test_failed_f0_result_keeps_existing_coordinate_history() -> None:
    comm = LowerComputerComm("")
    cursor = comm.feedback_history.mark()
    payload = struct.pack(">III", 42, 0, 0b1) + bytes(MOTOR_COUNT * 4)

    comm._process_packets([
        _connectivity_packet(1), _abs_packet(3, 900),
        _timestamped_packet(PACKET_TYPE_SERVO_REBASE_ZERO_RESULT, payload, 10),
    ])

    assert len(comm.feedback_history.read(cursor)) == 1


def test_state_reset_clears_history_and_invalidates_old_cursor() -> None:
    comm = LowerComputerComm("")
    cursor = comm.feedback_history.mark()
    comm._process_packets([_connectivity_packet(1), _abs_packet(3, 900)])

    comm._state_reducer.reset()

    with pytest.raises(FeedbackHistoryError):
        comm.feedback_history.read(cursor)
    assert comm.feedback_history.read(comm.feedback_history.mark()) == ()
    assert comm._state_reducer._feedback_status_clock_ms is None


def test_reset_ack_clears_feedback_history() -> None:
    comm = LowerComputerComm("")
    cursor = comm.feedback_history.mark()
    comm._process_packets([_connectivity_packet(1), _abs_packet(3, 900)])

    comm._process_packets([_timestamped_packet(
        0x06, bytes((CMD_RESET, 0, PROTO_ACK_STATUS_OK)), 10
    )])

    with pytest.raises(FeedbackHistoryError):
        comm.feedback_history.read(cursor)
    assert comm.feedback_history.read(comm.feedback_history.mark()) == ()


def test_disconnect_clears_feedback_history() -> None:
    comm = LowerComputerComm("")
    cursor = comm.feedback_history.mark()
    comm._process_packets([_connectivity_packet(1), _abs_packet(3, 900)])

    comm.disconnect()

    with pytest.raises(FeedbackHistoryError):
        comm.feedback_history.read(cursor)
    assert comm.feedback_history.read(comm.feedback_history.mark()) == ()


def test_serial_offline_clears_feedback_history() -> None:
    comm = LowerComputerComm("")
    cursor = comm.feedback_history.mark()
    comm._process_packets([_connectivity_packet(1), _abs_packet(3, 900)])

    comm._mark_comm_offline("test")

    with pytest.raises(FeedbackHistoryError):
        comm.feedback_history.read(cursor)
    assert comm.feedback_history.read(comm.feedback_history.mark()) == ()


def test_short_uptime_status_clock_regression_resets_history_within_same_batch() -> None:
    comm = LowerComputerComm("")
    cursor = comm.feedback_history.mark()

    comm._process_packets([
        _connectivity_packet(25), _encoder_packet(26, 700), _abs_packet(27, 900),
        _connectivity_packet(10), _encoder_packet(11, 710), _abs_packet(12, 0),
    ])

    with pytest.raises(FeedbackHistoryError):
        comm.feedback_history.read(cursor)
    frames = comm.feedback_history.read(comm.feedback_history.mark())
    assert [(frame.kind, frame.p4_time_ms) for frame in frames] == [
        ("encoder_raw", 11), ("servo_abs", 12)
    ]
    assert comm._state_reducer._feedback_status_clock_ms == 10


def test_status_clock_uint32_wrap_preserves_feedback_epoch() -> None:
    comm = LowerComputerComm("")
    cursor = comm.feedback_history.mark()

    comm._process_packets([
        _connectivity_packet(0xFFFFFFF0),
        _encoder_packet(0xFFFFFFF1, 700), _abs_packet(0xFFFFFFF2, 900),
        _connectivity_packet(15), _encoder_packet(16, 710), _abs_packet(17, 910),
    ])

    frames = comm.feedback_history.read(cursor)
    assert [frame.p4_time_ms for frame in frames] == [0xFFFFFFF1, 0xFFFFFFF2, 16, 17]
    assert comm._state_reducer._feedback_status_clock_ms == 15


def test_out_of_order_raw_and_abs_timestamps_cannot_reset_status_clock_epoch() -> None:
    comm = LowerComputerComm("")
    cursor = comm.feedback_history.mark()

    comm._process_packets([
        _connectivity_packet(100), _encoder_packet(200, 700), _abs_packet(250, 900),
        _encoder_packet(150, 710), _abs_packet(175, 910), _connectivity_packet(101),
    ])

    frames = comm.feedback_history.read(cursor)
    assert [frame.p4_time_ms for frame in frames] == [200, 250, 150, 175]
    assert comm._state_reducer._feedback_status_clock_ms == 101


def test_replayed_historical_diagnostic_cannot_reset_feedback_epoch() -> None:
    comm = LowerComputerComm("")
    cursor = comm.feedback_history.mark()
    diagnostics = []
    for task_id in TASK_RUNTIME_NAMES:
        diagnostic = bytearray(191)
        diagnostic[0] = 4
        diagnostic[1] = task_id
        struct.pack_into(">I", diagnostic, 29, 7)
        struct.pack_into(">I", diagnostic, 91, 10)
        diagnostics.append((PACKET_TYPE_TASK_DIAG, bytes(diagnostic)))

    comm._process_packets([
        _connectivity_packet(1000), _encoder_packet(1001, 700), _abs_packet(1002, 900),
        *diagnostics,
    ])
    assert comm._state_reducer.last_model.p4_time_ms == 10
    comm._process_packets([
        _connectivity_packet(1003), _encoder_packet(1004, 710), _abs_packet(1005, 910),
    ])

    assert len(comm.feedback_history.read(cursor)) == 4
    assert comm._state_reducer._feedback_status_clock_ms == 1003


def test_malformed_status_packet_cannot_change_feedback_clock_or_epoch() -> None:
    comm = LowerComputerComm("")
    cursor = comm.feedback_history.mark()

    comm._process_packets([
        _connectivity_packet(100), _encoder_packet(101, 700), _abs_packet(102, 900),
        _timestamped_packet(PACKET_TYPE_SYSTEM_STATUS, b"\x00", 1),
        _connectivity_packet(103),
    ])

    assert len(comm.feedback_history.read(cursor)) == 2
    assert comm._state_reducer._feedback_status_clock_ms == 103


def test_repeated_f0_result_echo_keeps_active_history_and_updates_model_timestamp() -> None:
    comm = LowerComputerComm("")
    payload = struct.pack(">III", 42, 0b1, 0) + bytes(MOTOR_COUNT * 4)
    comm._process_packets([
        _connectivity_packet(1),
        _timestamped_packet(PACKET_TYPE_SERVO_REBASE_ZERO_RESULT, payload, 10),
        _encoder_packet(11, 700), _abs_packet(12, 0),
    ])
    cursor = comm.feedback_history.mark()

    comm._process_packets([
        _timestamped_packet(PACKET_TYPE_SERVO_REBASE_ZERO_RESULT, payload, 20),
        _encoder_packet(21, 710), _abs_packet(22, 10),
    ])

    assert len(comm.feedback_history.read(cursor)) == 4
    assert comm._state_reducer.last_model.servo_rebase_zero_result_p4_time_ms == 20


@pytest.mark.parametrize("token,success_mask,failed_mask", [
    (42, 0b11, 0),
    (42, 0b1, 0b10),
    (43, 0b1, 0),
])
def test_changed_f0_result_identity_breaks_history(
    token: int, success_mask: int, failed_mask: int,
) -> None:
    comm = LowerComputerComm("")
    payload = struct.pack(">III", 42, 0b1, 0) + bytes(MOTOR_COUNT * 4)
    comm._process_packets([
        _connectivity_packet(1),
        _timestamped_packet(PACKET_TYPE_SERVO_REBASE_ZERO_RESULT, payload, 10),
        _encoder_packet(11, 700), _abs_packet(12, 0),
    ])
    cursor = comm.feedback_history.mark()
    changed_payload = struct.pack(">III", token, success_mask, failed_mask) + bytes(MOTOR_COUNT * 4)

    comm._process_packets([
        _timestamped_packet(PACKET_TYPE_SERVO_REBASE_ZERO_RESULT, changed_payload, 20),
    ])

    with pytest.raises(FeedbackHistoryError):
        comm.feedback_history.read(cursor)
    assert comm.feedback_history.read(comm.feedback_history.mark()) == ()


def test_f0_nonzero_reserved_area_cannot_change_result_or_coordinate_history() -> None:
    comm = LowerComputerComm("")
    payload = struct.pack(">III", 42, 0b1, 0) + bytes(MOTOR_COUNT * 4)
    comm._process_packets([
        _connectivity_packet(1),
        _timestamped_packet(PACKET_TYPE_SERVO_REBASE_ZERO_RESULT, payload, 10),
        _encoder_packet(11, 700), _abs_packet(12, 0),
    ])
    cursor = comm.feedback_history.mark()
    invalid_payload = bytearray(payload)
    struct.pack_into(">I", invalid_payload, 0, 43)
    invalid_payload[12] = 1

    comm._process_packets([
        _timestamped_packet(PACKET_TYPE_SERVO_REBASE_ZERO_RESULT, invalid_payload, 20),
        _encoder_packet(21, 710), _abs_packet(22, 10),
    ])

    assert len(comm.feedback_history.read(cursor)) == 4
    assert comm._state_reducer.last_model.servo_rebase_zero_result_token == 42
    assert comm._state_reducer.last_model.servo_rebase_zero_result_p4_time_ms == 10


def test_partial_serial_frame_is_a_transport_failure():
    transport = SerialTransport("COM_TEST", 921600, 0.02, _FakeSerial)
    transport.open()
    transport.serial.write = lambda data: len(data) - 1
    with pytest.raises(OSError, match="serial short write: 2/3 bytes"):
        transport.write(b"abc")


def test_adaptive_kp_packet_parser():
    from desktop.core.protocol import parse_adaptive_kp_packet
    payload = bytes([1])
    payload += b"".join(struct.pack(">h", 1000 + i) for i in range(ENCODER_COUNT))
    payload += b"".join(struct.pack(">h", 500 + i) for i in range(ENCODER_COUNT))
    parsed = parse_adaptive_kp_packet(payload)
    assert parsed is not None
    assert parsed["enabled"] is True
    assert parsed["scale"] == [1.0 + i / 1000.0 for i in range(ENCODER_COUNT)]
    assert parsed["effective"][0] == 0.5
    assert parse_adaptive_kp_packet(payload[:-1]) is None
