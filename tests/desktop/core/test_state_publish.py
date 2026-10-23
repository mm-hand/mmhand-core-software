"""State snapshot publish throttling: rate cap, critical-packet bypass, latest-value.

These tests drive the real LowerComputerComm reducer path with synthetic protocol
packets and a fake wall/monotonic clock, so they never touch serial hardware.
"""

import struct
from types import SimpleNamespace

import pytest

import desktop.core.communication.state_reducer as state_reducer_module
from desktop.core.comm_layer import LowerComputerComm
from desktop.core.protocol import (
    PACKET_TYPE_ENCODER_RAW,
    PACKET_TYPE_PROTO_ACK,
    PACKET_TYPE_DIRECT_TARGET_RESULT,
    PACKET_TYPE_SERVO_POSITION,
    PACKET_TYPE_SYSTEM_STATUS,
)


class _FakeClock:
    def __init__(self) -> None:
        self.wall = 1_700_000_000.0
        self.mono = 1000.0

    def time(self) -> float:
        return self.wall

    def monotonic(self) -> float:
        return self.mono


@pytest.fixture
def clock(monkeypatch):
    """Replace state_reducer's clock only; the rest of the process keeps real time."""
    fake = _FakeClock()
    monkeypatch.setattr(
        state_reducer_module,
        "time",
        SimpleNamespace(time=fake.time, monotonic=fake.monotonic),
    )
    return fake


def _comm() -> LowerComputerComm:
    return LowerComputerComm("")


def _abs_packet(first: int, timestamp_ms: int) -> tuple:
    values = [first] * 27
    return (
        PACKET_TYPE_SERVO_POSITION,
        struct.pack(">27h", *values) + struct.pack(">27i", *values) + struct.pack(">I", timestamp_ms),
    )


def _encoder_packet(first: int, timestamp_ms: int) -> tuple:
    values = [first] * 21
    return (
        PACKET_TYPE_ENCODER_RAW,
        struct.pack(">21h", *values) + struct.pack(">I", timestamp_ms),
    )


def _ack_packet(timestamp_ms: int = 1) -> tuple:
    return (PACKET_TYPE_PROTO_ACK, bytes((0xCC, 0, 0)) + struct.pack(">I", timestamp_ms))


def _direct_result_packet(timestamp_ms: int = 1) -> tuple:
    return (
        PACKET_TYPE_DIRECT_TARGET_RESULT,
        struct.pack(">IIIIBI", 12, 3, 3, 0, 2, timestamp_ms),
    )


def _system_status_packet(timestamp_ms: int = 1) -> tuple:
    payload = bytearray(173)
    struct.pack_into(">I", payload, 26, (1 << 27) - 1)  # servo online mask
    struct.pack_into(">I", payload, 30, (1 << 21) - 1)  # encoder online mask
    payload[34] = 1  # CAN link online
    return (
        PACKET_TYPE_SYSTEM_STATUS,
        bytes(payload) + struct.pack(">I", timestamp_ms),
    )


# ---------------------------------------------------------------------------
# 发布频率上限（默认 20 ms）
# ---------------------------------------------------------------------------


def test_telemetry_batches_publish_at_most_once_per_interval(clock):
    comm = _comm()
    publishes = 0
    for index in range(10):
        clock.mono += 0.001
        if comm._process_packets([_abs_packet(index, index)]) is not None:
            publishes += 1

    assert publishes == 1

    # 到期后的第一批遥测必须发布，且快照包含期间吸收的全部最新值。
    clock.mono += state_reducer_module.STATE_SNAPSHOT_MIN_INTERVAL_S
    published = comm._process_packets([_abs_packet(99, 99)])
    assert published is not None
    assert published.servo_angles[0] == 99
    assert published.servo_feedback_p4_time_ms == 99


def test_high_rate_batches_publish_at_no_more_than_interval_rate(clock):
    """约 1000 批/s 的合成遥测在 100 ms 内最多发布 1/20ms = 5 次。"""
    comm = _comm()
    publishes = 0
    for index in range(100):
        clock.mono += 0.0001
        if comm._process_packets([_abs_packet(index, index)]) is not None:
            publishes += 1

    assert publishes <= 1 + int(0.100 / state_reducer_module.STATE_SNAPSHOT_MIN_INTERVAL_S)
    assert publishes >= 1


def test_zero_interval_disables_throttle(clock, monkeypatch):
    monkeypatch.setattr(state_reducer_module, "STATE_SNAPSHOT_MIN_INTERVAL_S", 0.0)
    comm = _comm()
    for index in range(5):
        clock.mono += 0.0001
        assert comm._process_packets([_abs_packet(index, index)]) is not None


# ---------------------------------------------------------------------------
# 关键包必须立即发布（控制/标定等待 ACK 不能被节流）
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "kind",
    [
        "protocol_ack",
        "system_status",
        "direct_target_result",
        "joint_calibration_snapshot",
        "calibration_diagnostic",
        "manual_reference",
        "servo_rebase_zero_result",
        "multiturn_restore_result",
        "safety_event",
        "fault_status",
    ],
)
def test_critical_packet_kinds_are_whitelisted(kind):
    assert kind in state_reducer_module.STATE_SNAPSHOT_CRITICAL_PACKET_KINDS


@pytest.mark.parametrize(
    "critical_packet",
    [_ack_packet(), _system_status_packet(), _direct_result_packet()],
    ids=["ack", "system_status", "direct_target_result"],
)
def test_critical_packet_publishes_immediately_inside_interval(clock, critical_packet):
    comm = _comm()
    assert comm._process_packets([_abs_packet(1, 1)]) is not None  # 首个发布

    for index in range(5):
        clock.mono += 0.001
        assert comm._process_packets([_abs_packet(index + 2, index + 2)]) is None

    clock.mono += 0.001  # 仍在 20 ms 节流窗口内
    published = comm._process_packets([critical_packet])
    assert published is not None
    assert published is comm._state_reducer.last_model


def test_throttled_telemetry_before_ack_is_included_in_ack_snapshot(clock):
    comm = _comm()
    assert comm._process_packets([_abs_packet(1, 1)]) is not None

    clock.mono += 0.001
    assert comm._process_packets([_abs_packet(1234, 2)]) is None

    clock.mono += 0.001
    ack = comm._process_packets([_ack_packet(3)])
    assert ack is not None
    # 节流期间的工作模型更新不能丢，ACK 快照必须携带最新遥测。
    assert ack.servo_angles[0] == 1234
    assert ack.proto_ack_cmd == 0xCC


# ---------------------------------------------------------------------------
# latest-value 邮箱不积压 + 只读快照隔离
# ---------------------------------------------------------------------------


def test_published_snapshots_do_not_backlog_in_latest_value_mailbox(clock):
    comm = _comm()
    for index in range(4):
        clock.mono += state_reducer_module.STATE_SNAPSHOT_MIN_INTERVAL_S + 0.001
        model = comm._process_packets([_abs_packet(index, index)])
        if model is not None:
            comm._push_rx_model(model)

    assert comm.rx_queue.qsize() == 1
    latest = comm.rx_queue.get_nowait()
    assert latest is comm._state_reducer.last_model
    assert latest.servo_angles[0] == 3


def test_working_model_mutation_does_not_leak_into_published_snapshot(clock):
    comm = _comm()
    published = comm._process_packets([_abs_packet(11, 1000)])
    assert published is not None
    assert comm._state_reducer.last_model is published

    # 节流窗口内的遥测只更新常驻工作模型。
    clock.mono += 0.001
    assert comm._process_packets([_abs_packet(22, 1001)]) is None
    working = comm._state_reducer._working
    assert working is not None and working is not published
    assert working.servo_angles[0] == 22
    assert published.servo_angles[0] == 11
    # 已发布快照的容器不与工作模型共享，外部读者看不到后续批次。
    assert published.servo_angles is not working.servo_angles
    assert comm._state_reducer.last_model is published

    # 下一次发布把工作模型的最新状态物化到新快照，旧快照保持不变。
    clock.mono += state_reducer_module.STATE_SNAPSHOT_MIN_INTERVAL_S
    newer = comm._process_packets([_abs_packet(33, 1002)])
    assert newer is not None and newer.servo_angles[0] == 33
    assert published.servo_angles[0] == 11
    assert newer is not published


def test_begin_returns_the_resident_working_model_without_copy(clock):
    comm = _comm()
    reducer = comm._state_reducer
    first, _ = reducer.begin()
    second, _ = reducer.begin()
    assert first is second is reducer._working
    # begin 不再继承出一份新对象，也不能是已发布快照。
    assert reducer._last_model is None


# ---------------------------------------------------------------------------
# 断线模型：以最近已发布只读快照为基线，不反向污染该快照
# ---------------------------------------------------------------------------


def test_offline_model_is_built_from_published_snapshot_without_mutating_it(clock):
    comm = _comm()
    published = comm._process_packets([_abs_packet(7, 7)])
    assert published is not None
    assert published.calib_status == "IDLE"

    offline = comm._build_offline_model()
    assert offline.calib_status == "DISCONNECTED"
    assert offline.has_servo_angle_data is False
    # 断线模型是独立对象：已发布快照与离线模型互不影响。
    assert offline is not published
    assert published.calib_status == "IDLE"
    assert published.has_servo_angle_data is True
    assert published.servo_angles[0] == 7
    assert comm._state_reducer.last_model is offline

    # 下一次 begin 以断线快照为基线重建常驻工作模型，而不是别名复用。
    working, _ = comm._state_reducer.begin()
    assert working is not offline
    assert working.servo_angles[0] == 7
    assert working.calib_status == "DISCONNECTED"
    assert working is comm._state_reducer._working


# ---------------------------------------------------------------------------
# feedback_history 逐包记录，不因发布节流丢帧
# ---------------------------------------------------------------------------


def test_feedback_history_keeps_every_frame_while_snapshots_are_throttled(clock):
    comm = _comm()
    cursor = comm.feedback_history.mark()
    publishes = 0
    for index in range(20):
        clock.mono += 0.001
        model = comm._process_packets(
            [_abs_packet(index, index), _encoder_packet(index, index)]
        )
        if model is not None:
            publishes += 1

    frames = comm.feedback_history.read(cursor)
    assert len(frames) == 40
    assert [frame.kind for frame in frames[:2]] == ["servo_abs", "encoder_raw"]
    assert [frame.p4_time_ms for frame in frames[:2]] == [0, 0]
    # 发布被节流但逐包历史完整。
    assert publishes == 1
