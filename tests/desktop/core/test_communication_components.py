"""P1 通信组件拆分后的职责边界与兼容回归测试。

职责：验证传输、解帧、路由、状态归并和 ACK 跟踪可以脱离 UI 独立工作。
线程：pytest 主线程执行；ACK 用例以同步发布模拟 RX 线程结果。
输入：假串口、分片协议帧、路由包和最小 HandModel 更新函数。
输出：组件行为断言及 LowerComputerComm 兼容属性断言。
状态所有权：每个测试独占组件实例，不共享串口或模型状态。
禁止依赖：禁止启动真实串口、Qt 事件循环、ROS2 或 P4 硬件。
"""

from __future__ import annotations

import struct

from desktop.core.comm_layer import LowerComputerComm
from desktop.core.communication import (
    AckTracker,
    FrameDecoder,
    P4PacketRouter,
    SerialTransport,
    StateReducer,
)
from desktop.core.data_models import HandModel
from desktop.core.protocol import PROTOCOL_HEADER, PROTOCOL_TAIL, split_p4_timestamp


class _FakeSerial:
    """同时模拟 P4 构造式打开和触觉先配置后 open 的 pyserial 对象。"""

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
        self.is_open = True

    def close(self) -> None:
        self.is_open = False

    def setDTR(self, value: bool) -> None:
        self.dtr = bool(value)

    def setRTS(self, value: bool) -> None:
        self.rts = bool(value)

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


def test_tactile_transport_suppresses_dtr_and_rts() -> None:
    transport = SerialTransport(
        "COM_TOUCH",
        921_600,
        0.02,
        _FakeSerial,
        suppress_control_lines=True,
    )

    assert transport.open() is True
    assert transport.serial.dtr is False
    assert transport.serial.rts is False


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


def test_ack_tracker_rejects_stale_ack_and_exposes_monotonic_sequence() -> None:
    tracker = AckTracker()
    assert tracker.publish(0x10, 2, 0, received_at=10.0, p4_time_ms=100) == 1
    assert tracker.wait_for(0x10, timeout_s=0.0, received_after=11.0) is None

    assert tracker.publish(0x10, 3, 0, received_at=12.0, p4_time_ms=120) == 2
    ack = tracker.wait_for(0x10, timeout_s=0.0, received_after=11.0)
    assert ack == (0x10, 3, 0, 12.0)
    assert tracker.last_p4_time_ms == 120
    assert tracker.sequence == 2

    tracker.reset()
    assert tracker.sequence == 2
    assert tracker.last_ack is None


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
    model = reducer.reduce([routed], _apply, finalized.append)

    assert model is not None
    assert model is reducer.last_model
    assert model.calib_status == "SUCCESS"
    assert model.p4_time_ms == 77
    assert model.proto_ack_status == 5
    assert finalized == [model]


def test_lower_computer_comm_remains_a_compatible_facade() -> None:
    comm = LowerComputerComm("")
    model = HandModel()

    comm._last_model = model

    assert comm._last_model is model
    assert comm._state_reducer.last_model is model
    assert comm._ack_lock is comm._ack_tracker.condition
