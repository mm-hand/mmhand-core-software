"""验证 P4 同周期控制跟踪包的协议、时间戳剥离和状态归并。"""

import struct

from desktop.core.communication.ack_tracker import AckTracker
from desktop.core.communication.packet_router import P4PacketRouter
from desktop.core.communication.state_reducer import StateReducer
from desktop.core.generated_upper_protocol import (
    CMD_CONTROL_TRACE_SUBSCRIBE,
    PACKET_TYPE_CONTROL_TRACE,
)
from desktop.core.protocol import (
    build_control_trace_subscribe_cmd,
    parse_control_trace_packet,
)


def _business_payload() -> bytes:
    targets = [100 + index for index in range(21)]
    actual = [50 + index for index in range(21)]
    return (
        struct.pack(">IIIIII", 7, 11, 900, 995, 0x00008000, 0x00008000)
        + struct.pack(">21h", *targets)
        + struct.pack(">21h", *actual)
    )


def test_control_trace_subscription_has_explicit_on_off_payload() -> None:
    assert build_control_trace_subscribe_cmd(True) == bytes(
        [0xFE, 0x03, CMD_CONTROL_TRACE_SUBSCRIBE, 0x01, 0xFF]
    )
    assert build_control_trace_subscribe_cmd(False) == bytes(
        [0xFE, 0x03, CMD_CONTROL_TRACE_SUBSCRIBE, 0x00, 0xFF]
    )


def test_control_trace_parser_uses_fixed_payload_and_centi_degrees() -> None:
    parsed = parse_control_trace_packet(_business_payload())
    assert parsed is not None
    assert parsed["sequence"] == 7
    assert parsed["target_accepted_ms"] == 900
    assert parsed["encoder_timestamp_ms"] == 995
    assert parsed["target_deg"][15] == 1.15
    assert parsed["actual_deg"][15] == 0.65
    assert parse_control_trace_packet(_business_payload()[:-1]) is None


def test_control_trace_reducer_keeps_dedicated_p4_timestamps() -> None:
    class Owner:
        _heartbeat_enabled = False

        @staticmethod
        def _parse_protocol_payload(_name: str, payload: bytes):
            return parse_control_trace_packet(payload)

        @staticmethod
        def _apply_connectivity_visibility(_model) -> None:
            pass

    router = P4PacketRouter(
        {PACKET_TYPE_CONTROL_TRACE: "control_trace"},
        lambda _packet_type, payload: (payload[:-4], struct.unpack(">I", payload[-4:])[0]),
    )
    reducer = StateReducer(router, AckTracker())
    model = reducer.process_packets(
        Owner(),
        [(PACKET_TYPE_CONTROL_TRACE, _business_payload() + struct.pack(">I", 1000))],
    )
    assert model is not None
    assert model.control_trace_p4_time_ms == 1000
    assert model.control_trace_encoder_timestamp_ms == 995
    assert model.control_trace_target_accepted_ms == 900
    assert model.control_trace_actual_deg[15] == 0.65
