"""验证 P4 同周期控制跟踪包的协议、时间戳剥离和状态归并。"""

import struct

import desktop.core.communication.state_reducer as state_reducer_module

from desktop.core.communication.ack_tracker import AckTracker
from desktop.core.communication.packet_router import P4PacketRouter
from desktop.core.communication.state_reducer import StateReducer
from desktop.core.generated_upper_protocol import (
    CMD_DIAGNOSTICS_SUBSCRIBE,
    PACKET_TYPE_CONTROL_TRACE,
    PACKET_TYPE_CONTROL_COMPONENTS,
    PACKET_TYPE_SERVO_POSITION,
)
from desktop.core.protocol import (
    build_diagnostics_subscribe_cmd,
    parse_control_trace_packet,
)


def _business_payload() -> bytes:
    targets = [100 + index for index in range(21)]
    actual = [50 + index for index in range(21)]
    motor_commands = [10000 + index * 100 for index in range(27)]
    return (
        struct.pack(
            ">IIIIIII",
            7,
            11,
            900,
            995,
            0x00008000,
            0x00008000,
            1 << 19,
        )
        + struct.pack(">21h", *targets)
        + struct.pack(">21h", *actual)
        + struct.pack(">27i", *motor_commands)
    )


def test_control_trace_subscription_has_explicit_on_off_payload() -> None:
    assert build_diagnostics_subscribe_cmd(2) == bytes(
        [0xFE, 0x04, CMD_DIAGNOSTICS_SUBSCRIBE, 0x02, 0x05, 0xFF]
    )
    assert build_diagnostics_subscribe_cmd(0) == bytes(
        [0xFE, 0x04, CMD_DIAGNOSTICS_SUBSCRIBE, 0x00, 0x05, 0xFF]
    )


def test_control_trace_parser_uses_fixed_payload_and_centi_degrees() -> None:
    parsed = parse_control_trace_packet(_business_payload())
    assert parsed is not None
    assert parsed["sequence"] == 7
    assert parsed["target_accepted_ms"] == 900
    assert parsed["encoder_timestamp_ms"] == 995
    assert parsed["target_deg"][15] == 1.15
    assert parsed["actual_deg"][15] == 0.65
    assert parsed["motor_command_mask"] == 1 << 19
    assert parsed["motor_command_abs"][19] == 11900
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
    assert model.control_trace_motor_command_mask == 1 << 19
    assert model.control_trace_motor_command_abs[19] == 11900


def test_servo_abs_reducer_keeps_its_own_p4_timestamp() -> None:
    """后到的其他状态包不能篡改舵机 ABS 的真实采样时间。"""

    class Owner:
        _heartbeat_enabled = False

        @staticmethod
        def _parse_protocol_payload(name: str, _payload: bytes):
            assert name == "parse_servo_position_packet"
            return [0] * 27, [1000 + index for index in range(27)]

        @staticmethod
        def _apply_connectivity_visibility(_model) -> None:
            pass

    router = P4PacketRouter(
        {PACKET_TYPE_SERVO_POSITION: "servo_position"},
        lambda _packet_type, payload: (payload[:-4], struct.unpack(">I", payload[-4:])[0]),
    )
    reducer = StateReducer(router, AckTracker())
    model = reducer.process_packets(
        Owner(),
        [
            (PACKET_TYPE_SERVO_POSITION, b"servo" + struct.pack(">I", 1000)),
            (0xEE, b"other" + struct.pack(">I", 1015)),
        ],
    )
    assert model is not None
    assert model.p4_time_ms == 1015
    assert model.servo_feedback_p4_time_ms == 1000
    assert model.servo_angles[19] == 1019


def test_control_parts_match_both_timestamp_and_cycle_and_reset_on_disconnect(
    monkeypatch,
):
    # 连续批次都要拿到快照：关闭发布节流（默认 20 ms 只约束高频遥测）。
    monkeypatch.setattr(state_reducer_module, "STATE_SNAPSHOT_MIN_INTERVAL_S", 0.0)
    from desktop.core.protocol import parse_control_components_packet
    class Owner:
        _heartbeat_enabled = False
        _parse_protocol_payload = staticmethod(lambda name, payload: parse_control_trace_packet(payload))
        _apply_connectivity_visibility = staticmethod(lambda model: None)
    router = P4PacketRouter({PACKET_TYPE_CONTROL_TRACE: 'control_trace',
                            PACKET_TYPE_CONTROL_COMPONENTS: 'control_components'},
        lambda kind, payload: (payload[:-4], struct.unpack('>I', payload[-4:])[0]))
    reducer = StateReducer(router, AckTracker())
    part = bytes([1]) + struct.pack('>IB',7,5) + struct.pack('>27h',*range(27))*3 + struct.pack('>21h',*range(21))
    assert parse_control_components_packet(part)['integral_counts'][3] == 3
    assert parse_control_components_packet(part[:-1]) is None
    stamped = lambda body, stamp: body + struct.pack('>I',stamp)
    model = reducer.process_packets(Owner(), [(PACKET_TYPE_CONTROL_COMPONENTS, stamped(part,1000)),
        (PACKET_TYPE_CONTROL_TRACE, stamped(_business_payload(),1000))])
    assert model.control_trace_components['divisor'] == 5
    model = reducer.process_packets(Owner(), [(PACKET_TYPE_CONTROL_COMPONENTS, stamped(part,1000)),
        (PACKET_TYPE_CONTROL_TRACE, stamped(_business_payload(),1010))])
    assert model.control_trace_components == {}
    reducer.reset()
    model = reducer.process_packets(Owner(), [(PACKET_TYPE_CONTROL_TRACE, stamped(_business_payload(),1000))])
    assert model.control_trace_components == {}
