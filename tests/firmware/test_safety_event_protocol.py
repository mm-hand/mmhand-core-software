"""验证结构化安全事件在 P4 时间戳路由和业务解析之间保持同一契约。"""

import struct

from desktop.core.communication.packet_router import P4PacketRouter
from desktop.core.generated_upper_protocol import PACKET_TYPE_SAFETY_EVENT
from desktop.core.protocol import parse_safety_event_packet, split_p4_timestamp


def test_safety_event_packet_round_trip_contract() -> None:
    business_payload = struct.pack(
        ">IHBBBBiiI",
        17,       # sequence
        2005,     # rule_id
        3,        # motor_channel
        4,        # joint_index
        5,        # system_state
        2,        # owner
        1234,     # requested_value
        -88,      # observed_value
        2,        # overflow_count
    )
    router = P4PacketRouter(
        {PACKET_TYPE_SAFETY_EVENT: "safety_event"},
        split_p4_timestamp,
    )

    routed = router.route(
        PACKET_TYPE_SAFETY_EVENT,
        business_payload + struct.pack(">I", 999),
    )
    event = parse_safety_event_packet(routed.payload)

    assert routed.kind == "safety_event"
    assert routed.p4_time_ms == 999
    assert event == {
        "sequence": 17,
        "rule_id": 2005,
        "motor_channel": 3,
        "joint_index": 4,
        "system_state": 5,
        "owner": 2,
        "requested_value": 1234,
        "observed_value": -88,
        "overflow_count": 2,
    }


def test_safety_event_parser_rejects_wrong_length() -> None:
    assert parse_safety_event_packet(b"\x00" * 21) is None
