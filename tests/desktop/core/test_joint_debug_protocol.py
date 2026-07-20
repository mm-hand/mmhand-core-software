import struct

from desktop.core.protocol import parse_joint_debug_packet


def test_parse_joint_debug_packet_new_format() -> None:
    payload = struct.pack(
        ">BBfffBh",
        2,
        1,
        12.5,
        11.75,
        42.0,
        1,
        2048,
    )

    parsed = parse_joint_debug_packet(payload)

    assert parsed is not None
    assert parsed[0] == 2
    assert parsed[1] is True
    assert parsed[5] is True
    assert parsed[6] == 2048
    assert parsed[7] == 0


def test_parse_joint_debug_packet_new_format_with_timestamp() -> None:
    payload = (
        struct.pack(
            ">BBfffBh",
            3,
            1,
            20.0,
            19.5,
            -15.0,
            1,
            -1200,
        )
        + struct.pack(">I", 123456)
    )

    parsed = parse_joint_debug_packet(payload)

    assert parsed is not None
    assert parsed[0] == 3
    assert parsed[7] == 123456


def test_parse_joint_debug_packet_rejects_old_loop2_format() -> None:
    old_payload = struct.pack(
        ">BBfffffBh",
        2,
        1,
        12.5,
        11.75,
        42.0,
        2000.0,
        2010.0,
        1,
        2048,
    )

    assert parse_joint_debug_packet(old_payload) is None
