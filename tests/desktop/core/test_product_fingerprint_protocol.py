"""P4 产品指纹上行协议测试。"""

import struct

from desktop.core.protocol import parse_product_fingerprint_packet


def test_parse_product_fingerprint_packet() -> None:
    payload = struct.pack(
        ">BBBBIIIIII",
        1,
        2,
        3,
        0,
        0x11111111,
        0x22222222,
        0x33333333,
        0x44444444,
        0x55555555,
        0x66666666,
    )
    parsed = parse_product_fingerprint_packet(payload)
    assert parsed == {
        "schema_version": 1,
        "feedback_mapping_version": 2,
        "manual_scaled_mapping_version": 3,
        "control_model_crc": 0x11111111,
        "visual_model_crc": 0x22222222,
        "hardware_map_crc": 0x33333333,
        "parameter_crc": 0x44444444,
        "firmware_source_crc": 0x55555555,
        "structure_commit_crc": 0x66666666,
    }


def test_reject_truncated_product_fingerprint_packet() -> None:
    assert parse_product_fingerprint_packet(bytes(27)) is None
