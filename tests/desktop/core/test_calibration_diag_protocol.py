import struct

from desktop.core.protocol import parse_calib_diag_packet


def test_calibration_diag_packet_carries_exact_reason_and_result_masks():
    payload = bytes([1, 29, 6, 12, 16])
    payload += struct.pack(">iiiIIIII", 3400, 6668, 3242, 0x00030000,
                           123456, 7, 0x00000003, 0x00001000)

    parsed = parse_calib_diag_packet(payload)

    assert parsed is not None
    assert parsed["reason"] == 29
    assert parsed["phase"] == 6
    assert parsed["joint"] == 12
    assert parsed["motor"] == 16
    assert parsed["valueA"] == 3400
    assert parsed["sequence"] == 7
    assert parsed["completed_joint_mask"] == 0x00000003
    assert parsed["failed_joint_mask"] == 0x00001000


def test_calibration_diag_rejects_obsolete_short_packet():
    assert parse_calib_diag_packet(bytes(25)) is None
