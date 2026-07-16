import zlib

from desktop.core.protocol import manual_encoder_reference_entry_crc


def test_manual_entry_crc_matches_p4_ieee_crc32_layout():
    joint = 8
    min_raw = 15300
    max_raw = 1200
    payload = bytes((joint, min_raw >> 8, min_raw & 0xFF, max_raw >> 8, max_raw & 0xFF))

    assert manual_encoder_reference_entry_crc(joint, min_raw, max_raw) == (
        zlib.crc32(payload) & 0xFFFFFFFF
    )


def test_manual_entry_crc_changes_only_with_bound_joint_data():
    baseline = manual_encoder_reference_entry_crc(4, 1000, 5000)

    assert manual_encoder_reference_entry_crc(4, 1000, 5001) != baseline
    assert manual_encoder_reference_entry_crc(5, 1000, 5000) != baseline
