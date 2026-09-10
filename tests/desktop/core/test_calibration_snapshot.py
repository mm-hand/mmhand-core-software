import json

import pytest

from desktop.core.calibration_snapshot import (
    _empty_v6_snapshot,
    CalibrationSnapshotError,
    atomic_write_snapshot,
    invalidate_calibration_snapshot_scope,
    merge_snapshot,
    parse_mask,
    read_snapshot,
    seal_snapshot,
    update_manual_snapshot,
    update_snapshot_multiturn_anchors,
    validate_snapshot_for_load,
)
from desktop.core.protocol import ENCODER_COUNT, MOTOR_COUNT


def _result(joint: int, motor_a: int, motor_b: int = 0xFF) -> dict:
    return {
        "joint": joint,
        "success": True,
        "mappingVersion": 4,
        "urdfConfigCrc": 0x12345678,
        "manualReferenceCrc": joint + 100,
        "tendonLutValid": True,
        "tendonLutCount": 3,
        "tendonLutMotor": [motor_a, motor_b],
        "tendonLutMotorAbs": [[1, 2, 3], [4, 5, 6]],
    }


def _current_payload(joint: int, result: dict) -> dict:
    results = [None] * ENCODER_COUNT
    results[joint] = result
    valid_motor_mask = 0
    for motor in result["tendonLutMotor"]:
        if 0 <= motor < MOTOR_COUNT:
            valid_motor_mask |= 1 << motor
    return {
        "hand_profile": "v2",
        "feedback_mapping_version": 3,
        "manual_scaled_mapping_version": 4,
        "urdf_config_crc": "0x12345678",
        "manual_reference_crc": "0x11111111",
        "active_joint_mask": 1 << joint,
        "active_motor_mask": valid_motor_mask,
        "motor_abs_limit_valid_bitmap": valid_motor_mask,
        "motor_abs_limit_min": [1000 + idx for idx in range(MOTOR_COUNT)],
        "motor_abs_limit_max": [2000 + idx for idx in range(MOTOR_COUNT)],
        "manual_reference_valid_bitmap": (1 << ENCODER_COUNT) - 1,
        "manual_reference_min_raw": [100] * ENCODER_COUNT,
        "manual_reference_max_raw": [5000] * ENCODER_COUNT,
        "joint_calib_results": results,
    }


def test_empty_auto_snapshot_is_not_loadable():
    assert validate_snapshot_for_load(seal_snapshot(_empty_v6_snapshot())) == (
        "snapshot contains no automatic calibration result"
    )


def test_partial_recalibration_preserves_unaffected_joint_and_limits():
    previous = merge_snapshot(
        None,
        _current_payload(0, _result(0, 0, 1)),
        updated_joint_mask=1 << 0,
    )
    old_j0 = json.loads(json.dumps(previous["joint_calib_results"][0]))
    old_m0_limits = (
        previous["motor_abs_limit_min"][0],
        previous["motor_abs_limit_max"][0],
    )

    current = _current_payload(15, _result(15, 19))
    current["motor_abs_limit_min"][19] = 7777
    current["motor_abs_limit_max"][19] = 8888
    merged = merge_snapshot(
        previous,
        current,
        updated_joint_mask=1 << 15,
    )

    assert merged["joint_calib_results"][0] == old_j0
    assert merged["joint_calib_results"][15]["joint"] == 15
    assert (
        merged["motor_abs_limit_min"][0],
        merged["motor_abs_limit_max"][0],
    ) == old_m0_limits
    assert merged["motor_abs_limit_min"][19] == 7777
    assert merged["motor_abs_limit_max"][19] == 8888
    assert int(merged["active_joint_mask"], 0) == (1 << 0) | (1 << 15)


def test_atomic_snapshot_round_trip_and_integrity(tmp_path):
    payload = merge_snapshot(
        None,
        _current_payload(4, _result(4, 5, 6)),
        updated_joint_mask=1 << 4,
    )
    path = tmp_path / "hand.mmcal.json"
    atomic_write_snapshot(path, payload)
    loaded = read_snapshot(path)
    assert loaded["joint_calib_results"][4]["joint"] == 4

    damaged = json.loads(path.read_text(encoding="utf-8"))
    damaged["motor_abs_limit_min"][5] += 1
    path.write_text(json.dumps(damaged), encoding="utf-8")
    with pytest.raises(CalibrationSnapshotError, match="SHA-256"):
        read_snapshot(path)


def test_multiturn_anchor_update_preserves_calibrated_joint_snapshot(tmp_path):
    path = tmp_path / "hand.mmcal.json"
    motor_idx = 18
    bit = 1 << motor_idx
    payload = merge_snapshot(
        None,
        _current_payload(14, _result(14, motor_idx)),
        updated_joint_mask=1 << 14,
    )
    atomic_write_snapshot(path, payload)

    logical_abs = [0] * MOTOR_COUNT
    raw = [0] * MOTOR_COUNT
    logical_abs[motor_idx] = 8192
    raw[motor_idx] = 17
    updated = update_snapshot_multiturn_anchors(
        path,
        motor_mask=bit,
        logical_abs=logical_abs,
        raw=raw,
    )

    assert parse_mask(updated["multiturn_anchor_valid_bitmap"]) & bit
    persisted = read_snapshot(path)
    assert parse_mask(persisted["multiturn_anchor_valid_bitmap"]) & bit
    assert persisted["multiturn_anchor_logical_abs"][motor_idx] == 8192
    assert persisted["multiturn_anchor_raw"][motor_idx] == 17
    assert persisted["joint_calib_results"][14]["success"] is True


def test_old_snapshot_schema_is_rejected(tmp_path):
    path = tmp_path / "legacy.json"
    path.write_text(
        json.dumps({"schema_version": 3, "joint_calib_results": []}),
        encoding="utf-8",
    )
    with pytest.raises(CalibrationSnapshotError, match="unsupported"):
        read_snapshot(path)


def test_seal_does_not_make_legacy_shape_valid():
    with pytest.raises(CalibrationSnapshotError, match="21 entries"):
        from desktop.core.calibration_snapshot import validate_snapshot

        validate_snapshot(seal_snapshot({"joint_calib_results": []}))


def test_manual_update_rejects_v5_snapshot(tmp_path):
    path = tmp_path / "calibration_bundle.mmcal.json"
    path.write_text(
        json.dumps(
            {
                "format": "mmhand_calibration_bundle",
                "schema_version": 5,
                "legacy": True,
            }
        ),
        encoding="utf-8",
    )

    with pytest.raises(CalibrationSnapshotError, match="schema v5"):
        update_manual_snapshot(
            path,
            valid_mask=1 << 3,
            min_raw=[100] * ENCODER_COUNT,
            max_raw=[200] * ENCODER_COUNT,
        )
    assert len(list(tmp_path.glob("*.legacy_v5_*.json"))) == 0


def test_scope_invalidation_preserves_manual_when_all_auto_is_removed(tmp_path):
    """全手 Reset Zero 只能清 Auto/LUT/限位，不能删除 Manual JSON。"""
    path = tmp_path / "calibration_bundle.mmcal.json"
    payload = merge_snapshot(
        None,
        _current_payload(0, _result(0, 0, 1)),
        updated_joint_mask=1 << 0,
    )
    atomic_write_snapshot(path, payload)

    invalidate_calibration_snapshot_scope(
        path,
        motor_mask=(1 << MOTOR_COUNT) - 1,
        joint_mask=(1 << ENCODER_COUNT) - 1,
        reason="manual_prepare",
    )

    updated = read_snapshot(path)
    assert parse_mask(updated["manual_reference_valid_bitmap"]) == (
        1 << ENCODER_COUNT
    ) - 1
    assert updated["manual_reference_min_raw"] == [100] * ENCODER_COUNT
    assert updated["manual_reference_max_raw"] == [5000] * ENCODER_COUNT
    assert updated["joint_calib_results"] == [None] * ENCODER_COUNT
    assert parse_mask(updated["motor_abs_limit_valid_bitmap"]) == 0
    assert parse_mask(updated["active_joint_mask"]) == 0
    assert len(list(tmp_path.glob("*.before_scoped_manual_prepare_*.json"))) == 1
