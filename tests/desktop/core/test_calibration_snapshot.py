import json

import pytest

from desktop.core.calibration_snapshot import (
    _empty_v7_snapshot,
    CalibrationSnapshotError,
    atomic_write_snapshot,
    format_mask,
    invalidate_calibration_snapshot_scope,
    joint_calib_result_has_valid_tendon_lut,
    joint_calib_result_tendon_lut_abs_ranges,
    merge_snapshot,
    multiturn_anchor_fields,
    parse_mask,
    read_snapshot,
    seal_snapshot,
    update_manual_snapshot,
    update_snapshot_multiturn_anchors,
    validate_snapshot_for_load,
)
from desktop.core.protocol import ENCODER_COUNT, HAND_PROFILE, MOTOR_COUNT


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
    assert validate_snapshot_for_load(seal_snapshot(_empty_v7_snapshot())) == (
        "snapshot contains no calibration result or Manual endpoints"
    )


def test_manual_only_snapshot_is_loadable():
    payload = _empty_v7_snapshot()
    payload["manual_reference_valid_bitmap"] = format_mask((1 << ENCODER_COUNT) - 1)
    payload["manual_reference_min_raw"] = [100] * ENCODER_COUNT
    payload["manual_reference_max_raw"] = [5000] * ENCODER_COUNT
    assert validate_snapshot_for_load(seal_snapshot(payload)) is None


def test_partial_manual_snapshot_is_loadable():
    payload = _empty_v7_snapshot()
    payload["manual_reference_min_raw"] = [0] * ENCODER_COUNT
    payload["manual_reference_max_raw"] = [0] * ENCODER_COUNT
    payload["manual_reference_min_raw"][15] = 100
    payload["manual_reference_max_raw"][15] = 5000
    payload["manual_reference_valid_bitmap"] = format_mask(1 << 15)
    assert validate_snapshot_for_load(seal_snapshot(payload)) is None


def two_finger_snapshot(path):
    payload = merge_snapshot(None, _current_payload(11, _result(11, 14)),
                             updated_joint_mask=1 << 11)
    payload = merge_snapshot(payload, _current_payload(15, _result(15, 19)),
                             updated_joint_mask=1 << 15)
    atomic_write_snapshot(path, payload)
    return update_snapshot_multiturn_anchors(
        path, motor_mask=(1 << 14) | (1 << 19),
        logical_abs=[8192 + m for m in range(MOTOR_COUNT)],
        raw=[m for m in range(MOTOR_COUNT)])


@pytest.mark.parametrize('action', ['unchanged', 'changed', 'clear', 'release'])
def test_scoped_manual_and_release_preserve_other_finger_after_reconnect(tmp_path, action):
    path = tmp_path / 'hand.mmcal.json'
    before = two_finger_snapshot(path)
    mins, maxs = [0] * ENCODER_COUNT, [0] * ENCODER_COUNT
    mins[15], maxs[15] = 100, 5000
    if action == 'release':
        invalidate_calibration_snapshot_scope(path, motor_mask=1 << 19,
            joint_mask=1 << 15, reason='manual_prepare')
    else:
        if action == 'changed':
            maxs[15] = 5100
        if action == 'clear':
            mins[15] = maxs[15] = 0
        update_manual_snapshot(path, valid_mask=0 if action == 'clear' else 1 << 15,
            min_raw=mins, max_raw=maxs, updated_joint_mask=1 << 15)
    after = read_snapshot(path)
    for field, index in [('joint_calib_results', 11), ('manual_reference_min_raw', 11),
                         ('manual_reference_max_raw', 11), ('motor_abs_limit_min', 14),
                         ('motor_abs_limit_max', 14), ('multiturn_anchor_logical_abs', 14)]:
        assert after[field][index] == before[field][index], field
    for field, bit in [('manual_reference_valid_bitmap', 1 << 11),
                       ('motor_abs_limit_valid_bitmap', 1 << 14),
                       ('multiturn_anchor_valid_bitmap', 1 << 14)]:
        assert parse_mask(after[field]) & bit == parse_mask(before[field]) & bit
    if action == 'unchanged':
        assert after['joint_calib_results'] == before['joint_calib_results']
        assert parse_mask(after['updated_joint_mask']) == 0
    else:
        assert after['joint_calib_results'][15] is None
        assert not parse_mask(after['motor_abs_limit_valid_bitmap']) & (1 << 19)
        assert not parse_mask(after['multiturn_anchor_valid_bitmap']) & (1 << 19)


def test_full_manual_replace_still_removes_missing_entries(tmp_path):
    path = tmp_path / 'hand.mmcal.json'
    two_finger_snapshot(path)
    update_manual_snapshot(path, valid_mask=0, min_raw=[0] * ENCODER_COUNT,
                           max_raw=[0] * ENCODER_COUNT)
    after = read_snapshot(path)
    assert parse_mask(after['manual_reference_valid_bitmap']) == 0
    assert after['joint_calib_results'] == [None] * ENCODER_COUNT


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
    assert "multiturn_anchor_raw" not in persisted
    assert multiturn_anchor_fields(persisted)[2][motor_idx] == 0
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
    assert len(list((tmp_path / "backups").glob("*.before_scoped_manual_prepare_*.json"))) == 1


def test_manual_identity_is_written_atomically_and_foreign_file_is_rejected(tmp_path):
    path = tmp_path / "manual.mmcal.json"
    identity = {"device_id": "0000123456789ABC", "urdf_repo_path": "models/hand.urdf"}
    update_manual_snapshot(path, valid_mask=0, min_raw=[0] * 21,
                           max_raw=[0] * 21, identity=identity)
    assert read_snapshot(path)["device_id"] == identity["device_id"]
    before = path.read_bytes()
    foreign = dict(identity, device_id="0000123456789ABD")
    with pytest.raises(CalibrationSnapshotError, match="identity mismatch"):
        update_manual_snapshot(path, valid_mask=0, min_raw=[0] * 21,
                               max_raw=[0] * 21, identity=foreign)
    assert path.read_bytes() == before


def test_manual_snapshot_creation_includes_hand_profile(tmp_path):
    path = tmp_path / "hand.mmcal.json"
    update_manual_snapshot(path, identity={
        "device_id": "0" * 15 + "1", "urdf_repo_path": "models/hand.urdf",
        "product_fingerprint": {}},
        valid_mask=1 << 15, min_raw=[0] * ENCODER_COUNT, max_raw=[100] * ENCODER_COUNT,
        updated_joint_mask=1 << 15)
    assert read_snapshot(path)["hand_profile"] == HAND_PROFILE


def test_manual_update_backfills_missing_hand_profile(tmp_path):
    path = tmp_path / "hand.mmcal.json"
    payload = merge_snapshot(None, _current_payload(15, _result(15, 19)),
                             updated_joint_mask=1 << 15)
    payload.pop("hand_profile")
    atomic_write_snapshot(path, payload)
    update_manual_snapshot(path, valid_mask=1 << 15,
        min_raw=[0] * ENCODER_COUNT, max_raw=[100] * ENCODER_COUNT,
        updated_joint_mask=1 << 15)
    assert read_snapshot(path)["hand_profile"] == HAND_PROFILE


def _manual_only_bundle(path):
    payload = _empty_v7_snapshot()
    payload["manual_reference_valid_bitmap"] = format_mask((1 << ENCODER_COUNT) - 1)
    payload["manual_reference_min_raw"] = [100] * ENCODER_COUNT
    payload["manual_reference_max_raw"] = [5000] * ENCODER_COUNT
    atomic_write_snapshot(path, payload)
    return payload


def test_empty_scope_invalidation_registers_anchor_scope(tmp_path):
    """刷机后的空 bundle 首次标定：scoped 登记必须写入，锚点保存才被放行。"""
    path = tmp_path / "hand.mmcal.json"
    _manual_only_bundle(path)

    message = invalidate_calibration_snapshot_scope(
        path, motor_mask=0x000F8000, joint_mask=0x0000F000,
        reason="servo_abs_rebase_zero")

    assert message == "scope had no data; anchor scope registered"
    snapshot = read_snapshot(path)
    assert parse_mask(snapshot["scoped_servo_abs_rebase_zero_motor_mask"]) == 0x000F8000
    assert parse_mask(snapshot["scoped_servo_abs_rebase_zero_joint_mask"]) == 0x0000F000
    update_snapshot_multiturn_anchors(
        path, motor_mask=0x000F8000,
        logical_abs=[8192 + m for m in range(MOTOR_COUNT)],
        raw=[m for m in range(MOTOR_COUNT)])
    saved = read_snapshot(path)
    assert parse_mask(saved["multiturn_anchor_valid_bitmap"]) & 0x000F8000 == 0x000F8000


def test_empty_scope_invalidation_is_idempotent_without_rewrite(tmp_path):
    path = tmp_path / "hand.mmcal.json"
    _manual_only_bundle(path)
    invalidate_calibration_snapshot_scope(
        path, motor_mask=0x000F8000, joint_mask=0x0000F000,
        reason="servo_abs_rebase_zero")
    before = path.read_bytes()

    message = invalidate_calibration_snapshot_scope(
        path, motor_mask=0x000F8000, joint_mask=0x0000F000,
        reason="servo_abs_rebase_zero")

    assert message == "scope already invalid; snapshot not rewritten"
    assert path.read_bytes() == before


def test_retained_out_of_scope_anchors_do_not_block_new_anchor_save(tmp_path):
    """历史锚点在当前事务范围外时必须原样携带，不能毒化整次锚点保存。"""
    path = tmp_path / "hand.mmcal.json"
    _manual_only_bundle(path)
    invalidate_calibration_snapshot_scope(path, motor_mask=1, joint_mask=1,
                                          reason="servo_abs_rebase_zero")
    update_snapshot_multiturn_anchors(
        path, motor_mask=1,
        logical_abs=[8192 + m for m in range(MOTOR_COUNT)],
        raw=[m for m in range(MOTOR_COUNT)])

    invalidate_calibration_snapshot_scope(
        path, motor_mask=0x000F8000, joint_mask=0x0000F000,
        reason="servo_abs_rebase_zero")

    # 模拟真实采集路径：requested 是累计位图（含保留位），保留电机携带原值。
    logical_abs = [111 + m for m in range(MOTOR_COUNT)]
    raw = [7 + m for m in range(MOTOR_COUNT)]
    logical_abs[0], raw[0] = 8192, 0
    saved = update_snapshot_multiturn_anchors(
        path, motor_mask=0x000F8001,
        logical_abs=logical_abs, raw=raw)

    bitmap = parse_mask(saved["multiturn_anchor_valid_bitmap"])
    assert bitmap & 0x000F8001 == 0x000F8001
    # 范围外的 M00 保留锚点值不被新写入覆盖，也不被截掉。
    assert bitmap & 1
    assert saved["multiturn_anchor_logical_abs"][0] == 8192
    assert "multiturn_anchor_raw" not in saved
    assert multiturn_anchor_fields(saved)[2][0] == 0
    assert saved["multiturn_anchor_logical_abs"][15] == 111 + 15


def test_new_out_of_scope_anchor_is_still_rejected(tmp_path):
    path = tmp_path / "hand.mmcal.json"
    _manual_only_bundle(path)
    with pytest.raises(CalibrationSnapshotError,
                       match="new multiturn anchor motors are not active"):
        update_snapshot_multiturn_anchors(
            path, motor_mask=1 << 20,
            logical_abs=[0] * MOTOR_COUNT, raw=[0] * MOTOR_COUNT)


def test_full_5_degree_table_is_valid_and_ranges_follow_the_endpoints():
    result = _result(1, 2)
    result.update(
        tendonLutCount=24,
        tendonLutAngleStartDeg=-15.0,
        tendonLutAngleStepDeg=5.0,
        tendonLutMotorAbs=[[1000 + 37 * index for index in range(24)],
                           [0] * 24],
    )
    assert joint_calib_result_has_valid_tendon_lut(result, 1)
    assert joint_calib_result_tendon_lut_abs_ranges(result, 1) == [(2, 1000, 1851)]


def test_old_two_point_lut_still_validates():
    result = _result(1, 2)
    result.update(
        tendonLutCount=2,
        tendonLutAngleStartDeg=-15.0,
        tendonLutAngleStepDeg=105.0,
        tendonLutMotorAbs=[[4000, 9000], [0] * 24],
    )
    assert joint_calib_result_has_valid_tendon_lut(result, 1)
    assert joint_calib_result_tendon_lut_abs_ranges(result, 1) == [(2, 4000, 9000)]
