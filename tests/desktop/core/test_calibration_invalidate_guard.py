"""Repeated calibration invalidation must not rewrite the snapshot or add backups."""
from desktop.core.calibration_snapshot import (
    invalidate_calibration_snapshot_scope,
    read_snapshot,
)
from test_calibration_snapshot import two_finger_snapshot


def _backup_count(path):
    backups = path.parent / 'backups'
    return len(list(backups.glob('*'))) if backups.exists() else 0


def test_second_invalidate_of_same_scope_is_a_noop(tmp_path):
    path = tmp_path / 'hand.mmcal.json'
    two_finger_snapshot(path)

    first = invalidate_calibration_snapshot_scope(
        path, motor_mask=1 << 19, joint_mask=1 << 15, reason='manual_prepare'
    )
    after_first = read_snapshot(path)
    backups_after_first = _backup_count(path)
    assert 'backed up' in first
    assert after_first['joint_calib_results'][15] is None

    second = invalidate_calibration_snapshot_scope(
        path, motor_mask=1 << 19, joint_mask=1 << 15, reason='manual_prepare'
    )
    assert 'already invalid' in second
    assert read_snapshot(path) == after_first
    assert _backup_count(path) == backups_after_first


def test_invalidate_still_writes_when_only_limits_remain(tmp_path):
    path = tmp_path / 'hand.mmcal.json'
    two_finger_snapshot(path)
    invalidate_calibration_snapshot_scope(
        path, motor_mask=1 << 19, joint_mask=1 << 15, reason='manual_prepare'
    )
    # Other scope must still be invalidated and backed up independently.
    result = invalidate_calibration_snapshot_scope(
        path, motor_mask=1 << 14, joint_mask=1 << 11, reason='manual_prepare'
    )
    assert 'backed up' in result
    assert read_snapshot(path)['joint_calib_results'][11] is None
