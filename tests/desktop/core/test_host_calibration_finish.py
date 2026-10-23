"""旧标定收尾在上位机通过普通直控执行的行为验证。"""

from copy import deepcopy
from types import SimpleNamespace

import pytest

from desktop.core.host_calibration.config import joint_motors
from desktop.core.host_calibration.finish import finish_dual_zero


def _result(joint, *, rows=None, start=-2.5, step=5.0, maximum=7.5):
    return {
        'success': True, 'scaled': True, 'mappingVersion': 4,
        'tendonLutValid': True, 'tendonLutCount': len(rows[0]) if rows else 3,
        'tendonLutMotor': list(joint_motors(joint)),
        'tendonLutAngleStartDeg': start, 'tendonLutAngleStepDeg': step,
        'angleMax': maximum,
        'tendonLutMotorAbs': rows or [[0, 1, 2], [0, -1, -2]],
    }


class _IO:
    def __init__(self, joints, *, publish_ms=0):
        self.selected_joint_mask = sum(1 << joint for joint in joints)
        self.state = SimpleNamespace(joint_calib_results=[None] * 21)
        for joint in joints:
            if len(joint_motors(joint)) == 2:
                self.state.joint_calib_results[joint] = _result(joint)
        self.elapsed = 0
        self.publish_ms = publish_ms
        self.commands = []
        self.waits = []
        self.releases = 0
        self.cancelled = False
        self.publish_ok = True
        self.release_ok = True
        self.cancel_after_publish = False

    def _state(self):
        return deepcopy(self.state)

    def calibration_result(self, joint):
        return deepcopy(self.state.joint_calib_results[joint])

    def owns_control(self):
        return not self.cancelled

    def now_ms(self):
        return self.elapsed

    def sleep_ms(self, milliseconds):
        self.waits.append(milliseconds)
        self.elapsed += milliseconds

    def publish(self, targets, speed, acc):
        self.commands.append((deepcopy(targets), speed, acc, self.elapsed))
        self.elapsed += self.publish_ms
        if self.cancel_after_publish:
            self.cancelled = True
        return self.publish_ok

    def _release_motion(self):
        self.releases += 1
        return self.release_ok


def test_completed_selected_dual_joints_send_one_batch_each_in_joint_order():
    io = _IO((20, 16, 8, 4, 0, 1))
    success, _message = finish_dual_zero(io, (1 << 20) | (1 << 16) | (1 << 4) | (1 << 0) | (1 << 1))
    assert success
    assert [tuple(command[0]) for command in io.commands] == [
        joint_motors(0), joint_motors(4), joint_motors(16), joint_motors(20)]
    assert [command[3] for command in io.commands] == [0, 50, 100, 150]
    assert [command[1:3] for command in io.commands] == [
        (400, 254), (400, 254), (400, 254), (400, 254)]
    # lroundf 的 ±0.5 远离零，不能使用 Python 的银行家舍入。
    assert all(list(command[0].values()) == [1, -1] for command in io.commands)
    assert io.waits == [50, 50, 50, 50]
    assert io.releases == 1 and io.elapsed == 200


@pytest.mark.parametrize('duration,expected_waits,expected_starts', [
    (20, [30, 30], [0, 50]),
    (75, [], [0, 75]),
])
def test_receipt_latency_counts_toward_original_fifty_ms_period(duration, expected_waits, expected_starts):
    io = _IO((0, 4), publish_ms=duration)
    assert finish_dual_zero(io, io.selected_joint_mask)[0]
    assert io.waits == expected_waits
    assert [command[3] for command in io.commands] == expected_starts
    assert io.releases == 1


def test_last_short_sample_uses_static_angle_max_for_interpolation():
    io = _IO((0,))
    io.state.joint_calib_results[0] = _result(
        0, rows=[[0, 10, 20], [0, -10, -20]], start=-8.0, step=5.0, maximum=1.0)
    assert finish_dual_zero(io, io.selected_joint_mask)[0]
    assert list(io.commands[0][0].values()) == [18, -18]


def test_two_point_fitted_lut_finishes_zero():
    io = _IO((0,))
    io.state.joint_calib_results[0] = _result(
        0, rows=[[100, 300], [-100, -300]], start=-2.5, step=5.0, maximum=2.5)
    assert finish_dual_zero(io, io.selected_joint_mask)[0]
    assert list(io.commands[0][0].values()) == [200, -200]


def test_host_generated_result_is_used_without_waiting_for_p4_snapshot_echo():
    io = _IO((0,))
    host_result = _result(0, rows=[[100, 200, 300], [-100, -200, -300]])
    io.state.joint_calib_results[0] = None
    io.calibration_result = lambda joint: deepcopy(host_result)
    io._state = lambda: pytest.fail('host finish must not depend on an asynchronous P4 LUT echo')
    assert finish_dual_zero(io, io.selected_joint_mask)[0]
    assert list(io.commands[0][0].values()) == [150, -150]


def test_missing_current_host_result_cannot_fall_back_to_an_old_p4_lut():
    io = _IO((0,))
    io.calibration_result = lambda joint: None
    success, message = finish_dual_zero(io, io.selected_joint_mask)
    assert not success and 'valid dual LUT' in message
    assert io.commands == [] and io.releases == 0


def test_simple_fake_port_can_still_use_snapshot_only_interface():
    io = _IO((0,))
    io.calibration_result = None
    assert finish_dual_zero(io, io.selected_joint_mask)[0]
    assert len(io.commands) == 1 and io.releases == 1


@pytest.mark.parametrize('changes', [
    {'success': False}, {'scaled': False}, {'mappingVersion': 3},
    {'tendonLutValid': False}, {'tendonLutCount': 1},
    {'tendonLutMotor': [1, 0]}, {'tendonLutMotorAbs': [[1], [2]]},
    {'tendonLutAngleStartDeg': 1.0}, {'tendonLutAngleStepDeg': float('nan')},
])
def test_invalid_lut_cannot_start_zero_finish(changes):
    io = _IO((0, 4))
    io.state.joint_calib_results[4].update(changes)
    success, message = finish_dual_zero(io, io.selected_joint_mask)
    assert not success and 'J04' in message
    assert io.commands == [] and io.releases == 0


def test_no_selected_completed_dual_joint_needs_no_motion_or_release():
    io = _IO((0, 1))
    assert finish_dual_zero(io, 1 << 1)[0]
    assert io.commands == [] and io.releases == 0


def test_cancelled_finish_sends_no_command():
    io = _IO((0,))
    io.cancelled = True
    assert not finish_dual_zero(io, io.selected_joint_mask)[0]
    assert io.commands == [] and io.releases == 0


def test_cancel_after_first_batch_prevents_remaining_commands():
    io = _IO((0, 4))
    io.cancel_after_publish = True
    assert not finish_dual_zero(io, io.selected_joint_mask)[0]
    assert len(io.commands) == 1 and io.releases == 0


def test_publish_failure_is_not_retried_or_reported_as_complete():
    io = _IO((0, 4))
    io.publish_ok = False
    success, message = finish_dual_zero(io, io.selected_joint_mask)
    assert not success and 'target publish failed' in message
    assert len(io.commands) == 1 and io.releases == 0


def test_release_failure_is_not_reported_as_complete():
    io = _IO((0,))
    io.release_ok = False
    success, message = finish_dual_zero(io, io.selected_joint_mask)
    assert not success and 'release failed' in message
    assert len(io.commands) == 1 and io.releases == 1
