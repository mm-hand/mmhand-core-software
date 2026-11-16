"""Single-tendon calibration: absolute-angle endpoint sweeps.

A single tendon joint is wound to URDF Max, then paid out until the spring
returns it to URDF Min. Both endpoints are the absolute joint angle freezing
near the commanded end - there is no travel threshold.
"""

import pytest

from desktop.core import protocol
from desktop.core.host_calibration import config, single_motor, sweep
from desktop.core.host_calibration.math import manual_angle_deg


class FakeIO:
    def __init__(self, low_raw=3500, high_raw=5000, start_raw=3500):
        self.t = 0
        self.stop_at = 10 ** 9
        self.values = {motor: 0 for motor in range(27)}
        self.loads = {motor: 0 for motor in range(27)}
        self.encoder_value = start_raw
        self.low_raw, self.high_raw = low_raw, high_raw
        self.commands = []
        self.diags = []
        self.traces = []
        self.limits = {}
        self.online = (1 << 27) - 1
        self.on_publish = None
        self.reject_publish = False
        self.hold = {}
        self.aa_ready = True
        self.ranges = {motor: (-50000, 50000) for motor in range(27)}

    def now_ms(self):
        return self.t

    def sleep_ms(self, ms):
        self.t += ms

    def owns_control(self):
        return self.t < self.stop_at

    def encoder_raw(self, joint):
        return self.encoder_value

    def encoder(self, joint):
        return self.encoder_raw(joint)

    def angle(self, joint):
        return manual_angle_deg(self, joint)

    def servo_abs(self, motor):
        return self.values.get(motor) if self.online & (1 << motor) else None

    def servo_raw(self, motor):
        value = self.servo_abs(motor)
        return None if value is None else value % 4096

    def servo_load(self, motor):
        return self.loads.get(motor) if self.online & (1 << motor) else None

    def logical_range(self, motor):
        return self.ranges.get(motor)

    def publish(self, targets, speed, acc):
        if self.reject_publish:
            return False
        previous = dict(self.values)
        self.commands.append((self.t, dict(targets), speed, acc))
        if self.on_publish:
            self.on_publish(self, targets, previous)
        self.values.update(targets)
        return True

    def diag(self, *values):
        self.diags.append(values)

    def trace(self, *values, **details):
        self.traces.append((values, details))

    def clear_limits(self, joint):
        for motor in config.joint_motors(joint):
            self.limits.pop(motor, None)

    def set_limit(self, motor, minimum, maximum):
        self.limits[motor] = (minimum, maximum)

    def manual_reference(self, joint):
        return {"valid": True, "minRaw": self.low_raw, "maxRaw": self.high_raw}

    def urdf_limits(self, joint):
        return 0.0, 90.0

    def urdf_config_crc(self):
        return 7

    def prepare_finger_aa(self, joint):
        return (self.aa_ready, '' if self.aa_ready else 'AA mapping invalid')

    def hold_targets_except(self, joints, active_joint, skip_joint_mask, params):
        return None if self.hold is None else dict(self.hold)


def link(io, motor, start_raw):
    """Encoder follows the motor travel; clamped at the two mechanical stops."""
    pose = {"raw": start_raw, "abs": io.values[motor]}

    def hook(state, targets, previous):
        current = targets.get(motor, previous.get(motor, pose["abs"]))
        raw = pose["raw"] + (current - pose["abs"])
        pose["raw"] = min(io.high_raw, max(io.low_raw, raw))
        pose["abs"] = current
        state.encoder_value = pose["raw"]
        state.loads[motor] = 200
    return hook


def frozen(io, raw):
    def hook(state, targets, previous):
        state.encoder_value = raw
    return hook


def run(io, params=None):
    return single_motor.run_single(io, 1, params or config.AutoCalibrationParams(),
                                   (1,), 0, 0, 1 << 1)


def test_sweep_reaches_max():
    io = FakeIO(start_raw=3500)
    motor = config.joint_motors(1)[0]
    io.on_publish = link(io, motor, 3500)
    outcome = sweep.sweep_to_end(io, 1, sweep.END_MAX, motors=(motor,), pull=motor, release=None)
    assert outcome.point is not None
    assert abs(outcome.point.angle - 90.0) <= config.kEndToleranceDeg


def test_sweep_release_reaches_min_on_the_spring():
    io = FakeIO(start_raw=5000)
    motor = config.joint_motors(1)[0]
    io.on_publish = link(io, motor, 5000)
    outcome = sweep.sweep_to_end(io, 1, sweep.END_MIN, motors=(motor,), pull=None, release=motor)
    assert outcome.point is not None
    assert abs(outcome.point.angle - 0.0) <= config.kEndToleranceDeg


def test_sweep_does_not_treat_a_frozen_joint_as_an_endpoint():
    io = FakeIO(start_raw=4300)
    motor = config.joint_motors(1)[0]
    io.on_publish = frozen(io, 4300)
    io.loads[motor] = 250
    outcome = sweep.sweep_to_end(io, 1, sweep.END_MAX, motors=(motor,), pull=motor, release=None)
    assert outcome.point is None
    assert outcome.reason == protocol.CALIB_DIAG_REASON_END_NOT_REACHED


def test_sweep_aborts_when_the_joint_stops_past_the_manual_endpoint():
    io = FakeIO(start_raw=3500)
    motor = config.joint_motors(1)[0]
    # 机械端点比 Manual Max 远 500 counts：Manual 端点比真实端点靠里。
    io.manual_reference = lambda joint: {"valid": True, "minRaw": 3500, "maxRaw": 5000}
    io.high_raw = 5500
    move = link(io, motor, 3500)

    def move_loaded(state, targets, previous):
        move(state, targets, previous)
        state.loads[motor] = 1000
    io.on_publish = move_loaded
    outcome = sweep.sweep_to_end(io, 1, sweep.END_MAX, motors=(motor,), pull=motor, release=None)
    assert outcome.point is None
    assert outcome.reason == protocol.CALIB_DIAG_REASON_PAST_MANUAL_ENDPOINT


def test_sweep_caps_a_runaway_release():
    io = FakeIO(start_raw=4300)
    motor = config.joint_motors(1)[0]
    io.on_publish = frozen(io, 4300)
    params = config.AutoCalibrationParams(tightenSpeedCountsPerSec=20000)
    outcome = sweep.sweep_to_end(io, 1, sweep.END_MIN, motors=(motor,), pull=None,
                                 release=motor, params=params)
    assert outcome.point is None
    assert outcome.reason == protocol.CALIB_DIAG_REASON_WEAK_RETURN


def test_sweep_propagates_publish_failure():
    io = FakeIO()
    motor = config.joint_motors(1)[0]
    io.reject_publish = True
    outcome = sweep.sweep_to_end(io, 1, sweep.END_MAX, motors=(motor,), pull=motor, release=None)
    assert outcome.reason == protocol.CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED


def test_run_single_builds_result_and_limits():
    io = FakeIO()
    motor = config.joint_motors(1)[0]
    io.on_publish = link(io, motor, 3500)
    outcome = run(io)
    assert outcome["success"] is True
    result = outcome["result"]
    assert result["tendonLutValid"] is True
    assert result["encoderSpan"] == pytest.approx(1500, abs=8)
    assert outcome["limits"] == io.limits
    assert motor in io.limits


def test_run_single_requires_the_tendon_to_move_the_joint():
    io = FakeIO(start_raw=4300)
    motor = config.joint_motors(1)[0]
    io.on_publish = frozen(io, 4300)
    io.loads[motor] = 200
    outcome = run(io)
    assert outcome["success"] is False
    assert outcome["reason"] == protocol.CALIB_DIAG_REASON_END_NOT_REACHED


def test_run_single_hold_failure_is_reported():
    io = FakeIO()
    motor = config.joint_motors(1)[0]
    io.on_publish = link(io, motor, 3500)
    io.hold = None
    outcome = run(io)
    assert outcome["success"] is False
    assert outcome["reason"] == protocol.CALIB_DIAG_REASON_HOLD_TARGET_FAILED


def test_run_single_aborts_when_the_aa_is_not_ready():
    io = FakeIO()
    io.aa_ready = False
    outcome = run(io)
    assert outcome["success"] is False
    assert outcome["aborted"] is True
    assert outcome["reason"] == 'AA mapping invalid'
    assert not io.commands


def test_release_accepts_where_the_spring_stops():
    """J10: the spring returns the joint to +3 deg, not to the Manual Min at -9 deg."""
    io = FakeIO(start_raw=5000)
    motor = config.joint_motors(1)[0]
    pose = {"raw": 5000}

    def spring(state, targets, previous):
        pose["raw"] = max(3550, pose["raw"] - 50)
        state.encoder_value = pose["raw"]
        state.loads[motor] = 0

    io.on_publish = spring
    outcome = sweep.sweep_to_end(io, 1, sweep.END_MIN, motors=(motor,), pull=None, release=motor)
    assert outcome.point is not None
    assert outcome.point.angle == pytest.approx(3.0, abs=1.0)


def test_release_does_not_accept_the_start_position():
    io = FakeIO(start_raw=5000)
    motor = config.joint_motors(1)[0]
    io.on_publish = frozen(io, 5000)
    io.loads[motor] = 0
    outcome = sweep.sweep_to_end(io, 1, sweep.END_MIN, motors=(motor,), pull=None, release=motor)
    assert outcome.point is None
    assert outcome.reason == protocol.CALIB_DIAG_REASON_WEAK_RETURN


def test_run_single_accepts_a_spring_that_stops_short():
    """J10 end to end: the spring returns to raw 3550 and the LUT still builds."""
    io = FakeIO(start_raw=3500)
    motor = config.joint_motors(1)[0]
    mark = {"abs": None}

    def hook(state, targets, previous):
        current = targets.get(motor, previous.get(motor, 0))
        if mark["abs"] is None:
            state.encoder_value = min(5000, 3500 + current)
            if state.encoder_value >= 5000:
                mark["abs"] = current
        else:
            state.encoder_value = min(5000, max(3550, 5000 - (mark["abs"] - current)))
        state.loads[motor] = 150

    io.on_publish = hook
    outcome = run(io)
    assert outcome["success"] is True
    assert outcome["result"]["tendonLutValid"] is True
    assert outcome["result"]["autoMinRaw"] == 3550
    assert outcome["result"]["autoMaxRaw"] == 5000
