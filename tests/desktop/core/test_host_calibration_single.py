"""Recorded mechanical paths exercise the original single-tendon decision tree."""

from dataclasses import replace

from desktop.core import protocol as p
from desktop.core.host_calibration.config import AutoCalibrationParams, kSingleTendonMaxReleaseCounts
from desktop.core.host_calibration.math import TendonRawFitBuffer, orient_encoder_raw
from desktop.core.host_calibration.time_alignment import TimeAlignmentError
from desktop.core.host_calibration.single_motor import (
    AutoCalibState, JointCalibContext, run_single, step_single,
)


class FakeIO:
    def __init__(self):
        self.ms = 0
        self.abs = 0
        self.encoder_value = 2000
        self.raw_value = 14384
        self.load = 0
        self.owner = True
        self.direction_value = -1
        self.limits = {}
        self.diags = []
        self.traces = []
        self.commands = []
        self.dynamic = False
        self.publish_ok = True
        self.holds_ok = True
        self.range = (-30000, 30000)

    def now_ms(self):
        return self.ms

    def sleep_ms(self, value):
        self.ms += value
        assert self.ms < 120000, "fixture trajectory did not converge"

    def owns_control(self):
        return self.owner

    def encoder(self, joint):
        return self.encoder_value

    def encoder_raw(self, joint):
        return self.raw_value

    def servo_abs(self, motor):
        return self.abs

    def servo_raw(self, motor):
        return None if self.abs is None else self.abs % 4096

    def servo_load(self, motor):
        return self.load

    def logical_range(self, motor):
        return self.range

    def inside_limit(self, motor, target):
        return self.range[0] <= target <= self.range[1]

    def set_direction(self, joint, direction):
        self.direction_value = direction

    def direction(self, joint):
        return self.direction_value

    def clear_limits(self, joint):
        self.limits.clear()

    def set_limit(self, motor, lo, hi):
        self.limits[motor] = (lo, hi)

    def manual_reference(self, joint):
        return {"valid": True, "minRaw": 14384, "maxRaw": 10384}

    def urdf_limits(self, joint):
        return (0.0, 83.0)

    def urdf_config_crc(self):
        return 0x2BC44010

    def hold_targets_except(self, ordered, active, skip, params):
        return {} if self.holds_ok else None

    def prepare_finger_aa(self, joint):
        return True, ''

    def publish(self, targets, speed, acc):
        self.commands.append((self.ms, dict(targets), speed, acc))
        if self.dynamic and 2 in targets:
            self.abs = targets[2]
            self.encoder_value = 2000 + 4 * max(0, min(1000, self.abs))
            self.raw_value = orient_encoder_raw(self.encoder_value, -1)
            self.load = 250 if self.abs >= 1000 else 0
        return self.publish_ok

    def diag(self, *values):
        self.diags.append(values)

    def diag_if_empty(self, *values):
        if not self.diags:
            self.diag(*values)

    def trace(self, state, joint, motor=255, **details):
        self.traces.append((state, joint, motor, details))


def tighten_context(io):
    ctx = JointCalibContext(1)
    step_single(ctx, io, AutoCalibrationParams(), {})
    assert ctx.state == AutoCalibState.TIGHTEN
    return ctx


def release_context(io):
    ctx = tighten_context(io)
    ctx.start_found = True
    ctx.start_encoder = io.encoder_value
    ctx.start_servo_abs = io.abs
    return ctx


def test_tight_end_requires_load_motion_and_stall_in_same_200ms_window():
    io = FakeIO()
    ctx = tighten_context(io)
    io.ms, io.abs, io.load = 200, 80, 250
    io.encoder_value += 100
    step_single(ctx, io, AutoCalibrationParams(), {})
    assert not ctx.start_found  # load + motion alone does not satisfy angle stall
    io.ms, io.abs, io.load = 400, 160, 249
    step_single(ctx, io, AutoCalibrationParams(), {})
    assert not ctx.start_found  # motion + stall below load threshold
    io.ms, io.abs, io.load = 600, 200, 250
    step_single(ctx, io, AutoCalibrationParams(), {})
    assert not ctx.start_found  # load + stall but servo moved only 40 counts
    io.ms, io.abs = 800, 250
    targets = {}
    step_single(ctx, io, AutoCalibrationParams(), targets)
    assert ctx.start_found and ctx.state == AutoCalibState.PRE_RELEASE_SYNC
    assert targets == {2: 250}


def test_pre_release_catches_up_to_actual_and_requires_two_stable_windows():
    io = FakeIO()
    ctx = tighten_context(io)
    ctx.state, ctx.start_found = AutoCalibState.PRE_RELEASE_SYNC, True
    ctx.pre_release_sync_target_abs = 0
    io.ms, io.abs = 200, 41
    targets = {}
    step_single(ctx, io, AutoCalibrationParams(), targets)
    assert targets == {2: 41}
    assert ctx.pre_release_sync_stable_samples == 0
    io.ms = 400
    step_single(ctx, io, AutoCalibrationParams(), {})
    assert ctx.pre_release_sync_stable_samples == 1
    io.ms = 600
    step_single(ctx, io, AutoCalibrationParams(), {})
    assert ctx.state == AutoCalibState.TIGHTEN
    assert (ctx.command_abs, ctx.start_servo_abs, ctx.start_encoder) == (41, 41, 2000)
    assert ctx.fit_samples.count == 1 and ctx.fit_samples.next_travel_deg == 5.0


def test_two_turn_actual_guard_holds_feedback_and_fails_without_new_release():
    io = FakeIO()
    ctx = release_context(io)
    io.ms, io.abs = 20, -8193
    targets = {}
    step_single(ctx, io, AutoCalibrationParams(), targets)
    assert ctx.state == AutoCalibState.FAILED
    assert targets == {2: -8193}
    assert io.diags[-1][0] == p.CALIB_DIAG_REASON_RELEASE_OVER_TWO_TURNS
    assert io.diags[-1][4:7] == (8193, 8192, 8193)


def test_two_turn_command_guard_runs_before_actual_catches_up_and_boundary_is_allowed():
    io = FakeIO()
    ctx = release_context(io)
    params = replace(AutoCalibrationParams(), tightenSpeedCountsPerSec=50)
    ctx.command_abs = -8191
    io.ms = 20
    targets = {}
    step_single(ctx, io, params, targets)
    assert targets == {2: -8192} and ctx.state == AutoCalibState.TIGHTEN
    io.ms = 40
    targets = {}
    step_single(ctx, io, params, targets)
    assert targets == {2: 0} and ctx.state == AutoCalibState.FAILED
    assert io.diags[-1][4:7] == (0, kSingleTendonMaxReleaseCounts, 8193)


def test_release_angle_stall_requires_two_windows_and_at_least_12_degrees():
    io = FakeIO()
    ctx = release_context(io)
    ctx.start_encoder = 4000
    ctx.window_start_encoder = 3980
    io.encoder_value = 3980
    io.ms, io.abs = 200, -80
    step_single(ctx, io, AutoCalibrationParams(), {})
    assert ctx.release_angle_stall_stable == 1 and ctx.state == AutoCalibState.TIGHTEN
    io.ms, io.abs = 400, -160
    step_single(ctx, io, AutoCalibrationParams(), {})
    assert ctx.release_angle_stall_stable == 2 and ctx.state == AutoCalibState.TIGHTEN
    # 两个稳定窗口也不能绕过12°实际磁编进展。
    io.ms, io.abs, io.encoder_value = 600, -240, 3900
    step_single(ctx, io, AutoCalibrationParams(), {})
    assert ctx.release_angle_stall_stable == 0


def test_fractional_per_cycle_command_counts_are_accumulated():
    io = FakeIO()
    ctx = tighten_context(io)
    params = replace(AutoCalibrationParams(), tightenSpeedCountsPerSec=333)
    commands = []
    for ms in (20, 40, 60):
        io.ms = ms
        targets = {}
        step_single(ctx, io, params, targets)
        commands.append(targets[2])
    assert commands == [6, 13, 19]
    assert ctx.command_remainder == 980


def test_initial_feedback_wait_has_original_1200ms_cutoff_and_reason():
    io = FakeIO()
    io.encoder_value = io.raw_value = None
    ctx = JointCalibContext(1)
    for ms in (0, 1199):
        io.ms = ms
        step_single(ctx, io, AutoCalibrationParams(), {})
        assert ctx.state == AutoCalibState.INIT
    io.ms = 1200
    step_single(ctx, io, AutoCalibrationParams(), {})
    assert ctx.state == AutoCalibState.FAILED
    assert io.diags[-1][0] == p.CALIB_DIAG_REASON_ENCODER_ERROR


def test_full_mechanical_trajectory_runs_search_sync_release_lut_and_holds_auto_min():
    io = FakeIO()
    io.dynamic = True
    outcome = run_single(io, 1, AutoCalibrationParams(), [1], 0, 0, 1 << 1)
    assert outcome["success"] and not outcome["aborted"]
    result = outcome["result"]
    assert result["success"] and result["tendonLutValid"]
    assert result["tendonLutMotor"] == [2, 255]
    assert result["encoderSpan"] == 4000
    assert result["angleMax"] == 83.0
    assert "currentMaxDeg" not in result
    assert result["zeroServoAbs"] == result["startServoAbs"] + 500
    assert outcome["limits"] == io.limits
    events = [event[0] for event in io.traces]
    assert events.index("START_FOUND") < events.index("PRE_RELEASE_SYNC_READY") < events.index("BEGIN_RELEASE")
    assert events.index("BEGIN_RELEASE") < events.index("SINGLE_DONE_AT_AUTO_MIN")
    assert io.commands[-1][1] == {2: result["startServoAbs"]}
    assert all((speed, acc) == (400, 20) for _, _, speed, acc in io.commands)
    assert not io.diags


def test_single_release_uses_only_aligned_samples_including_both_endpoints():
    io = FakeIO()
    io.dynamic = True
    window = object()
    fit = TendonRawFitBuffer()
    for encoder, absolute in ((2000, 777), (4000, 888), (6000, 999)):
        fit.append(encoder, absolute, 0)

    def begin(joint, motors):
        assert (joint, motors) == (1, (2,))
        assert io.traces[-1][0] == 'PRE_RELEASE_SYNC'
        assert io.abs >= 1000  # Begin after the tight-end hold settles.
        return window

    def finish(value):
        assert value is window
        assert io.encoder_value == 2000
        return fit

    io.begin_lut_sampling, io.finish_lut_sampling = begin, finish
    outcome = run_single(io, 1, AutoCalibrationParams(), [1], 0, 0, 2)
    assert outcome['success']
    assert fit.count == 3
    result = outcome['result']
    assert result['tendonLutMotorAbs'][0][0] == 777
    assert result['tendonLutMotorAbs'][0][result['tendonLutCount'] - 1] == 999
    assert result['startServoAbs'] != 777  # Mechanical metadata stays measured.
    assert io.commands[-1][1] == {2: result['startServoAbs']}
    assert all((speed, acc) == (400, 20) for _, _, speed, acc in io.commands)


def test_single_alignment_failure_holds_actual_without_using_old_samples():
    io = FakeIO()
    io.dynamic = True
    io.begin_lut_sampling = lambda *_: object()

    def finish(_window):
        raise TimeAlignmentError('no common valid raw/ABS time interval')

    io.finish_lut_sampling = finish
    outcome = run_single(io, 1, AutoCalibrationParams(), [1], 0, 0, 2)
    assert not outcome['success']
    assert 'no common valid' in outcome['reason']
    assert outcome['result'] is None
    assert io.diags[0][0] == p.CALIB_DIAG_REASON_RESULT_BUILD_FAILED
    assert io.commands[-1][1] == {2: io.abs}
    assert io.limits == {}


def test_control_hold_and_publish_failures_abort_the_outer_sequence():
    for failure, expected in (("owner", p.CALIB_DIAG_REASON_CONTROL_LOST),
                              ("holds_ok", p.CALIB_DIAG_REASON_HOLD_TARGET_FAILED),
                              ("publish_ok", p.CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED)):
        io = FakeIO()
        setattr(io, failure, False)
        outcome = run_single(io, 1, AutoCalibrationParams(), [1], 0, 0, 2)
        assert outcome["aborted"] and not outcome["success"]
        assert io.diags[0][0] == expected


def test_lost_feedback_and_session_range_keep_original_diagnostic_phase():
    io = FakeIO()
    ctx = tighten_context(io)
    io.encoder_value = None
    step_single(ctx, io, AutoCalibrationParams(), {})
    assert io.diags[-1][:2] == (p.CALIB_DIAG_REASON_FEEDBACK_LOST, p.CALIB_DIAG_PHASE_FIND_MAX)
    io = FakeIO()
    ctx = tighten_context(io)
    io.ms = 20
    io.range = (-7, 7)
    targets = {}
    step_single(ctx, io, AutoCalibrationParams(), targets)
    assert ctx.state == AutoCalibState.FAILED
    assert targets == {2: 8}
    assert io.diags[-1][:2] == (p.CALIB_DIAG_REASON_ABS_LIMIT_INVALID, p.CALIB_DIAG_PHASE_FIND_MAX)


def test_outer_publish_failure_preserves_execution_timeout_diagnostic():
    io = FakeIO()
    io.publish_ok = False
    io.diag(p.CALIB_DIAG_REASON_TARGET_APPLY_TIMEOUT, p.CALIB_DIAG_PHASE_RELEASE,
            1, 2, 12, 0, 100, 4)
    outcome = run_single(io, 1, AutoCalibrationParams(), [1], 0, 0, 2)
    assert outcome["aborted"]
    assert len(io.diags) == 1
    assert io.diags[0][:4] == (p.CALIB_DIAG_REASON_TARGET_APPLY_TIMEOUT, p.CALIB_DIAG_PHASE_RELEASE, 1, 2)
