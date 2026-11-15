"""Dual-joint calibration: absolute-angle endpoint sweeps.

The trajectories below are the contract that replaced the old load/travel
predicates: a joint reaches an endpoint only when the absolute angle freezes
near the commanded URDF end. Load is never an endpoint test.
"""

import pytest

from desktop.core import protocol
from desktop.core.host_calibration import config, dual_motor as dual, sweep
from desktop.core.host_calibration.math import manual_angle_deg


class FakeIO:
    def __init__(self, joint=16, low_raw=3500, high_raw=5000, start_raw=3500):
        self.t = 0
        self.stop_at = 10 ** 9
        self.values = {motor: 0 for motor in range(27)}
        self.loads = {motor: 0 for motor in range(27)}
        self.encoder_value = start_raw
        self.encoder_values = {}
        self.low_raw, self.high_raw = low_raw, high_raw
        self.commands = []
        self.diags = []
        self.traces = []
        self.limits = {}
        self.online = (1 << 27) - 1
        self.unknown = self.pending = self.failed = 0
        self.configured = self.online
        self.age = 0
        self.on_sleep = None
        self.on_publish = None
        self.ranges = {motor: (-50000, 50000) for motor in range(27)}
        self.reject_publish = False
        self.selected_joint_mask = 0
        self.selected_motor_mask = 0
        self.workspace_helpers = []

    def now_ms(self):
        return self.t

    def sleep_ms(self, ms):
        self.t += ms
        if self.on_sleep:
            self.on_sleep(self)

    def owns_control(self):
        return self.t < self.stop_at

    def encoder_raw(self, joint):
        return self.encoder_values.get(joint, self.encoder_value)

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

    def inside_limit(self, motor, target):
        bounds = self.logical_range(motor)
        return bounds is not None and bounds[0] <= target <= bounds[1]

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

    def note_workspace_helper(self, joint):
        self.workspace_helpers.append(joint)

    def snapshot(self):
        return {"servo_feedback_age_ms": self.age,
                "servo_online_bitmap": self.online,
                "multi_turn_unknown_bitmap": self.unknown,
                "servo_multiturn_configured_bitmap": self.configured,
                "servo_recovery_pending_bitmap": self.pending,
                "servo_recovery_failed_bitmap": self.failed,
                "servo_route_conflict_bitmap": 0}

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


def link(io, joint, primary, secondary=None, taut=210):
    """Move the joint from the commanded motor travel; clamp at both endpoints."""
    pose = {"p": io.values[primary],
            "s": io.values[secondary] if secondary is not None else 0,
            "raw": io.encoder_raw(joint)}

    def hook(state, targets, previous):
        p = targets.get(primary, previous.get(primary, pose["p"]))
        s = targets.get(secondary, previous.get(secondary, pose["s"])) if secondary is not None else pose["s"]
        raw = pose["raw"] + (p - pose["p"]) - (s - pose["s"])
        pose["p"], pose["s"] = p, s
        pose["raw"] = min(io.high_raw, max(io.low_raw, raw))
        state.encoder_values[joint] = pose["raw"]
        for motor in (primary, secondary):
            if motor is not None and motor in targets:
                state.loads[motor] = taut
    return hook


def freeze(io, raw):
    """Pin the joint angle at `raw` (a mechanically stuck or still-slack joint)."""
    def hook(state, targets, previous):
        state.encoder_values[16] = raw
    return hook


def machine(io, joint=16, params=None):
    return dual._DualMachine(io, joint, params or config.AutoCalibrationParams())


@pytest.mark.parametrize("joint,expected", [
    (0, [(12, dual.POSE_B), (8, dual.POSE_B), (4, dual.POSE_B)]),
    (4, [(0, dual.POSE_A), (12, dual.POSE_B), (8, dual.POSE_B)]),
    (8, [(0, dual.POSE_A), (4, dual.POSE_A), (12, dual.POSE_B)]),
    (12, [(0, dual.POSE_A), (4, dual.POSE_A), (8, dual.POSE_A)]),
    (16, []), (20, []),
])
def test_workspace_order_matches_firmware(joint, expected):
    assert dual.build_dual_workspace_plan(FakeIO(), joint) == expected


def test_dynamic_workspace_excludes_fully_offline_helpers():
    io = FakeIO()
    io.online &= ~sum(1 << motor for motor in config.joint_motors(8))
    mask = dual.build_dynamic_workspace_motor_mask(io, [0])
    expected = sum(1 << motor for joint in (0, 4, 12) for motor in config.joint_motors(joint))
    assert mask == expected
    assert dual.build_dual_workspace_plan(io, 0) == [(12, dual.POSE_B), (4, dual.POSE_B)]
    assert not io.commands


@pytest.mark.parametrize("failure", ["partial", "unknown", "stale", "target_offline"])
def test_dynamic_workspace_rejects_unusable_pair_without_motion(failure):
    io = FakeIO()
    if failure == "partial":
        io.online &= ~(1 << config.joint_motors(4)[0])
    elif failure == "unknown":
        io.unknown = 1 << config.joint_motors(4)[0]
    elif failure == "stale":
        io.age = config.kFeedbackReadyServoTimeoutMs + 1
    else:
        io.online &= ~sum(1 << motor for motor in config.joint_motors(0))
    assert dual.build_dynamic_workspace_motor_mask(io, [0]) is None
    assert io.diags[0][0] == protocol.CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED
    assert not io.commands


def test_unknown_pairs_can_be_discovered_for_coordinate_preparation_only():
    io = FakeIO()
    io.unknown = sum(1 << motor for motor in config.joint_motors(0) + config.joint_motors(4))
    assert dual.build_dynamic_workspace_motor_mask(io, [0]) is None
    assert dual.build_dynamic_workspace_motor_mask(io, [0], allow_unknown=True) is not None
    assert not io.commands


def test_thumb_scope_has_no_four_finger_read_mask():
    io = FakeIO()
    io.online = 0
    io.age = 10000
    assert dual.build_dynamic_workspace_motor_mask(io, [16, 20]) == 0


def test_unselected_finger_still_yields_and_is_still_required():
    io = FakeIO()
    io.selected_joint_mask = 1 << 8
    io.selected_motor_mask = sum(1 << m for m in config.joint_motors(8))
    expected_mask = sum(1 << motor for joint in (0, 4, 8, 12)
                        for motor in config.joint_motors(joint))
    assert dual.build_dynamic_workspace_motor_mask(io, [8]) == expected_mask
    assert dual.build_dual_workspace_plan(io, 8) == [
        (0, dual.POSE_A), (4, dual.POSE_A), (12, dual.POSE_B)]
    assert not io.commands


def test_unselected_helper_with_unknown_turns_rejects_workspace_prep():
    io = FakeIO()
    io.selected_joint_mask = 1 << 8
    io.selected_motor_mask = sum(1 << m for m in config.joint_motors(8))
    io.unknown = sum(1 << motor for motor in config.joint_motors(12))
    assert dual.build_dynamic_workspace_motor_mask(io, [8]) is None
    assert dual.build_dual_workspace_plan(io, 8) is None
    assert not io.commands


@pytest.mark.parametrize("target,selected_bit,expected", [
    (12, 1 << 12, [(0, dual.POSE_A), (4, dual.POSE_A), (8, dual.POSE_A)]),
    (8, 1 << 8, [(0, dual.POSE_A), (4, dual.POSE_A), (12, dual.POSE_B)]),
])
def test_partial_selection_yields_all_other_fingers(target, selected_bit, expected):
    io = FakeIO()
    io.selected_joint_mask = selected_bit
    io.selected_motor_mask = sum(1 << m for m in config.joint_motors(target))
    assert dual.build_dual_workspace_plan(io, target) == expected


@pytest.mark.parametrize("joint", [joint for joint in range(21)
                                  if joint not in dual.WORKSPACE_DUAL_JOINTS])
def test_non_workspace_joint_needs_no_helper_feedback_or_plan(joint):
    io = FakeIO()
    io.online = 0
    io.age = 10000
    io.snapshot = lambda: pytest.fail("non-workspace joint must not inspect helpers")
    assert dual.build_dynamic_workspace_motor_mask(io, [joint]) == 0
    assert dual.build_dual_workspace_plan(io, joint) == []
    assert not io.diags
    assert not io.commands


def test_route_conflict_helper_is_rejected_before_workspace_movement():
    io = FakeIO()
    snapshot = io.snapshot
    io.snapshot = lambda: dict(snapshot(), servo_route_conflict_bitmap=1 << config.joint_motors(4)[0])
    assert dual.build_dynamic_workspace_motor_mask(io, [0]) is None
    assert not io.commands


@pytest.mark.parametrize('target', dual.WORKSPACE_DUAL_JOINTS)
def test_side_sway_joints_keep_other_fingers_in_safe_order(target):
    io = FakeIO()
    expected = [(j, dual.POSE_A) for j in (0, 4, 8, 12) if j < target]
    expected += [(j, dual.POSE_B) for j in (12, 8, 4, 0) if j > target]
    assert dual.build_dual_workspace_plan(io, target) == expected
    assert dual.build_dynamic_workspace_motor_mask(io, [target]) == sum(
        1 << motor for joint in dual.WORKSPACE_DUAL_JOINTS
        for motor in config.joint_motors(joint))


@pytest.mark.parametrize("end,target_deg", [(sweep.END_MAX, 90.0), (sweep.END_MIN, 0.0)])
def test_sweep_reaches_end_by_absolute_angle(end, target_deg):
    io = FakeIO(start_raw=3500 if end == sweep.END_MAX else 5000)
    primary, secondary = config.joint_motors(16)
    io.on_publish = link(io, 16, primary, secondary)
    pull, release = (primary, secondary) if end == sweep.END_MAX else (secondary, primary)
    outcome = sweep.sweep_to_end(io, 16, end, motors=(primary, secondary),
                                 pull=pull, release=release)
    assert outcome.point is not None
    assert abs(outcome.point.angle - target_deg) <= config.kEndToleranceDeg


def test_sweep_load_alone_is_not_an_endpoint():
    """Regression: 2026-11-15 J00 hit load 190 while the joint sat at URDF Min."""
    io = FakeIO(start_raw=3500)
    primary, secondary = config.joint_motors(16)
    io.on_publish = freeze(io, 3500)
    io.loads[primary] = 192
    outcome = sweep.sweep_to_end(io, 16, sweep.END_MAX, motors=(primary, secondary),
                                 pull=primary, release=secondary)
    assert outcome.point is None
    assert outcome.reason == protocol.CALIB_DIAG_REASON_END_NOT_REACHED


def test_sweep_reports_distance_to_the_endpoint():
    io = FakeIO(start_raw=4100)
    primary, secondary = config.joint_motors(16)
    io.on_publish = freeze(io, 4100)
    io.loads[primary] = 192
    outcome = sweep.sweep_to_end(io, 16, sweep.END_MAX, motors=(primary, secondary),
                                 pull=primary, release=secondary)
    assert outcome.reason == protocol.CALIB_DIAG_REASON_END_NOT_REACHED
    angle_x1000, distance_x1000, encoder = io.diags[0][4], io.diags[0][5], io.diags[0][6]
    assert angle_x1000 == pytest.approx(round(36.0 * 1000), abs=1000)
    assert distance_x1000 > 50 * 1000
    assert encoder == 4100


def test_sweep_reports_slack_runaway():
    io = FakeIO(start_raw=3500)
    primary, secondary = config.joint_motors(16)
    io.on_publish = freeze(io, 3500)
    io.loads[primary] = 40
    outcome = sweep.sweep_to_end(io, 16, sweep.END_MAX, motors=(primary, secondary),
                                 pull=primary, release=secondary)
    assert outcome.point is None
    assert outcome.reason == protocol.CALIB_DIAG_REASON_SLACK_TOO_LARGE


def test_sweep_stops_when_feedback_vanishes():
    io = FakeIO()
    primary, secondary = config.joint_motors(16)
    io.online &= ~(1 << primary)
    outcome = sweep.sweep_to_end(io, 16, sweep.END_MAX, motors=(primary, secondary),
                                 pull=primary, release=secondary)
    assert outcome.point is None
    assert outcome.reason == protocol.CALIB_DIAG_REASON_FEEDBACK_LOST


def test_sweep_propagates_publish_failure():
    io = FakeIO()
    primary, secondary = config.joint_motors(16)
    io.reject_publish = True
    outcome = sweep.sweep_to_end(io, 16, sweep.END_MAX, motors=(primary, secondary),
                                 pull=primary, release=secondary)
    assert outcome.point is None
    assert outcome.reason == protocol.CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED


def test_pose_helper_skips_a_helper_already_at_the_end():
    io = FakeIO()
    io.encoder_values[4] = 5000
    assert machine(io, 0)._pose_helper(4, dual.POSE_B) is True
    assert not io.commands


@pytest.mark.parametrize("pose,end_raw", [(dual.POSE_A, 3500), (dual.POSE_B, 5000)])
def test_pose_helper_drives_the_helper_to_its_end(pose, end_raw):
    io = FakeIO()
    io.encoder_values[4] = 5000 if end_raw == 3500 else 3500
    primary, secondary = config.joint_motors(4)
    io.on_publish = link(io, 4, primary, secondary)
    assert machine(io, 0)._pose_helper(4, pose) is True
    assert abs(io.encoder_values[4] - end_raw) <= 8


def test_pose_helper_records_the_moved_helper_for_the_finish_pull():
    io = FakeIO()
    io.encoder_values[4] = 3500
    primary, secondary = config.joint_motors(4)
    io.on_publish = link(io, 4, primary, secondary)
    assert machine(io, 0)._pose_helper(4, dual.POSE_B) is True
    assert io.workspace_helpers == [4]


def test_pose_helper_skip_leaves_the_helper_out_of_the_finish_pull():
    io = FakeIO()
    io.encoder_values[4] = 5000
    assert machine(io, 0)._pose_helper(4, dual.POSE_B) is True
    assert io.workspace_helpers == []


def test_pose_helper_failure_reports_workspace_diagnostic():
    io = FakeIO()
    io.encoder_values[4] = 4200
    primary, secondary = config.joint_motors(4)
    io.on_publish = freeze(io, 4200)
    io.loads[primary] = 192
    io.loads[secondary] = 192
    worker = machine(io, 0)
    assert worker._pose_helper(4, dual.POSE_B) is False
    assert worker.reason in (protocol.CALIB_DIAG_REASON_END_NOT_REACHED,
                            protocol.CALIB_DIAG_REASON_SLACK_TOO_LARGE)


def test_run_dual_fails_when_the_joint_cannot_reach_max():
    io = FakeIO(start_raw=3500)
    primary, secondary = config.joint_motors(16)
    io.on_publish = freeze(io, 3500)
    io.loads[primary] = 192
    outcome = dual.run_dual(io, 16, 0, 1, 1 << 16, config.AutoCalibrationParams())
    assert outcome["success"] is False
    assert outcome["reason"] == protocol.CALIB_DIAG_REASON_END_NOT_REACHED


def test_run_dual_completes_with_lut_and_limits():
    io = FakeIO()
    primary, secondary = config.joint_motors(16)
    io.on_publish = link(io, 16, primary, secondary)
    outcome = dual.run_dual(io, 16, 0, 1, 1 << 16, config.AutoCalibrationParams())
    assert outcome["success"] is True
    result = outcome["result"]
    assert result["success"] is True
    assert result["tendonLutValid"] is True
    assert result["tendonLutCount"] >= 3
    assert result["encoderSpan"] == pytest.approx(1500, abs=8)
    assert set(outcome["limits"]) == {primary, secondary}
    assert io.limits == outcome["limits"]


def test_run_dual_preflight_failure_keeps_status():
    io = FakeIO()
    io.online &= ~(1 << config.joint_motors(4)[0])
    outcome = dual.run_dual(io, 0, 0, 1, 1 << 0, config.AutoCalibrationParams())
    assert outcome["success"] is False
    assert outcome["preflight_failed"] is True
    assert not io.commands


def test_j00_2026_11_15_slack_then_move_reaches_max():
    """Replay of the failed run: 3400 counts of slack, load 190 before the joint moves."""
    io = FakeIO(low_raw=22, high_raw=4187, start_raw=48)
    primary, secondary = config.joint_motors(0)
    state = {"raw": 48.0}

    def hook(st, targets, previous):
        travel = targets.get(primary, previous.get(primary, 0))
        if travel > 3400:
            state["raw"] = min(4187, 48 + (travel - 3400))
        st.encoder_values[0] = state["raw"]
        st.loads[primary] = 192 if travel > 3200 else 120
        if secondary in targets:
            st.loads[secondary] = 80

    io.on_publish = hook
    outcome = sweep.sweep_to_end(io, 0, sweep.END_MAX, motors=(primary, secondary),
                                 pull=primary, release=secondary)
    assert outcome.point is not None
    assert outcome.point.angle > 14.0


def test_real_j00_geometry_full_calibration():
    """Manual 22->4187, URDF -75..16: span and LUT must match the field result (4167)."""
    io = FakeIO(joint=16, low_raw=22, high_raw=4187, start_raw=48)
    io.urdf_limits = lambda joint: (-75.0, 16.0)
    primary, secondary = config.joint_motors(16)
    io.on_publish = link(io, 16, primary, secondary)
    outcome = dual.run_dual(io, 16, 0, 1, 1 << 16, config.AutoCalibrationParams())
    assert outcome["success"] is True
    result = outcome["result"]
    assert result["encoderSpan"] == pytest.approx(4165, abs=4)
    assert result["totalDeg"] == pytest.approx(91.52, abs=0.2)
    assert result["tendonLutValid"] is True
    assert result["tendonLutCount"] == 20


def test_sweep_commands_counts_by_elapsed_time():
    """Regression: a fixed 8 counts per loop halved the real speed (publish costs ~100ms)."""
    class SlowPublishIO(FakeIO):
        def publish(self, targets, speed, acc):
            ok = super().publish(targets, speed, acc)
            self.t += 80
            return ok

    io = SlowPublishIO(start_raw=3500)
    primary, secondary = config.joint_motors(16)
    io.on_publish = link(io, 16, primary, secondary)
    sweep.sweep_to_end(io, 16, sweep.END_MAX, motors=(primary, secondary),
                       pull=primary, release=secondary)
    deltas = [b[1][primary] - a[1][primary]
              for a, b in zip(io.commands, io.commands[1:])
              if primary in a[1] and primary in b[1]]
    # 100 ms 一次循环 × 400 counts/s → 每次 40 counts（固定步长只有 8）。
    assert max(deltas) == pytest.approx(40, abs=2)


def test_sweep_keeps_going_while_the_joint_creeps():
    """Regression: J01/J02 were declared stuck while still creeping at 0.2 deg per window."""
    io = FakeIO(low_raw=3500, high_raw=5000, start_raw=3500)
    motor = config.joint_motors(1)[0]
    pose = {"raw": 3500}

    def creep(state, targets, previous):
        pose["raw"] = min(5000, pose["raw"] + 1)
        state.encoder_value = pose["raw"]
        state.loads[motor] = 150

    io.on_publish = creep
    outcome = sweep.sweep_to_end(io, 1, sweep.END_MAX, motors=(motor,),
                                 pull=motor, release=None)
    assert outcome.point is not None
    assert abs(outcome.point.angle - 90.0) <= config.kEndToleranceDeg
