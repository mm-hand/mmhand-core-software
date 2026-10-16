"""Dual-port trajectory checks against the original P4 branch predicates."""

import pytest

from desktop.core import protocol
from desktop.core.host_calibration import config, dual_motor as dual
from desktop.core.host_calibration.math import (
    TendonRawFitBuffer, manual_entry_crc, wrap_encoder_delta,
)
from desktop.core.host_calibration.time_alignment import TimeAlignmentError


class FakeIO:
    def __init__(self, joint=16):
        self.t = 0
        self.stop_at = 10000
        self.values = {motor: 0 for motor in range(27)}
        self.loads = {motor: 190 for motor in range(27)}
        self.encoder_value = 5000
        self.directions = {joint: config.default_encoder_direction(joint) for joint in range(21)}
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

    def now_ms(self):
        return self.t

    def sleep_ms(self, ms):
        self.t += ms
        if self.on_sleep:
            self.on_sleep(self)

    def owns_control(self):
        return self.t < self.stop_at

    def encoder(self, joint):
        return self.encoder_value

    def encoder_raw(self, joint):
        value = self.encoder(joint)
        return None if value is None else (-value if self.direction(joint) < 0 else value) % 16384

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
        self.commands.append((self.t, dict(targets), speed, acc))
        self.values.update(targets)
        if self.on_publish:
            self.on_publish(self, targets)
        return True

    def diag(self, *values):
        self.diags.append(values)

    def trace(self, *values, **details):
        self.traces.append((values, details))

    def snapshot(self):
        return {"servo_feedback_age_ms": self.age,
                "servo_online_bitmap": self.online,
                "multi_turn_unknown_bitmap": self.unknown,
                "servo_multiturn_configured_bitmap": self.configured,
                "servo_recovery_pending_bitmap": self.pending,
                "servo_recovery_failed_bitmap": self.failed}

    def clear_limits(self, joint):
        for motor in config.joint_motors(joint):
            self.limits.pop(motor, None)

    def set_limit(self, motor, minimum, maximum):
        self.limits[motor] = (minimum, maximum)

    def direction(self, joint):
        return self.directions[joint]

    def set_direction(self, joint, direction):
        self.directions[joint] = direction

    def manual_reference(self, joint):
        return {"valid": True, "minRaw": (16384 - 3500) % 16384,
                "maxRaw": (16384 - 5000) % 16384}

    def urdf_limits(self, joint):
        return 0.0, 90.0

    def urdf_config_crc(self):
        return 7


def machine(io, joint=16):
    return dual._DualMachine(io, joint, config.AutoCalibrationParams())


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


@pytest.mark.parametrize("gate", ["configured", "pending", "failed", "ready"])
def test_target_unknown_rebuild_requires_original_recovery_gates(gate):
    io = FakeIO()
    pair = sum(1 << motor for motor in config.joint_motors(0))
    io.unknown = pair
    if gate == "configured":
        io.configured &= ~pair
    elif gate == "pending":
        io.pending = pair
    elif gate == "failed":
        io.failed = pair
    result = dual.build_dynamic_workspace_motor_mask(io, [0])
    assert (result is not None) == (gate == "ready")
    assert not io.commands


def test_thumb_scope_has_no_four_finger_read_mask():
    io = FakeIO()
    io.online = 0
    io.age = 10000
    assert dual.build_dynamic_workspace_motor_mask(io, [16, 20]) == 0


def test_unselected_online_unknown_finger_is_neither_required_nor_moved():
    io = FakeIO()
    io.selected_joint_mask = 1 << 8
    io.unknown = sum(1 << motor for motor in config.joint_motors(12))
    assert dual.build_dynamic_workspace_motor_mask(io, [8]) == sum(
        1 << motor for motor in config.joint_motors(8))
    assert dual.build_dual_workspace_plan(io, 8) == []
    assert not io.commands


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


def test_helper_preflight_finishes_before_any_workspace_movement():
    io = FakeIO()
    # Last helper in the J00 plan lacks load, after two valid earlier helpers.
    io.loads[config.joint_motors(4)[1]] = None
    worker = machine(io, 0)
    assert not worker.prepare_workspace()
    assert io.t == config.kInitialFeedbackWaitMs
    assert io.commands == []
    assert worker.reason == protocol.CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED


@pytest.mark.parametrize("params", [config.AutoCalibrationParams(),
                                    config.AutoCalibrationParams(tightenSpeedCountsPerSec=200,
                                                                 servoSpeed=200, windowMs=400)])
def test_workspace_fresh_pose_releases_antagonist_before_tightening(params):
    io = FakeIO()
    worker = dual._DualMachine(io, 0, params)
    primary, secondary = config.joint_motors(4)
    assert worker._ensure_workspace_pose(4, dual.POSE_A)
    assert all(speed == 400 and acc == 20
               for _, _, speed, acc in io.commands)
    first = io.commands[0][1]
    assert first == {primary: -3200, secondary: 0}
    entry = dual._cache(io)[4]
    assert entry.valid and entry.current_pose == dual.POSE_A
    before = len(io.commands)
    # Instantaneous load may become zero once the measured posture is reached.
    io.loads[secondary] = 0
    assert worker._ensure_workspace_pose(4, dual.POSE_A)
    assert len(io.commands) == before


def test_custom_slow_calibration_preserves_workspace_tightening_rate():
    io = FakeIO()
    params = config.AutoCalibrationParams(tightenSpeedCountsPerSec=200,
                                          servoSpeed=200, windowMs=400)
    worker = dual._DualMachine(io, 0, params)
    primary, secondary = config.joint_motors(4)
    io.loads[secondary] = 0

    def load_after_tightening(state, targets):
        if targets.get(secondary, 0) >= 16:
            state.loads[secondary] = config.kWorkspaceTightLoadThreshold

    io.on_publish = load_after_tightening
    assert worker._ensure_workspace_pose(4, dual.POSE_A)
    assert io.commands[0][1] == {primary: -3200, secondary: 0}
    tightening = [targets[secondary] for _, targets, _, _ in io.commands
                  if targets[secondary] > 0]
    assert tightening[:2] == [8, 16]
    assert all(speed == 400 and acc == 20 for _, _, speed, acc in io.commands)


def test_workspace_cache_same_pose_drift_rebuilds_from_actual_abs():
    io = FakeIO()
    worker = machine(io, 0)
    primary, secondary = config.joint_motors(4)
    assert worker._ensure_workspace_pose(4, dual.POSE_A)
    io.values[primary] += config.kWorkspaceTargetToleranceCounts + 1
    actual = io.values[primary]
    before = len(io.commands)
    assert worker._ensure_workspace_pose(4, dual.POSE_A)
    assert io.commands[before][1][primary] == actual - config.kWorkspaceReleaseCounts


def test_workspace_a_to_b_and_back_use_cached_release_then_tighten():
    io = FakeIO()
    worker = machine(io, 0)
    primary, secondary = config.joint_motors(4)
    assert worker._ensure_workspace_pose(4, dual.POSE_A)
    before = len(io.commands)
    assert worker._ensure_workspace_pose(4, dual.POSE_B)
    assert io.commands[before][1] == {primary: -3200, secondary: -3200}
    before = len(io.commands)
    assert worker._ensure_workspace_pose(4, dual.POSE_A)
    assert io.commands[before][1] == {primary: -6400, secondary: -3200}


def test_workspace_direct_release_invalid_hardware_range_stops_before_publish():
    io = FakeIO()
    primary = config.joint_motors(4)[0]
    io.ranges[primary] = (-100, 100)
    worker = machine(io, 0)
    assert not worker._ensure_workspace_pose(4, dual.POSE_A)
    assert not io.commands
    assert worker.reason == protocol.CALIB_DIAG_REASON_ABS_LIMIT_INVALID


def test_reset_and_origin_change_only_invalidate_corresponding_pose():
    io = FakeIO()
    cache = dual._cache(io)
    cache[0].valid = cache[4].valid = True
    dual.invalidate_dual_workspace_cache_joint(io, 0)
    assert not dual._cache(io)[0].valid and dual._cache(io)[4].valid
    dual.reset_dual_workspace_cache(io)
    assert not any(entry.valid for entry in dual._cache(io).values())


@pytest.mark.parametrize("params,step,endpoint", [
    (config.AutoCalibrationParams(), 8, 152),
    (config.default_auto_calibration_params(16), 8, 152),
])
def test_primary_tight_requires_load_motion_and_two_stalled_windows(params, step, endpoint):
    io = FakeIO()
    worker = dual._DualMachine(io, 16, params)
    result = worker._find_tight(20, 0, 5000, 190)
    assert result is not None
    assert io.t == params.windowMs * config.kAngleStallStableSamples
    assert result[0] == endpoint
    assert all(speed == params.servoSpeed and acc == params.servoAcc
               for _, _, speed, acc in io.commands)
    assert [targets[20] for _, targets, _, _ in io.commands[:3]] == [step, 2 * step, 3 * step]
    assert io.commands[-1][1][20] == result[0]


def test_primary_tight_load_alone_does_not_replace_motion_condition():
    io = FakeIO()
    io.stop_at = 600
    # Servo feedback never moves, despite outgoing monotonically increasing commands.
    io.on_publish = lambda state, targets: state.values.update({20: 0})
    worker = machine(io)
    assert worker._find_tight(20, 0, 5000, 190) is None
    assert worker.reason == protocol.CALIB_DIAG_REASON_CONTROL_LOST


def test_primary_tight_stall_sample_counter_resets_on_angle_progress():
    io = FakeIO()
    io.on_sleep = lambda state: setattr(state, "encoder_value", 5100 if state.t >= 400 else 5000)
    worker = machine(io)
    assert worker._find_tight(20, 0, 5000, 190) is not None
    # 200ms stalled, 400ms moved(reset), then 600 and 800ms stalled.
    assert io.t == 800


@pytest.mark.parametrize("load,expected", [(799, True), (800, False), (-800, False), (1024 + 190, True)])
def test_batch_overload_uses_feetech_ten_bit_magnitude(load, expected):
    io = FakeIO()
    io.loads[20] = load
    worker = machine(io)
    assert worker._publish({20: 8}, protocol.CALIB_DIAG_PHASE_FIND_MAX, 16, 20) is expected
    if not expected:
        assert worker.reason == protocol.CALIB_DIAG_REASON_OVERLOAD
        assert not io.commands


def test_preload_accepts_load_without_encoder_stall_or_extra_motion():
    io = FakeIO()
    worker = machine(io)
    result = worker._find_secondary_preload(21, 0, 20, 120)
    assert result == (0, 11384, 5000, 190)
    assert io.t == 0
    assert io.commands[-1][1] == {21: 0, 20: 120}


@pytest.mark.parametrize("params,step,endpoint", [
    (config.AutoCalibrationParams(), 8, 72),
    (config.default_auto_calibration_params(16), 8, 72),
])
def test_exchange_endpoint_requires_12_degrees_and_both_motors_progress_not_angle_stall(
        params, step, endpoint):
    io = FakeIO()
    io.loads[21] = 250
    io.on_sleep = lambda state: setattr(state, "encoder_value", 5000 - 3 * state.t)
    fit = TendonRawFitBuffer()
    fit.append(5000, 0, 0)
    fit.next_travel_deg = 5.0
    worker = dual._DualMachine(io, 16, params)
    result = worker._find_exchange(21, 0, 20, 0, 5000, fit)
    assert result is not None and io.t == params.windowMs
    assert result[3] == 5000 - 3 * params.windowMs
    assert io.commands[0][1] == {21: step, 20: -step}
    assert len(fit.samples) >= 3
    assert result[0] == endpoint and result[1] == -endpoint


@pytest.mark.parametrize("stage", ["tight", "exchange"])
def test_side_sway_sub_fifty_count_feedback_does_not_accept_endpoint(stage):
    io = FakeIO(joint=12)
    params = config.default_auto_calibration_params(12)
    primary, secondary = config.joint_motors(12)
    io.stop_at = 1000
    worker = dual._DualMachine(io, 12, params)

    def noisy_feedback(state, _targets):
        # 在窗口边界形成 36 counts 的跳变，能通过旧 25 门禁，低于原 50 门禁。
        noise = 36 if (state.t // params.windowMs) % 2 == 0 else 0
        state.values[primary] = -noise
        state.values[secondary] = noise

    io.on_publish = noisy_feedback
    if stage == "tight":
        result = worker._find_tight(primary, 0, 5000, 190)
    else:
        io.loads[secondary] = 250
        io.on_sleep = lambda state: setattr(state, "encoder_value", 5000 - 3 * state.t)
        result = worker._find_exchange(secondary, 0, primary, 0, 5000,
                                       TendonRawFitBuffer())
    assert result is None
    assert worker.reason == protocol.CALIB_DIAG_REASON_CONTROL_LOST
    checks = [details for values, details in io.traces
              if values[0] == ("CHECK_POS80" if stage == "tight" else
                               "CHECK_DUAL_EXCHANGE_NEG3")]
    assert checks
    assert all(details["window_abs" if stage == "tight" else
                       "secondary_delta"] < 50 for details in checks)


def test_exchange_loaded_but_insufficient_encoder_travel_fails_after_two_stalled_windows():
    io = FakeIO()
    io.loads[21] = 250
    worker = machine(io)
    assert worker._find_exchange(21, 0, 20, 0, 5000, TendonRawFitBuffer()) is None
    assert io.t == 400
    assert worker.reason == protocol.CALIB_DIAG_REASON_DUAL_EXCHANGE_STALLED
    assert io.commands[-1][1][21] > 0


def test_exchange_progress_of_one_motor_is_insufficient():
    io = FakeIO()
    io.loads[21] = 250
    io.stop_at = 600
    io.on_sleep = lambda state: setattr(state, "encoder_value", 5000 - 3 * state.t)
    io.on_publish = lambda state, targets: state.values.update({20: 0})
    worker = machine(io)
    assert worker._find_exchange(21, 0, 20, 0, 5000, TendonRawFitBuffer()) is None
    assert worker.reason == protocol.CALIB_DIAG_REASON_CONTROL_LOST


@pytest.mark.parametrize("stage", ["tight", "preload", "exchange"])
def test_feedback_loss_is_root_reason(stage):
    io = FakeIO()
    io.encoder_value = None
    worker = machine(io)
    if stage == "tight":
        outcome = worker._find_tight(20, 0, 5000, 190)
    elif stage == "preload":
        outcome = worker._find_secondary_preload(21, 0, 20, 0)
    else:
        outcome = worker._find_exchange(21, 0, 20, 0, 5000, TendonRawFitBuffer())
    assert outcome is None
    assert worker.reason == protocol.CALIB_DIAG_REASON_FEEDBACK_LOST
    assert not io.commands


def test_complete_thumb_dual_trace_releases_then_searches_preloads_exchanges_and_builds_lut():
    io = FakeIO()
    io.loads[21] = 0
    observed = {"preload_abs": None, "exchange": False}

    def simulate(state, targets):
        if 21 not in targets or 20 not in targets:
            return
        if targets[20] < 152:
            if observed["preload_abs"] is None:
                observed["preload_abs"] = targets[21] - 8
            observed["exchange"] = True
            state.loads[21] = 250
            state.encoder_value = 5000 - (targets[21] - observed["preload_abs"]) * 10
        elif targets[21] >= -3120:
            state.loads[21] = 190

    io.on_publish = simulate
    outcome = dual.run_dual(io, 16, 0, 1, 1 << 16, config.AutoCalibrationParams())
    assert outcome["success"] and not outcome["preflight_failed"]
    assert observed["exchange"]
    assert io.commands[0][1] == {21: -3200}
    assert outcome["result"]["mappingVersion"] == 4
    assert outcome["result"]["tendonLutValid"]
    assert outcome["result"]["tendonLutMotor"] == [20, 21]
    assert outcome["result"]["encoderSpan"] >= 546
    assert io.values[20] < 152  # No extra primary tightening after exchange.
    assert set(outcome["limits"]) == {20, 21}
    assert outcome["limits"][20] == (152 - 20480, 152 + 900)
    assert io.limits == outcome["limits"]


def test_target_b_cache_reuse_uses_abs_and_accepts_zero_instantaneous_load(monkeypatch):
    io = FakeIO(joint=0)
    primary, secondary = config.joint_motors(0)
    io.values[primary], io.values[secondary] = 120, -3200
    io.loads[primary] = 0
    dual._cache(io)[0] = dual.WorkspaceCacheEntry(
        valid=True, current_pose=dual.POSE_B, primary_motor=primary,
        secondary_motor=secondary, primary_tight_abs=120, secondary_release_abs=-3200)
    worker = machine(io, 0)
    monkeypatch.setattr(worker, "_wait_motor_target", lambda *args: pytest.fail("duplicate slack"))
    monkeypatch.setattr(worker, "_find_tight", lambda *args: pytest.fail("duplicate POS80"))
    # Stop after checking that the preload starts at the cached primary hold.
    def stop(secondary_motor, start_abs, primary_motor, hold_abs):
        assert (secondary_motor, start_abs, primary_motor, hold_abs) == (secondary, -3200, primary, 120)
        return None
    monkeypatch.setattr(worker, "_find_secondary_preload", stop)
    assert worker.calibrate() is None
    assert not io.commands


@pytest.mark.parametrize("span,expected_success", [(0, False), (15360, False), (700, True)])
def test_dual_result_rejects_zero_and_near_full_wrap_spans(monkeypatch, span, expected_success):
    io = FakeIO()
    worker = machine(io)
    monkeypatch.setattr(worker, "_wait_motor_target", lambda *args: True)
    monkeypatch.setattr(worker, "_find_tight", lambda *args: (120, 11384, 5000, 190))
    monkeypatch.setattr(worker, "_find_secondary_preload", lambda *args: (20, 11384, 5000, 190))
    endpoint = (5000 - span) % 16384

    def exchange(*_args):
        # A genuine near-full turn needs consecutive feedback, not two points
        # whose shortest difference would instead describe a 1024-count move.
        for moved in range(4000, span, 4000):
            worker._result_travel.observe((5000 - moved) % 16384)
        worker._result_travel.observe(endpoint)
        return 80, -60, (-endpoint) % 16384, endpoint, 250

    monkeypatch.setattr(worker, "_find_exchange", exchange)
    result = worker.calibrate()
    assert (result is not None) is expected_success
    if not expected_success:
        assert worker.reason == protocol.CALIB_DIAG_REASON_RESULT_SPAN_INVALID
        assert io.limits == {}


@pytest.mark.parametrize("joint,tight,limit,manual_span", [
    (16, 9688, 10962, 4108),  # Supplied J16 log: formerly 15110 / 332 degrees.
    (20, 5595, 8307, 1476),  # Supplied J20 log: formerly 13672 / 300 degrees.
    (16, 5000, 4300, 1500),  # Original negative exchange direction.
    (16, 5000, 5700, 1500),
    (16, 16000, 400, 1500),  # Positive exchange across magnetic zero.
    (16, 400, 16000, 1500),  # Negative exchange across magnetic zero.
])
def test_dual_orders_result_endpoints_without_flipping_manual_or_lut_samples(
        monkeypatch, joint, tight, limit, manual_span):
    io = FakeIO(joint)
    io.encoder_value = tight
    worker = machine(io, joint)
    primary, secondary = config.joint_motors(joint)
    signed_travel = wrap_encoder_delta(limit, tight)
    start, end = (tight, limit) if signed_travel > 0 else (limit, tight)
    manual = {"valid": True, "minRaw": (-start) % 16384,
              "maxRaw": (-(start + manual_span)) % 16384}
    io.manual_reference = lambda _joint: dict(manual)
    manual_crc = manual_entry_crc(joint, manual)
    fit = TendonRawFitBuffer()
    fit.append(tight, 120, 20)
    fit.append((tight + signed_travel // 2) % 16384, 30, 50)
    fit.append(limit, -60, 80)
    original_samples = [(s.encoder, s.primary_abs, s.secondary_abs) for s in fit.samples]
    window = object()
    io.begin_lut_sampling = lambda *_args: window
    io.finish_lut_sampling = lambda value: fit if value is window else pytest.fail("wrong window")
    monkeypatch.setattr(worker, "_wait_motor_target", lambda *_args: True)
    monkeypatch.setattr(worker, "_find_tight", lambda *_args: (120, (-tight) % 16384, tight, 190))
    monkeypatch.setattr(worker, "_find_secondary_preload",
                        lambda *_args: (20, (-tight) % 16384, tight, 190))
    monkeypatch.setattr(worker, "_find_exchange",
                        lambda *_args: (80, -60, (-limit) % 16384, limit, 250))
    original_builder = dual.build_tendon_lut

    def build(calib_io, result_joint, result, p, s, count, raw):
        assert raw is fit
        assert [(v.encoder, v.primary_abs, v.secondary_abs) for v in raw.samples] == original_samples
        return original_builder(calib_io, result_joint, result, p, s, count, raw)

    monkeypatch.setattr(dual, "build_tendon_lut", build)
    outcome = worker.calibrate()
    assert outcome is not None
    result = outcome["result"]
    assert (result["startEncoder"], result["tightEncoder"]) == (start, end)
    assert result["encoderSpan"] == abs(signed_travel)
    assert result["autoSpanCounts"] == abs(signed_travel)
    assert result["totalDeg"] == pytest.approx(abs(signed_travel) * 360 / 16384)
    assert (result["autoMinRaw"], result["autoMaxRaw"]) == ((-start) % 16384, (-end) % 16384)
    assert result["zeroEncoder"] == (start + 228) % 16384
    assert result["encoderDirection"] == io.direction(joint) == -1
    assert result["manualSpanCounts"] == manual_span
    assert result["manualReferenceCrc"] == manual_crc
    assert result["tendonLutValid"]
    assert result["tendonLutMotor"] == [primary, secondary]
    assert result["startServoAbs"] == (120 if signed_travel > 0 else -60)
    assert result["tightServoAbs"] == (-60 if signed_travel > 0 else 120)
    assert outcome["limits"][primary] == (120 - 20480, 120 + 900)
    assert outcome["limits"][secondary] == (80 - 20480, 80 + 900)
    assert protocol.build_calib_load_joint_cmd(result)


@pytest.mark.parametrize("limit,expected_span", [(6200, 1200), (5000, 0)])
def test_dual_keeps_primary_endpoint_when_preload_moves_joint(monkeypatch, limit, expected_span):
    io = FakeIO()
    worker = machine(io)
    monkeypatch.setattr(worker, "_wait_motor_target", lambda *_args: True)
    monkeypatch.setattr(worker, "_find_tight", lambda *_args: (120, 11384, 5000, 190))

    def preload(*_args):
        io.encoder_value = 5600
        return 20, 10784, 5600, 190

    monkeypatch.setattr(worker, "_find_secondary_preload", preload)
    monkeypatch.setattr(worker, "_find_exchange",
                        lambda *_args: (80, -60, (-limit) % 16384, limit, 250))
    outcome = worker.calibrate()
    checks = [details for values, details in io.traces if values[0] == "DUAL_SPAN_CHECK"]
    assert len(checks) == 1
    assert checks[0]["preload_travel"] == 600
    assert checks[0]["exchange_travel"] == limit - 5600
    assert checks[0]["span"] == expected_span
    if expected_span:
        assert outcome is not None
        assert outcome["result"]["encoderSpan"] == expected_span
        assert outcome["result"]["startEncoder"] == 5000
    else:
        assert outcome is None
        assert worker.reason == protocol.CALIB_DIAG_REASON_RESULT_SPAN_INVALID
        assert not io.limits


@pytest.mark.parametrize("direction,final_travel", [(-1, 15360), (1, 15360), (-1, 700), (1, 700)])
def test_dual_rejects_observed_near_full_turn_even_if_motion_returns(
        monkeypatch, direction, final_travel):
    io = FakeIO()
    worker = machine(io)
    monkeypatch.setattr(worker, "_wait_motor_target", lambda *_args: True)
    monkeypatch.setattr(worker, "_find_tight", lambda *_args: (120, 11384, 5000, 190))
    monkeypatch.setattr(worker, "_find_secondary_preload", lambda *_args: (20, 11384, 5000, 190))

    def exchange(*_args):
        for moved in [4000, 8000, 12000, 15360]:
            worker._result_travel.observe((5000 + direction * moved) % 16384)
        if final_travel != 15360:
            for moved in [12000, 8000, 4000, final_travel]:
                worker._result_travel.observe((5000 + direction * moved) % 16384)
        endpoint = (5000 + direction * final_travel) % 16384
        return 80, -60, (-endpoint) % 16384, endpoint, 250

    monkeypatch.setattr(worker, "_find_exchange", exchange)
    assert worker.calibrate() is None
    assert worker.reason == protocol.CALIB_DIAG_REASON_RESULT_SPAN_INVALID
    check = [details for values, details in io.traces if values[0] == "DUAL_SPAN_CHECK"][0]
    assert check["span"] == final_travel
    assert check["observed_span"] == 15360
    assert not io.limits


@pytest.mark.parametrize("direction", [-1, 1])
def test_exchange_uses_continuous_feedback_travel_past_half_turn(direction):
    io = FakeIO()
    io.loads[21] = 250
    # 1000 counts per feedback step, 10000 counts per 200ms window.
    io.on_sleep = lambda state: setattr(state, "encoder_value", (5000 + direction * 50 * state.t) % 16384)
    worker = machine(io)
    result = worker._find_exchange(21, 0, 20, 0, 5000, None)
    assert result is not None
    assert io.t == 200
    checks = [details for values, details in io.traces if values[0] == "CHECK_DUAL_EXCHANGE_NEG3"]
    assert checks[-1]["signed_travel"] == direction * 10000
    assert checks[-1]["total_deg"] == pytest.approx(10000 * 360 / 16384)


@pytest.mark.parametrize("outside,cache_valid", [(-25, False), (-24, True), (0, True)])
def test_outside_ready_result_is_retained_but_only_in_window_pose_is_cached(monkeypatch, outside, cache_valid):
    io = FakeIO(joint=0)
    worker = machine(io, 0)
    monkeypatch.setattr(worker, "_wait_motor_target", lambda *args: True)
    monkeypatch.setattr(worker, "_find_tight", lambda *args: (120, 11384, 5000, 190))
    monkeypatch.setattr(worker, "_find_secondary_preload", lambda *args: (20, 11384, 5000, 190))
    monkeypatch.setattr(worker, "_find_exchange", lambda *args: (80, -60, 12084, 4300, 250))
    monkeypatch.setattr(dual, "map_manual_scaled_encoder_to_urdf",
                        lambda *args: {"valid": True, "outside_counts": outside,
                                       "urdf_deg": -0.5, "feedback_ready": False})
    result = worker.calibrate()
    assert result is not None
    assert result["requires_rope_init"] is (not cache_valid)
    entry = dual._cache(io)[0]
    assert entry.valid is cache_valid
    if cache_valid:
        assert entry.current_pose == dual.POSE_A
        assert (entry.primary_release_abs, entry.secondary_tight_abs) == (-60, 80)


def test_ready_map_invalid_rejects_result_before_any_limit_commit(monkeypatch):
    io = FakeIO(joint=0)
    worker = machine(io, 0)
    monkeypatch.setattr(worker, "_wait_motor_target", lambda *args: True)
    monkeypatch.setattr(worker, "_find_tight", lambda *args: (120, 11384, 5000, 190))
    monkeypatch.setattr(worker, "_find_secondary_preload", lambda *args: (20, 11384, 5000, 190))
    monkeypatch.setattr(worker, "_find_exchange", lambda *args: (80, -60, 12084, 4300, 250))
    monkeypatch.setattr(dual, "map_manual_scaled_encoder_to_urdf", lambda *args: None)
    assert worker.calibrate() is None
    assert worker.reason == protocol.CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED
    assert io.limits == {}


def test_dual_lut_build_failure_keeps_original_mechanical_commit_branch(monkeypatch):
    io = FakeIO(joint=0)
    worker = machine(io, 0)
    monkeypatch.setattr(worker, '_wait_motor_target', lambda *args: True)
    monkeypatch.setattr(worker, '_find_tight', lambda *args: (120, 11384, 5000, 190))
    monkeypatch.setattr(worker, '_find_secondary_preload', lambda *args: (20, 11384, 5000, 190))
    monkeypatch.setattr(worker, '_find_exchange', lambda *args: (80, -60, 12084, 4300, 250))
    original_builder = dual.build_tendon_lut
    monkeypatch.setattr(dual, 'build_tendon_lut',
                        lambda calib_io, joint, result, primary, secondary, count, raw:
                        original_builder(calib_io, joint, result, primary, secondary,
                                         count, dual.TendonRawFitBuffer()))
    monkeypatch.setattr(dual, 'map_manual_scaled_encoder_to_urdf',
                        lambda *args: {'valid': True, 'outside_counts': 0,
                                       'urdf_deg': 0, 'feedback_ready': True})
    outcome = worker.calibrate()
    assert outcome is not None
    assert outcome['result']['success']
    assert not outcome['result']['tendonLutValid']
    assert outcome['result']['tendonLutCount'] == 0
    assert len(outcome['limits']) == 2


@pytest.mark.parametrize('fail', [False, True])
def test_dual_exchange_replaces_complete_fit_and_rejects_alignment_failure(monkeypatch, fail):
    io = FakeIO()
    worker = machine(io)
    events = []
    window = object()
    fit = TendonRawFitBuffer()
    for sample in ((4300, 700, 200), (4650, 800, 150), (5000, 900, 100)):
        fit.append(*sample)
    monkeypatch.setattr(worker, '_wait_motor_target', lambda *_: True)
    monkeypatch.setattr(worker, '_find_tight', lambda *_: (120, 11384, 5000, 190))

    def preload(*_args):
        events.append('preload')
        return 20, 11384, 5000, 190

    def begin(joint, motors):
        assert (joint, motors) == (16, (20, 21))
        events.append('begin')
        return window

    def exchange(*args):
        assert args[-1] is None  # Independent getters cannot enter the buffer.
        events.append('exchange')
        return 80, -60, 12084, 4300, 250

    def finish(value):
        assert value is window
        events.append('finish')
        if fail:
            raise TimeAlignmentError('raw/ABS coverage missing')
        return fit

    monkeypatch.setattr(worker, '_find_secondary_preload', preload)
    monkeypatch.setattr(worker, '_find_exchange', exchange)
    io.begin_lut_sampling, io.finish_lut_sampling = begin, finish
    original_builder = dual.build_tendon_lut

    def build(calib_io, joint, result, primary, secondary, count, raw):
        assert raw is fit and raw.count == 3
        assert [(s.primary_abs, s.secondary_abs) for s in raw.samples] == [(700, 200), (800, 150), (900, 100)]
        return original_builder(calib_io, joint, result, primary, secondary, count, raw)

    monkeypatch.setattr(dual, 'build_tendon_lut', build)
    result = worker.calibrate()
    assert events == ['preload', 'begin', 'exchange', 'finish']
    if fail:
        assert result is None
        assert worker.reason == protocol.CALIB_DIAG_REASON_RESULT_BUILD_FAILED
        assert io.limits == {}
    else:
        assert result is not None and result['result']['tendonLutValid']
        assert fit.count == 3
        assert set(io.limits) == {20, 21}


def test_run_dual_preserves_preflight_failure_status_without_mechanical_commands():
    io = FakeIO(joint=0)
    io.online &= ~(1 << config.joint_motors(4)[0])
    outcome = dual.run_dual(io, 0, 0, 1, 1, config.AutoCalibrationParams())
    assert outcome["preflight_failed"] and not outcome["success"] and not outcome["aborted"]
    assert outcome["reason"] == protocol.CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED
    assert not io.commands


def test_target_slack_wait_does_not_extend_fixed_3200_count_target():
    io = FakeIO()
    io.stop_at = 80
    io.on_publish = lambda state, targets: state.values.update({21: 0})
    worker = machine(io)
    assert not worker._wait_motor_target(21, -3200)
    assert [targets[21] for _, targets, _, _ in io.commands] == [-3200] * 4
    assert worker.reason == protocol.CALIB_DIAG_REASON_CONTROL_LOST
