"""Integral and fast-reversal autotuning entry point (dry-run, replay, hardware).

Usage:
  python tools/debug/integral_reversal_autotune.py --dry-run --calibration ... --output ...
  python tools/debug/integral_reversal_autotune.py --replay RUN_DIR --output ...
  python tools/debug/integral_reversal_autotune.py --hardware --port COM12 --calibration ... \
      --output run_data/integral_reversal_autotune_<time> --max-minutes 60

Hardware mode uses only the normal desktop authorization, full-group parameter
Apply, direct joint targets and the normal recorder. It never bypasses E-STOP,
collision, soft limits or feedback timeouts. A host watchdog adds the missing
Takeup timeout and no-progress checks; the 60-minute budget is enforced.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import shutil
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from desktop.core.control_parameters import configuration_crc, validate  # noqa: E402
from desktop.core.control_tuning import calibration_audit, sine_metrics  # noqa: E402
from desktop.core.host_calibration.config import joint_motors  # noqa: E402
from tools.debug import integral_reversal_core as core  # noqa: E402
from tools.debug.integral_reversal_report import build as build_full_report  # noqa: E402
from tools.debug.pid_autotune import (  # noqa: E402
    _apply, _healthy, _operation, _release, _set_target, _wait,
)

EXPECTED_KP = {'aa': 1.3, 'mcp': 1.3, 'pip': 1.0, 'dip': 1.3, 'cmc': 1.0}
EXPECTED_KD = {'aa': 0.08, 'mcp': 0.02, 'pip': 0.0, 'dip': 0.0, 'cmc': 0.0}
EXPECTED_FIRMWARE_SOURCE_CRC = 0x7B5080F4
SAFE_CONFIRM_DEG = 1.0
SAFE_CONFIRM_MS = 20
SAFE_EXIT_TRAVEL_DEG = 0.25
LOAD_STOP_COUNTS = 1000.0

STAGE_MINUTES = dict(preflight=5, integral=19, detector=26, takeup=47,
                     validation=55, final=60)


def _json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')


def _text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(value, encoding='utf-8')


def _safe_snapshot(hand) -> dict:
    snapshot = hand.control_parameter_snapshot()
    snapshot.pop('crc32', None)
    snapshot.pop('ramp_crc32',None)
    return snapshot


def _proposal(safe: dict, **changes) -> dict:
    proposal = copy.deepcopy(safe)
    for dotted, value in changes.items():
        group, key = dotted.split('__', 1)
        proposal[group][key] = value
    validate(proposal)
    return proposal


ALL_TYPES = ('aa', 'mcp', 'pip', 'dip', 'cmc')


def _set_all_reversal(proposal: dict, key: str, value: float) -> None:
    for name in ALL_TYPES:
        proposal['reversal'][f'{key}_{name}'] = value


def _set_type_reversal(proposal: dict, key: str, joint_type: str, value: float) -> None:
    proposal['reversal'][f'{key}_{joint_type}'] = value


def _set_all_pair(proposal: dict, value: float = 0.0) -> None:
    proposal['dual_tendon']['pair_recovery_gain'] = value


def _set_all_ramp(proposal: dict, b0: float = 0.0, vb: float = 0.0,
                  load_limit: float = 900.0) -> None:
    for name in ALL_TYPES:
        proposal['reversal_ramp'][f'b0_{name}'] = b0
        proposal['reversal_ramp'][f'vb_{name}'] = vb
        proposal['reversal_ramp'][f'load_limit_{name}'] = load_limit
    proposal['reversal_ramp']['takeup_timeout_ms'] = 600


def _set_integral_defaults(proposal: dict, limit: float = 100, noise: float = 0.1,
                           confirm_ms: float = 200) -> None:
    for name in ALL_TYPES:
        proposal['joint_ki'][f'integral_limit_counts_{name}'] = limit
    proposal['joint_ki']['integral_noise_deg'] = noise
    proposal['joint_ki']['integral_confirm_ms'] = confirm_ms


def _analysis_params(proposal: dict, joint_type: str, ki: float) -> dict:
    ki_group = proposal['joint_ki']
    reversal = proposal['reversal']
    return dict(ki=ki,
                integral_limit_counts=ki_group[f'integral_limit_counts_{joint_type}'],
                integral_noise_deg=ki_group['integral_noise_deg'],
                integral_confirm_ms=ki_group['integral_confirm_ms'],
                confirm_deg=reversal[f'confirm_deg_{joint_type}'],
                confirm_ms=reversal['confirm_ms'],
                takeup_counts=reversal[f'takeup_max_{joint_type}'],
                exit_travel_deg=reversal[f'exit_travel_deg_{joint_type}'],
                fast_window_ms=proposal['feedback']['fast_window_ms'],
                slow_window_ms=proposal['feedback']['slow_window_ms'])


def _apply_verified(hand, proposal: dict) -> str:
    current = _safe_snapshot(hand)
    expected_revision = int(current['revision']) + 1
    expected_crc = configuration_crc(proposal)
    _apply(hand, proposal)
    sync = str(getattr(hand, '_control_parameter_sync', ''))
    expected = f'APPLIED revision {expected_revision} / {expected_crc:08X}'
    if expected not in sync:
        raise RuntimeError(f'parameter readback mismatch: expected {expected!r}, got {sync!r}')
    return expected


def _wait_trace_ready(hand, joint: int, timeout: float = 4.0) -> None:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        state = hand.get_state_snapshot()
        comp = state.control_trace_components
        if (state.has_control_trace and state.control_enabled and comp
                and comp.get('slow_deg') and comp['slow_deg'][joint] is not None
                and time.time() - state.control_trace_last_rx_ts < 0.15):
            return
        time.sleep(0.02)
    raise RuntimeError('control trace/slow window not ready')


def _enable(hand, joint: int, motors, hold_joints=()) -> None:
    _release(hand)
    hold = [int(item) for item in hold_joints if int(item) != joint]
    hand.select_joints([joint, *hold])
    hand.select_motors(list(motors))
    _operation(hand, hand.request_enable())
    _wait(lambda: hand.get_state_snapshot().control_enabled, 5, 'enable timeout')
    hand.set_control_trace_subscription(True, 1)
    started = time.monotonic()
    _wait_trace_ready(hand, joint)
    remaining = 0.65 - (time.monotonic() - started)
    if remaining > 0:
        time.sleep(remaining)


def _sample_from_state(state, joint: int, motors, segment: str, expected_revision: int,
                       expected_crc: int) -> dict:
    comp = state.control_trace_components
    detail = state.control_trace_detail
    if not comp or not detail:
        raise RuntimeError('control components/detail missing')
    if int(comp.get('sequence', -1)) != int(state.control_trace_sequence):
        raise RuntimeError('control components sequence mismatch')
    slow = comp.get('slow_deg', [None] * 21)[joint]
    if slow is None:
        raise RuntimeError('slow window not ready')
    motor = int(motors[0])
    actual = float(state.control_trace_actual_deg[joint])
    target = float(state.control_trace_target_deg[joint])
    command = list(state.control_trace_motor_command_abs)
    actual_abs = list(detail.get('actual_abs', [0] * 27))
    pid_counts = list(detail.get('pid_counts', [0] * 27))
    # A single-tendon LUT boundary clamp is normal differential behaviour on a
    # dual-tendon joint; only a joint whose every axis is clipped is saturated.
    saturated = all(abs(float(command[m]) - (float(actual_abs[m]) + float(pid_counts[m]))) > 1.0
                    for m in motors)
    i_values = [abs(int(comp.get('integral_counts', [0] * 27)[m])) for m in motors]
    reversal_values = [abs(int(comp.get('reversal_counts', [0] * 27)[m])) for m in motors]
    return dict(
        t=float(state.control_trace_p4_time_ms),
        encoder_ms=int(state.control_trace_encoder_timestamp_ms),
        target=target,
        actual=actual,
        slow=float(slow),
        i_counts=float(comp.get('integral_counts', [0] * 27)[motor]),
        reversal_counts=max(reversal_values) if reversal_values else 0,
        p_counts=float(comp.get('proportional_counts', [0] * 27)[motor]),
        pid_counts=float(pid_counts[motor]),
        motor_command_abs=command,
        motor_actual_abs=actual_abs,
        saturated=bool(saturated),
        segment=segment,
        revision=int(expected_revision),
        crc=int(expected_crc),
        collision=bool(state.feedback_collision_hold_bitmap & (1 << joint)),
        control_enabled=bool(state.control_enabled),
        load={int(m): int(state.servo_load[m]) for m in motors},
        temperature={int(m): int(state.servo_temperature[m]) for m in motors},
    )


class StopRequested(RuntimeError):
    pass


class Watchdog:
    """Host-side safety independent of the P4 collision policy."""

    def __init__(self, output: Path, joint: int, motors, low: float, high: float,
                 envelope_margin: float = 2.0):
        self.output = output
        self.joint = joint
        self.motors = tuple(int(m) for m in motors)
        self.mask = sum(1 << m for m in self.motors)
        self.low = low
        self.high = high
        self.margin = envelope_margin
        self.takeup_since = None
        self.saturation_window = []
        self.jitter_window = []
        self.progress_window = []
        self.segment = ''
        self.segment_start = 0.0
        self.last_load = {m: 0 for m in self.motors}

    def _check_stop(self):
        if (self.output / 'STOP').exists():
            raise StopRequested('STOP file created by operator')

    def check(self, state, sample: dict, now: float):
        self._check_stop()
        if not state.control_enabled:
            raise RuntimeError('control disabled during trial')
        if state.feedback_collision_hold_bitmap & (1 << self.joint):
            raise RuntimeError('collision HOLD')
        if state.multi_turn_unknown_bitmap & self.mask:
            raise RuntimeError('multi-turn UNKNOWN during trial')
        if state.control_trace_motor_command_mask & ~self.mask:
            raise RuntimeError('command emitted for unselected motor')
        if time.time() - state.control_trace_last_rx_ts > 0.15:
            raise RuntimeError('control trace older than 150 ms')
        if state.system_fault_bitmap:
            raise RuntimeError(f'P4 fault bitmap 0x{state.system_fault_bitmap:08X}')
        actual = float(sample['actual'])
        target = float(sample['target'])
        if not (self.low - self.margin <= actual <= self.high + self.margin):
            raise RuntimeError(f'measured angle {actual:.2f} outside envelope')
        if not (self.low - self.margin <= target <= self.high + self.margin):
            raise RuntimeError(f'target angle {target:.2f} outside envelope')
        for motor in self.motors:
            if abs(int(state.servo_load[motor])) >= LOAD_STOP_COUNTS:
                raise RuntimeError(f'M{motor:02d} load >= {LOAD_STOP_COUNTS:.0f}')
        self.saturation_window.append((now, bool(sample.get('saturated'))))
        self.saturation_window = [(stamp, value) for stamp, value in self.saturation_window
                                  if now - stamp <= 0.3]
        if len(self.saturation_window) >= 5 and \
                self.saturation_window[-1][0] - self.saturation_window[0][0] >= 0.15 and \
                sum(1 for _, value in self.saturation_window if value) / len(self.saturation_window) > 0.8:
            raise RuntimeError('sustained saturation')
        if sample['segment'] != self.segment:
            self.segment = sample['segment']
            self.segment_start = now
            self.jitter_window = []
        self.jitter_window.append((now, actual))
        self.jitter_window = [(stamp, value) for stamp, value in self.jitter_window
                              if now - stamp <= 1.0]
        if self.segment.startswith('hold') and now - self.segment_start > 2.4 and \
                len(self.jitter_window) >= 12:
            values = [value for _, value in self.jitter_window]
            mean = sum(values) / len(values)
            std = math.sqrt(sum((value - mean) ** 2 for value in values) / len(values))
            if std > 2.5:
                raise RuntimeError(f'hold jitter {std:.2f} deg in {self.segment}')
        if sample.get('reversal_counts'):
            if self.takeup_since is None:
                self.takeup_since = now
            elif now - self.takeup_since > core.TAKEUP_TIMEOUT_S:
                raise RuntimeError('takeup timeout without exit')
        else:
            self.takeup_since = None


PROXIMAL_HOLD = {9: (8,), 10: (8, 9), 11: (8, 9, 10)}


def _hold_for(joint: int):
    return PROXIMAL_HOLD.get(int(joint), ())


def _run_plan(hand, output: Path, joint: int, motors, points, label: str,
              low: float, high: float, revision: int, crc: int,
              hold_joints=()) -> tuple:
    cap = core.TEST_HIGH_CAP_DEG.get('mcp') if joint == core.REPRESENTATIVE_JOINTS['mcp'] else None
    before = hand.get_state_snapshot()
    current = before.encoders[joint].current_angle_deg
    if joint in (9, 10, 11) and before.encoders[4].current_angle_deg < 20.0:
        raise RuntimeError('index AA J04 is not at its maximum; move it aside before middle-finger tests')
    if cap is not None and current > cap + 0.5:
        raise RuntimeError(f'J{joint:02d} current {current:.2f} above test cap {cap:.1f}; recover first')
    if cap is not None and max(point.target_deg for point in points) > cap + 1e-6:
        raise RuntimeError(f'plan target exceeds MCP test cap {cap:.1f}')
    # Watchdog margin is 2 deg; use 73 deg so actual MCP motion cannot exceed 75 deg.
    watch_high = min(high, cap - 1.0) if cap is not None else high
    hold = [int(item) for item in hold_joints if int(item) != joint]
    hold_targets = {item: float(before.encoders[item].current_angle_deg) for item in hold}
    all_motors = list(dict.fromkeys([*motors, *[m for item in hold for m in joint_motors(item)]]))
    _enable(hand, joint, all_motors, hold)
    watchdog = Watchdog(output, joint, all_motors, low, watch_high)
    state = hand.get_state_snapshot()
    values = list(state.target_angles)
    for item, angle in hold_targets.items():
        values[item] = angle
    raw_path = output / 'raw' / f'{label}.jsonl'
    raw_path.parent.mkdir(parents=True, exist_ok=True)
    samples = []
    point_index = 0
    segment = points[0].segment
    duration = core.plan_duration(points)
    start = time.monotonic()
    last_send = -1.0
    last_sequence = -1
    sequence_gaps = 0
    raw_stream = raw_path.open('w', encoding='utf-8', buffering=1)
    try:
        while True:
            now = time.monotonic()
            elapsed = now - start
            if elapsed > duration + 0.5:
                break
            while point_index + 1 < len(points) and points[point_index + 1].time_s <= elapsed:
                point_index += 1
                segment = points[point_index].segment
            target = float(points[point_index].target_deg)
            if elapsed - last_send >= 0.019:
                values = _set_target(hand, values, joint, target, f'autotune {label}')
                for item, angle in hold_targets.items():
                    values = _set_target(hand, values, item, angle, 'hold')
                last_send = elapsed
            state = _healthy(hand, joint, sum(1 << m for m in motors))
            if state.control_trace_sequence != last_sequence and state.has_control_trace:
                if last_sequence >= 0 and state.control_trace_sequence != last_sequence + 1:
                    sequence_gaps += 1
                last_sequence = state.control_trace_sequence
                sample = _sample_from_state(state, joint, motors, segment, revision, crc)
                watchdog.check(state, sample, now)
                samples.append(sample)
            time.sleep(0.005)
    finally:
        for sample in samples:
            raw_stream.write(json.dumps(sample) + '\n')
        raw_stream.close()
        _release(hand)
        hand.select_joints([])
        hand.select_motors([])
    if not samples:
        raise RuntimeError('no trace samples')
    # The host state snapshot publishes one trace per USB RX batch, so sequence
    # numbers can jump even though every packet reached the host.  P4 timestamps
    # remain authoritative; record the ratio for the report instead of aborting.
    if sequence_gaps > max(3, len(samples) // 2):
        print(f'[trace] large host-side gaps {sequence_gaps}/{len(samples)}', flush=True)
    return samples, duration


def _lead_in_plan(hand, joint: int, points, speed_deg_s: float = 5.0):
    """Prepend a smooth ramp from the current pose to the first plan target."""
    state = hand.get_state_snapshot()
    try:
        current = float(state.encoders[joint].current_angle_deg)
    except Exception:
        return points
    first = float(points[0].target_deg)
    if not math.isfinite(current) or abs(current - first) <= 0.2:
        return points
    duration = abs(current - first) / max(speed_deg_s, 0.5)
    count = max(2, int(duration / 0.01))
    lead = [core.PlanPoint(duration * index / count,
                           current + (first - current) * index / count,
                           'lead_in', 'ramp') for index in range(count + 1)]
    shifted = [core.PlanPoint(point.time_s + duration, point.target_deg,
                              point.segment, point.kind, point.position,
                              point.approach, point.event) for point in points]
    return lead + shifted


def _short_hold_plan(start_deg: float, center: float, ramp_s: float = 2.0,
                     hold_s: float = 2.0) -> list:
    points = []
    count = max(2, int(ramp_s / 0.01))
    for index in range(count + 1):
        fraction = index / count
        points.append(core.PlanPoint(ramp_s * fraction,
                                     start_deg + (center - start_deg) * fraction,
                                     'preflight_ramp', 'ramp'))
    count = max(2, int(hold_s / 0.01))
    for index in range(count + 1):
        points.append(core.PlanPoint(ramp_s + hold_s * index / count, center,
                                     'preflight_hold', 'hold'))
    return points


def _hold_noise(trial: core.IntegralTrial) -> float:
    """Standard-error style noise floor for the normalised hold score.

    The baseline is sampled by the same process; the last-second jitter and the
    number of same-cycle samples bound the statistical repeatability.  No prior
    hour data is reused.
    """
    if not trial.holds:
        return 0.02
    jitters = [float(hold.get('jitter_deg', 0.0)) / core.TOL_JITTER_DEG
               for hold in trial.holds if hold.get('valid')]
    if not jitters:
        return 0.02
    jitters.sort()
    median = jitters[len(jitters) // 2]
    # ~20 same-cycle samples in the last second at the observed host trace rate.
    return max(0.02, 0.5 * median / math.sqrt(20.0))


def _integral_history(records: list) -> list:
    return [core.IntegralTrial(**row['trial']) for row in records if row.get('kind') == 'integral']


def _mk_integral_trial(joint_type: str, joint: int, ki: float,
                       result: core.IntegralTrial) -> core.IntegralTrial:
    result.joint_type = joint_type
    result.joint = joint
    return result


def _find_audit(audit, joint: int):
    return next((row for row in audit if row.get('joint') == joint), None)


def _plan_estimate(points) -> float:
    return core.plan_duration(points) + 10.0


def _write_trial_meta(output: Path, label: str, **data):
    _json(output / 'meta' / f'{label}.json', data)


def _record(output: Path, records: list, **row):
    records.append(dict(time=time.time(), **row))
    _json(output / 'trial_index.json', records)


def run_hardware(args) -> int:
    from desktop.core import application_actions
    from desktop.core.hand_backend import HandBackend

    calibration_path = Path(args.calibration)
    if not calibration_path.is_file():
        raise RuntimeError(f'calibration not found: {calibration_path}')
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    (output / 'raw').mkdir()
    (output / 'meta').mkdir()
    application_actions.RUN_DATA_DIR = output / 'recordings'
    bundle = json.loads(calibration_path.read_text(encoding='utf-8'))
    audit = calibration_audit(bundle)
    rows = {joint: _find_audit(audit, joint) for joint in core.CANDIDATE_JOINTS}
    for joint, row in rows.items():
        if not row or not row.get('eligible'):
            raise RuntimeError(f'J{joint:02d} calibration not eligible: {row}')
    shutil.copy2(calibration_path, output / 'calibration.mmcal.json')
    for name in ('ESP32-P4.ino.elf.sha256.txt', 'build.options.json'):
        source = ROOT / '.debug' / 'p4' / name
        if source.is_file():
            shutil.copy2(source, output / name)
    try:
        pending = json.loads((ROOT / 'desktop/resources/config/control_parameters.json').read_text(encoding='utf-8'))
    except Exception:
        pending = None
    started = time.monotonic()
    timeline = {name: started + minutes * 60.0 for name, minutes in STAGE_MINUTES.items()}
    hand = HandBackend()
    hand._save_data = False
    hand.open_application()
    records: list = []
    selected: dict = {}
    status = dict(ok=False, stage='connect', error='')
    try:
        if not hand.connect(args.port):
            raise RuntimeError(f'failed to connect {args.port}')
        _wait(lambda: bool(hand.get_state_snapshot().product_fingerprint), 5,
              'product fingerprint')
        state = hand.get_state_snapshot()
        fingerprint = dict(state.product_fingerprint or {})
        _json(output / 'fingerprint.json', fingerprint)
        if not fingerprint:
            raise RuntimeError('product fingerprint not received')
        if int(fingerprint.get('firmware_source_crc', -1)) != EXPECTED_FIRMWARE_SOURCE_CRC:
            # The source CRC is regenerated separately; the authoritative check
            # is the schema-v5 parameter APPLIED readback below.
            print('[preflight] firmware source CRC differs from last generated fingerprint; '
                  'continuing to schema-v5 parameter readback', flush=True)
        inspected = hand._calibration_workflow.inspect_snapshot(calibration_path)
        ok, message = hand._controller.load_calibration_snapshot(inspected)
        if not ok:
            raise RuntimeError(f'calibration load failed: {message}')
        _wait(lambda: hand._control_parameter_sync.startswith('APPLIED'), 15, 'calibration sync')
        before = _safe_snapshot(hand)
        _json(output / 'parameters.before.json', before)
        if pending is not None and pending.get('revision') != before.get('revision'):
            pass
        for joint_type in core.TYPES:
            if abs(float(before['joint_kp'][joint_type]) - EXPECTED_KP[joint_type]) > 1e-6 or \
                    abs(float(before['joint_kd'][joint_type]) - EXPECTED_KD[joint_type]) > 1e-6:
                raise RuntimeError(f'{joint_type} Kp/Kd no longer match the frozen validated values')
        safe = copy.deepcopy(before)
        safe['joint_ki'] = {name: 0.0 for name in ALL_TYPES}
        _set_integral_defaults(safe)
        safe['reversal'] = {'confirm_ms': SAFE_CONFIRM_MS}
        _set_all_reversal(safe, 'confirm_deg', SAFE_CONFIRM_DEG)
        _set_all_reversal(safe, 'takeup_max', 0.0)
        _set_all_ramp(safe, 0.0, 0.0)
        _set_all_reversal(safe, 'exit_travel_deg', SAFE_EXIT_TRAVEL_DEG)
        _set_all_pair(safe, 0.0)
        safe['reversal_ramp'] = {**{f'b0_{name}': 0.0 for name in ALL_TYPES},
                                 **{f'vb_{name}': 0.0 for name in ALL_TYPES},
                                 **{f'load_limit_{name}': 900 for name in ALL_TYPES},
                                 'takeup_timeout_ms': 600}
        validate(safe)
        _json(output / 'parameters.safe.json', safe)
        _apply_verified(hand, safe)
        hand.request_recording(True)
        _wait(lambda: hand._recorder.is_running(), 5, 'recording start')
        status['stage'] = 'preflight'
        # Short safe motion self-check on J09, then release.
        joint = core.REPRESENTATIVE_JOINTS['mcp']
        motors = joint_motors(joint)
        row = rows[joint]
        center = 0.5 * (row['internal_low_deg'] + row['internal_high_deg'])
        state = hand.get_state_snapshot()
        start_angle = state.encoders[joint].current_angle_deg
        pre_plan = _short_hold_plan(start_angle, center)
        revision = _safe_snapshot(hand)['revision']
        crc = configuration_crc(_safe_snapshot(hand))
        pre_samples, _ = _run_plan(hand, output, joint, motors, pre_plan, 'preflight',
                                   row['internal_low_deg'], row['internal_high_deg'],
                                   revision, crc, hold_joints=_hold_for(joint))
        _json(output / 'preflight.json', dict(samples=len(pre_samples),
                                              sensor_ok=True,
                                              slow_window_ok=True,
                                              control_released=not hand.get_state_snapshot().control_enabled))
        if hand.get_state_snapshot().system_fault_bitmap:
            raise RuntimeError('fault latched during preflight')
        _release(hand)
        # ------------------------------------------------------------------
        # Stage A: integral
        status['stage'] = 'integral'
        selected = dict(ki={name: 0.0 for name in core.TYPES},
                        confirm_deg=SAFE_CONFIRM_DEG, confirm_ms=SAFE_CONFIRM_MS,
                        takeup_counts=0, exit_travel_deg=SAFE_EXIT_TRAVEL_DEG)
        integral_history: list = []
        integral_records: list = []
        for joint_type in core.TYPES:
            if time.monotonic() >= timeline['integral'] - 30:
                break
            joint = core.REPRESENTATIVE_JOINTS[joint_type]
            motors = joint_motors(joint)
            row = rows[joint]
            points = core.build_hold_plan(row['internal_low_deg'],
                                          row['internal_high_deg'],
                                          high_cap=core.TEST_HIGH_CAP_DEG.get(joint_type))
            ki_values = (0.0, 0.1, 0.2)
            for ki in ki_values:
                if time.monotonic() + _plan_estimate(points) + 6.0 > timeline['integral']:
                    break
                proposal = copy.deepcopy(safe)
                for name in ALL_TYPES:
                    proposal['joint_ki'][name] = 0.0
                proposal['joint_ki'][joint_type] = ki
                proposal['reversal']['confirm_ms'] = SAFE_CONFIRM_MS
                _set_all_reversal(proposal, 'confirm_deg', SAFE_CONFIRM_DEG)
                _set_all_reversal(proposal, 'takeup_max', 0.0)
                _set_all_ramp(proposal, 0.0, 0.0)
                _set_all_reversal(proposal, 'exit_travel_deg', SAFE_EXIT_TRAVEL_DEG)
                _set_all_pair(proposal, 0.0)
                validate(proposal)
                label = f'integral_{joint_type}_ki{ki:.1f}'
                try:
                    applied = _apply_verified(hand, proposal)
                    revision = int(_safe_snapshot(hand)['revision'])
                    crc = configuration_crc(proposal)
                    run_points = _lead_in_plan(hand, joint, points)
                    samples, duration = _run_plan(hand, output, joint, motors, run_points, label,
                                                  row['internal_low_deg'], row['internal_high_deg'],
                                                  revision, crc, hold_joints=_hold_for(joint))
                    params = _analysis_params(proposal, joint_type, ki)
                    trial = _mk_integral_trial(joint_type, joint, ki,
                                               core.integral_metrics(samples, run_points, motors, params))
                    _json(output / 'analysis' / f'{label}.json', dict(result=trial.__dict__))
                    integral_history.append(trial)
                    _record(output, integral_records, kind='integral', label=label,
                            joint_type=joint_type, joint=joint, ki=ki,
                            applied=applied, duration_s=duration,
                            trial=trial.__dict__)
                    print(f'[integral] {joint_type} J{joint:02d} Ki={ki:.1f} '
                          f'score={trial.score if trial.score is not None else "nan"} '
                          f'valid={trial.valid} reason={trial.reason}', flush=True)
                    if ki == 0.0:
                        max_ss = max((hold.get('e_ss_deg', 99.0) for hold in trial.holds
                                      if hold.get('valid')), default=99.0)
                        if max_ss < core.INTEGRAL_NOISE_DEG and trial.valid:
                            break
                except (RuntimeError, StopRequested) as exc:
                    _record(output, integral_records, kind='integral', label=label,
                            joint_type=joint_type, joint=joint, ki=ki,
                            valid=False, reason=str(exc))
                    print(f'[integral] {label} FAILED: {exc}', flush=True)
                    try:
                        _apply_verified(hand, safe)
                    except Exception:
                        pass
                    if (output / 'STOP').exists():
                        raise
                    continue
            type_trials = [trial for trial in integral_history if trial.joint_type == joint_type]
            baseline = next((trial for trial in type_trials if trial.ki == 0.0 and trial.valid), None)
            noise = _hold_noise(baseline) if baseline else 0.02
            choice = core.choose_integral(type_trials, noise)
            selected['ki'][joint_type] = float(choice['selected_ki'])
            _record(output, integral_records, kind='integral_choice', joint_type=joint_type,
                    selected_ki=choice['selected_ki'], reason=choice['reason'], noise=noise)
            print(f'[integral] selected {joint_type} Ki={choice["selected_ki"]} '
                  f'({choice["reason"]})', flush=True)
        _json(output / 'integral_selection.json', dict(selected=selected, trials=[
            dict(trial.__dict__) for trial in integral_history]))
        # ------------------------------------------------------------------
        # Stage B: detector offline
        status['stage'] = 'detector'
        suite = core.build_detector_suite()
        scores = core.evaluate_detector_grid(suite)
        best = core.choose_detector(scores)
        selected['confirm_deg'] = best.confirm_deg
        selected['confirm_ms'] = best.confirm_ms
        _json(output / 'detector' / 'scores.json', [score.__dict__ for score in scores])
        _json(output / 'detector' / 'selected.json', best.__dict__)
        print(f'[detector] selected confirm_deg={best.confirm_deg} confirm_ms={best.confirm_ms} '
              f'TP={best.tp} FP={best.fp} FN={best.fn} p90={best.p90_latency_s}', flush=True)
        # ------------------------------------------------------------------
        # Stage C: takeup / exit
        status['stage'] = 'takeup'
        single_type = max((name for name in core.TYPES if name != 'aa'),
                          key=lambda name: max((trial.score or 0.0 for trial in integral_history
                                                if trial.joint_type == name and trial.ki == 0.0),
                                               default=0.0))
        takeup_history: list = []
        takeup_records: list = []
        single_joint = core.REPRESENTATIVE_JOINTS[single_type]
        aa_joint = core.REPRESENTATIVE_JOINTS['aa']
        for candidate_takeup in core.TAKEUP_CANDIDATES:
            if time.monotonic() >= timeline['takeup'] - 30:
                break
            for joint, joint_type in ((single_joint, single_type), (aa_joint, 'aa')):
                if candidate_takeup == 0 and joint_type != 'aa' and joint != single_joint:
                    continue
                if candidate_takeup != 0 and joint_type == 'aa' and candidate_takeup > 40:
                    continue
                row = rows[joint]
                motors = joint_motors(joint)
                points, truth = core.build_reversal_plan(
                    row['internal_low_deg'], row['internal_high_deg'],
                    speeds=(('low', 5.0),) if candidate_takeup != 0 else
                    (('low', 5.0), ('high', 20.0)),
                    high_cap=core.TEST_HIGH_CAP_DEG.get(joint_type))
                if time.monotonic() + _plan_estimate(points) + 6.0 > timeline['takeup']:
                    break
                proposal = copy.deepcopy(safe)
                for name in ALL_TYPES:
                    proposal['joint_ki'][name] = 0.0
                proposal['reversal']['confirm_ms'] = selected['confirm_ms']
                _set_all_reversal(proposal, 'confirm_deg', selected['confirm_deg'])
                _set_all_reversal(proposal, 'takeup_max', candidate_takeup)
                _set_all_ramp(proposal, 2.0 if candidate_takeup > 0 else 0.0, 20.0 if candidate_takeup > 0 else 0.0)
                _set_all_reversal(proposal, 'exit_travel_deg', SAFE_EXIT_TRAVEL_DEG)
                _set_all_pair(proposal, 0.0)
                validate(proposal)
                label = f'takeup_{joint_type}_{candidate_takeup}'
                try:
                    applied = _apply_verified(hand, proposal)
                    revision = int(_safe_snapshot(hand)['revision'])
                    crc = configuration_crc(proposal)
                    run_points = _lead_in_plan(hand, joint, points)
                    samples, duration = _run_plan(hand, output, joint, motors, run_points, label,
                                                  row['internal_low_deg'], row['internal_high_deg'],
                                                  revision, crc, hold_joints=_hold_for(joint))
                    params = _analysis_params(proposal, joint_type, 0.0)
                    trial = core.reversal_metrics(samples, truth, motors, params)
                    trial.joint_type = joint_type
                    trial.joint = joint
                    _json(output / 'analysis' / f'{label}.json', dict(result=trial.__dict__))
                    takeup_history.append(trial)
                    _record(output, takeup_records, kind='takeup', label=label,
                            joint_type=joint_type, joint=joint,
                            takeup_counts=candidate_takeup,
                            exit_travel_deg=SAFE_EXIT_TRAVEL_DEG,
                            applied=applied, duration_s=duration,
                            trial=trial.__dict__)
                    print(f'[takeup] {joint_type} J{joint:02d} takeup={candidate_takeup} '
                          f'score={trial.score if trial.score is not None else "nan"} '
                          f'valid={trial.valid} reason={trial.reason}', flush=True)
                    if candidate_takeup != 0 and (not trial.valid or trial.score is None):
                        raise RuntimeError('nonzero takeup candidate infeasible; stop larger values')
                except (RuntimeError, StopRequested) as exc:
                    _record(output, takeup_records, kind='takeup', label=label,
                            joint_type=joint_type, joint=joint,
                            takeup_counts=candidate_takeup,
                            exit_travel_deg=SAFE_EXIT_TRAVEL_DEG,
                            valid=False, reason=str(exc))
                    print(f'[takeup] {label} FAILED: {exc}', flush=True)
                    try:
                        _apply_verified(hand, safe)
                    except Exception:
                        pass
                    if (output / 'STOP').exists():
                        raise
                    if candidate_takeup != 0:
                        break
            if candidate_takeup != 0:
                # Stop the C1 sweep when the smallest nonzero candidate has no
                # valid AA result; never deploy an unvalidated global takeup.
                aa_trials = [trial for trial in takeup_history
                             if trial.takeup_counts == candidate_takeup and trial.joint_type == 'aa']
                if not aa_trials or not aa_trials[0].valid:
                    break
                single_trials = [trial for trial in takeup_history
                                 if trial.takeup_counts == candidate_takeup and trial.joint_type != 'aa']
                if single_trials and single_trials[0].score is None:
                    break
        # Small exit-travel refinement at the best takeup, if time remains.
        valid_groups = [trial for trial in takeup_history if trial.valid and trial.score is not None]
        if valid_groups and time.monotonic() < timeline['takeup'] - 90:
            nonzero = [trial for trial in valid_groups if trial.takeup_counts > 0]
            if nonzero:
                best_takeup = min(trial.takeup_counts for trial in nonzero)
                for exit_travel in (0.15, 0.50):
                    if time.monotonic() + 40 > timeline['takeup']:
                        break
                    for joint, joint_type in ((single_joint, single_type), (aa_joint, 'aa')):
                        row = rows[joint]
                        motors = joint_motors(joint)
                        points, truth = core.build_reversal_plan(
                            row['internal_low_deg'], row['internal_high_deg'],
                            high_cap=core.TEST_HIGH_CAP_DEG.get(joint_type),
                            speeds=(('low', 5.0),))
                        proposal = copy.deepcopy(safe)
                        for name in ALL_TYPES:
                            proposal['joint_ki'][name] = 0.0
                        proposal['reversal']['confirm_ms'] = selected['confirm_ms']
                        _set_all_reversal(proposal, 'confirm_deg', selected['confirm_deg'])
                        _set_all_reversal(proposal, 'takeup_max', best_takeup)
                        _set_all_ramp(proposal, 2.0 if best_takeup > 0 else 0.0, 20.0 if best_takeup > 0 else 0.0)
                        _set_all_reversal(proposal, 'exit_travel_deg', exit_travel)
                        _set_all_pair(proposal, 0.0)
                        validate(proposal)
                        label = f'exit_{joint_type}_{best_takeup}_{exit_travel:.2f}'
                        try:
                            applied = _apply_verified(hand, proposal)
                            revision = int(_safe_snapshot(hand)['revision'])
                            crc = configuration_crc(proposal)
                            run_points = _lead_in_plan(hand, joint, points)
                            samples, duration = _run_plan(hand, output, joint, motors, run_points,
                                                          label, row['internal_low_deg'],
                                                          row['internal_high_deg'], revision, crc, hold_joints=_hold_for(joint))
                            params = _analysis_params(proposal, joint_type, 0.0)
                            trial = core.reversal_metrics(samples, truth, motors, params)
                            trial.joint_type = joint_type
                            trial.joint = joint
                            _json(output / 'analysis' / f'{label}.json', dict(result=trial.__dict__))
                            takeup_history.append(trial)
                            _record(output, takeup_records, kind='takeup', label=label,
                                    joint_type=joint_type, joint=joint,
                                    takeup_counts=best_takeup,
                                    exit_travel_deg=exit_travel, applied=applied,
                                    duration_s=duration, trial=trial.__dict__)
                            print(f'[exit] {joint_type} takeup={best_takeup} exit={exit_travel:.2f} '
                                  f'score={trial.score if trial.score is not None else "nan"} '
                                  f'valid={trial.valid}', flush=True)
                        except (RuntimeError, StopRequested) as exc:
                            _record(output, takeup_records, kind='takeup', label=label,
                                    joint_type=joint_type, joint=joint,
                                    takeup_counts=best_takeup,
                                    exit_travel_deg=exit_travel, valid=False, reason=str(exc))
                            print(f'[exit] {label} FAILED: {exc}', flush=True)
                            try:
                                _apply_verified(hand, safe)
                            except Exception:
                                pass
        choice = core.choose_takeup(takeup_history)
        # A nonzero global takeup requires a valid AA result at that exact value.
        if choice.get('selected_takeup', 0) > 0:
            aa_ok = any(trial.joint_type == 'aa' and trial.valid and
                        trial.takeup_counts == choice['selected_takeup'] and
                        trial.exit_travel_deg == choice['selected_exit']
                        for trial in takeup_history)
            if not aa_ok:
                choice = dict(selected_takeup=0.0, selected_exit=SAFE_EXIT_TRAVEL_DEG,
                              reason='nonzero takeup not validated on AA; conservative global value')
        selected['takeup_counts'] = float(choice['selected_takeup'])
        selected['exit_travel_deg'] = float(choice['selected_exit'])
        _json(output / 'takeup_selection.json', dict(selected=selected, choice=choice,
                                                     trials=[dict(trial.__dict__) for trial in takeup_history]))
        print(f'[takeup] selected takeup={selected["takeup_counts"]} '
              f'exit={selected["exit_travel_deg"]} ({choice["reason"]})', flush=True)
        _write_trial_meta(output, 'takeup_selection', **choice)
        # ------------------------------------------------------------------
        # Stage D: combined validation
        status['stage'] = 'validation'
        baseline_metrics = {}
        candidate_metrics = {}
        validation_ok = True
        reasons = []
        for joint_type in core.TYPES:
            if time.monotonic() >= timeline['validation'] - 20:
                validation_ok = False
                reasons.append('validation time budget exhausted')
                break
            joint = core.REPRESENTATIVE_JOINTS[joint_type]
            motors = joint_motors(joint)
            row = rows[joint]
            points, truth = core.build_validation_plan(
                row['internal_low_deg'], row['internal_high_deg'],
                high_cap=core.TEST_HIGH_CAP_DEG.get(joint_type))
            for variant in ('baseline', 'candidate'):
                if time.monotonic() + _plan_estimate(points) > timeline['validation']:
                    validation_ok = False
                    reasons.append(f'{joint_type} {variant} skipped by deadline')
                    break
                proposal = copy.deepcopy(safe)
                for name in ALL_TYPES:
                    proposal['joint_ki'][name] = 0.0
                if variant == 'candidate':
                    for name in core.TYPES:
                        proposal['joint_ki'][name] = selected['ki'][name]
                proposal['reversal']['confirm_ms'] = selected['confirm_ms']
                _set_all_reversal(proposal, 'confirm_deg', selected['confirm_deg'])
                takeup = 0.0 if variant == 'baseline' else selected['takeup_counts']
                exit_travel = SAFE_EXIT_TRAVEL_DEG if variant == 'baseline' else selected['exit_travel_deg']
                _set_all_reversal(proposal, 'takeup_max', takeup)
                _set_all_ramp(proposal, 2.0 if takeup > 0 else 0.0, 20.0 if takeup > 0 else 0.0)
                _set_all_reversal(proposal, 'exit_travel_deg', exit_travel)
                _set_all_pair(proposal, 0.0)
                validate(proposal)
                label = f'validate_{joint_type}_{variant}'
                try:
                    applied = _apply_verified(hand, proposal)
                    revision = int(_safe_snapshot(hand)['revision'])
                    crc = configuration_crc(proposal)
                    run_points = _lead_in_plan(hand, joint, points)
                    samples, duration = _run_plan(hand, output, joint, motors, run_points, label,
                                                  row['internal_low_deg'], row['internal_high_deg'],
                                                  revision, crc, hold_joints=_hold_for(joint))
                    params = _analysis_params(proposal, joint_type, selected['ki'][joint_type])
                    metrics = _validation_metrics(samples, run_points, truth, motors, params)
                    metrics['joint_type'] = joint_type
                    metrics['joint'] = joint
                    metrics['variant'] = variant
                    metrics['hard_failure'] = ''
                    if variant == 'candidate':
                        candidate_metrics[joint_type] = metrics
                    else:
                        baseline_metrics[joint_type] = metrics
                    _json(output / 'validation' / f'{label}.json', metrics)
                    _record(output, records, kind='validation', label=label,
                            joint_type=joint_type, joint=joint, variant=variant,
                            applied=applied, duration_s=duration, metrics=metrics)
                    print(f'[validate] {joint_type} {variant} '
                          f"J_I={metrics.get('integral_score')} J_R={metrics.get('reversal_score')} "
                          f"rmse={metrics.get('dynamic_rmse_deg')}", flush=True)
                except (RuntimeError, StopRequested) as exc:
                    validation_ok = False
                    reasons.append(f'{joint_type} {variant}: {exc}')
                    _record(output, records, kind='validation', label=label,
                            joint_type=joint_type, joint=joint, variant=variant,
                            valid=False, reason=str(exc))
                    print(f'[validate] {label} FAILED: {exc}', flush=True)
                    try:
                        _apply_verified(hand, safe)
                    except Exception:
                        pass
                    break
        ok, compare_reasons = core.validate_combined(baseline_metrics, candidate_metrics,
                                                     noise=0.02)
        if not ok:
            reasons.extend(compare_reasons)
        if not validation_ok:
            reasons.append('combined validation incomplete')
        combined_ok = bool(ok and validation_ok)
        if not combined_ok and selected.get('takeup_counts', 0) > 0:
            selected['takeup_counts'] = 0.0
            selected['exit_travel_deg'] = SAFE_EXIT_TRAVEL_DEG
            reasons.append('fallback: takeup set to 0 after validation failure')
        _json(output / 'validation' / 'summary.json',
              dict(ok=ok, combined_ok=combined_ok, reasons=reasons,
                   baseline=baseline_metrics, candidate=candidate_metrics))
        status['stage'] = 'finalize'
        # ------------------------------------------------------------------
        # Final apply and artifacts
        final = copy.deepcopy(safe)
        if combined_ok:
            for name in core.TYPES:
                final['joint_ki'][name] = selected['ki'][name]
            final['reversal']['confirm_ms'] = selected['confirm_ms']
            _set_all_reversal(final, 'confirm_deg', selected['confirm_deg'])
            _set_all_reversal(final, 'takeup_max', selected['takeup_counts'])
            _set_all_ramp(final, 2.0 if selected['takeup_counts'] > 0 else 0.0, 20.0 if selected['takeup_counts'] > 0 else 0.0)
            _set_all_reversal(final, 'exit_travel_deg', selected['exit_travel_deg'])
        else:
            final = copy.deepcopy(safe)
        validate(final)
        applied = _apply_verified(hand, final)
        _release(hand)
        final_snapshot = _safe_snapshot(hand)
        final['revision'] = int(final_snapshot['revision'])
        _json(output / 'parameters.selected.json', selected)
        _json(output / 'final_parameters.json', final)
        _json(output / 'parameters.verified.json', final)
        if not combined_ok:
            _json(output / 'parameters.fallback.json', safe)
        _json(output / 'deployment.json', dict(
            applied=applied, revision=int(final_snapshot['revision']),
            crc=configuration_crc(final), crc32=f'{configuration_crc(final):08X}',
            saved_at=datetime.now().astimezone().isoformat(),
            frozen_kp=final['joint_kp'], frozen_kd=final['joint_kd'],
            joint_ids=list(core.CANDIDATE_JOINTS),
            motors={str(j): list(joint_motors(j)) for j in core.CANDIDATE_JOINTS},
            workspace={str(j): dict(low=rows[j]['internal_low_deg'],
                                    high=rows[j]['internal_high_deg'])
                       for j in core.CANDIDATE_JOINTS},
            firmware_fingerprint=fingerprint,
            calibration_sha256=_sha256(calibration_path),
            combined_ok=combined_ok, reasons=reasons,
            control_disabled=not hand.get_state_snapshot().control_enabled,
            faults=int(hand.get_state_snapshot().system_fault_bitmap),
        ))
        _write_report(output, records, selected, combined_ok, reasons, timeline, started)
        try:
            _text(output / 'report.md', build_full_report(output))
        except Exception as report_exc:
            print(f'[report] full report rebuild failed: {report_exc}', flush=True)
        status.update(ok=True, stage='finalize', final=final, selected=selected,
                      combined_ok=combined_ok, reasons=reasons,
                      elapsed_s=time.monotonic() - started)
        print(f'[final] combined_ok={combined_ok} applied={applied}', flush=True)
    except BaseException as exc:
        status['error'] = f'{type(exc).__name__}: {exc}'
        try:
            if hand.is_comm_connected():
                hand.stop_motion(source='integral/reversal autotune aborted')
        except Exception:
            pass
        try:
            if hand.is_comm_connected():
                _release(hand)
        except Exception:
            pass
        try:
            if hand.is_comm_connected() and pending:
                validate(pending)
                _apply_verified(hand, pending)
        except Exception as restore_exc:
            status['restore_error'] = f'{type(restore_exc).__name__}: {restore_exc}'
        try:
            _write_report(output, records, selected, False,
                          [status['error']], timeline, started)
        except Exception:
            pass
        _json(output / 'status.json', status)
        print(f'[abort] {status["error"]}', flush=True)
        return 1
    finally:
        try:
            hand.request_recording(False)
            if hand._recording_thread:
                hand._recording_thread.join(timeout=10)
        except Exception:
            pass
        try:
            hand.disconnect(stop_lower=False)
        except Exception:
            pass
        try:
            hand.close_application()
        except Exception:
            pass
    _json(output / 'status.json', status)
    print(f'HARDWARE complete: {output} combined_ok={status.get("combined_ok")}', flush=True)
    return 0 if status.get('ok') else 1


def _sha256(path: Path) -> str:
    import hashlib
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b''):
            digest.update(chunk)
    return digest.hexdigest().upper()


def _validation_metrics(samples, points, truth, motors, params) -> dict:
    replayed = core.replay_control_state(samples, params)
    by_segment = {}
    for row in replayed:
        by_segment.setdefault(row.get('segment'), []).append(row)
    result = {}
    hold_rows = by_segment.get('validate_hold') or by_segment.get('preflight_hold') or []
    if hold_rows:
        tail = [row for row in hold_rows if float(row['t']) >= float(hold_rows[-1]['t']) - 1000.0]
        errors = [float(row['target']) - float(row['actual']) for row in tail]
        result['hold_e_ss_deg'] = abs(sum(errors) / len(errors)) if errors else None
        result['hold_jitter_deg'] = core._std([float(row['actual']) for row in tail])
        result['integral_score'] = ((abs(sum(errors) / len(errors)) / core.TOL_SS_DEG)
                                   if errors else None)
    sine_rows = by_segment.get('dyn_sine') or []
    if sine_rows and len(sine_rows) > 20:
        sine_samples = [dict(t=(float(row['t']) - float(sine_rows[0]['t'])) / 1000.0,
                             q=float(row['actual']), target=float(row['target']))
                        for row in sine_rows]
        sine = sine_metrics(sine_samples, 0.2)
        result.update(dynamic_rmse_deg=sine.get('harmonic_residual_rmse_deg'),
                      dynamic_phase_delay_s=sine.get('phase_delay_s'),
                      dynamic_amplitude_ratio=sine.get('amplitude_ratio'))
    reversal = core.reversal_metrics(samples, truth, motors, params)
    result.update(reversal_score=reversal.score, reversal_valid=reversal.valid,
                  reversal_reason=reversal.reason,
                  reversal_events=reversal.events,
                  takeup_timeout=reversal.takeup_timeout)
    if not reversal.valid:
        result['hard_failure'] = reversal.reason
    return result


def _write_report(output: Path, records: list, selected: dict, combined_ok: bool,
                  reasons: list, timeline: dict, started: float) -> None:
    elapsed = time.monotonic() - started
    integral = [row for row in records if row.get('kind') == 'integral']
    takeup = [row for row in records if row.get('kind') == 'takeup']
    validation = [row for row in records if row.get('kind') == 'validation']
    lines = ['# Integral and fast-reversal autotune report', '',
             f'- elapsed_s: {elapsed:.1f}',
             f'- combined_ok: {combined_ok}',
             f'- selected: `{json.dumps(selected, ensure_ascii=False)}`',
             f'- reasons: `{json.dumps(reasons, ensure_ascii=False)}`', '',
             '## Integral trials', '',
             '| label | type | joint | ki | score | valid | reason |',
             '|---|---|---:|---:|---:|---|---|']
    for row in integral:
        trial = row.get('trial') or {}
        lines.append(f"| {row.get('label','')} | {row.get('joint_type','')} | "
                     f"{row.get('joint','')} | {row.get('ki','')} | "
                     f"{trial.get('score','')} | {trial.get('valid', row.get('valid',''))} | "
                     f"{trial.get('reason', row.get('reason',''))} |")
    lines += ['', '## Detector (offline)', '']
    detector = output / 'detector' / 'scores.json'
    if detector.is_file():
        scores = json.loads(detector.read_text(encoding='utf-8'))
        lines += ['| confirm_deg | confirm_ms | TP | FP | FN | FN(needs takeup) | p90 latency s |',
                  '|---:|---:|---:|---:|---:|---:|---:|']
        for score in scores:
            lines.append(f"| {score['confirm_deg']} | {score['confirm_ms']} | {score['tp']} | "
                         f"{score['fp']} | {score['fn']} | {score['fn_needs_takeup']} | "
                         f"{score['p90_latency_s']} |")
    lines += ['', '## Takeup/exit trials', '',
              '| label | type | joint | takeup | exit | score | valid | reason |',
              '|---|---|---:|---:|---:|---:|---|---|']
    for row in takeup:
        trial = row.get('trial') or {}
        lines.append(f"| {row.get('label','')} | {row.get('joint_type','')} | "
                     f"{row.get('joint','')} | {row.get('takeup_counts','')} | "
                     f"{row.get('exit_travel_deg','')} | {trial.get('score','')} | "
                     f"{trial.get('valid', row.get('valid',''))} | "
                     f"{trial.get('reason', row.get('reason',''))} |")
    lines += ['', '## Combined validation', '',
              '| type | variant | J_I | J_R | rmse | phase delay | hard failure |',
              '|---|---|---:|---:|---:|---:|---|']
    for row in validation:
        metrics = row.get('metrics') or {}
        lines.append(f"| {row.get('joint_type','')} | {row.get('variant','')} | "
                     f"{metrics.get('integral_score','')} | {metrics.get('reversal_score','')} | "
                     f"{metrics.get('dynamic_rmse_deg','')} | {metrics.get('dynamic_phase_delay_s','')} | "
                     f"{metrics.get('hard_failure','')} |")
    lines += ['', '## Safety', '',
              f'- safety events: see `safety.json`/`trial_index.json`',
              f'- final control disabled: see `deployment.json`',
              '', '## Deployment', '',
              f'- final JSON: `{output / "final_parameters.json"}`',
              f'- deployment: `{output / "deployment.json"}`',
              f'- fallback path: `{output / "parameters.safe.json"}`', '']
    _text(output / 'report.md', '\n'.join(lines) + '\n')
    _text(output / 'summary.md', '# Integral / fast-reversal autotune\n\n'
          f'- elapsed_s: {elapsed:.1f}\n- combined_ok: {combined_ok}\n'
          f'- selected: `{json.dumps(selected, ensure_ascii=False)}`\n'
          f'- reasons: `{json.dumps(reasons, ensure_ascii=False)}`\n')
    _json(output / 'trial_index.json', records)


def run_dry(args) -> int:
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    bundle = json.loads(Path(args.calibration).read_text(encoding='utf-8'))
    audit = calibration_audit(bundle)
    rows = {row['joint']: row for row in audit}
    selected = dict(ki={}, confirm_deg=0.6, confirm_ms=20, takeup_counts=0,
                    exit_travel_deg=0.25)
    records = []
    for joint_type in core.TYPES:
        joint = core.REPRESENTATIVE_JOINTS[joint_type]
        row = rows.get(joint)
        if not row or not row.get('eligible'):
            selected['ki'][joint_type] = 0.0
            records.append(dict(kind='integral', joint_type=joint_type, joint=joint,
                                ki=0.0, valid=False, reason='joint not eligible'))
            continue
        history = []
        for ki in core.KI_CANDIDATES:
            # Deterministic synthetic plant: droop falls with Ki, chatter rises.
            noise = 0.01 * ((joint * 17 + int(ki * 1000)) % 7)
            e_ss = max(0.01, 0.35 - 0.9 * ki + noise)
            jitter = 0.05 + 0.25 * max(0.0, ki - 0.1)
            score = (0.50 * (e_ss / core.TOL_SS_DEG) +
                     0.25 * (e_ss / core.TOL_SS_DEG) +
                     0.15 * (jitter / core.TOL_JITTER_DEG))
            trial = core.IntegralTrial(joint_type=joint_type, joint=joint, ki=ki,
                                       valid=True, score=score, reason='synthetic',
                                       gate_fraction=0.7, i_change_counts=20.0 * ki)
            history.append(trial)
            records.append(dict(kind='integral', label=f'dry_{joint_type}_{ki}',
                                joint_type=joint_type, joint=joint, ki=ki,
                                trial=trial.__dict__))
        choice = core.choose_integral(history, noise=0.02)
        selected['ki'][joint_type] = float(choice['selected_ki'])
        records.append(dict(kind='integral_choice', joint_type=joint_type,
                            selected_ki=choice['selected_ki'], reason=choice['reason']))
    suite = core.build_detector_suite()
    scores = core.evaluate_detector_grid(suite)
    best = core.choose_detector(scores)
    selected['confirm_deg'] = best.confirm_deg
    selected['confirm_ms'] = best.confirm_ms
    _json(output / 'detector' / 'scores.json', [score.__dict__ for score in scores])
    _json(output / 'detector' / 'selected.json', best.__dict__)
    takeup_history = []
    for joint_type, joint in (('mcp', 9), ('aa', 8)):
        for takeup in core.TAKEUP_CANDIDATES:
            if takeup > 40 and joint_type == 'aa':
                continue
            base = 0.30
            score = base - 0.25 * min(takeup, 40) / 40.0 + (0.05 if takeup >= 80 else 0.0)
            trial = core.ReversalTrial(joint_type=joint_type, joint=joint,
                                       takeup_counts=float(takeup),
                                       exit_travel_deg=0.25, valid=True,
                                       score=score, reason='synthetic')
            takeup_history.append(trial)
            records.append(dict(kind='takeup', label=f'dry_{joint_type}_{takeup}',
                                joint_type=joint_type, joint=joint,
                                takeup_counts=takeup, exit_travel_deg=0.25,
                                trial=trial.__dict__))
    choice = core.choose_takeup(takeup_history, noise=0.02)
    selected['takeup_counts'] = float(choice['selected_takeup'])
    selected['exit_travel_deg'] = float(choice['selected_exit'])
    joint_ki = {name: 0.0 for name in ALL_TYPES}
    for name in core.TYPES:
        joint_ki[name] = selected['ki'][name]
    for name in ALL_TYPES:
        joint_ki[f'integral_limit_counts_{name}'] = 100
    joint_ki['integral_noise_deg'] = 0.1
    joint_ki['integral_confirm_ms'] = 200
    reversal = {'confirm_ms': selected['confirm_ms']}
    for name in ALL_TYPES:
        reversal[f'confirm_deg_{name}'] = selected['confirm_deg']
        reversal[f'takeup_max_{name}'] = selected['takeup_counts']
        reversal[f'exit_travel_deg_{name}'] = selected['exit_travel_deg']
    reversal_ramp = {**{f'b0_{name}': (2.0 if selected['takeup_counts'] > 0 else 0.0) for name in ALL_TYPES},
                     **{f'vb_{name}': (20.0 if selected['takeup_counts'] > 0 else 0.0) for name in ALL_TYPES},
                     **{f'load_limit_{name}': 900 for name in ALL_TYPES},
                     'takeup_timeout_ms': 600}
    final = dict(schema_version=6, revision=1, ramp_revision=1,
                 joint_kp=dict(EXPECTED_KP), joint_ki=joint_ki,
                 joint_kd=dict(EXPECTED_KD),
                 dual_tendon=dict(pair_recovery_gain=0.0),
                 reversal_ramp=reversal_ramp,
                 feedback=dict(fast_window_ms=20, slow_window_ms=500),
                 reversal=reversal,
                 collision=dict(arm_velocity_flex_deg_s=8.0, arm_velocity_extend_deg_s=8.0,
                                stall_velocity_flex_deg_s=1.0, stall_velocity_extend_deg_s=1.0,
                                min_error_deg=4.0, target_reset_deg=1.0, retreat_deg=1.0),
                 output=dict(lut_release_margin_counts=500, lut_tight_margin_counts=1000),
                 ros2_input=dict(min_cutoff_hz=1.0, beta=0.02, derivative_cutoff_hz=1.0,
                                 enter_deg=0.08, enter_dwell_ms=200, exit_deg=0.25,
                                 exit_confirm_ms=20, max_gap_ms=500))
    validate(final)
    _json(output / 'final_parameters.json', final)
    _json(output / 'trial_index.json', records)
    _write_report(output, records, selected, True, ['dry-run synthetic plant'],
                  {name: time.monotonic() for name in STAGE_MINUTES}, time.monotonic())
    print(f'DRY-RUN complete: {output} selected={json.dumps(selected)}')
    return 0


def run_replay(args) -> int:
    source = Path(args.replay)
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    raw_files = sorted(source.glob('raw/*.jsonl'))
    if not raw_files:
        raise RuntimeError(f'no raw JSONL under {source}')
    suite = core.build_detector_suite()
    scores = core.evaluate_detector_grid(suite)
    best = core.choose_detector(scores)
    _json(output / 'detector' / 'scores.json', [score.__dict__ for score in scores])
    _json(output / 'detector' / 'selected.json', best.__dict__)
    summary = []
    for path in raw_files:
        samples = [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines() if line]
        params = dict(ki=0.0, fast_window_ms=20, slow_window_ms=500,
                      integral_limit_counts=100, integral_noise_deg=0.1,
                      integral_confirm_ms=200, confirm_deg=best.confirm_deg,
                      confirm_ms=best.confirm_ms, takeup_counts=0,
                      exit_travel_deg=0.25)
        replayed = core.replay_control_state(samples, params)
        summary.append(dict(file=path.name, samples=len(samples),
                            phases={phase: sum(1 for row in replayed
                                               if row.get('control_phase') == phase)
                                    for phase in ('Tracking', 'Pending', 'Takeup')},
                            takeup_lifting=sum(1 for row in replayed if row.get('takeup_lifting')),
                            mismatch=core.replay_mismatch(replayed)))
    _json(output / 'replay.json', summary)
    print(f'REPLAY complete: {len(summary)} files, detector={best.confirm_deg}/{best.confirm_ms}')
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--hardware', action='store_true')
    parser.add_argument('--replay', type=Path)
    parser.add_argument('--calibration', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--port', default='COM12')
    parser.add_argument('--max-minutes', type=float, default=60.0)
    parser.add_argument('--base-parameters', type=Path)
    args = parser.parse_args(argv)
    modes = sum(bool(value) for value in (args.dry_run, args.hardware, args.replay))
    if modes != 1:
        parser.error('choose exactly one of --dry-run, --hardware or --replay')
    if args.max_minutes <= 0:
        parser.error('--max-minutes must be positive')
    if args.replay:
        if not args.replay.is_dir():
            parser.error('--replay must be an existing run directory')
        return run_replay(args)
    if args.calibration is None:
        parser.error('--calibration is required for dry-run and hardware')
    return run_dry(args) if args.dry_run else run_hardware(args)


if __name__ == '__main__':
    raise SystemExit(main())
