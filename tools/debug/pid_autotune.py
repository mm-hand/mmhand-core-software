"""pid_autotune.py - 一小时 PID 自动整定入口（dry-run 与 hardware）。

用法：
  python tools/debug/pid_autotune.py --dry-run --calibration ... --output ...
  python tools/debug/pid_autotune.py --hardware --port COM12 --calibration ... --output ...

hardware 模式只通过正常桌面授权、参数 Apply 和直接关节目标流工作；不绕过急停、
碰撞、软限位或反馈超时保护。真机预算由 --max-minutes 硬限制，默认 60。
"""
from __future__ import annotations

import argparse
import copy
from dataclasses import asdict
import json
import math
from pathlib import Path
import random
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from desktop.core.control_parameters import CURRENT_PATH  # noqa: E402
from desktop.core.control_tuning import calibration_audit  # noqa: E402
from tools.debug.pid_autotune_core import (  # noqa: E402
    DYNAMIC_BINS, POSITION_BINS, DiscreteEI, GridPoint, ScoreThresholds,
    build_composite_trajectory, evaluate_trial,
)

TYPES = ('aa', 'mcp', 'pip', 'dip')
REPRESENTATIVES = {'aa': 8, 'mcp': 9, 'pip': 10, 'dip': 11}
KP_RANGE = {'aa': (0.7, 1.3), 'mcp': (0.7, 1.3), 'pip': (0.7, 1.3), 'dip': (0.9, 1.5)}
KD_VALUES = (0.0, 0.02, 0.04, 0.06, 0.08)


def grid_for(joint_type: str) -> list[GridPoint]:
    lo, hi = KP_RANGE[joint_type]
    kp_values = [round(lo + i * 0.1, 3) for i in range(int(round((hi - lo) / 0.1)) + 1)]
    return [GridPoint(kp, kd) for kp in kp_values for kd in KD_VALUES]


def initial_points(joint_type: str, base_kp: float) -> list[GridPoint]:
    lo, hi = KP_RANGE[joint_type]
    def clamp(value):
        return max(lo, min(hi, round(value, 3)))
    return [GridPoint(clamp(base_kp), 0.0), GridPoint(clamp(base_kp - 0.1), 0.02),
            GridPoint(clamp(base_kp + 0.1), 0.02), GridPoint(clamp(base_kp), 0.06)]


def _plot_trial_jsonl(source: Path, target: Path) -> bool:
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception:
        return False
    rows = [json.loads(line) for line in source.read_text(encoding='utf-8').splitlines() if line]
    if not rows:
        return False
    t = [row['time'] for row in rows]
    target_deg = [row['target'] for row in rows]
    actual_deg = [row['actual'] for row in rows]
    error = [a - b for a, b in zip(actual_deg, target_deg)]
    motor = [row.get('motor_cmd', 0) for row in rows]
    figure, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    axes[0].plot(t, target_deg, label='target', linewidth=1.0)
    axes[0].plot(t, actual_deg, label='actual', linewidth=1.0)
    axes[0].set_ylabel('angle (deg)')
    axes[0].legend(loc='best')
    axes[1].plot(t, error, color='tab:red', linewidth=0.9)
    axes[1].set_ylabel('error (deg)')
    axes[2].plot(t, motor, color='tab:green', linewidth=0.9)
    axes[2].set_ylabel('motor (counts)')
    axes[2].set_xlabel('P4 trace time (s)')
    figure.tight_layout()
    figure.savefig(target, dpi=120)
    plt.close(figure)
    return True


def _write_curves(raw_dir: Path) -> int:
    count = 0
    for source in sorted(raw_dir.glob('*.jsonl')):
        if _plot_trial_jsonl(source, source.with_suffix('.png')):
            count += 1
    return count


def _write_report(output: Path, records: list[dict], thresholds: ScoreThresholds, elapsed_s: float):
    output.mkdir(parents=True, exist_ok=True)
    (output / 'trial_index.json').write_text(json.dumps(records, indent=2), encoding='utf-8')
    lines = ['# PID autotune report', '',
             f'- elapsed_s: {elapsed_s:.1f}',
             f'- thresholds: `{json.dumps(asdict(thresholds))}`', '',
             '| type | kp | kd | score | valid | reason |', '|---|---:|---:|---:|---|---|']
    for row in records:
        lines.append(f"| {row['type']} | {row['kp']:.3f} | {row['kd']:.3f} | "
                     f"{row['score'] if row['score'] is not None else float('nan'):.4f} | "
                     f"{row['valid']} | {row['reason']} |")
    best = {}
    for row in records:
        if row['valid'] and (row['type'] not in best or row['score'] < best[row['type']]['score']):
            best[row['type']] = row
    lines += ['', '## Best', '', '| type | kp | kd | score |', '|---|---:|---:|---:|']
    for joint_type in TYPES:
        row = best.get(joint_type)
        if row:
            lines.append(f"| {joint_type} | {row['kp']:.3f} | {row['kd']:.3f} | {row['score']:.4f} |")
    (output / 'report.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')


def dry_score(joint_type: str, point: GridPoint, trajectory, seed: int) -> tuple:
    rng = random.Random(f'{joint_type}:{point.kp}:{point.kd}:{seed}')
    rows = []
    def error(speed_bin, position, direction):
        # Deterministic synthetic plant: higher Kp improves tracking, excessive Kd adds noise.
        base = 0.9 / (point.kp + 0.2)
        base += 0.035 * abs(point.kd - 0.04)
        base += 0.20 if speed_bin == 'high' else 0.0
        base += 0.10 * abs(position - 0.5)
        return base * (1.0 if direction == 'flex' else 0.8) + rng.uniform(-0.02, 0.02)
    for item in trajectory:
        if item.phase == 'dynamic':
            rows.append({'phase': 'dynamic', 'position_bin': item.position_bin,
                         'speed_bin': item.speed_bin, 'direction': item.direction,
                         'target': item.target_deg,
                         'actual': item.target_deg + error(item.speed_bin, item.position_bin, item.direction)})
        elif item.phase == 'hold':
            rows.append({'phase': 'hold', 'position_bin': item.position_bin, 'time': item.time_s,
                         'target': item.target_deg,
                         'actual': item.target_deg + 0.12 / (point.kp + 0.1) + rng.uniform(-0.015, 0.015),
                         'motor_cmd': 1000 + int(5 * abs(point.kd - 0.04) * 100)})
    return evaluate_trial(rows, ScoreThresholds())


def run_dry(args) -> int:
    bundle = json.loads(Path(args.calibration).read_text(encoding='utf-8'))
    audit = calibration_audit(bundle)
    by_joint = {row['joint']: row for row in audit}
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    records = []
    started = time.monotonic()
    for joint_type in getattr(args, 'types', TYPES):
        joint = REPRESENTATIVES[joint_type]
        row = by_joint.get(joint)
        if not row or not row.get('eligible'):
            records.append(dict(type=joint_type, kp=0, kd=0, score=None, valid=False,
                                reason=f'J{joint:02d} not eligible'))
            continue
        trajectory = build_composite_trajectory(row['internal_low_deg'], row['internal_high_deg'])
        optimizer = DiscreteEI(grid_for(joint_type), initial_points(joint_type, 1.2 if joint_type == 'dip' else 1.0))
        for trial in range(args.max_trials):
            point = optimizer.suggest()
            if point is None:
                break
            score = dry_score(joint_type, point, trajectory, trial)
            optimizer.observe(point, score.score, score.valid, '; '.join(score.failures))
            records.append(dict(type=joint_type, kp=point.kp, kd=point.kd, score=score.score,
                                valid=score.valid, reason='; '.join(score.failures),
                                dynamic=score.dynamic_bins, hold=score.hold_bins))
    _write_report(output, records, ScoreThresholds(), time.monotonic() - started)
    print(f'DRY-RUN complete: {len(records)} trials, report {output / "report.md"}')
    return 0


def _wait(predicate, timeout, message):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(0.02)
    raise RuntimeError(message)


def _operation(hand, request, timeout=25):
    if not request.accepted:
        raise RuntimeError(request.message)
    if request.operation_id is None:
        return
    def done():
        row = next((r for r in hand._application_operations if r['operation_id'] == request.operation_id), None)
        return row is not None and row['status'] in ('SUCCESS', 'FAILED')
    _wait(done, timeout, 'operation timeout')
    row = next(r for r in hand._application_operations if r['operation_id'] == request.operation_id)
    if row['status'] == 'FAILED':
        raise RuntimeError(row['message'])


def _release(hand):
    if not hand.is_comm_connected():
        return
    hand.comm.send_command('control_release')
    _wait(lambda: not hand.get_state_snapshot().control_enabled, 3, 'release timeout')


def _apply(hand, proposal):
    _release(hand)
    _operation(hand, hand.request_save_control_parameters(proposal))
    _wait(lambda: hand._control_parameter_sync.startswith('APPLIED'), 15, 'parameter apply timeout')
    _wait(lambda: not hand._application_busy, 3, 'application busy')


def _healthy(hand, joint, motor_mask):
    state = hand.get_state_snapshot()
    if not hand._fresh_application_state():
        raise RuntimeError('stale application state')
    if not state.control_enabled:
        raise RuntimeError('control disabled')
    if state.feedback_collision_hold_bitmap & (1 << joint):
        raise RuntimeError('collision hold')
    if time.time() - state.control_trace_last_rx_ts > 0.15:
        raise RuntimeError('control trace stale')
    if state.control_trace_valid_joint_mask & (1 << joint) == 0:
        raise RuntimeError('selected encoder invalid')
    if state.multi_turn_unknown_bitmap & motor_mask:
        raise RuntimeError('multi-turn unknown')
    return state


def _set_target(hand, values, joint, target, source):
    values = list(values)
    values[joint] = target
    result = hand.send_joint_targets(values, source=source, live=True, command_joint_mask=1 << joint)
    if not result.sent or result.clamped:
        raise RuntimeError(result.message)
    return values


def run_hardware(args) -> int:
    from desktop.core.hand_backend import HandBackend
    from desktop.core.host_calibration.config import joint_motors

    bundle = json.loads(Path(args.calibration).read_text(encoding='utf-8'))
    audit = {row['joint']: row for row in calibration_audit(bundle)}
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    hand = HandBackend()
    hand._save_data = False
    hand.open_application()
    if not hand.connect(args.port):
        raise RuntimeError(f'failed to connect {args.port}')
    inspected = hand._calibration_workflow.inspect_snapshot(Path(args.calibration))
    ok, message = hand._controller.load_calibration_snapshot(inspected)
    if not ok:
        raise RuntimeError(f'calibration load failed: {message}')
    _wait(lambda: hand._control_parameter_sync.startswith('APPLIED'), 15, 'calibration parameter sync timeout')
    current = hand._control_parameters.snapshot()
    current.pop('crc32', None)
    verified = copy.deepcopy(current)
    _operation(hand, hand.request_clear_faults(), 10)
    (output / 'parameters.before.json').write_text(json.dumps(current, indent=2), encoding='utf-8')
    started = time.monotonic()
    deadline = started + args.max_minutes * 60.0
    records = []
    raw_dir = output / 'raw'
    raw_dir.mkdir(parents=True, exist_ok=True)
    try:
        for joint_type in TYPES:
            if joint_type not in args.types:
                continue
            if time.monotonic() >= deadline - 60:
                break
            joint = REPRESENTATIVES[joint_type]
            row = audit.get(joint)
            if not row or not row.get('eligible'):
                records.append(dict(type=joint_type, kp=0, kd=0, score=None, valid=False,
                                    reason=f'J{joint:02d} not eligible'))
                continue
            motors = joint_motors(joint)
            motor_mask = sum(1 << motor for motor in motors)
            trajectory = build_composite_trajectory(row['internal_low_deg'], row['internal_high_deg'])
            optimizer = DiscreteEI(grid_for(joint_type), initial_points(joint_type, 1.2 if joint_type == 'dip' else 1.0))
            trials_this_type = 0
            consecutive_failures = 0
            while trials_this_type < args.max_trials and time.monotonic() < deadline - 60:
                point = optimizer.suggest()
                if point is None:
                    break
                proposal = copy.deepcopy(verified)
                proposal['joint_kp'][joint_type] = point.kp
                proposal['joint_kd'][joint_type] = point.kd
                proposal['joint_ki'][joint_type] = 0.0
                proposal['dual_tendon']['pair_recovery_gain'] = 0.0
                for _name in ('aa', 'mcp', 'pip', 'dip', 'cmc'):
                    proposal['reversal'][f'takeup_counts_{_name}'] = 0
                try:
                    _apply(hand, proposal)
                    hand.select_joints([joint])
                    hand.select_motors(list(motors))
                    _operation(hand, hand.request_enable())
                    _wait(lambda: hand.get_state_snapshot().control_enabled, 5, 'enable timeout')
                    hand.request_recording(True)
                    _wait(lambda: hand._recorder.is_running(), 5, 'recording not running')
                    hand.set_control_trace_subscription(True, 1)
                    _wait(lambda: hand.get_state_snapshot().has_control_trace and
                          time.time() - hand.get_state_snapshot().control_trace_last_rx_ts < 0.15,
                          3, 'control trace not ready')
                    samples = []
                    try:
                        state = _healthy(hand, joint, motor_mask)
                        values = list(state.target_angles)
                        start = state.encoders[joint].current_angle_deg
                        target = row['internal_low_deg'] + 0.10 * (row['internal_high_deg'] - row['internal_low_deg'])
                        approach_start = time.monotonic()
                        approach_duration = max(1.0, min(8.0, abs(target - start) / 5.0))
                        while time.monotonic() - approach_start < approach_duration:
                            fraction = min(1.0, (time.monotonic() - approach_start) / approach_duration)
                            values = _set_target(hand, values, joint, start + (target - start) * fraction, 'autotune approach')
                            time.sleep(0.02)
                        _healthy(hand, joint, motor_mask)
                        trajectory_start = time.monotonic()
                        p4_start = hand.get_state_snapshot().control_trace_p4_time_ms
                        last_sequence = -1
                        for item in trajectory:
                            while time.monotonic() - trajectory_start < item.time_s:
                                time.sleep(0.002)
                            values = _set_target(hand, values, joint, item.target_deg, 'autotune trajectory')
                            state = _healthy(hand, joint, motor_mask)
                            if state.control_trace_sequence == last_sequence:
                                continue
                            last_sequence = state.control_trace_sequence
                            elapsed = ((state.control_trace_p4_time_ms - p4_start) & 0xFFFFFFFF) * 0.001
                            actual = state.control_trace_actual_deg[joint]
                            if not row['internal_low_deg'] - 2.0 <= actual <= row['internal_high_deg'] + 2.0:
                                raise RuntimeError('measured angle outside test envelope')
                            samples.append({'phase': item.phase, 'position_bin': item.position_bin,
                                            'speed_bin': item.speed_bin, 'direction': item.direction,
                                            'time': elapsed, 'target': state.control_trace_target_deg[joint],
                                            'actual': actual, 'motor_cmd': state.control_trace_motor_command_abs[motors[0]]})
                    finally:
                        _release(hand)
                        hand.request_recording(False)
                        hand.set_control_trace_subscription(False, 5)
                    score = evaluate_trial(samples, ScoreThresholds())
                    raw_path = raw_dir / f'{joint_type}_J{joint:02d}_{trials_this_type + 1:02d}.jsonl'
                    raw_path.write_text('\n'.join(json.dumps(row) for row in samples) + '\n', encoding='utf-8')
                    optimizer.observe(point, score.score, score.valid, '; '.join(score.failures))
                    records.append(dict(type=joint_type, kp=point.kp, kd=point.kd, score=score.score,
                                        valid=score.valid, reason='; '.join(score.failures),
                                        dynamic=score.dynamic_bins, hold=score.hold_bins))
                    print(f"{joint_type} trial {trials_this_type + 1}/{args.max_trials} "
                          f"J{joint:02d} Kp={point.kp:.3f} Kd={point.kd:.3f} "
                          f"score={score.score:.4f} valid={score.valid} "
                          f"bins={score.dynamic_bins}/{score.hold_bins} {('; '.join(score.failures))}",
                          flush=True)
                    consecutive_failures = 0
                except RuntimeError as exc:
                    consecutive_failures += 1
                    reason = str(exc)
                    optimizer.observe(point, None, False, reason)
                    records.append(dict(type=joint_type, kp=point.kp, kd=point.kd, score=None,
                                        valid=False, reason=reason, dynamic=0, hold=0))
                    print(f"{joint_type} trial {trials_this_type + 1}/{args.max_trials} "
                          f"J{joint:02d} Kp={point.kp:.3f} Kd={point.kd:.3f} FAILED: {reason}",
                          flush=True)
                    try:
                        _apply(hand, verified)
                    except Exception:
                        pass
                trials_this_type += 1
                hand.select_joints([])
                hand.select_motors([])
                if consecutive_failures >= 3:
                    records.append(dict(type=joint_type, kp=0, kd=0, score=None, valid=False,
                                        reason='three consecutive candidate failures'))
                    break
            best = optimizer.best
            if best:
                verified['joint_kp'][joint_type] = best.point.kp
                verified['joint_kd'][joint_type] = best.point.kd
                records.append(dict(type=joint_type, phase='best', kp=best.point.kp,
                                    kd=best.point.kd, score=best.score, valid=True, reason='selected'))
        _apply(hand, verified)
        final_parameters = hand._control_parameters.snapshot()
        (output / 'parameters.verified.json').write_text(json.dumps(verified, indent=2), encoding='utf-8')
    finally:
        try:
            _release(hand)
            hand.disconnect(stop_lower=False)
        finally:
            hand.close_application()
    _write_report(output, [row for row in records if row.get('phase') != 'best'], ScoreThresholds(), time.monotonic() - started)
    curves = _write_curves(raw_dir)
    (output / 'parameters.after.json').write_text(json.dumps(final_parameters, indent=2), encoding='utf-8')
    print(f'HARDWARE complete: {len(records)} records, {curves} curves, report {output / "report.md"}')
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--hardware', action='store_true')
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--port', default='COM12')
    parser.add_argument('--max-minutes', type=float, default=60.0)
    parser.add_argument('--max-trials', type=int, default=11)
    parser.add_argument('--types', default=','.join(TYPES), help='comma-separated subset of aa,mcp,pip,dip')
    args = parser.parse_args(argv)
    if args.dry_run == args.hardware:
        parser.error('choose exactly one of --dry-run or --hardware')
    if args.max_minutes <= 0 or args.max_trials <= 0:
        parser.error('max values must be positive')
    selected_types = tuple(name.strip().lower() for name in args.types.split(',') if name.strip())
    if not selected_types or any(name not in TYPES for name in selected_types):
        parser.error(f'--types must be a subset of {",".join(TYPES)}')
    args.types = selected_types
    return run_dry(args) if args.dry_run else run_hardware(args)


if __name__ == '__main__':
    raise SystemExit(main())
