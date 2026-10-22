"""Find a takeup ramp that actually engages, then refine B0/vB.

Hardware mode uses the normal desktop authorization, full main+ramp parameter
Apply, direct joint targets and the normal recorder. It never bypasses STOP,
collision, soft limits or feedback timeouts.

R1 probes the bias cap Bmax with B0=5 / vB=50 counts per second.  The P4 applies
``b = min(Bmax, B0 + vB * t)`` with a 600 ms take-up timeout, so every ascending
Bmax above ``B0 + vB * timeout`` produces the identical bias within one event;
those equivalent rungs are skipped instead of repeating the same load.  If R1
engages but cannot reach the 0.25 deg exit, R2 retries the effective cap with a
smaller B0 and different vB values before the type is declared infeasible.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import shutil
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from desktop.core import application_actions  # noqa: E402
from desktop.core.control_parameters import configuration_crc, validate  # noqa: E402
from desktop.core.host_calibration.config import joint_motors  # noqa: E402
from tools.debug import integral_reversal_core as core  # noqa: E402
from tools.debug.integral_reversal_autotune import (  # noqa: E402
    ALL_TYPES, EXPECTED_KD, EXPECTED_KP, PreflightBlocked, SafetyAbort,
    StopRequested, _analysis_params, _apply_verified, _hold_for, _lead_in_plan,
    _release, _run_plan, _safe_snapshot, _set_all_pair, _set_all_ramp,
    _set_all_reversal, _set_integral_defaults, _wait, index_aa_clear,
)

LADDER = (40, 80, 160, 320, 640, 1000, 1500, 2000)
VBS = (20, 50, 100)
DEFAULT_LOAD_LIMIT = 900
TIMEOUT_MS = 600
EXIT_TRAVEL_DEG = 0.25
CONFIRM_MS = 20
REFINE_B0 = 2.0
# Frozen, validated gains. Tuning at the MCP troubleshooting Kp=0.2 would make
# the scores meaningless, so every proposal starts from these values.
EXPECTED_CONFIRM_DEG = {'aa': 0.6, 'mcp': 1.0, 'pip': 1.0, 'dip': 1.0, 'cmc': 1.0}
# The validated AA recording engages 10.1% of same-cycle samples (old fixed
# floor was 3.6%); 5% distinguishes a real ramp from a handful of samples.
ENGAGE_MIN = 0.05
EXIT_MIN = 0.8
RANGES = {'aa': (-25.8, 23.8), 'mcp': (-3.6, 74.0),
          'pip': (0.8, 79.2), 'dip': (-1.6, 89.6)}
FORBIDDEN_KEYS = ('crc32', 'ramp_crc32')


def _json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, default=str) + '\n',
                    encoding='utf-8')


def _now() -> str:
    return datetime.now().isoformat(timespec='seconds')


def _make_safe(snapshot: dict, confirm_deg: float | None = None,
               types=(), pair_gain: float = 0.0) -> dict:
    """Deterministic tuning base: validated gains, no compensation, requested pairing."""
    safe = copy.deepcopy(snapshot)
    for key in FORBIDDEN_KEYS:
        safe.pop(key, None)
    safe['joint_kp'] = dict(EXPECTED_KP)
    safe['joint_kd'] = dict(EXPECTED_KD)
    safe['joint_ki'] = {name: 0.0 for name in ALL_TYPES}
    _set_integral_defaults(safe)
    safe['reversal']['confirm_ms'] = CONFIRM_MS
    for name in ALL_TYPES:
        safe['reversal'][f'confirm_deg_{name}'] = EXPECTED_CONFIRM_DEG[name]
        safe['reversal'][f'takeup_max_{name}'] = 0.0
        safe['reversal'][f'exit_travel_deg_{name}'] = EXIT_TRAVEL_DEG
    if confirm_deg is not None:
        for name in types:
            if name in ALL_TYPES:
                safe['reversal'][f'confirm_deg_{name}'] = float(confirm_deg)
    _set_all_pair(safe, float(pair_gain))
    _set_all_ramp(safe, 0.0, 0.0, DEFAULT_LOAD_LIMIT)
    validate(safe)
    return safe


def _metrics(samples, truth, motors, params):
    trial = core.reversal_metrics(samples, truth, motors, params)
    rev = [abs(int(row.get('reversal_counts', 0))) for row in samples]
    engage = sum(1 for value in rev if value > 0) / len(rev) if rev else 0.0
    motor_set = {int(motor) for motor in motors}
    peak_load = 0
    for row in samples:
        for motor, value in (row.get('load') or {}).items():
            # Held proximal joints can be stiff; only the tested joint's load
            # decides whether this ramp candidate is safe and effective.
            if int(motor) not in motor_set:
                continue
            peak_load = max(peak_load, abs(int(value)))
    valid_events = sum(1 for event in trial.events if event.get('valid'))
    exit_rate = valid_events / len(truth) if truth else 0.0
    return dict(trial=trial, engage=engage, exit_rate=exit_rate, peak_load=peak_load,
                applied_max=max(rev) if rev else 0, applied_mean=statistics.mean(rev) if rev else 0)


def _feasible(row: dict, load_limit: float = DEFAULT_LOAD_LIMIT) -> bool:
    """A candidate is deployable only when it engaged, answered and stayed loaded."""
    return bool(row.get('valid')
                and float(row.get('exit_rate') or 0.0) >= EXIT_MIN
                and float(row.get('engage') or 0.0) >= ENGAGE_MIN
                and int(row.get('peak_load') or 0) < int(load_limit))


def _label(joint_type, bmax, vb, b0):
    return f'{joint_type}_max{int(bmax)}_vb{int(vb)}_b0{int(b0)}'


def _proposal(safe: dict, joint_type: str, bmax: float, b0: float, vb: float,
              load_limit: float, timeout_ms: float) -> dict:
    proposal = copy.deepcopy(safe)
    _set_all_reversal(proposal, 'takeup_max', 0.0)
    proposal['reversal'][f'takeup_max_{joint_type}'] = float(bmax)
    _set_all_ramp(proposal, 0.0, 0.0, load_limit)
    proposal['reversal_ramp'][f'b0_{joint_type}'] = float(b0)
    proposal['reversal_ramp'][f'vb_{joint_type}'] = float(vb)
    proposal['reversal_ramp'][f'load_limit_{joint_type}'] = float(load_limit)
    proposal['reversal_ramp']['takeup_timeout_ms'] = float(timeout_ms)
    validate(proposal)
    return proposal


def _angle(state, joint: int):
    try:
        value = float(state.encoders[joint].current_angle_deg)
    except (AttributeError, IndexError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _state_summary(state, assume_index_aa_clear: bool = False) -> dict:
    angles = {f'J{joint:02d}': _angle(state, joint) for joint in (4, 8, 9, 10, 11, 12)}
    errors = {f'J{joint:02d}': bool(getattr(state.encoders[joint], 'angle_error', True))
              for joint in (4, 8, 9, 10, 11, 12)}
    raws = {f'J{joint:02d}': int(getattr(state.encoders[joint], 'raw', 0))
            for joint in (4, 8, 9, 10, 11, 12)}
    zeros = {f'J{joint:02d}': int(getattr(state.encoders[joint], 'zero_raw', 0))
             for joint in (4, 8, 9, 10, 11, 12)}
    motors = sorted({motor for joint in (8, 9, 10, 11) for motor in joint_motors(joint)})
    loads = {f'M{motor:02d}': int(state.servo_load[motor])
             for motor in motors if motor < len(state.servo_load)}
    online = {f'M{motor:02d}': bool(state.servo_online_bitmap & (1 << motor))
              for motor in motors}
    cap = core.TEST_HIGH_CAP_DEG.get('mcp')
    mcp_angle = angles.get('J09')
    index_aa_readable = not errors.get('J12')
    return dict(
        angles_deg=angles, angle_error=errors, raw=raws, zero_raw=zeros,
        loads=loads, servo_online=online,
        control_enabled=bool(state.control_enabled),
        system_fault_bitmap=int(state.system_fault_bitmap),
        multi_turn_unknown_bitmap=int(state.multi_turn_unknown_bitmap),
        index_aa_readable=bool(index_aa_readable),
        ready_index_aa=bool(index_aa_clear(state, assume_index_aa_clear)),
        ready_mcp=bool(mcp_angle is not None and cap is not None and mcp_angle <= cap
                       and not errors.get('J09')),
        ready_faults=bool(int(state.system_fault_bitmap) == 0),
        ready_control_off=bool(not state.control_enabled))


def _trial_row(hand, output, joint, joint_type, motors, plan, truth, low, high,
               proposal, label, speed, assume_index_aa_clear=False) -> dict:
    applied = _apply_verified(hand, proposal)
    # A long lead-in at the test speed can overshoot the low envelope after a
    # previous trial ends near the high center.  Approach at 2 deg/s; the test
    # plan itself still uses the requested --speed.
    points = _lead_in_plan(hand, joint, plan, speed_deg_s=2.0)
    revision = int(_safe_snapshot(hand)['revision'])
    crc = configuration_crc(proposal)
    samples, duration = _run_plan(hand, output, joint, motors, points, label,
                                  low, high, revision, crc, hold_joints=_hold_for(joint),
                                  assume_index_aa_clear=assume_index_aa_clear)
    params = _analysis_params(proposal, joint_type, 0.0)
    metrics = _metrics(samples, truth, motors, params)
    trial = metrics['trial']
    proposal_ramp = proposal['reversal_ramp']
    return dict(
        type=joint_type, joint=int(joint), label=label,
        bmax=float(proposal['reversal'][f'takeup_max_{joint_type}']),
        b0=float(proposal_ramp[f'b0_{joint_type}']),
        vb=float(proposal_ramp[f'vb_{joint_type}']),
        load_limit=float(proposal_ramp[f'load_limit_{joint_type}']),
        timeout_ms=float(proposal_ramp['takeup_timeout_ms']),
        confirm_deg=float(params['confirm_deg']), exit_travel_deg=float(params['exit_travel_deg']),
        applied=applied, revision=revision, crc=crc,
        valid=bool(trial.valid), score=trial.score,
        engage=float(metrics['engage']), exit_rate=float(metrics['exit_rate']),
        peak_load=int(metrics['peak_load']), applied_max=int(metrics['applied_max']),
        applied_mean=float(metrics['applied_mean']), reason=str(trial.reason),
        duration=float(duration))


def _append(output: Path, results: list, row: dict) -> None:
    results.append(row)
    _json(output / 'trial_index.json', results)


def _reapply_safe(hand, safe: dict) -> None:
    try:
        _apply_verified(hand, safe)
    except Exception as exc:
        print(f'[ramp] safe re-apply failed: {exc}', flush=True)


def _refine(hand, output, safe, joint, joint_type, motors, plan, truth, low, high,
            anchor: dict, args, results: list, deadline: float,
            best: dict | None, best_key: tuple | None):
    if args.no_refine:
        return best, best_key
    bmax = anchor['bmax']
    for vb in VBS:
        if abs(float(vb) - float(args.vb)) < 1e-9:
            continue
        if time.monotonic() > deadline - 30:
            print('[ramp] deadline reached during refinement', flush=True)
            break
        proposal = _proposal(safe, joint_type, bmax, REFINE_B0, vb,
                             DEFAULT_LOAD_LIMIT, args.timeout_ms)
        label = _label(joint_type, bmax, vb, REFINE_B0)
        try:
            row = _trial_row(hand, output, joint, joint_type, motors, plan, truth,
                             low, high, proposal, label, args.speed,
                             args.assume_index_aa_clear)
        except (StopRequested, PreflightBlocked, SafetyAbort):
            raise
        except Exception as exc:
            _append(output, results, dict(type=joint_type, joint=int(joint), bmax=bmax,
                                          b0=REFINE_B0, vb=float(vb), valid=False,
                                          reason=f'{type(exc).__name__}: {exc}'))
            print('R2 FAIL', label, exc, flush=True)
            _reapply_safe(hand, safe)
            continue
        _append(output, results, row)
        print('R2', json.dumps(row, ensure_ascii=False), flush=True)
        if not _feasible(row):
            continue
        key = (row['score'] if row['score'] is not None else 1e9, row['applied_max'])
        if best is None or best_key is None or key < best_key:
            best, best_key = row, key
    return best, best_key


def _tune_type(hand, output: Path, safe: dict, joint_type: str, args,
               results: list, deadline: float) -> dict:
    joint = core.REPRESENTATIVE_JOINTS[joint_type]
    motors = joint_motors(joint)
    low, high = RANGES[joint_type]
    plan, truth = core.build_reversal_plan(
        low, high, delta_deg=min(4.0, 0.07 * (high - low)),
        speed_deg_s=args.speed, speeds=(('low', args.speed),),
        high_cap=core.TEST_HIGH_CAP_DEG.get(joint_type))
    reachable = float(args.b0) + float(args.vb) * float(args.timeout_ms) / 1000.0
    feasible = None
    probe = None
    failures = 0
    tested_equivalent = False
    for bmax in args.ladder:
        if time.monotonic() > deadline - 45:
            print('[ramp] deadline reached', flush=True)
            break
        equivalent = float(bmax) >= reachable
        if equivalent and tested_equivalent:
            print(f'[ramp] skip {joint_type} Bmax={bmax}: above B0+vB*t ({reachable:.1f})',
                  flush=True)
            continue
        proposal = _proposal(safe, joint_type, bmax, args.b0, args.vb,
                             DEFAULT_LOAD_LIMIT, args.timeout_ms)
        label = _label(joint_type, bmax, args.vb, args.b0)
        try:
            row = _trial_row(hand, output, joint, joint_type, motors, plan, truth,
                             low, high, proposal, label, args.speed,
                             args.assume_index_aa_clear)
        except (StopRequested, PreflightBlocked, SafetyAbort):
            raise
        except Exception as exc:
            _append(output, results, dict(type=joint_type, joint=int(joint), bmax=float(bmax),
                                          b0=args.b0, vb=args.vb, valid=False,
                                          reason=f'{type(exc).__name__}: {exc}'))
            print('R1 FAIL', label, exc, flush=True)
            _reapply_safe(hand, safe)
            failures += 1
            if failures >= 2:
                break
            continue
        if equivalent:
            tested_equivalent = True
        _append(output, results, row)
        print('R1', json.dumps(row, ensure_ascii=False), flush=True)
        if _feasible(row):
            feasible = row
            break
        if row['engage'] >= ENGAGE_MIN and probe is None:
            probe = row
        failures = failures + 1 if not row['valid'] else 0
        if failures >= 2:
            break

    best = feasible
    best_key = ((feasible['score'] if feasible and feasible['score'] is not None else 1e9),
                feasible['applied_max'] if feasible else 0)
    if feasible is not None:
        best, best_key = _refine(hand, output, safe, joint, joint_type, motors, plan, truth,
                                 low, high, feasible, args, results, deadline, best, best_key)
    elif probe is not None:
        # R1 engaged but did not exit: try faster growth at the same effective
        # cap instead of climbing Bmax, which cannot change the bias within the
        # 600 ms timeout.
        best, best_key = _refine(hand, output, safe, joint, joint_type, motors, plan, truth,
                                 low, high, probe, args, results, deadline, None, None)
    feasible = best if best is not None and _feasible(best) else None
    if feasible is None:
        return dict(takeup_max=0.0, b0=0.0, vb=0.0, load_limit=DEFAULT_LOAD_LIMIT,
                    confirm_deg=float(args.confirm_deg or EXPECTED_CONFIRM_DEG[joint_type]),
                    timeout_ms=float(args.timeout_ms), deployed=False,
                    reason='no feasible ramp within load limit')
    return dict(takeup_max=float(feasible['bmax']), b0=float(feasible['b0']),
                vb=float(feasible['vb']), load_limit=float(feasible['load_limit']),
                confirm_deg=float(feasible['confirm_deg']),
                timeout_ms=float(feasible['timeout_ms']), deployed=True,
                score=feasible['score'], engage=feasible['engage'],
                exit_rate=feasible['exit_rate'], peak_load=feasible['peak_load'],
                reason='best R1/R2 candidate', evidence=feasible['label'])


def _report(output: Path, final: dict, results: list, status: dict) -> str:
    lines = ['# Ramp takeup autotune', '',
             f'- output: `{output}`',
             f'- port: `{status.get("port", "")}`',
             f'- calibration: `{status.get("calibration", "")}`',
             f'- status: **{"OK" if status.get("ok") else "INCOMPLETE"}**'
             f'{(" 鈥?" + str(status.get("error"))) if status.get("error") else ""}',
             f'- elapsed: {status.get("elapsed_s", 0.0):.0f} s', '']
    for name, entry in final.items():
        lines.append(f'## {name.upper()}')
        lines.append('')
        lines.append('| label | Bmax | B0 | vB | engage | exit | peak load | valid | score | reason |')
        lines.append('|---|---:|---:|---:|---:|---:|---:|---|---:|---|')
        rows = [row for row in results if row.get('type') == name]
        for row in rows:
            lines.append('| {label} | {bmax} | {b0} | {vb} | {engage} | {exit_rate} | '
                         '{peak_load} | {valid} | {score} | {reason} |'.format(
                             label=row.get('label', ''),
                             bmax=row.get('bmax', ''), b0=row.get('b0', ''),
                             vb=row.get('vb', ''),
                             engage=f"{float(row.get('engage') or 0.0):.3f}",
                             exit_rate=f"{float(row.get('exit_rate') or 0.0):.3f}",
                             peak_load=row.get('peak_load', ''),
                             valid=row.get('valid', ''),
                             score=f"{float(row['score']):.3f}" if row.get('score') is not None else '',
                             reason=str(row.get('reason', '')).replace('|', '/')))
        lines.append('')
        lines.append(f'Final: `{json.dumps(entry, ensure_ascii=False)}`')
        lines.append('')
    return '\n'.join(lines) + '\n'


def _copy_context(output: Path, calibration: Path) -> None:
    shutil.copy2(calibration, output / 'calibration.mmcal.json')
    for name in ('ESP32-P4.ino.elf.sha256.txt', 'build.options.json'):
        source = ROOT / '.debug' / 'p4' / name
        if source.is_file():
            shutil.copy2(source, output / name)


def _connect(hand, calibration: Path, args) -> dict:
    if not hand.connect(args.port):
        raise RuntimeError(f'failed to connect {args.port}')
    _wait(lambda: bool(hand.get_state_snapshot().product_fingerprint), 5,
          'product fingerprint')
    inspected = hand._calibration_workflow.inspect_snapshot(calibration)
    ok, message = hand._controller.load_calibration_snapshot(inspected)
    if not ok:
        raise RuntimeError(f'calibration load failed: {message}')
    _wait(lambda: hand._control_parameter_sync.startswith('APPLIED'), 15,
          'calibration sync')
    return dict(hand.get_state_snapshot().product_fingerprint or {})


def run(args) -> int:
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    (output / 'raw').mkdir()
    application_actions.RUN_DATA_DIR = output / 'recordings'
    from desktop.core.hand_backend import HandBackend

    calibration = Path(args.calibration)
    if not calibration.is_file():
        raise RuntimeError(f'calibration not found: {calibration}')
    _copy_context(output, calibration)
    hand = HandBackend()
    hand._save_data = False
    hand.open_application()
    started = time.monotonic()
    deadline = started + args.max_minutes * 60.0
    results: list = []
    final: dict = {}
    status = dict(ok=False, stage='connect', error='', port=args.port,
                  calibration=str(calibration), started=_now(),
                  max_minutes=args.max_minutes, types=list(args.types),
                  pair_gain=float(args.pair_gain), ladder=list(args.ladder))
    safe = None
    safe_applied = False

    def save_status(**changes):
        status.update(changes)
        status['updated'] = _now()
        _json(output / 'status.json', status)

    _json(output / 'status.json', status)
    try:
        fingerprint = _connect(hand, calibration, args)
        _json(output / 'fingerprint.json', fingerprint)
        before = _safe_snapshot(hand)
        _json(output / 'parameters.before.json', before)
        if args.preflight_only:
            state = hand.get_state_snapshot()
            summary = _state_summary(state, args.assume_index_aa_clear)
            _json(output / 'preflight_state.json', summary)
            _release(hand)
            ready_keys = ('ready_index_aa', 'ready_mcp', 'ready_faults', 'ready_control_off')
            status['ok'] = all(bool(summary.get(key)) for key in ready_keys)
            status['stage'] = 'preflight_done'
            status['preflight'] = summary
            print('PREFLIGHT', json.dumps(summary, ensure_ascii=False), flush=True)
            return 0 if status['ok'] else 1
        safe = _make_safe(before, args.confirm_deg, args.types, args.pair_gain)
        _json(output / 'parameters.safe.json', safe)
        applied = _apply_verified(hand, safe)
        print('safe', applied, flush=True)
        state = hand.get_state_snapshot()
        _json(output / 'preflight_state.json',
              _state_summary(state, args.assume_index_aa_clear))
        save_status(stage='preflight', safe_applied=applied)
        save_status(stage='tuning')
        for joint_type in args.types:
            if joint_type not in core.TYPES:
                continue
            if time.monotonic() > deadline - 60:
                print('[ramp] deadline reached before next type', flush=True)
                break
            save_status(stage='tuning', current_type=joint_type)
            try:
                final[joint_type] = _tune_type(hand, output, safe, joint_type, args,
                                               results, deadline)
            except (StopRequested, PreflightBlocked, SafetyAbort) as exc:
                status['error'] = f'{type(exc).__name__}: {exc}'
                status['failed_type'] = joint_type
                raise
            print('FINAL', joint_type, json.dumps(final[joint_type], ensure_ascii=False),
                  flush=True)
            save_status(stage='type_done', current_type=joint_type,
                        final=final[joint_type])
        _apply_verified(hand, safe)
        safe_applied = True
        _release(hand)
        requested = [name for name in args.types if name in core.TYPES]
        status['ok'] = all(name in final for name in requested)
        status['stage'] = 'done' if status['ok'] else 'incomplete'
        return 0 if status['ok'] else 1
    except (StopRequested, PreflightBlocked, SafetyAbort) as exc:
        status['error'] = f'{type(exc).__name__}: {exc}'
        print('ABORT', status['error'], flush=True)
        return 1
    except Exception as exc:
        status['error'] = f'{type(exc).__name__}: {exc}'
        print('FAILED', status['error'], flush=True)
        return 1
    finally:
        try:
            _release(hand)
        except Exception:
            pass
        if safe is not None and not safe_applied and hand.is_comm_connected():
            try:
                _apply_verified(hand, safe)
                safe_applied = True
            except Exception as exc:
                status['final_apply_error'] = f'{type(exc).__name__}: {exc}'
                print('[ramp] final safe apply failed:', exc, flush=True)
        try:
            hand.disconnect(stop_lower=False)
        except Exception:
            pass
        try:
            hand.close_application()
        except Exception:
            pass
        status['elapsed_s'] = time.monotonic() - started
        status['finished'] = _now()
        status['trials'] = len(results)
        status['deployed'] = {name: bool(entry.get('deployed'))
                              for name, entry in final.items()}
        _json(output / 'results.json',
              dict(final=final, trials=results, status=status,
                   elapsed_s=status['elapsed_s']))
        (output / 'report.md').write_text(_report(output, final, results, status),
                                          encoding='utf-8')
        if safe is not None:
            _json(output / 'parameters.final.json', safe)
        save_status(stage='finished')


def _run_verify(args) -> int:
    """Re-measure the currently applied ramp parameters without writing any."""
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    (output / 'raw').mkdir()
    application_actions.RUN_DATA_DIR = output / 'recordings'
    from desktop.core.hand_backend import HandBackend

    calibration = Path(args.calibration)
    if not calibration.is_file():
        raise RuntimeError(f'calibration not found: {calibration}')
    _copy_context(output, calibration)
    hand = HandBackend()
    hand._save_data = False
    hand.open_application()
    started = time.monotonic()
    status = dict(ok=False, mode='verify', port=args.port, calibration=str(calibration),
                  started=_now(), types=list(args.types))
    rows: dict = {}
    try:
        _json(output / 'fingerprint.json', _connect(hand, calibration, args))
        snapshot = _safe_snapshot(hand)
        _json(output / 'parameters.applied.json', snapshot)
        gains_ok = all(abs(float(snapshot['joint_kp'][name]) - EXPECTED_KP[name]) <= 1e-6
                       and abs(float(snapshot['joint_kd'][name]) - EXPECTED_KD[name]) <= 1e-6
                       for name in ALL_TYPES)
        status['gains_ok'] = gains_ok
        if not gains_ok:
            raise RuntimeError('applied Kp/Kd do not match the frozen validated values')
        for joint_type in args.types:
            if joint_type not in core.TYPES:
                continue
            takeup = float(snapshot['reversal'][f'takeup_max_{joint_type}'])
            if takeup <= 0:
                rows[joint_type] = dict(deployed=False, reason='takeup_max=0')
                continue
            joint = core.REPRESENTATIVE_JOINTS[joint_type]
            motors = joint_motors(joint)
            low, high = RANGES[joint_type]
            plan, truth = core.build_reversal_plan(
                low, high, delta_deg=min(4.0, 0.07 * (high - low)),
                speed_deg_s=args.speed, speeds=(('low', args.speed),),
                high_cap=core.TEST_HIGH_CAP_DEG.get(joint_type))
            label = f'verify_{joint_type}'
            samples, duration = _run_plan(
                hand, output, joint, motors, _lead_in_plan(hand, joint, plan, args.speed),
                label, low, high, int(snapshot['revision']), configuration_crc(snapshot),
                hold_joints=_hold_for(joint),
                assume_index_aa_clear=args.assume_index_aa_clear)
            params = _analysis_params(snapshot, joint_type, 0.0)
            metrics = _metrics(samples, truth, motors, params)
            trial = metrics['trial']
            row = dict(type=joint_type, joint=int(joint), deployed=True,
                       takeup_max=takeup, b0=float(snapshot['reversal_ramp'][f'b0_{joint_type}']),
                       vb=float(snapshot['reversal_ramp'][f'vb_{joint_type}']),
                       load_limit=float(snapshot['reversal_ramp'][f'load_limit_{joint_type}']),
                       confirm_deg=float(params['confirm_deg']),
                       valid=bool(trial.valid), score=trial.score,
                       engage=float(metrics['engage']), exit_rate=float(metrics['exit_rate']),
                       peak_load=int(metrics['peak_load']),
                       applied_max=int(metrics['applied_max']), reason=str(trial.reason),
                       duration=float(duration))
            row['verified'] = _feasible(row)
            rows[joint_type] = row
            print('VERIFY', json.dumps(row, ensure_ascii=False), flush=True)
            _json(output / 'verify.json', rows)
        status['ok'] = gains_ok and all(
            (not row.get('deployed')) or bool(row.get('verified')) for row in rows.values())
        status['stage'] = 'done'
        return 0 if status['ok'] else 1
    except (StopRequested, PreflightBlocked, SafetyAbort) as exc:
        status['error'] = f'{type(exc).__name__}: {exc}'
        print('VERIFY ABORT', status['error'], flush=True)
        return 1
    except Exception as exc:
        status['error'] = f'{type(exc).__name__}: {exc}'
        print('VERIFY FAILED', status['error'], flush=True)
        return 1
    finally:
        try:
            _release(hand)
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
        status['elapsed_s'] = time.monotonic() - started
        status['finished'] = _now()
        _json(output / 'verify.json', dict(types=rows, status=status))
        (output / 'report.md').write_text(
            '# Ramp parameter verification\n\n' +
            '\n'.join(f'- {name}: `{json.dumps(row, ensure_ascii=False)}`'
                      for name, row in rows.items()) + '\n', encoding='utf-8')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--hardware', action='store_true', help='tune on the connected hand')
    mode.add_argument('--preflight-only', action='store_true',
                      help='apply the tuning base and dump the preflight state, no motion')
    mode.add_argument('--verify', action='store_true',
                      help='re-measure the currently applied parameters, no parameter writes')
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--port', default='COM12')
    parser.add_argument('--types', default='aa,mcp,dip,pip')
    parser.add_argument('--ladder', default=','.join(str(value) for value in LADDER))
    parser.add_argument('--b0', type=float, default=5.0)
    parser.add_argument('--vb', type=float, default=50.0)
    parser.add_argument('--timeout-ms', type=float, default=TIMEOUT_MS)
    parser.add_argument('--speed', type=float, default=5.0)
    parser.add_argument('--confirm-deg', type=float, default=None)
    parser.add_argument('--pair-gain', type=float, default=0.0,
                        help='pair recovery gain kept during the take-up sweep')
    parser.add_argument('--no-refine', action='store_true')
    parser.add_argument('--assume-index-aa-clear', action='store_true',
                        help='operator confirms index AA J04 is out of the way; needed because '
                             'J04 is outside the J08-J11 feedback scope and often unreadable')
    parser.add_argument('--max-minutes', type=float, default=30.0)
    args = parser.parse_args(argv)
    args.types = [name.strip() for name in args.types.split(',') if name.strip()]
    if not args.types:
        parser.error('--types must not be empty')
    try:
        args.ladder = sorted({float(value.strip()) for value in args.ladder.split(',')
                              if value.strip()})
    except ValueError:
        parser.error('--ladder must be a comma-separated list of numbers')
    if not args.ladder or args.ladder[0] <= 0:
        parser.error('--ladder values must be positive')
    if args.speed <= 0:
        parser.error('--speed must be positive')
    if args.b0 < 0 or args.vb < 0 or args.timeout_ms < 0:
        parser.error('--b0/--vb/--timeout-ms must not be negative')
    if args.b0 > args.ladder[0]:
        parser.error('--b0 must not exceed the first --ladder value')
    if args.confirm_deg is not None and not 0.1 <= args.confirm_deg <= 10.0:
        parser.error('--confirm-deg must be within 0.1-10.0')
    if not 0.0 <= args.pair_gain <= 0.5:
        parser.error('--pair-gain must be within 0-0.5')
    if args.verify or args.preflight_only:
        if args.verify:
            return _run_verify(args)
        return run(args)
    return run(args)


if __name__ == '__main__':
    raise SystemExit(main())
