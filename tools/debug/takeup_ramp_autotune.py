"""Find a takeup_max that actually engages, then refine B0/vB.

Hardware mode uses the normal desktop authorization, full main+ramp parameter
Apply, direct joint targets and the normal recorder. It never bypasses STOP,
collision, soft limits or feedback timeouts. The probe ladder increases
takeup_max until the additive PDF ramp produces joint exit (0.25 deg) under the
per-type load limit; otherwise the type is marked infeasible.
"""
from __future__ import annotations

import argparse
import copy
import json
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
    ALL_TYPES, _apply_verified, _lead_in_plan, _release, _run_plan,
    _safe_snapshot, _set_all_pair, _set_all_ramp, _set_all_reversal,
    _set_integral_defaults, _wait,
)

LADDER = (40, 80, 160, 320, 640, 1000, 1500, 2000)
VBS = (20, 50, 100)
DEFAULT_LOAD_LIMIT = 900
TIMEOUT_MS = 600
CONFIRM_DEG = 0.6


def _make_safe(hand):
    safe = _safe_snapshot(hand)
    safe['joint_ki'] = {name: 0.0 for name in ALL_TYPES}
    _set_integral_defaults(safe)
    safe['reversal'] = {'confirm_ms': 20}
    _set_all_reversal(safe, 'confirm_deg', CONFIRM_DEG)
    _set_all_reversal(safe, 'takeup_max', 0.0)
    _set_all_reversal(safe, 'exit_travel_deg', 0.25)
    _set_all_pair(safe, 0.0)
    _set_all_ramp(safe, 0.0, 0.0, DEFAULT_LOAD_LIMIT)
    validate(safe)
    return safe


def _metrics(samples, truth, motors, params):
    trial = core.reversal_metrics(samples, truth, motors, params)
    rev = [abs(int(row.get('reversal_counts', 0))) for row in samples]
    engage = sum(1 for value in rev if value > 0) / len(rev) if rev else 0.0
    peak_load = 0
    for row in samples:
        for motor, value in (row.get('load') or {}).items():
            peak_load = max(peak_load, abs(int(value)))
    valid_events = sum(1 for event in trial.events if event.get('valid'))
    exit_rate = valid_events / len(truth) if truth else 0.0
    return dict(trial=trial, engage=engage, exit_rate=exit_rate, peak_load=peak_load,
                applied_max=max(rev) if rev else 0, applied_mean=statistics.mean(rev) if rev else 0)


def _label(joint_type, bmax, vb, b0):
    return f'{joint_type}_max{bmax}_vb{vb}_b0{b0}'


def run(args):
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    (output / 'raw').mkdir()
    application_actions.RUN_DATA_DIR = output / 'recordings'
    from desktop.core.hand_backend import HandBackend

    calibration = Path(args.calibration)
    bundle = json.loads(calibration.read_text(encoding='utf-8'))
    hand = HandBackend()
    hand._save_data = False
    hand.open_application()
    started = time.monotonic()
    deadline = started + args.max_minutes * 60.0
    results = []
    final = {}
    try:
        if not hand.connect(args.port):
            raise RuntimeError(f'failed to connect {args.port}')
        inspected = hand._calibration_workflow.inspect_snapshot(calibration)
        ok, message = hand._controller.load_calibration_snapshot(inspected)
        if not ok:
            raise RuntimeError(f'calibration load failed: {message}')
        _wait(lambda: hand._control_parameter_sync.startswith('APPLIED'), 15, 'calibration sync')
        safe = _make_safe(hand)
        print('safe', _apply_verified(hand, safe), flush=True)
        ranges = {'aa': (-25.8, 23.8), 'mcp': (-3.6, 74.0),
                  'pip': (0.8, 79.2), 'dip': (-1.6, 89.6)}
        for joint_type in args.types:
            if joint_type not in core.TYPES:
                continue
            if time.monotonic() > deadline - 60:
                print('[ramp] deadline reached', flush=True)
                break
            joint = core.REPRESENTATIVE_JOINTS[joint_type]
            motors = joint_motors(joint)
            low, high = ranges[joint_type]
            plan, truth = core.build_reversal_plan(
                low, high, delta_deg=min(4.0, 0.07 * (high - low)),
                speeds=(('low', 5.0),), high_cap=core.TEST_HIGH_CAP_DEG.get(joint_type))
            base_points = plan
            feasible = None
            failures = 0
            for bmax in LADDER:
                if time.monotonic() > deadline - 45:
                    break
                proposal = copy.deepcopy(safe)
                _set_all_reversal(proposal, 'takeup_max', 0.0)
                proposal['reversal'][f'takeup_max_{joint_type}'] = bmax
                _set_all_ramp(proposal, 0.0, 0.0, DEFAULT_LOAD_LIMIT)
                proposal['reversal_ramp'][f'b0_{joint_type}'] = 5.0
                proposal['reversal_ramp'][f'vb_{joint_type}'] = 50.0
                proposal['reversal_ramp'][f'load_limit_{joint_type}'] = DEFAULT_LOAD_LIMIT
                proposal['reversal_ramp']['takeup_timeout_ms'] = TIMEOUT_MS
                validate(proposal)
                label = _label(joint_type, bmax, 50, 5)
                try:
                    applied = _apply_verified(hand, proposal)
                    points = _lead_in_plan(hand, joint, base_points)
                    revision = int(_safe_snapshot(hand)['revision'])
                    crc = configuration_crc(proposal)
                    samples, duration = _run_plan(hand, output, joint, motors, points, label,
                                                  low, high, revision, crc)
                    params = dict(proposal['joint_ki'])
                    params.update(proposal['reversal'])
                    params.update(proposal['feedback'])
                    params['ki'] = 0.0
                    metrics = _metrics(samples, truth, motors, params)
                    row = dict(type=joint_type, joint=joint, bmax=bmax, vb=50, b0=5,
                               applied=applied, valid=metrics['trial'].valid,
                               score=metrics['trial'].score,
                               engage=metrics['engage'], exit_rate=metrics['exit_rate'],
                               peak_load=metrics['peak_load'],
                               applied_max=metrics['applied_max'],
                               applied_mean=metrics['applied_mean'],
                               reason=metrics['trial'].reason, duration=duration)
                    results.append(row)
                    print('R1', json.dumps(row, ensure_ascii=False), flush=True)
                    if row['valid'] and row['exit_rate'] >= 0.8 and row['engage'] > 0.2 \
                            and row['peak_load'] < DEFAULT_LOAD_LIMIT:
                        feasible = row
                        break
                    failures = failures + 1 if not row['valid'] else 0
                    if failures >= 2:
                        break
                except Exception as exc:
                    results.append(dict(type=joint_type, joint=joint, bmax=bmax, vb=50, b0=5,
                                        valid=False, reason=str(exc)))
                    print('R1 FAIL', label, exc, flush=True)
                    try:
                        _apply_verified(hand, safe)
                    except Exception:
                        pass
                    failures += 1
                    if failures >= 2:
                        break
            if feasible is None:
                final[joint_type] = dict(takeup_max=0, b0=0, vb=0,
                                         load_limit=DEFAULT_LOAD_LIMIT,
                                         reason='no safe Bmax achieved exit under load limit')
                print('FINAL', joint_type, final[joint_type], flush=True)
                continue
            # R2: at the engaging Bmax, compare a few vB values with a small B0.
            best = feasible
            best_key = (feasible['score'] if feasible['score'] is not None else 1e9,
                        feasible['applied_max'])
            for vb in VBS:
                if vb == 50 or time.monotonic() > deadline - 30:
                    continue
                proposal = copy.deepcopy(safe)
                _set_all_reversal(proposal, 'takeup_max', 0.0)
                proposal['reversal'][f'takeup_max_{joint_type}'] = feasible['bmax']
                _set_all_ramp(proposal, 0.0, 0.0, DEFAULT_LOAD_LIMIT)
                proposal['reversal_ramp'][f'b0_{joint_type}'] = 2.0
                proposal['reversal_ramp'][f'vb_{joint_type}'] = float(vb)
                proposal['reversal_ramp'][f'load_limit_{joint_type}'] = DEFAULT_LOAD_LIMIT
                proposal['reversal_ramp']['takeup_timeout_ms'] = TIMEOUT_MS
                validate(proposal)
                label = _label(joint_type, feasible['bmax'], vb, 2)
                try:
                    applied = _apply_verified(hand, proposal)
                    points = _lead_in_plan(hand, joint, base_points)
                    revision = int(_safe_snapshot(hand)['revision'])
                    crc = configuration_crc(proposal)
                    samples, duration = _run_plan(hand, output, joint, motors, points, label,
                                                  low, high, revision, crc)
                    params = dict(proposal['joint_ki'])
                    params.update(proposal['reversal'])
                    params.update(proposal['feedback'])
                    params['ki'] = 0.0
                    metrics = _metrics(samples, truth, motors, params)
                    row = dict(type=joint_type, joint=joint, bmax=feasible['bmax'], vb=vb, b0=2,
                               applied=applied, valid=metrics['trial'].valid,
                               score=metrics['trial'].score,
                               engage=metrics['engage'], exit_rate=metrics['exit_rate'],
                               peak_load=metrics['peak_load'],
                               applied_max=metrics['applied_max'],
                               applied_mean=metrics['applied_mean'],
                               reason=metrics['trial'].reason, duration=duration)
                    results.append(row)
                    print('R2', json.dumps(row, ensure_ascii=False), flush=True)
                    if row['valid'] and row['exit_rate'] >= 0.8 and row['engage'] > 0.2:
                        key = (row['score'] if row['score'] is not None else 1e9,
                               row['applied_max'])
                        if key < best_key:
                            best, best_key = row, key
                except Exception as exc:
                    print('R2 FAIL', label, exc, flush=True)
                    results.append(dict(type=joint_type, joint=joint, bmax=feasible['bmax'],
                                        vb=vb, b0=2, valid=False, reason=str(exc)))
                    try:
                        _apply_verified(hand, safe)
                    except Exception:
                        pass
            final[joint_type] = dict(takeup_max=best['bmax'], b0=best['b0'], vb=best['vb'],
                                     load_limit=DEFAULT_LOAD_LIMIT,
                                     score=best['score'], engage=best['engage'],
                                     exit_rate=best['exit_rate'], peak_load=best['peak_load'],
                                     reason='best R1/R2 candidate')
            print('FINAL', joint_type, final[joint_type], flush=True)
        _apply_verified(hand, safe)
        _release(hand)
        (output / 'results.json').write_text(json.dumps(dict(final=final, trials=results,
            elapsed_s=time.monotonic() - started), indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
        (output / 'report.md').write_text(
            '# Ramp takeup autotune\n\n' +
            '\n'.join(f'- {name}: `{json.dumps(value, ensure_ascii=False)}`'
                      for name, value in final.items()) + '\n', encoding='utf-8')
        print('DONE', output, flush=True)
        return 0
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


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hardware', action='store_true')
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--port', default='COM12')
    parser.add_argument('--types', default='aa,mcp,dip,pip')
    parser.add_argument('--max-minutes', type=float, default=60.0)
    args = parser.parse_args(argv)
    if not args.hardware:
        parser.error('--hardware is required')
    args.types = [name.strip() for name in args.types.split(',') if name.strip()]
    return run(args)


if __name__ == '__main__':
    raise SystemExit(main())
