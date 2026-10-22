"""Compose validated take-up ramp results and apply them through the normal host path.

The tuning tool leaves the hand with every take-up disabled on purpose.  This
entry point merges the deployable per-type results into the production default
configuration, applies the whole group with the normal parameter transaction and
reads the applied revision/CRC back.  ``--restore`` re-applies the defaults after
an aborted campaign; ``--exclude`` rolls one type back to zero after a failed
post-apply verification.  No firmware, protocol or schema change is involved.
"""
from __future__ import annotations

import argparse
import copy
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from desktop.core.control_parameters import (  # noqa: E402
    CURRENT_PATH, DEFAULT_PATH, configuration_crc, ramp_configuration_crc, validate,
)
from tools.debug.takeup_ramp_autotune import (  # noqa: E402
    ALL_TYPES, DEFAULT_LOAD_LIMIT,
)

def _json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, default=str) + '\n',
                    encoding='utf-8')


def _text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(value, encoding='utf-8')


def load_defaults() -> dict:
    return validate(json.loads(DEFAULT_PATH.read_text(encoding='utf-8')))


def load_results(paths) -> dict:
    merged: dict = {}
    for path in paths:
        path = Path(path)
        if not path.is_file():
            raise RuntimeError(f'results file not found: {path}')
        data = json.loads(path.read_text(encoding='utf-8-sig'))
        final = data.get('final') if isinstance(data, dict) else None
        if not isinstance(final, dict):
            raise RuntimeError(f'results file has no final map: {path}')
        for name, entry in final.items():
            if name in merged:
                raise RuntimeError(f'duplicate result for {name}: {path}')
            merged[name] = dict(entry, results_file=str(path))
    return merged


def _zero(final: dict, name: str) -> None:
    final['reversal'][f'takeup_max_{name}'] = 0.0
    final['reversal_ramp'][f'b0_{name}'] = 0.0
    final['reversal_ramp'][f'vb_{name}'] = 0.0


def compose_final(base: dict, results: dict, exclude=()) -> dict:
    """Keep defaults for untested types, zero tested-but-infeasible or excluded ones."""
    final = copy.deepcopy(base)
    excluded = {str(name) for name in exclude}
    for name in ALL_TYPES:
        if name in excluded:
            _zero(final, name)
            continue
        entry = results.get(name)
        if not entry:
            continue
        if not entry.get('deployed'):
            _zero(final, name)
            continue
        takeup = float(entry.get('takeup_max') or 0.0)
        b0 = float(entry.get('b0') or 0.0)
        vb = float(entry.get('vb') or 0.0)
        if takeup <= 0:
            _zero(final, name)
            continue
        if b0 > takeup:
            raise RuntimeError(f'{name}: b0 {b0} exceeds takeup_max {takeup}')
        if b0 <= 0 and vb <= 0:
            raise RuntimeError(f'{name}: takeup_max {takeup} needs b0>0 or vb>0')
        final['reversal'][f'takeup_max_{name}'] = takeup
        final['reversal_ramp'][f'b0_{name}'] = b0
        final['reversal_ramp'][f'vb_{name}'] = vb
        final['reversal_ramp'][f'load_limit_{name}'] = float(
            entry.get('load_limit') or DEFAULT_LOAD_LIMIT)
        confirm = entry.get('confirm_deg')
        if confirm is not None:
            final['reversal'][f'confirm_deg_{name}'] = float(confirm)
    final['reversal_ramp']['takeup_timeout_ms'] = float(
        base['reversal_ramp']['takeup_timeout_ms'])
    return validate(final)


def _diff_table(final: dict, before: dict) -> str:
    groups = ('joint_kp', 'joint_kd', 'reversal', 'reversal_ramp')
    lines = ['| group.field | before | after |', '|---|---:|---:|']
    for group in groups:
        for field, value in final[group].items():
            old = before[group].get(field)
            if abs(float(value) - float(old)) > 1e-9:
                lines.append(f'| {group}.{field} | {old} | {value} |')
    return '\n'.join(lines)


def _apply(final: dict, args) -> dict:
    from desktop.core.hand_backend import HandBackend
    from tools.debug.integral_reversal_autotune import (
        _apply_verified, _release, _safe_snapshot, _wait,
    )

    calibration = Path(args.calibration)
    if not calibration.is_file():
        raise RuntimeError(f'calibration not found: {calibration}')
    hand = HandBackend()
    hand._save_data = False
    hand.open_application()
    try:
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
        before = _safe_snapshot(hand)
        applied = _apply_verified(hand, final)
        _release(hand)
        after = hand.control_parameter_snapshot()
        state = hand.get_state_snapshot()
        return dict(before=before, applied=applied, after=after,
                    sync=str(hand._control_parameter_sync),
                    faults=int(state.system_fault_bitmap),
                    control_enabled=bool(state.control_enabled),
                    fingerprint=dict(state.product_fingerprint or {}),
                    main_revision=int(after['revision']),
                    main_crc=int(configuration_crc(after)),
                    ramp_revision=int(after['ramp_revision']),
                    ramp_crc=int(ramp_configuration_crc(after)))
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


def _write_report(output: Path, final: dict, results: dict, deployment: dict,
                  defaults_written: bool, diff: str) -> None:
    lines = ['# Ramp parameter deployment', '',
             f'- host config: `{CURRENT_PATH}`',
             f'- defaults: `{DEFAULT_PATH}`',
             f'- defaults updated: **{defaults_written}**', '']
    if deployment:
        try:
            source_crc = f"0x{int(deployment.get('fingerprint', {}).get('firmware_source_crc', 0)):08X}"
        except (TypeError, ValueError):
            source_crc = str(deployment.get('fingerprint', {}).get('firmware_source_crc', ''))
        lines += [
            f'- APPLIED: `{deployment.get("applied", "")}`',
            f'- readback OK: **{deployment.get("readback_ok")}**',
            f'- main revision/CRC: {deployment.get("main_revision")} / '
            f'`{deployment.get("main_crc", 0):08X}`',
            f'- ramp revision/CRC: {deployment.get("ramp_revision")} / '
            f'`{deployment.get("ramp_crc", 0):08X}`',
            f'- faults: {deployment.get("faults")}; '
            f'control enabled: {deployment.get("control_enabled")}',
            f'- firmware source CRC: `{source_crc}`',
            '',
        ]
    lines += ['## Deployed per-type values', '',
              '| type | takeup_max | b0 | vB | load_limit | confirm_deg | source |',
              '|---|---:|---:|---:|---:|---:|---|']
    for name in ALL_TYPES:
        entry = results.get(name) or {}
        lines.append('| {name} | {takeup} | {b0} | {vb} | {load} | {confirm} | {source} |'.format(
            name=name, takeup=final['reversal'][f'takeup_max_{name}'],
            b0=final['reversal_ramp'][f'b0_{name}'],
            vb=final['reversal_ramp'][f'vb_{name}'],
            load=final['reversal_ramp'][f'load_limit_{name}'],
            confirm=final['reversal'][f'confirm_deg_{name}'],
            source=entry.get('results_file', 'defaults')))
    lines += ['', '## Parameter changes versus the previous host state', '',
              diff or '(no value change)', '']
    _text(output / 'report.md', '\n'.join(lines) + '\n')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results', nargs='*', type=Path, default=[])
    parser.add_argument('--port', default='COM12')
    parser.add_argument('--calibration', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--restore', action='store_true',
                        help='apply the shipped defaults after an aborted campaign')
    parser.add_argument('--update-defaults', action='store_true')
    parser.add_argument('--exclude', default='',
                        help='comma-separated types to roll back to takeup_max=0')
    args = parser.parse_args(argv)
    exclude = [name.strip() for name in args.exclude.split(',') if name.strip()]
    for name in exclude:
        if name not in ALL_TYPES:
            parser.error(f'--exclude {name} is not a known joint type')

    output = Path(args.output)
    base = load_defaults()
    if args.restore:
        final = validate(copy.deepcopy(base))
        results: dict = {}
    else:
        results = load_results(args.results)
        final = compose_final(base, results, exclude)
    before = json.loads(CURRENT_PATH.read_text(encoding='utf-8')) if CURRENT_PATH.is_file() else base
    diff = _diff_table(final, before)
    output.mkdir(parents=True, exist_ok=False)
    _json(output / 'final_parameters.json', final)

    deployment: dict = {}
    defaults_written = False
    rc = 0
    if not args.dry_run:
        if args.calibration is None:
            parser.error('--calibration is required unless --dry-run is used')
        deployment = _apply(final, args)
        _json(output / 'deployment.json', deployment)
        # _apply_verified checks the readback and returns the main-frame
        # prefix; the sync string also carries the ramp revision/CRC.
        deployment['readback_ok'] = str(deployment['sync']).startswith(str(deployment['applied']))
        _json(output / 'deployment.json', deployment)
        if not deployment['readback_ok']:
            rc = 1
        if deployment['faults'] or deployment['control_enabled']:
            rc = 1
        on_disk = json.loads(CURRENT_PATH.read_text(encoding='utf-8'))
        if on_disk['reversal'] != final['reversal'] or on_disk['reversal_ramp'] != final['reversal_ramp']:
            rc = 1
            deployment['on_disk_mismatch'] = True
        if args.update_defaults and rc == 0:
            defaults = copy.deepcopy(final)
            defaults['revision'] = 1
            defaults['ramp_revision'] = 1
            validate(defaults)
            _json(DEFAULT_PATH, defaults)
            defaults_written = True
    _write_report(output, final, results, deployment, defaults_written, diff)
    print('DEPLOYED', json.dumps({
        'applied': deployment.get('applied', 'dry-run'),
        'main_revision': deployment.get('main_revision'),
        'main_crc': f"{deployment.get('main_crc', 0):08X}" if deployment else '',
        'ramp_revision': deployment.get('ramp_revision'),
        'ramp_crc': f"{deployment.get('ramp_crc', 0):08X}" if deployment else '',
        'final': {name: final['reversal'][f'takeup_max_{name}'] for name in ALL_TYPES},
        'excluded': exclude, 'rc': rc,
    }, ensure_ascii=False), flush=True)
    return rc


if __name__ == '__main__':
    raise SystemExit(main())
