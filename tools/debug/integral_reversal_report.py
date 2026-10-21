"""Rebuild the integral/fast-reversal autotune report from a run directory.

The hardware runner writes selection JSON files incrementally; this offline
helper merges them into one auditable report without touching the device.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path


def _load(path: Path, default):
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return default


def _fmt(value):
    if value is None:
        return ''
    if isinstance(value, float):
        return f'{value:.4f}'
    return str(value)


def build(run: Path) -> str:
    status = _load(run / 'status.json', {})
    deployment = _load(run / 'deployment.json', {})
    integral = _load(run / 'integral_selection.json', {})
    takeup = _load(run / 'takeup_selection.json', {})
    detector = _load(run / 'detector/scores.json', [])
    final = _load(run / 'final_parameters.json', {})
    candidate = takeup.get('selected') or status.get('selected') or {}
    reasons = deployment.get('reasons') or status.get('reasons') or []
    lines = ['# Integral and fast-reversal autotune report', '',
             f"- elapsed_s: {_fmt(status.get('elapsed_s'))}",
             f"- combined_ok: {deployment.get('combined_ok', status.get('combined_ok'))}",
             f"- candidate_selection: `{json.dumps(candidate, ensure_ascii=False)}`",
             f"- final_deployed_ki: `{json.dumps(final.get('joint_ki'), ensure_ascii=False)}`",
             f"- final_deployed_reversal: `{json.dumps(final.get('reversal'), ensure_ascii=False)}`",
             f"- reasons: `{json.dumps(reasons, ensure_ascii=False)}`", '',
             '## Integral trials', '',
             '| type | joint | ki | score | valid | gate | clamp | I change | reason |',
             '|---|---:|---:|---:|---|---:|---:|---:|---|']
    for trial in integral.get('trials', []):
        lines.append(f"| {trial.get('joint_type','')} | {trial.get('joint','')} | "
                     f"{_fmt(trial.get('ki'))} | {_fmt(trial.get('score'))} | "
                     f"{trial.get('valid')} | {_fmt(trial.get('gate_fraction'))} | "
                     f"{_fmt(trial.get('clamp_fraction'))} | "
                     f"{_fmt(trial.get('i_change_counts'))} | {trial.get('reason','')} |")
    lines += ['', '## Detector (offline)', '',
              '| confirm_deg | confirm_ms | TP | FP | FN | FN(needs takeup) | p90 latency s |',
              '|---:|---:|---:|---:|---:|---:|---:|']
    for score in detector:
        lines.append(f"| {score.get('confirm_deg')} | {score.get('confirm_ms')} | "
                     f"{score.get('tp')} | {score.get('fp')} | {score.get('fn')} | "
                     f"{score.get('fn_needs_takeup')} | {_fmt(score.get('p90_latency_s'))} |")
    lines += ['', '## Takeup/exit trials', '',
              '| type | joint | takeup | exit | score | valid | extra counts | worst event s | reason |',
              '|---|---:|---:|---:|---:|---|---:|---:|---|']
    for trial in takeup.get('trials', []):
        lines.append(f"| {trial.get('joint_type','')} | {trial.get('joint','')} | "
                     f"{_fmt(trial.get('takeup_counts'))} | {_fmt(trial.get('exit_travel_deg'))} | "
                     f"{_fmt(trial.get('score'))} | {trial.get('valid')} | "
                     f"{_fmt(trial.get('extra_travel_counts'))} | "
                     f"{_fmt(trial.get('worst_event_s'))} | {trial.get('reason','')} |")
    summary = _load(run / 'validation/summary.json', {})
    lines += ['', '## Combined validation', '',
              '| type | variant | J_I | J_R | dynamic RMSE | phase delay | hard failure |',
              '|---|---|---:|---:|---:|---:|---|']
    for variant in ('baseline', 'candidate'):
        for joint_type, metrics in (summary.get(variant) or {}).items():
            lines.append(f"| {joint_type} | {variant} | {_fmt(metrics.get('integral_score'))} | "
                         f"{_fmt(metrics.get('reversal_score'))} | "
                         f"{_fmt(metrics.get('dynamic_rmse_deg'))} | "
                         f"{_fmt(metrics.get('dynamic_phase_delay_s'))} | "
                         f"{metrics.get('hard_failure','')} |")
    lines += ['', '## Safety and deployment', '',
              f"- control disabled: {deployment.get('control_disabled')}",
              f"- faults: {deployment.get('faults')}",
              f"- applied: {deployment.get('applied')}",
              f"- final JSON: `{run / 'final_parameters.json'}`",
              f"- fallback: `{run / 'parameters.safe.json'}`",
              '', '## Unverified / notes', '',
              '- PIP/DIP load-near-overload aborts are retained as infeasible; no Ki was deployed.',
              '- AA Ki baseline trial aborted with control disabled; AA Ki remains 0.',
              '- Nonzero takeup was not deployed because the DIP combined validation was incomplete.',
              '- Host trace sampling is ~20-25 Hz because the state snapshot publishes one frame per USB batch.',
              '']
    return '\n'.join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args(argv)
    if not (args.run / 'integral_selection.json').is_file() and \
            not (args.run / 'takeup_selection.json').is_file():
        raise SystemExit('not a hardware autotune run directory (selection JSON missing)')
    text = build(args.run)
    (args.run / 'report.md').write_text(text, encoding='utf-8')
    status = _load(args.run / 'status.json', {})
    deployment = _load(args.run / 'deployment.json', {})
    final = _load(args.run / 'final_parameters.json', {})
    takeup = _load(args.run / 'takeup_selection.json', {})
    summary = ('# Integral / fast-reversal autotune\n\n'
               f"- elapsed_s: {_fmt(status.get('elapsed_s'))}\n"
               f"- combined_ok: {deployment.get('combined_ok', status.get('combined_ok'))}\n"
               f"- candidate_selection: `{json.dumps(takeup.get('selected') or {}, ensure_ascii=False)}`\n"
               f"- final_deployed_ki: `{json.dumps(final.get('joint_ki'), ensure_ascii=False)}`\n"
               f"- final_deployed_reversal: `{json.dumps(final.get('reversal'), ensure_ascii=False)}`\n"
               f"- reasons: `{json.dumps(deployment.get('reasons') or status.get('reasons') or [], ensure_ascii=False)}`\n")
    (args.run / 'summary.md').write_text(summary, encoding='utf-8')
    print(text)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
