#!/usr/bin/env python3
"""Bench runner for adaptive Kp/vB firmware variants.

职责：顺序构建/烧录 P4 variant，headless 跑同一条 J00 正弦，录制并计算时延。
线程：单进程；所有硬件操作串行。
输入：variant JSON、replay CSV、串口和运行次数。
输出：每个 variant 的录制目录、指标 JSON 和 Markdown 汇总。
状态所有权：只拥有本次 bench 的进程状态；设备真值归 HandBackend/P4。
禁止依赖：禁止 UI、直接协议或跳过 HandBackend 安全门禁。
"""
from __future__ import annotations

import argparse
import csv
import hashlib
from datetime import datetime
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
BUILD_DIR = ROOT / '.debug' / 'p4'
SKETCH = ROOT / 'ESP32-P4'
ELF = BUILD_DIR / 'ESP32-P4.ino.elf'
CSV_J = ('02_joint_feedback.csv', '02_关节总表.csv')
CSV_E = ('06_operation_events.csv', '06_操作事件总表.csv')
CSV_R = ('08_raw_frames.csv', '08_原始接收帧.csv')
CSV_S = ('03_motor_feedback.csv', '03_电机反馈总表.csv')
P4_FQBN = (
    'esp32:esp32:esp32p4:UploadSpeed=921600,USBMode=hwcdc,CDCOnBoot=cdc,'
    'MSCOnBoot=default,DFUOnBoot=default,UploadMode=default,FlashFreq=80,'
    'FlashMode=qio,FlashSize=4M,PartitionScheme=default,DebugLevel=none,'
    'PSRAM=disabled,EraseFlash=none,JTAGAdapter=builtin,ChipVariant=prev3'
)
FLAG_NAMES = (
    ('kp_max', 'MMHAND_ADAPTIVE_KP_MAX'),
    ('kp_step', 'MMHAND_ADAPTIVE_KP_STEP'),
    ('kp_up_ms', 'MMHAND_ADAPTIVE_KP_UP_MS'),
    ('lag_ms', 'MMHAND_ADAPTIVE_LAG_MS'),
    ('vb_slow_ms', 'MMHAND_VB_SLOW_MS'),
    ('vb_step', 'MMHAND_VB_STEP'),
    ('vb_max', 'MMHAND_VB_MAX'),
    ('vb_min', 'MMHAND_VB_MIN'),
    ('vb_overshoot_step', 'MMHAND_VB_OVERSHOOT_STEP'),
)


def load_variants(path: Path) -> list[dict]:
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    if not isinstance(data, list) or not data:
        raise ValueError('variant file must be a non-empty JSON list')
    for variant in data:
        if not isinstance(variant, dict) or not variant.get('name'):
            raise ValueError('each variant needs a name')
        for key, _ in FLAG_NAMES:
            if key not in variant:
                raise ValueError(f'{variant["name"]}: missing {key}')
    return data


def variant_flags(variant: dict) -> list[str]:
    return [f'-D{macro}={variant[key]}' for key, macro in FLAG_NAMES]


def build_command(variant: dict) -> list[str]:
    flags = ' '.join(variant_flags(variant))
    return [shutil.which('arduino-cli') or 'arduino-cli', 'compile',
            '--fqbn', P4_FQBN, '--build-path', str(BUILD_DIR),
            '--optimize-for-debug', '--warnings', 'default', '--jobs', '4',
            '--clean', '--build-property', f'compiler.cpp.extra_flags={flags}',
            str(SKETCH)]


def upload_command(port: str) -> list[str]:
    return [shutil.which('arduino-cli') or 'arduino-cli', 'upload',
            '--port', port, '--fqbn', P4_FQBN,
            '--upload-property', 'upload.speed=921600', '--verify',
            '--input-dir', str(BUILD_DIR), str(SKETCH)]


def _run(command: list[str]) -> None:
    print('[bench]', ' '.join(command), flush=True)
    subprocess.run(command, cwd=ROOT, check=True)


def write_flash_markers(port: str) -> None:
    if not ELF.is_file():
        raise FileNotFoundError(ELF)
    digest = hashlib.sha256(ELF.read_bytes()).hexdigest().upper()
    (ELF.parent / (ELF.name + '.flashed.sha256')).write_text(
        f'{digest}  {ELF.name}\n', encoding='ascii')
    try:
        from serial.tools import list_ports
        serial = next((p.serial_number for p in list_ports.comports()
                       if p.device == port and p.vid == 0x303A and p.pid == 0x1001), '')
    except Exception:
        serial = ''
    if serial:
        (ELF.parent / (ELF.name + '.jtag_serial')).write_text(serial, encoding='ascii')


def _wait_result(hand, result, label: str, timeout: float = 20.0) -> dict:
    if not result.accepted:
        raise RuntimeError(f'{label}: {result.message}')
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        item = next((row for row in hand._application_operations
                     if row['operation_id'] == result.operation_id), None)
        if item is not None and item['status'] in ('SUCCESS', 'FAILED'):
            if item['status'] != 'SUCCESS':
                raise RuntimeError(f'{label}: {item.get("message") or item["status"]}')
            return item
        time.sleep(0.02)
    raise TimeoutError(f'{label} did not finish in {timeout:.0f}s')


def _wait_until(predicate, label: str, timeout: float, interval: float = 0.1) -> Any:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(interval)
    raise TimeoutError(f'{label} did not become ready in {timeout:.0f}s')


def run_replay(port: str, csv_path: Path, timeout: float = 90.0) -> tuple[Path, list[str]]:
    from desktop.core.hand_backend import HandBackend
    from desktop.core.protocol import SYSTEM_STATE_READY
    from desktop.replay.replay_service import ReplayService

    hand = HandBackend()
    hand.open_application(replay_service=ReplayService(hand))
    try:
        hand.request_recording(False)
        _wait_result(hand, hand.request_connect(port), 'Connect P4')
        _wait_until(lambda: hand.get_state_snapshot().system_state == SYSTEM_STATE_READY,
                    'P4 READY', 10.0)
        hand.select_joints([0])
        _wait_result(hand, hand.request_replay_upload(csv_path.name, csv_path.read_bytes()),
                     'Import CSV')
        _wait_result(hand, hand.request_recording(
            True, ['packet_frames', 'joint_wide', 'servo_wide', 'event_log']), 'Start recording')
        _wait_until(lambda: hand.application_snapshot(include_curves=False)['recording']['running'],
                    'Recording running', 10.0)
        _wait_result(hand, hand.request_replay_start(), 'Replay start', timeout=15.0)
        _wait_until(
            lambda: (hand.application_snapshot(include_curves=False)['replay'].get('state')
                     in ('completed', 'aborted', 'failed')),
            'Replay completion', timeout)
        snapshot = hand.application_snapshot(include_curves=False)
        if snapshot['replay'].get('state') != 'completed':
            raise RuntimeError(f"Replay ended: {snapshot['replay'].get('state')}")
        _wait_result(hand, hand.request_recording(False), 'Stop recording')
        _wait_until(lambda: not hand.application_snapshot(include_curves=False)['recording']['running'],
                    'Recording finalize', 10.0)
        _wait_until(lambda: not (hand._recording_thread and hand._recording_thread.is_alive()),
                    'Recording writer', 10.0)
        snapshot = hand.application_snapshot(include_curves=False)
        path = snapshot['recording'].get('path') or ''
        if not path:
            raise RuntimeError('recording did not produce a directory')
        return Path(path), list(snapshot.get('problems') or [])
    finally:
        try:
            hand.stop_motion(emergency=False, source='adaptive bench cleanup')
        except Exception:
            pass
        hand.close_application()


def _find(directory: Path, names: tuple[str, ...]) -> Path | None:
    for name in names:
        path = directory / name
        if path.is_file():
            return path
    return None


def _event_start(directory: Path) -> float | None:
    path = _find(directory, CSV_E)
    if path is None:
        return None
    with path.open(encoding='utf-8-sig', newline='') as handle:
        for row in csv.DictReader(handle):
            if 'Replay started' in (row.get('detail') or ''):
                return datetime.fromisoformat(row['host_time']).timestamp()
    return None


def _adaptive_state(directory: Path) -> dict:
    from desktop.core.protocol import parse_adaptive_kp_packet
    path = _find(directory, CSV_R)
    if path is None:
        return {}
    latest = None
    with path.open(encoding='utf-8-sig', newline='') as handle:
        for row in csv.DictReader(handle):
            if row.get('packet_type') == '31':
                parsed = parse_adaptive_kp_packet(bytes.fromhex(row['payload_hex']))
                if parsed:
                    latest = parsed
    if not latest:
        return {}
    return {
        'kp_scale': latest['scale'][0],
        'kp_effective': latest['effective'][0],
        'vb_scale': (latest.get('vb_scale') or [None])[0],
        'vb_effective': (latest.get('vb_effective') or [None])[0],
    }


def analyze_recording(directory: Path, frequency: float = 0.2) -> dict:
    from desktop.core.control_tuning import reversal_metrics, sine_metrics
    joint = _find(directory, CSV_J)
    if joint is None:
        raise FileNotFoundError('joint feedback CSV not found: ' + str(directory))
    start = _event_start(directory)
    samples = []
    with joint.open(encoding='utf-8-sig', newline='') as handle:
        for row in csv.DictReader(handle):
            if row.get('J00_active') != '1' or not row.get('J00_target_deg'):
                continue
            if row.get('J00_current_valid') != '1' or not row.get('J00_current_deg'):
                continue
            host = datetime.fromisoformat(row['host_time']).timestamp()
            if start is not None and host < start:
                continue
            samples.append(dict(
                t=(host - start) if start is not None else host,
                q=float(row['J00_current_deg']),
                target=float(row['J00_target_deg'])))
    if not samples:
        raise RuntimeError('no valid J00 samples in recording')
    samples.sort(key=lambda sample: sample['t'])
    base = samples[0]['t']
    for sample in samples:
        sample['t'] -= base
    sine = sine_metrics(samples, frequency, discard_s=1.5)
    reversal = reversal_metrics(samples, frequency)
    adaptive = _adaptive_state(directory)
    max_load = None
    servo = _find(directory, CSV_S)
    if servo is not None:
        with servo.open(encoding='utf-8-sig', newline='') as handle:
            for row in csv.DictReader(handle):
                for motor in ('M00', 'M01'):
                    value = row.get(f'{motor}_load_mag')
                    if value:
                        max_load = max(max_load or 0, abs(int(float(value))))
    return {
        'recording': str(directory),
        'samples': len(samples),
        'max_load': max_load,
        'sine': sine,
        'reversal': reversal,
        'adaptive': adaptive,
    }


def safety_ok(report: dict) -> tuple[bool, list[str]]:
    """Hard safety only; phase/reversal targets are acceptance, not safety."""
    reasons = []
    sine = report.get('sine') or {}
    if not sine.get('valid'):
        reasons.append('invalid sine metrics')
    ratio = sine.get('amplitude_ratio')
    if ratio is None or not 0.95 <= ratio <= 1.05:
        reasons.append('amplitude ratio outside 0.95-1.05')
    max_load = report.get('max_load')
    if max_load is not None and max_load >= 900:
        reasons.append('motor load >=900')
    return not reasons, reasons


def acceptance_ok(report: dict) -> tuple[bool, list[str]]:
    ok, reasons = safety_ok(report)
    sine = report.get('sine') or {}
    if sine.get('phase_delay_s') is None or sine['phase_delay_s'] > 0.12:
        reasons.append('phase delay >120ms')
    reversal = report.get('reversal') or {}
    median = reversal.get('median_s')
    if median is None or median > 0.55:
        reasons.append('reversal median >550ms')
    return ok and not reasons, reasons


def _gdb_check(variant: dict) -> None:
    """Manual code-path spot check; no motion is active yet, so a miss is not fatal."""
    command = [sys.executable, str(ROOT / 'tools/debug/gdb_batch.py'),
               '--target', 'p4', '--timeout', '30', '--break',
               'mmhand::control::JointFeedbackController::updateAdaptiveVb',
               '--print', 'this->vbScale[0]', '--print', 'this->states[0].phase']
    print('[bench] gdb check:', ' '.join(command), flush=True)
    try:
        subprocess.run(command, cwd=ROOT, check=True)
    except subprocess.CalledProcessError:
        print('[bench] gdb check did not hit (run it during an active motion); continuing',
              file=sys.stderr, flush=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--variants', type=Path,
                        default=ROOT / 'tools/debug/adaptive_variants.json')
    parser.add_argument('--csv', type=Path, required=False,
                        default=ROOT / 'logs/imports/J00_sine_c-30_A30_0p2Hz_12cycles_62s.csv')
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--runs', type=int, default=3)
    parser.add_argument('--timeout', type=float, default=90.0)
    parser.add_argument('--skip-build', action='store_true')
    parser.add_argument('--gdb-check', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)
    variants = load_variants(args.variants)
    csv_path = Path(args.csv)
    if not csv_path.is_file():
        print(f'[bench] CSV missing: {csv_path}', file=sys.stderr)
        return 2
    if args.dry_run:
        for variant in variants:
            print(f'[bench] {variant["name"]}: {" ".join(variant_flags(variant))}')
            print('[bench] build:', ' '.join(build_command(variant)))
            print('[bench] upload:', ' '.join(upload_command(args.port)))
        return 0
    out_dir = ROOT / 'logs' / f'adaptive_bench_{datetime.now():%Y%m%d_%H%M%S}'
    out_dir.mkdir(parents=True, exist_ok=True)
    summary = []
    safe_variants = []
    for variant in variants:
        if not args.skip_build:
            _run(build_command(variant))
        _run(upload_command(args.port))
        write_flash_markers(args.port)
        time.sleep(2.0)
        if args.gdb_check:
            _gdb_check(variant)
        for run in range(1, args.runs + 1):
            print(f'[bench] {variant["name"]} run {run}/{args.runs}', flush=True)
            directory, problems = run_replay(args.port, csv_path, args.timeout)
            report = analyze_recording(directory)
            safe, safety_reasons = safety_ok(report)
            accepted, reasons = acceptance_ok(report)
            if problems:
                safe = accepted = False
                safety_reasons += [f'snapshot problem: {problem}' for problem in problems]
                reasons += [f'snapshot problem: {problem}' for problem in problems]
            row = {'variant': variant, 'run': run, 'directory': str(directory),
                   'ok': safe, 'accepted': accepted,
                   'reasons': reasons, 'safety_reasons': safety_reasons, **report}
            summary.append(row)
            (out_dir / f'{variant["name"]}_run{run}.json').write_text(
                json.dumps(row, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
            print('[bench]', json.dumps({k: row[k] for k in ('ok', 'accepted', 'reasons')}), flush=True)
            if not safe:
                print('[bench] safety gate failed; stop this bench', file=sys.stderr)
                if safe_variants and not args.skip_build:
                    rollback = safe_variants[-1]
                    print(f'[bench] rolling back to {rollback["name"]}', file=sys.stderr, flush=True)
                    _run(build_command(rollback))
                    _run(upload_command(args.port))
                    write_flash_markers(args.port)
                break
        else:
            safe_variants.append(variant)
            continue
        break
    (out_dir / 'summary.json').write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    lines = ['# Adaptive bench', '', '| variant | run | phase (ms) | reversal median (ms) | max load | vB scale | accepted |',
             '|---|---:|---:|---:|---:|---:|---|']
    for row in summary:
        sine = row.get('sine') or {}
        reversal = row.get('reversal') or {}
        adaptive = row.get('adaptive') or {}
        lines.append('| {} | {} | {} | {} | {} | {} | {} |'.format(
            row['variant']['name'], row['run'],
            round(sine['phase_delay_s'] * 1000, 1) if sine.get('phase_delay_s') is not None else '—',
            round(reversal['median_s'] * 1000, 1) if reversal.get('median_s') is not None else '—',
            row.get('max_load', '—'),
            adaptive.get('vb_scale', '—'), 'yes' if row.get('accepted') else 'no'))
    (out_dir / 'summary.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print(f'[bench] report: {out_dir}', flush=True)
    return 0 if summary and all(row['ok'] for row in summary) else 1


if __name__ == '__main__':
    raise SystemExit(main())
