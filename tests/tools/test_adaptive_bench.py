"""Offline tests for the adaptive bench runner; no serial device or GUI."""
import csv
from datetime import datetime, timedelta
import json
import math
import struct
from pathlib import Path

from desktop.core.protocol import ENCODER_COUNT
from tools.debug import adaptive_bench


def test_dry_run_reports_variants_and_flags(tmp_path, capsys):
    variants = tmp_path / 'variants.json'
    variants.write_text(json.dumps([{
        'name': 'V-test', 'kp_max': 1.6, 'kp_step': 1.08, 'kp_up_ms': 1000,
        'lag_ms': 300, 'vb_slow_ms': 150.0, 'vb_step': 1.08, 'vb_max': 2.0,
        'vb_min': 0.5, 'vb_overshoot_step': 0.90,
    }]), encoding='utf-8')
    csv_path = tmp_path / 'motion.csv'
    csv_path.write_text('host_time\n', encoding='utf-8')
    assert adaptive_bench.main(['--dry-run', '--variants', str(variants),
                                '--csv', str(csv_path)]) == 0
    output = capsys.readouterr().out
    assert 'V-test' in output
    assert '-DMMHAND_ADAPTIVE_KP_MAX=1.6' in output
    assert '-DMMHAND_VB_MAX=2.0' in output
    assert 'arduino-cli compile' in output


def _write_payload(scale, effective, vb_scale, vb_effective):
    payload = bytes([1])
    payload += b''.join(struct.pack('>h', int(value * 1000)) for value in scale)
    payload += b''.join(struct.pack('>h', int(value * 1000)) for value in effective)
    payload += b''.join(struct.pack('>h', int(value * 1000)) for value in vb_scale)
    payload += b''.join(struct.pack('>h', int(value)) for value in vb_effective)
    assert len(payload) == 1 + 8 * ENCODER_COUNT
    return payload


def test_analysis_reads_delay_and_adaptive_vb(tmp_path):
    directory = tmp_path / 'recording'
    directory.mkdir()
    base = datetime(2026, 11, 19, 0, 0, 0)
    start = base + timedelta(seconds=5)
    with (directory / '06_operation_events.csv').open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=['host_time', 'detail'])
        writer.writeheader()
        writer.writerow({'host_time': start.isoformat(), 'detail': 'ok: Replay started'})
    rows = []
    for index in range(2000):
        t = index * 0.02
        target = -30 + 30 * math.sin(2 * math.pi * 0.2 * (t - 1.0))
        current = -30 + 30 * math.sin(2 * math.pi * 0.2 * (t - 1.2))
        rows.append({'host_time': (start + timedelta(seconds=t)).isoformat(),
                     'J00_active': '1', 'J00_current_valid': '1',
                     'J00_target_deg': f'{target:.6f}', 'J00_current_deg': f'{current:.6f}'})
    with (directory / '02_joint_feedback.csv').open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    payload = _write_payload([1.0] * ENCODER_COUNT, [0.5] * ENCODER_COUNT,
                             [1.25] * ENCODER_COUNT, [75] * ENCODER_COUNT)
    with (directory / '08_raw_frames.csv').open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=['host_time', 'packet_type', 'payload_hex'])
        writer.writeheader()
        writer.writerow({'host_time': (start + timedelta(seconds=30)).isoformat(),
                         'packet_type': '31', 'payload_hex': payload.hex()})
    report = adaptive_bench.analyze_recording(directory)
    assert report['sine']['valid']
    assert 0.1 < report['sine']['phase_delay_s'] < 0.3
    assert report['reversal']['valid']
    assert report['adaptive']['vb_scale'] == 1.25
    assert report['adaptive']['vb_effective'] == 75


def test_safety_and_acceptance_gates_are_separate():
    good = {'sine': {'valid': True, 'phase_delay_s': 0.1, 'amplitude_ratio': 1.0},
            'reversal': {'median_s': 0.4}}
    assert adaptive_bench.safety_ok(good) == (True, [])
    assert adaptive_bench.acceptance_ok(good) == (True, [])
    slow = {'sine': {'valid': True, 'phase_delay_s': 0.2, 'amplitude_ratio': 1.0},
            'reversal': {'median_s': 0.8}}
    assert adaptive_bench.safety_ok(slow) == (True, [])
    ok, reasons = adaptive_bench.acceptance_ok(slow)
    assert not ok and len(reasons) == 2
    unsafe = {'sine': {'valid': True, 'phase_delay_s': 0.1, 'amplitude_ratio': 1.2},
              'reversal': {'median_s': 0.4}}
    ok, reasons = adaptive_bench.safety_ok(unsafe)
    assert not ok and reasons == ['amplitude ratio outside 0.95-1.05']
    assert not adaptive_bench.acceptance_ok(unsafe)[0]
    loaded = {'sine': {'valid': True, 'phase_delay_s': 0.1, 'amplitude_ratio': 1.0},
              'reversal': {'median_s': 0.4}, 'max_load': 950}
    assert adaptive_bench.safety_ok(loaded)[1] == ['motor load >=900']
