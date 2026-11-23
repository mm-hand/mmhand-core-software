"""Offline tests for the adaptive bench runner; no serial device or GUI."""
import csv
from datetime import datetime, timedelta
import json
import math
import struct
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from desktop.core.protocol import ENCODER_COUNT
from tools.debug import adaptive_bench


def test_dry_run_reports_variants_and_flags(tmp_path, capsys):
    variants = tmp_path / 'variants.json'
    variants.write_text(json.dumps([{
        'name': 'V-test', 'kp_max': 1.6, 'kp_step': 1.08,
        'lag_ms': 300,
    }]), encoding='utf-8')
    csv_path = tmp_path / 'motion.csv'
    csv_path.write_text('host_time\n', encoding='utf-8')
    assert adaptive_bench.main(['--dry-run', '--variants', str(variants),
                                '--csv', str(csv_path)]) == 0
    output = capsys.readouterr().out
    assert 'V-test' in output
    assert '-DMMHAND_ADAPTIVE_KP_MAX=1.6' in output
    assert 'arduino-cli compile' in output


def test_run_replay_requires_device_confirmed_adaptive_kp(monkeypatch, tmp_path):
    from desktop.core import hand_backend
    from desktop.core.protocol import SYSTEM_STATE_READY

    hand = Mock()
    hand.get_state_snapshot.return_value = SimpleNamespace(system_state=SYSTEM_STATE_READY,
        has_adaptive_kp_data=True, adaptive_kp_enabled=False)
    monkeypatch.setattr(hand_backend, 'HandBackend', lambda: hand)
    monkeypatch.setattr(adaptive_bench, '_wait_result', Mock())

    with pytest.raises(RuntimeError, match='Enable Adaptive Control'):
        adaptive_bench.run_replay('test-port', tmp_path / 'motion.csv')
    hand.request_replay_start.assert_not_called()
    hand.close_application.assert_called_once()


def _write_payload(scale, effective):
    payload = bytes([1])
    payload += b''.join(struct.pack('>h', int(value * 1000)) for value in scale)
    payload += b''.join(struct.pack('>h', int(value * 1000)) for value in effective)
    assert len(payload) == 1 + 4 * ENCODER_COUNT
    return payload


def test_analysis_reads_delay_and_adaptive_kp(tmp_path):
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
    payload = _write_payload([1.25] * ENCODER_COUNT, [0.5] * ENCODER_COUNT)
    with (directory / '08_raw_frames.csv').open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=['host_time', 'packet_type', 'payload_hex'])
        writer.writeheader()
        writer.writerow({'host_time': (start + timedelta(seconds=30)).isoformat(),
                         'packet_type': '31', 'payload_hex': payload.hex()})
    report = adaptive_bench.analyze_recording(directory)
    assert report['sine']['valid']
    assert 0.1 < report['sine']['phase_delay_s'] < 0.3
    assert report['reversal']['valid']
    assert report['adaptive']['kp_scale'] == 1.25
    assert report['adaptive']['kp_effective'] == 0.5


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
    assert adaptive_bench.safety_ok(loaded) == (True, [])
