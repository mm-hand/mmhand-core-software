from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

from desktop.visualize.analyze_single_motor_pid import (
    JointTrace,
    analyze_steps,
    classify_target,
    load_joint_trace,
    resolve_motor,
)


def _write_csv(path: Path, fieldnames, rows) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def test_classifies_step_and_wave_targets() -> None:
    assert classify_target(np.asarray([0.0] * 10 + [5.0] * 10)) == "step"
    assert classify_target(np.sin(np.linspace(0.0, 8.0, 100))) == "wave"


def test_step_metrics_detect_rise_overshoot_and_settling() -> None:
    time_s = np.arange(0.0, 4.0, 0.1)
    target = np.where(time_s < 1.0, 0.0, 10.0)
    actual = np.zeros_like(time_s)
    after = time_s >= 1.0
    actual[after] = 10.0 * (1.0 - np.exp(-(time_s[after] - 1.0) / 0.25))
    empty = np.full(len(time_s), np.nan)
    trace = JointTrace(time_s, target, actual, empty, "synthetic")

    rows = analyze_steps(trace, tolerance_deg=0.5, settle_hold_s=0.3)

    assert len(rows) == 1
    assert 0.3 < rows[0]["rise_time_10_90_s"] < 0.8
    assert rows[0]["overshoot_pct"] < 1.0
    assert rows[0]["settling_time_s"] > 0.0


def test_loads_debug_data_and_infers_single_motor(tmp_path: Path) -> None:
    debug_rows = []
    for index in range(5):
        debug_rows.append(
            {
                "host_ts_wall": 100.0 + index * 0.01,
                "joint_index": 10,
                "valid": 1,
                "target_deg": 5.0,
                "actual_deg": 4.0 + index * 0.2,
                "loop1_output": 10.0,
            }
        )
    _write_csv(
        tmp_path / "14_joint_debug_packets.csv",
        list(debug_rows[0]),
        debug_rows,
    )
    _write_csv(
        tmp_path / "04_servo_state.csv",
        ["host_time", "joint_index", "motor_index", "online", "is_active_motor"],
        [
            {
                "host_time": 100.0,
                "joint_index": 10,
                "motor_index": 13,
                "online": 1,
                "is_active_motor": 1,
            }
        ],
    )

    trace = load_joint_trace(tmp_path, 10)

    assert len(trace.time_s) == 5
    assert trace.source == "14_joint_debug_packets.csv"
    assert resolve_motor(tmp_path, 10, None) == 13
