from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

from desktop.visualize.plot_sine_response import (
    PLOT_DATA_FIELDS,
    _interpolate_within_samples,
    write_plot_data_csv,
)


def _write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def test_interpolation_leaves_values_outside_source_range_empty() -> None:
    result = _interpolate_within_samples(
        np.asarray([-1.0, 0.5, 2.0]),
        np.asarray([0.0, 1.0]),
        np.asarray([10.0, 20.0]),
    )

    assert np.isnan(result[0])
    assert result[1] == 15.0
    assert np.isnan(result[2])


def test_plot_csv_contains_only_display_window_and_compact_columns(
    tmp_path: Path,
) -> None:
    session = tmp_path / "数据记录_20260922_100000"
    _write_csv(
        session / "02_关节总表.csv",
        ["sample_index", "host_time", "p4_time_ms", "M15_cmd_abs"],
        [
            {"sample_index": 0, "host_time": "t0", "p4_time_ms": 1000, "M15_cmd_abs": 100},
            {"sample_index": 1, "host_time": "t1", "p4_time_ms": 2000, "M15_cmd_abs": 200},
            {"sample_index": 2, "host_time": "t2", "p4_time_ms": 3000, "M15_cmd_abs": 300},
        ],
    )
    _write_csv(
        session / "03_舵机总表.csv",
        ["sample_index", "host_time", "p4_time_ms", "M15_abs"],
        [
            {"sample_index": 0, "host_time": "t0", "p4_time_ms": 1000, "M15_abs": 90},
            {"sample_index": 1, "host_time": "t1", "p4_time_ms": 2000, "M15_abs": 190},
            {"sample_index": 2, "host_time": "t2", "p4_time_ms": 3000, "M15_abs": 290},
        ],
    )
    trace = (
        session,
        np.asarray([0.0, 1.0, 2.0]),
        np.asarray([10.0, 20.0, 30.0]),
        np.asarray([0.0, 1.0, 2.0]),
        np.asarray([9.0, 19.0, 29.0]),
        1000.0,
        None,
    )
    output = tmp_path / "plot.csv"

    write_plot_data_csv(
        output,
        [12],
        {(session, 12): [15]},
        {12: [trace]},
        start_s=0.5,
        end_s=1.5,
    )

    with output.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)

    assert tuple(reader.fieldnames or ()) == PLOT_DATA_FIELDS
    assert len(rows) == 1
    assert rows[0] == {
        "session": "20260922_100000",
        "joint": "J12",
        "motor": "M15",
        "time_s": "1.0",
        "target_deg": "20.0",
        "actual_deg": "19.0",
        "cmd_counts": "200.0",
        "position_counts": "190.0",
    }
