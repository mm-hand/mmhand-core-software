import csv
import json

import pytest

from desktop.algorithm.replay_csv_loader import load_replay_csv
from desktop.core.protocol import ENCODER_COUNT


def _targets(value):
    return json.dumps([float(value + joint) for joint in range(ENCODER_COUNT)])


def _write_csv(path, rows):
    fieldnames = [
        "host_time",
        "source",
        "source_kind",
        "frame_kind",
        "replayable",
        "stream_id",
        "valid",
        "active_joint_mask",
        "command_joint_mask",
        "live_enabled",
        "blocked",
        "sent_to_backend",
        "mapping_id",
        "mapping_version",
        "urdf_config_crc",
        "output_joint_targets_json",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def test_loader_restores_full_timeline_and_actual_output(tmp_path):
    path = tmp_path / "07_遥操作总表.csv"
    _write_csv(
        path,
        [
            {
                "host_time": "2026-07-23 10:00:00.000",
                "source": "ROS2:/raw_ik_target",
                "valid": 1,
                "active_joint_mask": "0x00000003",
                "command_joint_mask": "0x00000003",
                "blocked": 0,
                "sent_to_backend": 1,
                "mapping_id": "mapping",
                "mapping_version": 3,
                "urdf_config_crc": 123,
                "output_joint_targets_json": _targets(1),
            },
            {
                "host_time": "2026-07-23 10:00:00.100",
                "source": "ROS2:/raw_ik_target",
                "valid": 1,
                "active_joint_mask": "0x00000003",
                "command_joint_mask": "0x00000003",
                "blocked": 0,
                "sent_to_backend": 0,
                "mapping_id": "mapping",
                "mapping_version": 3,
                "urdf_config_crc": 123,
                "output_joint_targets_json": _targets(50),
            },
            {
                "host_time": "2026-07-23 10:00:00.250",
                "source": "ROS2:/raw_ik_target",
                "valid": 1,
                "active_joint_mask": "0x00000003",
                "command_joint_mask": "0x00000003",
                "blocked": 0,
                "sent_to_backend": 1,
                "mapping_id": "mapping",
                "mapping_version": 3,
                "urdf_config_crc": 123,
                "output_joint_targets_json": _targets(3),
            },
        ],
    )

    trace = load_replay_csv(path)

    assert trace.frame_count == 2
    assert trace.duration_s == pytest.approx(0.25)
    assert trace.frames[0].target_urdf_deg[0] == 1.0
    assert trace.frames[1].target_urdf_deg[0] == 3.0
    assert trace.active_joint_mask == 3
    assert trace.dropped_row_count == 1


def test_loader_rejects_timestamp_rollback(tmp_path):
    path = tmp_path / "teleop.csv"
    base = {
        "source": "ROS2:/raw_ik_target",
        "valid": 1,
        "active_joint_mask": "1",
        "command_joint_mask": "1",
        "blocked": 0,
        "sent_to_backend": 1,
        "mapping_id": "mapping",
        "mapping_version": 3,
        "urdf_config_crc": 123,
        "output_joint_targets_json": _targets(1),
    }
    _write_csv(
        path,
        [
            {**base, "host_time": "2026-07-23 10:00:01.000"},
            {**base, "host_time": "2026-07-23 10:00:00.000"},
        ],
    )

    with pytest.raises(ValueError, match="倒退"):
        load_replay_csv(path)


def test_loader_reads_legacy_input_csv_without_reintroducing_legacy_writes(tmp_path):
    path = tmp_path / "16_遥操作输入_teleop_input.csv"
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "host_time_iso",
                "source",
                "valid",
                "mapping_id",
                "mapping_version",
                "urdf_config_crc",
                "joint_targets_json",
            ],
        )
        writer.writeheader()
        for milliseconds, value in ((0, 1), (50, 2)):
            writer.writerow(
                {
                    "host_time_iso": f"2026-07-23 10:00:00.{milliseconds:03d}",
                    "source": "ROS2:/raw_ik_target",
                    "valid": 1,
                    "mapping_id": "mapping",
                    "mapping_version": 3,
                    "urdf_config_crc": 123,
                    "joint_targets_json": _targets(value),
                }
            )

    trace = load_replay_csv(path)

    assert trace.frame_count == 2
    assert trace.duration_s == pytest.approx(0.05)
    assert any("sent_to_backend" in warning for warning in trace.warnings)
    assert any("active scope" in warning for warning in trace.warnings)


def test_loader_accepts_visual_retargeting_continuous_stream(tmp_path):
    path = tmp_path / "07_遥操作总表.csv"
    base = {
        "source": "Visual Retargeting",
        "valid": 1,
        "active_joint_mask": "0x00000003",
        "command_joint_mask": "0x00000003",
        "live_enabled": 1,
        "blocked": 0,
        "sent_to_backend": 1,
        "mapping_id": "mapping",
        "mapping_version": 3,
        "urdf_config_crc": 123,
    }
    _write_csv(
        path,
        [
            {
                **base,
                "host_time": "2026-07-23 10:00:00.000",
                "output_joint_targets_json": _targets(1),
            },
            {
                **base,
                "host_time": "2026-07-23 10:00:00.050",
                "output_joint_targets_json": _targets(2),
            },
        ],
    )

    trace = load_replay_csv(path)

    assert trace.frame_count == 2
    assert trace.frames[0].source_kind == "visual_retargeting"
    assert all(frame.frame_kind == "continuous" for frame in trace.frames)


def test_loader_keeps_single_ui_send_as_discrete_frame(tmp_path):
    path = tmp_path / "07_遥操作总表.csv"
    _write_csv(
        path,
        [
            {
                "host_time": "2026-07-23 10:00:00.000",
                "source": "Send J03",
                "valid": 1,
                "active_joint_mask": "0x00000008",
                "command_joint_mask": "0x00000008",
                "live_enabled": 0,
                "blocked": 0,
                "sent_to_backend": 1,
                "mapping_id": "mapping",
                "mapping_version": 3,
                "urdf_config_crc": 123,
                "output_joint_targets_json": _targets(3),
            }
        ],
    )

    trace = load_replay_csv(path)

    assert trace.frame_count == 1
    assert trace.frames[0].frame_kind == "discrete"
    assert trace.frames[0].command_joint_mask == 0x08


def test_loader_accepts_joint_wide_current_csv(tmp_path):
    path = tmp_path / "02_关节总表.csv"
    fieldnames = ["host_time"]
    for joint in range(ENCODER_COUNT):
        label = f"J{joint:02d}"
        fieldnames.extend(
            [
                f"{label}_current_deg",
                f"{label}_current_valid",
                f"{label}_active",
            ]
        )
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for milliseconds, value in ((0, 10.0), (50, 11.0)):
            row = {"host_time": f"2026-07-23 10:00:00.{milliseconds:03d}"}
            for joint in range(ENCODER_COUNT):
                label = f"J{joint:02d}"
                row[f"{label}_current_deg"] = value if joint == 0 else ""
                row[f"{label}_current_valid"] = 1 if joint == 0 else 0
                row[f"{label}_active"] = 1 if joint == 0 else 0
            writer.writerow(row)

    trace = load_replay_csv(path)

    assert trace.data_kind == "joint_actual"
    assert trace.active_joint_mask == 1
    assert [frame.target_urdf_deg[0] for frame in trace.frames] == [10.0, 11.0]
