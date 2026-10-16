import csv
import json

import pytest

from desktop.replay.replay_csv_loader import load_replay_csv
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
        "joint_order_crc",
        "output_joint_targets_json",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        from desktop.core.data_recording.motion_sampler import classify_motion_source
        current=[]
        for index, original in enumerate(rows):
            row=dict(original)
            source,kind,replayable=classify_motion_source(row.get('source',''),bool(row.get('live_enabled')))
            row.setdefault('source_kind', source)
            row.setdefault('frame_kind', kind)
            row.setdefault('replayable', int(replayable))
            row.setdefault('stream_id', f'{source}-stream' if kind=='continuous' else f'event-{index}')
            current.append(row)
        writer.writerows(current)


def test_loader_restores_full_timeline_and_actual_output(tmp_path):
    path = tmp_path / "07_遥操作总表.csv"
    _write_csv(
        path,
        [
            {
                "host_time": "2026-07-23 10:00:00.000",
                "source": "ROS2:/mmhand/joint_angle_deg",
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
                "source": "ROS2:/mmhand/joint_angle_deg",
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
                "source": "ROS2:/mmhand/joint_angle_deg",
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
        "source": "ROS2:/mmhand/joint_angle_deg",
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

    with pytest.raises(ValueError, match="moves backward"):
        load_replay_csv(path)


def test_loader_rejects_legacy_input_csv(tmp_path):
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
                    "source": "ROS2:/mmhand/joint_angle_deg",
                    "valid": 1,
                    "mapping_id": "mapping",
                    "mapping_version": 3,
                    "urdf_config_crc": 123,
                    "joint_targets_json": _targets(value),
                }
            )

    with pytest.raises(ValueError):
        load_replay_csv(path)


def test_loader_accepts_external_ros2_continuous_stream(tmp_path):
    path = tmp_path / "07_遥操作总表.csv"
    base = {
        "source": "ROS2:/mmhand/joint_angle_deg",
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
    assert trace.frames[0].source_kind == "ros2_external"
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


def test_loader_rejects_legacy_target_only_columns(tmp_path):
    path = tmp_path / "sparse_targets.csv"
    fieldnames = [
        "host_time",
        "source",
        "valid",
        "sent_to_backend",
        "active_joint_mask",
        "J00_target_deg",
        "J02_target_deg",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerow(
            {
                "host_time": "2026-07-23 10:00:00.000",
                "source": "UI Feedback",
                "valid": 1,
                "sent_to_backend": 1,
                "active_joint_mask": "0x5",
                "J00_target_deg": 10.0,
                "J02_target_deg": 20.0,
            }
        )
        writer.writerow(
            {
                "host_time": "2026-07-23 10:00:00.050",
                "source": "UI Feedback",
                "valid": 1,
                "sent_to_backend": 1,
                "active_joint_mask": "0x5",
                "J00_target_deg": 11.0,
                "J02_target_deg": "",
            }
        )

    with pytest.raises(ValueError):
        load_replay_csv(path)


def test_loader_accepts_joint_current_csv_with_only_one_named_joint(tmp_path):
    path = tmp_path / "partial_joint_state.csv"
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "host_time",
                "J14_current_deg",
                "J14_current_valid",
                "J14_active",
            ],
        )
        writer.writeheader()
        writer.writerow(
            {
                "host_time": "2026-07-23 10:00:00.000",
                "J14_current_deg": 6.0,
                "J14_current_valid": 1,
                "J14_active": 1,
            }
        )
        writer.writerow(
            {
                "host_time": "2026-07-23 10:00:00.050",
                "J14_current_deg": 7.0,
                "J14_current_valid": 1,
                "J14_active": 1,
            }
        )

    trace = load_replay_csv(path)

    assert trace.data_kind == "joint_actual"
    assert trace.active_joint_mask == (1 << 14)
    assert trace.frames[0].joint_value_mask == (1 << 14)
    assert trace.frames[-1].target_urdf_deg[14] == 7.0
