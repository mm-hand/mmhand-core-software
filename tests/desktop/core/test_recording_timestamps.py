import csv
import json
import struct
import time
from datetime import datetime

import pytest

from desktop.core.data_recording.recorder import DataSessionRecorder
from desktop.core.data_recording.topic_schema import (
    CSV_TOPIC_FILENAMES,
    DEFAULT_CSV_TOPICS,
    DEFAULT_TOPICS,
    TOPIC_FIELDNAMES,
)
from desktop.replay.replay_csv_loader import load_replay_csv
from desktop.core.comm_layer import LowerComputerComm
from desktop.core.protocol import (
    PACKET_TYPE_PROTO_ACK,
    PACKET_TYPE_SENSOR,
    split_p4_timestamp,
)


def test_split_p4_timestamp_from_current_packet() -> None:
    payload = b"\x01\x02" + struct.pack(">I", 0x12345678)

    business_payload, p4_time_ms = split_p4_timestamp(PACKET_TYPE_SENSOR, payload)

    assert business_payload == b"\x01\x02"
    assert p4_time_ms == 0x12345678


def test_current_timestamped_packet_rejects_missing_trailer() -> None:
    business_payload, p4_time_ms = split_p4_timestamp(PACKET_TYPE_SENSOR, b"\x01\x02")

    assert business_payload == b""
    assert p4_time_ms is None


def test_protocol_ack_exposes_exact_p4_processing_time() -> None:
    comm = LowerComputerComm("")
    payload = bytes([0xCC, 0x03, 0x00]) + struct.pack(">I", 7654)

    model = comm._process_packets([(PACKET_TYPE_PROTO_ACK, payload)])

    assert model is not None
    assert model.p4_time_ms == 7654
    assert model.proto_ack_p4_time_ms == 7654
    assert model.proto_ack_sequence == 1


def test_csv_exposes_only_host_and_p4_time_axes(tmp_path) -> None:
    recorder = DataSessionRecorder()
    recorder.start(
        str(tmp_path),
        session_name="timestamp_test",
        topics=["core_state"],
    )
    session_dir = recorder.path
    recorder.write(
        "core_state",
        {
            "host_time": 1_700_000_000.125,
            "p4_time_ms": 4321,
            "mode": "monitor",
        },
    )
    recorder.stop()

    csv_path = tmp_path / "timestamp_test" / CSV_TOPIC_FILENAMES["core_state"]
    with csv_path.open("r", encoding="utf-8-sig", newline="") as handle:
        rows = list(csv.DictReader(handle))

    assert session_dir
    assert rows[-1]["host_time"] == datetime.fromtimestamp(
        1_700_000_000.125
    ).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    assert rows[-1]["p4_time_ms"] == "4321"
    assert TOPIC_FIELDNAMES["core_state"][:3] == [
        "sample_index",
        "host_time",
        "p4_time_ms",
    ]
    for legacy_field in (
        "host_time_iso",
        "host_time_unix_s",
        "elapsed_s",
        "device_timestamp",
        "device_elapsed_s",
    ):
        assert legacy_field not in TOPIC_FIELDNAMES["core_state"]


def test_background_disk_error_stops_recorder_without_thread_exception(
    tmp_path, monkeypatch
) -> None:
    recorder = DataSessionRecorder()
    recorder.start(str(tmp_path), session_name="disk_error", topics=["core_state"])
    monkeypatch.setattr(
        recorder._csv,
        "write_row",
        lambda _topic, _row: (_ for _ in ()).throw(OSError(28, "No space left on device")),
    )

    recorder.write("core_state", {"host_time": time.time(), "mode": "monitor"})
    deadline = time.time() + 1.0
    while recorder.is_running() and time.time() < deadline:
        time.sleep(0.01)

    assert recorder.is_running() is False
    assert "No space left on device" in recorder.consume_error()
    assert recorder.consume_error() == ""
    recorder.stop()


def test_teleop_recording_is_one_self_contained_trajectory_topic() -> None:
    assert "teleop_output" not in TOPIC_FIELDNAMES
    assert "teleop_output" not in CSV_TOPIC_FILENAMES
    fields = TOPIC_FIELDNAMES["teleop_input"]
    assert "input_joint_targets_json" in fields
    assert "output_joint_targets_json" in fields
    assert "sent_to_backend" in fields
    assert "active_joint_mask" in fields


def test_recorded_teleop_csv_can_restore_every_sent_output_frame(tmp_path) -> None:
    recorder = DataSessionRecorder()
    recorder.start(
        str(tmp_path),
        session_name="teleop_replay_roundtrip",
        topics=["teleop_input"],
    )
    for index, target in enumerate((1.0, 2.0, 3.0)):
        values = [target] * 21
        recorder.write(
            "teleop_input",
            {
                "host_time": 1_700_000_000.0 + index * 0.05,
                "source": "ROS2:/raw_ik_target",
                "valid": 1,
                "active_joint_mask": "0x001FFFFF",
                "command_joint_mask": "0x001FFFFF",
                "blocked": 0,
                "sent_to_backend": 1,
                "mapping_id": "",
                "mapping_version": "",
                "urdf_config_crc": "",
                "input_joint_targets_json": json.dumps(values),
                "output_joint_targets_json": json.dumps(values),
            },
        )
    recorder.stop()

    csv_path = (
        tmp_path
        / "teleop_replay_roundtrip"
        / CSV_TOPIC_FILENAMES["teleop_input"]
    )
    trace = load_replay_csv(csv_path)

    assert trace.frame_count == 3
    assert trace.duration_s == pytest.approx(0.1)
    assert [frame.target_urdf_deg[0] for frame in trace.frames] == [1.0, 2.0, 3.0]


def test_default_csv_layout_contains_only_six_merged_tables(tmp_path) -> None:
    recorder = DataSessionRecorder()
    recorder.start(
        str(tmp_path),
        session_name="merged_csv_layout",
        topics=DEFAULT_TOPICS,
    )
    # 未选择的旧数据 Topic 不能重新创建额外 CSV 文件。
    recorder.write("deprecated_topic", {"joint_index": 0, "valid": 1})
    recorder.stop()

    session_dir = tmp_path / "merged_csv_layout"
    csv_names = sorted(path.name for path in session_dir.glob("*.csv"))
    expected_names = sorted(CSV_TOPIC_FILENAMES[topic] for topic in DEFAULT_CSV_TOPICS)

    assert csv_names == expected_names
    assert len(csv_names) == 7
    assert {path.name for path in session_dir.iterdir()} == {
        *expected_names,
        "meta.json",
        "00_说明_meta.json",
    }


def test_tactile_full_option_adds_exactly_one_csv_table(tmp_path) -> None:
    recorder = DataSessionRecorder()
    recorder.start(
        str(tmp_path),
        session_name="merged_csv_with_tactile_full",
        topics=[*DEFAULT_TOPICS, "tactile_full"],
        csv_topics=[*DEFAULT_CSV_TOPICS, "tactile_full"],
    )
    recorder.stop()

    session_dir = tmp_path / "merged_csv_with_tactile_full"
    csv_names = sorted(path.name for path in session_dir.glob("*.csv"))
    expected_names = sorted(
        CSV_TOPIC_FILENAMES[topic]
        for topic in [*DEFAULT_CSV_TOPICS, "tactile_full"]
    )

    assert csv_names == expected_names
    assert len(csv_names) == 8


def test_merged_joint_and_servo_tables_expose_required_fields() -> None:
    joint_fields = TOPIC_FIELDNAMES["joint_wide"]
    servo_fields = TOPIC_FIELDNAMES["servo_wide"]

    assert "J00_current_deg" in joint_fields
    assert "J00_current_valid" in joint_fields
    assert "M26_pid_counts" in joint_fields
    assert "M00_current_raw" in servo_fields
    assert "M26_current_raw" in servo_fields
