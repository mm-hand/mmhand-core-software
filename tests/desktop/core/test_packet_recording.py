import csv
import struct
import time

from desktop.core.data_recording.packet_recorder import PacketRecorder
from desktop.core.data_recording.recorder import DataSessionRecorder
from desktop.core.data_recording.topic_schema import CSV_TOPIC_FILENAMES
from desktop.core.protocol import PACKET_TYPE_CONTROL_TRACE


def frames(sequence, stamp):
    base = (struct.pack(">7I", sequence, 2, stamp - 2, stamp - 1, 1, 1, 1)
            + struct.pack(">21h", *range(21)) * 2 + struct.pack(">27i", *range(27)))
    detail = (struct.pack(">6I", sequence, stamp - 3, 10000, 1, 1, 0)
              + struct.pack(">27i", *range(100, 127))
              + struct.pack(">27h", *range(27)) * 2)
    assert len(base) == 220 and len(detail) == 240
    return [(PACKET_TYPE_CONTROL_TRACE, p + struct.pack(">I", stamp)) for p in (detail, base)]


def test_starting_new_recording_preserves_all_existing_sessions(tmp_path):
    existing = []
    for i in range(25):
        folder = tmp_path / f'数据记录_20260101_{i:06d}'
        folder.mkdir()
        path = folder / 'evidence.txt'
        path.write_text(str(i))
        existing.append(path)
    recorder = DataSessionRecorder()
    try:
        recorder.start(str(tmp_path))
    finally:
        recorder.stop()
    assert all(path.exists() and path.read_text() == str(i) for i, path in enumerate(existing))


def test_five_frames_in_one_batch_have_five_original_timestamps():
    rows = []
    recorder = PacketRecorder(lambda topic, row: rows.append((topic, row)))
    packets = sum((frames(i + 1, 1000 + i * 10) for i in range(5)), [])
    recorder.consume("p4", packets, 1.0)
    joints = [r for t, r in rows if t == "joint_wide"]
    assert [r["p4_time_ms"] for r in joints] == [1000, 1010, 1020, 1030, 1040]
    assert len([r for t, r in rows if t == "packet_frames"]) == 10
    assert all(r["trace_detail_valid"] == 1 for r in joints)
    assert all(r["M00_actual_abs"] == 100 for r in joints)
    assert joints[-1]["trace_sequence_gap"] == 0


def test_old_firmware_missing_detail_gap_and_retry():
    rows = []
    recorder = PacketRecorder(lambda topic, row: rows.append((topic, row)))
    recorder.consume("p4", [frames(1, 1000)[1], frames(3, 1020)[1], frames(3, 1020)[1]], 1)
    joints = [r for t, r in rows if t == "joint_wide"]
    assert len(joints) == 2
    assert joints[1]["trace_sequence_gap"] == 1
    assert joints[1]["trace_detail_valid"] == 0
    assert "M00_actual_abs" not in joints[1]


def test_stop_drains_batch_and_keeps_s3_payload(tmp_path):
    recorder = DataSessionRecorder()
    recorder.start(str(tmp_path), session_name="batch")
    recorder.record_packets("p4", sum((frames(i + 1, 1000 + i * 10) for i in range(5)), []))
    recorder.record_packets("s3", [(99, b"\x01\x02")])
    recorder.stop()
    with (tmp_path / "batch" / CSV_TOPIC_FILENAMES["joint_wide"]).open(encoding="utf-8-sig") as file:
        assert len(list(csv.DictReader(file))) == 5
    with (tmp_path / "batch" / CSV_TOPIC_FILENAMES["packet_frames"]).open(encoding="utf-8-sig") as file:
        rows = list(csv.DictReader(file))
        assert rows[-1]["source"] == "s3" and rows[-1]["payload_hex"] == "0102"


def test_periodic_flush_without_ten_rows(tmp_path):
    recorder = DataSessionRecorder()
    recorder.start(str(tmp_path), session_name="flush")
    try:
        recorder.record_packets("p4", frames(1, 1000))
        path = tmp_path / "flush" / CSV_TOPIC_FILENAMES["joint_wide"]
        deadline = time.monotonic() + 2
        found = False
        while time.monotonic() < deadline:
            with path.open(encoding="utf-8-sig") as file:
                found = len(list(csv.DictReader(file))) == 1
            if found:
                break
            time.sleep(0.02)
        assert found
    finally:
        recorder.stop()


def test_overflow_reports_incomplete_instead_of_discarding_oldest():
    recorder = DataSessionRecorder(max_queue=100)
    recorder._running = True
    recorder._selected_topics = {"event_log"}
    for i in range(101):
        recorder.write("event_log", {"detail": str(i)})
    assert not recorder.is_running()
    assert "overflow" in recorder.consume_error()
    assert recorder._queue.qsize() == 100
    assert recorder._queue.get_nowait()[1]["detail"] == "0"


def test_starting_recording_preserves_all_existing_sessions(tmp_path):
    from desktop.core.data_recording.recorder import DataSessionRecorder
    old = []
    for index in range(25):
        directory = tmp_path / f'session_{index:02d}'
        directory.mkdir()
        marker = directory / 'record.csv'
        marker.write_bytes(b'preserved historical data')
        old.append(marker)
    recorder = DataSessionRecorder()
    try:
        recorder.start(str(tmp_path), session_name='new-session')
    finally:
        recorder.stop()
    assert all(marker.read_bytes() == b'preserved historical data' for marker in old)
