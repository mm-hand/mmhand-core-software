from __future__ import annotations

import json
import os
import queue
import shutil
import threading
import time
from datetime import datetime
from typing import Dict, Iterable, Optional

from .csv_exporter import CsvSessionExporter
from .event_recorder import EventRecorder
from .hdf5_recorder import Hdf5SessionRecorder
from .topic_schema import (
    CSV_TOPIC_FILENAMES,
    DEFAULT_CSV_TOPICS,
    DEFAULT_TOPICS,
    TOPIC_FIELDNAMES,
)
from desktop.recording.rosbag_recorder import RosbagSessionRecorder


RUN_DATA_SESSION_KEEP_DIRS = 20


class DataSessionRecorder:
    """Topic-oriented multimodal recorder.

    The recorder writes human-readable per-topic CSV files immediately and can
    optionally emit HDF5 plus ROS2 bag streams for analysis and replay.
    """

    def __init__(self, flush_every: int = 10, max_queue: int = 5000, hdf5_compression: str = "gzip"):
        self.flush_every = max(1, int(flush_every))
        self.max_queue = max(100, int(max_queue))
        self._csv = CsvSessionExporter(flush_every=flush_every)
        self._hdf5 = Hdf5SessionRecorder(compression=hdf5_compression, flush_every=flush_every)
        self._rosbag = RosbagSessionRecorder()
        self._event = EventRecorder(self.write)
        self._queue: queue.Queue[tuple[str, Dict[str, object]]] = queue.Queue(maxsize=self.max_queue)
        # Replay要求恢复每一帧已下发目标，遥操轨迹不能沿用普通遥测的“队满丢最旧”策略。
        # 独立队列由同一个writer线程落盘，因此不引入CSV/HDF5并发写入。
        self._teleop_queue: queue.Queue[tuple[str, Dict[str, object]]] = queue.Queue()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._running = False
        self._session_dir = ""
        self._hdf5_path = ""
        self._rosbag_path = ""
        self._csv_enabled = False
        self._topic_fieldnames: Dict[str, Iterable[str]] = {}
        self._topic_sample_indices: Dict[str, int] = {}

    @property
    def path(self) -> str:
        return self._session_dir

    @property
    def hdf5_path(self) -> str:
        return self._hdf5_path

    @property
    def hdf5_enabled(self) -> bool:
        return bool(self._hdf5.enabled)

    @property
    def hdf5_error(self) -> str:
        return self._hdf5.error

    @property
    def rosbag_path(self) -> str:
        return self._rosbag_path

    @property
    def rosbag_enabled(self) -> bool:
        return bool(self._rosbag.enabled)

    @property
    def rosbag_error(self) -> str:
        return self._rosbag.error

    def start(
        self,
        root_dir: str,
        *,
        session_name: Optional[str] = None,
        meta: Optional[Dict[str, object]] = None,
        topics: Optional[Iterable[str]] = None,
        csv_topics: Optional[Iterable[str]] = None,
        enable_csv: bool = True,
        enable_hdf5: bool = True,
        enable_rosbag: bool = False,
    ) -> None:
        self.stop()
        os.makedirs(root_dir, exist_ok=True)
        self._prune_old_sessions(root_dir)
        session = session_name or f"数据记录_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        session_dir = os.path.join(root_dir, session)
        os.makedirs(session_dir, exist_ok=True)
        selected_topics = list(topics or DEFAULT_TOPICS)
        fieldnames = {topic: TOPIC_FIELDNAMES[topic] for topic in selected_topics if topic in TOPIC_FIELDNAMES}
        selected_csv_topics = list(csv_topics or DEFAULT_CSV_TOPICS)
        csv_fieldnames = {
            topic: TOPIC_FIELDNAMES[topic]
            for topic in selected_csv_topics
            if topic in TOPIC_FIELDNAMES
        }
        started_host_time = time.time()
        meta_row = dict(meta or {})
        meta_row.update(
            {
                "session_name": session,
                "session_dir": session_dir,
                "started_host_time": _host_time_text(started_host_time),
                "topics": selected_topics,
                "csv_topics": selected_csv_topics,
                "csv_topic_filenames": {
                    topic: CSV_TOPIC_FILENAMES.get(topic, f"{topic}.csv")
                    for topic in selected_csv_topics
                },
                "csv_requested": bool(enable_csv),
                "hdf5_requested": bool(enable_hdf5),
                "rosbag_requested": bool(enable_rosbag),
                "csv_enabled": bool(enable_csv),
            }
        )
        self._write_meta(session_dir, meta_row)
        if enable_csv:
            self._csv.start(
                session_dir,
                csv_fieldnames,
                topic_filenames=CSV_TOPIC_FILENAMES,
            )
        else:
            self._csv.stop()
        hdf5_path = os.path.join(session_dir, "训练数据_session.h5")
        if enable_hdf5:
            self._hdf5.start(hdf5_path, fieldnames, meta=meta_row)
        else:
            self._hdf5.start("", fieldnames, meta=meta_row)
            self._hdf5.enabled = False
            self._hdf5.error = "disabled"
        rosbag_path = os.path.join(session_dir, "ROS2_bag")
        if enable_rosbag:
            self._rosbag.start(rosbag_path, fieldnames, meta=meta_row)
        else:
            self._rosbag.stop()
            self._rosbag.error = "disabled"
        meta_row["hdf5_enabled"] = self._hdf5.enabled
        meta_row["hdf5_error"] = self._hdf5.error
        meta_row["rosbag_enabled"] = self._rosbag.enabled
        meta_row["rosbag_error"] = self._rosbag.error
        meta_row["rosbag_path"] = rosbag_path if self._rosbag.enabled else ""
        self._write_meta(session_dir, meta_row)

        with self._lock:
            self._session_dir = session_dir
            self._hdf5_path = hdf5_path if self._hdf5.enabled else ""
            self._rosbag_path = rosbag_path if self._rosbag.enabled else ""
            self._csv_enabled = bool(enable_csv)
            self._topic_fieldnames = fieldnames
            self._topic_sample_indices = {}
            self._stop.clear()
            self._running = True
            self._thread = threading.Thread(target=self._worker, daemon=True)
            self._thread.start()

        self.event("save_data_start", source="recorder", detail=session)

    def stop(self) -> None:
        thread = None
        with self._lock:
            if not self._running:
                self._csv.stop()
                self._hdf5.stop(write_file=False)
                self._rosbag.stop()
                return
            self._running = False
            self._stop.set()
            thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            # The worker owns all CSV/HDF5/rosbag writes. Let it drain the
            # bounded queue before closing writers so two threads never write
            # or close the same file concurrently.
            thread.join()
        elif thread is None:
            self._drain_queue()
        self._csv.stop()
        self._hdf5.stop(write_file=True)
        self._rosbag.stop()
        with self._lock:
            self._thread = None
            self._session_dir = ""
            self._hdf5_path = ""
            self._rosbag_path = ""
            self._csv_enabled = False
            self._topic_fieldnames = {}
            self._topic_sample_indices = {}

    def is_running(self) -> bool:
        with self._lock:
            return self._running

    def write(self, topic: str, row: Dict[str, object]) -> None:
        if not self.is_running():
            return
        item = (str(topic), dict(row))
        if str(topic) == "teleop_input":
            self._teleop_queue.put_nowait(item)
            return
        try:
            self._queue.put_nowait(item)
        except queue.Full:
            try:
                self._queue.get_nowait()
                self._queue.task_done()
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(item)
            except queue.Full:
                pass

    def event(self, event_type: str, **kwargs) -> None:
        if not self.is_running():
            return
        self._event.event(event_type, **kwargs)

    def _worker(self) -> None:
        while (
            not self._stop.is_set()
            or not self._teleop_queue.empty()
            or not self._queue.empty()
        ):
            try:
                topic, row = self._teleop_queue.get_nowait()
            except queue.Empty:
                pass
            else:
                self._write_now(topic, row)
                self._teleop_queue.task_done()
                continue
            try:
                topic, row = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue
            self._write_now(topic, row)
            self._queue.task_done()

    def _drain_queue(self) -> None:
        while True:
            try:
                topic, row = self._teleop_queue.get_nowait()
            except queue.Empty:
                break
            self._write_now(topic, row)
            self._teleop_queue.task_done()
        while True:
            try:
                topic, row = self._queue.get_nowait()
            except queue.Empty:
                break
            self._write_now(topic, row)
            self._queue.task_done()

    def _write_now(self, topic: str, row: Dict[str, object]) -> None:
        row = self._normalize_time_fields(topic, row)
        if self._csv_enabled:
            self._csv.write_row(topic, row)
        self._hdf5.write_row(topic, row)
        self._rosbag.write_row(topic, row)

    def _normalize_time_fields(self, topic: str, row: Dict[str, object]) -> Dict[str, object]:
        """统一生成上位机/P4两条时间轴，并彻底移除旧的伪设备时间与 elapsed 字段。"""
        out = dict(row)
        host_time = _to_float_or_none(out.get("host_time"))
        if host_time is None:
            host_time = time.time()
        out["host_time"] = _host_time_text(host_time)
        # ROS bag 仍需 Unix ns 作为索引，但该内部字段不会写入 CSV/HDF5/消息正文。
        out["_host_time_ns"] = int(host_time * 1_000_000_000)
        p4_time_ms = out.get("p4_time_ms", "")
        if p4_time_ms not in ("", None):
            try:
                out["p4_time_ms"] = int(p4_time_ms)
            except Exception:
                out["p4_time_ms"] = ""

        sample_index = self._topic_sample_indices.get(topic, 0)
        self._topic_sample_indices[topic] = sample_index + 1
        out.setdefault("sample_index", sample_index)
        return out

    @staticmethod
    def _write_meta(session_dir: str, meta: Dict[str, object]) -> None:
        path = os.path.join(session_dir, "meta.json")
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(meta, handle, ensure_ascii=False, indent=2, default=str)
            handle.write("\n")
        readable_path = os.path.join(session_dir, "00_说明_meta.json")
        with open(readable_path, "w", encoding="utf-8") as handle:
            json.dump(meta, handle, ensure_ascii=False, indent=2, default=str)
            handle.write("\n")

    @staticmethod
    def _prune_old_sessions(root_dir: str) -> None:
        try:
            entries = [
                os.path.join(root_dir, name)
                for name in os.listdir(root_dir)
                if (name.startswith("session_") or name.startswith("数据记录_"))
                and os.path.isdir(os.path.join(root_dir, name))
            ]
        except OSError:
            return
        entries.sort(key=lambda item: os.path.getmtime(item), reverse=True)
        for path in entries[RUN_DATA_SESSION_KEEP_DIRS - 1:]:
            try:
                shutil.rmtree(path)
            except OSError:
                pass


def _host_time_text(host_time: float) -> str:
    try:
        return datetime.fromtimestamp(float(host_time)).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    except Exception:
        return ""


def _to_float_or_none(value: object) -> Optional[float]:
    if value in ("", None):
        return None
    try:
        return float(value)
    except Exception:
        return None
