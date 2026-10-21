"""recorder.py - MMHand Desktop 模块契约。

职责：编排 CSV 和事件记录会话生命周期。
线程：采样入口可被通信线程调用，磁盘写入由记录器串行化；不得阻塞设备控制。
输入：带双时间戳的类型化 Topic 样本、事件和会话配置。
输出：CSV 文件以及结构化写入状态。
状态所有权：记录会话拥有文件句柄、采样序号和降采样缓存，不拥有设备真值。
禁止依赖：禁止依赖 UI 或改变控制状态，禁止在 writer 内重新解释协议业务语义。
"""
from __future__ import annotations

import json
import os
import queue
import threading
import time
from datetime import datetime
from typing import Dict, Iterable, Optional

from .csv_exporter import CsvSessionExporter
from .event_recorder import EventRecorder
from .packet_recorder import PacketRecorder
from .topic_schema import (
    CSV_TOPIC_FILENAMES,
    DEFAULT_TOPICS,
    TOPIC_FIELDNAMES,
)


class DataSessionRecorder:
    """按 Topic 异步写入人工可读 CSV 的会话记录器。"""

    def __init__(self, flush_every: int = 10, max_queue: int = 5000):
        self.flush_every = max(1, int(flush_every))
        self.max_queue = max(100, int(max_queue))
        self._csv = CsvSessionExporter(flush_every=flush_every)
        self._event = EventRecorder(self.write)
        self._queue: queue.Queue[tuple[str, Dict[str, object]]] = queue.Queue(maxsize=self.max_queue)
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._running = False
        self._session_dir = ""
        self._topic_sample_indices: Dict[str, int] = {}
        self._selected_topics: set[str] = set()
        self._fatal_error = ""
        self._packet_recorder = PacketRecorder(self._write_now)

    @property
    def path(self) -> str:
        return self._session_dir

    def start(
        self,
        root_dir: str,
        *,
        session_name: Optional[str] = None,
        meta: Optional[Dict[str, object]] = None,
        topics: Optional[Iterable[str]] = None,
        csv_topics: Optional[Iterable[str]] = None,
    ) -> None:
        self.stop()
        os.makedirs(root_dir, exist_ok=True)
        session = session_name or f"recordings_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}"
        session_dir = os.path.join(root_dir, session)
        os.makedirs(session_dir, exist_ok=True)
        selected_topics = list(topics or DEFAULT_TOPICS)
        selected_csv_topics = list(csv_topics if csv_topics is not None else selected_topics)
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
                "csv_enabled": True,
            }
        )
        self._write_meta(session_dir, meta_row)
        self._csv.start(
            session_dir,
            csv_fieldnames,
            topic_filenames=CSV_TOPIC_FILENAMES,
        )

        with self._lock:
            self._session_dir = session_dir
            self._topic_sample_indices = {}
            self._selected_topics = set(csv_fieldnames)
            self._stop.clear()
            self._fatal_error = ""
            self._packet_recorder = PacketRecorder(self._write_now)
            self._running = True
            self._thread = threading.Thread(target=self._worker, daemon=True)
            self._thread.start()

        self.event("save_data_start", source="recorder", detail=session)

    def stop(self) -> None:
        thread = None
        with self._lock:
            self._running = False
            self._stop.set()
            thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            # writer 线程负责排空队列后再关闭文件，避免跨线程关闭同一 CSV。
            thread.join()
        elif thread is None:
            self._drain_queue()
        try:
            self._csv.stop()
        except Exception as exc:
            self._mark_failed(exc)
        with self._lock:
            self._thread = None
            self._session_dir = ""
            self._topic_sample_indices = {}
            self._selected_topics = set()

    def is_running(self) -> bool:
        with self._lock:
            return self._running

    def consume_error(self) -> str:
        """取走后台写线程的首个致命错误，供 UI 主线程只提示一次。"""
        with self._lock:
            message = self._fatal_error
            self._fatal_error = ""
            return message

    def _mark_failed(self, exc: BaseException) -> None:
        """记录磁盘写入失败并关闭采样入口，禁止异常逃出后台线程。"""
        message = f"{type(exc).__name__}: {exc}"
        with self._lock:
            if not self._fatal_error:
                self._fatal_error = message
            self._running = False
            self._stop.set()

    def write(self, topic: str, row: Dict[str, object]) -> None:
        overflow = False
        with self._lock:
            if not self._running:
                return
            topic = str(topic)
            if topic not in self._selected_topics and topic != "_packets":
                return
            try:
                self._queue.put_nowait((topic, dict(row)))
            except queue.Full:
                self._running = False
                self._stop.set()
                overflow = True
        if overflow:
            self._mark_failed(RuntimeError("Recording queue overflow: session incomplete; no silent downsampling"))

    def record_packets(self, source: str, packets) -> None:
        """RX 线程只复制不可变载荷入队；解码和 CSV 写入均在 writer 中。"""
        if packets and self.is_running():
            self.write("_packets", {"source": source, "packets": tuple(
                (int(kind), bytes(payload)) for kind, payload in packets), "received_at": time.time()})

    def event(self, event_type: str, **kwargs) -> None:
        if not self.is_running():
            return
        self._event.event(event_type, **kwargs)

    def _worker(self) -> None:
        try:
            deadline = time.monotonic() + 0.05
            while (
                not self._stop.is_set()
                or not self._queue.empty()
            ):
                try:
                    topic, row = self._queue.get(timeout=max(0.0, deadline - time.monotonic()))
                except queue.Empty:
                    self._csv.flush()
                    deadline = time.monotonic() + 0.05
                    continue
                try:
                    self._write_now(topic, row)
                finally:
                    self._queue.task_done()
                if time.monotonic() >= deadline:
                    self._csv.flush()
                    deadline = time.monotonic() + 0.05
        except Exception as exc:
            self._mark_failed(exc)

    def _drain_queue(self) -> None:
        while True:
            try:
                topic, row = self._queue.get_nowait()
            except queue.Empty:
                break
            self._write_now(topic, row)
            self._queue.task_done()

    def _write_now(self, topic: str, row: Dict[str, object]) -> None:
        if topic == "_packets":
            self._packet_recorder.consume(row["source"], row["packets"], row["received_at"])
            return
        row = self._normalize_time_fields(topic, row)
        self._csv.write_row(topic, row)

    def _normalize_time_fields(self, topic: str, row: Dict[str, object]) -> Dict[str, object]:
        """统一生成上位机/P4两条时间轴，并彻底移除旧的伪设备时间与 elapsed 字段。"""
        out = dict(row)
        host_time = _to_float_or_none(out.get("host_time"))
        if host_time is None:
            host_time = time.time()
        out["host_time"] = _host_time_text(host_time)
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
