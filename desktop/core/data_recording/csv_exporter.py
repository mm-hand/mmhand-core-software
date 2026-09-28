"""csv_exporter.py - MMHand Desktop 模块契约。

职责：将类型化 Topic 样本输出为便于人工查看的分表 CSV。
线程：采样入口可被通信线程调用，磁盘写入由记录器串行化；不得阻塞设备控制。
输入：带双时间戳的类型化 Topic 样本、事件和会话配置。
输出：CSV 文件以及结构化写入状态。
状态所有权：记录会话拥有文件句柄、采样序号和降采样缓存，不拥有设备真值。
禁止依赖：禁止依赖 UI 或改变控制状态，禁止在 writer 内重新解释协议业务语义。
"""
from __future__ import annotations

import csv
import os
import threading
from typing import Dict, Iterable, Optional

class CsvTopicWriter:
    def __init__(self, path: str, fieldnames: Iterable[str], flush_every: int = 10):
        self.path = path
        self.fieldnames = list(fieldnames)
        self.flush_every = max(1, int(flush_every))
        self._file = open(path, "w", newline="", encoding="utf-8-sig")
        self._writer = csv.DictWriter(self._file, fieldnames=self.fieldnames, extrasaction="ignore")
        self._writer.writeheader()
        self._pending = 0

    def write_row(self, row: Dict[str, object]) -> None:
        self._writer.writerow(row)
        self._pending += 1

    def flush(self) -> None:
        try:
            self._file.flush()
        finally:
            self._pending = 0

    def close(self) -> None:
        try:
            self.flush()
        finally:
            self._file.close()


class CsvSessionExporter:
    def __init__(self, flush_every: int = 10):
        self.flush_every = max(1, int(flush_every))
        self.session_dir = ""
        self._writers: Dict[str, CsvTopicWriter] = {}
        self._fieldnames: Dict[str, list[str]] = {}
        self._topic_filenames: Dict[str, str] = {}
        self._lock = threading.Lock()

    def start(
        self,
        session_dir: str,
        topic_fieldnames: Dict[str, Iterable[str]],
        topic_filenames: Optional[Dict[str, str]] = None,
    ) -> None:
        self.stop()
        os.makedirs(session_dir, exist_ok=True)
        self.session_dir = session_dir
        self._fieldnames = {name: list(fields) for name, fields in topic_fieldnames.items()}
        self._topic_filenames = dict(topic_filenames or {})
        with self._lock:
            for topic, fields in self._fieldnames.items():
                path = self._path_for_topic(topic)
                self._writers[topic] = CsvTopicWriter(path, fields, flush_every=self.flush_every)

    def stop(self) -> None:
        with self._lock:
            writers = list(self._writers.values())
            self._writers = {}
            self._fieldnames = {}
            self._topic_filenames = {}
            self.session_dir = ""
        first_error: Optional[Exception] = None
        for writer in writers:
            try:
                writer.close()
            except Exception as exc:
                # 一个文件写满不能阻止其余文件句柄关闭；首个错误交给记录器统一上报。
                if first_error is None:
                    first_error = exc
        if first_error is not None:
            raise first_error

    def write_row(self, topic: str, row: Dict[str, object]) -> None:
        with self._lock:
            # CSV只允许启动时明确选择的总表，防止长表/debug回调再次动态创建文件。
            if topic not in self._fieldnames:
                return
            writer: Optional[CsvTopicWriter] = self._writers.get(topic)
            if writer is None:
                fields = self._fieldnames[topic]
                path = self._path_for_topic(topic)
                writer = CsvTopicWriter(path, fields, flush_every=self.flush_every)
                self._writers[topic] = writer
            writer.write_row(row)

    def flush(self) -> None:
        """由唯一 writer 每 50 ms 刷新全部有新行的文件。"""
        with self._lock:
            for writer in self._writers.values():
                if writer._pending:
                    writer.flush()

    def _path_for_topic(self, topic: str) -> str:
        filename = self._topic_filenames.get(topic) or f"{topic}.csv"
        return os.path.join(self.session_dir, filename)
