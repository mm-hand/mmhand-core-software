"""session_log.py - MMHand Desktop 模块契约。

职责：管理单次连接会话的文本日志文件和日志上下文。
线程：除 comm_layer 的串口收发线程外不主动创建 GUI 线程；共享状态必须按既有锁读取。
输入：P4 字节流、上层类型化命令、配置和持久化数据。
输出：线程安全的 HandModel 快照、协议帧、ACK 和后端操作结果。
状态所有权：通信对象拥有串口与解析缓存；HandModel 的写入只允许在核心同步边界内发生。
禁止依赖：禁止依赖 NiceGUI/UI、app、ROS2 或 Replay 页面，禁止把控件和展示规则带入核心层。
"""
import os
from collections import deque
from datetime import datetime
from threading import Lock
from typing import Deque, Iterable, Optional

from desktop.paths import LOG_DIR

SESSION_LOG_DIR = str(LOG_DIR)
SESSION_LOG_MAX_LINES = 2000
SESSION_LOG_MAX_LINE_CHARS = 300


class SessionLog:
    def __init__(
        self,
        max_lines: int = SESSION_LOG_MAX_LINES,
        max_line_chars: int = SESSION_LOG_MAX_LINE_CHARS,
    ):
        self.max_line_chars = max(40, int(max_line_chars))
        self._lines: Deque[str] = deque(maxlen=max(100, int(max_lines)))
        self._lock = Lock()
        self._sequence = 0
        self._entries = deque(maxlen=max(100, int(max_lines)))
        self._closed = False
        self._start_ts = datetime.now()
        self.add("SESSION", f"start {self._start_ts.strftime('%Y-%m-%d %H:%M:%S')}")

    def add(self, source: str, message: str) -> None:
        if self._closed:
            return
        text = str(message).replace("\r", "").strip("\n")
        if not text:
            return
        prefix = f"{datetime.now().strftime('%H:%M:%S')} [{str(source).strip() or 'LOG'}] "
        for raw_line in text.splitlines():
            line = (prefix + raw_line.strip())[: self.max_line_chars]
            with self._lock:
                self._lines.append(line)
                self._sequence += 1
                self._entries.append((self._sequence, line))

    def since(self, sequence=0):
        with self._lock:
            return {'sequence': self._sequence, 'lines': [line for number, line in self._entries if number > sequence]}

    def snapshot(self) -> Iterable[str]:
        with self._lock:
            return list(self._lines)

    def close_and_save(self) -> Optional[str]:
        if self._closed:
            return None
        self.add("SESSION", "closing")
        self._closed = True
        os.makedirs(SESSION_LOG_DIR, exist_ok=True)
        stamp = self._start_ts.strftime("%Y%m%d_%H%M%S")
        path = os.path.join(SESSION_LOG_DIR, f"session_{stamp}.log")
        lines = list(self.snapshot())
        with open(path, "w", encoding="utf-8", newline="\n") as handle:
            handle.write("\n".join(lines))
            handle.write("\n")
        return path



_SESSION_LOG = SessionLog()


def log_session(source: str, message: str) -> None:
    _SESSION_LOG.add(source, message)


def session_log_since(sequence=0):
    return _SESSION_LOG.since(sequence)


def close_session_log() -> Optional[str]:
    return _SESSION_LOG.close_and_save()
