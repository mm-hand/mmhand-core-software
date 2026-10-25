"""session_log.py - MMHand Desktop 模块契约。

职责：管理单次连接会话的文本日志文件、内存镜像和日志上下文。
线程：除 comm_layer 的串口收发线程外不主动创建 GUI 线程；共享状态必须按既有锁读取。
输入：P4 字节流、上层类型化命令、配置和持久化数据。
输出：线程安全的内存日志、持续落盘的会话文件、日志上下文。
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
# UI 只保留最近 2000 行、每行 300 字符；磁盘文件保存本次运行的全部日志。
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
        self._started = False
        self._file = None
        self._path = ""
        self._start_ts = datetime.now()
        self.add("SESSION", f"start {self._start_ts.strftime('%Y-%m-%d %H:%M:%S')}")

    def start(self, directory: str = SESSION_LOG_DIR) -> Optional[str]:
        """打开本次运行的会话文件，并把启动前已有的内存行补写进去。"""
        with self._lock:
            if self._file is not None:
                return self._path or None
            if self._closed:
                return None
            try:
                os.makedirs(directory, exist_ok=True)
                stamp = self._start_ts.strftime("%Y%m%d_%H%M%S")
                path = os.path.join(directory, f"session_{stamp}.log")
                # ponytail: 一次运行一个文件、不做轮转；单次运行日志无上限，需要长期留存再按天切分。
                handle = open(path, "a", encoding="utf-8", newline="\n")
            except OSError:
                # 日志目录不可写不能阻断主程序；内存日志仍然可用。
                return None
            self._file = handle
            self._path = path
            self._started = True
            try:
                for line in self._lines:
                    handle.write(line + "\n")
                handle.flush()
            except OSError:
                self._disable_file_no_lock()
                return None
            return path

    def add(self, source: str, message: str) -> None:
        if self._closed:
            return
        text = str(message).replace("\r", "").strip("\n")
        if not text:
            return
        prefix = f"{datetime.now().strftime('%H:%M:%S')} [{str(source).strip() or 'LOG'}] "
        with self._lock:
            for raw_line in text.splitlines():
                line = prefix + raw_line.strip()
                self._lines.append(line[: self.max_line_chars])
                self._sequence += 1
                self._entries.append((self._sequence, line[: self.max_line_chars]))
                self._write_no_lock(line)

    def since(self, sequence=0):
        with self._lock:
            return {'sequence': self._sequence, 'lines': [line for number, line in self._entries if number > sequence]}

    def snapshot(self) -> Iterable[str]:
        with self._lock:
            return list(self._lines)

    def close_and_save(self) -> Optional[str]:
        with self._lock:
            if self._closed:
                return self._path or None
            if not self._started:
                # 没有 start（例如单元测试直接构造 HandBackend）时不关闭全局日志，
                # 否则后续测试的 log_session 会被静默丢弃。
                return None
            self._closed = True
            path = self._path or None
            if self._file is not None:
                try:
                    self._write_no_lock(f"{datetime.now().strftime('%H:%M:%S')} [SESSION] closing")
                    self._file.flush()
                except OSError:
                    pass
                try:
                    self._file.close()
                except OSError:
                    pass
            self._file = None
            return path

    def _write_no_lock(self, line: str) -> None:
        if self._file is None:
            return
        try:
            self._file.write(line + "\n")
            self._file.flush()
        except OSError:
            self._disable_file_no_lock()

    def _disable_file_no_lock(self) -> None:
        try:
            if self._file is not None:
                self._file.close()
        except OSError:
            pass
        self._file = None
        self._path = ""



_SESSION_LOG = SessionLog()


def start_session_log(directory: str = SESSION_LOG_DIR) -> Optional[str]:
    return _SESSION_LOG.start(directory)


def log_session(source: str, message: str) -> None:
    _SESSION_LOG.add(source, message)


def session_log_since(sequence=0):
    return _SESSION_LOG.since(sequence)


def close_session_log() -> Optional[str]:
    return _SESSION_LOG.close_and_save()
