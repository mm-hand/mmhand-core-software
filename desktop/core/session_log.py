import os
from collections import deque
from datetime import datetime
from threading import Lock
from typing import Deque, Iterable, Optional

from desktop.paths import LOG_DIR

SESSION_LOG_DIR = str(LOG_DIR)
SESSION_LOG_MAX_LINES = 2000
SESSION_LOG_MAX_LINE_CHARS = 300
SESSION_LOG_KEEP_FILES = 20


class SessionLog:
    def __init__(
        self,
        max_lines: int = SESSION_LOG_MAX_LINES,
        max_line_chars: int = SESSION_LOG_MAX_LINE_CHARS,
    ):
        self.max_line_chars = max(40, int(max_line_chars))
        self._lines: Deque[str] = deque(maxlen=max(100, int(max_lines)))
        self._lock = Lock()
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
        self._prune_old_logs()
        return path

    @staticmethod
    def _prune_old_logs() -> None:
        try:
            entries = [
                os.path.join(SESSION_LOG_DIR, name)
                for name in os.listdir(SESSION_LOG_DIR)
                if name.startswith("session_") and name.endswith(".log")
            ]
        except OSError:
            return
        entries.sort(key=lambda p: os.path.getmtime(p), reverse=True)
        for path in entries[SESSION_LOG_KEEP_FILES:]:
            try:
                os.remove(path)
            except OSError:
                pass


_SESSION_LOG = SessionLog()


def log_session(source: str, message: str) -> None:
    _SESSION_LOG.add(source, message)


def close_session_log() -> Optional[str]:
    return _SESSION_LOG.close_and_save()
