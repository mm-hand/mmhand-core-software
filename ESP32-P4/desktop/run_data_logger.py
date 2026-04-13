import csv
import os
import threading
from typing import Dict, Iterable, Optional


class RunDataLogger:
    """Simple CSV logger for runtime snapshots."""

    def __init__(self, flush_every: int = 10):
        self._flush_every = max(1, int(flush_every))
        self._file = None
        self._writer: Optional[csv.DictWriter] = None
        self._fieldnames = None
        self._pending_rows = 0
        self._path = ""
        self._lock = threading.Lock()

    @property
    def path(self) -> str:
        return self._path

    def start(self, path: str, fieldnames: Optional[Iterable[str]] = None) -> None:
        with self._lock:
            self._close_no_lock()
            folder = os.path.dirname(path)
            if folder:
                os.makedirs(folder, exist_ok=True)
            self._file = open(path, "w", newline="", encoding="utf-8")
            self._writer = None
            self._pending_rows = 0
            self._path = path
            if fieldnames is not None:
                self._fieldnames = list(fieldnames)
                self._writer = csv.DictWriter(self._file, fieldnames=self._fieldnames, extrasaction="ignore")
                self._writer.writeheader()
                self._file.flush()
            else:
                self._fieldnames = None

    def stop(self) -> None:
        with self._lock:
            self._close_no_lock()

    def _close_no_lock(self) -> None:
        """Close file/resources. Caller must hold _lock."""
        if self._file is not None:
            try:
                self._file.flush()
            except Exception:
                pass
            try:
                self._file.close()
            except Exception:
                pass
        self._file = None
        self._writer = None
        self._fieldnames = None
        self._pending_rows = 0
        self._path = ""

    def is_running(self) -> bool:
        with self._lock:
            return self._file is not None

    def write_row(self, row: Dict[str, object]) -> None:
        with self._lock:
            if self._file is None:
                return

            if self._writer is None:
                self._fieldnames = list(row.keys())
                self._writer = csv.DictWriter(self._file, fieldnames=self._fieldnames, extrasaction="ignore")
                self._writer.writeheader()

            self._writer.writerow(row)
            self._pending_rows += 1
            if self._pending_rows >= self._flush_every:
                self._file.flush()
                self._pending_rows = 0
