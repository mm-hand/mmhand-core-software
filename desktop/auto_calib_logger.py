import os
from datetime import datetime
from threading import RLock
from typing import Optional


DESKTOP_DIR = os.path.dirname(os.path.abspath(__file__))
AUTO_CALIB_LOG_DIR = os.path.join(DESKTOP_DIR, "logs", "auto_calib")
AUTO_CALIB_LOG_KEEP_FILES = 20


class AutoCalibLogger:
    def __init__(self, port: str = ""):
        self._lock = RLock()
        self._file = None
        self._path = ""
        self._port = str(port or "").strip()

    @property
    def path(self) -> str:
        return self._path

    def start(self, port: str = "") -> str:
        with self._lock:
            self._close_no_lock()
            self._port = str(port or self._port or "").strip()
            os.makedirs(AUTO_CALIB_LOG_DIR, exist_ok=True)
            self._prune_old_logs()
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            suffix = self._safe_port_suffix(self._port)
            name = f"auto_calib_{stamp}{suffix}.log"
            self._path = os.path.join(AUTO_CALIB_LOG_DIR, name)
            self._file = open(self._path, "w", encoding="utf-8", newline="\n")
            self._write_no_lock("LOGGER", f"start port={self._port or '-'}")
            return self._path

    def log(self, source: str, message: str) -> None:
        text = str(message or "").strip()
        if not text:
            return
        with self._lock:
            if self._file is None:
                self.start(self._port)
            self._write_no_lock(source, text)

    def close(self) -> Optional[str]:
        with self._lock:
            path = self._path
            self._close_no_lock()
            self._prune_old_logs()
            return path or None

    def _write_no_lock(self, source: str, message: str) -> None:
        if self._file is None:
            return
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        for line in str(message).replace("\r", "").splitlines():
            line = line.strip()
            if line:
                self._file.write(f"{ts} [{str(source).strip() or 'LOG'}] {line}\n")
        self._file.flush()

    def _close_no_lock(self) -> None:
        if self._file is not None:
            try:
                self._write_no_lock("LOGGER", "close")
                self._file.flush()
            except Exception:
                pass
            try:
                self._file.close()
            except Exception:
                pass
        self._file = None
        self._path = ""

    @staticmethod
    def _safe_port_suffix(port: str) -> str:
        text = "".join(ch if ch.isalnum() else "_" for ch in str(port or "").strip())
        return f"_{text}" if text else ""

    @staticmethod
    def _prune_old_logs() -> None:
        if not os.path.isdir(AUTO_CALIB_LOG_DIR):
            return
        paths = [
            os.path.join(AUTO_CALIB_LOG_DIR, name)
            for name in os.listdir(AUTO_CALIB_LOG_DIR)
            if name.startswith("auto_calib_") and name.lower().endswith(".log")
        ]
        paths.sort(key=lambda path: os.path.getmtime(path), reverse=True)
        for path in paths[AUTO_CALIB_LOG_KEEP_FILES:]:
            try:
                os.remove(path)
            except OSError:
                pass
