"""auto_calib_logger.py - MMHand Desktop 模块契约。

职责：解析并持久化自动标定过程的结构化诊断日志。
线程：除 comm_layer 的串口收发线程外不主动创建 GUI 线程；共享状态必须按既有锁读取。
输入：P4 字节流、上层类型化命令、配置和持久化数据。
输出：线程安全的 HandModel 快照、协议帧、ACK 和后端操作结果。
状态所有权：通信对象拥有串口与解析缓存；HandModel 的写入只允许在核心同步边界内发生。
禁止依赖：禁止依赖 Qt/UI、app、ROS2 或 Replay 页面，禁止把控件和展示规则带入核心层。
"""
import os
from datetime import datetime
from threading import RLock
from typing import Optional

from desktop.paths import LOG_DIR

AUTO_CALIB_LOG_DIR = os.path.join(str(LOG_DIR), "auto_calib")


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
