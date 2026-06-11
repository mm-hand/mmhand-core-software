import ctypes
import json
import threading
import time
from collections import deque
from pathlib import Path
from typing import Optional

from .manus_receiver import MANUS_TIMEOUT_S, ManusFrame


class ManusSdkReceiver:
    def __init__(self, dll_path: Optional[str] = None, timeout_s: float = MANUS_TIMEOUT_S):
        self.dll_path = str(dll_path) if dll_path else self._default_dll_path()
        self.timeout_s = float(timeout_s)
        self._dll = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._latest_frame: Optional[ManusFrame] = None
        self._last_recv_time = 0.0
        self._last_recv_dt = 0.0
        self._packet_count = 0
        self._raw_packet_count = 0
        self._dropped_frames = 0
        self._last_seq: Optional[int] = None
        self._recv_times = deque(maxlen=200)
        self._error = ""
        self._last_event = ""
        self._client_connected = False

    @staticmethod
    def _default_dll_path() -> str:
        root = Path(__file__).resolve().parents[2]
        candidates = [
            root / "manus" / "mmhand_manus_sdk" / "build" / "windows" / "Release" / "mmhand_manus_sdk.dll",
            root / "manus" / "mmhand_manus_sdk" / "build" / "windows" / "Debug" / "mmhand_manus_sdk.dll",
            root / "manus" / "mmhand_manus_sdk" / "bin" / "mmhand_manus_sdk.dll",
            root / "manus" / "mmhand_manus_sdk" / "mmhand_manus_sdk.dll",
        ]
        for path in candidates:
            if path.exists():
                return str(path)
        return str(candidates[0])

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._load_dll()
        self._stop_event.clear()
        if not self._dll.mmhand_manus_start(1, 1, 5):
            self._set_error(self._dll_error() or "MANUS SDK start failed")
            return
        with self._lock:
            self._client_connected = True
            self._last_event = "Direct SDK started"
            self._error = ""
        self._thread = threading.Thread(target=self._run, name="ManusSdkReceiver", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        dll = self._dll
        if dll is not None:
            try:
                dll.mmhand_manus_stop()
            except Exception:
                pass
        thread = self._thread
        if thread and thread.is_alive():
            thread.join(timeout=0.5)
        self._thread = None
        with self._lock:
            self._client_connected = False
            self._last_event = "Direct SDK stopped"

    def restart(self) -> None:
        self.stop()
        self.start()

    def get_latest_frame(self) -> Optional[ManusFrame]:
        with self._lock:
            return self._latest_frame

    def is_alive(self) -> bool:
        with self._lock:
            return self._last_recv_time > 0.0 and (time.time() - self._last_recv_time) <= self.timeout_s

    def recv_dt(self) -> float:
        with self._lock:
            return self._last_recv_dt

    def recv_hz(self) -> float:
        with self._lock:
            if len(self._recv_times) < 2:
                return 0.0
            elapsed = self._recv_times[-1] - self._recv_times[0]
            return (len(self._recv_times) - 1) / elapsed if elapsed > 0.0 else 0.0

    def dropped_frames(self) -> int:
        with self._lock:
            return self._dropped_frames

    def packet_count(self) -> int:
        with self._lock:
            return self._packet_count

    def raw_packet_count(self) -> int:
        with self._lock:
            return self._raw_packet_count

    def thread_alive(self) -> bool:
        thread = self._thread
        return bool(thread and thread.is_alive())

    def connection_count(self) -> int:
        return 1 if self.client_connected() else 0

    def eof_count(self) -> int:
        return 0

    def client_connected(self) -> bool:
        with self._lock:
            return self._client_connected

    def last_event(self) -> str:
        with self._lock:
            return self._last_event

    def last_recv_age(self) -> float:
        with self._lock:
            if self._last_recv_time <= 0.0:
                return -1.0
            return time.time() - self._last_recv_time

    def last_error(self) -> str:
        with self._lock:
            return self._error

    def _load_dll(self) -> None:
        path = Path(self.dll_path)
        if not path.exists():
            raise RuntimeError(f"MANUS SDK wrapper DLL not found: {path}")
        self._dll = ctypes.CDLL(str(path))
        self._dll.mmhand_manus_start.argtypes = [ctypes.c_int, ctypes.c_uint, ctypes.c_uint]
        self._dll.mmhand_manus_start.restype = ctypes.c_int
        self._dll.mmhand_manus_stop.argtypes = []
        self._dll.mmhand_manus_stop.restype = None
        self._dll.mmhand_manus_is_connected.argtypes = []
        self._dll.mmhand_manus_is_connected.restype = ctypes.c_int
        self._dll.mmhand_manus_get_latest_frame_json.argtypes = [ctypes.c_char_p, ctypes.c_int]
        self._dll.mmhand_manus_get_latest_frame_json.restype = ctypes.c_int
        self._dll.mmhand_manus_last_error.argtypes = []
        self._dll.mmhand_manus_last_error.restype = ctypes.c_char_p

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                self._poll_once()
            except Exception as exc:
                self._set_error(f"MANUS Direct SDK poll failed: {exc}")
                time.sleep(0.1)
            time.sleep(0.005)

    def _poll_once(self) -> None:
        if self._dll is None:
            return
        buf_size = 8192
        buf = ctypes.create_string_buffer(buf_size)
        result = int(self._dll.mmhand_manus_get_latest_frame_json(buf, buf_size))
        if result == 0:
            return
        if result < 0:
            required = -result
            buf = ctypes.create_string_buffer(required)
            result = int(self._dll.mmhand_manus_get_latest_frame_json(buf, required))
            if result <= 0:
                return
        text = buf.value.decode("utf-8", errors="replace")
        packet = json.loads(text)
        frame = self._parse_frame(packet)
        self._handle_frame(frame)

    def _parse_frame(self, packet: dict) -> ManusFrame:
        if packet.get("type") != "manus_frame":
            raise ValueError(f"unexpected type {packet.get('type')!r}")
        names = packet.get("ergonomics_names") or []
        values = packet.get("ergonomics_values") or []
        if not isinstance(names, list) or not isinstance(values, list):
            raise ValueError("ergonomics_names and ergonomics_values must be lists")
        return ManusFrame(
            seq=int(packet.get("seq", 0)),
            timestamp=float(packet.get("timestamp", 0.0)),
            recv_time=time.time(),
            dt=float(packet.get("dt", 0.0)),
            glove_id=int(packet.get("glove_id", 0)),
            side=str(packet.get("side", "unknown")),
            ergonomics_names=[str(name) for name in names],
            ergonomics_values=[float(value) for value in values],
            source_addr="direct-sdk",
            valid=True,
        )

    def _handle_frame(self, frame: ManusFrame) -> None:
        with self._lock:
            self._raw_packet_count += 1
            if frame.seq > 0 and self._last_seq is not None and frame.seq != self._last_seq + 1:
                gap = frame.seq - self._last_seq - 1
                if gap > 5:
                    self._dropped_frames += gap
            self._last_seq = frame.seq
            if self._last_recv_time > 0.0:
                self._last_recv_dt = frame.recv_time - self._last_recv_time
            self._last_recv_time = frame.recv_time
            self._latest_frame = frame
            self._packet_count += 1
            self._recv_times.append(frame.recv_time)
            self._client_connected = bool(self._dll.mmhand_manus_is_connected()) if self._dll else False
            self._error = ""

    def _dll_error(self) -> str:
        if self._dll is None:
            return ""
        raw = self._dll.mmhand_manus_last_error()
        return raw.decode("utf-8", errors="replace") if raw else ""

    def _set_error(self, message: str) -> None:
        with self._lock:
            self._error = str(message)
            self._last_event = str(message)


__all__ = ["ManusSdkReceiver"]
