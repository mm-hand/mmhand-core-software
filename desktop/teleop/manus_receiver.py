import json
import socket
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


MANUS_CONTROL_ENABLED = False
MANUS_TIMEOUT_S = 2.0


@dataclass
class ManusFrame:
    seq: int
    timestamp: float
    recv_time: float
    recv_perf_time: float
    dt: float
    glove_id: int
    side: str
    ergonomics_names: List[str] = field(default_factory=list)
    ergonomics_values: List[float] = field(default_factory=list)
    raw_skeleton_nodes: List[Dict[str, Any]] = field(default_factory=list)
    skeleton: Dict[str, Any] = field(default_factory=dict)
    source_addr: str = ""
    valid: bool = True
    pending_packets_count: int = 0
    dropped_old_packets_count: int = 0
    latest_frame_id: int = 0
    processed_frame_id: int = 0
    timing_recv_call_ms: float = 0.0


class ManusReceiver:
    def __init__(self, ip: str = "127.0.0.1", port: int = 9001, timeout_s: float = MANUS_TIMEOUT_S):
        self.ip = str(ip)
        self.port = int(port)
        self.timeout_s = float(timeout_s)
        self._server_sock: Optional[socket.socket] = None
        self._client_sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._latest_frame: Optional[ManusFrame] = None
        self._last_seq: Optional[int] = None
        self._last_drop_seq: Optional[int] = None
        self._last_recv_time = 0.0
        self._last_recv_dt = 0.0
        self._last_consumed_seq: Optional[int] = None
        self._packet_count = 0
        self._raw_packet_count = 0
        self._dropped_frames = 0
        self._recv_times = deque(maxlen=200)
        self._restart_count = 0
        self._connection_count = 0
        self._eof_count = 0
        self._source_addr = ""
        self._client_connected = False
        self._last_event = ""
        self._error = ""

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._open_server()
        self._thread = threading.Thread(target=self._run, name="ManusTcpReceiver", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        for sock in (self._client_sock, self._server_sock):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
        self._client_sock = None
        self._server_sock = None
        thread = self._thread
        if thread and thread.is_alive():
            thread.join(timeout=0.5)
        self._thread = None

    def restart(self) -> None:
        self.stop()
        with self._lock:
            self._restart_count += 1
        self.start()

    def get_latest_frame(self) -> Optional[ManusFrame]:
        with self._lock:
            frame = self._latest_frame
            if frame is None:
                return None
            if frame.seq > 0:
                pending = 1 if self._last_consumed_seq != frame.seq else 0
                dropped_old = 0
                if self._last_consumed_seq is not None and frame.seq > self._last_consumed_seq + 1:
                    dropped_old = frame.seq - self._last_consumed_seq - 1
                self._last_consumed_seq = frame.seq
                frame.pending_packets_count = pending
                frame.dropped_old_packets_count = dropped_old
                frame.latest_frame_id = frame.seq
                frame.processed_frame_id = frame.seq
            return frame

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

    def restart_count(self) -> int:
        with self._lock:
            return self._restart_count

    def connection_count(self) -> int:
        with self._lock:
            return self._connection_count

    def eof_count(self) -> int:
        with self._lock:
            return self._eof_count

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

    def _open_server(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.ip, self.port))
        sock.listen(1)
        sock.settimeout(0.2)
        self._server_sock = sock

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                self._accept_and_read()
            except OSError as exc:
                if not self._stop_event.is_set():
                    self._set_error(str(exc))
                    time.sleep(0.2)

    def _accept_and_read(self) -> None:
        server_sock = self._server_sock
        if server_sock is None:
            return
        try:
            client, addr = server_sock.accept()
        except socket.timeout:
            return
        client.settimeout(0.2)
        self._client_sock = client
        source_addr = f"{addr[0]}:{addr[1]}"
        with self._lock:
            self._connection_count += 1
            self._client_connected = True
            self._source_addr = source_addr
            self._last_event = f"TCP connected from {source_addr}"
            self._error = ""

        buffer = ""
        try:
            while not self._stop_event.is_set():
                try:
                    chunk = client.recv(65536)
                except socket.timeout:
                    continue
                if not chunk:
                    with self._lock:
                        self._eof_count += 1
                        self._client_connected = False
                        self._last_event = f"TCP EOF from {source_addr}"
                    return
                buffer += chunk.decode("utf-8", errors="replace")
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if line:
                        self._handle_json_line(line, source_addr)
        finally:
            try:
                client.close()
            except OSError:
                pass
            if self._client_sock is client:
                self._client_sock = None
            with self._lock:
                self._client_connected = False

    def _handle_json_line(self, line: str, source_addr: str) -> None:
        with self._lock:
            self._raw_packet_count += 1
        try:
            packet = json.loads(line)
            frame = self._parse_frame(packet, source_addr)
        except Exception as exc:
            self._set_error(f"Invalid MANUS TCP frame: {exc}")
            return

        with self._lock:
            if frame.side == "heartbeat":
                if self._last_recv_time > 0.0:
                    self._last_recv_dt = frame.recv_time - self._last_recv_time
                self._last_recv_time = frame.recv_time
                self._packet_count += 1
                self._recv_times.append(frame.recv_time)
                self._error = ""
                return

            if frame.seq > 0:
                if self._last_drop_seq is not None and frame.seq != self._last_drop_seq + 1:
                    gap = frame.seq - self._last_drop_seq - 1
                    if gap > 5:
                        self._dropped_frames += gap
                self._last_drop_seq = frame.seq
                self._last_seq = frame.seq
            if self._last_recv_time > 0.0:
                self._last_recv_dt = frame.recv_time - self._last_recv_time
            self._last_recv_time = frame.recv_time
            self._latest_frame = frame
            self._packet_count += 1
            self._recv_times.append(frame.recv_time)
            self._error = ""

    def _parse_frame(self, packet: dict, source_addr: str) -> ManusFrame:
        if packet.get("type") != "manus_frame":
            raise ValueError(f"unexpected type {packet.get('type')!r}")
        names = packet.get("ergonomics_names") or []
        values = packet.get("ergonomics_values") or []
        if not isinstance(names, list) or not isinstance(values, list):
            raise ValueError("ergonomics_names and ergonomics_values must be lists")
        raw_skeleton_nodes = packet.get("raw_skeleton_nodes") or []
        skeleton = packet.get("skeleton") or {}
        if not isinstance(raw_skeleton_nodes, list):
            raise ValueError("raw_skeleton_nodes must be a list")
        if not isinstance(skeleton, dict):
            raise ValueError("skeleton must be an object")
        return ManusFrame(
            seq=int(packet.get("seq", 0)),
            timestamp=float(packet.get("timestamp", 0.0)),
            recv_time=time.time(),
            recv_perf_time=time.perf_counter(),
            dt=float(packet.get("dt", 0.0)),
            glove_id=int(packet.get("glove_id", 0)),
            side=str(packet.get("side", "unknown")),
            ergonomics_names=[str(name) for name in names],
            ergonomics_values=[float(value) for value in values],
            raw_skeleton_nodes=[node for node in raw_skeleton_nodes if isinstance(node, dict)],
            skeleton=skeleton,
            source_addr=source_addr,
            valid=True,
        )

    def _set_error(self, message: str) -> None:
        with self._lock:
            self._error = str(message)
            self._last_event = str(message)


__all__ = ["MANUS_CONTROL_ENABLED", "MANUS_TIMEOUT_S", "ManusFrame", "ManusReceiver"]
