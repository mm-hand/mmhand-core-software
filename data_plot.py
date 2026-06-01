#!/usr/bin/env python3
"""
S48 CAN-FD heatmap replay tool.

Usage:
    python data_plot.py
    python data_plot.py datas/1.csv
    python data_plot.py --summary datas/1.csv
    python data_plot.py --live COM3
    python data_plot.py --live COM3 --baud 921600
    python data_plot.py --live COM3 --record datas/live_test.csv

CSV replay uses only Python's standard library. Live serial mode needs
pyserial. The tool decodes S48 CAN-FD packets and replays the 8 x 6 tactile
array as a white background heatmap. Circle color is based on Z by default.
"""

from __future__ import annotations

import argparse
import csv
import datetime
import math
import os
import re
import sys
import time
import queue
import threading
import tkinter as tk
from dataclasses import dataclass
from tkinter import filedialog, messagebox, ttk
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


ROWS = 8
COLS = 6
POINTS = ROWS * COLS
PACKETS_PER_FRAME = 3
POINTS_PER_PACKET = 16
SPEED_PRESETS = ("0.1x", "0.2x", "0.5x", "1.0x", "1.5x", "2.0x")
LIVE_BAUDRATE = 921600
LIVE_HISTORY_LIMIT = 1000
LIVE_REDRAW_INTERVAL_MS = 50
LIVE_QUEUE_BATCH_LIMIT = 500

# Display orientation correction from the four-corner press test:
# keep the raw 8 x 6 shape and mirror columns left/right.
DISPLAY_ROWS = ROWS
DISPLAY_COLS = COLS

FUNC_DATA = 0x00
FUNC_FORCE = 0x04
PRODUCT_S48 = 0x01

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_RECORD_DIR = os.path.join(SCRIPT_DIR, "datas")
DEFAULT_CSV = os.path.join("datas", "1.csv")


@dataclass(frozen=True)
class ForcePoint:
    x: int
    y: int
    z: int

    @property
    def magnitude(self) -> float:
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)


@dataclass(frozen=True)
class ReplayFrame:
    source_row: int
    system_time: str
    time_tick: Optional[int]
    device_id: int
    packet_seq: int
    seen_mask: int
    points: Tuple[ForcePoint, ...]

    @property
    def complete(self) -> bool:
        return self.seen_mask == 0b111


@dataclass
class CsvStats:
    rows: int = 0
    decoded_packets: int = 0
    resultants: int = 0
    frames: int = 0
    complete_frames: int = 0
    devices: Tuple[int, ...] = ()
    encoding: str = ""
    reset_count: int = 0


@dataclass
class LiveStats:
    packets: int = 0
    frames: int = 0
    resultants: int = 0
    devices: Tuple[int, ...] = ()
    last_line: str = ""
    port: str = ""
    baudrate: int = LIVE_BAUDRATE


@dataclass(frozen=True)
class LiveResultantSample:
    system_time: str
    time_tick: int
    device_id: int
    fx: float
    fy: float
    fz: float
    raw_line: str = ""


@dataclass(frozen=True)
class LiveSensorPacket:
    system_time: str
    time_tick: int
    can_id: int
    device_id: int
    packet_seq: int
    payload: bytes


def open_csv_with_fallback(path: str):
    last_error = None
    for encoding in ("utf-8-sig", "gbk", "utf-8", "cp936", "latin1"):
        try:
            f = open(path, "r", encoding=encoding, errors="strict", newline="")
            sample = f.read(4096)
            f.seek(0)
            if "\ufffd" in sample:
                f.close()
                continue
            return f, encoding
        except UnicodeDecodeError as exc:
            last_error = exc
        except OSError:
            raise
    if last_error:
        raise last_error
    return open(path, "r", encoding="latin1", newline=""), "latin1"


def clean_system_time(value: str) -> str:
    return value.strip().replace("=", "").replace('"', "")


def parse_time_tick(value: str) -> Optional[int]:
    value = value.strip()
    if not value:
        return None
    try:
        return int(value, 16) if value.lower().startswith("0x") else int(value)
    except ValueError:
        return None


def parse_payload(value: str) -> bytes:
    if "|" in value:
        value = value.split("|", 1)[1]
    pairs = re.findall(r"(?<![0-9A-Fa-f])([0-9A-Fa-f]{2})(?![0-9A-Fa-f])", value)
    return bytes(int(item, 16) for item in pairs)


def parse_can_id(can_id: int) -> Tuple[int, int, int, int]:
    func_code = (can_id >> 26) & 0x07
    product_type = (can_id >> 20) & 0x3F
    device_id = (can_id >> 5) & 0x7FFF
    packet_seq = can_id & 0x07
    return func_code, product_type, device_id, packet_seq


def make_sensor_can_id(device_id: int, packet_seq: int) -> int:
    return (
        (FUNC_DATA << 26)
        | (PRODUCT_S48 << 20)
        | ((device_id & 0x7FFF) << 5)
        | (packet_seq & 0x07)
    )


def make_resultant_can_id(device_id: int) -> int:
    return (
        (FUNC_FORCE << 26)
        | (PRODUCT_S48 << 20)
        | ((device_id & 0x7FFF) << 5)
    )


def decode_force_word(raw: int) -> ForcePoint:
    x_mag = raw & 0x3FF
    y_mag = (raw >> 10) & 0x3FF
    z_val = (raw >> 20) & 0x3FF
    sign = (raw >> 30) & 0x03

    x_val = -x_mag if (sign >> 1) & 1 else x_mag
    y_val = -y_mag if sign & 1 else y_mag
    return ForcePoint(x_val, y_val, z_val)


def encode_force_word(point: ForcePoint) -> int:
    x_mag = max(0, min(0x3FF, abs(int(point.x))))
    y_mag = max(0, min(0x3FF, abs(int(point.y))))
    z_val = max(0, min(0x3FF, int(point.z)))
    sign = (0x02 if point.x < 0 else 0x00) | (0x01 if point.y < 0 else 0x00)
    return x_mag | (y_mag << 10) | (z_val << 20) | (sign << 30)


def decode_packet(payload: bytes) -> Tuple[ForcePoint, ...]:
    points: List[ForcePoint] = []
    for index in range(POINTS_PER_PACKET):
        offset = index * 4
        raw = int.from_bytes(payload[offset : offset + 4], "little")
        points.append(decode_force_word(raw))
    return tuple(points)


def encode_packet(points: Sequence[ForcePoint]) -> bytes:
    payload = bytearray()
    for point in points:
        payload.extend(encode_force_word(point).to_bytes(4, "little"))
    return bytes(payload)


def make_default_record_path(port: str) -> str:
    safe_port = re.sub(r"[^0-9A-Za-z_.-]+", "_", port.strip() or "serial")
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(DEFAULT_RECORD_DIR, f"live_{stamp}_{safe_port}.csv")


class LiveLineParser:
    RX_HEADER_RE = re.compile(r"^\s*RX\s+id=0x([0-9A-Fa-f]+)\s+len=(\d+)\s*$")
    HEX_BYTE_RE = re.compile(r"(?<![0-9A-Fa-f])([0-9A-Fa-f]{2})(?![0-9A-Fa-f])")

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.pending_id: Optional[int] = None
        self.expected_len = 0
        self.collecting = False
        self.buffer = bytearray()

    def feed(self, line: str) -> List[Tuple[int, bytes]]:
        packets: List[Tuple[int, bytes]] = []
        text = line.strip()
        if not text:
            return packets

        header = self.RX_HEADER_RE.match(text)
        if header:
            self.pending_id = int(header.group(1), 16)
            self.expected_len = int(header.group(2))
            self.collecting = False
            self.buffer = bytearray()
            return packets

        if self.pending_id is None:
            return packets

        if "data:" in text:
            self.collecting = True

        if self.collecting:
            for token in self.HEX_BYTE_RE.findall(text):
                self.buffer.append(int(token, 16))
                if len(self.buffer) >= self.expected_len:
                    packets.append((self.pending_id, bytes(self.buffer[: self.expected_len])))
                    self.reset()
                    break
        return packets


COMPACT_FRAME_RE = re.compile(
    r"^\s*FRAME\s+#(?P<frame>\d+)\s+device=0x(?P<device>[0-9A-Fa-f]+)\s+product=0x(?P<product>[0-9A-Fa-f]+)\s+data:(?P<data>.*)$"
)
COMPACT_WORD_RE = re.compile(r"(?<![0-9A-Fa-f])([0-9A-Fa-f]{8})(?![0-9A-Fa-f])")
RESULTANT_LINE_RE = re.compile(
    r"^\s*RESULTANT\s+device=0x(?P<device>[0-9A-Fa-f]+)\s+"
    r"(?P<fx>[-+]?\d+(?:\.\d+)?)\s*,\s*"
    r"(?P<fy>[-+]?\d+(?:\.\d+)?)\s*,\s*"
    r"(?P<fz>[-+]?\d+(?:\.\d+)?)\s+N\s*$"
)
PACKET_LINE_RE = re.compile(
    r"^\s*PKT\s+id=0x(?P<id>[0-9A-Fa-f]+)\s+len=(?P<len>\d+)\s+data:(?P<data>.*)$"
)


def parse_compact_frame_line(line: str) -> Optional[ReplayFrame]:
    text = line.strip()
    match = COMPACT_FRAME_RE.match(text)
    if not match:
        return None

    data_text = match.group("data")
    words = COMPACT_WORD_RE.findall(data_text)
    if len(words) < POINTS:
        return None

    points = tuple(decode_force_word(int(word, 16)) for word in words[:POINTS])
    frame_no = int(match.group("frame"))
    device_id = int(match.group("device"), 16)
    product_type = int(match.group("product"), 16)
    if product_type != PRODUCT_S48:
        return None
    # Compact live frames already represent a full 3-packet sensor frame.
    return ReplayFrame(
        source_row=frame_no,
        system_time=datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3],
        time_tick=time.perf_counter_ns() // 1000,
        device_id=device_id,
        packet_seq=2,
        seen_mask=0b111,
        points=points,
    )


def parse_resultant_line(line: str) -> Optional[LiveResultantSample]:
    text = line.strip()
    match = RESULTANT_LINE_RE.match(text)
    if not match:
        return None

    return LiveResultantSample(
        system_time=datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3],
        time_tick=time.perf_counter_ns() // 1000,
        device_id=int(match.group("device"), 16),
        fx=float(match.group("fx")),
        fy=float(match.group("fy")),
        fz=float(match.group("fz")),
        raw_line=text,
    )


def make_live_sensor_packet(can_id: int, payload: bytes) -> Optional[LiveSensorPacket]:
    func_code, product_type, device_id, packet_seq = parse_can_id(can_id)
    if (
        func_code != FUNC_DATA
        or product_type != PRODUCT_S48
        or packet_seq not in (0, 1, 2)
        or len(payload) < 64
    ):
        return None

    return LiveSensorPacket(
        system_time=datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3],
        time_tick=time.perf_counter_ns() // 1000,
        can_id=can_id,
        device_id=device_id,
        packet_seq=packet_seq,
        payload=payload[:64],
    )


def parse_packet_line(line: str) -> Optional[LiveSensorPacket]:
    text = line.strip()
    match = PACKET_LINE_RE.match(text)
    if not match:
        return None

    can_id = int(match.group("id"), 16)
    expected_len = int(match.group("len"))
    payload = parse_payload(match.group("data"))
    if len(payload) < expected_len:
        return None
    return make_live_sensor_packet(can_id, payload[:expected_len])


class LiveFrameAssembler:
    def __init__(self) -> None:
        self._states: Dict[int, Dict[str, object]] = {}
        self._sequence = 0

    def feed_packet(
        self,
        can_id: int,
        payload: bytes,
        system_time: Optional[str] = None,
        time_tick: Optional[int] = None,
    ) -> Optional[ReplayFrame]:
        func_code, product_type, device_id, packet_seq = parse_can_id(can_id)
        if (
            func_code != FUNC_DATA
            or product_type != PRODUCT_S48
            or packet_seq not in (0, 1, 2)
            or len(payload) < 64
        ):
            return None

        state = self._states.get(device_id)
        if state is None:
            state = {
                "points": [ForcePoint(0, 0, 0) for _ in range(POINTS)],
                "mask": 0,
            }
            self._states[device_id] = state

        points = decode_packet(payload[:64])
        start = packet_seq * POINTS_PER_PACKET
        state_points = state["points"]
        assert isinstance(state_points, list)
        state_points[start : start + POINTS_PER_PACKET] = list(points)
        state["mask"] = int(state["mask"]) | (1 << packet_seq)

        self._sequence += 1
        return ReplayFrame(
            source_row=self._sequence,
            system_time=system_time or datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3],
            time_tick=time_tick if time_tick is not None else time.perf_counter_ns() // 1000,
            device_id=device_id,
            packet_seq=packet_seq,
            seen_mask=int(state["mask"]),
            points=tuple(state_points),
        )

    def feed_live_packet(self, packet: LiveSensorPacket) -> Optional[ReplayFrame]:
        return self.feed_packet(packet.can_id, packet.payload, packet.system_time, packet.time_tick)


class LiveSerialFeed:
    def __init__(self, port: str, baudrate: int, out_queue: "queue.Queue[Tuple[str, object]]"):
        self.port = port
        self.baudrate = baudrate
        self.out_queue = out_queue
        self.stop_event = threading.Event()
        self.serial_port = None
        self.thread: Optional[threading.Thread] = None
        self.write_lock = threading.Lock()
        self.parser = LiveLineParser()
        self.assembler = LiveFrameAssembler()
        self.stats = LiveStats(port=port, baudrate=baudrate)

    def start(self) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed. Install pyserial first.")
        self.serial_port = serial.Serial(self.port, self.baudrate, timeout=0.1)
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def close(self) -> None:
        self.stop_event.set()
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except Exception:
                pass

    def send_command(self, command: str) -> None:
        if self.serial_port is None:
            raise RuntimeError("live serial is not open")
        data = command.encode("ascii", errors="ignore")
        if not data:
            return
        with self.write_lock:
            self.serial_port.write(data)
            self.serial_port.flush()

    def _run(self) -> None:
        assert self.serial_port is not None
        try:
            self.out_queue.put(("live_status", f"Live connected: {self.port} @ {self.baudrate}"))
            while not self.stop_event.is_set():
                raw = self.serial_port.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").rstrip("\r\n")
                self.stats.last_line = line
                packet = parse_packet_line(line)
                if packet is not None:
                    self.stats.packets += 1
                    devices = set(self.stats.devices)
                    devices.add(packet.device_id)
                    self.stats.devices = tuple(sorted(devices))
                    frame = self.assembler.feed_live_packet(packet)
                    self.stats.frames += 1
                    self.out_queue.put(("live_packet", (packet, frame)))
                    continue
                compact = parse_compact_frame_line(line)
                if compact is not None:
                    self.stats.packets += PACKETS_PER_FRAME
                    self.stats.frames += 1
                    devices = set(self.stats.devices)
                    devices.add(compact.device_id)
                    self.stats.devices = tuple(sorted(devices))
                    self.out_queue.put(("live_frame", compact))
                    continue
                resultant = parse_resultant_line(line)
                if resultant is not None:
                    self.stats.resultants += 1
                    devices = set(self.stats.devices)
                    devices.add(resultant.device_id)
                    self.stats.devices = tuple(sorted(devices))
                    self.out_queue.put(("live_resultant", resultant))
                    continue
                for can_id, payload in self.parser.feed(line):
                    packet = make_live_sensor_packet(can_id, payload)
                    if packet is None:
                        continue
                    self.stats.packets += 1
                    frame = self.assembler.feed_live_packet(packet)
                    if frame is not None:
                        self.stats.frames += 1
                        devices = set(self.stats.devices)
                        devices.add(frame.device_id)
                        self.stats.devices = tuple(sorted(devices))
                        self.out_queue.put(("live_packet", (packet, frame)))
        except Exception as exc:
            self.out_queue.put(("live_error", str(exc)))
        finally:
            try:
                if self.serial_port is not None:
                    self.serial_port.close()
            except Exception:
                pass
            self.out_queue.put(("live_closed", self.port))


class LiveCsvRecorder:
    HEADER = (
        "序号",
        "系统时间",
        "时间标识",
        "通道",
        "收/发",
        "ID",
        "Frame",
        "类型",
        "DLC",
        "CAN-FD",
        "数据",
    )

    def __init__(self, path: str):
        self.path = path
        self.file = None
        self.writer = None
        self.row_count = 0
        self.frame_count = 0
        self.resultant_count = 0

    def open(self) -> None:
        os.makedirs(os.path.dirname(os.path.abspath(self.path)), exist_ok=True)
        self.file = open(self.path, "w", encoding="utf-8-sig", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(self.HEADER)

    def close(self) -> None:
        if self.file is not None:
            self.file.flush()
            self.file.close()
            self.file = None
            self.writer = None

    def write_frame(self, frame: ReplayFrame) -> None:
        if self.writer is None:
            raise RuntimeError("recorder is not open")
        if len(frame.points) < POINTS:
            return

        self.frame_count += 1
        tick = frame.time_tick if frame.time_tick is not None else 0
        for packet_seq in range(PACKETS_PER_FRAME):
            start = packet_seq * POINTS_PER_PACKET
            packet_points = frame.points[start : start + POINTS_PER_PACKET]
            payload = encode_packet(packet_points)
            payload_text = "x| " + " ".join(f"{byte:02X}" for byte in payload) + " "
            self.row_count += 1
            self.writer.writerow(
                (
                    self.row_count,
                    f'="{frame.system_time}"',
                    f"0x{tick:X}",
                    1,
                    "Rx",
                    f"0x{make_sensor_can_id(frame.device_id, packet_seq):08X}",
                    "扩展帧",
                    "数据帧",
                    64,
                    "CANFD-加速",
                    payload_text,
                )
            )
        if self.frame_count % 10 == 0 and self.file is not None:
            self.file.flush()

    def write_resultant(self, sample: LiveResultantSample) -> None:
        if self.writer is None:
            raise RuntimeError("recorder is not open")

        self.resultant_count += 1
        self.row_count += 1
        tick = sample.time_tick if sample.time_tick is not None else 0
        self.writer.writerow(
            (
                self.row_count,
                f'="{sample.system_time}"',
                f"0x{tick:X}",
                1,
                "Rx",
                f"0x{make_resultant_can_id(sample.device_id):08X}",
                "扩展帧",
                "合力",
                8,
                "CANFD",
                f"Fx={sample.fx:.1f}, Fy={sample.fy:.1f}, Fz={sample.fz:.1f} N",
            )
        )
        if self.row_count % 20 == 0 and self.file is not None:
            self.file.flush()


class LiveCsvRecorder:
    HEADER = (
        "序号",
        "系统时间",
        "时间标识",
        "通道",
        "收/发",
        "ID",
        "Frame",
        "类型",
        "DLC",
        "CAN-FD",
        "数据",
    )

    def __init__(self, path: str):
        self.path = path
        self.file = None
        self.writer = None
        self.row_count = 0
        self.frame_count = 0
        self.packet_count = 0
        self.resultant_count = 0

    def open(self) -> None:
        os.makedirs(os.path.dirname(os.path.abspath(self.path)), exist_ok=True)
        self.file = open(self.path, "w", encoding="utf-8-sig", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(self.HEADER)

    def close(self) -> None:
        if self.file is not None:
            self.file.flush()
            self.file.close()
            self.file = None
            self.writer = None

    def _write_row(
        self,
        system_time: str,
        time_tick: Optional[int],
        can_id: int,
        frame_text: str,
        frame_type: str,
        dlc: int,
        canfd_text: str,
        data_text: str,
    ) -> None:
        if self.writer is None:
            raise RuntimeError("recorder is not open")
        self.row_count += 1
        tick = time_tick if time_tick is not None else 0
        self.writer.writerow(
            (
                self.row_count,
                f'="{system_time}"',
                f"0x{tick:X}",
                1,
                "Rx",
                f"0x{can_id:08X}",
                frame_text,
                frame_type,
                dlc,
                canfd_text,
                data_text,
            )
        )
        if self.row_count % 20 == 0 and self.file is not None:
            self.file.flush()

    def write_packet(self, packet: LiveSensorPacket) -> None:
        self.packet_count += 1
        payload_text = "x| " + " ".join(f"{byte:02X}" for byte in packet.payload) + " "
        self._write_row(
            packet.system_time,
            packet.time_tick,
            packet.can_id,
            "扩展帧",
            "数据帧",
            len(packet.payload),
            "CANFD-加速",
            payload_text,
        )

    def write_frame(self, frame: ReplayFrame) -> None:
        if len(frame.points) < POINTS:
            return
        self.frame_count += 1
        for packet_seq in range(PACKETS_PER_FRAME):
            start = packet_seq * POINTS_PER_PACKET
            packet_points = frame.points[start : start + POINTS_PER_PACKET]
            packet = LiveSensorPacket(
                system_time=frame.system_time,
                time_tick=frame.time_tick if frame.time_tick is not None else 0,
                can_id=make_sensor_can_id(frame.device_id, packet_seq),
                device_id=frame.device_id,
                packet_seq=packet_seq,
                payload=encode_packet(packet_points),
            )
            self.write_packet(packet)

    def write_resultant(self, sample: LiveResultantSample) -> None:
        self.resultant_count += 1
        self._write_row(
            sample.system_time,
            sample.time_tick,
            make_resultant_can_id(sample.device_id),
            "扩展帧",
            "合力",
            8,
            "CANFD",
            f"Fx={sample.fx:.1f}, Fy={sample.fy:.1f}, Fz={sample.fz:.1f} N",
        )


def value_for_metric(point: ForcePoint, metric: str) -> float:
    if metric == "Magnitude":
        return point.magnitude
    if metric == "X":
        return abs(point.x)
    if metric == "Y":
        return abs(point.y)
    return float(point.z)


def load_replay_frames(path: str) -> Tuple[List[ReplayFrame], CsvStats]:
    frames: List[ReplayFrame] = []
    states: Dict[int, List[ForcePoint]] = {}
    seen_masks: Dict[int, int] = {}
    devices_seen = set()
    stats = CsvStats()

    f, encoding = open_csv_with_fallback(path)
    stats.encoding = encoding
    with f:
        reader = csv.reader(f)
        try:
            next(reader)
        except StopIteration:
            return [], stats

        previous_serial: Optional[int] = None
        for row_index, row in enumerate(reader, start=2):
            stats.rows += 1
            if len(row) < 11:
                continue

            try:
                serial_no = int(row[0])
            except ValueError:
                serial_no = None

            if (
                serial_no is not None
                and previous_serial is not None
                and serial_no <= previous_serial
            ):
                states.clear()
                seen_masks.clear()
                stats.reset_count += 1
            if serial_no is not None:
                previous_serial = serial_no

            try:
                can_id = int(row[5], 16)
                dlc = int(row[8])
            except ValueError:
                continue

            func_code, product_type, device_id, packet_seq = parse_can_id(can_id)
            if func_code == FUNC_FORCE and product_type == PRODUCT_S48:
                stats.resultants += 1
                devices_seen.add(device_id)
                continue

            if (
                func_code != FUNC_DATA
                or product_type != PRODUCT_S48
                or packet_seq not in (0, 1, 2)
                or dlc != 64
            ):
                continue

            payload = parse_payload(row[10])
            if len(payload) < 64:
                continue

            packet_points = decode_packet(payload[:64])
            if device_id not in states:
                states[device_id] = [ForcePoint(0, 0, 0) for _ in range(POINTS)]
                seen_masks[device_id] = 0

            start = packet_seq * POINTS_PER_PACKET
            states[device_id][start : start + POINTS_PER_PACKET] = list(packet_points)
            seen_masks[device_id] |= 1 << packet_seq
            devices_seen.add(device_id)
            stats.decoded_packets += 1

            frames.append(
                ReplayFrame(
                    source_row=row_index,
                    system_time=clean_system_time(row[1]),
                    time_tick=parse_time_tick(row[2]),
                    device_id=device_id,
                    packet_seq=packet_seq,
                    seen_mask=seen_masks[device_id],
                    points=tuple(states[device_id]),
                )
            )

    stats.frames = len(frames)
    stats.complete_frames = sum(1 for frame in frames if frame.complete)
    stats.devices = tuple(sorted(devices_seen))
    return frames, stats


def color_for_value(value: float, max_value: float) -> str:
    if max_value <= 0:
        ratio = 0.0
    else:
        ratio = max(0.0, min(1.0, value / max_value))

    # Light blue to deep blue, readable on a white background.
    start = (239, 246, 255)
    end = (0, 82, 204)
    red = round(start[0] + (end[0] - start[0]) * ratio)
    green = round(start[1] + (end[1] - start[1]) * ratio)
    blue = round(start[2] + (end[2] - start[2]) * ratio)
    return f"#{red:02X}{green:02X}{blue:02X}"


class HeatmapReplayApp(tk.Tk):
    def __init__(self, initial_path: Optional[str] = None):
        super().__init__()
        self.title("S48 Heatmap Replay")
        self.geometry("920x760")
        self.minsize(760, 620)

        self.frames: List[ReplayFrame] = []
        self.stats = CsvStats()
        self.csv_path = ""
        self.index = 0
        self.playing = False
        self.after_id: Optional[str] = None
        self.metric_var = tk.StringVar(value="Z")
        self.speed_var = tk.StringVar(value="1.0x")
        self.active_speed = 1.0
        self.ignore_speed_callback = False
        self.status_var = tk.StringVar(value="Open a CANFDToolPro CSV file.")
        self.time_var = tk.StringVar(value="")
        self.live_port_var = tk.StringVar(value="")
        self.live_baudrate = LIVE_BAUDRATE
        self.live_mode = False
        self.live_feed: Optional[LiveSerialFeed] = None
        self.live_queue: "queue.Queue[Tuple[str, object]]" = queue.Queue()
        self.live_poll_after_id: Optional[str] = None
        self.live_record_enabled = True
        self.live_record_path: Optional[str] = None
        self.live_recorder: Optional[LiveCsvRecorder] = None
        self.live_status_last = 0.0
        self.live_last_saved_path: Optional[str] = None
        self.live_redraw_after_id: Optional[str] = None
        self.live_dirty = False
        self.live_last_resultant: Optional[LiveResultantSample] = None
        self.max_values: Dict[str, float] = {"Z": 1.0, "Magnitude": 1.0, "X": 1.0, "Y": 1.0}
        self.circle_items: List[int] = []
        self.play_start_wall: Optional[float] = None
        self.play_start_tick: Optional[int] = None
        self.virtual_tick: Optional[int] = None

        self._build_ui()
        self.refresh_ports()
        self._set_live_ui(False)
        if initial_path and os.path.exists(initial_path):
            self.load_file(initial_path)

    def _build_ui(self) -> None:
        toolbar = ttk.Frame(self, padding=(10, 8))
        toolbar.pack(side=tk.TOP, fill=tk.X)

        self.open_button = ttk.Button(toolbar, text="Open CSV", command=self.open_file)
        self.open_button.pack(side=tk.LEFT)

        self.play_button = ttk.Button(toolbar, text="Play", command=self.toggle_play)
        self.play_button.pack(side=tk.LEFT, padx=(8, 0))

        self.stop_button = ttk.Button(toolbar, text="Stop", command=self.stop)
        self.stop_button.pack(side=tk.LEFT, padx=(8, 0))

        ttk.Label(toolbar, text="Metric").pack(side=tk.LEFT, padx=(18, 4))
        metric_box = ttk.Combobox(
            toolbar,
            textvariable=self.metric_var,
            values=("Z", "Magnitude", "X", "Y"),
            width=10,
            state="readonly",
        )
        metric_box.pack(side=tk.LEFT)
        metric_box.bind("<<ComboboxSelected>>", lambda _event: self.draw_frame())

        ttk.Label(toolbar, text="Speed").pack(side=tk.LEFT, padx=(18, 4))
        speed = ttk.Combobox(
            toolbar,
            textvariable=self.speed_var,
            values=SPEED_PRESETS,
            width=6,
            state="readonly",
        )
        speed.pack(side=tk.LEFT)
        speed.bind("<<ComboboxSelected>>", self.on_speed_change)

        real_speed = ttk.Button(
            toolbar,
            text="真实速度",
            command=self.set_real_speed,
        )
        real_speed.pack(side=tk.LEFT, padx=(12, 0))

        self.file_label = ttk.Label(toolbar, text="", anchor="w")
        self.file_label.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(18, 0))

        live_group = ttk.Frame(toolbar)
        live_group.pack(side=tk.RIGHT)
        ttk.Label(live_group, text="Live").pack(side=tk.LEFT, padx=(0, 4))
        self.port_box = ttk.Combobox(
            live_group,
            textvariable=self.live_port_var,
            values=(),
            width=14,
            state="readonly",
        )
        self.port_box.pack(side=tk.LEFT)
        self.refresh_ports_button = ttk.Button(
            live_group,
            text="Refresh",
            command=self.refresh_ports,
        )
        self.refresh_ports_button.pack(side=tk.LEFT, padx=(6, 0))
        self.open_live_button = ttk.Button(
            live_group,
            text="Start Live",
            command=self.open_live,
        )
        self.open_live_button.pack(side=tk.LEFT, padx=(6, 0))
        self.close_live_button = ttk.Button(
            live_group,
            text="Stop Live",
            command=self.close_live,
        )
        self.close_live_button.pack(side=tk.LEFT, padx=(6, 0))
        self.sensor_full_button = ttk.Button(
            live_group,
            text="Full",
            command=lambda: self.send_live_command("f", "sensor full-send"),
        )
        self.sensor_full_button.pack(side=tk.LEFT, padx=(6, 0))
        self.sensor_event_button = ttk.Button(
            live_group,
            text="Event",
            command=lambda: self.send_live_command("e", "sensor event-driven"),
        )
        self.sensor_event_button.pack(side=tk.LEFT, padx=(4, 0))
        self.resultant_on_button = ttk.Button(
            live_group,
            text="Force On",
            command=lambda: self.send_live_command("g", "resultant on"),
        )
        self.resultant_on_button.pack(side=tk.LEFT, padx=(4, 0))
        self.resultant_off_button = ttk.Button(
            live_group,
            text="Force Off",
            command=lambda: self.send_live_command("o", "resultant off"),
        )
        self.resultant_off_button.pack(side=tk.LEFT, padx=(4, 0))

        self.canvas = tk.Canvas(self, bg="white", highlightthickness=0)
        self.canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10, pady=(0, 8))
        self.canvas.bind("<Configure>", lambda _event: self.draw_frame())
        self.canvas.bind("<Motion>", self.on_mouse_move)
        self.canvas.bind("<Leave>", lambda _event: self.draw_frame())

        bottom = ttk.Frame(self, padding=(10, 0, 10, 8))
        bottom.pack(side=tk.BOTTOM, fill=tk.X)

        self.slider = ttk.Scale(
            bottom,
            from_=0,
            to=0,
            orient=tk.HORIZONTAL,
            command=self.on_slider,
        )
        self.slider.pack(side=tk.TOP, fill=tk.X)

        info = ttk.Frame(bottom)
        info.pack(side=tk.TOP, fill=tk.X, pady=(4, 0))
        ttk.Label(info, textvariable=self.time_var).pack(side=tk.LEFT)
        ttk.Label(info, textvariable=self.status_var).pack(side=tk.RIGHT)

    def open_file(self) -> None:
        path = filedialog.askopenfilename(
            title="Open CANFDToolPro CSV",
            filetypes=(("CSV files", "*.csv"), ("All files", "*.*")),
        )
        if path:
            self.load_file(path)

    def load_file(self, path: str) -> None:
        self.close_live(silent=True)
        self.stop(reset_view=False)
        try:
            frames, stats = load_replay_frames(path)
        except Exception as exc:
            messagebox.showerror("Load failed", str(exc))
            return

        self.frames = frames
        self.stats = stats
        self.csv_path = path
        self.index = 0
        self.file_label.configure(text=os.path.basename(path))
        max_index = max(0, len(self.frames) - 1)
        self.slider.configure(to=max_index)
        self.slider.set(0)
        self._recalculate_max_values()

        if not self.frames:
            self.status_var.set("No S48 array packets were decoded from this CSV.")
            self.time_var.set("")
        else:
            devices = ", ".join(f"0x{item:04X}" for item in self.stats.devices)
            duration = self.timeline_duration_ms() / 1000.0
            avg_ms = self.average_update_ms()
            self.status_var.set(
                f"{self.stats.frames} updates, {self.stats.resultants} resultants, "
                f"{duration:.2f}s, avg {avg_ms:.2f}ms, devices {devices}"
            )
        self.draw_frame()

    def refresh_ports(self) -> None:
        if list_ports is None:
            self.port_box.configure(values=())
            if not self.live_port_var.get():
                self.live_port_var.set("")
            return

        ports = [item.device for item in list_ports.comports()]
        self.port_box.configure(values=ports)
        if ports:
            current = self.live_port_var.get()
            if current not in ports:
                self.live_port_var.set(ports[0])
        else:
            self.live_port_var.set("")

    def _set_live_ui(self, enabled: bool) -> None:
        self.live_mode = enabled
        if enabled:
            self.open_live_button.configure(state=tk.DISABLED)
            self.close_live_button.configure(state=tk.NORMAL)
            self.refresh_ports_button.configure(state=tk.DISABLED)
            self.open_button.configure(state=tk.DISABLED)
            self.play_button.configure(state=tk.DISABLED)
            self.stop_button.configure(state=tk.DISABLED)
            self.port_box.configure(state=tk.DISABLED)
            self.sensor_full_button.configure(state=tk.NORMAL)
            self.sensor_event_button.configure(state=tk.NORMAL)
            self.resultant_on_button.configure(state=tk.NORMAL)
            self.resultant_off_button.configure(state=tk.NORMAL)
        else:
            self.open_live_button.configure(state=tk.NORMAL)
            self.close_live_button.configure(state=tk.DISABLED)
            self.refresh_ports_button.configure(state=tk.NORMAL)
            self.open_button.configure(state=tk.NORMAL)
            self.play_button.configure(state=tk.NORMAL)
            self.stop_button.configure(state=tk.NORMAL)
            self.port_box.configure(state="readonly")
            self.sensor_full_button.configure(state=tk.DISABLED)
            self.sensor_event_button.configure(state=tk.DISABLED)
            self.resultant_on_button.configure(state=tk.DISABLED)
            self.resultant_off_button.configure(state=tk.DISABLED)

    def open_live(self) -> None:
        if serial is None:
            messagebox.showerror("Missing dependency", "pyserial is not installed.")
            return
        port = self.live_port_var.get().strip()
        if not port:
            messagebox.showerror("No port", "Select a serial port first.")
            return

        self.close_live(silent=True)
        self.stop(reset_view=False)
        self.live_queue = queue.Queue()
        self.frames = []
        self.stats = CsvStats()
        self.live_status_last = 0.0
        self.live_last_saved_path = None
        self.live_last_resultant = None
        self.live_dirty = False
        self.max_values = {"Z": 1.0, "Magnitude": 1.0, "X": 1.0, "Y": 1.0}
        self.index = 0
        self.slider.configure(to=0)
        self.slider.set(0)
        self.file_label.configure(text=f"Live: {port} @ {self.live_baudrate}")
        self.status_var.set(f"Opening {port} @ {self.live_baudrate} ...")
        self.time_var.set("Live display and recording are starting.")

        try:
            if self.live_record_enabled:
                record_path = self.live_record_path or make_default_record_path(port)
                self.live_recorder = LiveCsvRecorder(record_path)
                self.live_recorder.open()
            self.live_feed = LiveSerialFeed(port, self.live_baudrate, self.live_queue)
            self.live_feed.start()
        except Exception as exc:
            if self.live_recorder is not None:
                self.live_recorder.close()
                self.live_recorder = None
            self.live_feed = None
            self.status_var.set(str(exc))
            messagebox.showerror("Live open failed", str(exc))
            return

        self._set_live_ui(True)
        self.draw_frame()
        self.poll_live_events()

    def close_live(self, silent: bool = False) -> None:
        saved_path = None
        if self.live_feed is not None:
            self.live_feed.close()
            self.live_feed = None
        if self.live_recorder is not None:
            saved_path = self.live_recorder.path
            self.live_recorder.close()
            self.live_recorder = None
        if self.live_poll_after_id is not None:
            try:
                self.after_cancel(self.live_poll_after_id)
            except tk.TclError:
                pass
            self.live_poll_after_id = None
        if self.live_redraw_after_id is not None:
            try:
                self.after_cancel(self.live_redraw_after_id)
            except tk.TclError:
                pass
            self.live_redraw_after_id = None
        self.live_dirty = False
        if self.live_mode:
            self._set_live_ui(False)
        if not silent:
            if saved_path:
                self.live_last_saved_path = saved_path
                self.status_var.set(f"Saved {saved_path}. Loading replay...")
                try:
                    self.load_file(saved_path)
                except Exception:
                    pass
            else:
                self.status_var.set("Live stopped.")

    def send_live_command(self, command: str, label: str) -> None:
        if self.live_feed is None:
            messagebox.showinfo("Live is not running", "Start live mode before sending sensor commands.")
            return
        try:
            self.live_feed.send_command(command)
        except Exception as exc:
            self.status_var.set(f"Command failed: {exc}")
            messagebox.showerror("Command failed", str(exc))
            return
        self.status_var.set(f"Sent command: {label}")

    def poll_live_events(self) -> None:
        alive = self.live_feed is not None
        processed = 0
        try:
            while processed < LIVE_QUEUE_BATCH_LIMIT:
                kind, payload = self.live_queue.get_nowait()
                processed += 1
                if kind == "live_status":
                    status = str(payload)
                    if self.live_recorder is not None:
                        status += f" | recording {self.live_recorder.path}"
                    self.status_var.set(status)
                elif kind == "live_error":
                    self.status_var.set(f"Live error: {payload}")
                    messagebox.showerror("Live error", str(payload))
                    self.close_live(silent=True)
                    return
                elif kind == "live_closed":
                    if self.live_feed is not None:
                        self.close_live(silent=True)
                    return
                elif kind == "live_frame":
                    self.append_live_frame(payload)
                elif kind == "live_packet":
                    packet, frame = payload
                    self.append_live_packet(packet, frame)
                elif kind == "live_resultant":
                    self.append_live_resultant(payload)
        except queue.Empty:
            pass

        if alive and self.live_feed is not None:
            delay = 1 if processed >= LIVE_QUEUE_BATCH_LIMIT else 20
            self.live_poll_after_id = self.after(delay, self.poll_live_events)

    def append_live_packet(self, packet: LiveSensorPacket, frame: Optional[ReplayFrame]) -> None:
        if self.live_recorder is not None:
            try:
                self.live_recorder.write_packet(packet)
            except Exception as exc:
                self.status_var.set(f"Record failed: {exc}")
                messagebox.showerror("Record failed", str(exc))
                self.close_live(silent=True)
                return

        self.stats.frames += 1
        self.stats.decoded_packets += 1
        if frame is not None and frame.complete:
            self.stats.complete_frames += 1
        if frame is not None:
            self.append_live_display_frame(frame)
        devices = set(self.stats.devices)
        devices.add(packet.device_id)
        self.stats.devices = tuple(sorted(devices))

        now = time.perf_counter()
        if now - self.live_status_last >= 0.5:
            self.live_status_last = now
            saved = (
                f" saved {os.path.basename(self.live_recorder.path)}"
                if self.live_recorder is not None
                else ""
            )
            mask = frame.seen_mask if frame is not None else 0
            resultants = self.live_feed.stats.resultants if self.live_feed else 0
            self.status_var.set(
                f"Live {self.live_feed.port if self.live_feed else ''} "
                f"packets={self.stats.decoded_packets} resultants={resultants} "
                f"device=0x{packet.device_id:04X} seq={packet.packet_seq} mask={mask:03b}{saved}"
            )
            self.time_var.set(
                f"Live packets {self.stats.decoded_packets}, resultants {resultants}"
            )

    def append_live_frame(self, frame: ReplayFrame) -> None:
        if self.live_recorder is not None:
            try:
                self.live_recorder.write_frame(frame)
            except Exception as exc:
                self.status_var.set(f"Record failed: {exc}")
                messagebox.showerror("Record failed", str(exc))
                self.close_live(silent=True)
                return

        self.stats.frames += 1
        self.stats.complete_frames += 1
        self.stats.decoded_packets += PACKETS_PER_FRAME
        self.append_live_display_frame(frame)
        devices = set(self.stats.devices)
        devices.add(frame.device_id)
        self.stats.devices = tuple(sorted(devices))
        now = time.perf_counter()
        if now - self.live_status_last >= 0.5:
            self.live_status_last = now
            saved = (
                f" saved {os.path.basename(self.live_recorder.path)}"
                if self.live_recorder is not None
                else ""
            )
            self.status_var.set(
                f"Live {self.live_feed.port if self.live_feed else ''} "
                f"frames={self.stats.frames} device=0x{frame.device_id:04X}{saved}"
            )
            self.time_var.set(
                f"Live frames {self.stats.frames}, packets {self.stats.decoded_packets}"
            )

    def append_live_resultant(self, sample: LiveResultantSample) -> None:
        if self.live_recorder is not None:
            try:
                self.live_recorder.write_resultant(sample)
            except Exception as exc:
                self.status_var.set(f"Record failed: {exc}")
                messagebox.showerror("Record failed", str(exc))
                self.close_live(silent=True)
                return

        self.stats.resultants += 1
        self.live_last_resultant = sample
        devices = set(self.stats.devices)
        devices.add(sample.device_id)
        self.stats.devices = tuple(sorted(devices))

        now = time.perf_counter()
        if now - self.live_status_last >= 0.5:
            self.live_status_last = now
            saved = (
                f" saved {os.path.basename(self.live_recorder.path)}"
                if self.live_recorder is not None
                else ""
            )
            resultants = self.live_feed.stats.resultants if self.live_feed else self.stats.resultants
            self.status_var.set(
                f"Live {self.live_feed.port if self.live_feed else ''} "
                f"resultants={resultants}{saved}"
            )

    def append_live_display_frame(self, frame: ReplayFrame) -> None:
        self.frames.append(frame)
        if len(self.frames) > LIVE_HISTORY_LIMIT:
            overflow = len(self.frames) - LIVE_HISTORY_LIMIT
            del self.frames[:overflow]
        self.index = len(self.frames) - 1
        self.update_live_max_values(frame)
        self.schedule_live_redraw()

    def update_live_max_values(self, frame: ReplayFrame) -> None:
        values = self.max_values
        for point in frame.points:
            values["Z"] = max(values["Z"], float(point.z))
            values["Magnitude"] = max(values["Magnitude"], point.magnitude)
            values["X"] = max(values["X"], float(abs(point.x)))
            values["Y"] = max(values["Y"], float(abs(point.y)))

    def schedule_live_redraw(self) -> None:
        self.live_dirty = True
        if self.live_redraw_after_id is not None:
            return
        self.live_redraw_after_id = self.after(LIVE_REDRAW_INTERVAL_MS, self.flush_live_redraw)

    def flush_live_redraw(self) -> None:
        self.live_redraw_after_id = None
        if not self.live_dirty:
            return
        self.live_dirty = False
        if self.frames:
            self.index = len(self.frames) - 1
            self.slider.configure(to=max(0, len(self.frames) - 1))
            self.slider.set(self.index)
        self.draw_frame()

    def _recalculate_max_values(self) -> None:
        values = {"Z": 0.0, "Magnitude": 0.0, "X": 0.0, "Y": 0.0}
        for frame in self.frames:
            for point in frame.points:
                values["Z"] = max(values["Z"], float(point.z))
                values["Magnitude"] = max(values["Magnitude"], point.magnitude)
                values["X"] = max(values["X"], float(abs(point.x)))
                values["Y"] = max(values["Y"], float(abs(point.y)))
        for key in values:
            values[key] = max(1.0, values[key])
        self.max_values = values

    def timeline_is_usable(self) -> bool:
        if len(self.frames) < 2:
            return False
        first = self.frames[0].time_tick
        last = self.frames[-1].time_tick
        return first is not None and last is not None and last > first

    def timeline_duration_ms(self) -> float:
        if not self.timeline_is_usable():
            return 0.0
        first = self.frames[0].time_tick
        last = self.frames[-1].time_tick
        assert first is not None and last is not None
        return (last - first) / 1000.0

    def average_update_ms(self) -> float:
        if len(self.frames) < 2:
            return 0.0
        duration = self.timeline_duration_ms()
        return duration / max(1, len(self.frames) - 1)

    def reset_playback_clock(self, tick: Optional[int] = None) -> None:
        if not self.frames:
            return
        self.play_start_wall = time.perf_counter()
        self.play_start_tick = tick if tick is not None else self.frames[self.index].time_tick
        self.virtual_tick = self.play_start_tick

    def read_speed(self, value: Optional[str] = None) -> float:
        try:
            raw = value if value is not None else self.speed_var.get()
            speed = float(str(raw).strip().lower().removesuffix("x"))
        except (tk.TclError, ValueError):
            speed = 1.0
        return max(0.1, min(5.0, speed))

    def current_anchor_tick(self) -> Optional[int]:
        if not self.frames:
            return None
        if (
            self.playing
            and self.timeline_is_usable()
            and self.play_start_wall is not None
            and self.play_start_tick is not None
        ):
            elapsed_us = int(
                (time.perf_counter() - self.play_start_wall)
                * 1_000_000
                * self.active_speed
            )
            tick = self.play_start_tick + elapsed_us
            first_tick = self.frames[0].time_tick
            last_tick = self.frames[-1].time_tick
            if first_tick is not None:
                tick = max(first_tick, tick)
            if last_tick is not None:
                tick = min(last_tick, tick)
            return tick
        if self.virtual_tick is not None:
            return self.virtual_tick
        return self.frames[self.index].time_tick

    def on_speed_change(self, _event=None) -> None:
        if self.ignore_speed_callback:
            return
        anchor_tick = self.current_anchor_tick()
        self.active_speed = self.read_speed()
        if not self.playing or not self.timeline_is_usable():
            return
        # Keep the visible playback time continuous while the speed changes.
        self.reset_playback_clock(anchor_tick)

    def set_real_speed(self) -> None:
        anchor_tick = self.current_anchor_tick()
        self.ignore_speed_callback = True
        self.speed_var.set("1.0x")
        self.ignore_speed_callback = False
        self.active_speed = 1.0
        if self.playing and self.timeline_is_usable():
            self.reset_playback_clock(anchor_tick)
        self.draw_frame()

    def toggle_play(self) -> None:
        if self.live_mode or not self.frames:
            return
        self.playing = not self.playing
        self.play_button.configure(text="Pause" if self.playing else "Play")
        if self.playing:
            self.active_speed = self.read_speed()
            self.reset_playback_clock()
            self.schedule_next()

    def stop(self, reset_view: bool = True) -> None:
        self.playing = False
        self.play_button.configure(text="Play")
        self.play_start_wall = None
        self.play_start_tick = None
        if self.after_id is not None:
            try:
                self.after_cancel(self.after_id)
            except tk.TclError:
                pass
            self.after_id = None
        if reset_view and self.frames:
            self.index = 0
            self.virtual_tick = self.frames[0].time_tick
            self.slider.set(0)
            self.draw_frame()

    def schedule_next(self) -> None:
        if not self.playing:
            return
        if self.timeline_is_usable():
            self.after_id = self.after(33, self.advance_real_time)
            return
        delay = self.next_delay_ms()
        self.after_id = self.after(delay, self.advance)

    def next_delay_ms(self) -> int:
        if len(self.frames) < 2 or self.index >= len(self.frames) - 1:
            return 40
        current = self.frames[self.index]
        nxt = self.frames[self.index + 1]
        if current.time_tick is None or nxt.time_tick is None:
            raw_ms = 40.0
        else:
            raw_ms = (nxt.time_tick - current.time_tick) / 1000.0
            if raw_ms <= 0:
                raw_ms = 10.0
            # Fast mode compresses long capture gaps for quick browsing.
            raw_ms = min(raw_ms, 300.0)
        speed = self.read_speed()
        return max(8, int(raw_ms / speed))

    def advance_real_time(self) -> None:
        if not self.frames or not self.timeline_is_usable():
            self.advance()
            return
        if self.play_start_wall is None or self.play_start_tick is None:
            self.reset_playback_clock()
        if self.play_start_wall is None or self.play_start_tick is None:
            self.stop()
            return

        elapsed_us = int(
            (time.perf_counter() - self.play_start_wall)
            * 1_000_000
            * self.active_speed
        )
        target_tick = self.play_start_tick + elapsed_us
        self.virtual_tick = target_tick

        last_tick = self.frames[-1].time_tick
        if last_tick is not None and target_tick >= last_tick:
            self.index = len(self.frames) - 1
            self.slider.set(self.index)
            self.virtual_tick = last_tick
            self.draw_frame()
            self.stop()
            return

        while self.index + 1 < len(self.frames):
            next_tick = self.frames[self.index + 1].time_tick
            if next_tick is None or next_tick > target_tick:
                break
            self.index += 1

        self.slider.set(self.index)
        self.draw_frame()
        self.schedule_next()

    def advance(self) -> None:
        if not self.frames:
            self.stop()
            return
        if self.index >= len(self.frames) - 1:
            self.stop()
            return
        self.index += 1
        self.slider.set(self.index)
        self.draw_frame()
        self.schedule_next()

    def on_slider(self, value: str) -> None:
        if not self.frames:
            return
        try:
            new_index = int(float(value))
        except ValueError:
            return
        new_index = max(0, min(len(self.frames) - 1, new_index))
        if new_index != self.index:
            self.index = new_index
            if self.playing:
                self.reset_playback_clock()
            self.draw_frame()

    def grid_layout(self) -> Tuple[float, float, float, float, float]:
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        left = 90
        right = 60
        top = 82
        bottom = 70
        usable_w = max(200, width - left - right)
        usable_h = max(240, height - top - bottom)
        step_x = usable_w / max(1, DISPLAY_COLS - 1)
        step_y = usable_h / max(1, DISPLAY_ROWS - 1)
        radius = max(12.0, min(28.0, min(step_x, step_y) * 0.18))
        return left, top, step_x, step_y, radius

    def point_positions(self) -> List[Tuple[float, float, float]]:
        left, top, step_x, step_y, radius = self.grid_layout()
        positions: List[Tuple[float, float, float]] = []
        for index in range(POINTS):
            row = index // COLS
            col = index % COLS
            display_row = row
            display_col = COLS - 1 - col
            positions.append(
                (
                    left + display_col * step_x,
                    top + display_row * step_y,
                    radius,
                )
            )
        return positions

    def draw_frame(self, hover_index: Optional[int] = None) -> None:
        self.canvas.delete("all")
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        self.canvas.create_rectangle(0, 0, width, height, fill="white", outline="")

        if not self.frames:
            if self.live_mode:
                extra = ""
                if self.live_recorder is not None:
                    extra = f"\nSaving to: {os.path.basename(self.live_recorder.path)}"
                self.canvas.create_text(
                    width / 2,
                    height / 2,
                    text="Waiting for live S48 packets..." + extra,
                    fill="#475569",
                    font=("Segoe UI", 16),
                )
                return
            self.canvas.create_text(
                width / 2,
                height / 2,
                text="Open a CSV file to show the 8 x 6 heatmap.",
                fill="#475569",
                font=("Segoe UI", 16),
            )
            return

        frame = self.frames[self.index]
        metric = self.metric_var.get()
        max_value = self.max_values.get(metric, 1.0)
        positions = self.point_positions()

        self.canvas.create_text(
            28,
            24,
            text="Tactile Array 8 x 6",
            anchor="w",
            fill="#0F172A",
            font=("Segoe UI", 20, "bold"),
        )
        subtitle = (
            f"Device 0x{frame.device_id:04X}   Metric {metric}   "
            f"Packet C{frame.packet_seq}   View mirrored left/right"
        )
        if self.live_mode and self.live_last_resultant is not None:
            force = self.live_last_resultant
            subtitle += f"   Force {force.fx:.1f},{force.fy:.1f},{force.fz:.1f} N"

        self.canvas.create_text(
            28,
            52,
            text=subtitle,
            anchor="w",
            fill="#475569",
            font=("Segoe UI", 11),
        )

        left, top, step_x, step_y, _radius = self.grid_layout()
        for col in range(DISPLAY_COLS):
            x = left + col * step_x
            self.canvas.create_text(x, 66, text=str(col), fill="#64748B", font=("Segoe UI", 11))
        for row in range(DISPLAY_ROWS):
            y = top + row * step_y
            self.canvas.create_text(38, y, text=str(row), fill="#64748B", font=("Segoe UI", 11))

        for index, point in enumerate(frame.points):
            x, y, radius = positions[index]
            value = value_for_metric(point, metric)
            fill = color_for_value(value, max_value)
            outline = "#0F3E8A" if index == hover_index else "#CBD5E1"
            width_line = 2 if index == hover_index else 1
            self.canvas.create_oval(
                x - radius,
                y - radius,
                x + radius,
                y + radius,
                fill=fill,
                outline=outline,
                width=width_line,
            )

        self.draw_color_bar(metric, max_value)
        complete_text = "complete" if frame.complete else "partial"
        elapsed_text = ""
        if self.timeline_is_usable():
            first_tick = self.frames[0].time_tick
            shown_tick = self.virtual_tick if self.playing and self.virtual_tick is not None else frame.time_tick
            if first_tick is not None and shown_tick is not None:
                elapsed_text = f"  t+{max(0.0, (shown_tick - first_tick) / 1_000_000.0):.3f}s"
        self.time_var.set(
            f"{self.index + 1}/{len(self.frames)}  {frame.system_time}{elapsed_text}  "
            f"row {frame.source_row}  {complete_text}"
        )

    def draw_color_bar(self, metric: str, max_value: float) -> None:
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        bar_w = 220
        bar_h = 12
        x0 = width - bar_w - 44
        y0 = height - 42
        steps = 80
        for step in range(steps):
            ratio = step / max(1, steps - 1)
            color = color_for_value(ratio * max_value, max_value)
            x1 = x0 + step * bar_w / steps
            x2 = x0 + (step + 1) * bar_w / steps
            self.canvas.create_rectangle(x1, y0, x2, y0 + bar_h, fill=color, outline="")
        self.canvas.create_rectangle(x0, y0, x0 + bar_w, y0 + bar_h, outline="#CBD5E1")
        self.canvas.create_text(x0, y0 - 8, text="0", anchor="w", fill="#64748B", font=("Segoe UI", 9))
        self.canvas.create_text(
            x0 + bar_w,
            y0 - 8,
            text=f"{max_value:.0f}",
            anchor="e",
            fill="#64748B",
            font=("Segoe UI", 9),
        )
        self.canvas.create_text(
            x0 + bar_w / 2,
            y0 + 27,
            text=metric,
            fill="#64748B",
            font=("Segoe UI", 9),
        )

    def on_mouse_move(self, event) -> None:
        if not self.frames:
            return
        positions = self.point_positions()
        nearest = None
        nearest_dist = float("inf")
        for index, (x, y, radius) in enumerate(positions):
            dist = math.hypot(event.x - x, event.y - y)
            if dist < radius * 1.6 and dist < nearest_dist:
                nearest = index
                nearest_dist = dist
        if nearest is None:
            self.draw_frame()
            return

        frame = self.frames[self.index]
        point = frame.points[nearest]
        row = nearest // COLS
        col = nearest % COLS
        display_row = row
        display_col = COLS - 1 - col
        self.status_var.set(
            f"R{display_row} C{display_col}: X={point.x}, Y={point.y}, Z={point.z}, "
            f"|F|={point.magnitude:.1f}   raw R{row} C{col}"
        )
        self.draw_frame(hover_index=nearest)


def print_summary(path: str) -> int:
    frames, stats = load_replay_frames(path)
    print(f"file: {path}")
    print(f"encoding: {stats.encoding}")
    print(f"csv rows: {stats.rows}")
    print(f"decoded array packets: {stats.decoded_packets}")
    print(f"resultant packets: {stats.resultants}")
    print(f"replay updates: {stats.frames}")
    print(f"complete updates: {stats.complete_frames}")
    print(f"log resets: {stats.reset_count}")
    print("devices:", ", ".join(f"0x{item:04X}" for item in stats.devices) or "none")
    if frames:
        print(f"first time: {frames[0].system_time}")
        print(f"last time: {frames[-1].system_time}")
    return 0


def main(argv: Sequence[str]) -> int:
    parser = argparse.ArgumentParser(description="S48 tactile replay and live upper computer.")
    parser.add_argument("path", nargs="?", default=DEFAULT_CSV, help="CSV log to open in replay mode.")
    parser.add_argument("--summary", action="store_true", help="Print CSV summary and exit.")
    parser.add_argument("--live", metavar="PORT", help="Open live serial mode on this port.")
    parser.add_argument("--baud", type=int, default=LIVE_BAUDRATE, help="Live serial baudrate.")
    parser.add_argument("--record", metavar="CSV", help="Save live frames to this CSV path.")
    parser.add_argument("--no-record", action="store_true", help="Disable live CSV recording.")
    ns = parser.parse_args(list(argv))

    if ns.summary:
        return print_summary(ns.path)

    app = HeatmapReplayApp(ns.path if os.path.exists(ns.path) and not ns.live else None)
    if ns.live:
        app.live_baudrate = ns.baud
        app.live_record_enabled = not ns.no_record
        app.live_record_path = ns.record
        app.live_port_var.set(ns.live)
        app.after(100, app.open_live)
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
