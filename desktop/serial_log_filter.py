"""serial_log_filter.py - MMHand Desktop 模块契约。

职责：过滤串口文本日志并保留诊断所需记录。
线程：纯模块初始化或工具调用线程执行，不创建后台线程。
输入：包配置、路径参数或文本流。
输出：稳定导入边界、路径或过滤后的数据。
状态所有权：只拥有模块级不可变配置，不拥有设备运行状态。
禁止依赖：禁止导入 Qt 控件或直接操作下位机通信。
"""
import argparse
import os
import sys
import time
from typing import Dict

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

try:
    import serial
except ModuleNotFoundError:
    print("Missing Python dependency: serial")
    print("Run: python -m pip install pyserial")
    raise

from desktop.core.comm_layer import list_ports
from desktop.core.protocol import BAUDRATE, PROTOCOL_HEADER, PROTOCOL_TAIL, build_calibrate_cmd


PACKET_NAMES = {
    0x01: "sensorMapped",
    0x02: "calibAck",
    0x03: "servoAbs",
    0x04: "jointDebug",
    0x05: "servoTelem",
    0x06: "protoAck",
    0x07: "overloadFault",
    0x08: "releaseFault",
    0x09: "servoRaw",
    0x0A: "tactile",
    0x0B: "encoderRaw",
    0x0C: "systemStatus",
}


def resolve_port(port: str | None) -> str:
    if port:
        return port
    ports = list_ports()
    if len(ports) == 1:
        return ports[0][0]
    if not ports:
        raise RuntimeError("No serial port found. Use --port COMx.")
    names = ", ".join(p for p, _desc in ports)
    raise RuntimeError(f"Multiple serial ports found: {names}. Use --port COMx.")


def flush_text(text_buf: bytearray) -> None:
    if not text_buf:
        return
    text = bytes(text_buf).decode("utf-8", errors="ignore")
    text_buf.clear()
    if text:
        print(text, end="", flush=True)


def append_printable(text_buf: bytearray, value: int) -> None:
    if value in (9, 10, 13) or 32 <= value <= 126:
        text_buf.append(value)
    elif text_buf and text_buf[-1] not in (10, 13, 32):
        text_buf.append(32)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Hide binary protocol packets and print only firmware text logs."
    )
    parser.add_argument("--port", "-p", default=None, help="Serial port, e.g. COM19")
    parser.add_argument("--baud", type=int, default=BAUDRATE, help="Serial baudrate")
    parser.add_argument("--seconds", type=float, default=0.0, help="Run duration; 0 means forever")
    parser.add_argument("--packet-summary", action="store_true", help="Print decoded packet counts once per second")
    parser.add_argument("--calibrate", action="store_true", help="Send CALIBRATE command after opening the port")
    args = parser.parse_args()

    port = resolve_port(args.port)
    deadline = None if args.seconds <= 0 else time.time() + args.seconds
    buffer = bytearray()
    text_buf = bytearray()
    packet_counts: Dict[int, int] = {}
    last_summary = time.time()

    print(f"[LOG FILTER] open {port} @ {args.baud}. Close Arduino monitor / upper UI first.")
    print("[LOG FILTER] binary protocol packets are hidden; firmware text logs remain visible.\n")

    with serial.Serial(port, args.baud, timeout=0.05) as ser:
        if args.calibrate:
            ser.write(build_calibrate_cmd())
            ser.flush()
            print("[LOG FILTER] sent CALIBRATE command.\n", flush=True)

        while deadline is None or time.time() < deadline:
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                buffer.extend(chunk)

            while buffer:
                if buffer[0] != PROTOCOL_HEADER:
                    append_printable(text_buf, buffer.pop(0))
                    continue

                if len(buffer) < 4:
                    break

                frame_len = 2 + buffer[1]
                if frame_len < 4:
                    append_printable(text_buf, buffer.pop(0))
                    continue

                if len(buffer) < frame_len:
                    break

                if buffer[frame_len - 1] == PROTOCOL_TAIL:
                    flush_text(text_buf)
                    pkt_type = buffer[2]
                    packet_counts[pkt_type] = packet_counts.get(pkt_type, 0) + 1
                    del buffer[:frame_len]
                else:
                    append_printable(text_buf, buffer.pop(0))

            now = time.time()
            if args.packet_summary and now - last_summary >= 1.0:
                last_summary = now
                if packet_counts:
                    bits = [
                        f"0x{k:02X}:{PACKET_NAMES.get(k, 'unknown')}={packet_counts[k]}"
                        for k in sorted(packet_counts)
                    ]
                    print("\n[PACKETS] " + " ".join(bits), flush=True)
                    packet_counts.clear()

        flush_text(text_buf)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
