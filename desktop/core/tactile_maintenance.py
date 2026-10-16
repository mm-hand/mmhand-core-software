"""Direct S3 USB maintenance protocol, separate from P4 HostComm.

职责：编码S3独立维护帧并匹配事务响应。
线程：串口RX投递；操作线程等待条件变量。
输入：事务标识、选择器、型号和目标编号。
输出：CRC校验帧、超时和确定结果。
状态所有权：拥有有界响应缓存和事务序号。
禁止依赖：禁止UI、P4命令或盲目重发写入。
"""
from __future__ import annotations

import struct
import threading
import time

from .protocol import crc16_ccitt

REQUEST_TYPE = 0x31
RESULT_TYPE = 0x32
SCAN, RENUMBER, RECONCILE = 0, 1, 2
STATUS = {
    0: 'VERIFIED', 1: 'NONE', 2: 'DESTINATION_OCCUPIED', 3: 'UNKNOWN_MODEL',
    4: 'INVALID_TARGET', 5: 'BUSY', 6: 'WRITE_SENT_UNVERIFIED',
    7: 'TIMEOUT_AMBIGUOUS', 8: 'VERIFY_FAILED', 9: 'CANCELLED',
}


def request_frame(op: int, token: int, selector: int = 0xFF,
                  model: int = 0xFF, new_id: int = 0xFF) -> bytes:
    if op not in (SCAN, RENUMBER, RECONCILE) or not 0 < token <= 0xFFFFFFFF or not 0 <= selector < 15:
        raise ValueError('invalid S3 maintenance operation or token')
    if op in (RENUMBER, RECONCILE) and not (0 <= selector < 15 and model in (1, 2) and new_id in (0, 1, 2)):
        raise ValueError('tactile renumber requires scanned selector, model and new ID 0..2')
    body = struct.pack('>BBIBBBB', 2, op, token, selector, model, new_id, 0)
    body += struct.pack('>H', crc16_ccitt(body))
    return bytes((0xFE, len(body) + 2, REQUEST_TYPE)) + body + b'\xFF'


def parse_result(body: bytes) -> dict | None:
    if len(body) != 22 or body[0] != 2:
        return None
    if crc16_ccitt(body[:20]) != struct.unpack('>H', body[20:22])[0]:
        return None
    return dict(version=2, op=body[1], token=struct.unpack('>I', body[2:6])[0],
                stage=body[6], status=body[7], status_name=STATUS.get(body[7], 'UNKNOWN'),
                old_selector=body[8], new_selector=body[9], model=body[10],
                points=body[11], old_valid=bool(body[12]), new_valid=bool(body[13]),
                semantic=body[15],
                write_sent=bool(body[14]), s3_time_ms=struct.unpack('>I', body[16:20])[0])


class TactileMaintenanceChannel:
    def __init__(self):
        self.condition = threading.Condition()
        self.responses = {}
        self.sequence = int(time.time_ns() & 0xFFFFFFFF) or 1
        self.closed = False

    def feed(self, payload: bytes) -> bool:
        result = parse_result(payload)
        if result is None:
            return False
        with self.condition:
            self.responses[result['token']] = result
            while len(self.responses) > 16:
                self.responses.pop(next(iter(self.responses)))
            self.condition.notify_all()
        return True

    def close(self):
        with self.condition:
            self.closed = True
            self.responses.clear()
            self.condition.notify_all()

    def request(self, send, op: int, selector: int = 0xFF, model: int = 0xFF,
                new_id: int = 0xFF, timeout: float = 3.0, cancelled=lambda: False) -> dict:
        with self.condition:
            if self.closed:
                raise ConnectionError('S3 maintenance connection closed')
            self.sequence = (self.sequence + 1) & 0xFFFFFFFF or 1
            token = self.sequence
        packet = request_frame(op, token, selector, model, new_id)
        if send(packet) != len(packet):
            raise ConnectionError('S3 maintenance request was not sent')
        deadline = time.monotonic() + timeout
        with self.condition:
            while token not in self.responses:
                if self.closed or cancelled():
                    raise InterruptedError('S3 maintenance request cancelled; scan again')
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('S3 maintenance result timed out; scan again before any write')
                self.condition.wait(min(remaining, 0.05))
            result = self.responses.pop(token)
        if result['op'] != op:
            raise ValueError('S3 maintenance response operation mismatch')
        return result


def plan_tactile_ids(rows, targets):
    """Plan moves within each finger's three reachable selectors, including cycles with a free slot."""
    locations = {(int(row['bus']), int(row['id'])) for row in rows}
    if len(locations) != len(rows) or set(targets) != locations:
        raise ValueError('Tactile inventory changed; scan all fingers again')
    final = {}
    for location, value in targets.items():
        if value is None or value == '': value = location[1]
        if isinstance(value, bool) or int(value) != value or int(value) not in (0, 1, 2):
            raise ValueError('Tactile New IDs must be 0, 1 or 2; blank keeps the current ID')
        final[location] = int(value)
    moves = []
    for bus in range(5):
        occupied = {identifier for finger, identifier in locations if finger == bus}
        values = [value for (finger, _), value in final.items() if finger == bus]
        if len(values) != len(set(values)):
            raise ValueError(f'Finger {bus}: final tactile IDs must be unique on that bus')
        pending = {old: value for (finger, old), value in final.items() if finger == bus and old != value}
        while pending:
            source = next((old for old, target in pending.items() if target not in occupied), None)
            if source is not None:
                target = pending.pop(source)
            else:
                free = sorted({0, 1, 2} - occupied)
                if not free:
                    raise ValueError(f'Finger {bus}: an ID swap needs a free reachable ID 0–2; disconnect one sensor first')
                source = next(iter(pending)); target = free[0]
                pending[target] = pending.pop(source)
            moves.append((bus, source, target))
            occupied.remove(source); occupied.add(target)
    return moves
