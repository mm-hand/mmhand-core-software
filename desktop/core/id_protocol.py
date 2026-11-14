"""Token-correlated P4 motor maintenance frames; no UI or device ownership.

职责：编码电机维护协议并规划无冲突编号步骤。
线程：纯函数，由操作工作线程调用。
输入：设备清单和目标ID。
输出：维护字节帧、结果和移动序列。
状态所有权：无设备状态所有权。
禁止依赖：禁止串口、UI或文件IO。
"""
import struct
import threading
import time
from . import protocol

CAPABILITY, BEGIN, END, PROBE, RENAME, BUZZ = range(6)
TEMPORARY_MOTOR_ID = 253


def command_frame(command, token, session, operation, bus=0, old_id=0, new_id=0):
    payload = struct.pack('>IIBBBB', token, session, operation, bus, old_id, new_id)
    return protocol._build_command_frame(command, payload)


class MaintenanceChannel:
    def __init__(self):
        self.condition = threading.Condition()
        self.responses = {}
        self.sequence = int(time.time_ns() & 0xffffffff) or 1
        self.closed = False

    def feed(self, payload):
        if len(payload) != 16 or payload[14] != 3:
            return False
        fields = struct.unpack('>IIBBBBBBBB', payload[:16])
        result = dict(zip(('token', 'session', 'operation', 'bus', 'old_id', 'new_id',
                           'status', 'present', 'version', 'stage'), fields))
        with self.condition:
            self.responses[result['token']] = result
            while len(self.responses) > 32:
                self.responses.pop(next(iter(self.responses)))
            self.condition.notify_all()
        return True

    def close(self):
        with self.condition:
            self.closed = True
            self.responses.clear()
            self.condition.notify_all()

    def request(self, send, command, session, operation, bus=0, old_id=0, new_id=0,
                cancelled=lambda: False, timeout=2.0):
        with self.condition:
            if self.closed:
                raise ConnectionError('Maintenance connection is closed')
            self.sequence = (self.sequence + 1) & 0xffffffff or 1
            token = self.sequence
        if not send(command_frame(command, token, session, operation, bus, old_id, new_id)):
            raise ConnectionError('Maintenance command was not sent')
        deadline = time.monotonic() + timeout
        with self.condition:
            while token not in self.responses:
                if self.closed or cancelled():
                    raise InterruptedError('Maintenance cancelled; verify device addresses')
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('Maintenance result timed out; verify device addresses')
                self.condition.wait(min(remaining, .05))
            result = self.responses.pop(token)
        if (result['session'], result['operation'], result['bus'], result['old_id'], result['new_id']) != (session, operation, bus, old_id, new_id):
            raise ValueError('Maintenance response does not match request')
        return result


def plan_motor_ids(rows, targets):
    """Validate the scanned inventory and plan collision-free serial moves."""
    locations = {(int(r['bus']), int(r['id'])) for r in rows}
    if len(locations) != len(rows):
        raise ValueError('Duplicate BUS/ID entries; connect same-ID servos separately')
    if set(targets) != locations:
        raise ValueError('Scan the buses again before renumbering')
    final = {}
    for location, value in targets.items():
        if isinstance(value, bool) or value is None or int(value) != value or not 0 <= int(value) < TEMPORARY_MOTOR_ID:
            raise ValueError('New IDs must be integers in 0..252')
        final[location] = int(value)
    if len(set(final.values())) != len(final):
        raise ValueError('Final IDs must be unique across all buses')
    pending = {k: v for k, v in final.items() if k[1] != v}
    occupied = locations
    moves = []
    while pending:
        location = next((key for key, target in pending.items() if (key[0], target) not in occupied), None)
        if location is None:
            # With no free target, the remaining moves form cycles.
            location = next(iter(pending))
            new = TEMPORARY_MOTOR_ID
            pending[(location[0], new)] = pending.pop(location)
        else:
            new = pending.pop(location)
        bus, old = location
        moves.append((bus, old, new))
        occupied.remove(location)
        occupied.add((bus, new))
    return moves
