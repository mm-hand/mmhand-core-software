"""Token-correlated P4 motor maintenance frames; no UI or device ownership.

职责：编码电机维护协议并规划无冲突编号步骤。
线程：纯函数，由操作工作线程调用。
输入：设备清单、目标ID及恢复日志。
输出：维护字节帧、结果和移动序列。
状态所有权：无设备状态所有权。
禁止依赖：禁止串口、UI或文件IO。
"""
import struct
import threading
import time
from . import protocol

CAPABILITY, BEGIN, END, PROBE, RENAME = range(5)



def command_frame(command, token, session, operation, bus=0, old_id=0, new_id=0):
    payload = struct.pack('>IIBBBB', token, session, operation, bus, old_id, new_id)
    return bytes((254, len(payload) + 2, command)) + payload + b'\xff'


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
    """Validate the complete inventory and plan collision-free serial moves."""
    locations = {(int(r['bus']), int(r['id'])) for r in rows}
    if len(locations) != len(rows):
        raise ValueError('Duplicate BUS/ID entries; connect same-ID servos separately')
    if any(i == 253 for _, i in locations):
        raise ValueError('Temporary ID 253 is occupied')
    if set(targets) != locations:
        raise ValueError('Scan the buses again before renumbering')
    final = {}
    for location, value in targets.items():
        if isinstance(value, bool) or value is None or int(value) != value or not 0 <= int(value) <= 252:
            raise ValueError('New IDs must be integers in 0..252')
        final[location] = int(value)
    if len(set(final.values())) != len(final):
        raise ValueError('Final IDs must be unique across all buses')
    pending = {k: v for k, v in final.items() if k[1] != v}
    occupied = set(locations)
    moves = []
    while pending:
        free = next((k for k, target in pending.items() if (k[0], target) not in occupied), None)
        if free is not None:
            target = pending.pop(free)
            bus, old = free
            # A cycle's parked servo already owns the temporary ID.
            if old != 253 and (bus, 253) not in occupied:
                moves.append((bus, old, 253))
                moves.append((bus, 253, target))
            else:
                moves.append((bus, old, target))
            occupied.remove(free); occupied.add((bus, target))
            continue
        # Park one vertex to break a cycle, then resolve its predecessors.
        location = next(iter(pending))
        bus, old = location
        moves.append((bus, old, 253))
        occupied.remove(location); occupied.add((bus, 253))
        pending[(bus, 253)] = pending.pop(location)
    return moves


def reconcile_motor_journal(journal, rows):
    """Resolve a lost write result using a fresh full inventory, never by replaying it."""
    actual = {(r['bus'], r['id']) for r in rows}
    expected = {(r['bus'], r['id']) for r in journal['inventory']}
    moves = [tuple(move) for move in journal['moves']]
    completed = int(journal['completed'])
    for bus, old, new in moves[:completed]:
        expected.remove((bus, old)); expected.add((bus, new))
    if expected == actual:
        return completed
    if completed < len(moves) and journal.get('intent') == list(moves[completed]):
        bus, old, new = moves[completed]
        after = (expected - {(bus, old)}) | {(bus, new)}
        if after == actual:
            return completed + 1
    raise ValueError('Recorded batch does not match scanned addresses; inspect the wiring before continuing')


def motor_recovery_targets(journal, rows):
    completed = reconcile_motor_journal(journal, rows)
    positions = {index:(row['bus'],row['id']) for index,row in enumerate(journal['inventory'])}
    current = None
    for index,(bus,old,new) in enumerate(journal['moves']):
        if index == completed:
            current = dict(positions)
        actor = next(key for key,location in positions.items() if location==(bus,old))
        positions[actor] = (bus,new)
    if current is None:
        current = dict(positions)
    return {location:positions[actor][1] for actor,location in current.items()}
