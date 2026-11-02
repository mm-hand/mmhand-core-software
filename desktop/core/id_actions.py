"""Assembly ID transactions owned by HandBackend, never by the views.

职责：编排电机扫描、批量改号及恢复。
线程：应用操作工作线程。
输入：P4维护响应、用户编号和设备身份。
输出：逐步维护请求及只读进度。
状态所有权：拥有维护草稿和恢复日志。
禁止依赖：禁止直接访问UART或UI控件。
"""
from copy import deepcopy
import json
from uuid import uuid4
from desktop.paths import DEVICE_STATE_DIR, atomic_write_json
from .id_protocol import CAPABILITY, BEGIN, END, PROBE, RENAME, plan_motor_ids, reconcile_motor_journal, motor_recovery_targets

MOTOR_SCAN_ID_COUNT = 31
MOTOR_BUS_COUNT = 4


class IdResetActions:
    def _init_id_reset(self):
        self._id_reset = {
            'motor': dict(rows=[], message='Connect P4 and scan all buses.', generation=0, scanned=False, recovering=False),
        }
        self._id_scan_connections = {}

    def _id_update(self, kind, **fields):
        with self._application_lock:
            self._id_reset[kind].update(fields)

    def id_reset_snapshot(self, *, fresh=None, connection_generation=None):
        # Controller IO locks must be acquired before the application lock.
        if fresh is None:
            fresh = self._fresh_application_state()
        if connection_generation is None:
            connection_generation = self._controller.get_servo_abs_zero_generation_state()[-1]
        with self._application_lock:
            snapshot = deepcopy(self._id_reset)
        snapshot['motor']['available'] = fresh
        if self._id_scan_connections.get('motor') != (self.comm, connection_generation):
            snapshot['motor']['scanned'] = False
        return snapshot

    @staticmethod
    def _id_checked(result):
        if result['version'] != 3:
            raise ValueError('Unsupported ID maintenance protocol version')
        if result['status']:
            raise RuntimeError(f"Device rejected ID operation: status={result['status']}, stage={result['stage']}")
        return result

    _id_save = staticmethod(atomic_write_json)

    def _id_begin(self, comm, kind, session, cancelled):
        self._id_update(kind, message='Entering maintenance…')
        try:
            self._id_checked(comm.maintenance_request(session, BEGIN, cancelled=cancelled))
        except Exception as exc:
            self._id_update(kind, scanned=False, message=str(exc))
            try:
                comm.maintenance_request(session, END, timeout=1)
            except Exception:
                pass
            raise

    def request_id_scan(self):
        kind = 'motor'
        def scan(epoch):
            comm = self.comm
            session = int(uuid4().hex[:8], 16) or 1
            cancelled = lambda: epoch != self._application_epoch or comm is not self.comm
            self._id_update(kind, scanned=False, message='Checking maintenance support…')
            try:
                self._id_checked(comm.maintenance_request(0, CAPABILITY, cancelled=cancelled, timeout=2))
            except Exception as exc:
                self._id_update(kind, message=f'Maintenance handshake failed: {exc}. Check the connection and firmware version.')
                raise
            self._id_begin(comm, kind, session, cancelled)
            rows = []
            try:
                device = self._controller.servo_abs_zero_device_id(self.get_state_snapshot())
                path = DEVICE_STATE_DIR / f'id_reset_{device}.json'
                journal = json.loads(path.read_text(encoding='utf-8')) if device and path.exists() else {}
                recovering = bool(journal.get('inventory') and journal.get('status') != 'SUCCESS')
                for bus in range(MOTOR_BUS_COUNT):
                    for identifier in range(MOTOR_SCAN_ID_COUNT):
                        if cancelled():
                            raise InterruptedError('Scan cancelled')
                        reply = self._id_checked(comm.maintenance_request(session, PROBE, bus, identifier, cancelled=cancelled))
                        if reply['present']:
                            rows.append(dict(bus=bus, id=identifier))
                        self._id_update(kind, message=f'Scanning addresses {bus * MOTOR_SCAN_ID_COUNT + identifier + 1}/{MOTOR_BUS_COUNT * MOTOR_SCAN_ID_COUNT}; found {len(rows)} servos')
                # An interrupted transaction may have left a known servo at 253
                # or at a previously requested ID outside the ordinary scan.
                if recovering:
                    candidates = {(row['bus'], row['id']) for row in journal['inventory']}
                    candidates.update((bus, identifier) for bus, old, new in journal['moves'] for identifier in (old, new))
                    for bus, identifier in sorted(candidates):
                        if identifier < MOTOR_SCAN_ID_COUNT:
                            continue
                        reply = self._id_checked(comm.maintenance_request(session, PROBE, bus, identifier, cancelled=cancelled))
                        if reply['present']:
                            rows.append(dict(bus=bus, id=identifier))
                self._id_scan_connections[kind] = (comm, self._controller.get_servo_abs_zero_generation_state()[-1])
                message = f'Found {len(rows)} servos. Same-BUS duplicate IDs cannot be distinguished; connect them separately.'
                self._id_update(kind, recovering=recovering)
                if recovering:
                    recovery_targets = motor_recovery_targets(journal, rows)
                    for row in rows:
                        row['new_id'] = recovery_targets[(row['bus'],row['id'])]
                    message = 'Interrupted batch reconciled. Renumber All resumes the recorded sequence; inputs are locked.'
                self._id_update(kind, rows=rows, scanned=True, generation=self._id_reset[kind]['generation'] + 1, message=message)
                return True, message
            except Exception as exc:
                self._id_update(kind, message=str(exc))
                raise
            finally:
                try:
                    self._id_checked(comm.maintenance_request(session, END, timeout=1))
                except Exception as exc:
                    self._id_update(kind, scanned=False, message=f'Maintenance cleanup failed: {exc}; reconnect and scan again')
                    raise RuntimeError(f'Maintenance cleanup failed: {exc}') from exc
        return self.submit_operation('Scan All Buses', scan, requires_device=True)

    def request_motor_renumber(self, targets):
        from .application_actions import OperationResult
        state = self.id_reset_snapshot()['motor']
        if not state['available'] or not state['scanned']:
            return OperationResult(False, None, 'Connect P4 and scan all buses before renumbering')
        try:
            targets = dict(targets)
            device = self._controller.servo_abs_zero_device_id(self.get_state_snapshot())
            pending_path = DEVICE_STATE_DIR / f'id_reset_{device}.json'
            pending = json.loads(pending_path.read_text(encoding='utf-8')) if device and pending_path.exists() else {}
            recovering = bool(pending.get('inventory') and pending.get('status') != 'SUCCESS')
            if recovering:
                reconcile_motor_journal(pending, state['rows'])
                planned = [tuple(move) for move in pending['moves']]
            else:
                planned = plan_motor_ids(state['rows'], targets)
        except (ValueError, TypeError, OverflowError) as exc:
            return OperationResult(False, None, str(exc))
        if not state['rows']:
            return OperationResult(False, None, 'No motors were found')

        def renumber(epoch):
            from .calibration_snapshot import invalidate_calibration_snapshot_scope
            from .motor_mapping import MOTOR_TO_JOINT_INDEX
            controller = self._controller
            device = controller.servo_abs_zero_device_id(self.get_state_snapshot())
            if not device:
                return False, 'P4 identity is unavailable; wait for device fingerprint'
            comm = self.comm
            session = int(uuid4().hex[:8], 16) or 1
            cancelled = lambda: epoch != self._application_epoch or self.comm is not comm
            journal_path = DEVICE_STATE_DIR / f'id_reset_{device}.json'
            journal_path.parent.mkdir(parents=True, exist_ok=True)
            journal = deepcopy(pending) if recovering else dict(device=device, inventory=state['rows'], moves=planned, completed=0, status='RUNNING')
            if recovering:
                journal['completed'] = reconcile_motor_journal(journal, state['rows'])

            def save():
                atomic_write_json(journal_path, journal)

            self._id_begin(comm, 'motor', session, cancelled)
            started = recovering
            try:
                # Recheck the whole inventory immediately before EEPROM writes.
                for row in state['rows']:
                    probe = self._id_checked(comm.maintenance_request(session, PROBE, row['bus'], row['id'], cancelled=cancelled))
                    if not probe['present']:
                        raise ValueError('Inventory changed; scan all buses again')
                for bus in range(4):
                    if not recovering and self._id_checked(comm.maintenance_request(session, PROBE, bus, 253, cancelled=cancelled))['present']:
                        raise ValueError('Temporary ID 253 is occupied')
                # High target IDs are allowed, but are outside the quick scan.
                # Check those exact IDs on all buses before the first write.
                inventory = {(row['bus'], row['id']) for row in state['rows']}
                high_targets = {new for _, _, new in planned if MOTOR_SCAN_ID_COUNT <= new < 253}
                for identifier in sorted(high_targets):
                    for bus in range(MOTOR_BUS_COUNT):
                        if (bus, identifier) in inventory:
                            continue
                        reply = self._id_checked(comm.maintenance_request(session, PROBE, bus, identifier, cancelled=cancelled))
                        if reply['present']:
                            raise ValueError(f'Target ID {identifier} is occupied on BUS{bus}')
                affected = sum(1 << identifier for identifier in {i for _, old, new in planned for i in (old, new) if i < 27})
                joints = sum(1 << j for j in {MOTOR_TO_JOINT_INDEX[m] for m in range(27) if affected & (1 << m)})
                if affected:
                    invalidate_calibration_snapshot_scope(controller.calibration_snapshot_path, motor_mask=affected,
                                                          joint_mask=joints, reason='servo_id_reset')
                    controller._invalidate_coordinate_anchors(affected, 'servo_id_reset', self.get_state_snapshot())
                    with self._application_lock:
                        for joint in range(21):
                            if joints & (1 << joint):
                                self._motor_transactions[joint] = 'PENDING'
                save()
                started = True
                for index, (bus, old, new) in enumerate(planned):
                    if index < journal['completed']:
                        continue
                    if cancelled():
                        raise InterruptedError('Renumbering cancelled; scan current addresses')
                    journal['intent'] = [bus, old, new]; save()
                    self._id_update('motor', message=f'Renumbering step {index + 1}/{len(planned)}: BUS{bus} {old} → {new}')
                    try:
                        result = comm.maintenance_request(session, RENAME, bus, old, new, cancelled=cancelled)
                        self._id_checked(result)
                    except Exception:
                        # Never resend a write: locate every candidate address.
                        locations = {}
                        for candidate in {old, new, 253}:
                            try:
                                locations[candidate] = self._id_checked(comm.maintenance_request(session, PROBE, bus, candidate, timeout=1))['present']
                            except Exception:
                                locations[candidate] = None
                        journal['observed'] = locations
                        raise
                    journal['completed'] = index + 1; journal.pop('intent', None); save()
                current = {(r['bus'], r['id']) for r in journal['inventory']}
                for bus, old, new in planned:
                    current.remove((bus, old)); current.add((bus, new))
                verified = []
                for bus, identifier in sorted(current):
                    if not self._id_checked(comm.maintenance_request(session, PROBE, bus, identifier, cancelled=cancelled))['present']:
                        raise ValueError('Post-renumber scan failed; inspect current addresses')
                    verified.append(dict(bus=bus, id=identifier))
                journal['status'] = 'SUCCESS'; save()
                self._id_update('motor', rows=verified, generation=state['generation'] + 1,
                                message='Motor IDs verified. Repeat motor calibration after any ID change.', scanned=True, recovering=False)
                return True, 'Motor IDs verified'
            except Exception as exc:
                if started:
                    journal['status'] = 'NEEDS_RECOVERY'; journal['error'] = str(exc); save()
                self._id_update('motor', scanned=False, message=f'{exc}. Last verified steps: {journal["completed"]}/{len(planned)}')
                return False, str(exc)
            finally:
                try:
                    self._id_checked(comm.maintenance_request(session, END, timeout=1))
                    from .protocol import STATE_SYNC_ALL
                    self.request_state_sync(STATE_SYNC_ALL)
                except Exception:
                    self._id_update('motor', scanned=False, message='P4 maintenance cleanup failed; reconnect and scan current IDs.')
                    raise
        return self.submit_operation('Renumber All', renumber)
