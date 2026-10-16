"""Five-finger SPI inventory and journaled, serial tactile ID transactions.

职责：扫描五指触觉并编排逐台改号和恢复。
线程：应用操作工作线程。
输入：S3维护结果、逐台目标及USB身份。
输出：逐步改号请求和恢复进度。
状态所有权：拥有维护草稿和按设备保存的日志。
禁止依赖：禁止直接SPI操作或P4触觉维护。
"""
from copy import deepcopy
import json
from desktop.paths import DEVICE_STATE_DIR
from .id_protocol import reconcile_motor_journal, motor_recovery_targets
from .tactile_maintenance import SCAN, RENUMBER, RECONCILE, plan_tactile_ids

FINGER_NAMES = ('Little', 'Ring', 'Middle', 'Index', 'Thumb')


class TactileIdActions:
    def _tactile_device(self, comm):
        from serial.tools import list_ports
        serial = getattr(comm, 'serial_number', '') or next(
            (port.serial_number for port in list_ports.comports() if port.device == comm.port), '')
        if not serial:
            raise ValueError('S3 USB identity unavailable; reconnect before writing sensor IDs')
        return ''.join(character for character in serial if character.isalnum()).upper()

    def _tactile_scan_one(self, comm, selector, cancelled):
        result = comm.maintenance_request(SCAN, selector, cancelled=cancelled)
        if result['status'] == 1: return None
        if result['status'] not in (0, 3):
            raise ValueError(f"S3 scan failed at selector {selector}: {result['status_name']}")
        return dict(bus=selector // 3, id=selector % 3, selector=selector,
                    model=result['model'], points=result['points'], semantic=result.get('semantic', 255))

    def request_tactile_scan(self):
        from .application_actions import OperationResult
        comm = getattr(self, 'tactile_comm', None)
        if comm is None or not comm.is_connected():
            return OperationResult(False, None, 'Connect S3 directly by USB before scanning')

        def scan(epoch):
            cancelled = lambda: epoch != self._application_epoch or comm is not self.tactile_comm
            self._id_update('tactile', scanned=False, recovering=False, message='Scanning all five fingers…')
            rows = []
            for selector in range(15):
                if cancelled(): raise InterruptedError('Tactile scan cancelled')
                row = self._tactile_scan_one(comm, selector, cancelled)
                if row: rows.append(row)
                self._id_update('tactile', message=f'Scanning {selector + 1}/15 tactile addresses; found {len(rows)}')
            device = self._tactile_device(comm)
            path = DEVICE_STATE_DIR / f'tactile_id_reset_{device}.json'
            journal = json.loads(path.read_text(encoding='utf-8')) if path.exists() else {}
            recovering = bool(journal.get('inventory') and journal.get('status') != 'SUCCESS')
            message = f'Found {len(rows)} tactile sensors across five fingers.'
            if recovering:
                intent = journal.get('intent')
                if intent:
                    finger, old, new = intent
                    installed = deepcopy(journal['inventory'])
                    for bus, source, destination in journal['moves'][:journal.get('completed', 0)]:
                        previous = next(row for row in installed if (row['bus'], row['id']) == (bus, source))
                        previous['id'] = destination
                    original = next(row for row in installed if (row['bus'], row['id']) == (finger, old))
                    destination = next((row for row in rows if (row['bus'], row['id']) == (finger, new)), None)
                    old_present = any((row['bus'], row['id']) == (finger, old) for row in rows)
                    if not old_present and destination and any(destination.get(key) != original.get(key) for key in ('model', 'points', 'semantic')):
                        result = comm.maintenance_request(RECONCILE, finger * 3 + new,
                            original['model'], original['semantic'] % 3, cancelled=cancelled)
                        journal['route_reconciliation'] = result; self._id_save(path, journal)
                        if result['status'] != 0 or not result['new_valid']:
                            raise ValueError('Interrupted tactile model/route could not be verified; no ID write repeated')
                        destination.update(model=original['model'], points=original['points'], semantic=original['semantic'])
                desired = motor_recovery_targets(journal, rows)
                for row in rows: row['new_id'] = desired[(row['bus'], row['id'])]
                message += ' Interrupted batch reconciled; Renumber All continues from verified steps.'
            unknown = sum(row['model'] not in (1, 2) or row['points'] not in (25, 52) for row in rows)
            if unknown: message += f' {unknown} unknown-model response(s); those sensors cannot be renamed.'
            self._id_scan_connections['tactile'] = comm
            self._id_update('tactile', rows=rows, generation=self._id_reset['tactile']['generation'] + 1,
                            scanned=True, recovering=recovering, message=message)
            return True, message
        return self.submit_operation('Scan All Tactile Sensors', scan, requires_device=False)

    def request_tactile_renumber(self, targets):
        from .application_actions import OperationResult
        state = self.id_reset_snapshot()['tactile']
        comm = getattr(self, 'tactile_comm', None)
        if not state['available'] or not state['scanned'] or comm is not self._id_scan_connections.get('tactile'):
            return OperationResult(False, None, 'Connect S3 directly and scan all fingers first')
        try:
            device = self._tactile_device(comm)
            path = DEVICE_STATE_DIR / f'tactile_id_reset_{device}.json'
            previous = json.loads(path.read_text(encoding='utf-8')) if path.exists() else {}
            recovering = bool(previous.get('inventory') and previous.get('status') != 'SUCCESS')
            if recovering:
                completed = reconcile_motor_journal(previous, state['rows'])
                moves = [tuple(move) for move in previous['moves']]
            else:
                completed = 0
                for row in state['rows']:
                    value = targets.get((row['bus'], row['id']))
                    if value not in (None, '', row['id']) and (row['model'], row['points']) not in ((1,52),(2,25)):
                        raise ValueError('Unknown tactile model; refusing the entire batch before any write')
                moves = plan_tactile_ids(state['rows'], dict(targets))
            if not moves:
                return OperationResult(True, None, 'All tactile IDs are unchanged; no write needed')
        except (ValueError, TypeError, OverflowError, OSError) as exc:
            return OperationResult(False, None, str(exc))

        def renumber(epoch):
            cancelled = lambda: epoch != self._application_epoch or comm is not self.tactile_comm
            if cancelled() or self._tactile_device(comm) != device:
                return False, 'S3 connection changed; scan again'
            journal = deepcopy(previous) if recovering else dict(device=device, inventory=state['rows'], moves=moves, completed=0, status='RUNNING')
            journal['completed'] = completed
            rows = deepcopy(state['rows'])
            started = recovering
            self._id_update('tactile', scanned=False, last_result='', message='Checking tactile inventory before writing…')
            try:
                for row in rows:
                    current = self._tactile_scan_one(comm, row['selector'], cancelled)
                    if not current or (current['model'], current['points']) != (row['model'], row['points']):
                        raise ValueError('Tactile inventory or model changed; scan again')
                self._id_save(path, journal)
                started = True
                for index, (finger, old, new) in enumerate(moves):
                    if index < completed: continue
                    if cancelled(): raise InterruptedError('Tactile renumber cancelled; scan actual IDs')
                    row = next(row for row in rows if (row['bus'], row['id']) == (finger, old))
                    if (row['model'], row['points']) not in ((1, 52), (2, 25)):
                        raise ValueError('Unknown tactile model; refusing ID write')
                    journal['intent'] = [finger, old, new]; self._id_save(path, journal)
                    self._id_update('tactile', message=f'{FINGER_NAMES[finger]}: {old} → {new}, step {index + 1}/{len(moves)}')
                    result = comm.maintenance_request(RENUMBER, finger * 3 + old, row['model'], new,
                        timeout=4, cancelled=cancelled)
                    journal['observed'] = result; self._id_save(path, journal)
                    if result['status'] != 0 or not result['new_valid'] or result['old_valid']:
                        raise ValueError(f"{FINGER_NAMES[finger]} {old}→{new}: {result['status_name']}; write_sent={result['write_sent']}, old_valid={result['old_valid']}, new_valid={result['new_valid']}")
                    row.update(id=new, selector=finger * 3 + new)
                    journal['completed'] = index + 1; journal.pop('intent', None); self._id_save(path, journal)
                for row in rows:
                    current = self._tactile_scan_one(comm, row['selector'], cancelled)
                    if not current or (current['model'], current['points']) != (row['model'], row['points']):
                        raise ValueError('Final tactile ID/model readback failed')
                journal['status'] = 'SUCCESS'; self._id_save(path, journal)
                message = 'All requested tactile IDs verified. Power-cycle the sensors and scan again to check persistence.'
                self._id_update('tactile', rows=rows, generation=state['generation'] + 1, scanned=True,
                                recovering=False, message='Tactile IDs verified.', last_result=message)
                return True, message
            except Exception as exc:
                if started:
                    journal['status'] = 'NEEDS_RECOVERY'; journal['error'] = str(exc); self._id_save(path, journal)
                message = f'{exc}. Stopped after {journal["completed"]}/{len(moves)} verified steps; scan before continuing. No write is retried blindly.'
                self._id_update('tactile', scanned=False, message=message, last_result=message)
                return False, message
        return self.submit_operation('Renumber All Tactile Sensors', renumber, requires_device=False)
