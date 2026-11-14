"""Assembly ID transactions owned by HandBackend, never by the views.

职责：编排电机扫描和批量改号。
线程：应用操作工作线程。
输入：P4维护响应、用户编号和设备身份。
输出：逐步维护请求及只读进度。
状态所有权：拥有当前扫描结果。
禁止依赖：禁止直接访问UART或UI控件。
"""
from copy import deepcopy
from uuid import uuid4
from .id_protocol import (CAPABILITY, BEGIN, END, PROBE, RENAME, BUZZ,
                          TEMPORARY_MOTOR_ID, plan_motor_ids)

MOTOR_BUS_COUNT = 4
RESCAN = 'Scan (0–253) to check current motor IDs.'
BUZZ_AMPLITUDE = 128
# P4 status -> explanation; 0 never reaches this table.
BUZZ_ERRORS = {
    2: 'Controller refused the move; check for a latched STOP or an active control owner',
    3: 'P4 firmware does not support motor buzz; flash the matching firmware',
    6: 'Maintenance session expired; retry',
    7: 'BUS{bus} ID{ident} did not answer or could not move; check the bus, power and obstruction',
}


class IdResetActions:
    def _init_id_reset(self):
        self._motor_scan = dict(rows=[], message='Connect P4 and scan all buses.', generation=0, scanned=False)
        self._motor_scan_connection = None

    def _id_update(self, **fields):
        with self._application_lock:
            self._motor_scan.update(fields)

    def id_reset_snapshot(self, *, fresh=None, connection_generation=None):
        # Controller IO locks must be acquired before the application lock.
        if fresh is None:
            fresh = self._fresh_application_state()
        if connection_generation is None:
            connection_generation = self._controller.get_servo_abs_zero_generation_state()[-1]
        with self._application_lock:
            motor = deepcopy(self._motor_scan)
            if self._motor_scan_connection != (self.comm, connection_generation):
                motor['scanned'] = False
        motor['available'] = fresh
        return {'motor': motor}

    @staticmethod
    def _id_checked(result):
        if result['version'] != 3:
            raise ValueError('Unsupported ID maintenance protocol version')
        if result['status']:
            raise RuntimeError(f"Device rejected ID operation: status={result['status']}, stage={result['stage']}")
        return result

    def _id_begin(self, comm, session, cancelled):
        self._id_update(message='Entering maintenance…')
        try:
            self._id_checked(comm.maintenance_request(session, BEGIN, cancelled=cancelled))
        except Exception as exc:
            self._id_update(scanned=False, message=str(exc))
            try:
                comm.maintenance_request(session, END, timeout=1)
            except Exception:
                pass
            raise

    def request_id_scan(self, max_id=30):
        from .application_actions import OperationResult
        if type(max_id) is not int or max_id not in (30, TEMPORARY_MOTOR_ID):
            return OperationResult(False, None, 'Choose Scan (0–30) or Scan (0–253)')
        address_count = max_id + 1

        def scan(epoch):
            comm = self.comm
            connection_generation = self._controller.get_servo_abs_zero_generation_state()[-1]
            session = int(uuid4().hex[:8], 16) or 1
            cancelled = lambda: epoch != self._application_epoch or comm is not self.comm
            self._id_update(scanned=False, message='Checking maintenance support…')
            try:
                self._id_checked(comm.maintenance_request(0, CAPABILITY, cancelled=cancelled, timeout=2))
            except Exception as exc:
                self._id_update(message=f'Maintenance handshake failed: {exc}. Check the connection and firmware version.')
                raise
            self._id_begin(comm, session, cancelled)
            rows = []
            try:
                for bus in range(MOTOR_BUS_COUNT):
                    for identifier in range(address_count):
                        if cancelled():
                            raise InterruptedError('Scan cancelled')
                        reply = self._id_checked(comm.maintenance_request(session, PROBE, bus, identifier, cancelled=cancelled))
                        if reply['present']:
                            rows.append(dict(bus=bus, id=identifier))
                        checked = bus * address_count + identifier + 1
                        self._id_update(message=f'Scanning addresses {checked}/{MOTOR_BUS_COUNT * address_count}; found {len(rows)} servos')
            except Exception as exc:
                self._id_update(message=str(exc))
                raise
            finally:
                try:
                    self._id_checked(comm.maintenance_request(session, END, timeout=1))
                except Exception as exc:
                    self._id_update(message=f'Maintenance cleanup failed: {exc}; reconnect and scan again')
                    raise RuntimeError(f'Maintenance cleanup failed: {exc}') from exc
            message = f'Found {len(rows)} servos.'
            with self._application_lock:
                if cancelled():
                    return False, 'Scan cancelled'
                self._motor_scan_connection = (comm, connection_generation)
                self._motor_scan.update(rows=rows, scanned=True,
                                        generation=self._motor_scan['generation'] + 1, message=message)
            return True, message
        return self.submit_operation(f'Scan (0–{max_id})', scan)

    def request_motor_renumber(self, targets):
        from .application_actions import OperationResult
        state = self.id_reset_snapshot()['motor']
        if not state['available'] or not state['scanned']:
            return OperationResult(False, None, 'Connect P4 and scan all buses before renumbering')
        try:
            targets = dict(targets)
            planned = plan_motor_ids(state['rows'], targets)
        except (ValueError, TypeError, OverflowError) as exc:
            return OperationResult(False, None, str(exc))
        if not state['rows']:
            return OperationResult(False, None, 'No motors were found')

        def renumber(epoch):
            from .calibration_snapshot import invalidate_calibration_snapshot_scope
            from .motor_mapping import MOTOR_TO_JOINT_INDEX
            controller = self._controller
            comm = self.comm
            current_scan = self.id_reset_snapshot()['motor']
            if not current_scan['scanned'] or current_scan['generation'] != state['generation']:
                return False, f'The scan is no longer current. {RESCAN}'
            if not controller.servo_abs_zero_device_id(self.get_state_snapshot()):
                return False, 'P4 identity is unavailable; wait for device fingerprint'
            session = int(uuid4().hex[:8], 16) or 1
            cancelled = lambda: epoch != self._application_epoch or self.comm is not comm

            def present(bus, identifier):
                reply = comm.maintenance_request(session, PROBE, bus, identifier, cancelled=cancelled)
                return self._id_checked(reply)['present']

            self._id_begin(comm, session, cancelled)
            failure = ''
            try:
                changed = {(bus, old) for (bus, old), new in targets.items() if old != new}
                for bus, old in sorted(changed):
                    if not present(bus, old):
                        raise ValueError(f'BUS{bus} ID{old} did not answer')
                # Final IDs are unique across buses. Temporary IDs are local to a bus.
                planned_destinations = {(bus, new) for bus, _, new in planned}
                final_ids = {new for _, new in planned_destinations if new != TEMPORARY_MOTOR_ID}
                checks = {(bus, new) for bus in range(MOTOR_BUS_COUNT) for new in final_ids}
                checks.update((bus, new) for bus, new in planned_destinations if new == TEMPORARY_MOTOR_ID)
                for bus, identifier in sorted(checks - targets.keys()):
                    if present(bus, identifier):
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
                for index, (bus, old, new) in enumerate(planned, 1):
                    step = f'BUS{bus} ID{old} → ID{new}'
                    self._id_update(message=f'Renumbering step {index}/{len(planned)}: {step}')
                    try:
                        if cancelled():
                            raise InterruptedError('Renumbering cancelled')
                        result = comm.maintenance_request(session, RENAME, bus, old, new, cancelled=cancelled)
                        self._id_checked(result)
                    except Exception as exc:
                        raise RuntimeError(f'{step}: {exc}') from exc

                verified = []
                missing = []
                for bus, identifier in sorted((bus, int(new)) for (bus, _), new in targets.items()):
                    if not present(bus, identifier):
                        if (bus, identifier) in planned_destinations:
                            raise ValueError(f'Post-renumber check failed: BUS{bus} ID{identifier} did not answer')
                        missing.append(f'BUS{bus} ID{identifier}')
                        continue
                    verified.append(dict(bus=bus, id=identifier))
                message = 'Motor IDs verified.'
                if missing:
                    message += f' Unchanged motors did not answer: {", ".join(missing)}.'
                self._id_update(rows=verified, generation=state['generation'] + 1,
                                message=f'{message} Repeat motor calibration after any ID change.', scanned=True)
                return True, message
            except Exception as exc:
                failure = f'{exc}. {RESCAN}'
                self._id_update(scanned=False, message=failure)
                return False, failure
            finally:
                try:
                    self._id_checked(comm.maintenance_request(session, END, timeout=1))
                    from .protocol import STATE_SYNC_ALL
                    self.request_state_sync(STATE_SYNC_ALL)
                except Exception as exc:
                    message = f'P4 maintenance cleanup failed: {exc}. Reconnect and scan current IDs.'
                    if failure:
                        message = f'{failure} {message}'
                    self._id_update(scanned=False, message=message)
                    raise RuntimeError(message) from exc
        return self.submit_operation('Renumber All', renumber)

    def request_motor_buzz(self, bus, identifier, *, held=None):
        """Run one pulse, or repeat while the caller renews its hold."""
        from .application_actions import OperationResult
        state = self.id_reset_snapshot()['motor']
        if not state['available'] or not state['scanned']:
            return OperationResult(False, None, 'Connect P4 and scan all buses before buzzing a motor')
        bus, identifier = int(bus), int(identifier)
        if dict(bus=bus, id=identifier) not in state['rows']:
            return OperationResult(False, None, 'Scan all buses and select a motor to buzz')
        motor = f'BUS{bus} ID{identifier}'

        def buzz(epoch):
            if held is not None and not held():
                return True, f'Buzz released on {motor}'
            switched, message = self._prepare_maintenance_switch(epoch)
            if not switched:
                return False, message
            comm = self.comm
            session = int(uuid4().hex[:8], 16) or 1
            cancelled = lambda: epoch != self._application_epoch or comm is not self.comm
            self._id_begin(comm, session, cancelled)
            try:
                self._id_update(message=f'Buzzing {motor}…')
                while held is None or held():
                    if cancelled():
                        raise InterruptedError('Motor buzz cancelled')
                    result = comm.maintenance_request(
                        session, BUZZ, bus, identifier, BUZZ_AMPLITUDE, cancelled=cancelled)
                    if result['version'] != 3:
                        raise ValueError('Unsupported ID maintenance protocol version')
                    if result['status']:
                        text = BUZZ_ERRORS.get(result['status'],
                                               f'P4 refused the move (status={result["status"]})')
                        raise RuntimeError(text.format(bus=bus, ident=identifier))
                    if held is None:
                        break
                message = f'Buzz finished on {motor}'
                self._id_update(message=message)
                return True, message
            except Exception as exc:
                self._id_update(message=str(exc))
                return False, str(exc)
            finally:
                try:
                    self._id_checked(comm.maintenance_request(session, END, timeout=1))
                except Exception as exc:
                    self._id_update(scanned=False,
                                    message=f'Maintenance cleanup failed: {exc}; reconnect and scan again')
                    raise RuntimeError(f'Maintenance cleanup failed: {exc}') from exc
        return self.submit_operation('Buzz Motor', buzz)
