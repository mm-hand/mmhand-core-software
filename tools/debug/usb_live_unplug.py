"""Observe one real unplug during active P4 RX, then verify clean reconnection.

No reset, flash, STOP, torque enable or motion target is sent. Requires an
already calibrated/disabled P4 and a GUI with its device links disconnected.
"""
import argparse
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from serial.tools import list_ports
from desktop.core.hand_backend import HandBackend
from desktop.core.session_log import session_log_since


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--expected-manual', type=lambda v: int(v, 0))
    parser.add_argument('--expected-auto', type=lambda v: int(v, 0))
    args = parser.parse_args()
    if (args.expected_manual is None) != (args.expected_auto is None):
        parser.error('provide both expected calibration masks, or neither')
    args.output.mkdir(parents=True, exist_ok=False)
    log = (args.output / 'observations.jsonl').open('w', encoding='utf-8', buffering=1)
    hand = HandBackend(); hand._save_data = False; hand.open_application()

    def record(kind, **values):
        line = json.dumps(dict(time=time.time(), kind=kind, **values), ensure_ascii=False)
        print(line, flush=True); log.write(line + '\n')

    def port():
        return next((p.device for p in list_ports.comports()
                     if p.vid == 0x303a and p.pid == 0x1001 and p.serial_number == args.serial), None)

    def invariant():
        state = hand.get_state_snapshot()
        if (state.control_enabled or state.active_joint_mask or state.active_motor_mask
                or hand._selected_joints or hand._selected_motors):
            raise RuntimeError('non-motion invariant violated')
        return state

    def connect(device):
        request = hand.request_connect(device)
        if not request.accepted:
            raise RuntimeError(request.message)
        end = time.monotonic() + 25
        while time.monotonic() < end:
            operation = next(o for o in hand._application_operations if o['operation_id'] == request.operation_id)
            if operation['status'] == 'FAILED':
                raise RuntimeError(operation['message'])
            if operation['status'] == 'SUCCESS' and hand._control_parameter_sync.startswith('APPLIED'):
                break
            time.sleep(.05)
        else:
            raise RuntimeError('connection did not complete')
        end = time.monotonic() + 12
        state = invariant()
        if args.expected_manual is not None:
            while time.monotonic() < end:
                state = invariant()
                if (state.manual_reference_valid_bitmap == args.expected_manual
                        and state.auto_calibration_valid_bitmap == args.expected_auto):
                    break
                time.sleep(.05)
            else:
                raise RuntimeError('expected calibration masks were not read back')
        if not state.protocol_compatible or not hand._fresh_application_state():
            raise RuntimeError('no fresh valid protocol')
        return state

    def wait_presence(present):
        end = time.monotonic() + 900
        while time.monotonic() < end:
            invariant()
            found = port()
            if bool(found) == present:
                return found
            time.sleep(.05)
        raise RuntimeError('physical action/device enumeration timed out')

    try:
        state = connect(wait_presence(True))
        expected = (args.expected_manual, args.expected_auto)
        comm = hand.comm
        record('READY_UNPLUG_DURING_RX', port=port(), connection=comm.connection_diagnostics())
        wait_presence(False)
        record('REMOVED', connection=comm.connection_diagnostics())
        started = time.monotonic()
        hand.disconnect(stop_lower=False)
        elapsed = time.monotonic() - started
        diag = comm.connection_diagnostics()
        if elapsed > 5 or diag['rx_thread_alive'] or diag['tx_thread_alive']:
            raise RuntimeError(f'serial threads did not close promptly: {elapsed}s {diag}')
        record('THREADS_CLOSED', seconds=elapsed, connection=diag)
        state = connect(wait_presence(True))
        end = time.monotonic() + 12
        while time.monotonic() < end:
            state = invariant()
            if not hand._fresh_application_state():
                raise RuntimeError('feedback stale after live unplug recovery')
            if (args.expected_manual is not None and
                    (state.manual_reference_valid_bitmap, state.auto_calibration_valid_bitmap) != expected):
                raise RuntimeError('calibration changed during live unplug')
            time.sleep(.1)
        record('PASSED', calibration_verified=args.expected_manual is not None,
               expected_manual=expected[0], expected_auto=expected[1],
               connection=hand.comm.connection_diagnostics())
        return 0
    except Exception as exc:
        record('FAILED', error=f'{type(exc).__name__}: {exc}', session=session_log_since()['lines'])
        return 1
    finally:
        hand.disconnect(stop_lower=False); hand.close_application(); log.close()


if __name__ == '__main__':
    raise SystemExit(main())
