"""Observe real user unplug/cold-start cycles; never resets or enables motors.

Start with the GUI disconnected. Wait for READY before the first physical
action, and for CYCLE_PASSED before the next. Each cold-start must remove
ALL P4 power sources. Calibration is restored from the verified device JSON.
"""
import argparse
import json
from pathlib import Path
import re
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from serial.tools import list_ports
from desktop.core.hand_backend import HandBackend
from desktop.core.session_log import session_log_since


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial', required=True, help='verified native USB adapter serial')
    parser.add_argument('--cycles', type=int, required=True)
    parser.add_argument('--mode', choices=['unplug', 'cold'], required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.cycles < 1:
        parser.error('cycles must be positive')
    args.output.mkdir(parents=True, exist_ok=False)
    log = (args.output / 'physical.jsonl').open('w', encoding='utf-8', buffering=1)
    hand = HandBackend(); hand._save_data = False; hand.open_application()

    def record(kind, **values):
        row = dict(time=time.time(), kind=kind, **values)
        text = json.dumps(row, ensure_ascii=False)
        log.write(text + '\n'); print(text, flush=True)

    def port():
        return next((p.device for p in list_ports.comports()
                     if p.vid == 0x303a and p.pid == 0x1001 and p.serial_number == args.serial), None)

    def wait_presence(present):
        end = time.monotonic() + 900
        while time.monotonic() < end:
            found = port()
            if bool(found) == present:
                return found
            time.sleep(.1)
        raise RuntimeError('physical action/device enumeration timed out after15min')

    def connect_and_restore(device):
        request = hand.request_connect(device)
        if not request.accepted:
            raise RuntimeError(request.message)
        end = time.monotonic() + 25
        while time.monotonic() < end:
            operation = next(o for o in hand._application_operations if o['operation_id'] == request.operation_id)
            if operation['status'] in ('SUCCESS', 'FAILED'):
                if operation['status'] == 'FAILED':
                    raise RuntimeError(operation['message'])
                break
            time.sleep(.05)
        else:
            raise RuntimeError('connect did not finish')
        snapshot = hand._calibration_workflow.inspect_snapshot(Path(hand.calibration_storage_paths()['current']))
        ok, message = hand._controller.load_calibration_snapshot(snapshot)
        if not ok:
            raise RuntimeError(message)
        end = time.monotonic() + 12
        while time.monotonic() < end:
            state = hand.get_state_snapshot()
            if state.control_enabled or state.active_joint_mask or state.active_motor_mask:
                raise RuntimeError('non-motion invariant violated')
            if not hand._fresh_application_state():
                raise RuntimeError('feedback lost after physical reconnect')
            time.sleep(.1)
        return state

    try:
        device = wait_presence(True)
        connect_and_restore(device)
        hand.disconnect(stop_lower=False)
        record('READY', mode=args.mode, cycles=args.cycles, serial=args.serial)
        for cycle in range(1, args.cycles + 1):
            wait_presence(False)
            record('REMOVED', cycle=cycle)
            sequence = session_log_since()['sequence']
            device = wait_presence(True)
            state = connect_and_restore(device)
            firmware = [s for s in session_log_since(sequence)['lines'] if '[USB_' in s]
            if args.mode == 'cold' and not any(re.search(r'\breset=1\b', s) for s in firmware if '[USB_BOOT]' in s):
                raise RuntimeError('cold-start was not confirmed by POWERON reset diagnostic')
            record('CYCLE_PASSED', cycle=cycle, port=device,
                   manual=state.manual_reference_valid_bitmap, auto=state.auto_calibration_valid_bitmap,
                   firmware=firmware)
            hand.disconnect(stop_lower=False)
        record('PASSED', mode=args.mode, cycles=args.cycles)
        return 0
    except Exception as exc:
        record('FAILED', error=f'{type(exc).__name__}: {exc}', session=session_log_since()['lines'])
        return 1
    finally:
        hand.disconnect(stop_lower=False); hand.close_application(); log.close()


if __name__ == '__main__':
    raise SystemExit(main())
