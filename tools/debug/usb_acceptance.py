"""Non-motion P4 reconnect/JTAG/soak acceptance. Never resets or flashes a board.

Close the GUI's device connections first. The script stops on the FIRST failure
and preserves observations. Physical unplug/cold-start tests are manual and are
not simulated or counted here. Default:100 reconnects,10 JTAGs,2h soak.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from desktop.core.hand_backend import HandBackend
from desktop.core.session_log import session_log_since
from serial.tools import list_ports
import serial
from desktop.core.communication.serial_transport import SerialTransport


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--cycles', type=int, default=100)
    parser.add_argument('--jtag-cycles', type=int, default=10)
    parser.add_argument('--soak-seconds', type=float, default=7200)
    parser.add_argument('--backpressure-seconds', type=float, default=20)
    parser.add_argument('--elf', type=Path)
    parser.add_argument('--expected-manual', type=lambda v: int(v, 0))
    parser.add_argument('--expected-auto', type=lambda v: int(v, 0))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if min(args.cycles, args.jtag_cycles, args.soak_seconds, args.backpressure_seconds) < 0:
        parser.error('counts and duration must be nonnegative')
    args.output.mkdir(parents=True, exist_ok=False)
    events = (args.output / 'observations.jsonl').open('w', encoding='utf-8', buffering=1)
    hand = HandBackend()
    hand._save_data = False
    hand.open_application()
    sequence = 0

    def record(kind, **values):
        row = dict(time=time.time(), kind=kind, **values)
        events.write(json.dumps(row, ensure_ascii=False) + '\n')
        if kind != 'sample':
            print(json.dumps(row, ensure_ascii=False), flush=True)

    def wait_operation(request):
        if not request.accepted:
            raise RuntimeError(request.message)
        end = time.monotonic() + 25
        while time.monotonic() < end:
            operation = next(o for o in hand._application_operations
                             if o['operation_id'] == request.operation_id)
            if operation['status'] in ('SUCCESS', 'FAILED'):
                if operation['status'] != 'SUCCESS':
                    raise RuntimeError(operation['message'])
                return
            time.sleep(.02)
        raise RuntimeError('operation did not finish within25s')

    def observe():
        nonlocal sequence
        state = hand.get_state_snapshot()
        if state.control_enabled or state.active_joint_mask or state.active_motor_mask:
            raise RuntimeError('non-motion invariant violated: control or scope active')
        if hand._selected_joints or hand._selected_motors:
            raise RuntimeError('selection unexpectedly nonempty')
        if not hand._fresh_application_state() or not state.protocol_compatible:
            raise RuntimeError('status missing/stale or handshake rejected')
        if args.expected_manual is not None and state.manual_reference_valid_bitmap != args.expected_manual:
            raise RuntimeError('joint calibration bitmap changed')
        if args.expected_auto is not None and state.auto_calibration_valid_bitmap != args.expected_auto:
            raise RuntimeError('motor calibration bitmap changed')
        log = session_log_since(sequence)
        sequence = log['sequence']
        diagnostics = [line for line in log['lines']
                       if any(k in line for k in ('[USB_', '[COMM]', '[APP]', '[SYNC]'))]
        record('sample', p4_ms=state.p4_time_ms, manual=state.manual_reference_valid_bitmap,
               auto=state.auto_calibration_valid_bitmap,
               connection=hand.comm.connection_diagnostics(), firmware=diagnostics)

    def connect():
        wait_operation(hand.request_connect(args.port))
        end = time.monotonic() + 12
        while time.monotonic() < end:
            state = hand.get_state_snapshot()
            masks_ready = ((args.expected_manual is None or state.manual_reference_valid_bitmap == args.expected_manual)
                           and (args.expected_auto is None or state.auto_calibration_valid_bitmap == args.expected_auto))
            if hand._control_parameter_sync.startswith('APPLIED') and masks_ready:
                break
            time.sleep(.05)
        else:
            raise RuntimeError('control parameters or expected calibration masks were not verified')
        observe()

    def disconnect():
        # Already verified disabled/empty: no reset/STOP command is needed.
        hand.disconnect(stop_lower=False)

    try:
        record('start', arguments=vars(args) | {'output': str(args.output),
                    'elf': str(args.elf) if args.elf else None},
               ports=[dict(port=p.device, vid=p.vid, pid=p.pid, serial=p.serial_number)
                      for p in list_ports.comports()])
        for index in range(args.cycles):
            connect(); disconnect()
            if index % 10 == 0 or index == args.cycles - 1:
                record('reconnect', passed=index + 1)
        for index in range(args.jtag_cycles):
            command = [sys.executable, str(ROOT / 'tools/debug/gdb_batch.py'),
                       '--target', 'p4', '--threads', '--timeout', '30']
            if args.elf:
                command.extend(['--elf', str(args.elf)])
            done = subprocess.run(command, cwd=ROOT, capture_output=True, text=True,
                                  errors='replace', timeout=80)
            (args.output / f'jtag-{index + 1}.log').write_text(
                done.stdout + done.stderr, encoding='utf-8')
            if done.returncode != 0:
                raise RuntimeError(f'JTAG{index + 1} failed ({done.returncode}); see log')
            connect(); disconnect()
            record('jtag', passed=index + 1)
        if args.backpressure_seconds:
            # Verify the non-motion state before deliberately not reading.
            connect(); disconnect()
            transport = SerialTransport(args.port, 921600, .05, serial.Serial)
            try:
                transport.open()
                time.sleep(args.backpressure_seconds)
                received = len(transport.read_available())
            finally:
                transport.close()
            connect(); disconnect()
            record('backpressure', seconds=args.backpressure_seconds,
                   buffered_bytes=received, protocol_recovered=True)
        if args.soak_seconds:
            connect()
            start = time.monotonic()
            last_progress = start
            while time.monotonic() - start < args.soak_seconds:
                observe()
                if time.monotonic() - last_progress >= 30:
                    record('soak', elapsed_s=round(time.monotonic() - start, 1))
                    last_progress = time.monotonic()
                time.sleep(1)
            disconnect(); connect(); disconnect()
        record('passed', reconnects=args.cycles, jtag_cycles=args.jtag_cycles,
               soak_seconds=args.soak_seconds, physical_unplugs_tested=0, cold_starts_tested=0)
        return 0
    except Exception as exc:
        record('FAILED', error=f'{type(exc).__name__}: {exc}',
               ports=[p.device for p in list_ports.comports()],
               session=session_log_since()['lines'])
        return 1
    finally:
        hand.disconnect(stop_lower=False)
        hand.close_application()
        events.close()


if __name__ == '__main__':
    raise SystemExit(main())
