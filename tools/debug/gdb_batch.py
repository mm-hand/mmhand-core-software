#!/usr/bin/env python3
"""One-shot JTAG debug session for the MMHand ESP32 firmware (P4/S3).

AI-agent entry point for breakpoint debugging: starts OpenOCD if it is not
running, runs a single GDB batch session, and ALWAYS finishes with
`delete` + `detach`, followed by direct OpenOCD cleanup and verification of
all hardware core states. P4 orphaned triggers are cleared and read back
before resume. No automatic device reset is performed, including on GDB
timeout. The complete OpenOCD output is retained under .debug/.

Examples:
  python tools/debug/gdb_batch.py --target p4 \
      --break MotionControlTask.cpp:37 --locals --print nowMs --bt
  python tools/debug/gdb_batch.py --target p4 --threads
  python tools/debug/gdb_batch.py --target p4 --break servoBusTask \
      --print sharedData --cmd "info registers pc sp a0"

Requires: esp32 Arduino core installed (tools live under
%LOCALAPPDATA%/Arduino15/packages/esp32/tools) and a debug build produced by
tools/debug/build_debug.ps1 + upload_debug.ps1. No ESP-IDF involved.
"""
from __future__ import annotations

import argparse
import hashlib
import os
import subprocess
import sys
import time
import queue
import threading
import socket
from pathlib import Path

TARGETS = {
    "p4": {
        "board_cfg": "board/esp32p4-builtin.cfg",
        "gdb_leaf": "bin/riscv32-esp-elf-gdb.exe",
        "gdb_root": "riscv32-esp-elf-gdb",
        "elf": ".debug/p4/ESP32-P4.ino.elf",
        "hw_breakpoints": True,
    },
    "s3": {
        "board_cfg": "board/esp32s3-builtin.cfg",
        "gdb_leaf": "bin/xtensa-esp32s3-elf-gdb.exe",
        "gdb_root": "xtensa-esp-elf-gdb",
        "elf": ".debug/s3/ESP32-S3.ino.elf",
        # Xtensa flash runs at 0x42xxxxxx; with the memory map disabled GDB
        # treats it as writable RAM, tries a software breakpoint on continue
        # and aborts with "Cannot access memory". IBREAK comparators must be
        # requested explicitly (ESP32-S3 has two; a third --break fails).
        "hw_breakpoints": True,
    },
}


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def core_tools_dir() -> Path:
    base = Path(os.environ.get("LOCALAPPDATA", "")) / "Arduino15" / "packages" / "esp32" / "tools"
    if not base.is_dir():
        sys.exit(f"[gdb_batch] esp32 Arduino core tools not found: {base}")
    return base


def latest_tool(root: Path, leaf: str) -> Path | None:
    """Newest version directory wins (Espressif names sort chronologically)."""
    if not root.is_dir():
        return None
    for d in sorted((x for x in root.iterdir() if x.is_dir()), key=lambda p: p.name, reverse=True):
        exe = d / leaf
        if exe.is_file():
            return exe
    return None


def openocd_is_running() -> bool:
    out = subprocess.run(
        ["tasklist", "/FI", "IMAGENAME eq openocd.exe"],
        capture_output=True, text=True, errors="replace",
    ).stdout
    return "openocd.exe" in out.lower()


def drain_openocd_output(proc: subprocess.Popen, log_path: Path) -> queue.Queue:
    """Keep draining after startup: a full Windows pipe blocks the GDB server."""
    lines = queue.Queue(maxsize=256)

    def read():
        with log_path.open('w', encoding='utf-8') as log:
            for line in proc.stdout:
                log.write(line)
                log.flush()
                try:
                    lines.put_nowait(line)
                except queue.Full:
                    # Startup observations are bounded; the complete log is on disk.
                    pass

    threading.Thread(target=read, name='openocd-output', daemon=True).start()
    return lines


def spawn_openocd(openocd_exe: Path, scripts: Path, board_cfg: str, port: int, timeout: float,
                  adapter_serial: str = "") -> subprocess.Popen:
    cmd = [
        str(openocd_exe), "-s", str(scripts),
        # Live attach must not run the vendor flash-access reset-halt hook.
        # Flashing belongs to arduino-cli; all debug breakpoints are hardware.
        "-c", "set ESP_FLASH_SIZE 0", "-f", board_cfg,
    ]
    if adapter_serial:
        # Every ESP32 USB-Serial-JTAG board shares one VID/PID; without this
        # pin OpenOCD may attach a different board and fail with
        # "Unsupported DTM version: -1".
        cmd += ["-c", f"adapter serial {adapter_serial}"]
    # OpenOCD otherwise probes the flash map on every GDB connect and
    # rejects the connection outright when that probe fails.
    cmd += [
        "-c", "gdb_memory_map disable",
        "-f", str(repo_root() / 'tools/debug/esp_debug_cleanup.tcl'),
    ]
    print(f"[gdb_batch] starting OpenOCD: {' '.join(cmd)}", flush=True)
    proc = subprocess.Popen(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, errors="replace",
    )
    log_path = repo_root() / '.debug' / f'openocd-{time.time_ns()}.log'
    log_path.parent.mkdir(parents=True, exist_ok=True)
    lines = drain_openocd_output(proc, log_path)
    print(f'[gdb_batch] OpenOCD log: {log_path}', flush=True)
    deadline = time.monotonic() + timeout
    marker = f"Listening on port {port} for gdb connections"
    printed: list[str] = []
    while time.monotonic() < deadline:
        try:
            line = lines.get(timeout=min(0.1, max(0.001, deadline - time.monotonic())))
        except queue.Empty:
            if proc.poll() is not None:
                break
            continue
        printed.append(line.rstrip())
        if marker in line:
            return proc
    proc.terminate()
    tail = "\n".join(printed[-25:]) or "(no output)"
    sys.exit(f"[gdb_batch] OpenOCD did not reach 'Listening on port {port}':\n{tail}")


def sha256_of(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest().upper()


def validate_flashed_elf(elf: Path) -> str:
    marker = elf.with_name(elf.name + '.flashed.sha256')
    if not marker.is_file() or not marker.read_text().split():
        raise ValueError('No flashed-firmware marker; upload this matched build before debugging')
    if marker.read_text().split()[0] != sha256_of(elf):
        raise ValueError('ELF differs from the last uploaded firmware; re-flash before debugging')
    serial_marker = elf.with_name(elf.name + '.jtag_serial')
    serial = serial_marker.read_text().strip() if serial_marker.is_file() else ''
    if not serial:
        raise ValueError('No USB-JTAG identity recorded; use upload_debug.ps1 before debugging')
    return serial


def build_gdb_command_list(args: argparse.Namespace, hardware_breakpoints: bool) -> list[str]:
    cmds = [
        "set pagination off",
        "set confirm off",
        "set print pretty on",
        f"target remote :{args.port}",
    ]
    verb = "hbreak" if hardware_breakpoints else "break"
    cmds += [f"{verb} {b}" for b in args.breakpoint]
    if args.breakpoint:
        cmds.append("continue")
    if args.threads:
        cmds.append("info threads")
    if args.locals:
        cmds.append("info locals")
    cmds += [f"print {p}" for p in args.print_expr]
    if args.bt:
        cmds.append("bt")
    if args.regs:
        cmds.append("info registers")
    cmds += args.cmd
    # Cleanup is unconditional and always last: never detach or reset with
    # breakpoints armed (stale P4/S3 HW triggers wedge the chip).
    cmds += [
        "delete",
        "info breakpoints",
        "detach",
    ]
    return cmds


def bounded_gdb(argv: list[str], timeout: float) -> subprocess.CompletedProcess:
    # The Windows launcher spawns a second GDB process. Killing only the
    # launcher leaves its child holding stdout open, so subprocess.run's
    # timeout cleanup can wait forever and never reach breakpoint rescue.
    process = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               text=True, errors='replace')
    try:
        stdout, stderr = process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        if os.name == 'nt':
            subprocess.run(['taskkill', '/PID', str(process.pid), '/T', '/F'],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
        else:
            process.kill()
        process.communicate(timeout=10)
        raise
    return subprocess.CompletedProcess(argv, process.returncode, stdout, stderr)


def run_gdb(gdb_exe: Path, elf: Path, cmds: list[str], timeout: float) -> tuple[bool, str]:
    argv = [str(gdb_exe), "-q", str(elf), "--batch"]
    for c in cmds:
        argv += ["-ex", c]
    try:
        done = bounded_gdb(argv, timeout)
        output = done.stdout + done.stderr
        return done.returncode == 0, output
    except subprocess.TimeoutExpired as exc:
        def decode(value):
            return value.decode(errors='replace') if isinstance(value, bytes) else (value or '')
        partial = decode(exc.output) + decode(exc.stderr)
        return False, partial + f"\n[gdb_batch] GDB session timed out after {timeout}s (breakpoint never hit?)"


def openocd_rpc(command: str, port: int = 6666) -> str:
    """OpenOCD Tcl RPC terminates each request/reply with SUB (0x1a)."""
    with socket.create_connection(('127.0.0.1', port), timeout=5) as connection:
        connection.settimeout(10)
        connection.sendall(command.encode('utf-8') + b'\x1a')
        reply = bytearray()
        while b'\x1a' not in reply:
            chunk = connection.recv(4096)
            if not chunk:
                raise RuntimeError('OpenOCD closed the cleanup connection')
            reply.extend(chunk)
            if len(reply) > 1024 * 1024:
                raise RuntimeError('OpenOCD cleanup reply exceeded limit')
        return reply.split(b'\x1a')[0].decode('utf-8', errors='replace')


def verify_target_resumed(rpc=openocd_rpc) -> str:
    # GDB 'detached' is not evidence that SMP cores resumed. Delete EVERY
    # trigger first (including those orphaned by a killed GDB), then resume
    # halted cores and verify hardware state before shutting OpenOCD down.
    result = rpc('mmhand_debug_release; '
                 'set mm_targets [target names]; '
                 'sleep 50; set mm_states {}; '
                 'foreach t $mm_targets { $t arp_poll; '
                 'lappend mm_states "$t=[$t curstate]" }; '
                 'join $mm_states ","')
    states = result.strip().split(',')
    if not states or not all('=' in s and s.rsplit('=', 1)[1] == 'running' for s in states):
        raise RuntimeError(f'Hardware resume not confirmed: {result}')
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--target", choices=sorted(TARGETS), default="p4", help="debug target (default: p4)")
    parser.add_argument("--elf", type=Path, default=None, help="ELF to load symbols from (default: .debug/<target>/<sketch>.ino.elf)")
    parser.add_argument("--port", type=int, default=3333, help="OpenOCD gdb port (default: 3333)")
    parser.add_argument("--break", dest="breakpoint", action="append", default=[],
                        metavar="FILE:LINE|SYMBOL", help="set a breakpoint, then continue (repeatable)")
    parser.add_argument("--print", dest="print_expr", action="append", default=[],
                        metavar="EXPR", help="print a variable/expression after the hit (repeatable)")
    parser.add_argument("--locals", action="store_true", help="info locals after the hit")
    parser.add_argument("--bt", action="store_true", help="backtrace after the hit")
    parser.add_argument("--threads", action="store_true", help="list FreeRTOS threads after the hit")
    parser.add_argument("--regs", action="store_true", help="dump CPU registers after the hit")
    parser.add_argument("--cmd", action="append", default=[],
                        metavar="GDB_CMD", help="extra raw GDB command before cleanup (repeatable)")
    parser.add_argument("--timeout", type=float, default=90.0, help="gdb session timeout in seconds (default: 90)")
    parser.add_argument("--keep-openocd", action="store_true",
                        help="leave an OpenOCD started by this script running (for interactive follow-up)")
    args = parser.parse_args()

    target = TARGETS[args.target]
    root = repo_root()
    elf = args.elf or (root / target["elf"])
    if not elf.is_file():
        sys.exit(
            f"[gdb_batch] debug ELF not found: {elf}\n"
            f"Build it first:  powershell -File tools/debug/build_debug.ps1 -Target {args.target}\n"
            f"Flash it after:  powershell -File tools/debug/upload_debug.ps1 -Target {args.target} -Port COMx"
        )

    tools = core_tools_dir()
    gdb_exe = latest_tool(tools / target["gdb_root"], target["gdb_leaf"])
    openocd_exe = latest_tool(tools / "openocd-esp32", "bin/openocd.exe")
    if not gdb_exe or not openocd_exe:
        sys.exit("[gdb_batch] OpenOCD/GDB not found under the esp32 Arduino core tools.")
    scripts = openocd_exe.parent.parent / "share" / "openocd" / "scripts"

    try:
        jtag_serial = validate_flashed_elf(elf)
    except ValueError as exc:
        sys.exit(f'[gdb_batch] {exc}')

    spawned = None
    owns_debug_session = False
    result = 1
    try:
        if openocd_is_running():
            expected = 'esp32p4' if args.target == 'p4' else 'esp32s3'
            observed = openocd_rpc('target names').split()
            if not observed or any(not name.startswith(expected + '.') for name in observed):
                sys.exit('[gdb_batch] Existing OpenOCD targets do not match the requested board')
            if openocd_rpc('set _FLASH_SIZE').strip() != '0':
                sys.exit('[gdb_batch] Existing OpenOCD allows flash-attach resets; restart with start_openocd.ps1')
            if openocd_rpc('adapter serial').strip() != jtag_serial:
                sys.exit('[gdb_batch] Existing OpenOCD adapter identity does not match the flashed board')
            if 'mmhand_debug_release' not in openocd_rpc('info procs mmhand_debug_release'):
                sys.exit('[gdb_batch] Existing OpenOCD lacks the verified cleanup hook; restart with start_openocd.ps1')
            owns_debug_session = True
            print("[gdb_batch] reusing already-running OpenOCD", flush=True)
        else:
            spawned = spawn_openocd(openocd_exe, scripts, target["board_cfg"], args.port,
                                    timeout=30.0, adapter_serial=jtag_serial)
            owns_debug_session = True
            if args.keep_openocd:
                print(f"[gdb_batch] OpenOCD (pid {spawned.pid}) stays running for follow-up use.", flush=True)

        print(f"[gdb_batch] GDB session: {gdb_exe.name}  elf={elf}", flush=True)
        ok, output = run_gdb(gdb_exe, elf, build_gdb_command_list(args, bool(target["hw_breakpoints"])), args.timeout)
        print(output)
        if not ok and "timed out" in output:
            # A second symbol-less GDB can hit FreeRTOS thread assertions too.
            # Cleanup below talks directly to OpenOCD, not through another GDB.
            result = 2
        elif ok:
            print("[gdb_batch] GDB session finished; verifying hardware cleanup next.", flush=True)
            result = 0
    finally:
        if owns_debug_session:
            try:
                states = verify_target_resumed()
                print(f'[gdb_batch] Hardware cleanup verified: {states}', flush=True)
            except Exception as exc:
                print(f'[gdb_batch] CLEANUP FAILED: {exc}; target state is unconfirmed. '
                      'No reset was performed.', flush=True)
                cleanup_failed = True
            else:
                cleanup_failed = False
        else:
            cleanup_failed = False
        if spawned is not None and not args.keep_openocd:
            try:
                print(openocd_rpc('shutdown'), flush=True)
            except (OSError, RuntimeError):
                pass
            try:
                spawned.wait(timeout=5)
            except subprocess.TimeoutExpired:
                spawned.terminate()
                spawned.kill()
    return 3 if cleanup_failed else result


if __name__ == "__main__":
    sys.exit(main())
