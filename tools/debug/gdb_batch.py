#!/usr/bin/env python3
"""One-shot JTAG debug session for the MMHand ESP32 firmware (P4/S3).

AI-agent entry point for breakpoint debugging: starts OpenOCD if it is not
running, runs a single GDB batch session, and ALWAYS finishes with
`delete` + `detach` so the chip is never left with armed hardware
breakpoints. On the ESP32-P4 the hardware breakpoint triggers survive soft
resets; a stale trigger wedges the chip into a WDT reset loop that only an
esptool download-mode reset can clear. This script therefore offers no way
to reset or detach with breakpoints armed.

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
from pathlib import Path

TARGETS = {
    "p4": {
        "board_cfg": "board/esp32p4-builtin.cfg",
        "gdb_leaf": "bin/riscv32-esp-elf-gdb.exe",
        "gdb_root": "riscv32-esp-elf-gdb",
        "elf": ".debug/p4/ESP32-P4.ino.elf",
    },
    "s3": {
        "board_cfg": "board/esp32s3-builtin.cfg",
        "gdb_leaf": "bin/xtensa-esp32s3-elf-gdb.exe",
        "gdb_root": "xtensa-esp-elf-gdb",
        "elf": ".debug/s3/ESP32-S3.ino.elf",
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


def spawn_openocd(openocd_exe: Path, scripts: Path, board_cfg: str, port: int, timeout: float) -> subprocess.Popen:
    cmd = [
        str(openocd_exe), "-s", str(scripts), "-f", board_cfg,
        # OpenOCD otherwise probes the flash map on every GDB connect and
        # rejects the connection outright when that probe fails.
        "-c", "gdb_memory_map disable",
    ]
    print(f"[gdb_batch] starting OpenOCD: {' '.join(cmd)}", flush=True)
    proc = subprocess.Popen(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, errors="replace",
    )
    deadline = time.monotonic() + timeout
    marker = f"Listening on port {port} for gdb connections"
    printed: list[str] = []
    while time.monotonic() < deadline:
        line = proc.stdout.readline()  # type: ignore[union-attr]
        if not line:  # EOF: OpenOCD exited (device missing or wedged)
            break
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


def build_gdb_command_list(args: argparse.Namespace) -> list[str]:
    cmds = [
        "set pagination off",
        "set confirm off",
        "set print pretty on",
        f"target remote :{args.port}",
    ]
    cmds += [f"break {b}" for b in args.breakpoint]
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


def run_gdb(gdb_exe: Path, elf: Path, cmds: list[str], timeout: float) -> tuple[bool, str]:
    argv = [str(gdb_exe), "-q", str(elf), "--batch"]
    for c in cmds:
        argv += ["-ex", c]
    try:
        done = subprocess.run(
            argv, capture_output=True, text=True,
            errors="replace", timeout=timeout,
        )
        output = done.stdout + done.stderr
        return done.returncode == 0, output
    except subprocess.TimeoutExpired:
        return False, f"[gdb_batch] GDB session timed out after {timeout}s (breakpoint never hit?)"


def rescue_after_timeout(gdb_exe: Path, port: int) -> None:
    """A killed GDB leaves the target halted with HW breakpoints armed.

    Run a fresh cleanup session: remove breakpoints, detach (resumes cores).
    """
    rescue_cmds = ["set confirm off", f"target remote :{port}", "delete", "detach"]
    argv = [str(gdb_exe), "-q", "--batch"]
    for c in rescue_cmds:
        argv += ["-ex", c]
    try:
        done = subprocess.run(argv, capture_output=True, text=True, errors="replace", timeout=30)
        print(done.stdout + done.stderr)
    except subprocess.TimeoutExpired:
        print(
            "[gdb_batch] RESCUE FAILED: chip may still hold a stale breakpoint.\n"
            "Recover with: esptool -p COMx chip-id   (full system reset via download mode)",
            file=sys.stderr,
        )


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

    marker = elf.with_name(elf.name + ".flashed.sha256")
    if marker.is_file():
        flashed = marker.read_text().split()[0]
        if flashed != sha256_of(elf):
            print("[gdb_batch] WARNING: ELF differs from the last uploaded firmware; "
                  "rebuild + re-upload before trusting the debug session.", flush=True)
    else:
        print("[gdb_batch] WARNING: no flashed-firmware marker (upload_debug.ps1 has not "
              "run for this build); the ELF may not match the chip.", flush=True)

    spawned = None
    try:
        if openocd_is_running():
            print("[gdb_batch] reusing already-running OpenOCD", flush=True)
        else:
            spawned = spawn_openocd(openocd_exe, scripts, target["board_cfg"], args.port, timeout=30.0)
            if args.keep_openocd:
                print(f"[gdb_batch] OpenOCD (pid {spawned.pid}) stays running for follow-up use.", flush=True)

        print(f"[gdb_batch] GDB session: {gdb_exe.name}  elf={elf}", flush=True)
        ok, output = run_gdb(gdb_exe, elf, build_gdb_command_list(args), args.timeout)
        print(output)
        if not ok and "timed out" in output:
            rescue_after_timeout(gdb_exe, args.port)
            return 2
        if not ok:
            return 1
        print("[gdb_batch] session finished; breakpoints removed, target resumed.", flush=True)
        return 0
    finally:
        if spawned is not None and not args.keep_openocd:
            spawned.terminate()
            try:
                spawned.wait(timeout=5)
            except subprocess.TimeoutExpired:
                spawned.kill()


if __name__ == "__main__":
    sys.exit(main())
