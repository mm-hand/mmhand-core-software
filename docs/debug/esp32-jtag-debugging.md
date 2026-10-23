# ESP32 JTAG debugging (P4 / S3) — OpenOCD + GDB without ESP-IDF

How to do source-level breakpoint debugging (breakpoints, single-step,
variable inspection, FreeRTOS thread listing) on the MMHand ESP32-P4 and
ESP32-S3 firmware, from the command line, for humans and AI agents alike.

Verified end-to-end on 2026-10-13 against the ESP32-P4 (rev v1.3) over its
built-in USB-Serial-JTAG port: flash → attach → FreeRTOS thread awareness →
breakpoint hit at `MotionControlTask.cpp:37` → live variable reads → clean
detach.

## 1. Why this works without ESP-IDF

`idf.py` is just a wrapper around the ESP-IDF CMake build system plus
convenience entry points for flash/monitor/OpenOCD. The Arduino build never
invokes it: the esp32 Arduino core 3.x is built **on top of** precompiled
ESP-IDF libraries that ship inside the core package, so "the IDF part" is
already done once the core is installed.

Debugging needs only two standalone executables, which the esp32 Arduino
core already ships:

| Tool | Purpose | Location (this machine) |
| --- | --- | --- |
| `openocd.exe` (esp32 fork) | JTAG ↔ GDB server on `localhost:3333` | `%LOCALAPPDATA%\Arduino15\packages\esp32\tools\openocd-esp32\v0.12.0-esp32-20251215\bin\` |
| `riscv32-esp-elf-gdb.exe` | GDB for ESP32-P4 (RISC-V) | `...\tools\riscv32-esp-elf-gdb\16.3_20250913\bin\` |
| `xtensa-esp32s3-elf-gdb.exe` | GDB for ESP32-S3 (Xtensa) | `...\tools\xtensa-esp-elf-gdb\16.3_20250913\bin\` |
| `arduino-cli.exe` | compile (produces the debug ELF) + flash | `%LOCALAPPDATA%\Programs\Arduino IDE\resources\app\lib\backend\resources\` |

The pipeline: **arduino-cli compile (ELF with DWARF) → arduino-cli upload →
OpenOCD → GDB**. OpenOCD does not "generate" the ELF — it only exposes the
chip as a GDB server; GDB loads the *same* ELF to translate addresses back
into source lines and variable names.

## 2. Build the debug firmware

Debug builds use `--optimize-for-debug` (`-Og -g3`): slightly larger and
slower than the release build, but variables and line stepping survive
optimization. Builds land in `.debug/<target>/` (git-ignored); the ELF/BIN
pair there is the single source of truth shared by upload and GDB.

For the affected P4 Arduino SDK, `build_debug.ps1` first runs
`patch_p4_usb_fifo.py`. The vendor TX helper used a read/modify/write on EP1;
reading this FIFO consumes RX bytes, so full-duplex traffic could corrupt
commands. The backport uses a direct 32-bit write, matching
[Espressif's current implementation](https://github.com/espressif/esp-idf/blob/master/components/esp_hal_usb/esp32p4/include/hal/usb_serial_jtag_ll.h).
The script preserves the original header beside it as
`*.mmhand-before-fifo-fix.bak` and refuses unknown implementations. This
changes neither the USB connector nor CDC/JTAG mode. Arduino IDE users must
run the same prerequisite before compiling, and rebuild the core after the
first application (do not reuse a core archive built with the old header):

```powershell
python tools/debug/patch_p4_usb_fifo.py --tools-root "$env:LOCALAPPDATA/Arduino15/packages/esp32/tools"
```

```powershell
# P4 (S3: add -Target s3; FQBN and GDB are switched automatically)
powershell -File tools/debug/build_debug.ps1 -Target p4
powershell -File tools/debug/upload_debug.ps1 -Target p4 -Port COM12
```

`upload_debug.ps1` records the uploaded ELF's SHA256 next to the build
(`*.flashed.sha256`); `gdb_batch.py` warns when the ELF no longer matches
what is on the chip.

## 3. Human workflow — breakpoints and variable inspection

Two terminals, no IDE, no plugins:

**Terminal 1 — start the GDB server and keep it running:**

```powershell
powershell -File tools/debug/start_openocd.ps1 -Target p4
```

**Terminal 2 — interactive GDB session:**

```text
> & "$env:LOCALAPPDATA\Arduino15\packages\esp32\tools\riscv32-esp-elf-gdb\16.3_20250913\bin\riscv32-esp-elf-gdb.exe" -q .debug\p4\ESP32-P4.ino.elf
(gdb) target remote :3333
(gdb) break MotionControlTask.cpp:37        # file:line, or a symbol name
(gdb) continue
Breakpoint 1 hit — the core halts inside your source line
(gdb) info locals                           # all locals of this frame
(gdb) print nowMs                           # one variable/expression
(gdb) bt                                    # call stack
(gdb) info threads                          # every FreeRTOS thread, by name
(gdb) next                                  # step over
(gdb) step                                  # step into
(gdb) set var periodTicks = 20              # optional: change a variable
(gdb) continue                              # resume
(gdb) delete                                # ALWAYS before leaving
(gdb) detach                                # resumes the target
(gdb) quit
```

Then stop OpenOCD with `Ctrl+C` in Terminal 1. The firmware keeps running
normally afterwards and the chip can be re-attached at any time.

## 4. AI-agent workflow — one-shot batch sessions

Agents should not hold long-lived sessions; each debugging question is one
`gdb_batch.py` call that attaches, inspects, and detaches safely:

```bash
python tools/debug/gdb_batch.py --target p4 \
    --break MotionControlTask.cpp:37 --locals --print nowMs --print periodTicks --bt
python tools/debug/gdb_batch.py --target p4 --threads
python tools/debug/gdb_batch.py --target p4 --break servoBusTask --cmd "info registers pc sp a0"
```

The script auto-starts OpenOCD if none is running (reuses it otherwise),
prints the raw GDB output for the agent to read, and unconditionally ends
with `delete` → `info breakpoints` → `detach` on normal completion. These
messages alone do not prove that SMP cores resumed. Both launchers install
`esp_debug_cleanup.tcl`; the batch tool additionally verifies every hardware
core is running through OpenOCD Tcl RPC before graceful shutdown. On timeout,
cleanup uses OpenOCD directly, not a second symbol-less GDB (which can itself
assert in FreeRTOS thread selection). P4's three triggers per core are cleared
and read back before resume, including orphaned triggers. Cleanup failure
returns a nonzero status; it never silently resets the board. Full OpenOCD
logs are continuously drained to `.debug/openocd-*.log`.

Close CDC connections and release motion authorization before debugging;
reconnect after OpenOCD exits. See [USB stability evidence](control-tuning.md).

Interactive stepping (rarely needed by agents): start OpenOCD with
`--keep-openocd`, then drive GDB interactively as a human would.

## 5. GDB quick reference

| Command | Effect |
| --- | --- |
| `break FILE:LINE` / `break FUNC` | set a breakpoint (hardware, also in flash) |
| `continue` | run until the next breakpoint |
| `info threads` | all FreeRTOS threads with names + owning core |
| `info locals` / `print EXPR` | inspect this frame's variables |
| `bt` | backtrace of the current thread |
| `next` / `step` | step over / into a source line |
| `finish` | run until the current function returns |
| `watch EXPR` | break when a variable changes |
| `set var X = V` | modify a live variable |
| `thread N` | switch to another thread |
| `x/16xw ADDR` | raw memory dump |
| `delete` → `detach` | mandatory session teardown, in this order |

Thread note: after a breakpoint hit GDB switches to the hitting thread
automatically. `<optimized out>` locals are normal under `-Og`; pick another
breakpoint line or read registers / memory instead. Stepping into ESP-IDF
library code shows only disassembly (prebuilt `.a` files carry no debug
info); application code under `ESP32-P4/src` / `ESP32-S3/src` is fully
source-level.

## 6. Iron rules

1. **Never reset or detach with breakpoints armed.** On the ESP32-P4 the
   hardware breakpoint triggers survive soft resets (JTAG CPU reset). A
   stale trigger makes the firmware hit an orphan breakpoint → halt → HP
   watchdog reset → repeat, and OpenOCD then rejects every new GDB
   connection. Always `delete` first, then `detach`.
2. **OpenOCD owns the USB-JTAG port exclusively.** No esptool upload and no
   serial monitor while it runs. Stop OpenOCD first, flash, restart it.
3. **ELF and flashed BIN must come from the same build directory.** After
   recompiling, re-flash before debugging (the SHA256 markers catch this).
4. **Always start OpenOCD with `-c "gdb_memory_map disable"`.** Without it
   OpenOCD probes the flash map on every GDB connect and rejects the
   connection when the probe fails (`Connect failed ... connection
   rejected`).
5. Debug builds and release builds are different artifacts; flash what you
   debug. `-Og` costs a few percent of flash.

## 7. Recovery: chip wedged by a stale breakpoint

Symptoms: GDB connect dropped immediately, OpenOCD log shows
`could not remove breakpoint`, `Failed to probe flash`, `gdb connection
rejected`, or the board's serial output stops entirely.

First retain the failure logs. If OpenOCD can attach, use the repository
cleanup hook to clear/read back P4 hardware triggers and verify both cores
running; this can restore serial data without a reset. If that fails or
OpenOCD cannot attach, an explicit full system reset through download mode
clears debug trigger state:

```text
esptool.exe -p COM12 chip-id     # enter download mode, full system reset
esptool.exe -p COM12 run         # boot the application again
```

There is no `riscv clear_all_hw_breakpoints` command in
openocd-esp32 v0.12.0-esp32-20251215 (it prints a command list), so the
repository hook accesses the P4 trigger CSRs directly; do not substitute
unknown debugger commands. If even esptool cannot
connect, unplug and replug the board's USB cable (power cycle clears
everything).

## 8. Troubleshooting

| Symptom | Cause | Fix |
| --- | --- | --- |
| `arduino-cli board list` empty, no COM port | board not enumerated: charge-only cable, power switch off, wrong USB-C port | use a data cable, power the board, plug the USB-JTAG/serial debug port (check Device Manager for `USB JTAG/serial debug unit`, VID 303A) |
| GDB `target remote` dropped instantly | OpenOCD rejected the connect (flash probe failed) | restart OpenOCD with `-c "gdb_memory_map disable"` |
| `attempted 'gdb' connection rejected` in OpenOCD log | stale hardware breakpoint wedge (see §7) | esptool download-mode reset, then retry |
| `break FILE:LINE` says "No such file" | ELF mismatch — file changed since the build | rebuild + re-upload, keep paths relative to the repo root |
| Variables show `<optimized out>` | `-Og` optimization | choose a line after all locals are set, or read registers/memory |
| Breakpoint "hit" with PC reading `0x0` | P4 SMP halt-group quirk on first task entry | ignore; `info threads` + re-`continue`, or break at a line inside the loop body |
| Upload works, OpenOCD says no JTAG tap | plugged into the UART/OTG port, not the USB-Serial-JTAG port | use the debug USB-C port; JTAG needs VID 303A PID 1001 interface 2 |
| S3 target connects but symbols wrong | wrong GDB flavor | S3 uses `xtensa-esp32s3-elf-gdb` + `board/esp32s3-builtin.cfg` (handled by `-Target s3`) |

## 9. File map

| File | Role |
| --- | --- |
| `tools/debug/debug_env.ps1` | shared environment: auto-locates arduino-cli/OpenOCD/GDB, per-target FQBN, board cfg, build dir |
| `tools/debug/build_debug.ps1` | `-Og -g3` compile into `.debug/<target>/`, records build SHA256 |
| `tools/debug/upload_debug.ps1` | flashes the build, records flashed-firmware SHA256 marker |
| `tools/debug/start_openocd.ps1` | long-running GDB server (human workflow) |
| `tools/debug/verify_debug_env.ps1` | prints resolved toolchain, build/flashed hashes, USB visibility |
| `tools/debug/gdb_batch.py` | one-shot agent entry point (auto-OpenOCD, enforced cleanup) |
| `.debug/<target>/` | git-ignored build output: ELF (GDB) + BIN (flash) |


## Live attach must not reset the P4

P4 builds also verify two reproducible USB prerequisites:
`patch_p4_usb_fifo.py` (SDK direct FIFO write) and `patch_p4_hwcdc.py`
(Arduino HWCDC empty-IN wakeup). Both back up the vendor file and reject
unknown implementations. The latter currently accepts the audited3.3.10
baseline only. Its guarded empty-packet commit addresses an observed
re-enumeration state; it is not an automatic chip reset. See
[USB evidence and acceptance](control-tuning.md) for the actual validation
status. A different Arduino version requires a fresh baseline audit.

The launchers set `ESP_FLASH_SIZE 0` BEFORE the board configuration. The
vendor flash-enabled attach hook can otherwise run `reset halt` to disable
memory protection. This was reproduced on P4 and confirmed by a JTAG reset
diagnostic; it clears RAM calibration. `gdb_memory_map disable` alone does
not suppress this hook. Keep hardware breakpoints enabled on both targets;
use Arduino for uploading, not the live debug session for flash writes.
The batch tool requires an ELF matching its uploaded hash and recorded USB
identity. It refuses an incompatible pre-existing OpenOCD without touching
that server's targets.

## Waveshare ESP32-S3-Nano: recovery when native USB does not enumerate

This machine uses Waveshare ESP32-S3-Nano, with one native USB Type-C
connector and an RST button. Do not assume a separate USB-UART connector or
BOOT button from another S3 development board.

Waveshare's board-specific recovery procedure is to short **B1 to GND**
before applying power. B1 is GPIO0, not digital D1; do not substitute B0.
Disconnect USB and external power, leave sensors disconnected, fit the jumper,
then reconnect the USB data cable. Keep the board in download mode while
checking its actual enumerated COM port and flashing the matching debug build.
After successful upload, remove the jumper before reset/re-power for normal
execution. There is no need to erase the entire Flash for this recovery step.

Sources:
- https://docs.waveshare.com/ESP32-S3-Nano/FAQ
- https://files.waveshare.com/wiki/ESP32-S3-Nano/ESP32-S3-Nano-Schematic.pdf
- https://support.arduino.cc/hc/en-us/articles/9625819325212-About-the-B1-and-B0-pins-on-the-Nano-ESP32

USB descriptor failure alone does not establish whether the cause is the
application, debug state, USB connection or power. Successful ROM enumeration
is the next diagnostic observation; do not claim ROM mode was entered merely
because RST was pressed. Keep BIN and ELF matched and remove breakpoints
before detaching, as described above.
