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
with `delete` → `info breakpoints` → `detach` — even when GDB times out (a
rescue session removes breakpoints left behind by the killed GDB). There is
deliberately no way to reset or detach with breakpoints armed.

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

Fix — force a full system reset through download mode (clears the debug
trigger registers that a soft reset does not):

```text
esptool.exe -p COM12 chip-id     # enter download mode, full system reset
esptool.exe -p COM12 run         # boot the application again
```

There is no `riscv clear_all_hw_breakpoints` command in
openocd-esp32 v0.12.0-esp32-20251215 (it prints a command list), so the
esptool route is the reliable software recovery. If even esptool cannot
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
