# Repository agent instructions

When asked to generate or update the bilingual AI code guide in `docs/ai/`, read
`docs/ai/GENERATE.md` and follow its prepare → annotate → validate → publish
workflow. Use committed source and the exported graph packets as evidence.
Do not edit the generated README or HTML files directly.

## Hardware status (updated 2026-10-20, evening)

P4 reflashed with the current calibration/ABS refactor firmware
(ELF SHA256 `9F9EB112AD2086F7B7488B812FF7C3CF9059939FA4B26966C205B69DF0BF15CD`).
Servo route scan: valid=0x000FFC00, controllable=0x000FFC00, no conflicts;
CAN link is online and encoders report online=0x0000FF00. After reloading the
active calibration bundle, Manual and Auto valid masks are 0x00FF00 and the
active joint scope is 0x0000FF00 (all eight joints).

J14 was recalibrated alone after the release/state fix: Auto Min=9134,
Auto Max=4129, auto span=5005 vs Manual Min=8573 / Manual Max=3758 / manual
span=4815; autoMinOffsetCounts=-561 and autoMaxShortfallCounts=371 raise the
return/travel warning. The fresh value matches the earlier ~9137 observation,
so that value was not merely a stale host cache. The small negative ABS restore
path (M10 -73, M14 -15, M19 -384) was verified against 0x0B after the fix.

Control firmware: limit simplification + simplified take-up (rev63, ELF
SHA256 `F86AAE5E07F3512973AEEFA81B051A36A8B9AB75DB7B74C18438FD3CD6DA86F8`),
see `docs/debug/control-tuning.md` for the full record. Collision detection is
still parameter-disabled for testing (arm_velocity=100°/s); logic intact.

## ESP32 JTAG debugging (P4/S3)

The Arduino build produces an ELF with DWARF debug symbols. The debug stack is
arduino-cli (compile + flash) → openocd-esp32 → GDB, all bundled with the esp32
Arduino core under `%LOCALAPPDATA%\Arduino15\packages\esp32\tools\`. ESP-IDF /
idf.py is NOT needed. Full guide: `docs/debug/esp32-jtag-debugging.md`.

One-shot agent entry point (auto-starts OpenOCD, always cleans up):

```
python tools/debug/gdb_batch.py --target p4 \
    --break MotionControlTask.cpp:37 --locals --print nowMs --bt
```

Manual flow: `tools/debug/build_debug.ps1` → `upload_debug.ps1 -Port COMx` →
`start_openocd.ps1` (keep running) → GDB: `target remote :3333`. S3 differs
only by `-Target s3` (xtensa GDB + `board/esp32s3-builtin.cfg`).

Multiple boards: `upload_debug.ps1` records each target's USB-Serial-JTAG
serial to `.debug/<target>/`; `gdb_batch.py` and `start_openocd.ps1` pin that
adapter with OpenOCD `adapter serial`, so OpenOCD attaches the right board.
S3 breakpoints are always hardware (`hbreak`): Xtensa has no software
breakpoints in flash, and only two comparators exist.

Iron rules — violations wedge the chip or the connection:

1. Always `delete` breakpoints before `detach`/reset. P4/S3 hardware
   breakpoint triggers survive soft resets; a stale trigger causes a
   halt/WDT-reset loop and OpenOCD then rejects all GDB connections.
   Recovery: `esptool -p COMx chip-id` (download-mode full system reset).
2. OpenOCD owns the USB-JTAG port exclusively: no esptool or serial monitor
   while it runs.
3. Flash the BIN and load the ELF from the same `.debug/<target>/` build;
   re-flash after every rebuild.
4. Start OpenOCD with `-c "gdb_memory_map disable"`, or GDB connects are
   rejected when the flash-map probe fails.
5. `<optimized out>` variables under `-Og` are expected; move the breakpoint
   or read registers/memory instead.
