# Repository agent instructions

When asked to generate or update the bilingual AI code guide in `docs/ai/`, read
`docs/ai/GENERATE.md` and follow its prepare → annotate → validate → publish
workflow. Use committed source and the exported graph packets as evidence.
Do not edit the generated README or HTML files directly.

## Hardware status (updated 2026-10-18, evening)

TWO fingers are now attached again: index (M15–M19, bus 0) AND middle
(M10–M14). Servo route scan: valid=0x000FFC00, all ten controllable, no
conflicts. Index multiturn: M15/M17/M18/M19 restored OK; M16 restore via 0x0B
still REJECTED every connect — restore M16 or redo J12 Motor Calibration
before running J12. Middle finger: M10–M14 all multiturn UNKNOWN after restore
attempts — run recovery or recalibrate J08–J11 before middle-finger motion;
known M10 LUT plateau 13–28° remains (audit marks J08 ineligible).
OPEN BLOCKER (2026-10-18 evening): S3→P4 CAN link is down from P4's side
(can_link_online=False, zero CAN frames, encoder_online_bitmap=0) — magnetic
encoder feedback absent, feedback control gates refuse motion. Check CAN
wiring/S3 boot first; servos themselves are fine. Kp re-screening on J13 is
blocked on this. Collision detection is currently PARAMETER-DISABLED for
testing (arm_velocity=100°/s in control_parameters.json rev63) — logic intact.
Servo EEPROM 84/85/86 read 254/254/0 on all online index-finger motors —
verified 2026-10-18, archive `mmhand_data/experiments/control-tuning/20261016-183731/motor-profile-readback-20261018/`.
Control firmware: limit simplification + simplified take-up (rev63, ELF
SHA256 `F86AAE5E07F3512973AEEFA81B051A36A8B9AB75DB7B74C18438FD3CD6DA86F8`),
see `docs/debug/control-tuning.md` for the full record.

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
