# Repository agent instructions

When asked to generate or update the bilingual AI code guide in `docs/ai/`, read
`docs/ai/GENERATE.md` and follow its prepare → annotate → validate → publish
workflow. Use committed source and the exported graph packets as evidence.
Do not edit the generated README or HTML files directly.

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
