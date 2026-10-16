# P4 native USB stability investigation

## Runtime route and confirmed failure

The preferred runtime route remains expansion-board USB Serial/JTAG (GPIO24
D-, GPIO25 D+), Hardware CDC, CDC On Boot enabled. CH343 is an alternative
only after native USB isolation demonstrates an unresolved problem and the
UART route itself passes acceptance. Ports are identified by USB identity,
not by assuming COM numbers stay constant.

On2026-10-16, the old GDB timeout/rescue flow reported `detached`, but direct
OpenOCD examination found BOTH P4 cores still `halted`, at0x4ff0275a.
Explicit OpenOCD resume restored106295 serial bytes over2s without any reset
or flashing. This proves a debugger-exit failure mechanism. It does not prove
that every historical no-data incident has the same trigger.

An additional confirmed reset mechanism was reproduced on the diagnostic
firmware: the stock Espressif `esp_common.cfg` GDB-attach hook executes
`reset halt` when flash access requires disabling memory protection. The
OpenOCD log explicitly reported that action; the next `[USB_BOOT]` reported
`reset=12` (JTAG), with a retained previous uptime. RAM calibration was
cleared. This was an implicit debugger reset, not evidence of damaged Flash.
Both launchers now set `ESP_FLASH_SIZE 0` BEFORE loading the board config,
retain memory protection, disable flash-map probing and use hardware
breakpoints. Uploading remains a separate Arduino operation.

GDB also returned inconsistent FreeRTOS task context/registers in some
sessions. Such virtual-thread output alone is NOT evidence of actual memory
corruption or download mode. Use OpenOCD hardware core state and independent
serial observations to corroborate it.

Evidence (local, ignored):

- `.debug/usb-jtag-drained-first.log`: timed-out GDB/rescue.
- `.debug/usb-jtag-real-cores.log`: both cores remained halted.
- `.debug/usb-jtag-acceptance-baseline/`:10 successful corrected JTAG cycles
  followed by fresh protocol connections, plus5 software reconnects, on the
  original flashed firmware. These older observations did not yet include
  reset diagnostics and cannot exclude the implicit vendor attach reset.

The updated diagnostic firmware passed100 software reconnects, then exposed
the stock attach reset (`.debug/usb-firmware-final-acceptance/`, FAILED).
After disabling the flash attach hook,10 JTAG cycles passed with matching
calibration retained (`.debug/usb-live-attach-acceptance/`). However, its
subsequent20-second host-not-reading case FAILED: Windows could no longer
configure COM12 (error31). Physical replug restored communication. This
remaining USB/driver failure is distinct from the verified attach-reset
fix and prevents claiming complete stability. Physical and soak results
must be recorded separately as they complete.

The user subsequently moved P4 to the computer itself. Windows topology
confirmed the parent changed from the external `VID_35D6&PID_2510` hub to
`USB\\ROOT_HUB30` (`.debug/usb-direct-topology.json`). Before the comparison
could run, all serial devices disappeared: both Windows PnP and pyserial
reported no P4/S3/CH343. `.debug/usb-direct-restore.log` therefore failed on
port absence before calibration import; no firmware was flashed or reset.
Direct-port pressure/physical testing remains pending device enumeration.

A subsequent inventory first identified the actual CH343 as COM13,
VID1A86/PID55D3, serial `5AE6038249`; S3 remained COM14 and P4 native USB
was absent. A user RST with passive115200-baud UART0 capture produced
`ESP-ROM:esp32p4-eco2-20240710`, `SPI_FAST_FLASH_BOOT`, segment loads and
`entry 0x4ff29ed0`. This proves ROM selected Flash and transferred to the
second-stage loader; it does NOT prove the application completed startup.
Native USB was still absent after RST. Evidence:
`.debug/p4-ch343-boot-capture.bin` and `.log`. The minimal isolation sketch
adds independent CH343 text logging on Pico UART0 RX38/TX37 before/after
HWCDC initialization and once per second. This UART is diagnostic text only,
not a second control interface.

The minimal sketch compiled and uploaded/verified through COM13. Esptool
identified P4 revision1.3/MAC80:f1:b2:d0:6b:a5, confirming CH343 belongs to
the expected board. ELF SHA256:
`003B45F59A0EA8E1247E08642504D323DA3CBBDBCC60F3F183577C73A54B8270`.
UART captured monotonically increasing sequence1900..2600 and uptime20..27s,
heap442988, zero USB RX/reset counts. Native USB still did not enumerate.
TX counters here measure bytes accepted by HWCDC, NOT delivery to a host:
its disconnected policy may accept/drop data. Evidence:
`.debug/usb-probe-build.log`, `.debug/usb-probe-upload.log`,
`.debug/usb-probe-uart.log`. No business tasks are included in this image.
The production flashed-hash marker is removed while the probe is deployed,
preventing accidental debug with the production ELF. Production BIN/ELF and
the previous marker remain backed up; restoration is required after isolation.
A known-working CH343-cable-to-native-port swap is pending manual action.

After that cable swap, P4 native COM12/JTAG enumerated again, on the same
external-hub path1-7.1 formerly used by CH343. The first7s native read still
returned0 bytes although14 bytes were accepted for output. Matched probe
ELF live attach found both IDLE cores running and loop sequence60061;
interrupt enable0x20C, SOF/control requests observed, and OUT endpoint read
pointer14. Native USB's existence and a completed OS write are therefore
insufficient evidence of healthy bidirectional transport.
After two live inspections and an explicit diagnostic endpoint write
(`EP1.val=0x55`, `EP1_CONF.val=1`), native read recovered43720 bytes/5s;
another read recovered35802 bytes/4s with RX counter28. No reset/reflash
occurred. There was no read between the earlier live inspections and this
endpoint experiment, so it does NOT isolate the endpoint write as the cause
of recovery. No production endpoint-recovery patch is justified yet.
Evidence: `.debug/usb-probe-jtag-state.log`, `usb-probe-jtag-registers.log`,
`usb-probe-jtag-seed.log`, `usb-probe-after-seed.log`,
`usb-probe-recovered.log`, `usb-probe-cable-swap-pnp.json`.

After recovery the probe passed3 consecutive20s stopped-reading cases,
receiving48906/48750/49086 bytes, with RX count40/52/64. Then the original
production BIN/ELF was restored and flash verification passed on native COM12.
Joint calibration0xFF00, motor calibration0x100 and ABS limits0xC00 were
restored/read back from the device-specific JSON; enabled=false, scopes0,
selections empty. The unchanged production ELF hash is69BE0D6F...470FC2C.
On the known-working cable,100 software reconnects and20s stopped-reading
recovery passed (`.debug/usb-known-cable-production/`). This result does not
isolate cable replacement from the intervening debugger experiment/reflash,
and does not imply physical or2h acceptance. Hardware tests remain separate.

### Physical replug reproduction and endpoint-only recovery

The FIRST physical replug on that same known-working cable reproduced zero
RX after10s (`.debug/usb-known-cable-physical/`, FAILED). A read-only live
debug inspection found business tasks alive, enabled USB interrupts0x20C,
EP1_CONF=0 and IN_EP1_ST=1 (empty read/write pointers). After clean debugger
exit, a separate5s serial read still returned0 bytes. Thus read-only debugger
attachment alone did NOT recover this occurrence.
Only setting EP1_CONF.WR_DONE, with no FIFO payload write, then recovered
266240 bytes/5s. No chip reset/flash/calibration restore intervened.
Evidence: `usb-physical-known-cable-readonly-jtag.log`,
`usb-physical-after-readonly.bin`, `usb-physical-flush-only-jtag.log`,
`usb-physical-after-flush-only.bin` under `.debug/`.

`tools/debug/patch_p4_hwcdc.py` now backports a guarded empty-endpoint commit
in the producer's existing HWCDC critical section before arming IN_EMPTY.
It acts only on P4 and the observed EP1_CONF=0/IN_EP1_ST=1 signature, publishes
an empty packet, and neither fabricates payload nor resets the peripheral or
chip. It leaves nonempty endpoint pointers alone. The script accepts only
the audited official SHA256 or its exact patched form, saves the original,
and rejects unknown modifications. `build_debug.ps1` resolves the installed
Arduino core version and applies/verifies this prerequisite. Tests for both
driver patches and debugger cleanup:10 passed. Patched HWCDC SHA256:
`5FFD3BF0421554C337362A74CBDECED9E1417F6C3D1EC7937E5EDFB2E177608B`.
The patched firmware's physical validation is pending; endpoint-only recovery
is proven for this occurrence, but durable prevention must still be tested.

The patched production firmware was built and uploaded/verified through
native COM12. ELF SHA256:
`6258195D4007FF6EFBA0BAB0D7B3C45A47E24779E4283D8FFE229A5D0A5DA812`.
The source CRC remains0x4D8F76FE because this change is in the separately
audited Arduino core; use the ELF hash to distinguish builds. Calibration
0xFF00/0x100, limits0xC00 and disabled/empty invariants passed after restore.
Initial patched acceptance passed10 reconnects and20s stop-reading recovery
(`.debug/usb-empty-in-initial/`); physical watcher is ready separately.
The FIRST patched physical replug PASSED: native COM12 protocol returned,
12s fresh monitoring passed, joint/motor masks remained0xFF00/0x100 and
control/scopes stayed disabled/empty. Boot diagnostics retained the same
boot2/reset11 from upload, uptime230..235s: this replug did not reboot the
P4. `.debug/usb-empty-in-physical/physical.jsonl` records the actual USB
absence edge and result. Remaining physical/cold/JTAG/soak cases are pending.
The complete patched physical run subsequently PASSED all10 actual unplug/
replug cycles, with valid protocol, calibration0xFF00/0x100 and disabled/empty
control on each12s recovery observation. This repeats the same known-cable
condition that failed on its first cycle before the empty-IN fix. Cold starts,
patched JTAG stress and the2h soak remain separate pending cases.
The5 actual full cold starts subsequently PASSED as well. Each had
boot1/reset1(POWERON), retained=0, valid protocol, restored0xFF00/0x100
calibration and disabled/empty control (`.debug/usb-empty-in-cold/`). A
separate active-RX unplug test is needed because the10-cycle observer closes
its connection before each unplug. `usb_live_unplug.py` observes that separate
case, requires RX/TX threads to close within5s, then12s fresh recovery.
The matching final BIN/ELF, upload markers, FQBN/options, calibration JSON,
driver diff and SHA256 manifest are archived in
`.debug/usb-empty-in-delivery-6258195D/`.
The active-RX unplug case also PASSED transport/thread lifecycle: the link
had received7.45MB/75042 packets before removal; both RX/TX threads closed
in0.118s, then the next connection received641399 bytes/6428 packets over
12s with compatible protocol and no transport error. Physical removal's
ClearCommError is expected; it did not leave threads running.
This particular run's initial/final snapshot masks were0/0, so it is NOT
evidence that the loaded8/1 calibration set survived. The tool now accepts
explicit expected masks and otherwise reports calibration_verified=false.
Known JSON calibration was restored/read back again before the final full
100-reconnect/10-JTAG/7200s-soak run, which requires0xFF00/0x100 throughout.
Evidence: `.debug/usb-empty-in-live-unplug/observations.jsonl`,
`.debug/usb-final-soak-calibration.log`, `.debug/usb-final-empty-in-acceptance/`.
The final run passed100 reconnects,10 JTAG sessions and20s backpressure;
its7200s soak was then STOPPED AT THE USER'S REQUEST. The last periodic soak
record was90.2s, not2h. The process was interrupted and confirmed exited;
do not count the long-run or its final reconnect as passed. Evidence and an
explicit cancellation marker remain in the final acceptance directory.
No recurring follow-up was created (the attempted creation was rejected).

## Changes

`gdb_batch.py` continuously drains OpenOCD output into an individual log.
Its readiness timeout no longer depends on a blocking `readline`. Both the
batch tool and manual OpenOCD launcher install `esp_debug_cleanup.tcl`:
remove all break/watchpoints on all targets BEFORE resuming halted cores.
For P4, all three trigger CSRs on each core are also explicitly cleared and
read back while halted: a killed GDB reproduced failed logical breakpoint
removal. If OpenOCD warns that its metadata is inconsistent, restart that
server before further debugging; batch sessions already shut it down.
The batch tool also polls hardware state after GDB exits and requires every
core to be running; failed cleanup is a nonzero result. OpenOCD is shut down
gracefully. No automatic board reset or motor authorization is added.
An ELF hash matching the upload marker and recorded USB identity are
mandatory. Reusing a server requires matching target family, adapter identity,
no flash-attach reset and the cleanup hook; rejection leaves that server alone.

Close native CDC monitors before starting OpenOCD. Use the matching flashed
ELF/BIN; halt/resume interrupts feedback, and motion must not resume
automatically. A GDB timeout can remain a failed debug inspection even when
hardware cleanup successfully restores communication.

The desktop records per-connection byte/frame counts, errors and RX/TX
thread status independently of rate counters. Failed startup distinguishes
no bytes, no valid frames, handshake rejection and stale status. Opening a
port records DTR/RTS disabled; recovery never toggles them automatically.

P4 low-rate existing text logs add `[USB_BOOT]`, `[USB_DIAG]`, `[USB_TASK]`
and `[USB_TX]`: reset reason, firmware source CRC, boot/phase/uptime,
heap/stack/task counters, USB reset/byte/command counts, last activity,
short/zero writes and maximum write duration. Short writes/zero writes are
observations, not asserted SDK timeout causes. No wire packet, control
formula, ROS2 topic or calibration format changes.

Diagnostic snapshots update RAM once per second, with text emitted every5s
through zero-wait log admission. `.noinit` keeps a checksummed previous
snapshot across compatible software resets. Power-on/brownout discard it;
ROM download/flash may overwrite it. The boot counter is therefore a
retained-session counter, NOT a persistent lifetime counter. No diagnostic
flash writes are performed. No independent UART logger is enabled until
the CH343 supply path is verified.

## Driver and board evidence

Before this investigation's empty-IN patch, installed Arduino-ESP323.3.10
HWCDC.cpp was byte-for-byte identical to
the official tag: SHA256
`c5ed5fdd05aa0df9b74d390812643599223f96b459256718d0ad328aeaba6a8a`.
Do not attribute its pre-existing upstream fixes to local modifications.
The new guarded commit is recorded separately in `.debug/HWCDC-empty-in.diff`.
The P4
SDK direct FIFO-write backport is separate and checked by the build script.
Backups, official-source comparison and manifest are under the directory
recorded in `.debug/usb-investigation-current.txt`.

The supplied v8d7 KiCad schematic netlist matches these PCB connections:

- USB VBUS and the buck output share `/5V`; reverse-current/power sequencing
  remains a hardware investigation item.
- D2 PRTR5V0U2X pin1 is `/USB_SHIELD_1`, not direct GND. Its relationship to
  the connector's physical shell and transient return must be checked.
- USB has two5.1k CC pulldowns and routes D-/D+ to H1 pins19/20. Actual
  signal integrity and supply transients cannot be established from the
  netlist or a successful ROM reset.

Official references: [Waveshare schematic](https://files.waveshare.com/wiki/ESP32-P4-Pico/ESP32-P4-Pico-datasheet.pdf),
[ESD pin definitions](https://assets.nexperia.com/documents/data-sheet/PRTR5V0U2X-Q.pdf),
[Espressif hardware guidance](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32p4/schematic-checklist-esp32p4.html).

## Repeatable acceptance

Disconnect GUI device links first. The script sends no motion targets,
requires disabled control/empty scopes and selections, and stops on the first
failure without resetting or flashing. Restore the backed-up calibration
through the normal verified import before requiring its expected masks.

```powershell
python tools/debug/usb_acceptance.py --port COM12 `
  --cycles 100 --jtag-cycles 10 --soak-seconds 7200 `
  --expected-manual 0xFF00 --expected-auto 0x100 `
  --output .debug/usb-final-acceptance
```

The default backpressure case stops host reading for20s, then requires a
fresh protocol connection. Preserve JSONL observations and each JTAG log.
Physical10 unplug/replug cycles,5 cold starts, unplug during connection,
bare-board/cable/power comparisons and supply measurements are separate
manual cases; software disconnects must NEVER be counted as physical cycles.

`usb_physical_acceptance.py --serial 80:F1:B2:D0:6B:A5 --cycles 10 --mode unplug
--output .debug/usb-physical` records real absence/presence edges and verifies
fresh state plus calibration restore. Run a separate5-cycle `--mode cold`
case: it additionally requires POWERON reset diagnostics, not just a USB
reconnect. Wait for READY/CYCLE_PASSED before the next physical action. The
observer never simulates an unplug, resets a board or enables motors.

`tools/debug/usb_probe/usb_probe.ino` is an explicit isolation sketch using
the production native adapter, incrementing text frames and no motor/CAN
code. Flash it only as a documented comparison, retain the production
BIN/ELF, and restore that matched pair and calibration afterwards.
