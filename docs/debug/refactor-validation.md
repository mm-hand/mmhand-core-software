# Refactor validation — 2026-10-15

This records observed results; it does not certify untested motion behavior.

- Full regression under MSVC: 1350 passed, including desktop, native production
  control, protocol and firmware harnesses. Log: `.debug/refactor-all-msvc.txt`.
- Leaf-page toolbars now remain below the subpage navigation. Removed the
  rejected firmware-calibration command (0xCA) from both ends and reject unknown
  desktop commands before queueing; retained START for the active host
  calibration workflow. Deleted unused legacy PID position buffers.
- Serial writes now have a 500 ms timeout and incomplete-frame writes fail
  explicitly; 39 communication/handshake tests passed after this addition.
- Native production PI/filter/collision test passed under MSVC. Existing Direct
  ABS Zero harness was adapted to the new dispatcher and passed27 cases.
- Safety closure:77/77. Upper protocol and control-parameter generated files current.
- P4 and S3 were built with the Arduino debug configuration and flashed with
  verified Flash hashes. Matching ELF/hash/adapter identity markers are in.debug.
- Latest P4 debug ELF SHA256:
  `0FE544E8B738944A9FC9B695377C37A6C1AD488EEE979E1370BC9329025D58E8`.
- P4 handshake reports protocol4. Whole parameter revision1 with CRC19A723CD
  was applied and separately queried successfully.
- P4 local device-bound calibration for J12–J15 was restored after the final
  Flash. Readback:Manual=0xF000, Auto=0xF000, motor limits=0xF8000. Active joint
  and motor masks remain0, control disabled. Source calibration and multiturn
  records were backed up before firmware changes.
- Motor scan:BUS0–BUS3, ID0–30,124 addresses in about2.18s. BUS0 IDs15–19 found.
  On these five motors, registers84/85/86 were set to254/254/0 and independently
  read back. No motor ID was changed.
- S3 maintenance v2 scan initially returned one CRC-valid25-point IP installation
  at physical bus0(Little), current ID1. All15 addresses were scanned; no tactile
  ID write was sent.
- S3 JTAG thread inspection succeeded. A subsequent breakpoint-at-task-entry
  timed out; Windows GDB launcher child cleanup was fixed and regression tested.
  S3 temporarily stopped enumerating with a Windows USB descriptor error.
  On the subsequent retest it recovered as COM14, adapter10:B4:1D:CD:88:5C.
- Desktop is running at http://127.0.0.1:8080 with no preselected devices or scope.

## Still awaiting verification

- Sensor power-cycle persistence and physical placement/model confirmation
  for additional sensors; only one reachable sensor was found in this retest.
- Feedback/Direct handover, sustained control, reversal, integral and collision
  response on hardware with valid S3 encoder feedback. No motion test was run.
- Repeat calibration transfer under load:intermittent ACK timeouts occurred
  during bring-up. ACK retention was corrected to preserve per-command results;
  the latest write sequence also encountered a contract rejection and ACK
  timeouts with a blocked Windows serial write. All four LUT writes eventually
  acknowledged; a separate connection verified Manual=0xF000, Auto=0xF000,
  limits=0xF8000, scope=0 and control off. Bounded host writes prevent an
  indefinite wait, but the underlying intermittent transport issue remains.
- AI bilingual guide publication:generator requirements/extraction/validation
  were updated. Generated README/HTML were not hand-edited or published from
  uncommitted sources; committed-source annotations/audit/publication remain.

## S3 retest and correction

- All15 selectors answered maintenance requests; one sensor responded on Little
  bus at page ID1. Three complete scan rounds and17.19s of continuous capture
  yielded3018 valid serial frames and682818 bytes. Full-packet CRC errors,
  sequence mismatches and dropped assemblies were all0.
- A single attempted1→2 ID write revealed reversed selector indexing:
  page IDs were incorrectly encoded as3/2/1 while write destinations were1/2/3.
  The sensor moved to selector3, which the old scan mislabelled page ID0.
- Corrected selector encoding to1/2/3. Read-after-write now polls up to1s,
  without resending either write. Added maintenance operation2 for journal-based
  route reconciliation, with no sensor EEPROM write. The host preserves model,
  point count and semantic location from the interrupted transaction.
- Verified reconciliation at actual page ID2 as IP configuration,25points,
  semantic1. Corrected firmware then changed2→1 successfully: stage4/status0,
  old_valid=false, new_valid=true. Independent scans found ID1 and no ID2.
  The sensor is currently restored to its original ID1.
- Route persistence uses namespace pax-route-v2 so the reversed old index
  encoding is not interpreted as current mapping; default installation bits
  remain3/2/1 for tip/distal/proximal respectively.
- Added regression checks for page-ID/selector equivalence, default routing
  and model preservation after interrupted moves. Sensor power-cycle persistence
  is still awaiting the user's complete sensor power removal and rescan.

- After the user confirmed complete sensor power removal/restoration, Windows
  still listed COM14 but serial configuration failed twice (OS error31).
  No competing OpenOCD or test process was running. `pnputil /restart-device`
  was rejected by Windows with Access denied. Persistence is not inferred from
  this USB failure; the sensor has not yet been read after power removal.
- The final route-namespace build completed, ELF SHA256
  `55306F23BF9C3229EE107B799FEB2A040EA403AD17A0D51CAF290DFC859E5EF4`.
  Upload of this final build is pending COM14 recovery. The preceding corrected
  selector build was flashed and passed the live2→1 write/readback test.

- Final full regression after the S3 selector and route recovery changes:
  1350 passed; `.debug/refactor-all-msvc.txt`.

- After the next user-confirmed S3 USB unplug/replug, only P4 COM12
  enumerated. Windows showed an unknown USB device with descriptor-request
  failure; S3 serial and JTAG interfaces were absent. Sensor-disconnected,
  direct-PC USB isolation was requested before further flash attempts.

- User confirmed sensors disconnected and S3 directly connected to the PC.
  S3 still did not enumerate; Windows reported problem code43 for the unknown
  USB device. Manual BOOT+RESET ROM-download entry was requested to isolate
  application firmware from USB enumeration. Final S3 upload remains pending.

- After the user pressed BOOT/RESET, the PC still enumerated only P4 COM12.
  S3 ROM-download entry could not be confirmed; no S3 serial or JTAG adapter
  was present. Board model and availability of a separate UART USB interface
  were requested before selecting an alternative recovery path.

- Board identified by user as Waveshare ESP32-S3-Nano. Official board FAQ
  specifies B1-to-GND before power-on for recovery; prior generic BOOT-button
  instructions were not board-specific. Recovery guide updated with official
  schematic/FAQ links. Awaiting physical B1/GND strap and ROM enumeration.

- After user-confirmed B1/GND short and power-on, no serial ports enumerated.
  Windows hardware ID was USB\DEVICE_DESCRIPTOR_FAILURE, problem code43; no
  S3 JTAG or active OpenOCD/esptool was present. Bare Nano isolation from the
  expansion board and all external wiring was requested. ROM entry and the
  cause of the failure remain unconfirmed; no further flash was attempted.

- User confirmed bare Nano with B1/GND jumper. Windows detected a new failing
  device instance at Port_#0007.Hub_#0002, still USB\DEVICE_DESCRIPTOR_FAILURE
  and problem code43. No serial/JTAG interface enumerated. This does not
  establish firmware/debug state as the cause. Availability of a 3.3V-level
  USB-to-UART adapter was requested for independent ROM boot-log diagnosis.

## S3 final firmware upload after USB recovery

- S3 recovered as COM14, USB adapter10:B4:1D:CD:88:5C. Uploaded the final
  route-namespace build with verification; ELF SHA256:
  `55306F23BF9C3229EE107B799FEB2A040EA403AD17A0D51CAF290DFC859E5EF4`.
  The build and flashed markers now match. Logs: `.debug/s3-final-upload.log`.
- The firmware ran after reset and answered all15 maintenance scan addresses.
  Three repeated scan rounds found no sensors (the last requested wiring state
  had sensors disconnected). No sensor ID write was sent in this retest.
- Continuous capture for17.17s received2967 valid protocol frames,720585 bytes,
  with0 full-packet CRC errors,0 sequence mismatches,0 dropped assemblies.
  Sensor valid mask, summary-success mask and full-success mask remained0;
  USB link online is not a claim of a connected sensor.
- Sensor reconnection was requested to verify the previously restored ID1
  after complete power removal, real taxel data and preserved semantic/model
  routing. These sensor-level checks remain pending.

## S3 connected-sensor acceptance after complete power removal

- User reconnected tactile sensors and supplied power. The final flashed build
  answered all15 addresses; three scan rounds consistently found:
  Little bus ID1: registered IP/25points/semantic1;
  Little bus ID2: registered DP/52points/semantic0;
  Thumb bus ID1: registered IP/25points/semantic13.
  These are installation configurations validated with CRC-valid reads, not
  independent hardware-model identity reads. Little ID1 remained readable
  following the user's earlier complete sensor power removal/restoration.
- Read-only continuous capture ran47.06s, receiving8533 valid frames and2068353
  bytes, with0 full CRC errors,0 sequence mismatches and0 dropped assemblies.
  Three repeated inventories matched. The SVG renderer produced102 taxels,
  matching52+25+25 valid sensor points and manufacturer XY geometry.
  Evidence: `.debug/s3-connected-acceptance.json`,
  `.debug/s3-connected-final-scan.log`, `.debug/s3-live-tactile-hand.svg`.
- No sensor ID write or motor movement was sent in this acceptance run.
- Simultaneous independent P4 readback reported no live CAN tactile modules,
  encoder_online_bitmap=0, encoder_fault_bitmap=0x1FFFFF, active scopes0 and
  control disabled. USB-S3 tactile acquisition passed; live S3→P4 CAN relay,
  encoder feedback and closed-loop motion acceptance remain unverified.

## P4 power-on and CAN/USB follow-up

- User confirmed P4 external power. S3 CAN diagnostics reported RUNNING,
  no transmit enqueue failures and no recovery in the observed interval.
  P4 raw packets contained tactile mask0x2003. A dual-board observation found
  J08–J15 online and valid CAN snapshots with0 CRC errors/overruns; subsequent
  observations also captured stale intervals. CAN is not simply disconnected.
- Added bounded10s CAN health text to S3 and P4. S3 health records are visible
  in the desktop System Log without enabling noisy encoder/SPI dumps.
  S3 flashed ELF:0A040B0CAF2FDEC8BB955B0233BAE19EA2DD21223A261568EDA08D591E30E51F.
- Latest S3 direct-USB retest: three sensors remained present,3104 valid
  frames/746805 bytes over17s,0 CRC/sequence/drop errors, mask0x2003.
- Desktop/P4 tests still reproduced intermittent USB write timeout and missing
  calibration ACKs. RAM calibration was fully restored/read back during one
  session, but later reset/failed transfers mean it must be restored and checked
  again before motion. Local JSON and the original backup remain intact.
- Full regression after the CAN log changes:1351 passed. No motor enable or
  movement was sent in these tests.
- An A/B experiment replacing P4 Arduino HWCDC with the IDF native USB driver
  did not re-enumerate the P4 serial port. It was rejected and the exact HWCDC
  source restored from the pre-experiment copy. A matching rollback build is
  being prepared; manual P4 BOOT/RST was requested to recover the flash channel.
  No experimental native-driver implementation is retained in source.

## P4 rollback upload and runtime recovery follow-up

- After the user entered ROM download mode, COM12 returned and the HWCDC
  rollback upload completed with flash verification. The flashed ELF marker
  matches SHA256
  `9596E7F56BCE6C12BC108402CA8BE3453EE08FBB070FA8A0B526BF45718AE3F5`.
  Evidence: `.debug/p4-usb-rollback-upload.log`.
- Runtime handshake did not complete, including after the user pressed RST.
  Calibration restore stopped before its first command because the protocol
  gate was not ready; no successful restoration is claimed. The local schema6
  calibration snapshot remains the restoration source.
- A bounded JTAG inspection connected to the correct P4 adapter and detached
  with no breakpoints installed. It did not establish the runtime failure's
  cause. A subsequent no-reset ROM probe also encountered a USB write timeout;
  it did not establish that the chip remained in ROM download mode.
- Requested complete removal/restoration of P4 USB and external power to
  re-enumerate the USB connection. Runtime handshake, calibration readback and
  sustained CAN freshness remain pending. No motor enable/motion was sent.

## USB recovery: independent S3 acquisition and P4 full-duplex FIFO fix

- P4 COM12 was verified as native USB Serial/JTAG adapter
  `80:F1:B2:D0:6B:A5`; the CH343 connector was not used.
- The S3 optional USB stream blocked sensor acquisition when the host stopped
  reading it. Before correction, a 12s USB-closed phase had zero encoder-online
  mask in112/120 samples; after closing an open port,133/150 samples were zero.
  Removing the 21-channel encoder text dump and configuring nonblocking HWCDC
  output with a4096-byte TX buffer fixed this dependency.
- S3 uploaded/verified ELF SHA256:
  `47E8E7E25216EF032A571615E78E7CE7AEBE518584CCDFB792CB7A3FCBBD22B0`.
  The same closed/open/closed test then recorded419/419 valid tactile masks
  (`0x2003`) and J08–J15 online in every sample. CAN encoder reception stayed
  at2500 snapshots/10s; CRC, RX overrun and current bus-error counters were0.
  Other encoder channels still alternate between valid and physical read
  errors; this is not acceptance of all21 installed encoder channels.
  Evidence: `.debug/p4-s3-usb-independent-retest.log` and
  `.debug/p4-s3-usb-dependency.json` (latest run).
- Independent S3 USB acquisition after this flash received3321 valid frames /
  685601 bytes over approximately17s, with0 CRC/sequence/drop errors and the
  same three sensor installations. No sensor ID writes were issued.
  Evidence: `.debug/s3-usb-independent-stability.log`.
- P4 still lost host command bytes during full-duplex communication. Its
  affected SDK TX FIFO helper performed a register read/modify/write; reading
  EP1 consumes RX data. Upstream now uses a direct register write. The narrow
  backport in `tools/debug/patch_p4_usb_fifo.py` preserves the original SDK
  header and rejects unknown implementations; the debug build checks/applies
  it automatically. The Arduino core object was rebuilt, not reused.
- P4 uploaded/verified ELF SHA256:
  `7E582453ED9B4F153D7DEC954DA4E1B38D14A36ECA3D5FF09F7AC74FE796A65F`.
  All ten calibration-load/query ACKs arrived; independent readback confirmed
  Manual0xF000, Auto0xF000, limits0xF8000, J12–J15 calibration results.
  Evidence: `.debug/p4-usb-fifo-fixed-restore.log`.
- Host ACK preparation now invalidates only the preceding reply for that
  command before enqueueing. This prevents reuse when adjacent Windows
  wall-clock timestamps compare equal. Same-command transactions still must
  be serialized because the wire ACK has no transaction ID. ROS2 filtering
  uses `perf_counter` for fine timestamps on Windows Python before3.13.
- J13/M17 current-pose hold acceptance passed Feedback → Direct → Feedback,
  each confirmed by actual P4 state/owner/enabled telemetry,117 monitored
  samples. Maximum observed angle deviation was0.110degrees; measured motor
  position stayed2195 (0-count change). Control was released and the final
  state was READY, disabled, ownerNONE. No changed-position target was tested.
  Early test cleanup wrongly required a queued ACK to have status0 and then
  attempted an unsupported empty D7 scope; these harness issues were corrected
  before the successful acceptance run. No firmware admission gate was relaxed.
  Evidence: `.debug/p4-usb-fifo-fixed-hold-verified.log` and
  `.debug/p4-hold-handover-acceptance.json`.
- Full regression after these source/tool changes:1354 passed using the
  repository MSVC/Python3.13 environment. Larger-motion, reversal, integral and
  physical collision acceptance and committed-source AI-guide publication
  remain outstanding; a successful hold test does not establish these results.
- Three subsequent normal desktop-backend reconnects (the installed
  Python3.12 desktop environment) passed parameter revision/CRC verification
  and300/300 fresh status samples over30s. J12–J15 remained online, tactile
  mask0x2003 remained valid, Manual/Auto masks remained0xF000. Both selections
  were empty on every connection and no control was enabled. Evidence:
  `.debug/p4-usb-fixed-reconnect-{1,2,3}.json`.

### 2026-10-16: selected-finger calibration and reconnect fixes

- Motor Calibration workspace preparation now includes only selected joints.
  Unknown coordinates on an unselected finger no longer block or move that
  finger. Selected motor feedback, coordinate, route and overload checks remain.
- The host calibration ownership check previously treated the global
  SERVO_OFFLINE bit from unrelated absent motors as loss of control. This
  produced the generic J08 feedback-lost failure. Offline/unknown checks are
  now evaluated within the actual calibration motion scope; global overload
  remains blocking.
- Actual J08 Motor Calibration passed twice: about28s before flashing and25s
  on the final P4 firmware, including Rope Init and dual LUT zero finish.
  Manual bitmap0xFF00 (J08–J15) and all endpoint arrays match the pre-test
  backup. J08 Auto bitmap0x100 and motor limits0xC00 were saved and restored.
  Evidence: `.debug/j08-motor-debug.log`,
  `.debug/j08-postflash-motor-debug.log`,
  `.debug/pre-j08-debug-20261016-110257/`.
- P4 bootstrap no longer waits indefinitely for a USB host read indication.
  The desktop P4 connect operation requires fresh compatible status within10s.
  Native USB-JTAG COM12 flash verification passed; matching ELF SHA256:
  `F3B8566F60B9820154E830483AFDC7B2A341256961FAA61583834F9E64E4EA4E`.
  S3 firmware was unchanged in this pass.
- Three P4/S3 software disconnect/reconnect cycles passed, with90/90 paired
  samples Online, parameters applied, selections empty and control disabled.
  Evidence: `.debug/j08-reconnect-test.json`. Physical unplug recovery was
  not exercised in this pass and still requires separate acceptance.
- Port selectors no longer retain disconnected missing devices. Enumeration
  failures clear stale choices. A physically connected COM remains listed
  after software Disconnect, as expected.
- Fixed the S3 snapshot/RX timestamp race that could show no valid frames
  despite incoming valid data. Load Calibration now displays device-specific
  and earlier default paths and describes automatic saving. Joint capture
  success wording now says joint calibration saved / motor calibration needed.
  GUI was restarted, both boards connected Online and all motors left disabled.
- Full regression:1358 passed. Additional enumeration-failure regression and
  application suite:58 passed. Unplugged-choice, scoped fault and future RX
  timestamp cases have explicit regression coverage.

### 2026-10-16: subsequent P4 COM12 no-data incident

- User reported Connecting / Waiting for data. USB identity still matched
  P4 serial80:F1:B2:D0:6B:A5. Independent raw serial observation received
  zero bytes for3s, while a one-byte write completed. Thus enumeration/open
  success did not establish a working firmware connection.
- Matched-ELF JTAG capture showed PC0x4fc00b10, unavailable stack trace and
  inconsistent task records. This is an abnormal observation, not sufficient
  evidence to identify its trigger or conclusively classify download mode,
  memory corruption or a USB failure. Captures:
  `.debug/p4-no-rx-{jtag,registers,tcb}.log`.
- A ROM chip-id transaction and full system reset recovered178377 received
  bytes over4s including SYS_READY and binary telemetry. No flash write or
  firmware change was required. Calibration restore verified Manual0xFF00,
  Auto0x100, limits0xC00, parameters applied, ownerNONE, scopes0 and disabled.
  Evidence: `.debug/p4-no-rx-rom-reset.log`,
  `.debug/p4-no-rx-restored.log`. Recurrence trigger remains unconfirmed.
- A second JTAG comparison could enumerate all12 normal tasks and source
  stacks, but took an unusually long time; serial protocol was unavailable
  after detaching. Repeated ROM reset recovered it again. This debugger-side
  interruption is separately observed and must not be presented as proof of
  the trigger for the original user incident. Final restore evidence:
  `.debug/p4-no-rx-restored-final.log`. GUI COM12 then returned Online and
  displayed Manual8/21, Auto1/21. No motor was enabled during diagnosis.

### 2026-10-16: debugger-exit failure reproduced without resetting

- Baseline BIN/ELF, device JSON, Arduino build options, official/local HWCDC
  comparison and SDK metadata were backed up under
  `.debug/usb-investigation-20261016-131807/`. Installed HWCDC is identical
  to official Arduino-ESP32 3.3.10; the known SDK FIFO fix is separate.
- Old timeout/rescue reported GDB detached, but OpenOCD directly found both
  P4 cores halted at0x4ff0275a. Explicit hardware resume alone recovered
  106295 serial bytes over2s; no ROM reset/flash was needed. This confirms
  one software mechanism behind no-data incidents.
- Forced hardware-breakpoint timeout additionally reproduced failed
  OpenOCD breakpoint removal. P4 tdata1=0x2900104c and tdata2=0x40005ffa
  retained an enabled bootstrap breakpoint. Direct clearing/readback of all
  three triggers on each core returned disabled controls0x21000000.
- The two launchers now install a detach cleanup hook. GDB timeout cleanup
  uses OpenOCD directly, not a second symbol-less GDB; the latter also
  reproduced an internal FreeRTOS thread-selection assertion. All trigger
  access is performed halted, before any resume, with running-state polling.
  Failed verification remains a failed result, never an automatic reset.
- IMPORTANT correction: the earlier inconsistent GDB virtual-thread PC/TCB
  readings alone do not establish chip memory corruption or download mode.
  In some such sessions firmware resumed normally. Hardware state and serial
  behavior must corroborate debugger context output.
- Final tool flow on original firmware passed100 software reconnects,
  10 JTAG inspections/clean exits/reconnects,20s stopped reading/recovery,
  Manual0xFF00/Auto0x100, disabled control and empty scope/selection throughout.
  Evidence: `.debug/usb-tools-final-acceptance-restored/`. An initial run
  correctly stopped on missing RAM calibration after the user's reconnect;
  the backed-up JSON was imported/verified before the successful run.
- Forced GDB timeout after the final cleanup change returned failure for the
  timed-out inspection but verified both cores running. That run did not yet
  include reset diagnostics; it cannot exclude the vendor implicit attach reset.
  Evidence: `.debug/usb-jtag-forced-timeout-final.log`.
- Full regression1363 passed; subsequent connection-detail changes passed
  100 targeted desktop tests. Diagnostics firmware build and additional
  physical/cold-start/2h acceptance are tracked separately; they are not
  implied by these tool tests. See `usb-stability.md` for reproducible commands.

### Diagnostic firmware follow-up

- Final diagnostic ELF SHA256:
  `69BE0D6F0862355A484CAF55EB1BFBB3D145D76E21B479674A7F21DC1470FC2C`;
  firmware source CRC `0x4D8F76FE`, uploaded successfully through COM12.
-100 reconnects passed, but the next stock GDB attach reset the P4. OpenOCD
  reported memory-protection `reset halt`; retained diagnostics reported
  `reset=12` (JTAG). The corresponding acceptance run FAILED.
- Set `ESP_FLASH_SIZE 0` before board config in both launchers, keep hardware
  breakpoints and strict uploaded ELF/USB identity checks.10 subsequent JTAG
  sessions/cleanups/reconnections preserved calibration and passed.
- The subsequent20s stopped-reading test FAILED with Windows port configure
  error31. A user USB replug restored communication. The first monitored
  physical cycle then FAILED with ClearCommError/Windows error22 before any
  bytes arrived. No complete physical/cold-start/2h pass is claimed.
- Windows topology identifies P4 behind `USB\\VID_35D6&PID_2510` hub. This is
  a pending comparison variable, not a proven cause. PnP status was OK despite
  serial failure; absence of a PnP problem code is not communication success.
- Added debugger marker/reset-hook/rejected-server tests:8 targeted tests
  passed. Evidence: `.debug/usb-live-attach-acceptance/`,
  `.debug/usb-physical-unplug/`, `.debug/usb-physical-failure-*.json`.
