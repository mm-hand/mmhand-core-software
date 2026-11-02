# Settings: motor IDs

The five main pages are Calibration, Control, Replay, Plot and Settings.
Calibration / Joint Calibration opens by default. Settings contains Renumber
Motor, Control Parameters and Diagnostics.

Tactile sensor renumbering was removed: the PaXini modules on this hand do not
accept the documented SPI ID write (correct unlock/rename frames, 1 MHz and
100 kHz, split and single CS session, unlock bytes A1-A3 — the address never
changed, including after a power cycle). Tactile data still arrives over the
direct S3 USB link; only the ID editor is gone.

## Motor inventory and renumbering

Connect P4 and wait for fresh status. Scan All Buses probes BUS0–BUS3, IDs0–30:
124 addresses. Enter New IDs and press Renumber All. New IDs are0–252 and
must be globally unique;253 is the temporary address,254 is broadcast.
Occupied targets above30 and the temporary address are checked separately.
The host orders moves, including exchanges and cycles, and journals each intent
before sending one step to P4. A timeout stops the batch without blindly
repeating an EEPROM write. Rescan to reconcile actual addresses before recovery.

Firmware/configuration checks run automatically when a motor comes online.
Renumber All only changes and verifies IDs. It does not read or write registers
84/85/86. After maintenance ends, rediscovery performs the common online check.
Affected Auto/LUT, ABS limits and multiturn anchors become invalid. Manual
joint endpoints remain. Recalibrate before control; fixed product bindings do
not change.

**Multiple motors with the same ID on one bus cannot be identified or
renumbered separately. Connect them separately before renumbering.**

## Control and calibration

No joint or motor is selected at startup or reconnect. ENABLE Selected releases
the old control scope and holds the measured current pose in the new mode.
ENABLED follows P4 confirmation. Feedback and Direct remain enabled until a
scope/mode change, STOP, disconnect or fault. There is no Pause or Exit Control.
STOP latches; Clear Faults is explicit and never restores movement.

Control Parameters stores defaults/current JSON under desktop/resources/config.
The host sends and separately queries the entire revision/CRC before allowing
control. ROS2 input is /mmhand/joint_angle_deg, Float32MultiArray of21 degrees
[J00_deg,...,J20_deg]. Only ROS2 input uses One Euro plus fixed-anchor hysteresis.

Set Min/Max uses a fresh click-time raw snapshot, or waits at most1s for a
sample under300ms old. The row shows capture/save/verification and disables
repeat requests. Motor Calibration highlights only the current joint as
CALIBRATING. Connection Rx age measures age of a valid frame, not round-trip
latency; only direct S3 frames update its own link clock.
