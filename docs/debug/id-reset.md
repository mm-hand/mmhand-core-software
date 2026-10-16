# Settings: motor and PaXini tactile IDs

The five main pages are Calibration, Control, Replay, Plot and Settings.
Calibration / Joint Calibration opens by default. Settings contains Renumber
Motor, Renumber Tactile, Control Parameters and Diagnostics.

## Motor inventory and renumbering

Connect P4 and wait for fresh status. Scan All Buses probes BUS0–BUS3, IDs0–30:
124 addresses. Enter New IDs and press Renumber All. New IDs are0–252 and
must be globally unique;253 is the temporary address,254 is broadcast.
Occupied targets above30 and the temporary address are checked separately.
The host orders moves, including exchanges and cycles, and journals each intent
before sending one step to P4. A timeout stops the batch without blindly
repeating an EEPROM write. Rescan to reconcile actual addresses before recovery.

Renumber All also sets registers84/85/86 to254/254/0 for every scanned motor,
including unchanged IDs: maximum speed, maximum acceleration, acceleration
multiplier. ID and profile readback must all pass before success is reported.
Affected Auto/LUT, ABS limits and multiturn anchors become invalid. Manual
joint endpoints remain. Recalibrate before control; fixed product bindings do
not change.

**Multiple motors with the same ID on one bus cannot be identified or
renumbered separately. Connect them separately before renumbering.**

## Five-finger tactile inventory and renumbering

Connect S3 directly by its USB Serial/JTAG port. P4 relayed tactile data does
not authorize maintenance. Scan All Fingers probes15 reachable addresses:
three IDs on each of Little, Ring, Middle, Index and Thumb. The editor groups
results by finger; each row has Current ID and a New ID input. Blank means
unchanged. Enter0,1 or2, then Renumber All. IDs may repeat across finger buses,
but final IDs must be unique within each bus. The GUI lists Thumb first;
physical bus0 is Little and bus4 is Thumb.

Position convention: Fingertip0, Distal segment1, Proximal segment2; vendor
selector values are respectively1,2,3. Known installed models are DP-S2015-Elite
PXSR-STDDP03G (52 taxels, tip) and IP-S1610-Elite PXSR-STDIP03B (25 taxels, pad).
S3 preserves the installation/model route independently of the new ID and
validates full-data CRC with the registered model length. It does not claim to
read a manufacturer model identifier. Unknown models cannot be renamed.

**Multiple tactile sensors with the same ID on one finger bus cannot be
identified or renumbered separately. Connect them separately before renumbering.**
Identical simultaneous replies can look like one device; scanning cannot prove
that no duplicate exists. This limitation applies to motors too.

The desktop sends version2 maintenance requests directly to S3; P4 has no
such command. The tactile task owns SPI and handles requests between acquisition
rounds. Each write sends the manufacturer unlock7A01000100A1CE and rename
7E23000100(dest_id)(CRC8), with dest_id=New ID+1. It sends once, checks a
CRC-valid full read at the new selector and no reply at the old selector, then
persists the model route. Page ID 0/1/2 maps to selector bits 01/10/11,
exactly the manufacturer destination 1/2/3. The initial installation route
remains tip=selector3, distal=selector2, proximal=selector1; changing an ID
moves that installation route rather than changing its model. Route storage
uses the current `pax-route-v2` namespace; the obsolete reversed index encoding
is not read. No speculative save command is sent.

Maintenance operation2 reconciles a journaled installation route without
sending sensor unlock or ID writes. The host uses it only after the old ID is
absent and the destination is present, preserving the recorded model and
semantic slot. Live verification probes for up to1s after a single write;
ambiguous results stop the batch.

Moves to free addresses are ordered first. A two-sensor exchange can use the
third reachable ID. With all three IDs occupied, a cycle requires disconnecting
one sensor because no reachable temporary ID remains. Global STOP cancels
subsequent steps. Interrupted batches are journaled against S3 USB identity;
rescan before resuming. Success means live readback passed. Power-cycle the
sensor and scan again to verify nonvolatile persistence; sending bytes alone
never counts as a successful ID change.

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
