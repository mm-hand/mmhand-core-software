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

Connect P4 and wait for fresh status. **Scan (0–30)** probes 124 addresses
across BUS0–BUS3. **Scan (0–253)** probes 1016 addresses on the same buses.
Run either scan after changing which motors are connected.
Enter New IDs and press **Renumber All**. New IDs range from 0 to 252 and must
be globally unique. A motor found at 253 can be assigned a new ID.
The host checks source and target addresses before writing and uses 253 only
to resolve an exchange or cycle. If a write fails or times out, use
**Scan (0–253)** to read the current addresses and enter new targets.

Firmware/configuration checks run automatically when a motor comes online.
Renumber All only changes and verifies IDs. It does not read or write registers
84/85/86. After maintenance ends, rediscovery performs the common online check.
Affected Auto/LUT, ABS limits and multiturn anchors become invalid. Manual
joint endpoints remain. Recalibrate before control; fixed product bindings do
not change.

**Multiple motors with the same ID on one bus cannot be identified or
renumbered separately. Connect them separately before renumbering.**

Hold **Buzz** to move the motor about 11.25° away from its starting position and
back repeatedly. Releasing the button lets the current pulse finish.

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
