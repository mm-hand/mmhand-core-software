# Host error catalog

Every error the desktop app shows is rendered as three paragraphs:

```
What you see: <the headline the app displays>
<detail line, when the app has one>
Cause: <plain-language reason>
Fix:
1. <first step>
2. <second step>
```

Source of truth: `desktop/resources/config/safety_guidance.json` (`host` section).
If the text is edited there, update this file. P4 device events (`P4-0000` style codes,
73 entries in the `rules` section) are a separate family and are not listed here.

## A. The 51 UI codes

### HOST_ABS_REBASE_FAILED

**What you see:** Motor position reset did not complete

**Cause:** The controller reported that position reset failed. The specific cause is not confirmed.

**Fix:**
1. Open Details to identify the affected motors.
2. Check current motor status before repeating the reset.

### HOST_ACTION_FAILED

**What you see:** The app could not complete this action

**Cause:** The exact cause is not confirmed by the available result.

**Fix:**
1. Open Details for the original device response.
2. Save the session log if the problem continues.

### HOST_CALIBRATION_FAILED

**What you see:** Calibration did not complete

**Cause:** The affected joints are not confirmed calibrated.

**Fix:**
1. Open Details for the failed phase and affected channels.
2. Correct the reported cause, then calibrate the affected joints.

### HOST_CALIBRATION_MISSING

**What you see:** Selected joints need calibration data

**Cause:** The selected joints are not ready for this action.

**Fix:**
1. Open Joint Calibration and check the selected joints' Manual Min and Max.
2. Open Motor Calibration and calibrate the affected joints.

### HOST_CANCELLED

**What you see:** The action was cancelled

**Cause:** The action ended before completion; earlier completed steps may remain applied.

**Fix:**
1. Check the current device state before starting the intended action again.

### HOST_CONNECTION_LOST

**What you see:** The app is not receiving current device data

**Cause:** Device status cannot be confirmed.

**Fix:**
1. Check the USB connection and controller power.
2. Click Connect and wait for current device data.

### HOST_ELECTRICS_CAN

**What you see:** No joint position data is arriving over CAN

**Cause:** The app cannot read the magnetic encoders while the CAN link or its wiring is down.

**Fix:**
1. Check CANH and CANL between S3 and P4 for swapped, open, or loose wires.
2. Check the common ground and the 120 ohm terminations at both ends.
3. Check that both CAN transceivers are powered and not in standby (STB tied low).
4. Check that the S3 board is powered.

### HOST_ENCODER_WIRING

**What you see:** A magnetic encoder is not reporting a valid position

**Cause:** The affected joint has no usable raw feedback even though CAN is working.

**Fix:**
1. Check the finger connector, the sensor board power, and the magnet on the affected joint.
2. Check the shared MISO/SCLK/MOSI wires for the affected finger.
3. Open Diagnostics for the per-joint raw and latched encoder error code.

### HOST_FEEDBACK_RECOVERY_FAILED

**What you see:** Calibration could not obtain reliable motor feedback

**Cause:** Calibration stopped before the selected motors could be prepared safely.

**Fix:**
1. Check the affected motors show Online and that live position readings update.
2. Check motor power and bus connections.
3. If a scan finds all motors but live feedback is still missing, save the session log for support.

### HOST_FIRMWARE_MISMATCH

**What you see:** The app and device versions do not match

**Cause:** This connection cannot be used for control yet.

**Fix:**
1. Install matching app and controller firmware versions.
2. Reconnect and wait for the version check to complete.

### HOST_MANUAL_INVALID

**What you see:** The Manual endpoints could not be used

**Cause:** The entered or captured joint endpoints are not valid.

**Fix:**
1. Open Joint Calibration and check the affected joint's Min and Max.
2. Capture valid endpoints in the indicated direction and save them.

### HOST_MANUAL_MISSING

**What you see:** Selected joints need Manual Min and Max values

**Cause:** The selected joints are not ready for this action.

**Fix:**
1. Open Joint Calibration and check the selected joints' Manual Min and Max.
2. Open Motor Calibration and calibrate the affected joints.

### HOST_MOTORS_NOT_READY

**What you see:** Motors are not ready for this action

**Cause:** The controller has not finished checking these motors, or a latched STOP is blocking the check.

**Fix:**
1. Wait a few seconds for the motor online check to finish, then retry.
2. If it persists, check the motor bus and power for the motors shown in Details.
3. Reconnect P4 if a latched STOP cannot be cleared.

### HOST_OPERATION_BUSY

**What you see:** Another action is still finishing

**Cause:** The app cannot start this action yet.

**Fix:**
1. Wait for the current action to finish.
2. To cancel it, use Stop; clear the resulting stop before enabling again.

### HOST_PARAMETERS_INVALID

**What you see:** Some control parameters are not valid

**Cause:** The controller cannot use this parameter set.

**Fix:**
1. Open Control Parameters and correct the field shown in Details, or use Restore Defaults.
2. Apply the parameters before continuing.

### HOST_PARAMETERS_PENDING

**What you see:** Control parameters have not been applied yet

**Cause:** Wait for parameter synchronization before starting calibration or control.

**Fix:**
1. Open Control Parameters and wait for Applied.
2. If synchronization failed, open Details and correct the reported parameter.

### HOST_QUALITY_WARNING

**What you see:** Calibration found a possible assembly issue

**Cause:** A quality warning was reported; the mechanical cause has not been measured.

**Fix:**
1. Check the affected joint, tendon routing, and preload.
2. Open Details for the measured result before recalibrating.

### HOST_RESULT_PARTIAL

**What you see:** Only part of the action completed

**Cause:** Some channels completed the action; others did not.

**Fix:**
1. Open Details to check the completed and failed channels.
2. Resolve the reported cause before repeating the action.

### HOST_RESULT_UNKNOWN

**What you see:** The app could not confirm the action result

**Cause:** The action may have reached the controller. Its completion is not confirmed.

**Fix:**
1. Wait for current device status or reconnect.
2. Review the returned result in Details before starting the action again.

### HOST_SELECTION_EMPTY

**What you see:** Select joints or motors first

**Cause:** The action has no selected channels.

**Fix:**
1. Select the channels you want to use, then repeat the action.

### HOST_TARGET_INVALID

**What you see:** The requested movement is outside the allowed range

**Cause:** The requested target could not be used as entered.

**Fix:**
1. Choose a target within the range shown for the selected joint or motor.
2. Check calibration if the displayed range looks incorrect.

### HOST_CALIB_1

**What you see:** Calibration stopped: no joints are selected

**Cause:** This action needs at least one selected joint.

**Fix:**
1. Select the joints you want to use, then retry the action.

### HOST_CALIB_5

**What you see:** Calibration stopped: joint feedback is unavailable

**Cause:** The controller cannot use the affected joint position readings.

**Fix:**
1. Check the S3 sensor board and its power and CAN connections.
2. Wait for current joint readings to return before enabling control.

### HOST_CALIB_6

**What you see:** Calibration stopped: motor locations are not known yet

**Cause:** Reliable feedback is needed before these motors can be controlled.

**Fix:**
1. Check motor power and the affected bus connections.
2. Wait for the affected motors to show Online, then retry the original action.

### HOST_CALIB_8

**What you see:** Calibration stopped: a motor load safety limit was reached

**Cause:** A load protection was triggered.

**Fix:**
1. Check for an obstruction, a tight tendon, or a joint that cannot move freely.
2. After correcting the cause, retry the original action.

### HOST_CALIB_13

**What you see:** Calibration stopped: the zero position could not be estimated

**Cause:** The recorded positions cannot produce a valid calibration.

**Fix:**
1. Check the calibration start pose and magnetic encoder installation.
2. Check tendon routing and preload, then calibrate the affected joint again.

### HOST_CALIB_15

**What you see:** Calibration stopped: a requested motor position is outside the safe range

**Cause:** The requested motor target was not accepted because it exceeded a safe position boundary.

**Fix:**
1. Check the requested target angle and the affected joint.
2. If ordinary targets keep reaching this limit, check calibration before continuing.

### HOST_CALIB_16

**What you see:** Calibration stopped: the motor command could not be used

**Cause:** A calibration motor target was not accepted.

**Fix:**
1. Open Details to check the target rejection or control ownership.
2. Wait for the current mode change to finish; if Stop is active, check the hand and retry calibration.

### HOST_CALIB_17

**What you see:** Calibration stopped: position feedback was lost

**Cause:** The app could no longer confirm all required motor and joint positions.

**Fix:**
1. Check live motor positions and joint encoder readings for the affected joints.
2. Check power and connections for the board or motors whose readings stopped.
3. Wait for reliable feedback before starting calibration again.

### HOST_CALIB_18

**What you see:** Calibration stopped: the motor command could not be used

**Cause:** The controller reported an internal problem.

**Fix:**
1. Stop using motion controls and open Details.
2. When the hand is safe, restart the controller and reconnect.
3. If it happens again, save the session log for support.

### HOST_CALIB_20

**What you see:** Calibration stopped: another task controls the motors

**Cause:** The controller did not accept this request in its current state.

**Fix:**
1. Wait for the current action or mode change to finish.
2. If the app remains busy, click Stop, then retry Enable.

### HOST_CALIB_21

**What you see:** Calibration stopped: other joints could not be held safely

**Cause:** The controller could not hold the other selected joints during calibration.

**Fix:**
1. Check the affected motors are Online and their positions are known.
2. Open Details to identify the affected motors and check their power and bus connections.

### HOST_CALIB_22

**What you see:** Calibration stopped: the joint selection could not be used

**Cause:** The requested calibration joint list was invalid.

**Fix:**
1. Select the required joints again.
2. If this repeats, save the session log for support.

### HOST_CALIB_23

**What you see:** Calibration stopped: a joint was listed twice

**Cause:** The calibration sequence was rejected before it could run correctly.

**Fix:**
1. Select the joints again.
2. If this repeats, save the session log for support.

### HOST_CALIB_24

**What you see:** Calibration stopped: the required starting pose was not reached

**Cause:** The helper joint could not reach the pose needed for this calibration.

**Fix:**
1. Check the helper joint and its motors in Details.
2. Check for an obstruction, tendon binding, or missing motor feedback before retrying calibration.

### HOST_CALIB_25

**What you see:** Calibration stopped: a joint did not complete its calibration

**Cause:** No specific mechanical cause was reported for this failure.

**Fix:**
1. Check the failed joints and preceding messages in Details.
2. Correct the reported problem before calibrating those joints again.

### HOST_CALIB_26

**What you see:** Calibration stopped: motor recovery did not finish

**Cause:** Reliable motor feedback and trusted positions were not recovered in time.

**Fix:**
1. Open Details to identify the unresolved motors.
2. Check those motors show Online and their live positions update.
3. Check motor power and bus connections before calibrating again.

### HOST_CALIB_28

**What you see:** Calibration stopped: the joint-to-motor mapping is invalid

**Cause:** The app cannot determine which motors should move this joint.

**Fix:**
1. Stop using motion controls.
2. Verify the configured joint and motor mapping with support before calibrating again.

### HOST_CALIB_30

**What you see:** Calibration stopped: the samples could not form a usable calibration

**Cause:** A valid motor-to-joint mapping could not be built from the recorded samples.

**Fix:**
1. Check the Manual endpoints and live encoder readings.
2. Inspect the samples in Details and correct missing feedback before recalibrating.

### HOST_CALIB_31

**What you see:** Calibration stopped: the new result could not be saved

**Cause:** The affected joint is not confirmed to have the new calibration installed.

**Fix:**
1. Open Details and check the returned save or device rejection message.
2. Keep the session log; reconnect and verify the installed calibration before continuing.

### HOST_CALIB_32

**What you see:** Calibration stopped: motor positions need to be recovered

**Cause:** The controller cannot safely interpret the affected motor positions.

**Fix:**
1. Wait for position recovery to finish.
2. If recovery fails, open Motor Calibration and calibrate the affected joints.

### HOST_CALIB_33

**What you see:** Calibration stopped: the app could not confirm the action result

**Cause:** The action may have reached the controller. Its completion is not confirmed.

**Fix:**
1. Wait for current device status or reconnect.
2. Review the returned result in Details before starting the action again.

### HOST_CALIB_34

**What you see:** Calibration stopped: the joint did not move during the tendon exchange

**Cause:** The expected joint motion was not confirmed; the cause is not yet known.

**Fix:**
1. Check for binding and check both tendon winding directions and preload.
2. Watch whether live encoder readings follow joint movement before recalibrating.

### HOST_CALIB_39

**What you see:** Calibration stopped: endpoint search timed out

**Cause:** The pulling tendon never reached the expected load, or the joint never arrived near the endpoint, before the phase time limit.

**Fix:**
1. Check that the tendon is attached and follows the motor; remove any trapped slack.
2. Push the joint by hand to both mechanical stops and re-capture this joint's Manual Min/Max.
3. Run Motor Calibration again for this joint.

### HOST_CALIB_WARNING

**What you see:** Calibration completed with an assembly warning

**Cause:** Calibration results were saved, but the measured travel or return needs inspection.

**Fix:**
1. Open Motor Calibration and inspect the affected joints.
2. Check tendon routing, preload and joint or encoder installation before continuing.

## B. Raw messages that can still fall back to "cause not confirmed"

These are result strings produced by gates across `desktop/core`. 24 of them now resolve
to a specific code; the rest still show `HOST_ACTION_FAILED` ("The exact cause is not
confirmed"). Cause and fix below describe what actually happened in each group.
See `docs/desktop-safety-catalog.md` for the per-line mapping.

### Enable / ownership

Messages: `Control handover was not queued`, `P4 did not acknowledge control handover`, `P4 did not release the old control owner`, `A maintenance or recovery operation owns the device`, `Enable coordinate recovery failed`, `Scope setup failed`, `Enable preparation was not confirmed by P4`, `Enable preparation changed before it could be used`, `P4 did not report READY after workspace recovery`, `Enable blocked: P4 readiness or mapped feedback changed`, `Enable blocked: J.. current angle is invalid`, `Workspace recovery was not queued`, `P4 workspace recovery ACK timed out`, `P4 rejected workspace recovery`

**What you see:** the generic "The app could not complete this action"

**Cause:** P4 did not confirm ownership or readiness in time (busy bus, latched STOP, stale feedback).

**Fix:** Read state and owner in Details, wait for stable status, clear STOP, reconnect.

### Reset / zero

Messages: `P4 did not acknowledge RESET`, `RESET acknowledged, but a fresh stationary P4 status was not confirmed`, `Current ABS is stale or untrusted`, `P4 stop/owner release was not confirmed; ABS zero was not sent`, `P4 device identity changed; ABS zero was not sent`, `J.. zero finish target publish failed`, `zero finish direct owner release failed`, `Cannot reset motor positions while Stop is active`

**What you see:** the generic "The app could not complete this action"

**Cause:** Reset or zero requires a stationary device with an unchanged identity.

**Fix:** Press STOP, confirm the device identity, reconnect, then retry.

### Parameters

Messages: `Wait for calibration to finish before changing control parameters`, `Parameters saved; P4 synchronization pending until idle`, `Default parameters restored; P4 synchronization pending until idle`, `Parameters changed during synchronization; apply the new revision first`

**What you see:** the generic "The app could not complete this action"

**Cause:** Parameters can only synchronize while the device is idle.

**Fix:** Wait until idle, Apply, and wait for Applied.

### Direct / targets

Messages: `Targets must contain 21 finite angle values`, `Select a valid joint scope`, `Select valid joints`, `Joint selection is out of range`, `Motor selection is out of range`, `Not connected or empty motor scope`, `ordinary controller is not enabled; press ENABLE first`, `Motor target send failed`, `P4 rejected motor target`, `Motor execution confirmation timed out`, `Current-position hold was not queued`, `P4 rejected current-position hold`, `Direct enable execution was not confirmed`

**What you see:** the generic "The app could not complete this action"

**Cause:** Invalid target or scope, controller not enabled, or the confirmation timed out.

**Fix:** Enable first, select valid joints, keep targets in range, check the connection on timeout.

### Scan / identity

Messages: `Choose Scan (0-30) or Scan (0-253)`, `No motors were found`, `The scan is no longer current`, `P4 identity is unavailable; wait for device fingerprint`, `Select a device. The port is never changed automatically.`

**What you see:** the generic "The app could not complete this action"

**Cause:** Not connected, wrong scan range, or the scan is stale.

**Fix:** Connect P4, choose the right range, rescan.

### Feedback freshness

Messages: `P4 is still scanning the servo buses; wait for the scan to finish`, `Servo feedback is not available yet; wait for the next feedback frame`, `Motor position feedback is not fresh; wait for the next feedback frame`

**What you see:** the generic "The app could not complete this action"

**Cause:** The first feedback frames have not arrived yet.

**Fix:** Wait a few seconds and retry.

### Recording

Messages: `Turn Save Data OFF before changing recording options`, `Recording options updated`

**What you see:** the generic "The app could not complete this action"

**Cause:** Recording options are locked while Save Data is on.

**Fix:** Turn Save Data off, change the options, turn it back on.

### Calibration import / snapshot

Messages: `Calibration file exceeds 10 MB`, `Import requires a validated file and fresh, idle P4 connection`, `unsupported snapshot schema`, `snapshot Manual CRC mismatch`, `current URDF mapping is invalid`, `invalid motor ABS limits`, `invalid motor ABS limit count`, `invalid joint calibration count`, `no valid joint calibration result in snapshot`, `P4 Manual references missing active joints`, `P4 Manual endpoint readback is incomplete`, `P4 Manual RAM image was not committed`, `failed to read current Manual RAM before restore`, `failed to read Manual RAM after restore`, `J.. Manual RAM restore rejected`, `J.. Manual commit timed out`, `J.. Manual RAM readback mismatch`, `P4 Manual RAM readback mask is incomplete`, `motor ABS limit load ACK timeout/reject`, `J.. load ACK timeout/reject`, `device or model identity changed before calibration commit`

**What you see:** the generic "The app could not complete this action"

**Cause:** The file, snapshot, firmware or URDF contract does not match, or P4 is not idle.

**Fix:** Export and import with the current firmware, check URDF and mapping, reconnect and retry.

### Manual capture

Messages: `Invalid Manual capture arguments`, `P4 connection changed; capture the endpoint again`, `J.. raw feedback is invalid or stale`, `J.. not saved`, `Manual readback failed`, `J..: ..`

**What you see:** the generic "The app could not complete this action"

**Cause:** Feedback or the connection changed during capture.

**Fix:** Reconnect, wait for stable feedback, re-capture.

### Multi-turn

Messages: `no multi-turn anchors to reconcile`, `multi-turn frame shift refused`, `multi-turn reconciliation reload failed`, `device connection changed during coordinate reconciliation`, `P4 did not accept the multi-turn trust request`, `no persistent multi-turn anchor`, `waiting for post-reset native ABS`, `post-reset native ABS unavailable`

**What you see:** the generic "The app could not complete this action"

**Cause:** The multi-turn anchor does not match the current readings.

**Fix:** Do not reconcile while the connection changes; reconnect; recalibrate if it repeats.

### ROS2

Messages: `ROS2 enable blocked: host calibration is active`, `ROS2 enable blocked: device transaction is active`, `ROS2 enable blocked: P4 serial is not connected`, `An operation is running. Wait for it to finish before authorizing ROS Input.`

**What you see:** the generic "The app could not complete this action"

**Cause:** The host is busy or not connected.

**Fix:** Finish the operation, connect P4.

### Replay

Messages: `Replay service is not running`, `Replay is not running`

**What you see:** the generic "The app could not complete this action"

**Cause:** The replay service was not started.

**Fix:** Restart the app or reopen Replay.

### Calibration commit

Messages: `invalid host joint calibration result`, `mechanical result retained in host diagnostics; LUT build invalid, not uploaded`, `could not release motion before Manual/LUT commit`, `Manual writeback failed`, `joint LUT writeback failed`, `calibration motor limits incomplete`, `joint ABS limits writeback failed`, `calibration applied to P4 but trusted native ABS feedback did not return`, `calibration applied to P4 but runtime anchor persistence failed`, `calibration complete; motion owner release failed`, `calibration complete; active scope restore failed`, `Pull Joint into Workspace failed`

**What you see:** the generic "The app could not complete this action"

**Cause:** P4 accepted the result, but writeback, anchor persistence or ownership release did not complete.

**Fix:** Keep the session log, reconnect, and verify the installed calibration before using the hand.

### Internal / lifecycle

Messages: `Application is shutting down`, `Missing operation id`, `Unknown operation`, `Operation has no calibration alert`, `The operation worker is closed`, `Connection failed on ..`, `system status stale or missing`, `faults=0x.., route conflicts=0x..`, `state=.., owner=.., token=.., result=..`, `not stopped: state=.., owner=..`, `accepted / acknowledged / updated success strings`

**What you see:** the generic "The app could not complete this action"

**Cause:** Internal sequencing or app shutdown, not a user action.

**Fix:** Save the log and reconnect; report a bug if it repeats.

