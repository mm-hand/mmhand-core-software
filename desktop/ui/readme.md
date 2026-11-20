# MM-Hand NiceGUI UI

Launch from the repository root with `python run.py --no-ros2`, then open `http://127.0.0.1:8080`.
If port 8080 is occupied, the app automatically serves on the next free port
and prints the chosen port. Omit `--no-ros2` when the ROS2 environment is
available.

`browser_view.py` builds the fixed connection bar, status bar, four main tabs with subpages, dialogs
and System Log. `web/` has one ordinary build/refresh module per page, and
`common.py` provides small native component helpers. No page base class or
separate frontend project is used.

Calibration / Joint Calibration is the default page. Main tabs are Calibration,
Control, Plot and Settings. Calibration contains Joint, Contact and Motor
Calibration; Control contains Feedback Control, Direct Control and Replay; Plot
contains Hand and Curves; Settings contains Renumber Motor,
Control Parameters and Diagnostics. Plot > Hand splits the preview left/right:
the left pane is the interactive URDF pose scene (gray current pose, light-green
target pose, Reset view), the right pane is the MMHand palm stereo feed; the
System Log stays below both. The tactile editor was removed: the
PaXini modules on this hand do not accept the documented SPI ID write.
The motor editor warns that same-bus duplicate IDs cannot be distinguished or
renamed separately. See `docs/debug/id-reset.md`.

The connection bar also owns the palm UVC camera: the `Palm camera` picker and
`Connect` button sit after the P4 and S3 controls. The palm camera is
process-wide because a V4L2 node has one owner; `Connect` opens it, `Disconnect`
releases it, and Plot > Hand shows the latest frame at native aspect ratio
(`OFF` while disconnected). Camera enumeration and the latest-frame reader live
in `desktop/core/palm_camera.py` and mirror the Teleop application, so both
programs accept the same nodes; `opencv-python` is required. RealSense exposes
its depth node (Z16) next to the colour node, and sysfs gives them the same
truncated name; only the colour node can be captured. A depth node fails with
the same "cannot open" message, which names both a busy device and an
unsupported colour format.

ROS2 receives only `/mmhand/joint_angle_deg`:21 degree values J00–J20. Control
settings and defaults live in `desktop/resources/config`. Every connection
applies a complete parameter revision to P4 and queries its actual revision/CRC.
There is no Pause or Exit Control. No joints or motors are preselected.

Hardware operations go through HandBackend. UI owns only target drafts and
display caches. Endpoint confirmations, transactions, recording lifecycle and
quality interpretation belong to core/application_actions.py and
core/console_snapshot.py.

Snapshots refresh at three rates: connection/status/Rx age and badges at 10 Hz,
non-plot page content at 2 Hz, and visible plots/tactile charts at 1 Hz. Every
tick is change-detected and back-pressured; a stalled tick is skipped. A visible
page whose heartbeat stops for 3 s shows a reconnect overlay and reloads once
(or twice at most). Input edits never send commands. Blocking device writes run
outside the NiceGUI event loop. STOP uses the existing backend cancellation path.

This is the only GUI: Qt pages, compatibility helpers and Qt-only tests were
removed and are not coming back. Core control, communications, calibration,
ROS2 and replay stay independent of UI.

## Page roles and control handover

- Joint Calibration captures Manual Min/Max, loads a calibration bundle and
  releases tendons. It shows Manual/URDF/JSON status only; automatic and motor
  calibration status belongs to Motor Calibration.
- Motor Calibration runs the host automatic calibration and shows Auto
  Min/Max/Span plus return-offset/travel-shortfall diagnostics.
- Feedback Control and Direct Control own one control mode each. Pressing
  `ENABLE Selected` on the target page performs the P4 ownership handover; tabs
  only change the view. The top-right button is `■ STOP` while running and
  becomes `Clear Faults` after a latched STOP.
- Starting a calibration or maintenance action automatically releases an active
  control owner. A latched STOP is never cleared automatically.
- Calibration failure dialogs are scoped to the page that started the operation
  and are acknowledged once; they do not replay on page reloads or other tabs.
