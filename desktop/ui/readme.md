# MM-Hand NiceGUI UI

Launch with `python -m desktop --no-ros2`, then open `http://127.0.0.1:8080`.
If port 8080 is occupied, the app automatically serves on the next free port
and prints the chosen port. Omit `--no-ros2` when the ROS2 environment is
available.

`browser_view.py` builds the fixed connection bar, status bar, five main tabs with subpages, import
dialog and System Log. `web/` has one ordinary build/refresh module per page,
and `common.py` provides small native component helpers. No page base class or
separate frontend project is used.

Calibration / Joint Calibration is the default page. Main pages are Calibration,
Control, Replay, Plot and Settings. Settings contains Renumber Motor, Renumber
Tactile, Control Parameters and Diagnostics. The tactile editor scans all five
fingers, with per-sensor Current ID and New ID inputs and a batch Renumber All.
Both editors warn that same-bus duplicate IDs cannot be distinguished or renamed
separately. See `docs/debug/id-reset.md`.

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

Replaced Qt pages, compatibility helpers and Qt-only tests are removed. Core
control, communications, calibration, ROS2 and replay stay independent of UI.

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
