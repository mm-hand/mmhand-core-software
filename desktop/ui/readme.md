# Desktop Qt UI Architecture

The desktop upper-computer UI now runs on Qt through `PySide6`/`PyQt5` compatibility plus `pyqtgraph`. Use the package entrypoint for normal launches:

```bash
python -m desktop ui
```

## Overview

- `desktop.core.hand_backend` defines `HandBackend`, the UI-free application backend facade. It owns `HandController`, tactile serial lifecycle, tendon guard orchestration, state fan-out, and the command API that Qt/ROS/teleop/algorithm code should share.
- `ui_qt_compat.py` selects the Qt binding. It tries `PySide6` first, then falls back to `PyQt5`, and sets `PYQTGRAPH_QT_LIB` before pyqtgraph is imported.
- `ui_app.py` defines `HandGUI(QMainWindow)`. It owns the Qt window, top toolbar, main splitter, state polling timers, display updates, and UI logging. Hardware lifecycle and command orchestration go through `HandBackend`.
- `ui_launcher.py` defines the Qt startup dialog for port and mode selection.
- `ui_windows.py` manages the `View` menu and the singleton popout windows: hand pose, tactile, and cameras.
- `ui_context.py` exposes a small component-facing facade around `HandGUI` and its `HandBackend`.

## Components

- `ui_control_panels.py`: Qt-native control surface with `QTableWidget` channel tables, ENABLE/E-STOP/Pause, Calibrate/Clear Fault/Rope Init/Flash/ABS-limit controls, Send All, local test controls, Teleop toggle, and Algorithm output.
- `ui_pose_view.py`: `pyqtgraph.opengl.GLViewWidget` URDF skeleton view. It reuses line/scatter items for updates.
- `ui_tactile_view.py`: `pyqtgraph.ImageItem` tactile heatmap with latest-only background matrix preparation and throttled refresh.
- `ui_camera_view.py`: OpenCV capture thread with main-thread `QImage/QPixmap` display.
- `ui_joint_plot_view.py`: `pyqtgraph.PlotWidget` real-time target/actual curve window with reusable curve items.
- `ui_services.py`: latest-only state buffer, async CSV logger, and `HandModel` snapshot helper.

## Data Flow

1. `HandController` receives device state from `LowerComputerComm`.
2. `HandBackend` registers as the controller callback, clones the state, and fans it out to UI/ROS-style consumers.
3. `HandGUI._on_state_update` writes the latest snapshot into `LatestStateBuffer`; it does not touch Qt widgets.
4. A `QTimer` consumes the latest snapshot every 50 ms.
5. The main thread updates control feedback, display popouts, Plot data, status text, and optional CSV logging.
6. Display popouts only update while open.

## Backend Boundary

- Qt components should call `context.backend` or explicit `HandGuiContext` methods instead of importing or owning `HandController`.
- `HandController` remains the hardware primitive layer: serial connection, protocol queues, current hardware state, and low-level command methods.
- `HandBackend` is the application orchestration layer: connect/disconnect, shared command API, tendon guard sync/apply, tactile serial management, state snapshots, and protocol diagnostics.
- A future ROS node should wrap `HandBackend` directly, not `ui_app.py` or `HandController`.

## Threading Rules

- Qt widgets, `QPixmap`, and pyqtgraph items are updated only on the main Qt thread.
- Serial receive, camera capture, tactile matrix preparation, and CSV writing may run in background threads.
- Background producers must use latest-only or bounded queues to avoid UI backlog.

## Import Notes

- Import `HandGUI` from `desktop.ui.ui_app` and `LauncherWindow` from `desktop.ui.ui_launcher`.
- `HandGUI` accepts a `HandBackend`; passing a legacy `HandController` still works because it is wrapped in a backend at construction time.
- Hardware protocol, calibration files, and `HandController` command methods are not changed by the UI migration.
- If `PySide6.QtWidgets` fails to import because of a DLL issue, the UI falls back to installed `PyQt5`.
