# Desktop Qt UI Architecture

The desktop upper-computer UI now runs on Qt through `PySide6`/`PyQt5` compatibility plus `pyqtgraph`. Use the package entrypoint for normal launches:

```bash
python -m desktop ui
```

## Overview

- `desktop.core.hand_backend` defines `HandBackend`, the UI-free application backend facade. It owns `HandController`, tactile serial lifecycle, tendon guard orchestration, state fan-out, Replay motion ownership, and the command API that Qt/ROS2/Replay code shares.
- `ui_qt_compat.py` selects the Qt binding. It tries `PySide6` first, then falls back to `PyQt5`, and sets `PYQTGRAPH_QT_LIB` before pyqtgraph is imported.
- `ui_app.py` defines `HandGUI(QMainWindow)`. It owns the Qt window, top toolbar, main splitter, state polling timers, display updates, and UI logging. Hardware lifecycle and command orchestration go through `HandBackend`.
- `ui_windows.py` manages the `View` menu and the singleton hand-pose and tactile popout windows.
- `ui_context.py` exposes a small component-facing facade around `HandGUI` and its `HandBackend`.

## Components

- `ui_control_panels.py`: Qt-native control surface with channel tables, device controls, ROS2 output authorization, and the Replay CSV view.
- `ui_replay_view.py`: imports an action-command CSV or joint-current CSV, shows live 21-joint Current/Replay Target/Error values, and exposes a single Send/Stop control.
- `ui_pose_view.py`: 基于 Qt QPainter 的活动 URDF 骨架视图；不依赖 OpenGL，避免部分 Windows 显卡驱动在首帧绘制时失败。
- `ui_tactile_view.py`: `pyqtgraph.ImageItem` tactile heatmap with latest-only background matrix preparation and throttled refresh.
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
- Serial receive, ROS2 executor, tactile matrix preparation, and CSV writing may run in background threads.
- Background producers must use latest-only or bounded queues to avoid UI backlog.

## Import Notes

- Import `HandGUI` from `desktop.ui.ui_app`.
- `HandGUI` accepts a `HandBackend`; wrap lower-level controllers before constructing the window.
- Hardware protocol, calibration files, and `HandController` command methods are not changed by the UI migration.
- If `PySide6.QtWidgets` fails to import because of a DLL issue, the UI falls back to installed `PyQt5`.
