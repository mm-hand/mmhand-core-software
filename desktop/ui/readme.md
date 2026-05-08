# Desktop Qt UI Architecture

The desktop upper-computer UI now runs on Qt through `PySide6`/`PyQt5` compatibility plus `pyqtgraph`. The public entrypoint remains unchanged:

```python
from ui_main import HandGUI, LauncherWindow
```

## Overview

- `qt_compat.py` selects the Qt binding. It tries `PySide6` first, then falls back to `PyQt5`, and sets `PYQTGRAPH_QT_LIB` before pyqtgraph is imported.
- `app.py` defines `HandGUI(QMainWindow)`. It owns the controller, top toolbar, main splitter, state polling timers, logging, and lifecycle cleanup.
- `launcher.py` defines the Qt startup dialog for port and mode selection.
- `windows.py` manages the `View` menu and the singleton popout windows: hand pose, tactile, and cameras.
- `context.py` exposes a small component-facing facade around `HandGUI`.

## Components

- `control_panels.py`: Qt-native control surface with `QTableWidget` channel tables, Start/Stop/Home/Reset/Calibrate, Send All, local test controls, Teleop toggle, and Algorithm output.
- `pose_view.py`: `pyqtgraph.opengl.GLViewWidget` URDF skeleton view. It reuses line/scatter items for updates.
- `tactile_view.py`: `pyqtgraph.ImageItem` tactile heatmap with latest-only background matrix preparation and throttled refresh.
- `camera_view.py`: OpenCV capture thread with main-thread `QImage/QPixmap` display.
- `joint_plot_view.py`: `pyqtgraph.PlotWidget` real-time target/actual curve window with reusable curve items.
- `services.py`: latest-only state buffer, async CSV logger, and `HandModel` snapshot helper.

## Data Flow

1. `HandController` receives device state and calls `HandGUI._on_state_update`.
2. The callback writes a cloned snapshot into `LatestStateBuffer`; it does not touch Qt widgets.
3. A `QTimer` consumes the latest snapshot every 50 ms.
4. The main thread updates control feedback, display popouts, Plot data, status text, and optional CSV logging.
5. Display popouts only update while open.

## Threading Rules

- Qt widgets, `QPixmap`, and pyqtgraph items are updated only on the main Qt thread.
- Serial receive, camera capture, tactile matrix preparation, and CSV writing may run in background threads.
- Background producers must use latest-only or bounded queues to avoid UI backlog.

## Compatibility Notes

- `ui_main.py`, `main.py`, `modes.py`, and `example_teleop.py` can keep importing `HandGUI` as before.
- Hardware protocol, calibration files, and `HandController` command methods are not changed by the UI migration.
- If `PySide6.QtWidgets` fails to import because of a DLL issue, the UI falls back to installed `PyQt5`.
