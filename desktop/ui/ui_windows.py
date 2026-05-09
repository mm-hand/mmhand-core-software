import time
from typing import Dict, Optional

from .ui_qt_compat import QtCore, QtGui, QtWidgets, Signal
from .ui_camera_view import CameraView
from .ui_pose_view import Pose3DView
from .ui_tactile_view import TactileView


POSE_REAL_INTERVAL_S = 0.10
POSE_PREVIEW_INTERVAL_S = 0.25


class _DisplayWindow(QtWidgets.QDialog):
    closed = Signal(str)

    def __init__(self, key: str, title: str, component, parent=None):
        super().__init__(parent)
        self.key = key
        self.component = component
        if key != "pose":
            self.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
        self.setWindowTitle(title)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        component.build(self)
        layout.addWidget(component)

    def closeEvent(self, event):
        self.closed.emit(self.key)
        try:
            self.component.close()
        except Exception:
            pass
        if self.key == "pose":
            self.hide()
            event.ignore()
            return
        super().closeEvent(event)


class DisplayWindowManager:
    def __init__(self, owner):
        self.owner = owner
        self.context = getattr(owner, "context", owner)
        self.windows: Dict[str, _DisplayWindow] = {}
        self.components: Dict[str, object] = {}
        self.actions: Dict[str, object] = {}
        self._last_pose_update_ts = 0.0

    def attach_menu(self, menubar: QtWidgets.QMenuBar) -> None:
        menu = menubar.addMenu("View")
        for key, label in [("pose", "Hand Pose"), ("tactile", "Tactile"), ("cameras", "Cameras")]:
            action_cls = getattr(QtGui, "QAction", None) or getattr(QtWidgets, "QAction")
            action = action_cls(label, self.owner)
            action.setCheckable(True)
            action.toggled.connect(lambda checked, k=key: self.open(k) if checked else self.close(k))
            menu.addAction(action)
            self.actions[key] = action

    def apply_main_geometry(self) -> None:
        screen = QtWidgets.QApplication.primaryScreen()
        if screen is None:
            self.owner.resize(1180, 760)
            return
        area = screen.availableGeometry()
        self.owner.resize(min(1280, int(area.width() * 0.78)), min(820, int(area.height() * 0.82)))
        self.owner.move(area.x() + 24, area.y() + 24)

    def open_pose(self) -> None:
        self.open("pose")

    def open_tactile(self) -> None:
        self.open("tactile")

    def open_cameras(self) -> None:
        self.open("cameras")

    def is_open(self, key: str) -> bool:
        win = self.windows.get(key)
        if win is None:
            return False
        try:
            return bool(win.isVisible())
        except RuntimeError:
            self.windows.pop(key, None)
            self.components.pop(key, None)
            self._set_action(key, False)
            return False

    def open(self, key: str) -> None:
        if self.is_open(key):
            self.windows[key].raise_()
            self.windows[key].activateWindow()
            return
        hidden = self.windows.get(key)
        if key == "pose" and hidden is not None:
            self.components[key] = hidden.component
            try:
                hidden.component.resume()
            except Exception:
                pass
            self._last_pose_update_ts = 0.0
            self._set_action(key, True)
            self.layout_open_windows()
            hidden.show()
            hidden.raise_()
            hidden.activateWindow()
            return
        title, component, size = self._make_component(key)
        win = _DisplayWindow(key, title, component, self.owner)
        win.closed.connect(self._on_window_closed)
        win.resize(*size)
        self.windows[key] = win
        self.components[key] = component
        if key == "pose":
            self._last_pose_update_ts = 0.0
        self._set_action(key, True)
        self.layout_open_windows()
        win.show()
        win.raise_()

    def close(self, key: str) -> None:
        win = self.windows.get(key) if key == "pose" else self.windows.pop(key, None)
        self.components.pop(key, None)
        self._set_action(key, False)
        if win is not None:
            try:
                if key == "pose":
                    try:
                        win.component.pause()
                    except Exception:
                        pass
                    win.hide()
                else:
                    win.blockSignals(True)
                    win.close()
            except RuntimeError:
                if key == "pose":
                    self.windows.pop(key, None)
        self.layout_open_windows()

    def close_all(self) -> None:
        for key in list(self.windows.keys()):
            if key == "pose":
                win = self.windows.pop(key, None)
                self.components.pop(key, None)
                self._set_action(key, False)
                if win is not None:
                    try:
                        win.component.close()
                    except Exception:
                        pass
                    try:
                        win.deleteLater()
                    except RuntimeError:
                        pass
            else:
                self.close(key)

    def update_views(self, state) -> None:
        self.update_pose_if_due(state)
        self.update_tactile_if_due(state)

    def update_pose_if_due(self, state, *, force: bool = False) -> None:
        comp = self.components.get("pose")
        if comp is None:
            return
        interval = POSE_REAL_INTERVAL_S if self.owner.is_comm_connected() else POSE_PREVIEW_INTERVAL_S
        now = time.time()
        if not force and now - self._last_pose_update_ts < interval:
            return
        self._last_pose_update_ts = now
        comp.update(state)

    def update_tactile_if_due(self, state, *, force: bool = False) -> None:
        comp = self.components.get("tactile")
        if comp is not None:
            comp.update_if_due(state, force=force)

    def update_cameras(self) -> None:
        comp = self.components.get("cameras")
        if comp is not None:
            comp.update()

    def layout_open_windows(self) -> None:
        visible = [k for k in ("pose", "tactile", "cameras") if self.is_open(k)]
        if not visible:
            return
        geo = self.owner.geometry()
        screen = QtWidgets.QApplication.primaryScreen()
        area = screen.availableGeometry() if screen else geo
        x = geo.x() + geo.width() + 16
        y = geo.y()
        right_w = area.right() - x - 24
        if right_w < 420:
            x = area.x() + 24
            y = geo.y() + geo.height() + 16
            right_w = area.width() - 48
        right_w = max(420, right_w)
        each_h = max(300, min(520, (area.bottom() - y - 24) // max(1, len(visible))))
        for idx, key in enumerate(visible):
            win = self.windows.get(key)
            if win is not None:
                win.setGeometry(x, y + idx * (each_h + 12), min(860, right_w), each_h)

    def _set_action(self, key: str, checked: bool) -> None:
        action = self.actions.get(key)
        if action is None:
            return
        action.blockSignals(True)
        action.setChecked(checked)
        action.blockSignals(False)

    def _on_window_closed(self, key: str) -> None:
        if key != "pose":
            self.windows.pop(key, None)
        self.components.pop(key, None)
        self._set_action(key, False)
        self.layout_open_windows()

    def _make_component(self, key: str):
        if key == "pose":
            return "Hand Pose", Pose3DView(self.context), (760, 520)
        if key == "tactile":
            return "Tactile", TactileView(self.context), (760, 520)
        if key == "cameras":
            return "Cameras", CameraView(self.context), (860, 460)
        raise ValueError(f"Unknown display window: {key}")
