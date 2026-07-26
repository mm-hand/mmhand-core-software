import os
from typing import List, Optional

import numpy as np

from desktop.paths import DEFAULT_URDF_PATH
from desktop.core.protocol import ENCODER_COUNT
from desktop.visualize.plot_hand_from_urdf import (
    DISPLAY_TRANSFORM,
    UrdfKinematics,
    apply_display_transform,
    motor_deg_to_urdf_joint_dict,
)
from .ui_qt_compat import QtCore, QtWidgets


class Pose3DView(QtWidgets.QWidget):
    def __init__(self, context, parent=None):
        super().__init__(parent)
        self.context = context
        self._kin = None
        self._gl = None
        self._grid_item = None
        self._label = None
        self._status_label = None
        self._reload_btn = None
        self._link_items = {}
        self._node_item = None
        self._last_angles: Optional[List[float]] = None
        self._closed = False
        self._paused = False
        self._build_ui()
        self._load_urdf()

    def _build_ui(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        toolbar = QtWidgets.QHBoxLayout()
        self._reload_btn = QtWidgets.QPushButton("Reload URDF")
        self._reload_btn.setToolTip("Reload the hand URDF file and redraw the pose view")
        self._reload_btn.clicked.connect(lambda _checked=False: self.reload_urdf(reset_angles=True))
        toolbar.addWidget(self._reload_btn)
        self._status_label = QtWidgets.QLabel("")
        self._status_label.setWordWrap(True)
        toolbar.addWidget(self._status_label, 1)
        layout.addLayout(toolbar)
        try:
            import pyqtgraph.opengl as gl

            self._gl_mod = gl
            self._gl = gl.GLViewWidget()
            self._gl.setBackgroundColor("w")
            self._reset_camera()
            layout.addWidget(self._gl, 1)
            self._install_grid()
        except Exception as exc:
            self._gl_mod = None
            self._label = QtWidgets.QLabel(f"OpenGL pose view unavailable:\n{exc}")
            self._label.setAlignment(QtCore.Qt.AlignCenter)
            layout.addWidget(self._label, 1)

    def _load_urdf(self) -> bool:
        return self.reload_urdf(reset_angles=True)

    def reload_urdf(self, *, reset_angles: bool = False) -> bool:
        urdf_path = str(DEFAULT_URDF_PATH)
        self._clear_scene()
        if reset_angles or self._last_angles is None:
            self._last_angles = [0.0] * ENCODER_COUNT
        try:
            self._kin = UrdfKinematics(urdf_path)
        except Exception as exc:
            self._kin = None
            text = f"URDF load failed:\n{urdf_path}\n{exc}"
            if self._label is not None:
                self._label.setText(text)
            self._set_status(text)
            return False
        self._closed = False
        self._paused = False
        self._reset_camera()
        self._set_status(f"URDF loaded: {os.path.basename(urdf_path)}")
        self.draw_angles(self._last_angles)
        return True

    def _set_status(self, text: str) -> None:
        if self._status_label is not None:
            self._status_label.setText(str(text))

    def _install_grid(self) -> None:
        if self._gl is None or self._gl_mod is None:
            return
        try:
            grid = self._gl_mod.GLGridItem()
            grid.setSize(0.22, 0.22)
            grid.setSpacing(0.02, 0.02)
            self._grid_item = grid
            self._gl.addItem(grid)
        except Exception as exc:
            self._grid_item = None
            self._set_status(f"Pose grid unavailable: {exc}")

    def _reset_camera(self) -> None:
        if self._gl is None:
            return
        self._gl.opts["distance"] = 0.42
        self._gl.opts["elevation"] = 18
        self._gl.opts["azimuth"] = -68

    def _remove_gl_item(self, item) -> None:
        if self._gl is None or item is None:
            return
        try:
            self._gl.removeItem(item)
        except Exception:
            pass

    def _clear_scene(self) -> None:
        if self._gl is not None:
            for item in list(getattr(self._gl, "items", [])):
                if item is self._grid_item:
                    continue
                self._remove_gl_item(item)
        self._link_items.clear()
        self._node_item = None

    def build(self, _parent) -> None:
        pass

    def update(self, state) -> None:
        if self._paused:
            return
        self.draw_angles(getattr(state, "angles", [0.0] * ENCODER_COUNT))

    def draw_angles(self, angles: List[float], state=None, *, generate: Optional[bool] = None) -> None:
        _ = state, generate
        self._last_angles = list(angles or [0.0] * ENCODER_COUNT)
        if self._closed or self._paused or self._gl is None or self._kin is None:
            return
        joint_pos = motor_deg_to_urdf_joint_dict(self._last_angles)
        poses = apply_display_transform(self._kin.compute_link_poses(joint_pos), DISPLAY_TRANSFORM)
        gl = self._gl_mod
        used = set()
        for j in self._kin.joints:
            parent = j["parent"]
            child = j["child"]
            if parent not in poses or child not in poses:
                continue
            p0 = poses[parent][:3, 3]
            p1 = poses[child][:3, 3]
            pts = np.vstack([p0, p1]).astype(float)
            key = (parent, child)
            used.add(key)
            item = self._link_items.get(key)
            if item is None:
                item = gl.GLLinePlotItem(pos=pts, color=(0.02, 0.02, 0.02, 1.0), width=2.0, antialias=True)
                self._link_items[key] = item
                self._gl.addItem(item)
            else:
                item.setData(pos=pts)
        for key in list(self._link_items.keys()):
            if key not in used:
                try:
                    self._gl.removeItem(self._link_items[key])
                except Exception:
                    pass
                self._link_items.pop(key, None)
        pts = np.array([T[:3, 3] for T in poses.values()], dtype=float)
        if pts.size == 0:
            return
        if self._node_item is None:
            self._node_item = gl.GLScatterPlotItem(pos=pts, color=(1.0, 0.0, 0.0, 1.0), size=5.0, pxMode=True)
            self._gl.addItem(self._node_item)
        else:
            self._node_item.setData(pos=pts)
        self._fit_camera_to_points(pts)

    def _fit_camera_to_points(self, pts: np.ndarray) -> None:
        if self._gl is None or pts is None:
            return
        arr = np.asarray(pts, dtype=float)
        if arr.ndim != 2 or arr.shape[1] != 3:
            return
        arr = arr[np.all(np.isfinite(arr), axis=1)]
        if arr.size == 0:
            return
        bbox_min = arr.min(axis=0)
        bbox_max = arr.max(axis=0)
        center = (bbox_min + bbox_max) * 0.5
        span = bbox_max - bbox_min
        if span[2] > 0:
            center[2] += span[2] * 0.03
        max_span = max(float(np.linalg.norm(span)), float(np.max(span)), 1e-3)
        distance = max(0.13, min(1.05, max_span * 2.15))
        try:
            import pyqtgraph as pg

            self._gl.opts["center"] = pg.Vector(float(center[0]), float(center[1]), float(center[2]))
        except Exception:
            self._gl.opts["center"] = tuple(float(v) for v in center)
        self._gl.opts["distance"] = distance

    def clear(self) -> None:
        self.draw_angles([0.0] * ENCODER_COUNT)

    def pause(self) -> None:
        self._paused = True

    def resume(self) -> None:
        self._paused = False
        self._closed = False
        if self._last_angles is not None:
            self.draw_angles(self._last_angles)

    def close(self) -> None:
        self.pause()
