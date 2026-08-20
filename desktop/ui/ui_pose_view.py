"""ui_pose_view.py - MMHand Desktop 模块契约。

职责：显示活动 URDF 的手部姿态并支持模型重载。
线程：Qt GUI 主线程执行；耗时工作必须委托后台服务，禁止阻塞界面事件循环。
输入：用户操作、HandModel 只读快照以及 HandBackend/服务返回结果。
输出：Qt 控件状态、用户提示和经门面提交的应用命令。
状态所有权：仅拥有本视图控件和瞬时交互状态；设备真值由 core 层拥有。
禁止依赖：禁止直接导入串口通信和底层控制实现，禁止在 UI 中复制协议或安全规则。
"""
import os
from typing import List, Optional

import numpy as np

from desktop.core.angle_mapping import (
    JOINT_SPECS,
    calibrated_angles_to_urdf_joint_positions,
)
from desktop.core.protocol import ENCODER_COUNT
from desktop.core.session_log import log_session
from desktop.paths import DEFAULT_URDF_PATH
from desktop.visualize.urdf_kinematics import (
    DISPLAY_TRANSFORM,
    UrdfKinematics,
    apply_display_transform,
)
from .ui_qt_compat import QtCore, QtGui, QtWidgets


class PoseSkeletonCanvas(QtWidgets.QWidget):
    """纯 Qt 骨架画布，避免 pyqtgraph.opengl 在部分 Windows 驱动上的 paintGL 崩溃。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._links: List[tuple[np.ndarray, np.ndarray]] = []
        self._nodes = np.empty((0, 3), dtype=float)
        self.setMinimumSize(420, 300)

    def set_poses(self, kin: UrdfKinematics, poses) -> None:
        links: List[tuple[np.ndarray, np.ndarray]] = []
        for joint in kin.joints:
            parent, child = joint["parent"], joint["child"]
            if parent in poses and child in poses:
                links.append((poses[parent][:3, 3].copy(), poses[child][:3, 3].copy()))
        self._links = links
        self._nodes = np.asarray(
            [pose[:3, 3] for pose in poses.values()], dtype=float
        )
        self.update()

    def clear(self) -> None:
        self._links = []
        self._nodes = np.empty((0, 3), dtype=float)
        self.update()

    @staticmethod
    def _project(points: np.ndarray, rect) -> np.ndarray:
        """正交投影显示 X/Z 平面：Z 向上，Y 仅保留在 URDF 姿态计算中。"""
        if points.size == 0:
            return np.empty((0, 2), dtype=float)
        plane = np.column_stack((points[:, 0], -points[:, 2]))
        lower, upper = plane.min(axis=0), plane.max(axis=0)
        span = np.maximum(upper - lower, 1e-4)
        scale = min(rect.width() / span[0], rect.height() / span[1]) * 0.88
        center = (lower + upper) * 0.5
        screen_center = np.array([rect.center().x(), rect.center().y()], dtype=float)
        return (plane - center) * scale + screen_center

    def paintEvent(self, event) -> None:
        _ = event
        painter = QtGui.QPainter(self)
        try:
            painter.fillRect(self.rect(), QtGui.QColor("#ffffff"))
            render_hint_enum = getattr(QtGui.QPainter, "RenderHint", QtGui.QPainter)
            painter.setRenderHint(render_hint_enum.Antialiasing, True)
            painter.setPen(QtGui.QPen(QtGui.QColor("#d0d7de"), 1))
            painter.drawRect(self.rect().adjusted(0, 0, -1, -1))
            if self._nodes.size == 0:
                painter.setPen(QtGui.QColor("#57606a"))
                painter.drawText(self.rect(), QtCore.Qt.AlignCenter, "Waiting for Hand Pose data…")
                return

            draw_rect = self.rect().adjusted(20, 20, -20, -20)
            all_points = np.vstack(
                [self._nodes] + [np.vstack(link) for link in self._links]
            )
            projected_all = self._project(all_points, draw_rect)
            point_index = len(self._nodes)
            nodes_projected = projected_all[:point_index]

            painter.setPen(QtGui.QPen(QtGui.QColor("#24292f"), 2))
            for _link in self._links:
                line = projected_all[point_index: point_index + 2]
                point_index += 2
                painter.drawLine(
                    QtCore.QPointF(float(line[0, 0]), float(line[0, 1])),
                    QtCore.QPointF(float(line[1, 0]), float(line[1, 1])),
                )

            painter.setPen(QtGui.QPen(QtGui.QColor("#cf222e"), 1))
            painter.setBrush(QtGui.QBrush(QtGui.QColor("#cf222e")))
            for point in nodes_projected:
                painter.drawEllipse(
                    QtCore.QPointF(float(point[0]), float(point[1])), 3.0, 3.0
                )
            painter.setPen(QtGui.QColor("#57606a"))
            painter.drawText(12, 22, "Active URDF skeleton · orthographic X/Z view")
        finally:
            painter.end()


class Pose3DView(QtWidgets.QWidget):
    """活动 URDF 的实时骨架视图。

    名称保留以兼容现有窗口管理器；渲染实现改为 Qt QPainter，不依赖 OpenGL。
    """

    def __init__(self, context, parent=None):
        super().__init__(parent)
        self.context = context
        self._kin: Optional[UrdfKinematics] = None
        self._canvas: Optional[PoseSkeletonCanvas] = None
        self._status_label = None
        self._last_angles: Optional[List[float]] = None
        self._closed = False
        self._paused = False
        self._renderer_initialized = False
        self._renderer_failed = False
        self._build_ui()
        self._load_urdf()

    def _build_ui(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        toolbar = QtWidgets.QHBoxLayout()
        reload_button = QtWidgets.QPushButton("Reload URDF")
        reload_button.setToolTip("Reload the active hand URDF and redraw the pose view")
        reload_button.clicked.connect(
            lambda _checked=False: self.reload_urdf(reset_angles=True)
        )
        toolbar.addWidget(reload_button)
        self._status_label = QtWidgets.QLabel("")
        self._status_label.setWordWrap(True)
        toolbar.addWidget(self._status_label, 1)
        layout.addLayout(toolbar)
        self._canvas = PoseSkeletonCanvas(self)
        layout.addWidget(self._canvas, 1)

    def _load_urdf(self) -> bool:
        return self.reload_urdf(reset_angles=True)

    def activate_renderer(self) -> bool:
        """兼容窗口管理器接口；Qt 画布不需要 OpenGL 上下文。"""
        if self._renderer_failed:
            return False
        self._renderer_initialized = self._canvas is not None
        if self._renderer_initialized and self._kin is not None and self._last_angles is not None:
            self.draw_angles(self._last_angles)
        self._set_status("Pose renderer ready (Qt painter, no OpenGL)")
        log_session("POSE", "Qt painter renderer initialized")
        return self._renderer_initialized

    def is_renderer_ready(self) -> bool:
        return bool(self._renderer_initialized and not self._renderer_failed and self._canvas is not None)

    def reload_urdf(self, *, reset_angles: bool = False) -> bool:
        """重新读取活动 URDF，并校验 21 路显示映射没有遗留旧模型名称。"""
        urdf_path = str(DEFAULT_URDF_PATH)
        self._clear_scene()
        if reset_angles or self._last_angles is None:
            self._last_angles = [0.0] * ENCODER_COUNT
        try:
            kin = UrdfKinematics(urdf_path)
            urdf_joint_names = set(kin.joint_by_name())
            required_names = {
                JOINT_SPECS[index].urdf_joint_name for index in range(ENCODER_COUNT)
            }
            missing_names = sorted(required_names - urdf_joint_names)
            if missing_names:
                raise ValueError(
                    "active URDF misses mapped joints: " + ", ".join(missing_names)
                )
            self._kin = kin
        except Exception as exc:
            self._kin = None
            text = f"URDF load failed: {os.path.basename(urdf_path)}\n{exc}"
            self._set_status(text)
            log_session("POSE", f"URDF load failed path={urdf_path}: {exc}")
            return False
        self._closed = False
        self._paused = False
        self._set_status(f"URDF loaded: {os.path.basename(urdf_path)} (21 mapped joints)")
        if self.is_renderer_ready():
            self.draw_angles(self._last_angles)
        return True

    def _set_status(self, text: str) -> None:
        if self._status_label is not None:
            self._status_label.setText(str(text))

    def _clear_scene(self) -> None:
        if self._canvas is not None:
            self._canvas.clear()

    def build(self, _parent) -> None:
        """DisplayWindow 已直接持有 QWidget，本方法保留统一组件接口。"""

    def update(self, state) -> None:
        if not self._paused:
            self.draw_angles(getattr(state, "angles", [0.0] * ENCODER_COUNT))

    def draw_angles(
        self,
        angles: List[float],
        state=None,
        *,
        generate: Optional[bool] = None,
    ) -> None:
        _ = state, generate
        self._last_angles = list(angles or [0.0] * ENCODER_COUNT)
        if self._closed or self._paused or not self.is_renderer_ready() or self._kin is None:
            return
        try:
            joint_positions = calibrated_angles_to_urdf_joint_positions(self._last_angles)
            poses = apply_display_transform(
                self._kin.compute_link_poses(joint_positions), DISPLAY_TRANSFORM
            )
            self._canvas.set_poses(self._kin, poses)
        except Exception as exc:
            self._renderer_failed = True
            self._set_status(f"Pose update failed: {exc}")
            log_session("POSE", f"Pose update failed: {exc}")

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
