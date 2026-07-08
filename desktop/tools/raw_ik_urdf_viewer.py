import argparse
import json
import math
import os
import struct
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Optional

import numpy as np


DESKTOP_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT_ROOT = os.path.dirname(DESKTOP_ROOT)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from desktop.core.protocol import ENCODER_COUNT  # noqa: E402
from desktop.teleop.Posture.hand_retargeting.constants import DEFAULT_URDF_PATH  # noqa: E402
from desktop.teleop.Posture.hand_retargeting.math_utils import make_transform, parse_xyz, rpy_to_rotation  # noqa: E402
from desktop.teleop.Posture.hand_retargeting.urdf_model import URDFRobotHandModel  # noqa: E402
from desktop.ui.ui_qt_compat import QtCore, QtGui, QtWidgets  # noqa: E402


SEMANTIC_JOINT_TO_ENCODER_INDEX = {
    "little_aa_joint": 0,
    "mmhand_finger_4_proximal_phalanx_1_mcp_fe_joint": 1,
    "mmhand_finger_4_distal_phalanx_1_pip_joint": 2,
    "mmhand_finger_4_fingertip_1_dip_joint": 3,
    "ring_aa_joint": 4,
    "mmhand_finger_3_proximal_phalanx_1_mcp_fe_joint": 5,
    "mmhand_finger_3_distal_phalanx_1_pip_joint": 6,
    "mmhand_finger_3_fingertip_1_dip_joint": 7,
    "middle_aa_joint": 8,
    "mmhand_finger_2_proximal_phalanx_1_mcp_fe_joint": 9,
    "mmhand_finger_2_distal_phalanx_1_pip_joint": 10,
    "mmhand_finger_2_fingertip_1_dip_joint": 11,
    "index_aa_joint": 12,
    "mmhand_finger_1_proximal_phalanx_1_mcp_fe_joint": 13,
    "mmhand_finger_1_distal_phalanx_1_pip_joint": 14,
    "mmhand_finger_1_fingertip_1_dip_joint": 15,
    "thumb_aa_joint": 16,
    "mmhand_thumb_1_mmhand_finger_5_proximal_phalanx_1_mcp_fe_joint": 17,
    "mmhand_thumb_1_mmhand_finger_5_distal_phalanx_1_pip_joint": 18,
    "mmhand_thumb_1_mmhand_finger_5_fingertip_1_dip_joint": 19,
    "thumb_cmc1_joint": 20,
}


def _encoder_index_for_joint(joint_name: str) -> Optional[int]:
    return SEMANTIC_JOINT_TO_ENCODER_INDEX.get(str(joint_name))


def _encoder_degrees_to_urdf_angle(joint_idx: int, value_deg: float) -> float:
    value_rad = math.radians(float(value_deg))
    if joint_idx in {1, 2, 3, 5, 6, 7, 9, 10, 11, 13, 14, 15, 17, 18, 19}:
        return -abs(value_rad)
    return value_rad


FINGER_COLORS = {
    "thumb": QtGui.QColor(38, 128, 220),
    "index": QtGui.QColor(214, 226, 242),
    "middle": QtGui.QColor(190, 205, 226),
    "ring": QtGui.QColor(170, 188, 214),
    "pinky": QtGui.QColor(48, 132, 220),
}

FINGER_GL_COLORS = {
    "thumb": (0.15, 0.50, 0.86, 1.0),
    "index": (0.84, 0.89, 0.95, 1.0),
    "middle": (0.74, 0.81, 0.90, 1.0),
    "ring": (0.66, 0.74, 0.84, 1.0),
    "pinky": (0.19, 0.52, 0.86, 1.0),
}


def _joint_angle_map(model: URDFRobotHandModel, angles_deg: list[float]) -> dict[str, float]:
    out: dict[str, float] = {}
    for joint_name in model.revolute_joint_order:
        idx = _encoder_index_for_joint(joint_name)
        if idx is None or not 0 <= idx < ENCODER_COUNT:
            continue
        out[joint_name] = _encoder_degrees_to_urdf_angle(idx, float(angles_deg[idx]))
    return out


def _compute_finger_points(model: URDFRobotHandModel, angles_deg: list[float]) -> dict[str, list[np.ndarray]]:
    joint_angles = _joint_angle_map(model, angles_deg)
    points: dict[str, list[np.ndarray]] = {}
    for finger_name, chain in model.finger_chains.items():
        transform = chain.base_transform.copy()
        finger_points = [transform[:3, 3].copy()]
        for joint_name in chain.revolute_joint_names:
            spec = model.joint_specs[joint_name]
            angle = float(joint_angles.get(joint_name, 0.0))
            transform = transform @ model.joint_local_transform(spec, angle)
            finger_points.append(transform[:3, 3].copy())
        if len(finger_points) >= 2:
            direction = finger_points[-1] - finger_points[-2]
            norm = float(np.linalg.norm(direction))
            if norm > 1e-8:
                finger_points.append(finger_points[-1] + direction / norm * 0.018)
        points[finger_name] = finger_points
    return points


def _view_rotation() -> np.ndarray:
    azimuth = math.radians(-48.0)
    elevation = math.radians(26.0)
    cz, sz = math.cos(azimuth), math.sin(azimuth)
    cx, sx = math.cos(elevation), math.sin(elevation)
    rot_z = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]], dtype=float)
    rot_x = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]], dtype=float)
    return rot_x @ rot_z


class FastHandCanvas(QtWidgets.QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setMinimumSize(720, 540)
        self.setAutoFillBackground(False)
        self.model = URDFRobotHandModel.from_urdf(Path(DEFAULT_URDF_PATH))
        self.angles_deg = [0.0] * ENCODER_COUNT
        self._view = _view_rotation()
        self._cached_points: dict[str, list[np.ndarray]] = {}
        self._last_error = ""

    def set_angles(self, angles_deg: list[float]) -> None:
        values = [0.0] * ENCODER_COUNT
        for idx, value in enumerate(list(angles_deg)[:ENCODER_COUNT]):
            values[idx] = float(value)
        self.angles_deg = values
        try:
            self._cached_points = self._compute_finger_points(values)
            self._last_error = ""
        except Exception as exc:
            self._last_error = str(exc)
        self.update()

    def _joint_angle_map(self, angles_deg: list[float]) -> dict[str, float]:
        return _joint_angle_map(self.model, angles_deg)

    def _compute_finger_points(self, angles_deg: list[float]) -> dict[str, list[np.ndarray]]:
        return _compute_finger_points(self.model, angles_deg)

    def paintEvent(self, _event) -> None:
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QtGui.QColor(68, 68, 68))

        if self._last_error:
            painter.setPen(QtGui.QColor("#eeeeee"))
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter, self._last_error)
            painter.end()
            return

        all_points = [point for points in self._cached_points.values() for point in points]
        if not all_points:
            painter.setPen(QtGui.QColor("#eeeeee"))
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter, "waiting for q_target")
            painter.end()
            return

        gap = 10
        header_h = 22
        content = self.rect().adjusted(8, header_h + 8, -8, -8)
        left = QtCore.QRectF(content.left(), content.top(), (content.width() - gap) * 0.58, content.height())
        right = QtCore.QRectF(left.right() + gap, content.top(), (content.width() - gap) * 0.42, content.height())
        self._draw_view(painter, left, "Palm view: thumb/finger relation", axes=(0, 1), depth_axis=2)
        self._draw_view(painter, right, "Side view: flexion", axes=(0, 2), depth_axis=1)

        painter.setPen(QtGui.QColor("#dddddd"))
        painter.drawText(10, 18, "Fast URDF q_target viewer  |  yellow = thumb to nearest fingertip")
        painter.end()

    def _draw_view(
        self,
        painter: QtGui.QPainter,
        rect: QtCore.QRectF,
        title: str,
        axes: tuple[int, int],
        depth_axis: int,
    ) -> None:
        painter.setPen(QtGui.QPen(QtGui.QColor(150, 150, 150, 120), 1.0))
        painter.setBrush(QtGui.QColor(58, 58, 58, 255))
        painter.drawRect(rect)

        all_points = [point for points in self._cached_points.values() for point in points]
        arr = np.vstack(all_points)
        xy = arr[:, list(axes)]
        center = 0.5 * (xy.min(axis=0) + xy.max(axis=0))
        span = np.maximum(xy.max(axis=0) - xy.min(axis=0), 1e-6)
        margin = 34.0
        scale = min(
            (rect.width() - 2.0 * margin) / float(span[0]),
            (rect.height() - 2.0 * margin) / float(span[1]),
        )

        def project(point: np.ndarray) -> tuple[QtCore.QPointF, float]:
            p = np.asarray(point, dtype=float)
            v = p[list(axes)]
            x = (v[0] - center[0]) * scale + rect.center().x()
            y = rect.center().y() - (v[1] - center[1]) * scale
            return QtCore.QPointF(float(x), float(y)), float(p[depth_axis])

        self._draw_grid(painter, rect)
        self._draw_palm(painter, project)
        self._draw_thumb_relation(painter, project)
        self._draw_fingers(painter, project)

        painter.setPen(QtGui.QColor("#dddddd"))
        painter.drawText(rect.adjusted(8, 6, -8, -6), QtCore.Qt.AlignTop | QtCore.Qt.AlignLeft, title)

    @staticmethod
    def _draw_grid(painter: QtGui.QPainter, rect: QtCore.QRectF) -> None:
        painter.setPen(QtGui.QPen(QtGui.QColor(145, 145, 145, 70), 1.0))
        painter.drawLine(
            QtCore.QPointF(rect.left() + 8.0, rect.center().y()),
            QtCore.QPointF(rect.right() - 8.0, rect.center().y()),
        )
        painter.drawLine(
            QtCore.QPointF(rect.center().x(), rect.top() + 8.0),
            QtCore.QPointF(rect.center().x(), rect.bottom() - 8.0),
        )

    def _draw_fingers(self, painter: QtGui.QPainter, project) -> None:
        segments = []
        for finger_name, points in self._cached_points.items():
            for idx in range(len(points) - 1):
                p0, d0 = project(points[idx])
                p1, d1 = project(points[idx + 1])
                segments.append((0.5 * (d0 + d1), finger_name, idx, p0, p1))
        segments.sort(key=lambda item: item[0])

        for _depth, finger_name, idx, p0, p1 in segments:
            color = QtGui.QColor(FINGER_COLORS.get(finger_name, QtGui.QColor(210, 220, 235)))
            color.setAlpha(180 if idx == 0 else 235)
            width = 7.0 if idx == 0 else (9.5 if finger_name != "thumb" else 8.5)
            painter.setPen(QtGui.QPen(color, width, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap, QtCore.Qt.RoundJoin))
            painter.drawLine(p0, p1)

        for finger_name, points in self._cached_points.items():
            color = QtGui.QColor(FINGER_COLORS.get(finger_name, QtGui.QColor(210, 220, 235)))
            color.setAlpha(255)
            painter.setBrush(color)
            painter.setPen(QtGui.QPen(QtGui.QColor(30, 40, 55, 170), 1.0))
            for point in points[:-1]:
                qpoint, _depth = project(point)
                painter.drawEllipse(qpoint, 3.8, 3.8)
            tip, _depth = project(points[-1])
            painter.drawEllipse(tip, 5.2, 5.2)
            painter.setPen(QtGui.QColor("#eeeeee"))
            painter.drawText(tip + QtCore.QPointF(5.0, -5.0), finger_name)

    def _draw_thumb_relation(self, painter: QtGui.QPainter, project) -> None:
        thumb = self._cached_points.get("thumb")
        if not thumb:
            return
        thumb_tip = thumb[-1]
        nearest = None
        for finger_name in ("index", "middle", "ring", "pinky"):
            points = self._cached_points.get(finger_name)
            if not points:
                continue
            distance = float(np.linalg.norm(points[-1] - thumb_tip))
            if nearest is None or distance < nearest[0]:
                nearest = (distance, finger_name, points[-1])
        if nearest is None:
            return
        p0, _ = project(thumb_tip)
        p1, _ = project(nearest[2])
        painter.setPen(QtGui.QPen(QtGui.QColor(255, 213, 79, 220), 2.0, QtCore.Qt.DashLine))
        painter.drawLine(p0, p1)
        mid = QtCore.QPointF((p0.x() + p1.x()) * 0.5, (p0.y() + p1.y()) * 0.5)
        painter.setPen(QtGui.QColor(255, 235, 140))
        painter.drawText(mid + QtCore.QPointF(4.0, -4.0), f"{nearest[1]} {nearest[0] * 1000.0:.0f}mm")

    def _draw_palm(self, painter: QtGui.QPainter, project) -> None:
        roots = []
        for finger_name in ("index", "middle", "ring", "pinky"):
            points = self._cached_points.get(finger_name)
            if points:
                roots.append(points[0])
        if len(roots) < 2:
            return
        arr = np.vstack(roots)
        center = arr.mean(axis=0)
        width_vec = roots[-1] - roots[0]
        if float(np.linalg.norm(width_vec)) < 1e-8:
            width_vec = np.array([0.0, 1.0, 0.0], dtype=float)
        width_vec = width_vec / max(float(np.linalg.norm(width_vec)), 1e-8)
        forward_vec = center - np.zeros(3, dtype=float)
        if float(np.linalg.norm(forward_vec)) < 1e-8:
            forward_vec = np.array([1.0, 0.0, 0.0], dtype=float)
        forward_vec = forward_vec / max(float(np.linalg.norm(forward_vec)), 1e-8)
        palm = [
            roots[0] - width_vec * 0.010,
            roots[-1] + width_vec * 0.010,
            roots[-1] - forward_vec * 0.055 + width_vec * 0.004,
            roots[0] - forward_vec * 0.050 - width_vec * 0.004,
        ]
        polygon = QtGui.QPolygonF([project(point)[0] for point in palm])
        painter.setPen(QtGui.QPen(QtGui.QColor(40, 65, 95, 135), 1.0))
        painter.setBrush(QtGui.QColor(36, 101, 170, 95))
        painter.drawPolygon(polygon)


class FastHandGlView(QtWidgets.QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.urdf_path = Path(DEFAULT_URDF_PATH)
        self.model = URDFRobotHandModel.from_urdf(Path(DEFAULT_URDF_PATH))
        self._cached_points: dict[str, list[np.ndarray]] = {}
        self._link_poses: dict[str, np.ndarray] = {}
        self._visual_meshes = self._load_urdf_visual_meshes(self.urdf_path)
        self._mesh_items = {}
        self._segment_items = {}
        self._joint_item = None
        self._pinch_item = None
        self._palm_item = None

        import pyqtgraph as pg
        import pyqtgraph.opengl as gl

        self.pg = pg
        self.gl = gl

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.view = gl.GLViewWidget()
        self.view.setBackgroundColor((62, 62, 62))
        self.view.opts["distance"] = 0.28
        self.view.opts["elevation"] = 24
        self.view.opts["azimuth"] = -52
        layout.addWidget(self.view, 1)

        grid = gl.GLGridItem()
        grid.setSize(0.22, 0.22)
        grid.setSpacing(0.02, 0.02)
        self.view.addItem(grid)

    def set_angles(self, angles_deg: list[float]) -> None:
        values = [0.0] * ENCODER_COUNT
        for idx, value in enumerate(list(angles_deg)[:ENCODER_COUNT]):
            values[idx] = float(value)
        self._cached_points = _compute_finger_points(self.model, values)
        self._link_poses = self._compute_link_poses(_joint_angle_map(self.model, values))
        self._draw()

    def _draw(self) -> None:
        all_points = self._draw_visual_meshes()
        if all_points:
            pts = np.vstack(all_points)
            colors = np.tile(np.array([[0.95, 0.10, 0.08, 1.0]], dtype=float), (pts.shape[0], 1))
            if self._joint_item is None:
                self._joint_item = self.gl.GLScatterPlotItem(pos=pts, color=colors, size=7.0, pxMode=True)
                self.view.addItem(self._joint_item)
            else:
                self._joint_item.setData(pos=pts, color=colors, size=7.0)
            self._fit_camera(pts)
        self._draw_pinch_line()

    def _draw_visual_meshes(self) -> list[np.ndarray]:
        used = set()
        all_points = [np.asarray(points, dtype=float) for points in self._cached_points.values() if points]
        for spec in self._visual_meshes:
            link_name = spec["link"]
            pose = self._link_poses.get(link_name)
            if pose is None:
                continue
            vertices = spec["vertices"]
            world = vertices @ pose[:3, :3].T + pose[:3, 3]
            key = link_name
            used.add(key)
            item = self._mesh_items.get(key)
            color = spec["color"]
            if item is None:
                item = self.gl.GLMeshItem(
                    vertexes=world,
                    faces=spec["faces"],
                    smooth=False,
                    drawEdges=True,
                    edgeColor=(0.08, 0.12, 0.18, 0.32),
                    color=color,
                )
                try:
                    item.setGLOptions("translucent")
                except Exception:
                    pass
                self._mesh_items[key] = item
                self.view.addItem(item)
            else:
                item.setMeshData(vertexes=world, faces=spec["faces"])
                try:
                    item.setColor(color)
                except Exception:
                    pass
            all_points.append(world[:: max(1, len(world) // 20)])

        for key in list(self._mesh_items.keys()):
            if key not in used:
                self.view.removeItem(self._mesh_items.pop(key))
        return all_points

    def _compute_link_poses(self, joint_angles: dict[str, float]) -> dict[str, np.ndarray]:
        children: dict[str, list[str]] = {}
        all_links: set[str] = set()
        child_links: set[str] = set()
        for joint_name, spec in self.model.joint_specs.items():
            children.setdefault(spec.parent, []).append(joint_name)
            all_links.add(spec.parent)
            all_links.add(spec.child)
            child_links.add(spec.child)

        roots = ["base_link"] if "base_link" in all_links else sorted(all_links - child_links)
        poses: dict[str, np.ndarray] = {root: np.eye(4, dtype=float) for root in roots}
        queue = list(roots)
        while queue:
            parent = queue.pop(0)
            parent_pose = poses[parent]
            for joint_name in children.get(parent, []):
                spec = self.model.joint_specs[joint_name]
                angle = float(joint_angles.get(joint_name, 0.0))
                child_pose = parent_pose @ self.model.joint_local_transform(spec, angle)
                poses[spec.child] = child_pose
                queue.append(spec.child)
        return poses

    def _load_urdf_visual_meshes(self, urdf_path: Path) -> list[dict]:
        root = ET.parse(urdf_path).getroot()
        meshes: list[dict] = []
        for link in root.findall("link"):
            link_name = link.attrib.get("name", "")
            visual = link.find("visual")
            if not link_name or visual is None:
                continue
            mesh_elem = visual.find("geometry/mesh")
            if mesh_elem is None:
                continue
            filename = mesh_elem.attrib.get("filename", "")
            mesh_path = (urdf_path.parent / filename).resolve()
            if not mesh_path.exists():
                continue
            vertices, faces = self._load_stl(mesh_path)
            if vertices.size == 0 or faces.size == 0:
                continue
            scale = parse_xyz(mesh_elem.attrib.get("scale", "1 1 1"))
            origin = self._visual_origin_transform(visual)
            vertices = vertices * scale
            vertices = vertices @ origin[:3, :3].T + origin[:3, 3]
            vertices, faces = self._decimate_mesh(vertices, faces, max_triangles=4500)
            meshes.append(
                {
                    "link": link_name,
                    "vertices": vertices.astype(float),
                    "faces": faces.astype(int),
                    "color": self._mesh_color_for_link(link_name),
                }
            )
        return meshes

    @staticmethod
    def _visual_origin_transform(visual_elem) -> np.ndarray:
        origin_elem = visual_elem.find("origin")
        if origin_elem is None:
            return np.eye(4, dtype=float)
        xyz = parse_xyz(origin_elem.attrib.get("xyz", "0 0 0"))
        rpy = parse_xyz(origin_elem.attrib.get("rpy", "0 0 0"))
        return make_transform(rpy_to_rotation(*rpy), xyz)

    @staticmethod
    def _load_stl(path: Path) -> tuple[np.ndarray, np.ndarray]:
        data = path.read_bytes()
        if len(data) < 84:
            return np.zeros((0, 3), dtype=float), np.zeros((0, 3), dtype=int)
        tri_count = struct.unpack("<I", data[80:84])[0]
        expected = 84 + tri_count * 50
        if expected > len(data):
            return np.zeros((0, 3), dtype=float), np.zeros((0, 3), dtype=int)
        vertices = []
        faces = []
        offset = 84
        for tri_idx in range(tri_count):
            values = struct.unpack("<12fH", data[offset : offset + 50])
            base = len(vertices)
            vertices.extend((values[3:6], values[6:9], values[9:12]))
            faces.append((base, base + 1, base + 2))
            offset += 50
        return np.asarray(vertices, dtype=float), np.asarray(faces, dtype=int)

    @staticmethod
    def _decimate_mesh(vertices: np.ndarray, faces: np.ndarray, max_triangles: int) -> tuple[np.ndarray, np.ndarray]:
        if len(faces) <= max_triangles:
            return vertices, faces
        step = max(1, int(math.ceil(len(faces) / float(max_triangles))))
        sampled_faces = faces[::step]
        used = np.unique(sampled_faces.reshape(-1))
        remap = {int(old): idx for idx, old in enumerate(used)}
        remapped = np.vectorize(lambda value: remap[int(value)])(sampled_faces)
        return vertices[used], remapped.astype(int)

    @staticmethod
    def _mesh_color_for_link(link_name: str) -> tuple[float, float, float, float]:
        if "thumb" in link_name:
            return (0.15, 0.50, 0.86, 0.58)
        if "palm" in link_name or link_name == "base_link":
            return (0.18, 0.42, 0.72, 0.35)
        if "finger_4" in link_name:
            return (0.19, 0.52, 0.86, 0.62)
        if "finger_3" in link_name:
            return (0.66, 0.74, 0.84, 0.70)
        if "finger_2" in link_name:
            return (0.74, 0.81, 0.90, 0.72)
        return (0.84, 0.89, 0.95, 0.74)

    def _draw_palm(self) -> None:
        roots = []
        for finger_name in ("index", "middle", "ring", "pinky"):
            points = self._cached_points.get(finger_name)
            if points:
                roots.append(points[0])
        if len(roots) < 2:
            return
        arr = np.vstack(roots)
        center = arr.mean(axis=0)
        width_vec = roots[-1] - roots[0]
        width_norm = float(np.linalg.norm(width_vec))
        if width_norm < 1e-8:
            width_vec = np.array([0.0, 1.0, 0.0], dtype=float)
        else:
            width_vec = width_vec / width_norm
        forward_vec = center - np.zeros(3, dtype=float)
        forward_norm = float(np.linalg.norm(forward_vec))
        if forward_norm < 1e-8:
            forward_vec = np.array([1.0, 0.0, 0.0], dtype=float)
        else:
            forward_vec = forward_vec / forward_norm
        normal_vec = np.cross(forward_vec, width_vec)
        normal_norm = float(np.linalg.norm(normal_vec))
        if normal_norm < 1e-8:
            normal_vec = np.array([0.0, 0.0, 1.0], dtype=float)
        else:
            normal_vec = normal_vec / normal_norm
        palm_center = center - forward_vec * 0.030
        vertices, faces = self._oriented_box(
            palm_center,
            forward_vec,
            width_vec,
            normal_vec,
            np.array([0.065, 0.095, 0.018], dtype=float),
        )
        if self._palm_item is None:
            self._palm_item = self.gl.GLMeshItem(
                vertexes=vertices,
                faces=faces,
                smooth=False,
                drawEdges=True,
                edgeColor=(0.10, 0.15, 0.22, 0.45),
                color=(0.18, 0.42, 0.72, 0.35),
            )
            try:
                self._palm_item.setGLOptions("translucent")
            except Exception:
                pass
            self.view.addItem(self._palm_item)
        else:
            self._palm_item.setMeshData(vertexes=vertices, faces=faces)
            try:
                self._palm_item.setColor((0.18, 0.42, 0.72, 0.35))
            except Exception:
                pass

    def _draw_pinch_line(self) -> None:
        thumb = self._cached_points.get("thumb")
        if not thumb:
            return
        thumb_tip = thumb[-1]
        nearest = None
        for finger_name in ("index", "middle", "ring", "pinky"):
            points = self._cached_points.get(finger_name)
            if not points:
                continue
            distance = float(np.linalg.norm(points[-1] - thumb_tip))
            if nearest is None or distance < nearest[0]:
                nearest = (distance, points[-1])
        if nearest is None:
            return
        arr = np.vstack([thumb_tip, nearest[1]]).astype(float)
        if self._pinch_item is None:
            self._pinch_item = self.gl.GLLinePlotItem(
                pos=arr,
                color=(1.0, 0.82, 0.18, 1.0),
                width=3.0,
                antialias=True,
            )
            self.view.addItem(self._pinch_item)
        else:
            self._pinch_item.setData(pos=arr, color=(1.0, 0.82, 0.18, 1.0), width=3.0)

    def _fit_camera(self, pts: np.ndarray) -> None:
        arr = np.asarray(pts, dtype=float)
        if arr.ndim != 2 or arr.shape[1] != 3:
            return
        center = 0.5 * (arr.min(axis=0) + arr.max(axis=0))
        span = arr.max(axis=0) - arr.min(axis=0)
        try:
            self.view.opts["center"] = self.pg.Vector(float(center[0]), float(center[1]), float(center[2]))
        except Exception:
            self.view.opts["center"] = tuple(float(v) for v in center)
        self.view.opts["distance"] = max(0.16, min(0.55, float(np.linalg.norm(span)) * 2.0))

    @staticmethod
    def _cuboid_between(p0: np.ndarray, p1: np.ndarray, width: float) -> tuple[np.ndarray, np.ndarray]:
        p0 = np.asarray(p0, dtype=float)
        p1 = np.asarray(p1, dtype=float)
        axis = p1 - p0
        length = float(np.linalg.norm(axis))
        if length < 1e-8:
            axis = np.array([1.0, 0.0, 0.0], dtype=float)
            length = 1e-8
        x_axis = axis / length
        helper = np.array([0.0, 0.0, 1.0], dtype=float)
        if abs(float(np.dot(x_axis, helper))) > 0.92:
            helper = np.array([0.0, 1.0, 0.0], dtype=float)
        y_axis = np.cross(x_axis, helper)
        y_axis /= max(float(np.linalg.norm(y_axis)), 1e-8)
        z_axis = np.cross(x_axis, y_axis)
        z_axis /= max(float(np.linalg.norm(z_axis)), 1e-8)
        return FastHandGlView._oriented_box(
            0.5 * (p0 + p1),
            x_axis,
            y_axis,
            z_axis,
            np.array([length, width, width], dtype=float),
        )

    @staticmethod
    def _oriented_box(
        center: np.ndarray,
        x_axis: np.ndarray,
        y_axis: np.ndarray,
        z_axis: np.ndarray,
        size_xyz: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        center = np.asarray(center, dtype=float)
        axes = [
            np.asarray(x_axis, dtype=float),
            np.asarray(y_axis, dtype=float),
            np.asarray(z_axis, dtype=float),
        ]
        axes = [axis / max(float(np.linalg.norm(axis)), 1e-8) for axis in axes]
        half = 0.5 * np.asarray(size_xyz, dtype=float)
        vertices = []
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                for sz in (-1.0, 1.0):
                    vertices.append(
                        center
                        + sx * half[0] * axes[0]
                        + sy * half[1] * axes[1]
                        + sz * half[2] * axes[2]
                    )
        faces = np.array(
            [
                [0, 1, 3], [0, 3, 2],
                [4, 6, 7], [4, 7, 5],
                [0, 4, 5], [0, 5, 1],
                [2, 3, 7], [2, 7, 6],
                [0, 2, 6], [0, 6, 4],
                [1, 5, 7], [1, 7, 3],
            ],
            dtype=int,
        )
        return np.asarray(vertices, dtype=float), faces


class RawIkUrdfViewer(QtWidgets.QWidget):
    def __init__(self, stream_path: str, parent=None) -> None:
        super().__init__(parent)
        self.stream_path = os.path.abspath(stream_path)
        self._last_timestamp: Optional[float] = None
        self._last_mtime: Optional[float] = None

        self.setWindowTitle("Raw IK URDF q_target Viewer")
        self.resize(760, 620)

        layout = QtWidgets.QVBoxLayout(self)
        self.status_label = QtWidgets.QLabel(f"stream: {self.stream_path}")
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)

        try:
            self.canvas = FastHandGlView()
            self.status_suffix = "3D OpenGL"
        except Exception as exc:
            self.canvas = FastHandCanvas()
            self.status_suffix = f"software fallback: {exc}"
        layout.addWidget(self.canvas, 1)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._poll_stream)
        self.timer.start(33)
        self._poll_stream()

    def _poll_stream(self) -> None:
        try:
            stat = os.stat(self.stream_path)
        except OSError:
            self.status_label.setText(f"waiting for stream: {self.stream_path}")
            return
        if self._last_mtime is not None and stat.st_mtime <= self._last_mtime:
            self._update_age()
            return
        self._last_mtime = stat.st_mtime
        try:
            with open(self.stream_path, "r", encoding="utf-8") as handle:
                payload = json.load(handle)
            angles = list(payload.get("angles_deg", []))
            while len(angles) < ENCODER_COUNT:
                angles.append(0.0)
            angles = [float(value) for value in angles[:ENCODER_COUNT]]
            self._last_timestamp = float(payload.get("timestamp", time.time()))
            self.canvas.set_angles(angles)
            self._update_age()
        except Exception as exc:
            self.status_label.setText(f"stream read failed: {exc}")

    def _update_age(self) -> None:
        if self._last_timestamp is None:
            return
        age_ms = max(0.0, (time.time() - self._last_timestamp) * 1000.0)
        self.status_label.setText(
            f"stream: {self.stream_path}    age={age_ms:.0f}ms    {self.status_suffix}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="Standalone Raw IK URDF q_target viewer.")
    parser.add_argument(
        "--stream",
        default=os.path.join(DESKTOP_ROOT, "config", "raw_ik_q_target.json"),
        help="Path to q_target JSON stream written by the main UI.",
    )
    args = parser.parse_args()

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    window = RawIkUrdfViewer(args.stream)
    window.show()
    return int(app.exec())


if __name__ == "__main__":
    raise SystemExit(main())
