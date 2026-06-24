import os
import re
import math
import struct
import xml.etree.ElementTree as ET
from typing import List, Optional

import numpy as np

from protocol import ENCODER_COUNT
from visualize.plot_hand_from_urdf import (
    DISPLAY_TRANSFORM,
    UrdfKinematics,
    apply_display_transform,
    motor_deg_to_urdf_joint_dict,
)
try:
    from teleop.manus_raw_skeleton_retarget import (
        _encoder_degrees_to_urdf_angle,
        _encoder_index_for_joint,
    )
except Exception:
    _encoder_degrees_to_urdf_angle = None
    _encoder_index_for_joint = None
from .ui_qt_compat import QtCore, QtGui, QtWidgets


DESKTOP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _unit_vector(vec: np.ndarray) -> np.ndarray:
    arr = np.asarray(vec, dtype=float)
    norm = float(np.linalg.norm(arr))
    if norm < 1e-10:
        return np.zeros_like(arr)
    return arr / norm


SEMANTIC_URDF_REVOLUTE_JOINTS = (
    "little_aa_joint",
    "mmhand_finger_4_proximal_phalanx_1_mcp_fe_joint",
    "mmhand_finger_4_distal_phalanx_1_pip_joint",
    "mmhand_finger_4_fingertip_1_dip_joint",
    "ring_aa_joint",
    "mmhand_finger_3_proximal_phalanx_1_mcp_fe_joint",
    "mmhand_finger_3_distal_phalanx_1_pip_joint",
    "mmhand_finger_3_fingertip_1_dip_joint",
    "middle_aa_joint",
    "mmhand_finger_2_proximal_phalanx_1_mcp_fe_joint",
    "mmhand_finger_2_distal_phalanx_1_pip_joint",
    "mmhand_finger_2_fingertip_1_dip_joint",
    "index_aa_joint",
    "mmhand_finger_1_proximal_phalanx_1_mcp_fe_joint",
    "mmhand_finger_1_distal_phalanx_1_pip_joint",
    "mmhand_finger_1_fingertip_1_dip_joint",
    "thumb_cmc1_joint",
    "thumb_aa_joint",
    "mmhand_thumb_1_mmhand_finger_5_proximal_phalanx_1_mcp_fe_joint",
    "mmhand_thumb_1_mmhand_finger_5_distal_phalanx_1_pip_joint",
    "mmhand_thumb_1_mmhand_finger_5_fingertip_1_dip_joint",
)


class Pose3DView(QtWidgets.QWidget):
    def __init__(self, context, parent=None, *, show_projection: bool = False, render_style: str = "lines"):
        super().__init__(parent)
        self.context = context
        self._show_projection = bool(show_projection)
        self._render_style = str(render_style or "lines")
        self._kin = None
        self._gl = None
        self._grid_item = None
        self._label = None
        self._projection_label = None
        self._link_items = {}
        self._mesh_items = {}
        self._visual_meshes = []
        self._semantic_tip_offsets = {}
        self._node_item = None
        self._joint_node_item = None
        self._last_angles: Optional[List[float]] = None
        self._closed = False
        self._paused = False
        self._build_ui()
        self._load_urdf()

    def _build_ui(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        if self._render_style in ("points21", "software3d"):
            label = (
                "URDF q_target software 3D waiting"
                if self._render_style == "software3d"
                else "URDF q_target 21-point projection waiting"
            )
            self._projection_label = QtWidgets.QLabel(label)
            self._projection_label.setAlignment(QtCore.Qt.AlignCenter)
            self._projection_label.setMinimumHeight(300)
            self._projection_label.setStyleSheet("background:#4e4e4e;color:#dddddd;border:1px solid #d0d0d0;")
            layout.addWidget(self._projection_label, 1)
            return
        try:
            import pyqtgraph.opengl as gl

            self._gl_mod = gl
            self._gl = gl.GLViewWidget()
            if self._render_style in ("boxes", "mesh"):
                self._gl.setBackgroundColor((78, 78, 78))
                self._gl.opts["distance"] = 0.34
                self._gl.opts["elevation"] = 28
                self._gl.opts["azimuth"] = -48
            else:
                self._gl.setBackgroundColor("w")
                self._gl.opts["distance"] = 0.42
                self._gl.opts["elevation"] = 18
                self._gl.opts["azimuth"] = -68
            layout.addWidget(self._gl, 1)
            grid = gl.GLGridItem()
            grid.setSize(0.24, 0.24)
            grid.setSpacing(0.04 if self._render_style in ("boxes", "mesh") else 0.02, 0.04 if self._render_style in ("boxes", "mesh") else 0.02)
            self._grid_item = grid
            self._gl.addItem(grid)
        except Exception as exc:
            self._gl_mod = None
            self._label = QtWidgets.QLabel(f"OpenGL pose view unavailable:\n{exc}")
            self._label.setAlignment(QtCore.Qt.AlignCenter)
            layout.addWidget(self._label, 1)
        if self._show_projection:
            self._projection_label = QtWidgets.QLabel("URDF projection waiting for q_target")
            self._projection_label.setAlignment(QtCore.Qt.AlignCenter)
            self._projection_label.setMinimumHeight(180)
            self._projection_label.setStyleSheet("background:white;color:#555;border:1px solid #d0d0d0;")
            layout.addWidget(self._projection_label, 1)

    def _load_urdf(self) -> None:
        urdf_name = "hand_glb.urdf" if self._render_style == "mesh" else "hand.urdf"
        urdf_path = os.path.join(DESKTOP_DIR, "hku_hand_v2_urdf", "urdf", urdf_name)
        try:
            self._kin = UrdfKinematics(urdf_path)
            self._visual_meshes = self._load_urdf_visual_meshes(urdf_path)
            self._semantic_tip_offsets = self._load_semantic_tip_offsets(urdf_path)
        except Exception as exc:
            self._kin = None
            if self._label is not None:
                self._label.setText(f"URDF load failed:\n{urdf_path}\n{exc}")
        self.draw_angles([0.0] * ENCODER_COUNT)

    def build(self, _parent) -> None:
        pass

    def update(self, state) -> None:
        if self._paused:
            return
        self.draw_angles(getattr(state, "angles", [0.0] * ENCODER_COUNT))

    def draw_angles(self, angles: List[float], state=None, *, generate: Optional[bool] = None) -> None:
        _ = state, generate
        self._last_angles = list(angles or [0.0] * ENCODER_COUNT)
        if self._closed or self._paused or self._kin is None:
            return
        if self._gl is None and self._render_style not in ("points21", "software3d"):
            return
        joint_pos = self._encoder_degrees_to_urdf_joint_dict(self._last_angles)
        poses = apply_display_transform(self._kin.compute_link_poses(joint_pos), DISPLAY_TRANSFORM)
        if self._render_style == "software3d":
            self._draw_software_meshes(poses)
            return
        if self._render_style == "points21":
            self._draw_projection(poses)
            return
        self._draw_projection(poses)
        if self._render_style in ("mesh", "boxes"):
            self._draw_visual_meshes(poses)
            return
        self._draw_line_links(poses)

    def _draw_visual_meshes(self, poses: dict) -> None:
        gl = self._gl_mod
        if gl is None or self._gl is None:
            return
        if not self._visual_meshes:
            self._draw_line_links(poses)
            return
        if self._node_item is not None:
            try:
                self._gl.removeItem(self._node_item)
            except Exception:
                pass
            self._node_item = None
        if self._joint_node_item is not None:
            try:
                self._gl.removeItem(self._joint_node_item)
            except Exception:
                pass
            self._joint_node_item = None
        for item in list(self._link_items.values()):
            try:
                self._gl.removeItem(item)
            except Exception:
                pass
        self._link_items.clear()

        used = set()
        all_points = []
        for spec in self._visual_meshes:
            link_name = spec["link"]
            if link_name not in poses:
                continue
            key = (link_name, spec["path"])
            used.add(key)
            transform = poses[link_name] @ spec["origin"]
            vertices = self._transform_vertices(spec["vertices"], transform)
            faces = spec["faces"]
            if vertices.size == 0 or faces.size == 0:
                continue
            all_points.append(vertices)
            color = tuple(spec["color"])
            draw_edges = self._render_style == "boxes"
            if self._render_style == "boxes" and len(color) == 4:
                color = (color[0], color[1], color[2], 0.76)
            item = self._mesh_items.get(key)
            if item is None:
                item = gl.GLMeshItem(
                    vertexes=vertices,
                    faces=faces,
                    smooth=False,
                    drawEdges=draw_edges,
                    edgeColor=(0.25, 0.32, 0.42, 0.55),
                    color=color,
                )
                try:
                    item.setGLOptions("translucent" if self._render_style == "boxes" else "opaque")
                except Exception:
                    pass
                self._mesh_items[key] = item
                self._gl.addItem(item)
            else:
                item.setMeshData(vertexes=vertices, faces=faces)
                try:
                    item.setColor(color)
                except Exception:
                    pass

        for key in list(self._mesh_items.keys()):
            if key not in used:
                try:
                    self._gl.removeItem(self._mesh_items[key])
                except Exception:
                    pass
                self._mesh_items.pop(key, None)

        if all_points:
            self._fit_camera_to_points(np.vstack(all_points))

    def _load_urdf_visual_meshes(self, urdf_path: str) -> list[dict]:
        urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
        root = ET.parse(urdf_path).getroot()
        meshes: list[dict] = []
        for link in root.findall("link"):
            link_name = link.attrib.get("name", "")
            if not link_name:
                continue
            visual = link.find("visual")
            if visual is None:
                continue
            geometry = visual.find("geometry")
            mesh_elem = geometry.find("mesh") if geometry is not None else None
            if mesh_elem is None:
                continue
            filename = mesh_elem.attrib.get("filename", "")
            if not filename:
                continue
            mesh_path = self._resolve_mesh_path(urdf_dir, filename)
            if not os.path.exists(mesh_path):
                continue
            scale = self._parse_xyz(mesh_elem.attrib.get("scale", "1 1 1"))
            origin = self._visual_origin_transform(visual)
            vertices, faces = self._load_stl(mesh_path)
            if vertices.size == 0 or faces.size == 0:
                continue
            vertices = vertices * scale.reshape(1, 3)
            max_triangles = 1200 if self._render_style == "software3d" else 3500
            vertices, faces = self._decimate_mesh(vertices, faces, max_triangles=max_triangles)
            meshes.append(
                {
                    "link": link_name,
                    "path": mesh_path,
                    "origin": origin,
                    "vertices": vertices,
                    "faces": faces,
                    "bbox_min": vertices.min(axis=0),
                    "bbox_max": vertices.max(axis=0),
                    "color": self._mesh_color_for_link(link_name),
                }
            )
        return meshes

    def _draw_software_meshes(self, poses: dict) -> None:
        if self._projection_label is None or self._kin is None:
            return
        width = max(320, int(self._projection_label.width() or 720))
        height = max(240, int(self._projection_label.height() or 360))
        pixmap = QtGui.QPixmap(width, height)
        pixmap.fill(QtGui.QColor("#4e4e4e"))

        meshes = []
        all_vertices = []
        for spec in self._visual_meshes:
            link_name = spec["link"]
            if link_name not in poses:
                continue
            transform = poses[link_name] @ spec["origin"]
            vertices = self._transform_vertices(spec["vertices"], transform)
            faces = spec["faces"]
            if vertices.size == 0 or faces.size == 0:
                continue
            meshes.append((vertices, faces, tuple(spec["color"])))
            all_vertices.append(vertices)

        if not all_vertices:
            painter = QtGui.QPainter(pixmap)
            painter.setPen(QtGui.QColor("#dddddd"))
            painter.drawText(pixmap.rect(), QtCore.Qt.AlignCenter, "No URDF poses to draw")
            painter.end()
            self._projection_label.setPixmap(pixmap)
            return

        points = np.vstack(all_vertices)
        view = self._software_view_transform(points)
        projected = points @ view.T
        center = 0.5 * (projected.min(axis=0) + projected.max(axis=0))
        span = np.maximum(projected.max(axis=0) - projected.min(axis=0), 1e-6)
        margin = 20.0
        scale = min((width - 2.0 * margin) / float(span[0]), (height - 2.0 * margin) / float(span[2]))

        def project(vertices: np.ndarray) -> np.ndarray:
            p = vertices @ view.T
            xy = np.empty((p.shape[0], 2), dtype=float)
            xy[:, 0] = (p[:, 0] - center[0]) * scale + width * 0.5
            xy[:, 1] = height * 0.5 - (p[:, 2] - center[2]) * scale
            return xy

        triangles = []
        light = _unit_vector(np.array([-0.25, -0.55, 0.95], dtype=float))
        for vertices, faces, color in meshes:
            xy = project(vertices)
            v_view = vertices @ view.T
            rgb = np.array(color[:3], dtype=float)
            alpha = float(color[3]) if len(color) >= 4 else 1.0
            faces_arr = np.asarray(faces, dtype=int)
            for face in faces_arr:
                if face.size < 3 or np.any(face < 0) or np.any(face >= vertices.shape[0]):
                    continue
                tri = v_view[face[:3]]
                normal = np.cross(tri[1] - tri[0], tri[2] - tri[0])
                normal_norm = float(np.linalg.norm(normal))
                if normal_norm < 1e-10:
                    continue
                normal = normal / normal_norm
                shade = 0.45 + 0.55 * max(0.0, float(np.dot(normal, light)))
                face_color = np.clip(rgb * shade, 0.0, 1.0)
                depth = float(np.mean(tri[:, 1]))
                triangles.append((depth, xy[face[:3]], face_color, alpha))

        triangles.sort(key=lambda item: item[0])
        painter = QtGui.QPainter(pixmap)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        painter.fillRect(pixmap.rect(), QtGui.QColor("#4e4e4e"))

        grid_pen = QtGui.QPen(QtGui.QColor(130, 130, 130, 95), 1.0)
        painter.setPen(grid_pen)
        cx = width * 0.5
        cy = height * 0.5
        painter.drawLine(QtCore.QPointF(0.0, cy + 80.0), QtCore.QPointF(width, cy - 80.0))
        painter.drawLine(QtCore.QPointF(cx - 180.0, height), QtCore.QPointF(cx + 180.0, 0.0))

        edge_pen = QtGui.QPen(QtGui.QColor(45, 55, 70, 85), 0.55)
        for _depth, tri_xy, face_color, alpha in triangles:
            qcolor = QtGui.QColor(
                int(face_color[0] * 255.0),
                int(face_color[1] * 255.0),
                int(face_color[2] * 255.0),
                int(max(0.25, min(alpha, 1.0)) * 255.0),
            )
            polygon = QtGui.QPolygonF([QtCore.QPointF(float(x), float(y)) for x, y in tri_xy])
            painter.setPen(edge_pen)
            painter.setBrush(qcolor)
            painter.drawPolygon(polygon)

        painter.setPen(QtGui.QColor("#dddddd"))
        painter.drawText(8, 18, "URDF q_target software 3D")
        painter.end()
        self._projection_label.setPixmap(pixmap)

    @staticmethod
    def _software_view_transform(points: np.ndarray) -> np.ndarray:
        _ = points
        azimuth = math.radians(-42.0)
        elevation = math.radians(24.0)
        cz, sz = math.cos(azimuth), math.sin(azimuth)
        cx, sx = math.cos(elevation), math.sin(elevation)
        rot_z = np.array(
            [
                [cz, -sz, 0.0],
                [sz, cz, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        rot_x = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, cx, -sx],
                [0.0, sx, cx],
            ],
            dtype=float,
        )
        return rot_x @ rot_z

    @staticmethod
    def _resolve_mesh_path(urdf_dir: str, filename: str) -> str:
        name = str(filename)
        if name.startswith("package://"):
            name = name.split("/", 3)[-1] if "/" in name else name
        name = name.replace("\\", os.sep).replace("/", os.sep)
        path = os.path.normpath(os.path.join(urdf_dir, name))
        if os.path.exists(path):
            return path
        alt = os.path.normpath(os.path.join(os.path.dirname(urdf_dir), name))
        if os.path.exists(alt):
            return alt
        return path

    @staticmethod
    def _parse_xyz(text: str) -> np.ndarray:
        values = [float(x) for x in re.split(r"[\s,]+", str(text).strip()) if x]
        while len(values) < 3:
            values.append(0.0)
        return np.asarray(values[:3], dtype=float)

    @classmethod
    def _visual_origin_transform(cls, visual_elem) -> np.ndarray:
        origin_elem = visual_elem.find("origin")
        if origin_elem is None:
            xyz = np.zeros(3, dtype=float)
            rpy = np.zeros(3, dtype=float)
        else:
            xyz = cls._parse_xyz(origin_elem.attrib.get("xyz", "0 0 0"))
            rpy = cls._parse_xyz(origin_elem.attrib.get("rpy", "0 0 0"))
        transform = np.eye(4, dtype=float)
        transform[:3, :3] = cls._rpy_to_rotation(rpy)
        transform[:3, 3] = xyz
        return transform

    @staticmethod
    def _rpy_to_rotation(rpy: np.ndarray) -> np.ndarray:
        roll, pitch, yaw = [float(v) for v in rpy[:3]]
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=float)
        ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=float)
        rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=float)
        return rz @ ry @ rx

    @staticmethod
    def _load_stl(path: str) -> tuple[np.ndarray, np.ndarray]:
        data = open(path, "rb").read()
        vertices = []
        if len(data) >= 84:
            tri_count = struct.unpack("<I", data[80:84])[0]
            if 84 + tri_count * 50 == len(data):
                offset = 84
                for _ in range(tri_count):
                    offset += 12
                    for _vertex in range(3):
                        vertices.append(struct.unpack("<fff", data[offset:offset + 12]))
                        offset += 12
                    offset += 2
                verts = np.asarray(vertices, dtype=float)
                faces = np.arange(verts.shape[0], dtype=np.uint32).reshape((-1, 3))
                return verts, faces

        text = data.decode("utf-8", errors="ignore")
        for match in re.finditer(r"vertex\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)", text):
            vertices.append((float(match.group(1)), float(match.group(2)), float(match.group(3))))
        verts = np.asarray(vertices, dtype=float)
        if verts.size == 0:
            return np.empty((0, 3), dtype=float), np.empty((0, 3), dtype=np.uint32)
        faces = np.arange(verts.shape[0], dtype=np.uint32).reshape((-1, 3))
        return verts, faces

    @staticmethod
    def _decimate_mesh(vertices: np.ndarray, faces: np.ndarray, max_triangles: int) -> tuple[np.ndarray, np.ndarray]:
        if max_triangles <= 0 or faces.shape[0] <= max_triangles:
            return vertices, faces
        indices = np.linspace(0, faces.shape[0] - 1, max_triangles, dtype=np.int64)
        selected = faces[indices].reshape(-1)
        reduced_vertices = vertices[selected]
        reduced_faces = np.arange(reduced_vertices.shape[0], dtype=np.uint32).reshape((-1, 3))
        return reduced_vertices, reduced_faces

    @staticmethod
    def _transform_vertices(vertices: np.ndarray, transform: np.ndarray) -> np.ndarray:
        verts = np.asarray(vertices, dtype=float)
        homogeneous = np.ones((verts.shape[0], 4), dtype=float)
        homogeneous[:, :3] = verts
        return (homogeneous @ transform.T)[:, :3]

    @staticmethod
    def _box_from_bounds(bbox_min: np.ndarray, bbox_max: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        mn = np.asarray(bbox_min, dtype=float)
        mx = np.asarray(bbox_max, dtype=float)
        if mn.shape != (3,) or mx.shape != (3,) or not np.all(np.isfinite(mn)) or not np.all(np.isfinite(mx)):
            return np.empty((0, 3), dtype=float), np.empty((0, 3), dtype=int)
        eps = 1e-4
        mx = np.maximum(mx, mn + eps)
        vertices = np.array(
            [
                [mn[0], mn[1], mn[2]],
                [mn[0], mn[1], mx[2]],
                [mn[0], mx[1], mn[2]],
                [mn[0], mx[1], mx[2]],
                [mx[0], mn[1], mn[2]],
                [mx[0], mn[1], mx[2]],
                [mx[0], mx[1], mn[2]],
                [mx[0], mx[1], mx[2]],
            ],
            dtype=float,
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
        return vertices, faces

    @classmethod
    def _software_link_box(
        cls,
        link_name: str,
        bbox_min: np.ndarray,
        bbox_max: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        name = str(link_name).lower()
        if name == "base_link":
            return cls._box_from_center_size(
                np.array([-0.010, 0.000, -0.012], dtype=float),
                np.array([0.070, 0.082, 0.030], dtype=float),
            )
        if "finger_base" in name:
            return cls._box_from_center_size(
                np.zeros(3, dtype=float),
                np.array([0.020, 0.022, 0.014], dtype=float),
            )
        if "thumb_base" in name:
            return cls._box_from_center_size(
                np.zeros(3, dtype=float),
                np.array([0.022, 0.026, 0.018], dtype=float),
            )
        return cls._box_from_bounds(bbox_min, bbox_max)

    @classmethod
    def _box_from_center_size(
        cls,
        center: np.ndarray,
        size_xyz: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        center = np.asarray(center, dtype=float)
        half = 0.5 * np.asarray(size_xyz, dtype=float)
        return cls._box_from_bounds(center - half, center + half)

    @staticmethod
    def _mesh_color_for_link(link_name: str) -> tuple[float, float, float, float]:
        name = str(link_name).lower()
        if name == "base_link":
            return (0.20, 0.42, 0.70, 0.48)
        if "finger_base" in name:
            return (0.20, 0.42, 0.70, 0.68)
        if "thumb" in name:
            return (0.18, 0.46, 0.78, 1.0)
        if "abduction_adduction" in name:
            return (0.30, 0.54, 0.82, 1.0)
        return (0.72, 0.78, 0.88, 1.0)

    def _draw_line_links(self, poses: dict) -> None:
        gl = self._gl_mod
        if gl is None or self._gl is None:
            return
        used = set()
        segments = self._semantic_revolute_segments(poses) + self._semantic_tip_segments(poses)
        all_points = []
        for key, p0, p1, _joint in segments:
            if not np.all(np.isfinite(p0)) or not np.all(np.isfinite(p1)):
                continue
            length = float(np.linalg.norm(p1 - p0))
            if length < 1e-6:
                continue
            used.add(key)
            all_points.extend([p0, p1])
            vertices, faces = self._cuboid_between(p0, p1, 0.004)
            item = self._link_items.get(key)
            if item is None:
                item = gl.GLMeshItem(
                    vertexes=vertices,
                    faces=faces,
                    smooth=False,
                    drawEdges=False,
                    color=(0.02, 0.02, 0.02, 1.0),
                )
                self._link_items[key] = item
                self._gl.addItem(item)
            else:
                item.setMeshData(vertexes=vertices, faces=faces)
                try:
                    item.setColor((0.02, 0.02, 0.02, 1.0))
                except Exception:
                    pass
        for key in list(self._link_items.keys()):
            if key not in used:
                try:
                    self._gl.removeItem(self._link_items[key])
                except Exception:
                    pass
                self._link_items.pop(key, None)
        pts = self._semantic_revolute_points(segments)
        self._draw_joint_nodes(pts)
        if all_points:
            self._fit_camera_to_points(np.asarray(all_points, dtype=float))

    def _draw_box_links(self, poses: dict) -> None:
        gl = self._gl_mod
        if gl is None or self._gl is None or self._kin is None:
            return
        if self._node_item is not None:
            try:
                self._gl.removeItem(self._node_item)
            except Exception:
                pass
            self._node_item = None

        used = set()
        all_points = []
        joint_points = []

        if "base_link" in poses:
            base_pose = np.asarray(poses["base_link"], dtype=float)
            palm_vertices, palm_faces = self._cuboid_at_pose(
                base_pose,
                np.array([-0.010, 0.000, -0.012], dtype=float),
                np.array([0.070, 0.082, 0.030], dtype=float),
            )
            key = ("semantic_palm", "base_link")
            used.add(key)
            all_points.append(palm_vertices)
            item = self._link_items.get(key)
            color = (0.22, 0.48, 0.78, 0.58)
            if item is None:
                item = gl.GLMeshItem(
                    vertexes=palm_vertices,
                    faces=palm_faces,
                    smooth=False,
                    drawEdges=True,
                    edgeColor=(0.30, 0.38, 0.48, 0.65),
                    color=color,
                )
                try:
                    item.setGLOptions("translucent")
                except Exception:
                    pass
                self._link_items[key] = item
                self._gl.addItem(item)
            else:
                item.setMeshData(vertexes=palm_vertices, faces=palm_faces)
                item.setColor(color)

        for key, p0, p1, joint in self._semantic_revolute_segments(poses) + self._semantic_tip_segments(poses):
            if not np.all(np.isfinite(p0)) or not np.all(np.isfinite(p1)):
                continue
            length = float(np.linalg.norm(p1 - p0))
            used.add(key)
            all_points.extend([p0, p1])
            joint_points.extend([p0, p1])
            parent = str(joint.get("parent", ""))
            child = str(joint.get("child", ""))
            joint_name = str(joint.get("name", ""))
            if self._joint_is_base_block(joint_name, parent, child):
                child_pose = np.asarray(poses[child], dtype=float)
                vertices, faces = self._cuboid_at_pose(
                    child_pose,
                    np.zeros(3, dtype=float),
                    self._base_block_size_for_joint(joint_name, parent, child),
                )
            else:
                if length < 1e-5:
                    continue
                width = self._box_width_for_link(parent, child, length)
                vertices, faces = self._cuboid_between(p0, p1, width)
            color = self._box_color_for_link(parent, child)
            item = self._link_items.get(key)
            if item is None:
                item = gl.GLMeshItem(
                    vertexes=vertices,
                    faces=faces,
                    smooth=False,
                    drawEdges=True,
                    edgeColor=(0.30, 0.38, 0.48, 0.70),
                    color=color,
                )
                try:
                    item.setGLOptions("translucent")
                except Exception:
                    pass
                self._link_items[key] = item
                self._gl.addItem(item)
            else:
                try:
                    item.setMeshData(vertexes=vertices, faces=faces)
                    item.setColor(color)
                except Exception:
                    try:
                        self._gl.removeItem(item)
                    except Exception:
                        pass
                    item = gl.GLMeshItem(
                        vertexes=vertices,
                        faces=faces,
                        smooth=False,
                        drawEdges=True,
                        edgeColor=(0.30, 0.38, 0.48, 0.70),
                        color=color,
                    )
                    try:
                        item.setGLOptions("translucent")
                    except Exception:
                        pass
                    self._link_items[key] = item
                    self._gl.addItem(item)

        for key in list(self._link_items.keys()):
            if key not in used:
                try:
                    self._gl.removeItem(self._link_items[key])
                except Exception:
                    pass
                self._link_items.pop(key, None)

        if all_points:
            self._draw_joint_nodes(np.asarray(joint_points, dtype=float))
            fit_points = []
            for item in all_points:
                arr = np.asarray(item, dtype=float)
                if arr.ndim == 1:
                    fit_points.append(arr.reshape(1, 3))
                elif arr.ndim == 2 and arr.shape[1] == 3:
                    fit_points.append(arr)
            if fit_points:
                self._fit_camera_to_points(np.vstack(fit_points))

    @staticmethod
    def _joint_is_base_block(joint_name: str, parent: str, child: str) -> bool:
        _ = parent, child
        name = str(joint_name).lower()
        return (
            "aa_joint" in name
            or "cmc" in name
        )

    @staticmethod
    def _base_block_size_for_joint(joint_name: str, parent: str, child: str) -> np.ndarray:
        name = f"{joint_name} {parent} {child}".lower()
        if "thumb" in name:
            return np.array([0.020, 0.022, 0.018], dtype=float)
        return np.array([0.020, 0.020, 0.016], dtype=float)

    def _draw_joint_nodes(self, points: np.ndarray) -> None:
        gl = self._gl_mod
        if gl is None or self._gl is None:
            return
        pts = np.asarray(points, dtype=float)
        if pts.ndim != 2 or pts.shape[1] != 3 or pts.size == 0:
            return
        vertices, faces = self._cubes_at_points(pts, 0.0055)
        if vertices.size == 0 or faces.size == 0:
            return
        if self._joint_node_item is None:
            self._joint_node_item = gl.GLMeshItem(
                vertexes=vertices,
                faces=faces,
                smooth=False,
                drawEdges=False,
                color=(0.92, 0.08, 0.08, 1.0),
            )
            self._gl.addItem(self._joint_node_item)
        else:
            self._joint_node_item.setMeshData(vertexes=vertices, faces=faces)
            try:
                self._joint_node_item.setColor((0.92, 0.08, 0.08, 1.0))
            except Exception:
                pass

    def _semantic_revolute_segments(self, poses: dict) -> list[tuple[tuple[str, str], np.ndarray, np.ndarray, dict]]:
        if self._kin is None:
            return []
        joints_by_name = {
            str(joint.get("name", "")): joint
            for joint in self._kin.joints
            if joint.get("type") == "revolute"
        }
        ordered = [joints_by_name[name] for name in SEMANTIC_URDF_REVOLUTE_JOINTS if name in joints_by_name]
        if not ordered:
            ordered = [joint for joint in self._kin.joints if joint.get("type") == "revolute"]

        segments: list[tuple[tuple[str, str], np.ndarray, np.ndarray, dict]] = []
        for joint in ordered:
            parent = joint["parent"]
            child = joint["child"]
            if parent not in poses or child not in poses:
                continue
            p0 = np.asarray(poses[parent][:3, 3], dtype=float)
            p1 = np.asarray(poses[child][:3, 3], dtype=float)
            key = (str(joint.get("name", "")), str(child))
            segments.append((key, p0, p1, joint))
        return segments

    def _semantic_tip_segments(self, poses: dict) -> list[tuple[tuple[str, str], np.ndarray, np.ndarray, dict]]:
        if self._kin is None or not self._semantic_tip_offsets:
            return []
        joints_by_child = {
            str(joint.get("child", "")): joint
            for joint in self._kin.joints
            if joint.get("type") == "revolute"
        }
        segments: list[tuple[tuple[str, str], np.ndarray, np.ndarray, dict]] = []
        for link_name, tip_offset in self._semantic_tip_offsets.items():
            if link_name not in poses:
                continue
            pose = np.asarray(poses[link_name], dtype=float)
            p0 = pose[:3, 3].copy()
            p1 = (pose @ np.array([tip_offset[0], tip_offset[1], tip_offset[2], 1.0], dtype=float))[:3]
            joint = joints_by_child.get(link_name, {"name": f"{link_name}_virtual_tip", "parent": link_name, "child": f"{link_name}_tip"})
            key = (str(joint.get("name", "")), f"{link_name}_virtual_tip")
            segments.append((key, p0, p1, joint))
        return segments

    @staticmethod
    def _load_semantic_tip_offsets(urdf_path: str) -> dict[str, np.ndarray]:
        offsets: dict[str, np.ndarray] = {}
        try:
            root = ET.parse(urdf_path).getroot()
        except Exception:
            return offsets
        for link in root.findall("link"):
            link_name = str(link.attrib.get("name", ""))
            if "fingertip" not in link_name:
                continue
            origin_elem = link.find("inertial/origin")
            if origin_elem is None:
                continue
            xyz = Pose3DView._parse_xyz(origin_elem.attrib.get("xyz", "0 0 0"))
            offsets[link_name] = 2.0 * xyz
        return offsets

    @staticmethod
    def _semantic_revolute_points(
        segments: list[tuple[tuple[str, str], np.ndarray, np.ndarray, dict]],
    ) -> np.ndarray:
        if not segments:
            return np.empty((0, 3), dtype=float)
        points = []
        seen = set()
        for _key, p0, p1, joint in segments:
            for label, point in ((str(joint.get("parent", "")), p0), (str(joint.get("child", "")), p1)):
                if label in seen:
                    continue
                seen.add(label)
                points.append(point)
        return np.asarray(points, dtype=float)

    @staticmethod
    def _cuboid_between(p0: np.ndarray, p1: np.ndarray, width: float) -> tuple[np.ndarray, np.ndarray]:
        # 用 parent joint origin 到 child joint origin 这一段生成一个有厚度的长方体。
        # 这个只是 q_target 可视化，不参与 IK、reference 或发给电机的角度计算。
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
        y_norm = float(np.linalg.norm(y_axis))
        if y_norm < 1e-8:
            y_axis = np.array([0.0, 1.0, 0.0], dtype=float)
        else:
            y_axis /= y_norm
        z_axis = np.cross(x_axis, y_axis)
        z_axis /= max(float(np.linalg.norm(z_axis)), 1e-8)

        center = 0.5 * (p0 + p1)
        hx = 0.5 * length
        hy = 0.5 * float(width)
        hz = 0.5 * float(width)
        vertices = []
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                for sz in (-1.0, 1.0):
                    vertices.append(center + sx * hx * x_axis + sy * hy * y_axis + sz * hz * z_axis)
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

    @staticmethod
    def _cuboid_at_pose(
        pose: np.ndarray,
        local_center: np.ndarray,
        size_xyz: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        pose = np.asarray(pose, dtype=float)
        center = np.asarray(local_center, dtype=float)
        half = 0.5 * np.asarray(size_xyz, dtype=float)
        vertices_local = []
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                for sz in (-1.0, 1.0):
                    vertices_local.append(center + np.array([sx * half[0], sy * half[1], sz * half[2]], dtype=float))
        local = np.asarray(vertices_local, dtype=float)
        vertices = (local @ pose[:3, :3].T) + pose[:3, 3]
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
        return vertices, faces

    @staticmethod
    def _cubes_at_points(points: np.ndarray, size: float) -> tuple[np.ndarray, np.ndarray]:
        pts = np.asarray(points, dtype=float)
        if pts.ndim != 2 or pts.shape[1] != 3 or pts.size == 0:
            return np.empty((0, 3), dtype=float), np.empty((0, 3), dtype=int)
        pts = pts[np.all(np.isfinite(pts), axis=1)]
        if pts.size == 0:
            return np.empty((0, 3), dtype=float), np.empty((0, 3), dtype=int)

        half = 0.5 * float(size)
        offsets = np.array(
            [
                [sx * half, sy * half, sz * half]
                for sx in (-1.0, 1.0)
                for sy in (-1.0, 1.0)
                for sz in (-1.0, 1.0)
            ],
            dtype=float,
        )
        cube_faces = np.array(
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

        vertices = []
        faces = []
        for idx, point in enumerate(pts):
            base = idx * 8
            vertices.append(point.reshape(1, 3) + offsets)
            faces.append(cube_faces + base)
        return np.vstack(vertices), np.vstack(faces)

    @staticmethod
    def _box_width_for_link(parent: str, child: str, length: float) -> float:
        name = f"{parent} {child}".lower()
        if "base" in name or "palm" in name:
            return max(0.014, min(0.026, float(length) * 0.55))
        if "abduction_adduction" in name:
            return max(0.010, min(0.016, float(length) * 0.85))
        if "thumb" in name:
            return max(0.007, min(0.014, float(length) * 0.35))
        return max(0.006, min(0.012, float(length) * 0.32))

    @staticmethod
    def _box_color_for_link(parent: str, child: str) -> tuple[float, float, float, float]:
        name = f"{parent} {child}".lower()
        if "base" in name or "palm" in name:
            return (0.22, 0.48, 0.78, 0.42)
        if "thumb" in name:
            return (0.18, 0.47, 0.78, 0.72)
        if "finger_4" in name or "little" in name:
            return (0.22, 0.50, 0.82, 0.76)
        return (0.66, 0.73, 0.84, 0.82)

    @staticmethod
    def _software_segment_width(joint_name: str, parent: str, child: str, length: float) -> float:
        name = f"{joint_name} {parent} {child}".lower()
        if "thumb" in name:
            return max(0.008, min(0.014, float(length) * 0.34))
        if "aa_joint" in name or "abduction_adduction" in name:
            return max(0.010, min(0.016, float(length) * 0.80))
        return max(0.007, min(0.013, float(length) * 0.32))

    @staticmethod
    def _software_joint_block_size(joint_name: str, parent: str, child: str) -> np.ndarray:
        name = f"{joint_name} {parent} {child}".lower()
        if "thumb" in name:
            return np.array([0.020, 0.022, 0.018], dtype=float)
        return np.array([0.018, 0.018, 0.014], dtype=float)

    def _encoder_degrees_to_urdf_joint_dict(self, angles_deg: List[float]) -> dict:
        # 新版 mmhand URDF 使用语义 joint name，旧版可视化函数使用 1-1/4-2 这类旧 joint name。
        # 这里优先按当前 URDF 的真实 joint name 查 J 编号，保证 Raw IK q_target 输入的是当前 URDF。
        if self._kin is None or _encoder_index_for_joint is None or _encoder_degrees_to_urdf_angle is None:
            return motor_deg_to_urdf_joint_dict(angles_deg)

        deg = list(angles_deg or [])
        while len(deg) < ENCODER_COUNT:
            deg.append(0.0)
        out = {}
        matched = 0
        for joint in self._kin.joints:
            if joint.get("type") != "revolute":
                continue
            joint_name = str(joint.get("name", ""))
            idx = _encoder_index_for_joint(joint_name)
            if idx is None or not 0 <= idx < ENCODER_COUNT:
                continue
            out[joint_name] = _encoder_degrees_to_urdf_angle(idx, float(deg[idx]))
            matched += 1
        return out if matched else motor_deg_to_urdf_joint_dict(deg)

    def _draw_projection(self, poses: dict) -> None:
        # Raw IK 页使用这个 2D 投影作为 OpenGL 的兜底显示。
        # 坐标使用 display transform 后的 x-z 平面：横向为 x，纵向为 z。
        if self._projection_label is None or self._kin is None:
            return
        width = max(240, int(self._projection_label.width() or 640))
        height = max(160, int(self._projection_label.height() or 220))
        pixmap = QtGui.QPixmap(width, height)
        pixmap.fill(QtGui.QColor("white"))

        points = []
        joint_points = []
        seen_points = set()
        segments = []
        for _key, p0, p1, _joint in self._semantic_revolute_segments(poses) + self._semantic_tip_segments(poses):
            if not np.all(np.isfinite(p0)) or not np.all(np.isfinite(p1)):
                continue
            points.extend([p0, p1])
            segments.append((p0, p1))
            for point in (p0, p1):
                key = tuple(round(float(value), 7) for value in point)
                if key in seen_points:
                    continue
                seen_points.add(key)
                joint_points.append(point)

        painter = QtGui.QPainter(pixmap)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        if not points:
            painter.setPen(QtGui.QColor("#777777"))
            painter.drawText(pixmap.rect(), QtCore.Qt.AlignCenter, "No URDF poses to draw")
            painter.end()
            self._projection_label.setPixmap(pixmap)
            return

        arr = np.stack(points, axis=0)
        xy = arr[:, [0, 2]]
        bbox_min = xy.min(axis=0)
        bbox_max = xy.max(axis=0)
        span = np.maximum(bbox_max - bbox_min, 1e-6)
        margin = 16.0
        scale = min((width - 2 * margin) / float(span[0]), (height - 2 * margin) / float(span[1]))
        center = 0.5 * (bbox_min + bbox_max)

        def to_screen(point: np.ndarray) -> QtCore.QPointF:
            v = np.asarray(point, dtype=float)[[0, 2]]
            x = (v[0] - center[0]) * scale + width * 0.5
            y = height * 0.5 - (v[1] - center[1]) * scale
            return QtCore.QPointF(float(x), float(y))

        painter.setPen(QtGui.QPen(QtGui.QColor("#202020"), 2.0))
        for p0, p1 in segments:
            painter.drawLine(to_screen(p0), to_screen(p1))

        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor("#d32f2f"))
        radius = 3.2
        for point in joint_points:
            s = to_screen(point)
            painter.drawEllipse(s, radius, radius)

        painter.setPen(QtGui.QColor("#666666"))
        painter.drawText(8, 16, "URDF q_target 21-point projection")
        painter.end()
        self._projection_label.setPixmap(pixmap)

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
