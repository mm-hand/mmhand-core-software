from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import math
import os
import re
import struct
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np

from data_models import (
    HandModel,
    FingerTactile,
    TactileSensor,
    tactile_band_shape,
    tactile_segment_shape,
    tactile_upsample_tip_grid_to_band0,
    tactile_upsample_pad_grid_to_band,
    TACTILE_BAND_WIDTH,
    TACTILE_FINGER_STRIP_ROWS,
    TACTILE_NBANDS,
    TACTILE_STITCH_SPEC,
    TACTILE_BAND_SENSOR_INDEX,
    TACTILE_FOUR_FINGER_GAP,
    TACTILE_JOINT_BAND_INDICES,
    tactile_apply_fingertip_round_cap,
)
from .ui_qt_compat import QtCore, QtWidgets
import pyqtgraph as pg


TACTILE_REAL_INTERVAL_S = 0.10
TACTILE_FAKE_INTERVAL_S = 0.25
TACTILE_NOISE_FLOOR = 3.0
TACTILE_RISE_ALPHA = 0.55
TACTILE_FALL_ALPHA = 0.45
TACTILE_DISPLAY_FLOOR = 6.0
TACTILE_AXIS_LEVELS = (70.0, 70.0, 180.0)
TACTILE_2D_ARRAY_SCALE = 8
TACTILE_3D_RISE_ALPHA = 0.62
TACTILE_3D_FALL_ALPHA = 0.48
TACTILE_3D_PRESS_LIFT = 0.0
TACTILE_3D_DISPLAY_FLOOR = 2.0
TACTILE_3D_DOT_RADIUS = 0.00155
TACTILE_3D_HALO_RADIUS = 0.0054
TACTILE_3D_SURFACE_OFFSET = 0.003
DESKTOP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF_HAND_PATH = os.path.join(DESKTOP_DIR, "hku_hand_v2_urdf", "urdf", "hand.urdf")
URDF_MAX_TRIANGLES_PER_LINK = 0
_URDF_HAND_MESH_CACHE = None
_URDF_HAND_LINK_VERTEX_CACHE = None


@dataclass
class _TactileRenderJob:
    token: int
    state: Optional[HandModel]
    axis: int


@dataclass
class _TactileRenderResult:
    token: int
    axis: int
    matrix: np.ndarray
    has_data: bool


class _LatestTactileMatrixWorker:
    def __init__(self, build_matrix):
        self._build_matrix = build_matrix
        self._pending: Optional[_TactileRenderJob] = None
        self._latest: Optional[_TactileRenderResult] = None
        self._lock = threading.Lock()
        self._event = threading.Event()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def submit(self, job: _TactileRenderJob) -> None:
        with self._lock:
            self._pending = job
        self._event.set()

    def consume_latest(self) -> Optional[_TactileRenderResult]:
        with self._lock:
            result = self._latest
            self._latest = None
            return result

    def close(self) -> None:
        self._stop.set()
        self._event.set()
        self._thread.join(timeout=0.5)

    def _run(self) -> None:
        while not self._stop.is_set():
            self._event.wait(timeout=0.25)
            self._event.clear()
            if self._stop.is_set():
                return
            with self._lock:
                job = self._pending
                self._pending = None
            if job is None:
                continue
            try:
                mat = self._build_matrix(job.state, job.axis)
            except Exception:
                continue
            has_data = bool(job.state and getattr(job.state, "tactile_data", None))
            with self._lock:
                self._latest = _TactileRenderResult(job.token, job.axis, mat, has_data)


class _TactileHand3DWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._available = False
        self._gl_mod = None
        self._view = None
        self._scatter = None
        self._dot_mesh = None
        self._halo_mesh = None
        self._dot_vertices = np.empty((0, 3), dtype=float)
        self._dot_faces = np.empty((0, 3), dtype=np.uint32)
        self._halo_vertices = np.empty((0, 3), dtype=float)
        self._halo_faces = np.empty((0, 3), dtype=np.uint32)
        self._faces_per_dot = 0
        self._faces_per_halo = 0
        self._base_pos = np.empty((0, 3), dtype=float)
        self._normals = np.empty((0, 3), dtype=float)
        self._finger_index = np.empty((0,), dtype=np.int16)
        self._sensor_index = np.empty((0,), dtype=np.int16)
        self._local_uv = np.empty((0, 2), dtype=float)
        self._display_intensity = np.empty((0,), dtype=float)
        self._hand_mesh_bounds: Optional[Tuple[np.ndarray, np.ndarray]] = None
        self._build_ui()

    def is_available(self) -> bool:
        return self._available

    def _build_ui(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        try:
            import pyqtgraph as pg
            import pyqtgraph.opengl as gl

            self._gl_mod = gl
            self._view = gl.GLViewWidget()
            self._view.setBackgroundColor("w")
            self._view.setMouseTracking(True)
            self._view.setFocusPolicy(QtCore.Qt.StrongFocus)
            self._view.opts["distance"] = 0.42
            self._view.opts["elevation"] = 28
            self._view.opts["azimuth"] = -64
            self._view.opts["center"] = pg.Vector(0.0, 0.065, 0.018)
            layout.addWidget(self._view, 1)

            self._build_geometry()
            self._add_robot_hand_body()
            self._build_dot_mesh()
            self._dot_mesh = gl.GLMeshItem(
                vertexes=self._dot_vertices,
                faces=self._dot_faces,
                faceColors=self._face_colors_from_intensity(np.zeros(self._base_pos.shape[0], dtype=float)),
                smooth=False,
                drawEdges=False,
                shader=None,
                computeNormals=False,
                glOptions="opaque",
            )
            self._view.addItem(self._dot_mesh)
            self._halo_mesh = gl.GLMeshItem(
                vertexes=self._halo_vertices,
                faces=self._halo_faces,
                faceColors=self._halo_face_colors_from_intensity(np.zeros(self._base_pos.shape[0], dtype=float)),
                smooth=False,
                drawEdges=False,
                shader=None,
                computeNormals=False,
                glOptions="translucent",
            )
            self._view.addItem(self._halo_mesh)
            self._fit_camera()
            self._available = True
        except Exception as exc:
            label = QtWidgets.QLabel(f"OpenGL tactile view unavailable:\n{exc}")
            label.setAlignment(QtCore.Qt.AlignCenter)
            layout.addWidget(label, 1)

    def _build_geometry(self) -> None:
        positions: List[np.ndarray] = []
        normals: List[np.ndarray] = []
        finger_indices: List[np.ndarray] = []
        sensor_indices: List[np.ndarray] = []
        local_uvs: List[np.ndarray] = []

        finger_specs = [
            (0, (-0.098, -0.050), -0.78, 0.88),
            (1, (-0.068, -0.038), 0.06, 0.98),
            (2, (-0.022, -0.040), 0.02, 1.08),
            (3, (0.022, -0.040), -0.03, 1.02),
            (4, (0.066, -0.036), -0.10, 0.88),
        ]
        for fi, base_xy, angle, scale in finger_specs:
            for si, offset, length, width, rows, cols, rounded, row_counts in [
                (2, 0.000, 0.042, 0.026, 5, 5, False, None),
                (1, 0.056, 0.042, 0.026, 5, 5, False, None),
                (0, 0.112, 0.047, 0.034, 0, 0, True, (3, 5, 6, 6, 6, 5)),
            ]:
                pos, normal, uv = self._module_points(
                    base_xy=base_xy,
                    angle=angle,
                    offset=offset * scale,
                    length=length * scale,
                    width=width * (0.98 if fi == 4 else 1.0),
                    rows=rows,
                    cols=cols,
                    rounded=rounded,
                    row_counts=row_counts,
                )
                positions.append(pos)
                normals.append(normal)
                finger_indices.append(np.full(pos.shape[0], fi, dtype=np.int16))
                sensor_indices.append(np.full(pos.shape[0], si, dtype=np.int16))
                local_uvs.append(uv)

        self._base_pos = np.vstack(positions).astype(float)
        self._normals = np.vstack(normals).astype(float)
        self._base_pos = self._orient_points_for_upright_view(self._base_pos)
        self._normals = self._orient_points_for_upright_view(self._normals)
        self._finger_index = np.concatenate(finger_indices)
        self._sensor_index = np.concatenate(sensor_indices)
        self._local_uv = np.vstack(local_uvs).astype(float)
        self._display_intensity = np.zeros(self._base_pos.shape[0], dtype=float)

    @staticmethod
    def _orient_points_for_upright_view(points: np.ndarray) -> np.ndarray:
        arr = np.asarray(points, dtype=float).copy()
        if arr.ndim != 2 or arr.shape[1] < 3:
            return arr
        x = arr[:, 0].copy()
        y = arr[:, 1].copy()
        z = arr[:, 2].copy()
        # Build the tactile hand in a conventional OpenGL frame:
        # X = left/right, Y = depth, Z = fingers upward.
        arr[:, 0] = -y
        arr[:, 1] = z
        arr[:, 2] = x
        return arr

    def _build_dot_mesh(self) -> None:
        if self._base_pos.size == 0:
            return
        self._dot_vertices, self._dot_faces, self._faces_per_dot = self._make_disc_mesh(
            TACTILE_3D_DOT_RADIUS,
            10,
        )
        self._halo_vertices, self._halo_faces, self._faces_per_halo = self._make_disc_mesh(
            TACTILE_3D_HALO_RADIUS,
            18,
        )

    def _make_disc_mesh(self, dot_radius: float, segments: int) -> Tuple[np.ndarray, np.ndarray, int]:
        vertices: List[np.ndarray] = []
        faces: List[Tuple[int, int, int]] = []
        up = np.array([0.0, 1.0, 0.0], dtype=float)
        side = np.array([1.0, 0.0, 0.0], dtype=float)
        for p, n in zip(self._base_pos, self._normals):
            t1 = np.cross(up, n)
            if np.linalg.norm(t1) < 1e-8:
                t1 = np.cross(side, n)
            t1 = t1 / max(np.linalg.norm(t1), 1e-9)
            t2 = np.cross(n, t1)
            t2 = t2 / max(np.linalg.norm(t2), 1e-9)
            start = len(vertices)
            vertices.append(p)
            for k in range(segments):
                theta = 2.0 * np.pi * k / segments
                vertices.append(p + (np.cos(theta) * t1 + np.sin(theta) * t2) * dot_radius)
            for k in range(segments):
                faces.append((start, start + 1 + k, start + 1 + ((k + 1) % segments)))
        return np.asarray(vertices, dtype=float), np.asarray(faces, dtype=np.uint32), segments

    def _fit_camera(self) -> None:
        if self._view is None or self._base_pos.size == 0:
            return
        if self._hand_mesh_bounds is not None:
            mesh_min, mesh_max = self._hand_mesh_bounds
            scene_min = np.minimum(np.min(self._base_pos, axis=0), mesh_min)
            scene_max = np.maximum(np.max(self._base_pos, axis=0), mesh_max)
            center = 0.5 * (scene_min + scene_max)
        else:
            center = np.mean(self._base_pos, axis=0)
        try:
            import pyqtgraph as pg

            pos = pg.Vector(float(center[0]), float(center[1]), float(center[2]))
        except Exception:
            pos = (float(center[0]), float(center[1]), float(center[2]))
        self._view.setCameraPosition(pos=pos, distance=0.44, elevation=8, azimuth=90)

    def _add_urdf_hand_mesh(self) -> bool:
        if self._view is None or self._gl_mod is None or self._base_pos.size == 0:
            return False
        try:
            vertices, faces, face_colors = self._load_urdf_hand_mesh()
        except Exception:
            return False
        if vertices.size == 0 or faces.size == 0:
            return False

        gl = self._gl_mod
        aligned = self._align_urdf_mesh_to_tactile(vertices)
        self._hand_mesh_bounds = (np.min(aligned, axis=0), np.max(aligned, axis=0))
        src_min = np.min(vertices, axis=0)
        src_max = np.max(vertices, axis=0)
        try:
            aligned_links = {
                name: self._align_urdf_mesh_with_bounds(link_vertices, src_min, src_max)
                for name, link_vertices in self._load_urdf_link_vertices().items()
                if link_vertices.size
            }
            self._fit_tactile_points_to_urdf_surface(aligned_links)
        except Exception:
            pass
        self._view.addItem(gl.GLMeshItem(
            vertexes=aligned,
            faces=faces,
            faceColors=face_colors,
            smooth=True,
            drawEdges=False,
            shader="shaded",
            glOptions="opaque",
        ))
        return True

    def _align_urdf_mesh_to_tactile(self, vertices: np.ndarray) -> np.ndarray:
        src = np.asarray(vertices, dtype=float)
        src_min = np.min(src, axis=0)
        src_max = np.max(src, axis=0)
        return self._align_urdf_mesh_with_bounds(src, src_min, src_max)

    def _align_urdf_mesh_with_bounds(
        self,
        vertices: np.ndarray,
        src_min: np.ndarray,
        src_max: np.ndarray,
    ) -> np.ndarray:
        src = np.asarray(vertices, dtype=float)
        dst = np.empty_like(src)
        src_min = np.asarray(src_min, dtype=float)
        src_max = np.asarray(src_max, dtype=float)
        src_span = np.maximum(src_max - src_min, 1e-9)

        dot_min = np.min(self._base_pos, axis=0)
        dot_max = np.max(self._base_pos, axis=0)
        dot_span = np.maximum(dot_max - dot_min, 1e-9)
        dot_center = 0.5 * (dot_min + dot_max)

        sx = dot_span[0] / src_span[0] * 1.08
        sz = dot_span[2] / src_span[2] * 1.03
        sy = min(sx, sz) * 0.42

        # URDF display coordinates and the tactile scene both use Z as
        # "fingers upward". Keep the STL slightly behind the tactile dot layer
        # so red contacts remain visible.
        dst[:, 0] = (src[:, 0] - 0.5 * (src_min[0] + src_max[0])) * sx + dot_center[0]
        dst[:, 1] = (src[:, 1] - 0.5 * (src_min[1] + src_max[1])) * sy - 0.030
        dst[:, 2] = dot_min[2] + (src[:, 2] - src_min[2]) * sz + 0.006
        return dst

    def _fit_tactile_points_to_urdf_surface(self, link_vertices: Dict[str, np.ndarray]) -> None:
        if self._base_pos.size == 0 or not link_vertices:
            return

        fitted_pos = self._base_pos.copy()
        fitted_normals = self._normals.copy()
        changed = False
        for finger_index in range(5):
            for sensor_index in range(3):
                link_name = self._tactile_urdf_link_name(finger_index, sensor_index)
                vertices = link_vertices.get(link_name)
                if vertices is None or vertices.shape[0] < 3:
                    continue
                mask = (self._finger_index == finger_index) & (self._sensor_index == sensor_index)
                if not np.any(mask):
                    continue
                pos, normals = self._module_surface_points(vertices, self._local_uv[mask])
                fitted_pos[mask] = pos
                fitted_normals[mask] = normals
                changed = True

        if changed:
            self._base_pos = fitted_pos
            self._normals = fitted_normals

    @staticmethod
    def _tactile_urdf_link_name(finger_index: int, sensor_index: int) -> str:
        prefix_by_finger = (5, 1, 2, 3, 4)
        if finger_index < 0 or finger_index >= len(prefix_by_finger):
            return ""
        prefix = prefix_by_finger[finger_index]
        if prefix == 5:
            segment_by_sensor = {2: 3, 1: 4, 0: 5}
        else:
            segment_by_sensor = {2: 2, 1: 3, 0: 4}
        segment = segment_by_sensor.get(sensor_index)
        return f"{prefix}-{segment}_Link" if segment is not None else ""

    @staticmethod
    def _module_surface_points(vertices: np.ndarray, uv: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        verts = np.asarray(vertices, dtype=float)
        xz = verts[:, [0, 2]]
        center = np.mean(xz, axis=0)
        centered = xz - center
        if centered.shape[0] >= 2:
            _, _, vh = np.linalg.svd(centered, full_matrices=False)
            length_axis = vh[0]
        else:
            length_axis = np.array([0.0, 1.0], dtype=float)
        if length_axis[1] < 0.0:
            length_axis = -length_axis
        length_axis = length_axis / max(np.linalg.norm(length_axis), 1e-9)
        width_axis = np.array([-length_axis[1], length_axis[0]], dtype=float)

        along = xz @ length_axis
        across = xz @ width_axis
        lo_l, hi_l = np.percentile(along, [8.0, 92.0])
        lo_w, hi_w = np.percentile(across, [10.0, 90.0])
        length = max(0.012, float(hi_l - lo_l) * 0.82)
        width = max(0.010, float(hi_w - lo_w) * 0.78)
        center_l = 0.5 * (lo_l + hi_l)
        center_w = 0.5 * (lo_w + hi_w)
        patch_center = length_axis * center_l + width_axis * center_w

        u = np.asarray(uv[:, 0], dtype=float) - 0.5
        v = np.asarray(uv[:, 1], dtype=float) - 0.5
        pos = np.zeros((uv.shape[0], 3), dtype=float)
        pos[:, 0] = patch_center[0] + width_axis[0] * u * width + length_axis[0] * v * length
        pos[:, 2] = patch_center[1] + width_axis[1] * u * width + length_axis[1] * v * length
        pos[:, 1] = float(np.percentile(verts[:, 1], 96.0)) + TACTILE_3D_SURFACE_OFFSET

        normals = np.zeros_like(pos)
        normals[:, 1] = 1.0
        return pos, normals

    @classmethod
    def _load_urdf_hand_mesh(cls) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        global _URDF_HAND_MESH_CACHE, _URDF_HAND_LINK_VERTEX_CACHE
        if _URDF_HAND_MESH_CACHE is not None:
            return _URDF_HAND_MESH_CACHE
        if not os.path.exists(URDF_HAND_PATH):
            raise FileNotFoundError(URDF_HAND_PATH)

        robot = ET.parse(URDF_HAND_PATH).getroot()
        urdf_dir = os.path.dirname(os.path.abspath(URDF_HAND_PATH))
        visuals: Dict[str, Tuple[str, np.ndarray]] = {}
        link_names: List[str] = []
        for link in robot.findall("link"):
            link_name = link.attrib.get("name", "")
            if link_name:
                link_names.append(link_name)
            visual = link.find("visual")
            geometry = visual.find("geometry") if visual is not None else None
            mesh = geometry.find("mesh") if geometry is not None else None
            if link_name and mesh is not None:
                xyz, rpy = cls._parse_urdf_origin(visual)
                visuals[link_name] = (mesh.attrib.get("filename", ""), cls._homogeneous(xyz, rpy))

        children: Dict[str, List[dict]] = {}
        joints: List[dict] = []
        for joint in robot.findall("joint"):
            parent = joint.find("parent")
            child = joint.find("child")
            if parent is None or child is None:
                continue
            axis_elem = joint.find("axis")
            axis = cls._parse_float_vec(axis_elem.attrib.get("xyz", "0 0 1")) if axis_elem is not None else np.array([0.0, 0.0, 1.0])
            xyz, rpy = cls._parse_urdf_origin(joint)
            item = {
                "name": joint.attrib.get("name", ""),
                "type": joint.attrib.get("type", "fixed"),
                "parent": parent.attrib.get("link", ""),
                "child": child.attrib.get("link", ""),
                "origin": cls._homogeneous(xyz, rpy),
                "axis": axis,
            }
            children.setdefault(item["parent"], []).append(item)
            joints.append(item)

        child_links = {item["child"] for item in joints}
        parent_links = {item["parent"] for item in joints}
        roots = sorted(parent_links - child_links)
        root = roots[0] if roots else (link_names[0] if link_names else "base_link")
        poses: Dict[str, np.ndarray] = {root: np.eye(4, dtype=float)}

        def visit(parent_name: str) -> None:
            for joint in children.get(parent_name, []):
                rot = np.eye(4, dtype=float)
                # Tactile view is a stable monitor; use the open zero pose for
                # the body mesh and let the tactile colors carry live feedback.
                if joint["type"] == "revolute":
                    rot[:3, :3] = cls._rodrigues(joint["axis"], 0.0)
                child_name = joint["child"]
                poses[child_name] = poses[parent_name] @ joint["origin"] @ rot
                visit(child_name)

        visit(root)

        display_rot = cls._urdf_display_rotation()
        vertices: List[np.ndarray] = []
        faces: List[np.ndarray] = []
        colors: List[np.ndarray] = []
        link_vertices: Dict[str, np.ndarray] = {}
        for link_name, (filename, visual_tf) in visuals.items():
            mesh_path = cls._resolve_urdf_mesh_path(urdf_dir, filename)
            mesh_vertices, mesh_faces = cls._load_stl(mesh_path)
            if mesh_vertices.size == 0 or mesh_faces.size == 0:
                continue
            mesh_vertices, mesh_faces = cls._decimate_mesh(mesh_vertices, mesh_faces, URDF_MAX_TRIANGLES_PER_LINK)
            transform = poses.get(link_name, np.eye(4, dtype=float)) @ visual_tf
            hom = np.c_[mesh_vertices, np.ones(mesh_vertices.shape[0], dtype=float)]
            transformed = (transform @ hom.T).T[:, :3] @ display_rot.T
            link_vertices[link_name] = transformed.astype(float)

            start = sum(item.shape[0] for item in vertices)
            vertices.append(transformed)
            faces.append(mesh_faces + start)
            colors.append(np.repeat([cls._link_color(link_name)], mesh_faces.shape[0], axis=0))

        if not vertices:
            raise RuntimeError("No URDF hand meshes loaded")
        all_vertices = np.vstack(vertices).astype(float)
        all_faces = np.vstack(faces).astype(np.uint32)
        all_colors = np.vstack(colors).astype(float)
        _URDF_HAND_MESH_CACHE = (all_vertices, all_faces, all_colors)
        _URDF_HAND_LINK_VERTEX_CACHE = link_vertices
        return _URDF_HAND_MESH_CACHE

    @classmethod
    def _load_urdf_link_vertices(cls) -> Dict[str, np.ndarray]:
        global _URDF_HAND_LINK_VERTEX_CACHE
        if _URDF_HAND_LINK_VERTEX_CACHE is None:
            cls._load_urdf_hand_mesh()
        return _URDF_HAND_LINK_VERTEX_CACHE or {}

    @staticmethod
    def _parse_float_vec(text: str) -> np.ndarray:
        parts = [float(item) for item in str(text).replace(",", " ").split() if item]
        while len(parts) < 3:
            parts.append(0.0)
        return np.asarray(parts[:3], dtype=float)

    @classmethod
    def _parse_urdf_origin(cls, elem) -> Tuple[np.ndarray, np.ndarray]:
        origin = elem.find("origin") if elem is not None else None
        if origin is None:
            return np.zeros(3, dtype=float), np.zeros(3, dtype=float)
        return (
            cls._parse_float_vec(origin.attrib.get("xyz", "0 0 0")),
            cls._parse_float_vec(origin.attrib.get("rpy", "0 0 0")),
        )

    @classmethod
    def _homogeneous(cls, xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
        matrix = np.eye(4, dtype=float)
        matrix[:3, :3] = cls._rpy_to_rotation(rpy)
        matrix[:3, 3] = np.asarray(xyz, dtype=float)
        return matrix

    @staticmethod
    def _rpy_to_rotation(rpy: np.ndarray) -> np.ndarray:
        roll, pitch, yaw = [float(v) for v in np.asarray(rpy, dtype=float)[:3]]
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=float)
        ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=float)
        rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=float)
        return rz @ ry @ rx

    @staticmethod
    def _rodrigues(axis: np.ndarray, angle: float) -> np.ndarray:
        vec = np.asarray(axis, dtype=float)
        norm = np.linalg.norm(vec)
        if norm < 1e-12:
            return np.eye(3, dtype=float)
        x, y, z = vec / norm
        c, s = np.cos(float(angle)), np.sin(float(angle))
        k = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=float)
        return np.eye(3, dtype=float) + s * k + (1.0 - c) * (k @ k)

    @staticmethod
    def _urdf_display_rotation() -> np.ndarray:
        rx_angle = -np.pi / 2.0
        cx, sx = np.cos(rx_angle), np.sin(rx_angle)
        rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]], dtype=float)
        ry_angle = -np.pi / 2.0
        cy, sy = np.cos(ry_angle), np.sin(ry_angle)
        ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]], dtype=float)
        rz = np.array([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]], dtype=float)
        return rz @ ry @ rx

    @staticmethod
    def _resolve_urdf_mesh_path(urdf_dir: str, filename: str) -> str:
        path = os.path.normpath(os.path.join(urdf_dir, filename))
        if os.path.exists(path):
            return path
        convex_path = path + ".convex.stl"
        if os.path.exists(convex_path):
            return convex_path
        return path

    @staticmethod
    def _load_stl(path: str) -> Tuple[np.ndarray, np.ndarray]:
        data = open(path, "rb").read()
        vertices: List[Tuple[float, float, float]] = []
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
    def _decimate_mesh(vertices: np.ndarray, faces: np.ndarray, max_triangles: int) -> Tuple[np.ndarray, np.ndarray]:
        if max_triangles <= 0 or faces.shape[0] <= max_triangles:
            return vertices, faces
        indices = np.linspace(0, faces.shape[0] - 1, max_triangles, dtype=np.int64)
        selected = faces[indices].reshape(-1)
        reduced_vertices = vertices[selected]
        reduced_faces = np.arange(reduced_vertices.shape[0], dtype=np.uint32).reshape((-1, 3))
        return reduced_vertices, reduced_faces

    @staticmethod
    def _link_color(link_name: str) -> Tuple[float, float, float, float]:
        if link_name == "base_link":
            return (0.22, 0.23, 0.24, 1.0)
        if re.match(r"^[1-5]-1_Link$", link_name):
            return (0.08, 0.085, 0.09, 1.0)
        return (0.42, 0.44, 0.42, 1.0)

    def _add_robot_hand_body(self) -> None:
        if self._add_urdf_hand_mesh():
            return
        if self._view is None or self._gl_mod is None or self._base_pos.size == 0:
            return
        gl = self._gl_mod
        metal = (0.70, 0.72, 0.70, 1.0)
        dark = (0.06, 0.065, 0.07, 1.0)
        edge = (0.0, 0.0, 0.0, 0.22)

        body_v: List[np.ndarray] = []
        body_f: List[Tuple[int, int, int]] = []
        body_c: List[Tuple[float, float, float, float]] = []
        joint_v: List[np.ndarray] = []
        joint_f: List[Tuple[int, int, int]] = []
        joint_c: List[Tuple[float, float, float, float]] = []

        z_back = float(np.min(self._base_pos[:, 2]) - 0.006)
        z_body = z_back - 0.003
        palm_center = np.array([0.006, 0.0, z_body], dtype=float)
        self._append_box_mesh(
            body_v,
            body_f,
            body_c,
            palm_center,
            np.array([0.002, 1.0, 0.0], dtype=float),
            np.array([1.0, -0.002, 0.0], dtype=float),
            0.112,
            0.126,
            0.014,
            metal,
        )
        self._append_box_mesh(
            body_v,
            body_f,
            body_c,
            palm_center + np.array([0.0, -0.086, -0.018], dtype=float),
            np.array([0.0, 1.0, 0.0], dtype=float),
            np.array([1.0, 0.0, 0.0], dtype=float),
            0.036,
            0.106,
            0.032,
            (0.22, 0.23, 0.23, 1.0),
        )

        for fi in range(5):
            finger_mask = self._finger_index == fi
            if not np.any(finger_mask):
                continue
            finger_pts = self._base_pos[finger_mask]
            finger_xy = finger_pts[:, :2]
            direction = self._principal_axis(finger_xy)
            across = np.array([-direction[1], direction[0], 0.0], dtype=float)
            direction3 = np.array([direction[0], direction[1], 0.0], dtype=float)
            sensor_centers: List[np.ndarray] = []
            sensor_widths: List[float] = []
            for si in range(3):
                mask = finger_mask & (self._sensor_index == si)
                if not np.any(mask):
                    continue
                pts = self._base_pos[mask]
                center = np.mean(pts, axis=0)
                rel = pts[:, :2] - center[:2]
                length = max(0.018, float(np.ptp(rel @ direction)) + 0.018)
                width = max(0.016, float(np.ptp(rel @ across[:2])) + 0.012)
                sensor_centers.append(center)
                sensor_widths.append(width)
                self._append_box_mesh(
                    body_v,
                    body_f,
                    body_c,
                    np.array([center[0], center[1], z_body + 0.004], dtype=float),
                    direction3,
                    across,
                    length,
                    width,
                    0.010,
                    metal,
                )
                self._append_box_mesh(
                    body_v,
                    body_f,
                    body_c,
                    np.array([center[0], center[1], z_body + 0.010], dtype=float),
                    direction3,
                    across,
                    length * 0.76,
                    width * 0.64,
                    0.003,
                    (0.18, 0.19, 0.20, 1.0),
                )

            if sensor_centers:
                ordered_centers = sorted(sensor_centers, key=lambda p: float(p[:2] @ direction))
                boundary_centers: List[np.ndarray] = []
                first = ordered_centers[0]
                last = ordered_centers[-1]
                boundary_centers.append(first - direction3 * 0.026)
                for a, b in zip(ordered_centers[:-1], ordered_centers[1:]):
                    boundary_centers.append((a + b) * 0.5)
                boundary_centers.append(last + direction3 * 0.026)
            else:
                boundary_centers = []
            joint_width = max(sensor_widths or [0.018]) * 0.82
            for center in boundary_centers:
                self._append_cylinder_mesh(
                    joint_v,
                    joint_f,
                    joint_c,
                    np.array([center[0], center[1], z_body + 0.014], dtype=float),
                    across,
                    radius=0.0048 if fi else 0.0042,
                    length=joint_width,
                    color=dark,
                )

        if body_v:
            self._view.addItem(gl.GLMeshItem(
                vertexes=np.asarray(body_v, dtype=float),
                faces=np.asarray(body_f, dtype=np.uint32),
                faceColors=np.asarray(body_c, dtype=float),
                smooth=False,
                drawEdges=True,
                edgeColor=edge,
                shader="shaded",
                glOptions="opaque",
            ))
        if joint_v:
            self._view.addItem(gl.GLMeshItem(
                vertexes=np.asarray(joint_v, dtype=float),
                faces=np.asarray(joint_f, dtype=np.uint32),
                faceColors=np.asarray(joint_c, dtype=float),
                smooth=False,
                drawEdges=False,
                shader="shaded",
                glOptions="opaque",
            ))

    @staticmethod
    def _principal_axis(xy: np.ndarray) -> np.ndarray:
        arr = np.asarray(xy, dtype=float)
        if arr.ndim != 2 or arr.shape[0] < 2:
            return np.array([0.0, 1.0], dtype=float)
        centered = arr - np.mean(arr, axis=0)
        _, _, vh = np.linalg.svd(centered, full_matrices=False)
        axis = vh[0]
        if axis[1] < 0.0:
            axis = -axis
        norm = np.linalg.norm(axis)
        if norm < 1e-9:
            return np.array([0.0, 1.0], dtype=float)
        return axis / norm

    @staticmethod
    def _append_box_mesh(
        vertices: List[np.ndarray],
        faces: List[Tuple[int, int, int]],
        colors: List[Tuple[float, float, float, float]],
        center: np.ndarray,
        length_axis: np.ndarray,
        width_axis: np.ndarray,
        length: float,
        width: float,
        height: float,
        color: Tuple[float, float, float, float],
    ) -> None:
        la = np.asarray(length_axis, dtype=float)
        wa = np.asarray(width_axis, dtype=float)
        la = la / max(np.linalg.norm(la), 1e-9)
        wa = wa - la * float(np.dot(wa, la))
        wa = wa / max(np.linalg.norm(wa), 1e-9)
        ha = np.array([0.0, 0.0, 1.0], dtype=float)
        c = np.asarray(center, dtype=float)
        lx = la * (length * 0.5)
        wx = wa * (width * 0.5)
        hx = ha * (height * 0.5)
        start = len(vertices)
        vertices.extend([
            c - lx - wx - hx,
            c + lx - wx - hx,
            c + lx + wx - hx,
            c - lx + wx - hx,
            c - lx - wx + hx,
            c + lx - wx + hx,
            c + lx + wx + hx,
            c - lx + wx + hx,
        ])
        box_faces = [
            (0, 1, 2), (0, 2, 3),
            (4, 6, 5), (4, 7, 6),
            (0, 4, 5), (0, 5, 1),
            (1, 5, 6), (1, 6, 2),
            (2, 6, 7), (2, 7, 3),
            (3, 7, 4), (3, 4, 0),
        ]
        faces.extend(tuple(start + i for i in face) for face in box_faces)
        colors.extend([color] * len(box_faces))

    @staticmethod
    def _append_cylinder_mesh(
        vertices: List[np.ndarray],
        faces: List[Tuple[int, int, int]],
        colors: List[Tuple[float, float, float, float]],
        center: np.ndarray,
        axis: np.ndarray,
        *,
        radius: float,
        length: float,
        color: Tuple[float, float, float, float],
    ) -> None:
        ax = np.asarray(axis, dtype=float)
        ax = ax / max(np.linalg.norm(ax), 1e-9)
        up = np.array([0.0, 0.0, 1.0], dtype=float)
        side = np.cross(ax, up)
        if np.linalg.norm(side) < 1e-8:
            side = np.cross(ax, np.array([1.0, 0.0, 0.0], dtype=float))
        side = side / max(np.linalg.norm(side), 1e-9)
        normal = np.cross(ax, side)
        normal = normal / max(np.linalg.norm(normal), 1e-9)
        segments = 18
        c = np.asarray(center, dtype=float)
        start = len(vertices)
        left_center = c - ax * (length * 0.5)
        right_center = c + ax * (length * 0.5)
        vertices.append(left_center)
        vertices.append(right_center)
        for k in range(segments):
            theta = 2.0 * np.pi * k / segments
            ring = np.cos(theta) * side + np.sin(theta) * normal
            vertices.append(left_center + ring * radius)
            vertices.append(right_center + ring * radius)
        for k in range(segments):
            l0 = start + 2 + 2 * k
            r0 = l0 + 1
            l1 = start + 2 + 2 * ((k + 1) % segments)
            r1 = l1 + 1
            faces.extend([
                (l0, r0, r1),
                (l0, r1, l1),
                (start, l1, l0),
                (start + 1, r0, r1),
            ])
            colors.extend([color, color, color, color])

    @staticmethod
    def _module_points(
        *,
        base_xy: Tuple[float, float],
        angle: float,
        offset: float,
        length: float,
        width: float,
        rows: int,
        cols: int,
        rounded: bool,
        row_counts: Optional[Tuple[int, ...]] = None,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        direction = np.array([np.sin(angle), np.cos(angle), 0.0], dtype=float)
        across = np.array([-direction[1], direction[0], 0.0], dtype=float)
        base = np.array([base_xy[0], base_xy[1], 0.0], dtype=float) + direction * offset
        curvature = 0.78
        radius = width / (2.0 * np.sin(curvature))

        pos: List[np.ndarray] = []
        normals: List[np.ndarray] = []
        uv_list: List[Tuple[float, float]] = []
        row_layout = tuple(row_counts or ())
        if row_layout:
            row_iter = []
            for r, count in enumerate(row_layout):
                v = r / max(1, len(row_layout) - 1)
                cap_scale = 0.62 + 0.38 * np.sin(np.pi * min(max(v, 0.0), 1.0))
                u_values = np.linspace(-cap_scale, cap_scale, int(count))
                row_iter.extend((v, (u + 1.0) * 0.5, u) for u in u_values)
        else:
            row_iter = []
            for r in range(rows):
                v = r / max(1, rows - 1)
                for c in range(cols):
                    u01 = c / max(1, cols - 1)
                    row_iter.append((v, u01, u01 * 2.0 - 1.0))

        for v, u01, u in row_iter:
            if rounded and not row_layout and v > 0.72:
                cap_v = (v - 0.72) / 0.28
                if abs(u) > np.sqrt(max(0.0, 1.0 - cap_v * cap_v)):
                    continue
            theta = u * curvature
            x = radius * np.sin(theta)
            z = radius * (np.cos(theta) - np.cos(curvature))
            y = v * length
            p = base + across * x + direction * y + np.array([0.0, 0.0, z], dtype=float)
            n = across * (-np.sin(theta)) + np.array([0.0, 0.0, np.cos(theta)], dtype=float)
            n_norm = np.linalg.norm(n)
            if n_norm > 1e-9:
                n = n / n_norm
            pos.append(p)
            normals.append(n)
            uv_list.append((u01, v))
        return np.asarray(pos, dtype=float), np.asarray(normals, dtype=float), np.asarray(uv_list, dtype=float)

    def _add_hand_guides(self) -> None:
        if self._view is None or self._gl_mod is None:
            return
        gl = self._gl_mod
        guide_color = (0.0, 0.0, 0.0, 0.18)
        palm = np.array([
            [-0.078, -0.062, -0.007],
            [0.078, -0.060, -0.007],
            [0.069, 0.028, -0.007],
            [0.045, 0.054, -0.007],
            [-0.055, 0.052, -0.007],
            [-0.088, 0.014, -0.007],
            [-0.078, -0.062, -0.007],
        ], dtype=float)
        palm = self._orient_points_for_upright_view(palm)
        self._view.addItem(gl.GLLinePlotItem(pos=palm, color=guide_color, width=1.4, antialias=True))
        for fi in range(5):
            mask = self._finger_index == fi
            if not np.any(mask):
                continue
            pts = self._base_pos[mask]
            low = pts[np.argmin(pts[:, 1])]
            high = pts[np.argmax(pts[:, 1])]
            line = np.vstack([low, high])
            line[:, 2] = -0.006
            self._view.addItem(gl.GLLinePlotItem(pos=line, color=guide_color, width=1.0, antialias=True))

    def clear(self) -> None:
        if self._dot_mesh is None:
            return
        self._display_intensity = np.zeros(self._base_pos.shape[0], dtype=float)
        self._dot_mesh.setMeshData(
            vertexes=self._dot_vertices,
            faces=self._dot_faces,
            faceColors=self._face_colors_from_intensity(self._display_intensity),
        )
        if self._halo_mesh is not None:
            self._halo_mesh.setMeshData(
                vertexes=self._halo_vertices,
                faces=self._halo_faces,
                faceColors=self._halo_face_colors_from_intensity(self._display_intensity),
            )

    def update_state(self, state: Optional[HandModel], axis: int, has_data: bool) -> None:
        if self._dot_mesh is None or self._base_pos.size == 0:
            return
        raw = np.zeros(self._base_pos.shape[0], dtype=float)
        if has_data and state is not None and getattr(state, "tactile_data", None):
            tactile = state.tactile_data
            for fi in range(5):
                if fi >= len(tactile):
                    continue
                finger = tactile[fi]
                for si in range(3):
                    if si >= len(finger.sensors):
                        continue
                    sensor = finger.sensors[si]
                    resultant = getattr(sensor, "resultant", None)
                    point_mask = (self._finger_index == fi) & (self._sensor_index == si)
                    if not np.any(point_mask):
                        continue
                    uv = self._local_uv[point_mask]
                    values = self._sample_contact_intensity(getattr(sensor, "contact_forces", None), axis, uv)
                    if (
                        values.size == 0
                        or float(np.max(values)) < TACTILE_3D_DISPLAY_FLOOR
                    ) and resultant is not None and len(resultant) >= 3:
                        force = np.asarray(resultant[:3], dtype=float)
                        values = self._sensor_point_intensity(force, axis, uv)
                    raw[point_mask] = values

        if self._display_intensity.shape != raw.shape:
            self._display_intensity = raw
        else:
            rise = raw >= self._display_intensity
            fall = ~rise
            self._display_intensity[rise] += TACTILE_3D_RISE_ALPHA * (raw[rise] - self._display_intensity[rise])
            self._display_intensity[fall] += TACTILE_3D_FALL_ALPHA * (raw[fall] - self._display_intensity[fall])
        self._display_intensity[self._display_intensity < TACTILE_3D_DISPLAY_FLOOR] = 0.0

        vmax = TACTILE_AXIS_LEVELS[min(max(int(axis), 0), len(TACTILE_AXIS_LEVELS) - 1)]
        norm = np.clip(self._display_intensity / max(vmax, 1.0), 0.0, 1.0)
        _ = norm
        self._dot_mesh.setMeshData(
            vertexes=self._dot_vertices,
            faces=self._dot_faces,
            faceColors=self._face_colors_from_intensity(self._display_intensity, axis),
        )
        if self._halo_mesh is not None:
            self._halo_mesh.setMeshData(
                vertexes=self._halo_vertices,
                faces=self._halo_faces,
                faceColors=self._halo_face_colors_from_intensity(self._display_intensity, axis),
            )

    @staticmethod
    def _sample_contact_intensity(contact_forces, axis: int, uv: np.ndarray) -> np.ndarray:
        if contact_forces is None:
            return np.zeros(uv.shape[0], dtype=float)
        arr = np.asarray(contact_forces, dtype=float)
        if arr.ndim < 3 or arr.shape[0] <= 0 or arr.shape[1] <= 0:
            return np.zeros(uv.shape[0], dtype=float)

        axis_index = min(max(int(axis), 0), arr.shape[2] - 1)
        if axis_index == 2:
            field = np.clip(arr[:, :, axis_index], 0.0, None)
        else:
            field = np.abs(arr[:, :, axis_index])

        # If the selected axis is quiet but the module clearly has contact
        # force, use vector magnitude so the 3D hand still shows the press.
        if float(np.nanmax(field)) < TACTILE_3D_DISPLAY_FLOOR:
            z = np.clip(arr[:, :, min(2, arr.shape[2] - 1)], 0.0, None)
            if arr.shape[2] >= 2:
                field = np.sqrt(arr[:, :, 0] * arr[:, :, 0] + arr[:, :, 1] * arr[:, :, 1] + z * z)
            else:
                field = z

        field = _TactileHand3DWidget._spread_contact_field(field)

        rows, cols = field.shape
        if rows == 1 and cols == 1:
            return np.full(uv.shape[0], float(field[0, 0]), dtype=float)
        u = np.clip(np.asarray(uv[:, 0], dtype=float), 0.0, 1.0) * max(0, cols - 1)
        v = np.clip(np.asarray(uv[:, 1], dtype=float), 0.0, 1.0) * max(0, rows - 1)
        x0 = np.floor(u).astype(int)
        y0 = np.floor(v).astype(int)
        x1 = np.clip(x0 + 1, 0, cols - 1)
        y1 = np.clip(y0 + 1, 0, rows - 1)
        wx = u - x0
        wy = v - y0
        top = field[y0, x0] * (1.0 - wx) + field[y0, x1] * wx
        bottom = field[y1, x0] * (1.0 - wx) + field[y1, x1] * wx
        return np.asarray(top * (1.0 - wy) + bottom * wy, dtype=float)

    @staticmethod
    def _spread_contact_field(field: np.ndarray) -> np.ndarray:
        src = np.asarray(field, dtype=float)
        if src.size == 0 or float(np.nanmax(src)) < TACTILE_3D_DISPLAY_FLOOR:
            return src
        padded = np.pad(src, 1, mode="edge")
        center = padded[1:-1, 1:-1]
        cross = (
            padded[:-2, 1:-1]
            + padded[2:, 1:-1]
            + padded[1:-1, :-2]
            + padded[1:-1, 2:]
        )
        diagonal = (
            padded[:-2, :-2]
            + padded[:-2, 2:]
            + padded[2:, :-2]
            + padded[2:, 2:]
        )
        spread = center * 0.46 + cross * 0.13 + diagonal * 0.035
        return np.maximum(src, spread)

    @staticmethod
    def _sensor_point_intensity(force: np.ndarray, axis: int, uv: np.ndarray) -> np.ndarray:
        axis_index = min(max(int(axis), 0), 2)
        if axis_index == 2:
            magnitude = max(0.0, float(force[2]))
        else:
            magnitude = abs(float(force[axis_index]))
        if magnitude < TACTILE_3D_DISPLAY_FLOOR:
            magnitude = max(abs(float(force[0])), abs(float(force[1])), max(0.0, float(force[2])))
        if magnitude < TACTILE_3D_DISPLAY_FLOOR:
            return np.zeros(uv.shape[0], dtype=float)

        fz = max(0.0, float(force[2]))
        scale = max(28.0, min(110.0, max(magnitude, fz)))
        cx = 0.5 + np.clip(float(force[0]) / scale, -0.42, 0.42) * 0.36
        cy = 0.55 - np.clip(float(force[1]) / scale, -0.42, 0.42) * 0.30
        pressure_norm = min(1.0, max(magnitude, fz) / 180.0)
        sigma_x = 0.17 + 0.06 * pressure_norm
        sigma_y = 0.20 + 0.07 * pressure_norm
        du = uv[:, 0] - cx
        dv = uv[:, 1] - cy
        weight = np.exp(-((du * du) / (2.0 * sigma_x * sigma_x) + (dv * dv) / (2.0 * sigma_y * sigma_y)))
        return magnitude * weight

    @staticmethod
    def _colors_from_intensity(intensity: np.ndarray, axis: int = 2) -> np.ndarray:
        values = np.asarray(intensity, dtype=float)
        vmax = TACTILE_AXIS_LEVELS[min(max(int(axis), 0), len(TACTILE_AXIS_LEVELS) - 1)]
        norm = np.clip(values / max(vmax, 1.0), 0.0, 1.0)
        colors = np.zeros((values.shape[0], 4), dtype=float)
        colors[:, 3] = 1.0
        active = norm > 0.0
        colors[active, 0] = 1.0
        colors[active, 1] = 0.015 * (1.0 - norm[active])
        colors[active, 2] = 0.01 * (1.0 - norm[active])
        return colors

    def _face_colors_from_intensity(self, intensity: np.ndarray, axis: int = 2) -> np.ndarray:
        point_colors = self._colors_from_intensity(intensity, axis)
        return np.repeat(point_colors, max(1, self._faces_per_dot), axis=0)

    def _halo_face_colors_from_intensity(self, intensity: np.ndarray, axis: int = 2) -> np.ndarray:
        values = np.asarray(intensity, dtype=float)
        vmax = TACTILE_AXIS_LEVELS[min(max(int(axis), 0), len(TACTILE_AXIS_LEVELS) - 1)]
        norm = np.clip(values / max(vmax, 1.0), 0.0, 1.0)
        colors = np.zeros((values.shape[0], 4), dtype=float)
        active = norm > 0.0
        colors[active, 0] = 1.0
        colors[active, 1] = 0.02
        colors[active, 2] = 0.0
        colors[active, 3] = 0.35 + 0.65 * norm[active]
        return np.repeat(colors, max(1, self._faces_per_halo), axis=0)

    def close(self) -> None:
        pass


class TactileView(QtWidgets.QWidget):
    def __init__(self, context, parent=None):
        super().__init__(parent)
        self.context = context
        self.axis = 2
        self.fake_enabled = False
        self._last_update_ts = 0.0
        self._token = 0
        self._last_applied_token = 0
        self._fake_state: Optional[HandModel] = None
        self._display_matrix: Optional[np.ndarray] = None
        self._last_matrix_shape = None
        self._worker = _LatestTactileMatrixWorker(self._build_matrix)
        self.hand_3d: Optional[_TactileHand3DWidget] = None
        self.use_3d = False
        self._build_ui()

    def _build_ui(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        top = QtWidgets.QHBoxLayout()
        top.addWidget(QtWidgets.QLabel("Axis:"))
        self.axis_group = QtWidgets.QButtonGroup(self)
        for idx, label in enumerate(["X", "Y", "Z"]):
            btn = QtWidgets.QRadioButton(label)
            btn.setChecked(idx == self.axis)
            self.axis_group.addButton(btn, idx)
            top.addWidget(btn)
        self.fake_check = QtWidgets.QCheckBox("Fake")
        self.fake_check.setChecked(False)
        top.addWidget(self.fake_check)
        self.view3d_check = QtWidgets.QCheckBox("3D")
        self.view3d_check.setChecked(False)
        top.addWidget(self.view3d_check)
        top.addStretch(1)
        layout.addLayout(top)
        self.axis_group.idClicked.connect(self._set_axis)
        self.fake_check.toggled.connect(self._set_fake)
        self.view3d_check.toggled.connect(self._set_view3d)

        self.plot = pg.GraphicsLayoutWidget()
        self.plot.setBackground("w")
        self.view = self.plot.addViewBox(lockAspect=True)
        self.view.invertY(True)
        self.image = pg.ImageItem()
        try:
            self.image.setAutoDownsample(False)
        except Exception:
            pass
        self.view.addItem(self.image)
        self.no_data = QtWidgets.QLabel("No tactile data")
        self.no_data.setAlignment(QtCore.Qt.AlignCenter)
        self.no_data.setStyleSheet("color: white; background: rgba(0, 0, 0, 120); padding: 4px;")
        stack = QtWidgets.QStackedLayout()
        holder = QtWidgets.QWidget()
        holder_layout = QtWidgets.QVBoxLayout(holder)
        holder_layout.setContentsMargins(0, 0, 0, 0)
        holder_layout.addWidget(self.plot)
        overlay = QtWidgets.QWidget()
        overlay_layout = QtWidgets.QVBoxLayout(overlay)
        overlay_layout.addStretch(1)
        overlay_layout.addWidget(self.no_data, alignment=QtCore.Qt.AlignCenter)
        overlay_layout.addStretch(1)
        stack.addWidget(holder)
        stack.addWidget(overlay)
        stack.setStackingMode(QtWidgets.QStackedLayout.StackAll)
        container = QtWidgets.QWidget()
        container.setLayout(stack)
        self.flat_container = container
        self.display_stack = QtWidgets.QStackedWidget()
        self.display_stack.addWidget(container)
        self.display_stack.setCurrentWidget(container)
        layout.addWidget(self.display_stack, 1)
        lut = np.zeros((256, 4), dtype=np.ubyte)
        lut[0] = (255, 255, 255, 255)  # outside the hand/sensor area
        lut[1] = (0, 0, 0, 255)        # valid sensor area with no pressure
        lut[2:, 0] = np.linspace(55, 255, 254).astype(np.ubyte)
        lut[2:, 1] = 0
        lut[2:, 2] = 0
        lut[2:, 3] = 255
        self.image.setLookupTable(lut)
        self.values_label = QtWidgets.QLabel("")
        self.values_label.setStyleSheet("color: #334155; font-family: Consolas, 'Courier New', monospace;")
        layout.addWidget(self.values_label)
        self._apply_matrix(self._build_matrix(None, self.axis), self.axis, False)

    def build(self, _parent) -> None:
        pass

    def update(self, state: Optional[HandModel]) -> None:
        self.update_if_due(state, force=True)

    def update_if_due(self, state: Optional[HandModel], *, force: bool = False) -> bool:
        self._apply_ready_frame()
        use_fake = self._should_use_fake(state)
        interval = TACTILE_FAKE_INTERVAL_S if use_fake else TACTILE_REAL_INTERVAL_S
        now = time.time()
        if not force and now - self._last_update_ts < interval:
            return False
        self._last_update_ts = now
        display_state = self._fake_display_state(state) if use_fake else state
        self.values_label.setText(self._format_values(display_state))
        has_data = bool(display_state and getattr(display_state, "tactile_data", None))
        if self.use_3d and self.hand_3d is not None:
            self.hand_3d.update_state(display_state, self.axis, has_data)
        if force:
            mat = self._build_matrix(display_state, self.axis)
            self._apply_matrix(mat, self.axis, has_data)
            return True
        self._token += 1
        self._worker.submit(_TactileRenderJob(self._token, display_state, self.axis))
        return True

    def close(self) -> None:
        self._worker.close()
        if self.hand_3d is not None:
            self.hand_3d.close()

    def clear(self) -> None:
        self._display_matrix = None
        if self.hand_3d is not None:
            self.hand_3d.clear()
        self._apply_matrix(self._build_matrix(None, self.axis), self.axis, False)

    def _set_axis(self, axis: int) -> None:
        self.axis = int(axis)
        self._display_matrix = None
        self.update_if_due(self.context.get_state(), force=True)

    def _set_fake(self, checked: bool) -> None:
        self.fake_enabled = bool(checked)
        self._fake_state = None
        self._display_matrix = None
        self.update_if_due(self.context.get_state(), force=True)

    def _set_view3d(self, checked: bool) -> None:
        if not checked:
            self.use_3d = False
            if self.display_stack is not None:
                self.display_stack.setCurrentWidget(self.flat_container)
            self.update_if_due(self.context.get_state(), force=True)
            return

        if self.hand_3d is None:
            self.view3d_check.setEnabled(False)
            QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
            try:
                self.hand_3d = _TactileHand3DWidget()
                self.display_stack.addWidget(self.hand_3d)
            finally:
                QtWidgets.QApplication.restoreOverrideCursor()
                self.view3d_check.setEnabled(True)

        self.use_3d = self.hand_3d is not None and self.hand_3d.is_available()
        if not self.use_3d:
            self.view3d_check.blockSignals(True)
            self.view3d_check.setChecked(False)
            self.view3d_check.blockSignals(False)
            self.display_stack.setCurrentWidget(self.flat_container)
            return

        self.display_stack.setCurrentWidget(self.hand_3d)
        self.update_if_due(self.context.get_state(), force=True)

    @staticmethod
    def _format_values(state: Optional[HandModel]) -> str:
        tactile = state.tactile_data if state and state.tactile_data else []
        chunks = []
        for fi in range(5):
            vals = []
            finger = tactile[fi] if fi < len(tactile) else FingerTactile(sensors=[])
            for si in range(3):
                if si < len(finger.sensors):
                    resultant = getattr(finger.sensors[si], "resultant", None)
                    value = float(resultant[2]) if resultant is not None and len(resultant) >= 3 else 0.0
                else:
                    value = 0.0
                if 0.0 < value < TACTILE_DISPLAY_FLOOR:
                    value = 0.0
                vals.append(f"{int(round(max(0.0, value))):03d}")
            chunks.append(f"G{fi}:{','.join(vals)}")
        return "  ".join(chunks)

    def _should_use_fake(self, state: Optional[HandModel]) -> bool:
        if not self.fake_enabled:
            return False
        if self.context.is_comm_connected() and state is not None:
            return False
        return True

    def _apply_ready_frame(self) -> None:
        result = self._worker.consume_latest()
        if result is None or result.axis != self.axis or result.token <= self._last_applied_token:
            return
        self._last_applied_token = result.token
        self._apply_matrix(result.matrix, result.axis, result.has_data)

    def _apply_matrix(self, mat: np.ndarray, axis: int, has_data: bool) -> None:
        data = np.asarray(mat, dtype=float)
        finite_mask = np.isfinite(data) & (data >= 0.0)
        filtered = data.copy()
        low_mask = finite_mask & (np.abs(filtered) < TACTILE_NOISE_FLOOR)
        filtered[low_mask] = 0.0

        if self._display_matrix is None or self._display_matrix.shape != filtered.shape:
            smoothed = filtered
        else:
            prev = self._display_matrix
            smoothed = filtered.copy()
            blend_mask = finite_mask & np.isfinite(prev)
            rise_mask = blend_mask & (filtered >= prev)
            fall_mask = blend_mask & ~rise_mask
            smoothed[rise_mask] = prev[rise_mask] + TACTILE_RISE_ALPHA * (filtered[rise_mask] - prev[rise_mask])
            smoothed[fall_mask] = prev[fall_mask] + TACTILE_FALL_ALPHA * (filtered[fall_mask] - prev[fall_mask])
        display_floor_mask = np.isfinite(smoothed) & (smoothed >= 0.0) & (smoothed < TACTILE_DISPLAY_FLOOR)
        smoothed[display_floor_mask] = 0.0
        self._display_matrix = smoothed

        vmax = TACTILE_AXIS_LEVELS[min(max(int(axis), 0), len(TACTILE_AXIS_LEVELS) - 1)]
        finite = np.isfinite(smoothed)
        display = np.zeros(smoothed.shape, dtype=float)
        display[finite] = 1.0
        positive = finite & (smoothed > 0.0)
        norm = np.sqrt(np.clip(smoothed[positive] / vmax, 0.0, 1.0))
        display[positive] = 2.0 + norm * 253.0
        self.image.setImage(display.T, levels=(0.0, 255.0), autoLevels=False)
        if self._last_matrix_shape != data.shape:
            self._last_matrix_shape = data.shape
            self.view.setRange(xRange=(0, data.shape[0]), yRange=(0, data.shape[1]), padding=0.02)
        self.no_data.setVisible(not has_data)

    def _fake_display_state(self, state: Optional[HandModel]) -> HandModel:
        now = time.time()
        if self._fake_state is None or now - getattr(self, "_last_fake_ts", 0.0) >= TACTILE_FAKE_INTERVAL_S:
            out = HandModel()
            out.tactile_data = self._fake_tactile()
            self._fake_state = out
            self._last_fake_ts = now
        return self._fake_state

    def _fake_tactile(self) -> List[FingerTactile]:
        t = time.time()
        out: List[FingerTactile] = []
        for finger_idx in range(5):
            sensors: List[TactileSensor] = []
            for seg in (2, 0, 0):
                r, c = tactile_segment_shape(seg)
                cf = np.zeros((r, c, 3), dtype=float)
                phase = t * 0.8 + finger_idx + len(sensors) * 1.3
                cf[:, :, 2] = 18.0 + 10.0 * np.sin(phase) + np.random.rand(r, c) * 24.0
                cf[:, :, 0] = np.random.rand(r, c) * 6.0
                cf[:, :, 1] = np.random.rand(r, c) * 6.0
                sensors.append(TactileSensor(contact_forces=cf, resultant=np.mean(cf, axis=(0, 1))))
            out.append(FingerTactile(sensors=sensors))
        return out

    @staticmethod
    def _extract_finger_strip(fd: FingerTactile, axis: int) -> np.ndarray:
        strip = np.full((TACTILE_FINGER_STRIP_ROWS, TACTILE_BAND_WIDTH), np.nan, dtype=float)
        row = 0
        for band in range(TACTILE_NBANDS):
            h, w = tactile_band_shape(band)
            if band in TACTILE_JOINT_BAND_INDICES:
                row += h
                continue
            si = TACTILE_BAND_SENSOR_INDEX.get(band)
            if si is not None and si < len(fd.sensors):
                cf = getattr(fd.sensors[si], "contact_forces", None)
                if cf is not None and hasattr(cf, "shape") and len(cf.shape) >= 3:
                    raw_ch = np.asarray(cf[:, :, min(axis, cf.shape[2] - 1)], dtype=float)
                    ch = np.clip(raw_ch, 0, None) if axis == 2 else np.abs(raw_ch)
                    if ch.shape == (h, w):
                        strip[row:row + h, :w] = ch
                    elif band == 0:
                        strip[row:row + h, :] = tactile_upsample_tip_grid_to_band0(ch)
                    else:
                        strip[row:row + h, :] = tactile_upsample_pad_grid_to_band(ch, h, w)
            row += h
        return strip

    @staticmethod
    def _extract_sensor_array(fd: FingerTactile, sensor_index: int, axis: int) -> np.ndarray:
        expected = tactile_segment_shape(2 if sensor_index == 0 else 0)
        out = np.zeros(expected, dtype=float)
        if sensor_index >= len(fd.sensors):
            return out
        cf = getattr(fd.sensors[sensor_index], "contact_forces", None)
        if cf is None or not hasattr(cf, "shape") or len(cf.shape) < 3:
            return out

        arr = np.asarray(cf, dtype=float)
        axis_index = min(max(int(axis), 0), arr.shape[2] - 1)
        raw = arr[:, :, axis_index]
        values = np.clip(raw, 0.0, None) if axis_index == 2 else np.abs(raw)
        if values.shape == expected:
            return values
        if values.size == expected[0] * expected[1]:
            return values.reshape(expected)
        rows = min(expected[0], values.shape[0])
        cols = min(expected[1], values.shape[1])
        out[:rows, :cols] = values[:rows, :cols]
        return out

    @staticmethod
    def _place_sensor_array(
        mat: np.ndarray,
        top: int,
        left: int,
        height: int,
        width: int,
        values: np.ndarray,
    ) -> None:
        scale = int(TACTILE_2D_ARRAY_SCALE)
        region_top = int(top * scale)
        region_left = int(left * scale)
        region_h = max(1, int(height * scale))
        region_w = max(1, int(width * scale))
        rows, cols = values.shape
        if rows <= 0 or cols <= 0:
            return

        row_pitch = region_h / rows
        col_pitch = region_w / cols
        radius = max(2, int(round(min(row_pitch, col_pitch) * 0.38)))
        for r in range(rows):
            cy = region_top + (r + 0.5) * row_pitch
            for c in range(cols):
                cx = region_left + (c + 0.5) * col_pitch
                r0 = max(0, int(math.floor(cy - radius)))
                r1 = min(mat.shape[0], int(math.ceil(cy + radius + 1)))
                c0 = max(0, int(math.floor(cx - radius)))
                c1 = min(mat.shape[1], int(math.ceil(cx + radius + 1)))
                if r1 <= r0 or c1 <= c0:
                    continue
                yy, xx = np.ogrid[r0:r1, c0:c1]
                mask = (yy - cy) * (yy - cy) + (xx - cx) * (xx - cx) <= radius * radius
                patch = mat[r0:r1, c0:c1]
                patch[mask] = values[r, c]

    def _place_finger_arrays(
        self,
        mat: np.ndarray,
        finger: FingerTactile,
        top: int,
        left: int,
        axis: int,
    ) -> None:
        strip = self._extract_finger_strip(finger, axis)
        row = 0
        for band in range(TACTILE_NBANDS):
            h, w = tactile_band_shape(band)
            if band not in TACTILE_JOINT_BAND_INDICES:
                values = strip[row:row + h, :w]
                self._place_sensor_array(mat, top + row, left, h, w, values)
            row += h

    def _build_matrix(self, state: Optional[HandModel], axis: int) -> np.ndarray:
        spec = TACTILE_STITCH_SPEC
        scale = int(TACTILE_2D_ARRAY_SCALE)
        mat = np.full((spec.height * scale, spec.width * scale), np.nan, dtype=float)
        tactile_list = state.tactile_data if state and state.tactile_data else []

        def fd(idx: int) -> FingerTactile:
            return tactile_list[idx] if idx < len(tactile_list) else FingerTactile(sensors=[])

        self._place_finger_arrays(mat, fd(0), spec.thumb_r, spec.thumb_c, axis)
        for fi in range(4):
            step = TACTILE_BAND_WIDTH + TACTILE_FOUR_FINGER_GAP
            r0, c0 = spec.four_r, spec.four_c + fi * step
            self._place_finger_arrays(mat, fd(fi + 1), r0, c0, axis)
        return mat
