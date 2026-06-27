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

from hand_backend import list_ports
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
from .official_tactile_layout import get_official_tactile_layout
from .ui_qt_compat import QtCore, QtWidgets
import pyqtgraph as pg


TACTILE_REAL_INTERVAL_S = 0.033
TACTILE_FAKE_INTERVAL_S = 0.25
TACTILE_NOISE_FLOOR = 1.0
TACTILE_RISE_ALPHA = 0.85
TACTILE_FALL_ALPHA = 0.45
TACTILE_DISPLAY_FLOOR = 6.0
TACTILE_AXIS_LEVELS = (70.0, 70.0, 180.0)
TACTILE_2D_ARRAY_SCALE = 8
TACTILE_2D_MIN_DOT_RADIUS = 2.4
TACTILE_2D_MAX_DOT_RADIUS = 13.0
TACTILE_2D_DENSE_POINT_LIMIT = 1500
TACTILE_2D_DENSE_MIN_DOT_RADIUS = 1.0
TACTILE_2D_DENSE_MAX_DOT_RADIUS = 4.2
TACTILE_2D_OFFICIAL_INTERP_STRIDE = 12
TACTILE_2D_Z_VISUAL_LEVEL = 35.0
TACTILE_2D_ARROW_FLOOR = 5.0
TACTILE_2D_ARROW_PAD_FLOOR = 3.0
TACTILE_2D_ARROW_CONTACT_FLOOR = TACTILE_DISPLAY_FLOOR
TACTILE_2D_ARROW_CONTACT_RATIO = 0.16
TACTILE_2D_ARROW_MAX_LEVEL = 12.0
TACTILE_2D_ARROW_GRID_STEP = 18
TACTILE_2D_ARROW_MAX_COUNT = 12
TACTILE_2D_ARROW_RESULTANT_FLOOR = 3.0
TACTILE_2D_ARROW_RESULTANT_Z_RATIO = 0.35
TACTILE_2D_SHOW_ARROWS_IN_Z_VIEW = True
TACTILE_2D_SHOW_HAND_GUIDE = True
TACTILE_2D_GUIDE_FILL = (248, 250, 252, 255)
TACTILE_2D_GUIDE_PALM_FILL = (241, 245, 249, 255)
TACTILE_2D_GUIDE_EDGE = (148, 163, 184, 255)
TACTILE_2D_GUIDE_SOFT_EDGE = (203, 213, 225, 255)
TACTILE_2D_IDLE_DOT = (71, 85, 105, 255)
TACTILE_2D_MANUAL_ORIGIN_AT_ROOT = True
TACTILE_2D_MANUAL_MIRROR_COLS = False
TACTILE_2D_REQUIRE_RESULTANT_GATE = False
TACTILE_3D_RISE_ALPHA = 0.62
TACTILE_3D_FALL_ALPHA = 0.48
TACTILE_3D_PRESS_LIFT = 0.0
TACTILE_3D_DISPLAY_FLOOR = 2.0
TACTILE_3D_DOT_RADIUS = 0.00155
TACTILE_3D_HALO_RADIUS = 0.0054
TACTILE_3D_SURFACE_OFFSET = 0.003
TACTILE_FINGER_LABELS = ("Thumb", "Index", "Middle", "Ring", "Pinky")
TACTILE_MODULE_DISPLAY_ORDER = (2, 1, 0)
TACTILE_SENSOR_INDEX_FROM_MODULE = {2: 0, 1: 1, 0: 2}
DESKTOP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF_HAND_PATH = os.path.join(DESKTOP_DIR, "hku_hand_v2_urdf", "urdf", "hand.urdf")
URDF_MAX_TRIANGLES_PER_LINK = 0
_URDF_HAND_MESH_CACHE = None
_URDF_HAND_LINK_VERTEX_CACHE = None
_OFFICIAL_TACTILE_PROJECTION_CACHE = {}
S3_TACTILE_PORT_SETTINGS_KEY = "s3_tactile/last_port"


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


@dataclass(frozen=True)
class _TaxelPeak:
    score: float
    raw_index: int
    raw_row: int
    raw_col: int
    ui_row: int
    ui_col: int
    raw_xyz: Tuple[int, int, int]
    processed_xyz: Tuple[int, int, int]
    byte_z: Optional[int]


@dataclass(frozen=True)
class _TaxelCenter:
    total: float
    raw_row: float
    raw_col: float
    ui_row: float
    ui_col: float


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

    def clear(self) -> None:
        with self._lock:
            self._pending = None
            self._latest = None

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
        self.raw_visible = False
        self._raw_text_cache = ""
        self.module_status_labels: List[List[QtWidgets.QLabel]] = []
        self._settings = QtCore.QSettings("mmhand", "desktop")
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
        self.raw_check = QtWidgets.QCheckBox("Raw")
        self.raw_check.setChecked(False)
        top.addWidget(self.raw_check)
        self.view3d_check = QtWidgets.QCheckBox("3D")
        self.view3d_check.setChecked(False)
        top.addWidget(self.view3d_check)
        top.addStretch(1)
        layout.addLayout(top)
        self.axis_group.idClicked.connect(self._set_axis)
        self.fake_check.toggled.connect(self._set_fake)
        self.raw_check.toggled.connect(self._set_raw_visible)
        self.view3d_check.toggled.connect(self._set_view3d)

        s3_row = QtWidgets.QHBoxLayout()
        s3_row.setSpacing(6)
        s3_row.addWidget(QtWidgets.QLabel("S3 Tactile Port:"))
        self.s3_port_combo = QtWidgets.QComboBox()
        self.s3_port_combo.setMinimumWidth(230)
        self.s3_port_combo.setToolTip("Only for S3 tactile serial. Do not select the P4 main control port.")
        s3_row.addWidget(self.s3_port_combo)
        self.s3_refresh_btn = QtWidgets.QPushButton("Refresh")
        self.s3_refresh_btn.setToolTip("Refresh available serial ports for the S3 tactile board")
        s3_row.addWidget(self.s3_refresh_btn)
        self.s3_connect_btn = QtWidgets.QPushButton("Connect S3")
        self.s3_connect_btn.setToolTip("Connect the selected port as the S3 tactile serial stream")
        s3_row.addWidget(self.s3_connect_btn)
        self.s3_disconnect_btn = QtWidgets.QPushButton("Disconnect")
        self.s3_disconnect_btn.setToolTip("Disconnect only the S3 tactile serial stream")
        s3_row.addWidget(self.s3_disconnect_btn)
        self.s3_status_label = QtWidgets.QLabel("S3 tactile: disconnected")
        self.s3_status_label.setMinimumWidth(220)
        self.s3_status_label.setStyleSheet("color:#475569;")
        s3_row.addWidget(self.s3_status_label, 1)
        s3_hint = QtWidgets.QLabel("S3 tactile only, not P4 main port")
        s3_hint.setStyleSheet("color:#64748b;")
        s3_row.addWidget(s3_hint)
        layout.addLayout(s3_row)
        module_row = QtWidgets.QHBoxLayout()
        module_row.setSpacing(10)
        module_row.addWidget(QtWidgets.QLabel("Touch modules:"))
        self.module_status_labels = []
        for finger_idx, finger_name in enumerate(TACTILE_FINGER_LABELS):
            group = QtWidgets.QHBoxLayout()
            group.setSpacing(3)
            name_label = QtWidgets.QLabel(finger_name)
            name_label.setStyleSheet("color:#334155;")
            group.addWidget(name_label)
            labels: List[QtWidgets.QLabel] = []
            for module_id in TACTILE_MODULE_DISPLAY_ORDER:
                item = QtWidgets.QLabel(str(module_id))
                item.setAlignment(QtCore.Qt.AlignCenter)
                item.setFixedSize(20, 20)
                item.setToolTip(f"{finger_name} tactile module {module_id}")
                labels.append(item)
                group.addWidget(item)
            holder = QtWidgets.QWidget()
            holder.setLayout(group)
            module_row.addWidget(holder)
            self.module_status_labels.append(labels)
        module_row.addStretch(1)
        layout.addLayout(module_row)
        self.s3_refresh_btn.clicked.connect(lambda _checked=False: self._refresh_s3_ports())
        self.s3_connect_btn.clicked.connect(lambda _checked=False: self._connect_s3_tactile_port())
        self.s3_disconnect_btn.clicked.connect(lambda _checked=False: self._disconnect_s3_tactile_port())
        self._refresh_s3_ports()
        self._refresh_s3_status()

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
        self.values_label.setVisible(False)
        self.peak_label = QtWidgets.QLabel("")
        self.peak_label.setWordWrap(True)
        self.peak_label.setStyleSheet("color: #0f172a; font-family: Consolas, 'Courier New', monospace;")
        layout.addWidget(self.peak_label)
        self.peak_label.setVisible(False)
        self.raw_text = QtWidgets.QPlainTextEdit()
        self.raw_text.setReadOnly(True)
        self.raw_text.setMaximumHeight(260)
        self.raw_text.setVisible(False)
        try:
            self.raw_text.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        except AttributeError:
            self.raw_text.setLineWrapMode(QtWidgets.QPlainTextEdit.LineWrapMode.NoWrap)
        self.raw_text.setStyleSheet(
            "color: #0f172a; background: #f8fafc; "
            "font-family: Consolas, 'Courier New', monospace; font-size: 11px;"
        )
        layout.addWidget(self.raw_text)
        self._apply_matrix(self._build_matrix(None, self.axis), self.axis, False)
        self._refresh_tactile_module_status(None)

    def build(self, _parent) -> None:
        pass

    def update(self, state: Optional[HandModel]) -> None:
        self.update_if_due(state, force=True)

    def update_if_due(self, state: Optional[HandModel], *, force: bool = False) -> bool:
        use_fake = self._should_use_fake(state)
        interval = TACTILE_FAKE_INTERVAL_S if use_fake else TACTILE_REAL_INTERVAL_S
        now = time.time()
        if not force and now - self._last_update_ts < interval:
            return False
        self._last_update_ts = now
        self._refresh_s3_status()
        display_state = self._fake_display_state(state) if use_fake else state
        self._refresh_tactile_module_status(display_state)
        if self.values_label.isVisible():
            self.values_label.setText(self._format_values(display_state))
        if self.peak_label.isVisible():
            self.peak_label.setText(self._format_peak_values(display_state, self.axis))
        if self.raw_visible:
            self._update_raw_text(self._format_raw_values(display_state, self.axis))
        has_data = bool(display_state and getattr(display_state, "tactile_data", None))
        if self.use_3d and self.hand_3d is not None:
            self.hand_3d.update_state(display_state, self.axis, has_data)

        self._apply_ready_frame()
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
        self.raw_text.clear()
        self._raw_text_cache = ""
        self._refresh_tactile_module_status(None)
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

    def _set_raw_visible(self, checked: bool) -> None:
        self.raw_visible = bool(checked)
        self.raw_text.setVisible(self.raw_visible)
        if not self.raw_visible:
            self.raw_text.clear()
            self._raw_text_cache = ""
            return
        self.update_if_due(self.context.get_state(), force=True)

    def _update_raw_text(self, text: str) -> None:
        if text == self._raw_text_cache:
            return
        self._raw_text_cache = text
        vbar = self.raw_text.verticalScrollBar()
        hbar = self.raw_text.horizontalScrollBar()
        old_v = vbar.value()
        old_h = hbar.value()
        self.raw_text.setUpdatesEnabled(False)
        try:
            self.raw_text.setPlainText(text)
            vbar.setValue(min(old_v, vbar.maximum()))
            hbar.setValue(min(old_h, hbar.maximum()))
        finally:
            self.raw_text.setUpdatesEnabled(True)

        def restore_scroll() -> None:
            vbar.setValue(min(old_v, vbar.maximum()))
            hbar.setValue(min(old_h, hbar.maximum()))

        QtCore.QTimer.singleShot(0, restore_scroll)

    def _selected_s3_port(self) -> str:
        data = self.s3_port_combo.currentData() if hasattr(self, "s3_port_combo") else None
        return str(data or "").strip()

    def _preferred_s3_port(self) -> str:
        try:
            current = str(self.context.get_tactile_serial_port() or "").strip()
        except Exception:
            current = ""
        if current:
            return current
        try:
            saved = self._settings.value(S3_TACTILE_PORT_SETTINGS_KEY, "", type=str)
        except TypeError:
            saved = self._settings.value(S3_TACTILE_PORT_SETTINGS_KEY, "")
        return str(saved or "").strip()

    def _refresh_s3_ports(self) -> None:
        preferred = self._preferred_s3_port() or self._selected_s3_port()
        self.s3_port_combo.blockSignals(True)
        self.s3_port_combo.clear()
        self.s3_port_combo.addItem("Select S3 tactile port", "")
        for device, desc in list_ports():
            self.s3_port_combo.addItem(f"{device} ({desc})", device)
        if preferred:
            idx = self.s3_port_combo.findData(preferred)
            if idx < 0:
                self.s3_port_combo.addItem(f"{preferred} (saved, not currently listed)", preferred)
                idx = self.s3_port_combo.findData(preferred)
            if idx >= 0:
                self.s3_port_combo.setCurrentIndex(idx)
        self.s3_port_combo.blockSignals(False)
        self._refresh_s3_status()

    def _p4_main_port(self) -> str:
        return str(getattr(self.context, "current_port", "") or "").strip()

    def _connect_s3_tactile_port(self) -> None:
        port = self._selected_s3_port()
        if not port:
            self.s3_status_label.setText("S3 tactile: select a port first")
            return
        p4_port = self._p4_main_port()
        if p4_port and port.upper() == p4_port.upper():
            QtWidgets.QMessageBox.warning(
                self,
                "S3触觉串口提示",
                (
                    f"{port} 当前看起来是 P4 主控串口。\n\n"
                    "S3触觉串口需要选择另一块 S3 板子的串口，避免和 P4 控制串口混用。"
                ),
            )
            self.s3_status_label.setText("S3 tactile: blocked, selected P4 port")
            return
        try:
            ok = bool(self.context.connect_tactile_serial(port))
        except Exception as exc:
            self.s3_status_label.setText(f"S3 tactile: failed ({exc})")
            return
        if ok:
            self._settings.setValue(S3_TACTILE_PORT_SETTINGS_KEY, port)
            self.s3_status_label.setText(f"S3 tactile connected: {port}")
            self.context.set_status(f"S3 tactile serial connected: {port}")
        else:
            self.s3_status_label.setText(f"S3 tactile connect failed: {port}")
        self._refresh_s3_status()

    def _disconnect_s3_tactile_port(self) -> None:
        try:
            self.context.disconnect_tactile_serial()
        except Exception as exc:
            self.s3_status_label.setText(f"S3 tactile disconnect failed: {exc}")
            return
        self.s3_status_label.setText("S3 tactile: disconnected")
        self.clear()
        self._refresh_s3_status()

    def _refresh_s3_status(self) -> None:
        if not hasattr(self, "s3_status_label"):
            return
        try:
            connected = bool(self.context.is_tactile_serial_connected())
        except Exception:
            connected = False
        try:
            port = str(self.context.get_tactile_serial_port() or "").strip()
        except Exception:
            port = ""
        if connected:
            text = f"S3 tactile connected: {port or self._selected_s3_port()}"
            color = "#166534"
        else:
            text = "S3 tactile: disconnected"
            color = "#475569"
        self.s3_status_label.setText(text)
        self.s3_status_label.setStyleSheet(f"color:{color};")
        self.s3_connect_btn.setEnabled(not connected)
        self.s3_disconnect_btn.setEnabled(connected)

    def _refresh_tactile_module_status(self, state: Optional[HandModel]) -> None:
        if not getattr(self, "module_status_labels", None):
            return
        module_online = list(getattr(state, "tactile_module_online", []) or []) if state is not None else []
        module_ts = list(getattr(state, "tactile_module_last_rx_ts", []) or []) if state is not None else []
        tactile_online = bool(getattr(state, "tactile_online", False)) if state is not None else False
        tactile = state.tactile_data if state is not None and getattr(state, "tactile_data", None) else []
        now = time.time()
        for finger_idx, labels in enumerate(self.module_status_labels):
            finger_name = TACTILE_FINGER_LABELS[finger_idx] if finger_idx < len(TACTILE_FINGER_LABELS) else f"G{finger_idx}"
            finger = tactile[finger_idx] if finger_idx < len(tactile) else FingerTactile(sensors=[])
            for label_idx, module_id in enumerate(TACTILE_MODULE_DISPLAY_ORDER):
                if label_idx >= len(labels):
                    continue
                sensor_idx = TACTILE_SENSOR_INDEX_FROM_MODULE.get(int(module_id), int(module_id))
                status_idx = finger_idx * 3 + int(sensor_idx)
                known = status_idx < len(module_online)
                sensor_active = (
                    int(sensor_idx) < len(finger.sensors) and
                    self._sensor_has_tactile_activity(finger.sensors[int(sensor_idx)])
                )
                online = bool(tactile_online and ((known and module_online[status_idx]) or sensor_active))
                ts = float(module_ts[status_idx]) if status_idx < len(module_ts) else 0.0
                age = (now - ts) if ts > 0.0 else None
                item = labels[label_idx]
                if online:
                    item.setStyleSheet(
                        "QLabel { color:#ffffff; background:#16a34a; border:1px solid #15803d; border-radius:3px; }"
                    )
                    age_text = f"{age:.2f}s ago" if age is not None else "recently"
                    source = "live values" if sensor_active else "module status"
                    item.setToolTip(f"{finger_name} module {module_id}: tactile data active ({source}), last data {age_text}")
                elif known or ts > 0.0:
                    item.setStyleSheet(
                        "QLabel { color:#ffffff; background:#dc2626; border:1px solid #991b1b; border-radius:3px; }"
                    )
                    age_text = f"{age:.2f}s ago" if age is not None else "unknown"
                    item.setToolTip(f"{finger_name} module {module_id}: no tactile data or stale, last data {age_text}")
                else:
                    item.setStyleSheet(
                        "QLabel { color:#475569; background:#e2e8f0; border:1px solid #cbd5e1; border-radius:3px; }"
                    )
                    item.setToolTip(f"{finger_name} module {module_id}: unknown, no tactile data yet")

    @staticmethod
    def _sensor_has_tactile_activity(sensor: TactileSensor) -> bool:
        resultant = getattr(sensor, "resultant", None)
        if resultant is not None and len(resultant) >= 3:
            force = np.asarray(resultant[:3], dtype=float)
            if np.any(np.isfinite(force)):
                level = max(abs(float(force[0])), abs(float(force[1])), max(0.0, float(force[2])))
                if level > TACTILE_NOISE_FLOOR:
                    return True

        for attr in ("raw_contact_forces", "contact_forces", "raw_unsigned_contact_forces"):
            values = getattr(sensor, attr, None)
            if values is None:
                continue
            arr = np.asarray(values, dtype=float)
            if arr.size == 0 or arr.ndim < 3 or arr.shape[2] < 3:
                continue
            if attr == "raw_unsigned_contact_forces":
                z = arr[:, :, 2]
                if np.any(np.isfinite(z)) and float(np.nanmax(z)) > TACTILE_NOISE_FLOOR:
                    return True
                continue
            xyz = arr[:, :, :3]
            if np.any(np.isfinite(xyz)) and float(np.nanmax(np.abs(xyz))) > TACTILE_NOISE_FLOOR:
                return True
        return False

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
    def _axis_name(axis: int) -> str:
        names = ("X", "Y", "Z")
        return names[min(max(int(axis), 0), len(names) - 1)]

    @staticmethod
    def _channel_matrix(values, axis: int) -> Optional[np.ndarray]:
        if values is None:
            return None
        arr = np.asarray(values, dtype=float)
        if arr.ndim == 2:
            return arr
        if arr.ndim < 3 or arr.shape[2] <= 0:
            return None
        axis_index = min(max(int(axis), 0), arr.shape[2] - 1)
        return np.asarray(arr[:, :, axis_index], dtype=float)

    @staticmethod
    def _matrix_stats(matrix: Optional[np.ndarray]) -> str:
        if matrix is None:
            return "none"
        arr = np.asarray(matrix, dtype=float)
        finite = arr[np.isfinite(arr)]
        if finite.size == 0:
            return "none"
        return (
            f"min={int(round(float(np.min(finite))))} "
            f"max={int(round(float(np.max(finite))))} "
            f"abs_sum={int(round(float(np.sum(np.abs(finite)))))}"
        )

    @staticmethod
    def _vector_sum(values) -> Tuple[int, int, int]:
        if values is None:
            return (0, 0, 0)
        arr = np.asarray(values, dtype=float)
        if arr.ndim < 3 or arr.shape[2] <= 0:
            return (0, 0, 0)
        out = []
        for channel in range(3):
            if channel >= arr.shape[2]:
                out.append(0)
                continue
            ch = np.asarray(arr[:, :, channel], dtype=float)
            finite = np.where(np.isfinite(ch), ch, 0.0)
            out.append(int(round(float(np.sum(finite)))))
        return (out[0], out[1], out[2])

    @staticmethod
    def _resultant_tuple(sensor: TactileSensor) -> Tuple[int, int, int]:
        resultant = getattr(sensor, "resultant", None)
        if resultant is None or len(resultant) < 3:
            return (0, 0, 0)
        out = []
        for channel in range(3):
            value = float(resultant[channel])
            out.append(int(round(value)) if np.isfinite(value) else 0)
        return (out[0], out[1], out[2])

    @staticmethod
    def _vector_at(values, row: int, col: int) -> Tuple[int, int, int]:
        if values is None:
            return (0, 0, 0)
        arr = np.asarray(values, dtype=float)
        if arr.ndim < 3 or arr.shape[2] <= 0 or row >= arr.shape[0] or col >= arr.shape[1]:
            return (0, 0, 0)
        out = []
        for channel in range(3):
            if channel >= arr.shape[2]:
                out.append(0)
                continue
            value = arr[row, col, channel]
            out.append(int(round(float(value))) if np.isfinite(value) else 0)
        return (out[0], out[1], out[2])

    @staticmethod
    def _ui_rc_from_raw_rc(rows: int, cols: int, row: int, col: int) -> Tuple[int, int]:
        ui_row = rows - 1 - row if TACTILE_2D_MANUAL_ORIGIN_AT_ROOT else row
        ui_col = cols - 1 - col if TACTILE_2D_MANUAL_MIRROR_COLS else col
        return ui_row, ui_col

    @staticmethod
    def _ui_rc_from_raw_xy(rows: int, cols: int, row: float, col: float) -> Tuple[float, float]:
        ui_row = float(rows - 1) - row if TACTILE_2D_MANUAL_ORIGIN_AT_ROOT else row
        ui_col = float(cols - 1) - col if TACTILE_2D_MANUAL_MIRROR_COLS else col
        return ui_row, ui_col

    @staticmethod
    def _is_dp52_shape(rows: int, cols: int) -> bool:
        return int(rows) == 4 and int(cols) == 13

    @classmethod
    def _ui_rc_from_raw_rc_for_sensor(
        cls,
        sensor_index: int,
        rows: int,
        cols: int,
        row: int,
        col: int,
    ) -> Tuple[int, int]:
        if int(sensor_index) == 0 and cls._is_dp52_shape(rows, cols):
            return int(cols) - 1 - int(col), int(row)
        return cls._ui_rc_from_raw_rc(rows, cols, row, col)

    @classmethod
    def _ui_rc_from_raw_xy_for_sensor(
        cls,
        sensor_index: int,
        rows: int,
        cols: int,
        row: float,
        col: float,
    ) -> Tuple[float, float]:
        if int(sensor_index) == 0 and cls._is_dp52_shape(rows, cols):
            return float(cols - 1) - float(col), float(row)
        return cls._ui_rc_from_raw_xy(rows, cols, row, col)

    @classmethod
    def _find_sensor_peak(
        cls,
        sensor: TactileSensor,
        axis: int,
        sensor_index: int = -1,
    ) -> Optional[_TaxelPeak]:
        raw_values = getattr(sensor, "raw_contact_forces", None)
        processed_values = getattr(sensor, "contact_forces", None)
        source = raw_values if raw_values is not None else processed_values
        if source is None:
            return None
        arr = np.asarray(source, dtype=float)
        if arr.ndim < 3 or arr.shape[0] <= 0 or arr.shape[1] <= 0 or arr.shape[2] <= 0:
            return None

        axis_index = min(max(int(axis), 0), arr.shape[2] - 1)
        channel = np.asarray(arr[:, :, axis_index], dtype=float)
        if axis_index == 2:
            scores = np.where(np.isfinite(channel), np.clip(channel, 0.0, None), -np.inf)
        else:
            scores = np.where(np.isfinite(channel), np.abs(channel), -np.inf)
        if not np.any(np.isfinite(scores)):
            return None
        score = float(np.nanmax(scores))
        if score <= 0.0:
            return None

        raw_row, raw_col = np.unravel_index(int(np.nanargmax(scores)), scores.shape)
        raw_row = int(raw_row)
        raw_col = int(raw_col)
        raw_index = raw_row * int(scores.shape[1]) + raw_col
        ui_row, ui_col = cls._ui_rc_from_raw_rc_for_sensor(
            sensor_index,
            int(scores.shape[0]),
            int(scores.shape[1]),
            raw_row,
            raw_col,
        )
        raw_unsigned = getattr(sensor, "raw_unsigned_contact_forces", None)
        byte_z = None
        if raw_unsigned is not None:
            unsigned_arr = np.asarray(raw_unsigned, dtype=float)
            if (
                unsigned_arr.ndim >= 3
                and raw_row < unsigned_arr.shape[0]
                and raw_col < unsigned_arr.shape[1]
                and unsigned_arr.shape[2] >= 3
            ):
                z_value = unsigned_arr[raw_row, raw_col, 2]
                byte_z = int(round(float(z_value))) if np.isfinite(z_value) else None
        return _TaxelPeak(
            score=score,
            raw_index=raw_index,
            raw_row=raw_row,
            raw_col=raw_col,
            ui_row=int(ui_row),
            ui_col=int(ui_col),
            raw_xyz=cls._vector_at(raw_values, raw_row, raw_col),
            processed_xyz=cls._vector_at(processed_values, raw_row, raw_col),
            byte_z=byte_z,
        )

    @classmethod
    def _find_sensor_center(
        cls,
        sensor: TactileSensor,
        axis: int,
        sensor_index: int = -1,
    ) -> Optional[_TaxelCenter]:
        raw_values = getattr(sensor, "raw_contact_forces", None)
        processed_values = getattr(sensor, "contact_forces", None)
        source = raw_values if raw_values is not None else processed_values
        if source is None:
            return None
        arr = np.asarray(source, dtype=float)
        if arr.ndim < 3 or arr.shape[0] <= 0 or arr.shape[1] <= 0 or arr.shape[2] <= 0:
            return None

        axis_index = min(max(int(axis), 0), arr.shape[2] - 1)
        channel = np.asarray(arr[:, :, axis_index], dtype=float)
        if axis_index == 2:
            weights = np.where(np.isfinite(channel), np.clip(channel, 0.0, None), 0.0)
        else:
            weights = np.where(np.isfinite(channel), np.abs(channel), 0.0)
        total = float(np.sum(weights))
        if total <= 0.0:
            return None

        row_grid, col_grid = np.indices(weights.shape, dtype=float)
        raw_row = float(np.sum(row_grid * weights) / total)
        raw_col = float(np.sum(col_grid * weights) / total)
        ui_row, ui_col = cls._ui_rc_from_raw_xy_for_sensor(
            sensor_index,
            int(weights.shape[0]),
            int(weights.shape[1]),
            raw_row,
            raw_col,
        )
        return _TaxelCenter(
            total=total,
            raw_row=raw_row,
            raw_col=raw_col,
            ui_row=float(ui_row),
            ui_col=float(ui_col),
        )

    @staticmethod
    def _format_peak_detail(peak: Optional[_TaxelPeak]) -> str:
        if peak is None:
            return "none"
        address = 1038 + peak.raw_index * 3
        byte_z = "-" if peak.byte_z is None else str(peak.byte_z)
        return (
            f"i0={peak.raw_index:02d} i1={peak.raw_index + 1:02d} addr={address} "
            f"raw_rc=({peak.raw_row},{peak.raw_col}) ui_rc=({peak.ui_row},{peak.ui_col}) "
            f"raw=({peak.raw_xyz[0]},{peak.raw_xyz[1]},{peak.raw_xyz[2]}) "
            f"byteZ={byte_z} "
            f"proc=({peak.processed_xyz[0]},{peak.processed_xyz[1]},{peak.processed_xyz[2]}) "
            f"score={int(round(peak.score))}"
        )

    @staticmethod
    def _format_center_detail(center: Optional[_TaxelCenter]) -> str:
        if center is None:
            return "none"
        return (
            f"raw_rc=({center.raw_row:.2f},{center.raw_col:.2f}) "
            f"ui_rc=({center.ui_row:.2f},{center.ui_col:.2f}) "
            f"total={int(round(center.total))}"
        )

    @classmethod
    def _format_sum_detail(cls, sensor: TactileSensor) -> str:
        raw_sum = cls._vector_sum(getattr(sensor, "raw_contact_forces", None))
        proc_sum = cls._vector_sum(getattr(sensor, "contact_forces", None))
        summary = cls._resultant_tuple(sensor)
        raw_delta = (
            raw_sum[0] - summary[0],
            raw_sum[1] - summary[1],
            raw_sum[2] - summary[2],
        )
        proc_delta = (
            proc_sum[0] - summary[0],
            proc_sum[1] - summary[1],
            proc_sum[2] - summary[2],
        )
        return (
            f"raw_sum=({raw_sum[0]},{raw_sum[1]},{raw_sum[2]}) "
            f"proc_sum=({proc_sum[0]},{proc_sum[1]},{proc_sum[2]}) "
            f"summary=({summary[0]},{summary[1]},{summary[2]}) "
            f"raw-summary=({raw_delta[0]},{raw_delta[1]},{raw_delta[2]}) "
            f"proc-summary=({proc_delta[0]},{proc_delta[1]},{proc_delta[2]})"
        )

    @classmethod
    def _format_peak_values(cls, state: Optional[HandModel], axis: int) -> str:
        tactile = state.tactile_data if state and state.tactile_data else []
        axis_index = min(max(int(axis), 0), 2)
        axis_name = cls._axis_name(axis_index)
        threshold = TACTILE_DISPLAY_FLOOR if axis_index == 2 else TACTILE_NOISE_FLOOR
        chunks = []
        for fi in range(5):
            finger = tactile[fi] if fi < len(tactile) else FingerTactile(sensors=[])
            for si in range(3):
                if si >= len(finger.sensors):
                    continue
                peak = cls._find_sensor_peak(finger.sensors[si], axis_index, si)
                if peak is None or peak.score < threshold:
                    continue
                center = cls._find_sensor_center(finger.sensors[si], axis_index, si)
                chunks.append(
                    f"G{fi}S{si} {axis_name}:i0={peak.raw_index:02d}/i1={peak.raw_index + 1:02d} "
                    f"raw_rc={peak.raw_row},{peak.raw_col} ui_rc={peak.ui_row},{peak.ui_col} "
                    f"center={cls._format_center_detail(center)} "
                    f"raw=({peak.raw_xyz[0]},{peak.raw_xyz[1]},{peak.raw_xyz[2]}) "
                    f"{cls._format_sum_detail(finger.sensors[si])}"
                )
        if not chunks:
            return f"Peak {axis_name}: none >= {int(round(threshold))}"
        return "Peak " + " | ".join(chunks)

    @staticmethod
    def _format_matrix(matrix: Optional[np.ndarray]) -> str:
        if matrix is None:
            return "  <none>"
        arr = np.asarray(matrix, dtype=float)
        if arr.ndim != 2:
            return "  <bad-shape>"
        lines = []
        for row in arr:
            cells = []
            for value in row:
                if not np.isfinite(value):
                    cells.append("   .")
                else:
                    cells.append(f"{int(round(float(value))):4d}")
            lines.append(" ".join(cells))
        return "\n".join(lines)

    @classmethod
    def _format_raw_values(cls, state: Optional[HandModel], axis: int) -> str:
        tactile = state.tactile_data if state and state.tactile_data else []
        axis_index = min(max(int(axis), 0), 2)
        axis_name = cls._axis_name(axis_index)
        if not tactile:
            return f"Axis {axis_name}\nNo tactile data."

        lines = [f"Axis {axis_name} raw debug"]
        if state is not None:
            summary_seq = getattr(state, "tactile_summary_seq", None)
            full_seq = getattr(state, "tactile_full_seq", None)
            full_summary_seq = getattr(state, "tactile_full_summary_seq", None)
            full_crc = getattr(state, "tactile_full_crc", None)
            crc_text = f"0x{int(full_crc) & 0xFFFF:04X}" if full_crc is not None else "-"
            last_rx = float(getattr(state, "tactile_last_rx_ts", 0.0) or 0.0)
            age_text = f"{time.time() - last_rx:.2f}s" if last_rx > 0.0 else "-"
            module_online = list(getattr(state, "tactile_module_online", []) or [])
            module_text = []
            for fi in range(5):
                vals = []
                finger = tactile[fi] if fi < len(tactile) else FingerTactile(sensors=[])
                for module_id in TACTILE_MODULE_DISPLAY_ORDER:
                    sensor_idx = TACTILE_SENSOR_INDEX_FROM_MODULE.get(int(module_id), int(module_id))
                    idx = fi * 3 + int(sensor_idx)
                    sensor_active = (
                        int(sensor_idx) < len(finger.sensors) and
                        cls._sensor_has_tactile_activity(finger.sensors[int(sensor_idx)])
                    )
                    vals.append(
                        "1"
                        if (
                            (idx < len(module_online) and module_online[idx]) or
                            sensor_active
                        )
                        else "0"
                    )
                module_text.append("".join(vals))
            lines.append(
                "link "
                f"online={1 if bool(getattr(state, 'tactile_online', False)) else 0} "
                f"age={age_text} "
                f"modules(210)={','.join(module_text) if module_text else '-'} "
                f"summary_seq={summary_seq if summary_seq is not None else '-'} "
                f"full_seq={full_seq if full_seq is not None else '-'} "
                f"full_summary_seq={full_summary_seq if full_summary_seq is not None else '-'} "
                f"full_crc={crc_text} "
                f"crc_err={int(getattr(state, 'tactile_full_crc_errors', 0))} "
                f"seq_mismatch={int(getattr(state, 'tactile_full_seq_mismatches', 0))} "
                f"dropped={int(getattr(state, 'tactile_full_dropped', 0))}"
            )
        for fi in range(5):
            finger = tactile[fi] if fi < len(tactile) else FingerTactile(sensors=[])
            for si in range(3):
                if si >= len(finger.sensors):
                    continue
                sensor = finger.sensors[si]
                resultant = getattr(sensor, "resultant", None)
                if resultant is not None and len(resultant) >= 3:
                    result_text = (
                        f"({int(round(float(resultant[0]))):4d},"
                        f"{int(round(float(resultant[1]))):4d},"
                        f"{int(round(float(resultant[2]))):4d})"
                    )
                else:
                    result_text = "(   0,   0,   0)"

                raw_signed = cls._channel_matrix(getattr(sensor, "raw_contact_forces", None), axis_index)
                raw_unsigned_z = cls._channel_matrix(
                    getattr(sensor, "raw_unsigned_contact_forces", None),
                    2,
                )
                processed = cls._channel_matrix(getattr(sensor, "contact_forces", None), axis_index)

                lines.append(
                    f"G{fi} S{si} resultant={result_text} "
                    f"raw[{cls._matrix_stats(raw_signed)}] "
                    f"proc[{cls._matrix_stats(processed)}]"
                )
                lines.append(f"PEAK {axis_name}: {cls._format_peak_detail(cls._find_sensor_peak(sensor, axis_index, si))}")
                lines.append(f"CENTER {axis_name}: {cls._format_center_detail(cls._find_sensor_center(sensor, axis_index, si))}")
                lines.append(f"SUM: {cls._format_sum_detail(sensor)}")
                lines.append(f"RAW signed {axis_name}")
                lines.append(cls._format_matrix(raw_signed))
                if axis_index == 2:
                    lines.append("RAW byte Z")
                    lines.append(cls._format_matrix(raw_unsigned_z))
                lines.append(f"PROC {axis_name}")
                lines.append(cls._format_matrix(processed))
                lines.append("")
        return "\n".join(lines).rstrip()

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

    @staticmethod
    def _state_has_active_tactile_summary(state: Optional[HandModel]) -> bool:
        tactile = state.tactile_data if state and state.tactile_data else []
        for finger in tactile:
            for sensor in getattr(finger, "sensors", []) or []:
                resultant = getattr(sensor, "resultant", None)
                if resultant is None or len(resultant) < 3:
                    continue
                try:
                    if max(0.0, float(resultant[2])) >= TACTILE_DISPLAY_FLOOR:
                        return True
                except (TypeError, ValueError):
                    continue
        return False

    def _apply_ready_frame(self) -> None:
        result = self._worker.consume_latest()
        if result is None or result.axis != self.axis or result.token <= self._last_applied_token:
            return
        self._last_applied_token = result.token
        self._apply_matrix(result.matrix, result.axis, result.has_data)

    @staticmethod
    def _zero_tactile_data() -> List[FingerTactile]:
        fingers: List[FingerTactile] = []
        for _finger_idx in range(5):
            sensors: List[TactileSensor] = []
            for sensor_idx in range(3):
                rows, cols = tactile_segment_shape(0 if sensor_idx == 0 else 2)
                sensors.append(
                    TactileSensor(
                        contact_forces=np.zeros((rows, cols, 3), dtype=float),
                        resultant=np.zeros(3, dtype=float),
                    )
                )
            fingers.append(FingerTactile(sensors=sensors))
        return fingers

    def _build_zero_matrix(self, axis: int) -> np.ndarray:
        state = HandModel(tactile_data=self._zero_tactile_data())
        return self._build_matrix(state, axis)

    def _apply_matrix(
        self,
        mat: np.ndarray,
        axis: int,
        has_data: bool,
        *,
        reset_smoothing: bool = False,
    ) -> None:
        data = np.asarray(mat, dtype=float)
        if data.ndim == 2:
            vector_data = np.full(data.shape + (3,), np.nan, dtype=float)
            vector_data[:, :, 2] = data
        else:
            vector_data = data.copy()

        finite_mask = np.isfinite(vector_data[:, :, 2])
        current = vector_data.copy()
        z_low_mask = finite_mask & (np.abs(current[:, :, 2]) < TACTILE_NOISE_FLOOR)
        current[:, :, 2][z_low_mask] = 0.0
        xy_mag = np.sqrt(current[:, :, 0] * current[:, :, 0] + current[:, :, 1] * current[:, :, 1])
        xy_low_mask = finite_mask & (xy_mag < TACTILE_NOISE_FLOOR)
        current[:, :, 0][xy_low_mask] = 0.0
        current[:, :, 1][xy_low_mask] = 0.0

        smoothed = current.copy()
        previous = self._display_matrix
        if (
            not reset_smoothing
            and previous is not None
            and previous.shape == current.shape
        ):
            prev_finite = np.isfinite(previous[:, :, 2])
            blend_mask = finite_mask & prev_finite
            if np.any(blend_mask):
                current_abs = np.abs(current)
                previous_abs = np.abs(previous)
                alpha = np.where(
                    current_abs >= previous_abs,
                    TACTILE_RISE_ALPHA,
                    TACTILE_FALL_ALPHA,
                )
                smoothed[blend_mask] = (
                    previous[blend_mask] * (1.0 - alpha[blend_mask])
                    + current[blend_mask] * alpha[blend_mask]
                )

        display_floor_mask = finite_mask & (smoothed[:, :, 2] >= 0.0) & (smoothed[:, :, 2] < TACTILE_DISPLAY_FLOOR)
        smoothed[:, :, 2][display_floor_mask] = 0.0
        self._display_matrix = smoothed

        rendered = self._render_vector_matrix(smoothed, axis)
        self.image.setLookupTable(None)
        self.image.setImage(rendered.transpose(1, 0, 2), autoLevels=False)
        if self._last_matrix_shape != data.shape[:2]:
            self._last_matrix_shape = data.shape[:2]
            self.view.setRange(xRange=(0, data.shape[0]), yRange=(0, data.shape[1]), padding=0.02)
        self.no_data.setVisible(not has_data)

    @staticmethod
    def _tactile_band_for_matrix_point(row: int, col: int) -> Optional[int]:
        scale = max(1, int(TACTILE_2D_ARRAY_SCALE))
        base_row = int(row) // scale
        base_col = int(col) // scale
        spec = TACTILE_STITCH_SPEC

        def find_in_strip(top: int, left: int) -> Optional[int]:
            if not (
                top <= base_row < top + TACTILE_FINGER_STRIP_ROWS
                and left <= base_col < left + TACTILE_BAND_WIDTH
            ):
                return None
            local_row = base_row - top
            band_top = 0
            for band in range(TACTILE_NBANDS):
                band_h, _band_w = tactile_band_shape(band)
                if band_top <= local_row < band_top + band_h:
                    return band
                band_top += band_h
            return None

        thumb_band = find_in_strip(spec.thumb_r, spec.thumb_c)
        if thumb_band is not None:
            return thumb_band

        step = TACTILE_BAND_WIDTH + TACTILE_FOUR_FINGER_GAP
        for fi in range(4):
            band = find_in_strip(spec.four_r, spec.four_c + fi * step)
            if band is not None:
                return band
        return None

    @staticmethod
    def _fill_rect(
        image: np.ndarray,
        row: int,
        col: int,
        height: int,
        width: int,
        color: Tuple[int, int, int, int],
    ) -> None:
        r0 = max(0, int(row))
        c0 = max(0, int(col))
        r1 = min(image.shape[0], int(row + height))
        c1 = min(image.shape[1], int(col + width))
        if r1 <= r0 or c1 <= c0:
            return
        image[r0:r1, c0:c1] = color

    @classmethod
    def _draw_rect_outline(
        cls,
        image: np.ndarray,
        row: int,
        col: int,
        height: int,
        width: int,
        color: Tuple[int, int, int, int],
        *,
        line_width: int = 1,
    ) -> None:
        r0 = int(row)
        c0 = int(col)
        r1 = int(row + height - 1)
        c1 = int(col + width - 1)
        cls._draw_line(image, r0, c0, r0, c1, color, width=line_width)
        cls._draw_line(image, r1, c0, r1, c1, color, width=line_width)
        cls._draw_line(image, r0, c0, r1, c0, color, width=line_width)
        cls._draw_line(image, r0, c1, r1, c1, color, width=line_width)

    @classmethod
    def _draw_tactile_hand_guide(cls, image: np.ndarray) -> None:
        if not TACTILE_2D_SHOW_HAND_GUIDE:
            return
        spec = TACTILE_STITCH_SPEC
        scale = max(1, int(TACTILE_2D_ARRAY_SCALE))

        def rect(units_row: int, units_col: int, units_h: int, units_w: int):
            return (
                int(units_row * scale),
                int(units_col * scale),
                int(units_h * scale),
                int(units_w * scale),
            )

        palm = rect(spec.palm_r, spec.palm_c, spec.palm_h, spec.palm_w)
        cls._fill_rect(image, *palm, TACTILE_2D_GUIDE_PALM_FILL)
        cls._draw_rect_outline(image, *palm, TACTILE_2D_GUIDE_EDGE, line_width=1)

        finger_rects: List[Tuple[int, int, int, int]] = [
            rect(spec.thumb_r, spec.thumb_c, spec.thumb_h, spec.thumb_w)
        ]
        step = TACTILE_BAND_WIDTH + TACTILE_FOUR_FINGER_GAP
        for fi in range(4):
            finger_rects.append(
                rect(
                    spec.four_r,
                    spec.four_c + fi * step,
                    TACTILE_FINGER_STRIP_ROWS,
                    TACTILE_BAND_WIDTH,
                )
            )

        for r, c, h, w in finger_rects:
            cls._fill_rect(image, r, c, h, w, TACTILE_2D_GUIDE_FILL)
            cls._draw_rect_outline(image, r, c, h, w, TACTILE_2D_GUIDE_EDGE, line_width=1)

        # Soft connection lines make the separate sensor islands read as one hand.
        palm_top = spec.palm_r * scale
        palm_center_col = (spec.palm_c + spec.palm_w * 0.5) * scale
        for r, c, h, w in finger_rects[1:]:
            finger_root_row = r + h
            finger_center_col = c + w * 0.5
            cls._draw_line(
                image,
                palm_top,
                palm_center_col,
                finger_root_row,
                finger_center_col,
                TACTILE_2D_GUIDE_SOFT_EDGE,
                width=1,
            )

        thumb_r, thumb_c, thumb_h, thumb_w = finger_rects[0]
        cls._draw_line(
            image,
            spec.palm_r * scale + spec.palm_h * scale * 0.50,
            spec.palm_c * scale,
            thumb_r + thumb_h,
            thumb_c + thumb_w,
            TACTILE_2D_GUIDE_SOFT_EDGE,
            width=1,
        )

    def _render_vector_matrix(self, field: np.ndarray, axis: int) -> np.ndarray:
        rows, cols = field.shape[:2]
        image = np.full((rows, cols, 4), 255, dtype=np.uint8)
        finite = np.isfinite(field[:, :, 2])
        if not np.any(finite):
            return image

        self._draw_tactile_hand_guide(image)

        vmax_z = TACTILE_2D_Z_VISUAL_LEVEL
        positions = np.argwhere(finite)
        dense_mode = positions.shape[0] >= TACTILE_2D_DENSE_POINT_LIMIT
        min_radius = TACTILE_2D_DENSE_MIN_DOT_RADIUS if dense_mode else TACTILE_2D_MIN_DOT_RADIUS
        max_radius = TACTILE_2D_DENSE_MAX_DOT_RADIUS if dense_mode else TACTILE_2D_MAX_DOT_RADIUS
        base_rows = positions[:, 0]
        base_cols = positions[:, 1]
        idle_radius = 1 if dense_mode else 2
        self._draw_many_circles(
            image,
            base_rows,
            base_cols,
            idle_radius,
            TACTILE_2D_IDLE_DOT,
        )

        z_values = np.where(finite, np.clip(field[:, :, 2], 0.0, None), 0.0)
        active = finite & (z_values > 0.0)
        if np.any(active):
            active_positions = np.argwhere(active)
            active_rows = active_positions[:, 0]
            active_cols = active_positions[:, 1]
            z_norm = np.clip(z_values[active] / max(vmax_z, 1.0), 0.0, 1.0)
            radii = np.rint(
                min_radius
                + np.sqrt(z_norm) * (max_radius - min_radius)
            ).astype(np.int16)
            for radius in np.unique(radii):
                group = radii == radius
                ratio = (float(radius) - min_radius) / max(
                    1.0,
                    max_radius - min_radius,
                )
                red = int(round(180 + 75 * np.clip(ratio, 0.0, 1.0)))
                self._draw_many_circles(
                    image,
                    active_rows[group],
                    active_cols[group],
                    int(radius),
                    (red, 0, 0, 255),
                )

        if int(axis) == 2 and not TACTILE_2D_SHOW_ARROWS_IN_Z_VIEW:
            return image

        arrow_color = (0, 92, 255, 255)
        arrow_samples: Dict[Tuple[int, int], Tuple[float, int, int, float, float]] = {}
        fx_values = np.asarray(field[base_rows, base_cols, 0], dtype=float)
        fy_values = np.asarray(field[base_rows, base_cols, 1], dtype=float)
        fx_values = np.where(np.isfinite(fx_values), fx_values, 0.0)
        fy_values = np.where(np.isfinite(fy_values), fy_values, 0.0)
        mag_values = np.hypot(fx_values, fy_values)
        z_at_points = z_values[base_rows, base_cols]
        arrow_floor = np.full(mag_values.shape, float(TACTILE_2D_ARROW_PAD_FLOOR), dtype=float)
        contact = z_at_points >= TACTILE_2D_ARROW_CONTACT_FLOOR
        arrow_floor[contact] = np.maximum(
            arrow_floor[contact],
            z_at_points[contact] * TACTILE_2D_ARROW_CONTACT_RATIO,
        )
        arrow_indices = np.nonzero(mag_values >= arrow_floor)[0]
        for point_index in arrow_indices:
            row = int(base_rows[point_index])
            col = int(base_cols[point_index])
            fx = float(fx_values[point_index])
            fy = float(fy_values[point_index])
            mag = float(mag_values[point_index])
            z = max(0.0, float(z_at_points[point_index]))
            score = mag + z * 0.25
            key = (int(row) // TACTILE_2D_ARROW_GRID_STEP, int(col) // TACTILE_2D_ARROW_GRID_STEP)
            if key not in arrow_samples or score > arrow_samples[key][0]:
                arrow_samples[key] = (score, int(row), int(col), fx, fy)

        arrow_values = sorted(arrow_samples.values(), key=lambda item: item[0], reverse=True)
        for _score, row, col, fx, fy in arrow_values[:TACTILE_2D_ARROW_MAX_COUNT]:
            mag = math.hypot(fx, fy)
            unit_x = fx / max(mag, 1e-9)
            unit_y = -fy / max(mag, 1e-9)
            length = 12.0 + min(1.0, mag / max(TACTILE_2D_ARROW_MAX_LEVEL, 1.0)) * 34.0
            end_row = row + unit_y * length
            end_col = col + unit_x * length
            self._draw_line(image, row, col, end_row, end_col, (255, 255, 255, 255), width=4)
            self._draw_line(image, row, col, end_row, end_col, arrow_color, width=2)
            self._draw_arrow_head(image, row, col, end_row, end_col, arrow_color, width=3)
        return image

    @staticmethod
    def _draw_many_circles(
        image: np.ndarray,
        rows: np.ndarray,
        cols: np.ndarray,
        radius: int,
        color: Tuple[int, int, int, int],
    ) -> None:
        if radius <= 0 or rows.size == 0:
            return
        rows = np.asarray(rows, dtype=np.int32)
        cols = np.asarray(cols, dtype=np.int32)
        r2 = radius * radius
        for dy in range(-radius, radius + 1):
            span = int(math.floor(math.sqrt(max(0, r2 - dy * dy))))
            rr = rows + dy
            row_mask = (rr >= 0) & (rr < image.shape[0])
            if not np.any(row_mask):
                continue
            valid_rr = rr[row_mask]
            valid_cols = cols[row_mask]
            for dx in range(-span, span + 1):
                cc = valid_cols + dx
                mask = (cc >= 0) & (cc < image.shape[1])
                if np.any(mask):
                    image[valid_rr[mask], cc[mask]] = color

    @staticmethod
    def _draw_circle(
        image: np.ndarray,
        row: float,
        col: float,
        radius: int,
        color: Tuple[int, int, int, int],
    ) -> None:
        r0 = max(0, int(math.floor(row - radius)))
        r1 = min(image.shape[0], int(math.ceil(row + radius + 1)))
        c0 = max(0, int(math.floor(col - radius)))
        c1 = min(image.shape[1], int(math.ceil(col + radius + 1)))
        if r1 <= r0 or c1 <= c0:
            return
        yy, xx = np.ogrid[r0:r1, c0:c1]
        mask = (yy - row) * (yy - row) + (xx - col) * (xx - col) <= radius * radius
        image[r0:r1, c0:c1][mask] = color

    @classmethod
    def _draw_line(
        cls,
        image: np.ndarray,
        row0: float,
        col0: float,
        row1: float,
        col1: float,
        color: Tuple[int, int, int, int],
        *,
        width: int = 1,
    ) -> None:
        steps = max(1, int(math.ceil(max(abs(row1 - row0), abs(col1 - col0)) * 1.6)))
        t = np.linspace(0.0, 1.0, steps + 1, dtype=float)
        rows = np.rint(row0 + (row1 - row0) * t).astype(np.int32)
        cols = np.rint(col0 + (col1 - col0) * t).astype(np.int32)
        radius = max(0, int(width))
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dy * dy + dx * dx > radius * radius:
                    continue
                rr = rows + dy
                cc = cols + dx
                mask = (rr >= 0) & (rr < image.shape[0]) & (cc >= 0) & (cc < image.shape[1])
                if np.any(mask):
                    image[rr[mask], cc[mask]] = color

    @classmethod
    def _draw_arrow_head(
        cls,
        image: np.ndarray,
        row0: float,
        col0: float,
        row1: float,
        col1: float,
        color: Tuple[int, int, int, int],
        *,
        width: int = 2,
    ) -> None:
        dx = col1 - col0
        dy = row1 - row0
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return
        ux = dx / length
        uy = dy / length
        head = min(11.0, max(6.0, length * 0.42))
        spread = 0.62
        left_col = col1 - ux * head - uy * head * spread
        left_row = row1 - uy * head + ux * head * spread
        right_col = col1 - ux * head + uy * head * spread
        right_row = row1 - uy * head - ux * head * spread
        cls._draw_line(image, row1, col1, left_row, left_col, color, width=width)
        cls._draw_line(image, row1, col1, right_row, right_col, color, width=width)

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
            for seg in (0, 2, 2):
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
    def _orient_manual_sensor_array(values: np.ndarray) -> np.ndarray:
        out = np.asarray(values, dtype=float)
        if TACTILE_2D_MANUAL_ORIGIN_AT_ROOT:
            out = np.flip(out, axis=0)
        if TACTILE_2D_MANUAL_MIRROR_COLS:
            out = np.flip(out, axis=1)
        return np.ascontiguousarray(out)

    @classmethod
    def _orient_sensor_array(cls, values: np.ndarray, sensor_index: int) -> np.ndarray:
        out = np.asarray(values, dtype=float)
        if int(sensor_index) == 0 and out.ndim >= 2 and cls._is_dp52_shape(out.shape[0], out.shape[1]):
            out = np.swapaxes(out, 0, 1)
            out = np.flip(out, axis=0)
            return np.ascontiguousarray(out)
        return cls._orient_manual_sensor_array(out)

    @staticmethod
    def _sensor_resultant_is_active(sensor: TactileSensor) -> bool:
        if not TACTILE_2D_REQUIRE_RESULTANT_GATE:
            return True
        cf = getattr(sensor, "contact_forces", None)
        if cf is not None and hasattr(cf, "shape") and len(cf.shape) >= 3:
            arr = np.asarray(cf, dtype=float)
            if arr.size and arr.shape[2] >= 3:
                z = np.clip(arr[:, :, 2], 0.0, None)
                if np.any(np.isfinite(z)) and float(np.nanmax(z)) >= TACTILE_DISPLAY_FLOOR:
                    return True
        resultant = getattr(sensor, "resultant", None)
        if resultant is None or len(resultant) < 3:
            return True
        force = np.asarray(resultant[:3], dtype=float)
        if not np.all(np.isfinite(force)):
            return True
        return max(0.0, float(force[2])) >= TACTILE_DISPLAY_FLOOR

    @staticmethod
    def _sensor_resultant_allows_arrows(sensor: TactileSensor) -> bool:
        resultant = getattr(sensor, "resultant", None)
        if resultant is None or len(resultant) < 3:
            return True
        force = np.asarray(resultant[:3], dtype=float)
        if not np.all(np.isfinite(force)):
            return True
        xy_mag = math.hypot(float(force[0]), float(force[1]))
        z_mag = max(0.0, float(force[2]))
        if xy_mag < TACTILE_2D_ARROW_RESULTANT_FLOOR:
            return False
        if z_mag >= TACTILE_DISPLAY_FLOOR and xy_mag < z_mag * TACTILE_2D_ARROW_RESULTANT_Z_RATIO:
            return False
        return True

    @classmethod
    def _extract_sensor_vector_array_raw(cls, fd: FingerTactile, sensor_index: int) -> np.ndarray:
        expected = tactile_segment_shape(0 if sensor_index == 0 else 2)
        out = np.zeros(expected + (3,), dtype=float)
        if sensor_index >= len(fd.sensors):
            return out
        sensor = fd.sensors[sensor_index]
        if not cls._sensor_resultant_is_active(sensor):
            return out
        cf = getattr(sensor, "contact_forces", None)
        if cf is None or not hasattr(cf, "shape") or len(cf.shape) < 3:
            return out

        arr = np.asarray(cf, dtype=float)
        rows = min(expected[0], arr.shape[0])
        cols = min(expected[1], arr.shape[1])
        chans = min(3, arr.shape[2])
        out[:rows, :cols, :chans] = arr[:rows, :cols, :chans]
        out[:, :, 2] = np.clip(out[:, :, 2], 0.0, None)
        return out

    @classmethod
    def _extract_sensor_vector_array(cls, fd: FingerTactile, sensor_index: int) -> np.ndarray:
        expected = tactile_segment_shape(0 if sensor_index == 0 else 2)
        out = np.zeros(expected + (3,), dtype=float)
        if sensor_index >= len(fd.sensors):
            return cls._orient_sensor_array(out, sensor_index)
        sensor = fd.sensors[sensor_index]
        if not cls._sensor_resultant_is_active(sensor):
            return cls._orient_sensor_array(out, sensor_index)
        cf = getattr(sensor, "contact_forces", None)
        if cf is None or not hasattr(cf, "shape") or len(cf.shape) < 3:
            return cls._orient_sensor_array(out, sensor_index)

        arr = np.asarray(cf, dtype=float)
        rows = min(expected[0], arr.shape[0])
        cols = min(expected[1], arr.shape[1])
        chans = min(3, arr.shape[2])
        out[:rows, :cols, :chans] = arr[:rows, :cols, :chans]
        out[:, :, 2] = np.clip(out[:, :, 2], 0.0, None)
        return cls._orient_sensor_array(out, sensor_index)

    @classmethod
    def _place_official_sensor_array(
        cls,
        mat: np.ndarray,
        top: int,
        left: int,
        height: int,
        width: int,
        values: np.ndarray,
    ) -> bool:
        arr = np.asarray(values, dtype=float)
        if arr.ndim < 3 or arr.shape[2] < 3:
            return False

        flat = arr.reshape((-1, arr.shape[2]))
        scale = max(1, int(TACTILE_2D_ARRAY_SCALE))
        region_top = int(top * scale)
        region_left = int(left * scale)
        region_h = max(1, int(height * scale))
        region_w = max(1, int(width * scale))
        projection = cls._official_sensor_projection(flat.shape[0], region_h, region_w)
        if projection is None:
            return False

        local_rows, local_cols, raw_indices, neighbors, weights, interp_mask = projection
        vectors = np.zeros((local_rows.shape[0], 3), dtype=float)
        raw_mask = raw_indices >= 0
        if np.any(raw_mask):
            vectors[raw_mask] = flat[raw_indices[raw_mask], :3]
            vectors[raw_mask, 2] = np.clip(vectors[raw_mask, 2], 0.0, None)

        if np.any(interp_mask):
            interp_neighbors = neighbors[interp_mask]
            interp_weights = weights[interp_mask]
            safe_neighbors = np.clip(interp_neighbors, 0, flat.shape[0] - 1)
            gathered = flat[safe_neighbors, :3] * interp_weights[:, :, None]
            gathered[interp_neighbors < 0] = 0.0
            vectors[interp_mask] = np.sum(gathered, axis=1)
            vectors[interp_mask, 2] = np.abs(vectors[interp_mask, 2])

        vectors = np.where(np.isfinite(vectors), vectors, 0.0)
        rows = region_top + local_rows
        cols = region_left + local_cols
        in_bounds = (
            (rows >= 0)
            & (rows < mat.shape[0])
            & (cols >= 0)
            & (cols < mat.shape[1])
        )
        if not np.any(in_bounds):
            return True

        scores = np.abs(vectors[:, 2]) + 0.2 * np.hypot(vectors[:, 0], vectors[:, 1])
        order = np.argsort(scores[in_bounds])
        use_rows = rows[in_bounds][order]
        use_cols = cols[in_bounds][order]
        use_vectors = vectors[in_bounds][order]
        mat[use_rows, use_cols, :] = use_vectors
        return True

    @staticmethod
    def _official_sensor_projection(taxel_count: int, region_h: int, region_w: int):
        key = (
            int(taxel_count),
            int(region_h),
            int(region_w),
            int(TACTILE_2D_OFFICIAL_INTERP_STRIDE),
        )
        cached = _OFFICIAL_TACTILE_PROJECTION_CACHE.get(key)
        if cached is not None:
            return cached

        layout = get_official_tactile_layout(taxel_count)
        if layout is None:
            return None

        span_x = max(1e-9, float(layout.max_x - layout.min_x))
        span_y = max(1e-9, float(layout.max_y - layout.min_y))
        pad = max(2.0, min(region_h, region_w) * 0.035)
        usable_h = max(1.0, region_h - 1 - 2.0 * pad)
        usable_w = max(1.0, region_w - 1 - 2.0 * pad)

        selected = []
        interp_stride = max(1, int(TACTILE_2D_OFFICIAL_INTERP_STRIDE))
        for point_index, point in enumerate(layout.points):
            if point.idx is None and interp_stride > 1 and point_index % interp_stride != 0:
                continue
            local_col = pad + ((float(point.x) - float(layout.min_x)) / span_x) * usable_w
            local_row = pad + ((float(layout.max_y) - float(point.y)) / span_y) * usable_h
            selected.append((int(round(local_row)), int(round(local_col)), point))

        if not selected:
            return None

        max_neighbors = max((len(point.neighbor) for _row, _col, point in selected), default=0)
        max_neighbors = max(1, max_neighbors)
        local_rows = np.asarray([row for row, _col, _point in selected], dtype=np.int32)
        local_cols = np.asarray([col for _row, col, _point in selected], dtype=np.int32)
        raw_indices = np.full((len(selected),), -1, dtype=np.int32)
        neighbors = np.full((len(selected), max_neighbors), -1, dtype=np.int32)
        weights = np.zeros((len(selected), max_neighbors), dtype=float)
        interp_mask = np.zeros((len(selected),), dtype=bool)

        for index, (_row, _col, point) in enumerate(selected):
            if point.idx is not None:
                raw_indices[index] = int(point.idx)
                continue
            interp_mask[index] = True
            count = min(max_neighbors, len(point.neighbor), len(point.shape))
            if count:
                neighbors[index, :count] = np.asarray(point.neighbor[:count], dtype=np.int32)
                weights[index, :count] = np.asarray(point.shape[:count], dtype=float)

        projection = (local_rows, local_cols, raw_indices, neighbors, weights, interp_mask)
        _OFFICIAL_TACTILE_PROJECTION_CACHE[key] = projection
        return projection

    @classmethod
    def _extract_sensor_array(cls, fd: FingerTactile, sensor_index: int, axis: int) -> np.ndarray:
        expected = tactile_segment_shape(0 if sensor_index == 0 else 2)
        out = np.zeros(expected, dtype=float)
        if sensor_index >= len(fd.sensors):
            return cls._orient_sensor_array(out, sensor_index)
        sensor = fd.sensors[sensor_index]
        if not cls._sensor_resultant_is_active(sensor):
            return cls._orient_sensor_array(out, sensor_index)
        cf = getattr(sensor, "contact_forces", None)
        if cf is None or not hasattr(cf, "shape") or len(cf.shape) < 3:
            return cls._orient_sensor_array(out, sensor_index)

        arr = np.asarray(cf, dtype=float)
        axis_index = min(max(int(axis), 0), arr.shape[2] - 1)
        raw = arr[:, :, axis_index]
        values = np.clip(raw, 0.0, None) if axis_index == 2 else np.abs(raw)
        if values.shape == expected:
            return cls._orient_sensor_array(values, sensor_index)
        if values.size == expected[0] * expected[1]:
            return cls._orient_sensor_array(values.reshape(expected), sensor_index)
        rows = min(expected[0], values.shape[0])
        cols = min(expected[1], values.shape[1])
        out[:rows, :cols] = values[:rows, :cols]
        return cls._orient_sensor_array(out, sensor_index)

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
        rows, cols = values.shape[:2]
        if rows <= 0 or cols <= 0:
            return

        row_pitch = region_h / rows
        col_pitch = region_w / cols
        for r in range(rows):
            cy = region_top + (r + 0.5) * row_pitch
            rr = int(round(cy))
            if rr < 0 or rr >= mat.shape[0]:
                continue
            for c in range(cols):
                cx = region_left + (c + 0.5) * col_pitch
                cc = int(round(cx))
                if cc < 0 or cc >= mat.shape[1]:
                    continue
                mat[rr, cc, ...] = values[r, c, ...] if values.ndim == 3 else values[r, c]

    def _place_finger_arrays(
        self,
        mat: np.ndarray,
        finger: FingerTactile,
        top: int,
        left: int,
        axis: int,
    ) -> None:
        _ = axis
        row = 0
        for band in range(TACTILE_NBANDS):
            h, w = tactile_band_shape(band)
            if band not in TACTILE_JOINT_BAND_INDICES:
                si = TACTILE_BAND_SENSOR_INDEX.get(band)
                if si is not None:
                    raw_values = self._extract_sensor_vector_array_raw(finger, si)
                    if not self._place_official_sensor_array(
                        mat,
                        top + row,
                        left,
                        h,
                        w,
                        raw_values,
                    ):
                        values = self._extract_sensor_vector_array(finger, si)
                        self._place_sensor_array(mat, top + row, left, h, w, values)
            row += h

    def _build_matrix(self, state: Optional[HandModel], axis: int) -> np.ndarray:
        spec = TACTILE_STITCH_SPEC
        scale = int(TACTILE_2D_ARRAY_SCALE)
        mat = np.full((spec.height * scale, spec.width * scale, 3), np.nan, dtype=float)
        tactile_list = state.tactile_data if state and state.tactile_data else []

        def fd(idx: int) -> FingerTactile:
            return tactile_list[idx] if idx < len(tactile_list) else FingerTactile(sensors=[])

        self._place_finger_arrays(mat, fd(0), spec.thumb_r, spec.thumb_c, axis)
        for fi in range(4):
            step = TACTILE_BAND_WIDTH + TACTILE_FOUR_FINGER_GAP
            r0, c0 = spec.four_r, spec.four_c + fi * step
            self._place_finger_arrays(mat, fd(fi + 1), r0, c0, axis)
        return mat
