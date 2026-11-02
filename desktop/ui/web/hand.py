"""Three URDF poses: original model, contact-calibrated model, and target.

职责：把只读关节反馈、目标角和触觉合力/法向分布画到 URDF 三维场景。
线程：NiceGUI 页面事件循环；仅消费快照，不创建任务。
输入：后端同步快照。
输出：三只手的位姿、触觉点云和合力箭头。
状态所有权：不拥有设备状态，只缓存已显示的位姿和场景对象。
禁止依赖：禁止直接操作协议、串口和控制器。
"""
import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
from nicegui import app, ui

from desktop.core.contact_calibration import active_origins
from desktop.core.robot_model import manifest as robot_model_manifest
from desktop.core.tactile_geometry import SENSOR_LINKS, module_points
from desktop.core.tactile_heatmap import colormap, grid_coords, render_png

ROOT = Path(__file__).resolve().parents[2] / 'resources'
MODEL = ROOT / 'mmhand_urdf'
# mmhand.urdf puts the fingers along +z, so the camera looks down +y at a point
# level with the palm and keeps up exactly on +z: any lean in `up` rolls the
# whole hand over. The distance is fixed by the 0.80 vertical fill in the test.
CAMERA = dict(x=-0.019, y=-0.4555, z=0.0917, look_at_x=-0.019,
              look_at_y=-0.0536, look_at_z=0.0917,
              up_x=0.0, up_y=0.0, up_z=1.0)
FOV = 42
SCENE = dict(width=1200, height=620)
ARROW_RADIUS = 0.0012
ARROW_MIN = 5.0
POINT_SIZE = 0.0032
# 贴面热力图用 NiceGUI 的 texture 原语；关掉即退回放大后的点阵。
HEATMAP = True
# 贴图加载失败时仍能看见触觉：点阵默认保留。
SHOW_POINTS = True
_PLACEHOLDER_PNG = (b'\x89PNG\r\n\x1a\n' + bytes.fromhex(
    '0000000d49484452000000010000000108060000001f15c4890000000a4944415478'
    '9c6300010000050001' '0d0a2db40000000049454e44ae426082'))
_FRAME_DIR: Path | None = None


def _frame_dir() -> Path:
    """贴图目录按进程共享：静态路由只能注册一次，每客户端各建目录会 404。"""
    global _FRAME_DIR
    if _FRAME_DIR is None:
        _FRAME_DIR = Path(tempfile.mkdtemp(prefix='tactile-frames-'))
        try:
            app.add_static_files('/tactile-frames', str(_FRAME_DIR), max_cache_age=0)
        except RuntimeError:
            pass
    return _FRAME_DIR


def _vec(text, default):
    return tuple(float(v) for v in (text or default).split())


def _rotation_from_force(force):
    direction = np.asarray(force, float)
    norm = float(np.linalg.norm(direction))
    if norm <= 1e-9:
        return None, 0.0
    y = direction / norm
    helper = np.array([1.0, 0.0, 0.0]) if abs(y[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x = np.cross(helper, y)
    x /= max(np.linalg.norm(x), 1e-9)
    z = np.cross(x, y)
    return np.column_stack([x, y, z]).tolist(), norm


def build(hand):
    try:
        app.add_static_files('/hand-meshes', str(MODEL / 'meshes'))
    except RuntimeError:
        pass
    urdf = ET.parse(MODEL / 'urdf' / 'mmhand.urdf').getroot()
    mapping = robot_model_manifest()['joints']
    indices = {j['urdf_joint_name']: j['joint_index'] for j in mapping}
    links = {e.attrib['name']: e for e in urdf.findall('link')}
    joints = urdf.findall('joint')
    children = {j.find('child').attrib['link'] for j in joints}
    roots = set(links) - children
    by_parent = {}
    for j in joints:
        by_parent.setdefault(j.find('parent').attrib['link'], []).append(j)
    sensor_by_link = {link: key for key, link in SENSOR_LINKS.items()}
    corrections = active_origins()
    frame_dir = _frame_dir()
    frame_slot = {}

    with ui.row().classes('page-toolbar'):
        ui.label('Light gray: original URDF · Gray: contact-calibrated · Green: target').classes('text-sm')
        reset = ui.button('Reset view')
    status = ui.label('Waiting for feedback')
    camera = ui.scene.perspective_camera(fov=FOV, near=0.01)
    for key, value in CAMERA.items():
        setattr(camera, key, value)
    scene = ui.scene(grid=False, camera=camera, background_color='#ffffff',
                     **SCENE).classes('w-full')
    shown = False
    reset.on_click(lambda: scene.move_camera(**CAMERA))
    poses, tactile = [], {}

    def add_marker(key, parent, points):
        finger, segment = key
        kind = 'tip' if segment == 0 else 'pad'
        (frame_dir / f'{finger}-{segment}-0.png').write_bytes(_PLACEHOLDER_PNG)
        with parent:
            cloud = scene.point_cloud(points.tolist(),
                                      colors=colormap(np.zeros(len(points))),
                                      point_size=POINT_SIZE)
            cloud.visible(SHOW_POINTS)
            surface = scene.texture(f'/tactile-frames/{finger}-{segment}-0.png', grid_coords(kind))
            surface.visible(False)
            arrow = scene.cylinder(ARROW_RADIUS, ARROW_RADIUS, 1.0, 8)
            arrow.visible(False)
            dot = scene.sphere(1.0)
            dot.visible(False)
        frame_slot[key] = [0, None, 0]
        tactile[key] = (cloud, arrow, dot, surface, points, points.mean(axis=0))

    def add_link(name, motors, color, opacity, corrected, with_tactile):
        link = links[name]
        for visual in link.findall('visual'):
            mesh = visual.find('geometry/mesh')
            if mesh is None:
                continue
            origin = visual.find('origin')
            xyz = _vec(origin.get('xyz') if origin is not None else None, '0 0 0')
            rpy = _vec(origin.get('rpy') if origin is not None else None, '0 0 0')
            scale = _vec(mesh.get('scale'), '1 1 1')
            scene.stl('/hand-meshes/' + Path(mesh.attrib['filename']).name).move(*xyz).rotate(*rpy).scale(*scale).material(color, opacity=opacity)
        for joint in by_parent.get(name, []):
            override = corrections.get(joint.attrib['name']) if corrected else None
            origin = joint.find('origin')
            xyz = tuple(override['xyz']) if override else _vec(
                origin.get('xyz') if origin is not None else None, '0 0 0')
            rpy = tuple(override['rpy']) if override else _vec(
                origin.get('rpy') if origin is not None else None, '0 0 0')
            axis_element = joint.find('axis')
            axis = _vec(axis_element.get('xyz') if axis_element is not None else None, '1 0 0')
            child = joint.find('child').attrib['link']
            with scene.group().move(*xyz).rotate(*rpy):
                with scene.group() as rotation:
                    idx = indices.get(joint.attrib['name'])
                    if idx is not None:
                        motors.append((idx, axis, rotation))
                    key = sensor_by_link.get(child)
                    if with_tactile and key is not None:
                        add_marker(key, rotation, module_points(key[1]))
                    add_link(child, motors, color, opacity, corrected,
                             with_tactile and key is None)

    def create_models():
        if poses:
            return
        with scene:
            for name, color, opacity, corrected, key, always in (
                    ('original', '#d1d5db', 0.30, False, 'angle', True),
                    ('current', '#808080', 0.85, True, 'angle', True),
                    ('target', '#20b45a', 0.45, True, 'target', False)):
                motors = []
                with scene.group() as model:
                    for root in roots:
                        add_link(root, motors, color, opacity, corrected, name == 'current')
                if not always:
                    model.visible(False)
                poses.append((model, motors, key, always))
    scene.on('init', create_models)

    def refresh(snapshot):
        nonlocal shown
        if not shown:
            scene.run_method('resize')
            shown = True
        rows = snapshot['joints']
        valid = sum(row.get('angle') is not None and math.isfinite(row['angle']) for row in rows)
        tactile_state = snapshot.get('tactile') or {}
        status.set_text(
            f"Feedback {valid}/{len(rows)} · Tactile {tactile_state.get('level', 'NONE')}")
        control = snapshot.get('control_mode') in ('FEEDBACK', 'DIRECT')
        online = snapshot.get('communication') == 'Online'
        for model, motors, key, always in poses:
            model.visible(always or (control and online))
            for idx, axis, rotation in motors:
                value = rows[idx].get(key)
                if value is None or not math.isfinite(value):
                    continue
                x, y, z = axis
                norm = math.sqrt(x * x + y * y + z * z)
                if norm == 0:
                    continue
                x, y, z = x / norm, y / norm, z / norm
                angle = math.radians(value)
                c = math.cos(angle)
                s = math.sin(angle)
                t = 1 - c
                rotation.rotate_R([[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
                                   [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
                                   [t * x * z - s * y, t * y * z + s * x, t * z * z + c]])
        for module in tactile_state.get('modules') or []:
            key = (module['finger'], module['module'])
            marker = tactile.get(key)
            if marker is None:
                continue
            cloud, arrow, dot, surface, points, center = marker
            taxels = module.get('taxels')
            full = bool(module.get('valid') and tactile_state.get('level') == 'FULL' and taxels)
            if full:
                values = np.asarray(taxels, float)[..., 2].reshape(-1)
                try:
                    png = render_png('tip' if key[1] == 0 else 'pad', values)
                except Exception:
                    png = None
                if png is not None:
                    slot, previous, version = frame_slot[key]
                    if png != previous:
                        slot = 1 - slot
                        version += 1
                        frame_dir.joinpath(f'{key[0]}-{key[1]}-{slot}.png').write_bytes(png)
                        # 带版本号查询串，避免浏览器复用上一个槽位的缓存。
                        surface.set_url(f'/tactile-frames/{key[0]}-{key[1]}-{slot}.png?v={version}')
                        frame_slot[key] = [slot, png, version]
                if SHOW_POINTS:
                    cloud.set_points(points.tolist(), colormap(values))
            cloud.visible(SHOW_POINTS and full)
            surface.visible(HEATMAP and full)
            force = module.get('force') if module.get('valid') else None
            rotation, magnitude = (None, 0.0) if force is None else _rotation_from_force(force)
            if full or rotation is None or magnitude < ARROW_MIN:
                dot.visible(False)
            else:
                radius = min(0.006, 0.0015 + 0.00006 * magnitude)
                dot.move(*center).scale(radius, radius, radius).material('#ef4444')
                dot.visible(True)
            if rotation is None or magnitude < ARROW_MIN:
                arrow.visible(False)
                continue
            length = min(0.02, 0.005 + 0.00012 * magnitude)
            direction = np.asarray(rotation, float)[:, 1]
            arrow.move(*(center + direction * length / 2.0)) \
                 .rotate_R(rotation).scale(1.0, length, 1.0).material('#ef4444')
            arrow.visible(True)
    return refresh
