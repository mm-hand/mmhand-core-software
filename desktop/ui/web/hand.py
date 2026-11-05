"""Two URDF poses: latest contact-corrected current pose and the target.

职责：把只读关节反馈/目标角画到 URDF 三维场景，并叠加触觉点云/热力图/合力箭头。
线程：NiceGUI 页面事件循环；仅消费快照，不创建任务。
输入：后端同步快照。
输出：灰色当前手、红色目标手、触觉显示。
状态所有权：不拥有设备状态，只缓存已显示的位姿和场景对象。
禁止依赖：禁止直接操作协议、串口和控制器。
"""
import math
from pathlib import Path
import tempfile

import numpy as np
from nicegui import app, ui

from desktop.core.contact_calibration import active_origins
from desktop.core.tactile_geometry import SENSOR_LINKS, module_points
from desktop.core.tactile_heatmap import colormap, grid_coords, render_png

from .hand_scene import CAMERA, SCENE, HandScene, make_camera, register_meshes

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
    register_meshes()
    sensor_by_link = {link: key for key, link in SENSOR_LINKS.items()}
    frame_dir = _frame_dir()
    frame_slot = {}

    with ui.row().classes('page-toolbar'):
        ui.label('Gray: latest corrected current pose · Red: target').classes('text-sm')
        reset = ui.button('Reset view')
    status = ui.label('Waiting for feedback')
    scene = ui.scene(grid=False, camera=make_camera(), background_color='#ffffff',
                     **SCENE).classes('w-full')
    view = HandScene(scene)
    shown = False
    reset.on_click(lambda: scene.move_camera(**CAMERA))
    tactile = {}

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

    def add_tactile(link_name, rotation):
        key = sensor_by_link.get(link_name)
        if key is not None:
            add_marker(key, rotation, module_points(key[1]))

    def create_models():
        origins = active_origins()
        view.add_model('current', '#808080', 0.85, origins, on_link=add_tactile)
        view.add_model('target', '#ef4444', 0.45, origins)
        view.show('target', False)
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
        if view.models:
            origins = active_origins()
            view.set_origins('current', origins)
            view.set_origins('target', origins)
            view.apply('current', [row.get('angle') for row in rows])
            view.apply('target', [row.get('target') for row in rows])
            view.show('current', True)
            view.show('target', bool(control and online))
        for module in tactile_state.get('modules') or []:
            marker = tactile.get((module['finger'], module['module']))
            if marker is None:
                continue
            cloud, arrow, dot, surface, points, center = marker
            taxels = module.get('taxels')
            full = bool(module.get('valid') and tactile_state.get('level') == 'FULL' and taxels)
            if full:
                values = np.asarray(taxels, float)[..., 2].reshape(-1)
                try:
                    png = render_png('tip' if module['module'] == 0 else 'pad', values)
                except Exception:
                    png = None
                if png is not None:
                    slot, previous, version = frame_slot[(module['finger'], module['module'])]
                    if png != previous:
                        slot = 1 - slot
                        version += 1
                        frame_dir.joinpath(
                            f"{module['finger']}-{module['module']}-{slot}.png").write_bytes(png)
                        # 带版本号查询串，避免浏览器复用上一个槽位的缓存。
                        surface.set_url(
                            f"/tactile-frames/{module['finger']}-{module['module']}-{slot}.png?v={version}")
                        frame_slot[(module['finger'], module['module'])] = [slot, png, version]
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
