"""Overlaid live URDF poses using the local STL assets.

职责：把只读关节反馈和目标角叠加到 URDF 三维场景。
线程：NiceGUI 页面事件循环；仅消费快照，不创建任务。
输入：后端同步快照。
输出：场景位姿和静态说明。
状态所有权：不拥有设备状态，只缓存已显示的位姿。
禁止依赖：禁止直接操作协议、串口和控制器。
"""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from nicegui import app, ui

ROOT = Path(__file__).resolve().parents[2] / 'resources'
MODEL = ROOT / 'hku_hand_v2_urdf'

def _vec(text, default):
    return tuple(float(v) for v in (text or default).split())

def build(hand):
    try:
        app.add_static_files('/hand-meshes', str(MODEL / 'meshes'))
    except RuntimeError:
        pass
    urdf = ET.parse(MODEL / 'urdf' / 'hand.urdf').getroot()
    mapping = json.loads((ROOT / 'config' / 'robot_model.json').read_text())['manifest']['joints']
    indices = {j['urdf_joint_name']: j['joint_index'] for j in mapping}
    links = {e.attrib['name']: e for e in urdf.findall('link')}
    joints = urdf.findall('joint')
    children = {j.find('child').attrib['link'] for j in joints}
    roots = set(links) - children
    by_parent = {}
    for j in joints:
        by_parent.setdefault(j.find('parent').attrib['link'], []).append(j)
    with ui.row().classes('page-toolbar'):
        ui.label('Green: target · Red: measured').classes('text-sm')
        reset = ui.button('Reset view')
    status = ui.label('Waiting for feedback')
    camera = ui.scene.perspective_camera(near=0.01)
    camera.x, camera.y, camera.z = 0.16, -0.22, 0.19
    scene = ui.scene(width=800, height=550, grid=False, camera=camera).classes('w-full')
    shown = False
    reset.on_click(lambda: scene.move_camera(
        x=0.16, y=-0.22, z=0.19, look_at_x=0, look_at_y=0, look_at_z=0))
    poses = []
    def add_link(name, motors, color):
        link = links[name]
        for visual in link.findall('visual'):
            mesh = visual.find('geometry/mesh')
            if mesh is None:
                continue
            origin = visual.find('origin')
            xyz = _vec(origin.get('xyz') if origin is not None else None, '0 0 0')
            rpy = _vec(origin.get('rpy') if origin is not None else None, '0 0 0')
            scale = _vec(mesh.get('scale'), '1 1 1')
            scene.stl('/hand-meshes/' + Path(mesh.attrib['filename']).name).move(*xyz).rotate(*rpy).scale(*scale).material(color, opacity=0.5)
        for joint in by_parent.get(name, []):
            origin = joint.find('origin')
            xyz = _vec(origin.get('xyz') if origin is not None else None, '0 0 0')
            rpy = _vec(origin.get('rpy') if origin is not None else None, '0 0 0')
            axis_element = joint.find('axis')
            axis = _vec(axis_element.get('xyz') if axis_element is not None else None, '1 0 0')
            with scene.group().move(*xyz).rotate(*rpy):
                with scene.group() as rotation:
                    idx = indices.get(joint.attrib['name'])
                    if idx is not None: motors.append((idx, axis, rotation))
                    add_link(joint.find('child').attrib['link'], motors, color)
    with scene:
        for color in ('#20b45a', '#e53935'):
            motors = []
            with scene.group():
                for root in roots: add_link(root, motors, color)
            poses.append(motors)
    def refresh(snapshot):
        nonlocal shown
        if not shown:
            # Plot panels are kept alive while hidden, so the scene's initial
            # resize can see a zero-width tab. Re-measure after Hand is shown.
            scene.run_method('resize')
            shown = True
        rows = snapshot['joints']
        valid = sum(row.get('angle') is not None for row in rows)
        status.set_text(f'Live feedback · {valid}/{len(rows)} joints' if valid == len(rows)
                        else f'Partial feedback · {valid}/{len(rows)} joints')
        for motors, key in zip(poses, ('target', 'angle')):
            for idx, axis, rotation in motors:
                value = rows[idx].get(key)
                if key == 'target' and snapshot.get('communication') != 'Online':
                    value = None
                rotation.visible(value is not None and math.isfinite(value))
                if value is not None and math.isfinite(value):
                    # URDF axes are unit vectors; Rodrigues rotation handles arbitrary axes.
                    x,y,z = axis; norm = math.sqrt(x*x+y*y+z*z)
                    if norm == 0: continue
                    x,y,z=x/norm,y/norm,z/norm
                    angle=math.radians(value); c=math.cos(angle); s=math.sin(angle); t=1-c
                    rotation.rotate_R([[t*x*x+c,t*x*y-s*z,t*x*z+s*y], [t*x*y+s*z,t*y*y+c,t*y*z-s*x], [t*x*z-s*y,t*y*z+s*x,t*z*z+c]])
    return refresh
