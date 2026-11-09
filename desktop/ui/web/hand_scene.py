"""hand_scene.py - shared read-only URDF hand scene for the Hand and Contact pages.

职责：解析 mmhand.urdf、注册网格静态路由、按模型规格建手、更新关节原点和角度。
线程：NiceGUI 页面事件循环；只读 URDF 资源和场景对象。
输入：scene 元素、每个模型的颜色/透明度/原点修正/可选 link 回调。
输出：可直接 apply 角度的模型组。
状态所有权：只拥有本场景的模型组和显示缓存。
禁止依赖：禁止直接操作协议、串口和控制器。
"""
from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET

from nicegui import app, ui

from desktop.core.robot_model import manifest as robot_model_manifest

ROOT = Path(__file__).resolve().parents[2] / 'resources'
MODEL = ROOT / 'mmhand_urdf'
MESH_ROUTE = '/hand-meshes'
# mmhand.urdf puts the fingers along +z, so the camera looks down +y at a point
# level with the palm and keeps up exactly on +z: any lean in `up` rolls the
# whole hand over. The distance is fixed by the 0.80 vertical fill in the test.
CAMERA = dict(x=-0.019, y=-0.4555, z=0.0917, look_at_x=-0.019,
              look_at_y=-0.0536, look_at_z=0.0917,
              up_x=0.0, up_y=0.0, up_z=1.0)
FOV = 42
SCENE = dict(width=1200, height=620)


def _vec(text, default):
    return tuple(float(value) for value in (text or default).split())


def register_meshes():
    try:
        app.add_static_files(MESH_ROUTE, str(MODEL / 'meshes'))
    except RuntimeError:
        pass


def make_camera():
    camera = ui.scene.perspective_camera(fov=FOV, near=0.01)
    for key, value in CAMERA.items():
        setattr(camera, key, value)
    return camera


def _tree():
    urdf = ET.parse(MODEL / 'urdf' / 'mmhand.urdf').getroot()
    indices = {joint['urdf_joint_name']: joint['joint_index']
               for joint in robot_model_manifest()['joints']}
    links = {element.attrib['name']: element for element in urdf.findall('link')}
    joints = urdf.findall('joint')
    children = {joint.find('child').attrib['link'] for joint in joints}
    roots = set(links) - children
    by_parent = {}
    for joint in joints:
        by_parent.setdefault(joint.find('parent').attrib['link'], []).append(joint)
    return links, roots, by_parent, indices


class HandScene:
    """One scene, any number of hand models that share the same URDF tree."""

    def __init__(self, scene):
        self.scene = scene
        self.links, self.roots, self.by_parent, self.indices = _tree()
        self.models = {}
        self.meshes = {}

    def add_model(self, name, color, opacity, origins=None, on_link=None):
        motors, origin_groups = [], {}
        with self.scene:
            with self.scene.group() as group:
                for root in self.roots:
                    self._add_link(root, color, opacity, origins or {},
                                   motors, origin_groups, on_link)
        self.models[name] = {'group': group, 'motors': motors,
                             'origin_groups': origin_groups,
                             'origins': dict(origins or {}),
                             'color': color, 'opacity': opacity}

    def _add_link(self, name, color, opacity, origins, motors, origin_groups,
                  on_link):
        link = self.links[name]
        for visual in link.findall('visual'):
            mesh = visual.find('geometry/mesh')
            if mesh is None:
                continue
            origin = visual.find('origin')
            xyz = _vec(origin.get('xyz') if origin is not None else None, '0 0 0')
            rpy = _vec(origin.get('rpy') if origin is not None else None, '0 0 0')
            scale = _vec(mesh.get('scale'), '1 1 1')
            item = self.scene.stl(MESH_ROUTE + '/' + Path(mesh.attrib['filename']).name) \
                .move(*xyz).rotate(*rpy).scale(*scale) \
                .material(color, opacity=opacity)
            self.meshes.setdefault(name, []).append(item)
        for joint in self.by_parent.get(name, []):
            origin = joint.find('origin')
            xyz = _vec(origin.get('xyz') if origin is not None else None, '0 0 0')
            rpy = _vec(origin.get('rpy') if origin is not None else None, '0 0 0')
            override = origins.get(joint.attrib['name'])
            if override:
                xyz, rpy = tuple(override['xyz']), tuple(override['rpy'])
            axis_node = joint.find('axis')
            axis = _vec(axis_node.get('xyz') if axis_node is not None else None,
                        '1 0 0')
            child = joint.find('child').attrib['link']
            with self.scene.group().move(*xyz).rotate(*rpy) as origin_group:
                origin_groups[joint.attrib['name']] = (origin_group, xyz, rpy)
                with self.scene.group() as rotation:
                    index = self.indices.get(joint.attrib['name'])
                    if index is not None:
                        motors.append((index, axis, rotation))
                    if on_link is not None:
                        on_link(child, rotation)
                    self._add_link(child, color, opacity, origins, motors,
                                   origin_groups, on_link)

    def set_link_color(self, name, link_names, color):
        """Recolor the meshes of specific URDF links on one model."""
        model = self.models[name]
        for link in link_names:
            for item in self.meshes.get(link, ()):
                item.material(color, opacity=model['opacity'])

    def set_origins(self, name, origins):
        """原地更新关节原点；返回是否真的变了。"""
        model = self.models[name]
        origins = origins or {}
        if origins == model['origins']:
            return False
        for joint_name, (group, xyz, rpy) in model['origin_groups'].items():
            override = origins.get(joint_name)
            if override:
                group.move(*override['xyz']).rotate(*override['rpy'])
            else:
                group.move(*xyz).rotate(*rpy)
        model['origins'] = dict(origins)
        return True

    def apply(self, name, angles):
        for index, axis, rotation in self.models[name]['motors']:
            value = angles[index] if index < len(angles) else None
            if value is None:
                continue
            try:
                value = float(value)
            except (TypeError, ValueError):
                continue
            if not math.isfinite(value):
                continue
            x, y, z = axis
            norm = math.sqrt(x * x + y * y + z * z)
            if norm == 0:
                continue
            x, y, z = x / norm, y / norm, z / norm
            angle = math.radians(value)
            c, s = math.cos(angle), math.sin(angle)
            t = 1 - c
            rotation.rotate_R([
                [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
                [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
                [t * x * z - s * y, t * y * z + s * x, t * z * z + c]])

    def show(self, name, visible):
        self.models[name]['group'].visible(visible)
