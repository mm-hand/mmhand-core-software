"""Plot > Hand camera must frame the URDF hand upright and centred; no nicegui import."""
import ast
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
HAND = ROOT / 'desktop' / 'ui' / 'web' / 'hand_scene.py'
MODEL = ROOT / 'desktop' / 'resources' / 'mmhand_urdf'


def _constant(name):
    for node in ast.parse(HAND.read_text(encoding='utf-8')).body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], 'id', '') == name:
            if isinstance(node.value, ast.Call):
                return {kw.arg: ast.literal_eval(kw.value) for kw in node.value.keywords}
            return ast.literal_eval(node.value)
    raise AssertionError(f'{name} missing from hand_scene.py')


def _origin(element):
    """Rigid transform of an <origin> element (missing attributes mean identity)."""
    xyz = [float(v) for v in (element.get('xyz') if element is not None else '0 0 0').split()]
    rpy = [float(v) for v in (element.get('rpy') if element is not None else '0 0 0').split()]
    cr, sr, cp, sp, cy, sy = (np.cos(rpy[0]), np.sin(rpy[0]), np.cos(rpy[1]),
                              np.sin(rpy[1]), np.cos(rpy[2]), np.sin(rpy[2]))
    rotation = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                         [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                         [-sp, cp * sr, cp * cr]])
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = xyz
    return matrix


def _stl_vertices(path):
    raw = path.read_bytes()
    count = struct.unpack('<I', raw[80:84])[0]
    data = np.frombuffer(raw[84:84 + count * 50], dtype=np.uint8).reshape(count, 50)
    vertices = np.frombuffer(data[:, 12:48].tobytes(), dtype='<f4').reshape(-1, 3)
    return vertices.astype(float)[::25]


def _hand_points():
    urdf = ET.parse(MODEL / 'urdf' / 'mmhand.urdf').getroot()
    parent_joint = {j.find('child').attrib['link']: j for j in urdf.findall('joint')}
    points = []
    for link in urdf.findall('link'):
        world = np.eye(4)
        link_name = link.attrib['name']
        while link_name in parent_joint:
            joint = parent_joint[link_name]
            world = _origin(joint.find('origin')) @ world
            link_name = joint.find('parent').attrib['link']
        for visual in link.findall('visual'):
            mesh = visual.find('geometry/mesh')
            path = MODEL / 'meshes' / Path(mesh.get('filename')).name
            scale = np.array([float(v) for v in (mesh.get('scale') or '1 1 1').split()])
            vertices = _stl_vertices(path) * scale
            homogeneous = np.c_[vertices, np.ones(len(vertices))]
            points.append((world @ _origin(visual.find('origin')) @ homogeneous.T).T[:, :3])
    return np.vstack(points)


def _screen(camera, points, width, height):
    position = np.array([camera['x'], camera['y'], camera['z']])
    look_at = np.array([camera['look_at_x'], camera['look_at_y'], camera['look_at_z']])
    forward = look_at - position
    forward /= np.linalg.norm(forward)
    up = np.array([camera['up_x'], camera['up_y'], camera['up_z']])
    assert abs(float(up @ forward)) < 1e-3, 'up vector must be perpendicular to the view direction'
    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    vertical = np.cross(right, forward)
    offset = points - position
    depth = offset @ forward
    assert depth.min() > 0, 'the hand must sit in front of the camera'
    tan = np.tan(np.radians(_constant('FOV') / 2))
    return (offset @ right) / depth / (tan * width / height), (offset @ vertical) / depth / tan


def _link_positions():
    urdf = ET.parse(MODEL / 'urdf' / 'mmhand.urdf').getroot()
    parent_joint = {j.find('child').attrib['link']: j for j in urdf.findall('joint')}
    positions = {}
    for name in ('link_finger_tip_2', 'link_finger_tip_5'):
        world = np.eye(4)
        link_name = name
        while link_name in parent_joint:
            joint = parent_joint[link_name]
            world = _origin(joint.find('origin')) @ world
            link_name = joint.find('parent').attrib['link']
        positions[name] = world[:3, 3]
    return positions


def test_camera_frames_the_hand_upright_and_centred():
    camera, scene = _constant('CAMERA'), _constant('SCENE')
    points = _hand_points()
    width, height = scene['width'], scene['height']
    x, y = _screen(camera, points, width, height)
    assert np.abs(x).max() < 1.0, 'hand leaves the canvas horizontally'
    assert np.abs(y).max() < 1.0, 'hand leaves the canvas vertically'
    # Fit alone is not enough: the pre-6f9c681 pose aimed at the old model's axes
    # kept the hand inside the frustum while shoving it into a corner.
    centre = ((x.min() + x.max()) / 2, (y.min() + y.max()) / 2)
    assert max(abs(c) for c in centre) < 0.25, f'hand is off-centre at {centre}'
    # A tall, thin hand must fill the canvas height instead of floating small.
    assert (y.max() - y.min()) / 2 > 0.75, 'hand is too small in the canvas'
    # The fingers leave the palm along the model's +z, so that axis must land on
    # the screen's vertical. A rolled camera leans it (the tuned pose leaned it
    # 1.03 deg because `up` carried leftover tilt).
    axis = _screen(camera, np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.01]]), width, height)
    tilt = np.degrees(np.arctan2(axis[0][1] - axis[0][0], axis[1][1] - axis[1][0]))
    assert abs(float(tilt)) < 0.5, f'hand is tilted {tilt:.2f} deg from vertical'
