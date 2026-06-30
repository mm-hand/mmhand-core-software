"""Shared constants for hand tracking and robot retargeting."""

from pathlib import Path

from desktop.paths import DEFAULT_URDF_PATH as DESKTOP_DEFAULT_URDF_PATH

PACKAGE_DIR = Path(__file__).resolve().parent
POSTURE_DIR = PACKAGE_DIR.parent

MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/"
    "hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task"
)

DEFAULT_MODEL_PATH = POSTURE_DIR / "models" / "hand_landmarker.task"
DEFAULT_URDF_PATH = DESKTOP_DEFAULT_URDF_PATH

# 官方 21 个手部关键点连线拓扑（用于可视化骨架）。
HAND_CONNECTIONS = (
    (0, 1),
    (1, 2),
    (2, 3),
    (3, 4),
    (0, 5),
    (5, 6),
    (6, 7),
    (7, 8),
    (5, 9),
    (9, 10),
    (10, 11),
    (11, 12),
    (9, 13),
    (13, 14),
    (14, 15),
    (15, 16),
    (13, 17),
    (0, 17),
    (17, 18),
    (18, 19),
    (19, 20),
)

# MediaPipe 手指关键点映射（0: wrist, 1-4:thumb, 5-8:index, ...）。
HUMAN_FINGER_LANDMARKS = {
    "index": (5, 6, 7, 8),
    "middle": (9, 10, 11, 12),
    "ring": (13, 14, 15, 16),
    "pinky": (17, 18, 19, 20),
    "thumb": (1, 2, 3, 4),
}

# retarget 锚点默认用 MCP，拇指用 landmark 2（thumb_mcp）更稳定。
HUMAN_FINGER_ANCHOR = {
    "index": 5,
    "middle": 9,
    "ring": 13,
    "pinky": 17,
    "thumb": 2,
}

# URDF 手指编号映射。
ROBOT_FINGER_PREFIX = {
    "index": "1",
    "middle": "2",
    "ring": "3",
    "pinky": "4",
    "thumb": "5",
}
