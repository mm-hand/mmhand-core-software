"""constants.py - MMHand Desktop 模块契约。

职责：集中保存视觉手部重定向的稳定常量和默认资源路径。
线程：接收器可在 SDK/摄像头工作线程执行；纯映射和数学函数不创建线程。
输入：手套、骨架或视觉关键点，以及活动 URDF/标定配置。
输出：统一 J00-J20 目标、接收状态和可诊断质量信息。
状态所有权：接收器拥有最近输入帧；角度映射和模型配置按只读快照使用。
禁止依赖：禁止依赖 Qt/UI 或直接写串口，目标必须经过统一角度适配和后端门禁。
"""

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
