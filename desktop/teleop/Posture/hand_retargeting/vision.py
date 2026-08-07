"""vision.py - MMHand Desktop 模块契约。

职责：封装 MediaPipe 检测和视觉诊断绘制。
线程：接收器可在 SDK/摄像头工作线程执行；纯映射和数学函数不创建线程。
输入：手套、骨架或视觉关键点，以及活动 URDF/标定配置。
输出：统一 J00-J20 目标、接收状态和可诊断质量信息。
状态所有权：接收器拥有最近输入帧；角度映射和模型配置按只读快照使用。
禁止依赖：禁止依赖 Qt/UI 或直接写串口，目标必须经过统一角度适配和后端门禁。
"""

from __future__ import annotations

import threading
from pathlib import Path

import numpy as np

from .constants import HAND_CONNECTIONS


class LatestResultStore:
    """线程安全地保存最新一帧的异步推理结果。"""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.result = None
        self.timestamp_ms = 0

    def update(self, result, timestamp_ms: int) -> None:
        with self._lock:
            self.result = result
            self.timestamp_ms = timestamp_ms

    def snapshot(self):
        with self._lock:
            return self.result, self.timestamp_ms


def ensure_model_exists(model_path: Path, model_url: str) -> None:
    """确保本地模型存在；不存在则自动下载官方 .task 文件。"""
    if model_path.exists():
        return

    print(f"Model not found at {model_path}. Downloading from official source...")
    model_path.parent.mkdir(parents=True, exist_ok=True)

    try:
        import requests
    except ImportError as exc:
        raise RuntimeError(
            "Missing dependency 'requests'. Install with: pip install requests"
        ) from exc

    # 先下载到临时文件，成功后原子替换，避免中断导致模型损坏。
    tmp_path = model_path.with_suffix(model_path.suffix + ".part")
    try:
        with requests.get(model_url, stream=True, timeout=60) as response:
            response.raise_for_status()
            with tmp_path.open("wb") as f:
                for chunk in response.iter_content(chunk_size=1024 * 1024):
                    if chunk:
                        f.write(chunk)
        tmp_path.replace(model_path)
    except Exception as exc:
        if tmp_path.exists():
            tmp_path.unlink(missing_ok=True)
        raise RuntimeError(f"Failed to download model from {model_url}: {exc}") from exc

    print(f"Model downloaded to: {model_path}")


def draw_landmarks(image, detection_result, cv2) -> None:
    """把检测结果绘制到图像：关键点、骨架、左右手标签。"""
    if detection_result is None:
        return

    image_height, image_width = image.shape[:2]

    hand_landmarks_list = detection_result.hand_landmarks
    handedness_list = detection_result.handedness

    for idx, hand_landmarks in enumerate(hand_landmarks_list):
        # 先画骨架线，便于后续关键点覆盖在上层更清晰。
        for start_idx, end_idx in HAND_CONNECTIONS:
            start = hand_landmarks[start_idx]
            end = hand_landmarks[end_idx]

            x1 = int(start.x * image_width)
            y1 = int(start.y * image_height)
            x2 = int(end.x * image_width)
            y2 = int(end.y * image_height)
            cv2.line(image, (x1, y1), (x2, y2), (80, 220, 80), 2)

        # 再画关键点。
        for lm in hand_landmarks:
            x = int(lm.x * image_width)
            y = int(lm.y * image_height)
            cv2.circle(image, (x, y), 4, (20, 120, 255), -1)

        # 在手腕附近绘制左右手与置信度标签。
        wrist = hand_landmarks[0]
        wx = int(wrist.x * image_width)
        wy = int(wrist.y * image_height)

        label = "Hand"
        if idx < len(handedness_list) and handedness_list[idx]:
            category = handedness_list[idx][0]
            label = f"{category.category_name} ({category.score:.2f})"

        cv2.putText(
            image,
            label,
            (wx + 10, max(20, wy - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )


def print_landmarks(detection_result, image_shape, timestamp_ms: int) -> None:
    """在终端打印每只手的 21 个关键点坐标（像素坐标 + 归一化坐标）。"""
    if detection_result is None or not detection_result.hand_landmarks:
        return

    image_height, image_width = image_shape[:2]
    handedness_list = detection_result.handedness

    print(f"\n[{timestamp_ms}] Detected {len(detection_result.hand_landmarks)} hand(s)")
    for hand_idx, hand_landmarks in enumerate(detection_result.hand_landmarks):
        label = "Hand"
        if hand_idx < len(handedness_list) and handedness_list[hand_idx]:
            category = handedness_list[hand_idx][0]
            label = f"{category.category_name} ({category.score:.2f})"

        print(f"  Hand {hand_idx + 1}: {label}")
        for i, lm in enumerate(hand_landmarks):
            px = int(lm.x * image_width)
            py = int(lm.y * image_height)
            print(
                f"    #{i:02d} pixel=({px:4d}, {py:4d}) "
                f"norm=({lm.x:.4f}, {lm.y:.4f}, {lm.z:.4f})"
            )


def extract_hand_points_for_retargeting(detection_result) -> np.ndarray | None:
    """优先使用 world landmarks（米），退化到归一化 landmarks。"""
    if detection_result is None:
        return None

    world_sets = getattr(detection_result, "hand_world_landmarks", None)
    if world_sets and len(world_sets) > 0:
        hand = world_sets[0]
        return np.array([[lm.x, lm.y, lm.z] for lm in hand], dtype=float)

    if detection_result.hand_landmarks and len(detection_result.hand_landmarks) > 0:
        hand = detection_result.hand_landmarks[0]
        return np.array([[lm.x, lm.y, lm.z] for lm in hand], dtype=float)

    return None
