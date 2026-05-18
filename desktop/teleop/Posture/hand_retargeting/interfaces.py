"""Output and integration interfaces."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import threading
from typing import Callable
import time


@dataclass(frozen=True)
class JointAngleFrame:
    """一帧目标关节角数据。"""

    seq_id: int
    timestamp_ms: int
    joint_names: tuple[str, ...]
    joint_angles: tuple[float, ...]


class JointAngleStream:
    """线程安全的连续关节角数据链。"""

    def __init__(self, maxlen: int = 2048) -> None:
        self._buffer: deque[JointAngleFrame] = deque(maxlen=maxlen)
        self._next_seq_id = 1
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._callbacks: list[Callable[[JointAngleFrame], None]] = []

    def publish(self, timestamp_ms: int, joint_names: list[str], joint_angles: list[float]) -> JointAngleFrame:
        """发布一帧目标角度，并唤醒等待中的读取方。"""
        if len(joint_names) != len(joint_angles):
            raise ValueError(
                f"joint_names/joint_angles length mismatch: {len(joint_names)} != {len(joint_angles)}"
            )

        frame = JointAngleFrame(
            seq_id=0,
            timestamp_ms=timestamp_ms,
            joint_names=tuple(joint_names),
            joint_angles=tuple(joint_angles),
        )

        with self._cond:
            frame = JointAngleFrame(
                seq_id=self._next_seq_id,
                timestamp_ms=frame.timestamp_ms,
                joint_names=frame.joint_names,
                joint_angles=frame.joint_angles,
            )
            self._next_seq_id += 1
            self._buffer.append(frame)
            self._cond.notify_all()

        callbacks = self._callbacks.copy()
        for callback in callbacks:
            try:
                callback(frame)
            except Exception:
                # 回调异常不影响主链路。
                pass

        return frame

    def get_latest(self) -> JointAngleFrame | None:
        """获取最新一帧，不阻塞。"""
        with self._lock:
            if not self._buffer:
                return None
            return self._buffer[-1]

    def get_since(self, last_seq_id: int) -> list[JointAngleFrame]:
        """增量读取：返回 seq_id > last_seq_id 的所有帧。"""
        with self._lock:
            return [frame for frame in self._buffer if frame.seq_id > last_seq_id]

    def wait_next(self, last_seq_id: int, timeout: float | None = None) -> JointAngleFrame | None:
        """阻塞等待下一帧（seq_id > last_seq_id）。"""
        with self._cond:
            for frame in self._buffer:
                if frame.seq_id > last_seq_id:
                    return frame

            notified = self._cond.wait(timeout=timeout)
            if not notified:
                return None

            for frame in self._buffer:
                if frame.seq_id > last_seq_id:
                    return frame
            return None

    def register_callback(self, callback: Callable[[JointAngleFrame], None]) -> None:
        """注册实时回调，收到新帧立即触发。"""
        with self._lock:
            self._callbacks.append(callback)

    def unregister_callback(self, callback: Callable[[JointAngleFrame], None]) -> bool:
        """注销已注册回调，返回是否成功删除。"""
        with self._lock:
            if callback in self._callbacks:
                self._callbacks.remove(callback)
                return True
        return False

    def clear(self) -> None:
        """清空缓冲区（不重置 seq）。"""
        with self._lock:
            self._buffer.clear()

    def get_status(self) -> dict[str, int]:
        """返回流状态，便于监控与调试。"""
        with self._lock:
            latest_seq = self._buffer[-1].seq_id if self._buffer else 0
            return {
                "buffer_size": len(self._buffer),
                "latest_seq_id": latest_seq,
                "next_seq_id": self._next_seq_id,
                "callback_count": len(self._callbacks),
            }


# 全局连续数据链：下游程序直接读取这个流即可。
JOINT_ANGLE_STREAM = JointAngleStream()


def get_joint_angle_stream() -> JointAngleStream:
    """对外暴露连续角度流对象。"""
    return JOINT_ANGLE_STREAM


def print_robot_joint_angles(timestamp_ms: int, joint_angles: dict[str, float]) -> None:
    """默认打印 21 个机械手关节角（单位：rad）。"""
    print(f"[{timestamp_ms}] Robot target joint angles (rad):")
    for name, value in joint_angles.items():
        print(f"  {name:>4}: {value:+.4f}")


# ===============================
# EXTERNAL INTERFACE (明显留口)
# ===============================
def on_target_joint_sequence(
    timestamp_ms: int,
    joint_names: list[str],
    joint_angles: list[float],
) -> None:
    """外部回传接口：连续发布目标关节角序列，供下游程序实时读取。"""
    JOINT_ANGLE_STREAM.publish(
        timestamp_ms=timestamp_ms,
        joint_names=joint_names,
        joint_angles=joint_angles,
    )


def _parse_args() -> argparse.Namespace:
    from .constants import DEFAULT_MODEL_PATH, DEFAULT_URDF_PATH

    parser = argparse.ArgumentParser(
        description="Interface entrypoint: webcam hand keypoints + joint-angle stream print."
    )
    parser.add_argument("--camera-id", type=int, default=0, help="Camera index.")
    parser.add_argument("--num-hands", type=int, default=1, help="Maximum number of hands.")
    parser.add_argument(
        "--model-path",
        default=DEFAULT_MODEL_PATH,
        help="Path to hand_landmarker.task model file.",
    )
    parser.add_argument(
        "--urdf-path",
        default=DEFAULT_URDF_PATH,
        help="Path to robot hand URDF file.",
    )
    parser.add_argument(
        "--min-hand-detection-confidence",
        type=float,
        default=0.5,
        help="Min confidence for palm detection.",
    )
    parser.add_argument(
        "--min-hand-presence-confidence",
        type=float,
        default=0.5,
        help="Min confidence for hand presence.",
    )
    parser.add_argument(
        "--min-tracking-confidence",
        type=float,
        default=0.5,
        help="Min confidence for hand tracking.",
    )
    return parser.parse_args()


def _print_frame(frame: JointAngleFrame) -> None:
    print(f"[stream] seq={frame.seq_id} ts={frame.timestamp_ms} joints={len(frame.joint_angles)}")
    for name, angle in zip(frame.joint_names, frame.joint_angles):
        print(f"  {name:>4}: {angle:+.4f}")


def main() -> int:
    args = _parse_args()

    try:
        import cv2
        import mediapipe as mp
        from pathlib import Path
    except ImportError as exc:
        print(
            "Missing dependencies. Install with:\n"
            "  pip install mediapipe opencv-python numpy requests"
        )
        raise SystemExit(1) from exc

    from .constants import MODEL_URL
    from .retargeting import HandRetargeter
    from .urdf_model import URDFRobotHandModel
    from .vision import (
        LatestResultStore,
        draw_landmarks,
        ensure_model_exists,
        extract_hand_points_for_retargeting,
    )

    ensure_model_exists(Path(args.model_path), MODEL_URL)

    try:
        robot_model = URDFRobotHandModel.from_urdf(Path(args.urdf_path))
    except Exception as exc:
        print(f"Failed to parse URDF model: {exc}")
        return 1
    retargeter = HandRetargeter(robot_model)

    BaseOptions = mp.tasks.BaseOptions
    HandLandmarker = mp.tasks.vision.HandLandmarker
    HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
    RunningMode = mp.tasks.vision.RunningMode

    result_store = LatestResultStore()

    def result_callback(result, output_image, timestamp_ms: int) -> None:
        del output_image
        result_store.update(result, timestamp_ms)

    options = HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=str(args.model_path)),
        running_mode=RunningMode.LIVE_STREAM,
        num_hands=args.num_hands,
        min_hand_detection_confidence=args.min_hand_detection_confidence,
        min_hand_presence_confidence=args.min_hand_presence_confidence,
        min_tracking_confidence=args.min_tracking_confidence,
        result_callback=result_callback,
    )

    cap = cv2.VideoCapture(args.camera_id)
    if not cap.isOpened():
        print(f"Error: failed to open camera with id {args.camera_id}.")
        return 1

    stream = get_joint_angle_stream()
    last_published_timestamp = -1
    last_read_seq = 0
    frame_count = 0
    last_fps_time = time.time()
    fps = 0.0

    with HandLandmarker.create_from_options(options) as landmarker:
        try:
            while True:
                ok, frame_bgr = cap.read()
                if not ok:
                    print("Warning: failed to read frame from camera.")
                    break

                frame_bgr = cv2.flip(frame_bgr, 1)
                frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)
                timestamp_ms = time.monotonic_ns() // 1_000_000
                landmarker.detect_async(mp_image, timestamp_ms)

                latest_result, latest_timestamp = result_store.snapshot()
                draw_landmarks(frame_bgr, latest_result, cv2)

                if latest_timestamp > 0 and latest_timestamp != last_published_timestamp:
                    points_3d = extract_hand_points_for_retargeting(latest_result)
                    if points_3d is not None and points_3d.shape == (21, 3):
                        joint_angles = retargeter.solve_joint_angles(points_3d)
                        joint_names = list(joint_angles.keys())
                        joint_values = [joint_angles[name] for name in joint_names]
                        on_target_joint_sequence(latest_timestamp, joint_names, joint_values)
                    last_published_timestamp = latest_timestamp

                new_frames = stream.get_since(last_read_seq)
                for f in new_frames:
                    _print_frame(f)
                    last_read_seq = f.seq_id

                frame_count += 1
                now = time.time()
                elapsed = now - last_fps_time
                if elapsed >= 1.0:
                    fps = frame_count / elapsed
                    frame_count = 0
                    last_fps_time = now

                cv2.putText(
                    frame_bgr,
                    f"FPS: {fps:.1f}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (50, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

                cv2.imshow("Interface Demo: Hand Keypoints + Joint Stream", frame_bgr)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    break
        finally:
            cap.release()
            cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
