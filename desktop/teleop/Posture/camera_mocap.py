#!/usr/bin/env python3
"""Realtime hand keypoint capture, URDF retargeting, and joint angle output."""

from __future__ import annotations

import argparse
import math
import platform
import sys
import threading
import time
from pathlib import Path
from typing import Callable

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
PROJECT_ROOT = SCRIPT_DIR.parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from hand_retargeting.constants import DEFAULT_MODEL_PATH, DEFAULT_URDF_PATH, MODEL_URL
from desktop.teleop.joint_target_adapter import named_urdf_radians_to_joint_targets


StatusCallback = Callable[[str], None]
FrameCallback = Callable[[object, float], None]
JointCallback = Callable[[int, list[str], list[float], list[float]], None]

DEFAULT_CAMERA_WIDTH = 640
DEFAULT_CAMERA_HEIGHT = 480
DEFAULT_CAMERA_FPS = 30.0
DEFAULT_INFERENCE_FPS = 20.0
DEFAULT_RETARGET_FPS = 12.0
DEFAULT_IK_MAX_ITERS = 12


def _emit_status(callback: StatusCallback | None, message: str) -> None:
    if callback is not None:
        callback(message)


def _interval_from_fps(fps: float) -> float:
    fps = float(fps)
    return 0.0 if fps <= 0.0 else 1.0 / fps


def open_camera(
    camera_id: int,
    cv2,
    width: int = DEFAULT_CAMERA_WIDTH,
    height: int = DEFAULT_CAMERA_HEIGHT,
    fps: float = DEFAULT_CAMERA_FPS,
):
    """Open a camera with Windows-friendly backend fallbacks."""
    backends = [
        getattr(cv2, "CAP_DSHOW", 700),
        getattr(cv2, "CAP_MSMF", 1400),
        getattr(cv2, "CAP_ANY", 0),
    ]
    if platform.system().lower() != "windows":
        backends = [getattr(cv2, "CAP_ANY", 0)]

    for backend in backends:
        cap = cv2.VideoCapture(camera_id, backend)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_FPS, float(fps))
            return cap
        cap.release()

    return cv2.VideoCapture(camera_id)


def joint_angle_dict_to_encoder_degrees(joint_angles: dict[str, float]) -> list[float]:
    """Map semantic or legacy URDF names to canonical J00-J20 URDF degrees."""
    adaptation = named_urdf_radians_to_joint_targets(joint_angles)
    if joint_angles and not adaptation.mapped_indices:
        unknown = ", ".join(adaptation.unknown_names[:4])
        raise ValueError(f"URDF retarget produced no recognized joint names: {unknown}")
    return list(adaptation.urdf_targets_deg)


def run_mocap_loop(
    *,
    camera_id: int = 0,
    num_hands: int = 1,
    min_hand_detection_confidence: float = 0.5,
    min_hand_presence_confidence: float = 0.5,
    min_tracking_confidence: float = 0.5,
    model_path: Path = DEFAULT_MODEL_PATH,
    urdf_path: Path = DEFAULT_URDF_PATH,
    camera_width: int = DEFAULT_CAMERA_WIDTH,
    camera_height: int = DEFAULT_CAMERA_HEIGHT,
    camera_fps: float = DEFAULT_CAMERA_FPS,
    inference_fps: float = DEFAULT_INFERENCE_FPS,
    retarget_fps: float = DEFAULT_RETARGET_FPS,
    ik_max_iters: int = DEFAULT_IK_MAX_ITERS,
    retarget_enabled: bool = True,
    overlay_enabled: bool = True,
    profile: bool = False,
    print_landmark_output: bool = False,
    print_joint_output: bool = False,
    show_window: bool = False,
    stop_event: threading.Event | None = None,
    status_callback: StatusCallback | None = None,
    frame_callback: FrameCallback | None = None,
    joint_callback: JointCallback | None = None,
) -> int:
    """Run webcam hand tracking and retargeting until stopped."""
    try:
        import cv2
        import mediapipe as mp
        import numpy as np  # noqa: F401
    except ImportError as exc:
        _emit_status(
            status_callback,
            "Missing dependencies. Install with: pip install mediapipe opencv-python numpy requests",
        )
        raise RuntimeError("Missing Posture mocap dependencies") from exc

    from hand_retargeting.interfaces import on_target_joint_sequence, print_robot_joint_angles
    from hand_retargeting.retargeting import HandRetargeter
    from hand_retargeting.urdf_model import URDFRobotHandModel
    from hand_retargeting.vision import (
        LatestResultStore,
        draw_landmarks,
        ensure_model_exists,
        extract_hand_points_for_retargeting,
        print_landmarks,
    )

    model_path = Path(model_path)
    urdf_path = Path(urdf_path)
    _emit_status(status_callback, "Loading MediaPipe hand model...")
    ensure_model_exists(model_path, MODEL_URL)

    retargeter = None
    if retarget_enabled:
        try:
            robot_model = URDFRobotHandModel.from_urdf(urdf_path)
        except Exception as exc:
            _emit_status(status_callback, f"Failed to parse URDF model: {exc}")
            return 1
        retargeter = HandRetargeter(robot_model)

    BaseOptions = mp.tasks.BaseOptions
    HandLandmarker = mp.tasks.vision.HandLandmarker
    HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
    RunningMode = mp.tasks.vision.RunningMode

    result_store = LatestResultStore()
    inference_lock = threading.Lock()
    inference_in_flight = False
    last_inference_submit_time = 0.0
    last_retarget_time = 0.0
    last_processed_timestamp = -1
    inference_interval = _interval_from_fps(inference_fps)
    retarget_interval = _interval_from_fps(retarget_fps)
    display_frames = 0
    inference_submits = 0
    inference_callbacks = 0
    inference_skips = 0
    retarget_frames = 0
    retarget_total_ms = 0.0
    last_profile_time = time.perf_counter()
    last_fps_time = time.perf_counter()
    fps = 0.0

    def result_callback(result, output_image, timestamp_ms: int) -> None:
        nonlocal inference_callbacks, inference_in_flight
        del output_image
        result_store.update(result, timestamp_ms)
        inference_callbacks += 1
        with inference_lock:
            inference_in_flight = False

    options = HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=str(model_path)),
        running_mode=RunningMode.LIVE_STREAM,
        num_hands=num_hands,
        min_hand_detection_confidence=min_hand_detection_confidence,
        min_hand_presence_confidence=min_hand_presence_confidence,
        min_tracking_confidence=min_tracking_confidence,
        result_callback=result_callback,
    )

    cap = open_camera(camera_id, cv2, width=int(camera_width), height=int(camera_height), fps=float(camera_fps))
    if not cap.isOpened():
        _emit_status(status_callback, f"Error: failed to open camera with id {camera_id}.")
        return 1

    stop_event = stop_event or threading.Event()
    _emit_status(status_callback, f"Camera {camera_id} started")

    with HandLandmarker.create_from_options(options) as landmarker:
        try:
            while not stop_event.is_set():
                ok, frame_bgr = cap.read()
                if not ok:
                    _emit_status(status_callback, "Warning: failed to read frame from camera.")
                    break

                now = time.perf_counter()
                display_frames += 1
                frame_bgr = cv2.flip(frame_bgr, 1)

                should_submit = inference_interval <= 0.0 or (now - last_inference_submit_time) >= inference_interval
                if should_submit:
                    with inference_lock:
                        can_submit = not inference_in_flight
                        if can_submit:
                            inference_in_flight = True
                    if can_submit:
                        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)
                        timestamp_ms = time.monotonic_ns() // 1_000_000
                        try:
                            landmarker.detect_async(mp_image, timestamp_ms)
                        except Exception as exc:
                            with inference_lock:
                                inference_in_flight = False
                            _emit_status(status_callback, f"MediaPipe detect_async failed: {exc}")
                        else:
                            inference_submits += 1
                            last_inference_submit_time = now
                    else:
                        inference_skips += 1

                latest_result, latest_timestamp = result_store.snapshot()
                if overlay_enabled:
                    draw_landmarks(frame_bgr, latest_result, cv2)

                should_retarget = (
                    retarget_enabled
                    and latest_timestamp > 0
                    and latest_timestamp != last_processed_timestamp
                    and (retarget_interval <= 0.0 or (now - last_retarget_time) >= retarget_interval)
                )
                if should_retarget:
                    if print_landmark_output:
                        print_landmarks(latest_result, frame_bgr.shape, latest_timestamp)

                    points_3d = extract_hand_points_for_retargeting(latest_result)
                    if retargeter is not None and points_3d is not None and points_3d.shape == (21, 3):
                        retarget_start = time.perf_counter()
                        joint_angles = retargeter.solve_joint_angles(points_3d, max_iters=max(1, int(ik_max_iters)))
                        retarget_total_ms += (time.perf_counter() - retarget_start) * 1000.0
                        retarget_frames += 1
                        if print_joint_output:
                            print_robot_joint_angles(latest_timestamp, joint_angles)

                        joint_names = list(joint_angles.keys())
                        joint_values = [joint_angles[name] for name in joint_names]
                        on_target_joint_sequence(latest_timestamp, joint_names, joint_values)
                        encoder_degrees = joint_angle_dict_to_encoder_degrees(joint_angles)
                        if joint_callback is not None:
                            joint_callback(latest_timestamp, joint_names, joint_values, encoder_degrees)

                    last_processed_timestamp = latest_timestamp
                    last_retarget_time = now

                elapsed = now - last_fps_time
                if elapsed >= 1.0:
                    fps = display_frames / elapsed
                    display_frames = 0
                    last_fps_time = now

                if overlay_enabled:
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

                if profile and (now - last_profile_time) >= 1.0:
                    avg_retarget_ms = retarget_total_ms / retarget_frames if retarget_frames else 0.0
                    print(
                        "[profile] "
                        f"display_fps={fps:.1f} "
                        f"infer_submit={inference_submits} "
                        f"infer_done={inference_callbacks} "
                        f"infer_skip={inference_skips} "
                        f"retarget={retarget_frames} "
                        f"retarget_avg_ms={avg_retarget_ms:.1f}"
                    )
                    inference_submits = 0
                    inference_callbacks = 0
                    inference_skips = 0
                    retarget_frames = 0
                    retarget_total_ms = 0.0
                    last_profile_time = now

                if frame_callback is not None:
                    frame_callback(frame_bgr, fps)

                if show_window:
                    cv2.imshow("MediaPipe Hand Landmarker + URDF Retargeting", frame_bgr)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord("q")):
                        break
        finally:
            cap.release()
            if show_window:
                cv2.destroyAllWindows()
            _emit_status(status_callback, "Posture mocap stopped")

    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Realtime webcam hand keypoint detection + palm frame + "
            "URDF retargeting + robot joint IK"
        )
    )
    parser.add_argument("--camera-id", type=int, default=0, help="Camera index.")
    parser.add_argument("--num-hands", type=int, default=1, help="Maximum number of hands.")
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
    parser.add_argument(
        "--model-path",
        type=Path,
        default=DEFAULT_MODEL_PATH,
        help="Path to hand_landmarker.task model file.",
    )
    parser.add_argument(
        "--urdf-path",
        type=Path,
        default=DEFAULT_URDF_PATH,
        help="Path to robot hand URDF file.",
    )
    parser.add_argument(
        "--camera-width",
        type=int,
        default=DEFAULT_CAMERA_WIDTH,
        help="Requested camera capture width.",
    )
    parser.add_argument(
        "--camera-height",
        type=int,
        default=DEFAULT_CAMERA_HEIGHT,
        help="Requested camera capture height.",
    )
    parser.add_argument(
        "--camera-fps",
        type=float,
        default=DEFAULT_CAMERA_FPS,
        help="Requested camera capture FPS.",
    )
    parser.add_argument(
        "--inference-fps",
        type=float,
        default=DEFAULT_INFERENCE_FPS,
        help="Maximum MediaPipe inference submissions per second. Use 0 for no throttle.",
    )
    parser.add_argument(
        "--retarget-fps",
        type=float,
        default=DEFAULT_RETARGET_FPS,
        help="Maximum URDF retarget solves per second. Use 0 for every new result.",
    )
    parser.add_argument(
        "--ik-max-iters",
        type=int,
        default=DEFAULT_IK_MAX_ITERS,
        help="Maximum IK iterations per finger solve.",
    )
    parser.add_argument(
        "--no-retarget",
        action="store_true",
        help="Disable URDF IK retargeting and only run camera + MediaPipe preview.",
    )
    parser.add_argument(
        "--no-overlay",
        action="store_true",
        help="Disable landmark/FPS overlay drawing for maximum preview smoothness.",
    )
    parser.add_argument(
        "--profile",
        action="store_true",
        help="Print one-line performance counters once per second.",
    )
    parser.add_argument(
        "--print-landmarks",
        action="store_true",
        help="Print 21 human hand landmarks in terminal.",
    )
    parser.add_argument(
        "--print-joints",
        action="store_true",
        help="Print robot joint angles when retargeting publishes a frame.",
    )
    parser.add_argument(
        "--no-print-joints",
        action="store_true",
        help="Compatibility flag; joint printing is already disabled by default.",
    )
    return parser.parse_args()


def main() -> int:
    """CLI entrypoint for webcam hand tracking and retargeting."""
    args = parse_args()
    return run_mocap_loop(
        camera_id=args.camera_id,
        num_hands=args.num_hands,
        min_hand_detection_confidence=args.min_hand_detection_confidence,
        min_hand_presence_confidence=args.min_hand_presence_confidence,
        min_tracking_confidence=args.min_tracking_confidence,
        model_path=args.model_path,
        urdf_path=args.urdf_path,
        camera_width=args.camera_width,
        camera_height=args.camera_height,
        camera_fps=args.camera_fps,
        inference_fps=args.inference_fps,
        retarget_fps=args.retarget_fps,
        ik_max_iters=args.ik_max_iters,
        retarget_enabled=not args.no_retarget,
        overlay_enabled=not args.no_overlay,
        profile=args.profile,
        print_landmark_output=args.print_landmarks,
        print_joint_output=bool(args.print_joints and not args.no_print_joints),
        show_window=True,
    )


if __name__ == "__main__":
    raise SystemExit(main())
