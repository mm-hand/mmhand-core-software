"""Wuji SDK subscription and MediaPipe-21 to retarget skeleton adaptation."""

from __future__ import annotations

import threading
import time
import sys
import math
from pathlib import Path
from typing import Any, Optional

from wuji_mmhand_teleop.types import WujiFrame
from wuji_mmhand_teleop.sources.one_euro_filter import OneEuroFilter3D
from wuji_mmhand_teleop.sources.wuji_fingertip_cp import (
    TIP_SEMANTICS,
    active_fingertip_contact_transforms,
    apply_fingertip_contact_points,
)


_VENDOR_DIR = Path(__file__).resolve().parents[1] / "_vendor"
if _VENDOR_DIR.is_dir() and str(_VENDOR_DIR) not in sys.path:
    sys.path.insert(0, str(_VENDOR_DIR))


MEDIAPIPE_NAMES = (
    "wrist",
    "thumb_cmc", "thumb_mcp", "thumb_ip", "thumb_tip",
    "index_finger_mcp", "index_finger_pip", "index_finger_dip", "index_finger_tip",
    "middle_finger_mcp", "middle_finger_pip", "middle_finger_dip", "middle_finger_tip",
    "ring_finger_mcp", "ring_finger_pip", "ring_finger_dip", "ring_finger_tip",
    "pinky_mcp", "pinky_pip", "pinky_dip", "pinky_tip",
)

FINGER_POINTS = {
    "thumb": ("thumb_cmc", "thumb_mcp", "thumb_ip", "thumb_tip"),
    "index": ("index_finger_mcp", "index_finger_pip", "index_finger_dip", "index_finger_tip"),
    "middle": ("middle_finger_mcp", "middle_finger_pip", "middle_finger_dip", "middle_finger_tip"),
    "ring": ("ring_finger_mcp", "ring_finger_pip", "ring_finger_dip", "ring_finger_tip"),
    "pinky": ("pinky_mcp", "pinky_pip", "pinky_dip", "pinky_tip"),
}

FILTERED_SKELETON_NODES = {
    "thumb": ("mcp", "ip", "tip"),
    "index": ("pip", "dip", "tip"),
    "middle": ("pip", "dip", "tip"),
    "ring": ("pip", "dip", "tip"),
    "pinky": ("pip", "dip", "tip"),
}


def _value(obj: Any, name: str, default: Any = None) -> Any:
    if isinstance(obj, dict):
        return obj.get(name, default)
    return getattr(obj, name, default)


def _xyz(value: Any) -> list[float]:
    if value is None:
        raise ValueError("missing position")
    if all(hasattr(value, axis) for axis in "xyz"):
        return [float(value.x), float(value.y), float(value.z)]
    return [float(value[0]), float(value[1]), float(value[2])]


def _quat_wxyz(value: Any) -> list[float]:
    if value is None:
        return [1.0, 0.0, 0.0, 0.0]
    if all(hasattr(value, axis) for axis in "xyzw"):
        return [float(value.w), float(value.x), float(value.y), float(value.z)]
    if isinstance(value, dict):
        return [float(value.get("w", 1.0)), float(value.get("x", 0.0)),
                float(value.get("y", 0.0)), float(value.get("z", 0.0))]
    # Wuji's documented serialization order is x, y, z, w.
    return [float(value[3]), float(value[0]), float(value[1]), float(value[2])]


def _node(joint: Any, confidence_threshold: float) -> Optional[dict[str, Any]]:
    confidence = float(_value(joint, "confidence", 1.0))
    if confidence < confidence_threshold:
        return None
    pose = _value(joint, "pose")
    return {
        "pos": _xyz(_value(pose, "position")),
        "rot": _quat_wxyz(_value(pose, "orientation")),
        "confidence": confidence,
    }


def adapt_hand_skeleton(message: Any, side: str, confidence_threshold: float) -> WujiFrame:
    header = _value(message, "header", {})
    joints = list(_value(message, "joints", []) or [])
    by_name: dict[str, dict[str, Any]] = {}
    for index, joint in enumerate(joints):
        name = str(_value(joint, "name", "") or "")
        if not name and index < len(MEDIAPIPE_NAMES):
            name = MEDIAPIPE_NAMES[index]
        node = _node(joint, confidence_threshold)
        if name and node is not None:
            by_name[name] = node

    wrist = by_name.get("wrist")
    if wrist is None:
        raise ValueError("Wuji skeleton has no reliable wrist point")

    skeleton: dict[str, Any] = {"hand": wrist}
    for finger, names in FINGER_POINTS.items():
        points = [by_name.get(name) for name in names]
        if any(point is None for point in points):
            continue
        if finger == "thumb":
            skeleton[finger] = {
                "cmc": points[0], "mcp": points[1],
                "ip": points[2], "tip": points[3],
            }
        else:
            # Preserve the native MediaPipe MCP/PIP/DIP/tip chain exactly.
            skeleton[finger] = {
                "mcp": points[0], "pip": points[1],
                "dip": points[2], "tip": points[3],
            }

    now = time.time()
    timestamp_us = int(_value(header, "timestamp_us", 0) or 0)
    return WujiFrame(
        seq=int(_value(header, "seq", 0) or 0),
        timestamp=timestamp_us / 1_000_000.0,
        recv_time=now,
        recv_perf_time=time.perf_counter(),
        dt=0.0,
        glove_id=0,
        side=str(side).lower(),
        skeleton=skeleton,
        frame_id=str(_value(header, "frame_id", "") or ""),
    )


class WujiReceiver:
    def __init__(
        self,
        device_sn: str = "",
        side: str = "left",
        confidence_threshold: float = 0.4,
        *,
        filter_enabled: bool = True,
        filter_min_cutoff: float = 0.5,
        filter_beta: float = 5.0,
        filter_d_cutoff: float = 1.0,
    ):
        self.device_sn = str(device_sn)
        self.side = str(side).lower()
        self.confidence_threshold = float(confidence_threshold)
        self.filter_enabled = bool(filter_enabled)
        self._filter_parameters = {
            "min_cutoff": float(filter_min_cutoff),
            "beta": float(filter_beta),
            "d_cutoff": float(filter_d_cutoff),
        }
        self._skeleton_filters: dict[str, OneEuroFilter3D] = {}
        self._filter_time_source: Optional[str] = None
        self._last_sdk_filter_timestamp: Optional[float] = None
        self.tip_semantics = TIP_SEMANTICS
        self.hand_profile = ""
        self.hand_model_path = ""
        self._fingertip_contact_transforms = {}
        self._manager = None
        self._glove = None
        self._subscriptions = []
        self._lock = threading.Lock()
        self._latest_frame: Optional[WujiFrame] = None
        self._latest_angles: list[list[float]] = []
        self._latest_angle_confidence: list[float] = []
        self._last_recv_time = 0.0
        self._error = ""

    def start(self) -> None:
        self._reset_skeleton_filters()
        try:
            from wuji_sdk import SdkManager

            self._manager = SdkManager.instance()
            devices = list(self._manager.scan())
            if self.device_sn:
                devices = [item for item in devices if str(_value(item, "sn", "")) == self.device_sn]
            if not devices:
                raise RuntimeError(f"Wuji glove not found (sn={self.device_sn or 'auto'})")
            device = devices[0]
            sn = str(_value(device, "sn", ""))
            self._glove = self._manager.connect(sn=sn, device_name="glove")
            profile, model_path, transforms = active_fingertip_contact_transforms(self._glove)
            self.hand_profile = profile
            self.hand_model_path = str(model_path)
            self._fingertip_contact_transforms = transforms
            self._subscriptions = [
                self._glove.hand_skeleton().subscribe_with_callback(self._on_skeleton),
                self._glove.hand_joint_angles().subscribe_with_callback(self._on_joint_angles),
            ]
        except Exception as exc:
            self._set_error(str(exc))
            self.stop()
            raise

    def stop(self) -> None:
        self._reset_skeleton_filters()
        subscriptions, self._subscriptions = self._subscriptions, []
        for subscription in subscriptions:
            try:
                subscription.close()
            except Exception:
                pass
        glove, self._glove = self._glove, None
        if glove is not None and hasattr(glove, "close"):
            try:
                glove.close()
            except Exception:
                pass

    def _on_skeleton(self, message: Any) -> None:
        try:
            frame = adapt_hand_skeleton(message, self.side, self.confidence_threshold)
        except Exception as exc:
            self._set_error(f"invalid Wuji skeleton: {exc}")
            return
        try:
            apply_fingertip_contact_points(frame.skeleton, self._fingertip_contact_transforms)
        except Exception as exc:
            self._set_error(f"invalid Wuji fingertip_cp: {exc}")
            return
        if self.filter_enabled:
            self._filter_skeleton_positions(frame)
        with self._lock:
            previous = self._latest_frame
            if previous is not None:
                frame.dt = max(0.0, frame.recv_time - previous.recv_time)
            frame.joint_angles_rad = [list(values) for values in self._latest_angles]
            frame.joint_confidence = list(self._latest_angle_confidence)
            self._latest_frame = frame
            self._last_recv_time = frame.recv_time
            self._error = ""

    def _filter_skeleton_positions(self, frame: WujiFrame) -> None:
        timestamp = self._filter_timestamp(frame)
        for finger_name, node_keys in FILTERED_SKELETON_NODES.items():
            finger = frame.skeleton.get(finger_name)
            if not isinstance(finger, dict):
                continue
            for node_key in node_keys:
                node = finger.get(node_key)
                if not isinstance(node, dict) or "pos" not in node:
                    continue
                filter_key = f"{finger_name}.{node_key}"
                position_filter = self._skeleton_filters.get(filter_key)
                if position_filter is None:
                    position_filter = OneEuroFilter3D(**self._filter_parameters)
                    self._skeleton_filters[filter_key] = position_filter
                node["pos"] = position_filter.update(node["pos"], timestamp).tolist()

    def _filter_timestamp(self, frame: WujiFrame) -> float:
        sdk_timestamp = float(frame.timestamp)
        sdk_valid = math.isfinite(sdk_timestamp) and sdk_timestamp > 0.0
        if self._filter_time_source is None:
            self._filter_time_source = "sdk" if sdk_valid else "receive"
        if self._filter_time_source == "sdk":
            if (sdk_valid and (self._last_sdk_filter_timestamp is None
                              or sdk_timestamp > self._last_sdk_filter_timestamp)):
                self._last_sdk_filter_timestamp = sdk_timestamp
                return sdk_timestamp
            # A frozen or reversed SDK clock would repeatedly reset every node.
            # Fall back to the local monotonic clock for the rest of this session.
            self._skeleton_filters.clear()
            self._filter_time_source = "receive"
        return float(frame.recv_perf_time)

    def _reset_skeleton_filters(self) -> None:
        self._skeleton_filters.clear()
        self._filter_time_source = None
        self._last_sdk_filter_timestamp = None

    def _on_joint_angles(self, message: Any) -> None:
        try:
            fingers = list(_value(message, "fingers", []) or [])
            angles = [
                [float(value) for value in list(_value(finger, "angles", []) or [])]
                for finger in fingers
            ]
            confidence = [float(_value(finger, "confidence", 0.0)) for finger in fingers]
            if len(angles) != 5 or any(len(values) < 5 for values in angles):
                raise ValueError("expected five fingers with five angle slots")
        except Exception as exc:
            self._set_error(f"invalid Wuji joint angles: {exc}")
            return
        with self._lock:
            self._latest_angles = angles
            self._latest_angle_confidence = confidence

    def get_latest_frame(self) -> Optional[WujiFrame]:
        with self._lock:
            return self._latest_frame

    def is_alive(self, timeout_s: float = 2.0) -> bool:
        with self._lock:
            return self._last_recv_time > 0.0 and time.time() - self._last_recv_time <= timeout_s

    def last_error(self) -> str:
        with self._lock:
            return self._error

    def _set_error(self, message: str) -> None:
        with self._lock:
            self._error = str(message)


__all__ = ["FILTERED_SKELETON_NODES", "MEDIAPIPE_NAMES", "WujiReceiver", "adapt_hand_skeleton"]
