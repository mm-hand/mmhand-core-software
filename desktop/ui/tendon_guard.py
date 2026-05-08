import json
import os
import time
from typing import List, Sequence, Tuple

from protocol import ENCODER_COUNT, MOTOR_ABS_CMD_MAX, MOTOR_ABS_CMD_MIN


MOTOR_TO_JOINT_INDEX = [
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    11, 12, 13, 14, 15, 16, 16, 17, 18, 19, 20,
]
JOINT_TO_PRIMARY_MOTOR_INDEX = [-1] * ENCODER_COUNT
for _motor_idx, _joint_idx in enumerate(MOTOR_TO_JOINT_INDEX):
    if 0 <= _joint_idx < ENCODER_COUNT and JOINT_TO_PRIMARY_MOTOR_INDEX[_joint_idx] < 0:
        JOINT_TO_PRIMARY_MOTOR_INDEX[_joint_idx] = _motor_idx


def clamp_motor_abs(value: int) -> int:
    return max(MOTOR_ABS_CMD_MIN, min(MOTOR_ABS_CMD_MAX, int(value)))


def motor_target_from_servo_feedback(servo_raw_single: int) -> int:
    return max(0, min(4095, int(servo_raw_single) % 4096))


class TendonGuard:
    def __init__(self, config_path: str):
        self.config_path = config_path
        self.release_margin_counts = 0
        self.release_angle_eps_deg = 0.0
        self.warn_interval_s = 0.5
        self._last_warn_ts = 0.0
        self._last_synced_fingerprint = None
        self.config = self.load()

    @staticmethod
    def default_config() -> List[dict]:
        return [{"enabled": False, "pull_sign": 1, "x1_abs": 0} for _ in range(ENCODER_COUNT)]

    def load(self) -> List[dict]:
        defaults = self.default_config()
        if not self.config_path or not os.path.isfile(self.config_path):
            return defaults
        try:
            with open(self.config_path, "r", encoding="utf-8") as handle:
                data = json.load(handle)
        except (OSError, json.JSONDecodeError):
            return defaults
        joints = data.get("joints", [])
        if not isinstance(joints, list):
            return defaults
        out: List[dict] = []
        for idx in range(ENCODER_COUNT):
            item = joints[idx] if idx < len(joints) and isinstance(joints[idx], dict) else {}
            out.append(self._normalize_item(item))
        return out

    def save(self) -> bool:
        if not self.config_path:
            return False
        try:
            os.makedirs(os.path.dirname(self.config_path), exist_ok=True)
            payload = {
                "comment": "Manual tendon calibration (+/- pull direction and x1 absolute anchor).",
                "joints": [self._normalize_item(item) for item in self.config[:ENCODER_COUNT]],
            }
            with open(self.config_path, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=2, ensure_ascii=False)
            return True
        except OSError:
            return False

    def set_joint(self, joint_idx: int, enabled: bool, pull_sign: int, x1_abs: int) -> bool:
        if joint_idx < 0 or joint_idx >= ENCODER_COUNT:
            return False
        while len(self.config) <= joint_idx:
            self.config.append({"enabled": False, "pull_sign": 1, "x1_abs": 0})
        self.config[joint_idx] = {
            "enabled": bool(enabled),
            "pull_sign": 1 if int(pull_sign) >= 0 else -1,
            "x1_abs": clamp_motor_abs(int(x1_abs)),
        }
        return self.save()

    def payload(self) -> List[Tuple[bool, int, int]]:
        out: List[Tuple[bool, int, int]] = []
        for idx in range(ENCODER_COUNT):
            item = self.config[idx] if idx < len(self.config) else {}
            cfg = self._normalize_item(item)
            out.append((bool(cfg["enabled"]), int(cfg["pull_sign"]), int(cfg["x1_abs"])))
        return out

    def sync_to_controller(self, controller, *, force: bool = False) -> None:
        sender = getattr(controller, "set_tendon_guard_config", None)
        if sender is None:
            return
        payload = self.payload()
        fingerprint = tuple((1 if e else 0, int(s), int(x)) for e, s, x in payload)
        if not force and fingerprint == self._last_synced_fingerprint:
            return
        try:
            sender(payload)
            self._last_synced_fingerprint = fingerprint
        except Exception:
            pass

    def apply(
        self,
        state,
        target_angles: Sequence[float],
        *,
        source: str = "",
    ) -> Tuple[List[float], List[int], str]:
        guarded = [float(v) for v in list(target_angles)[:ENCODER_COUNT]]
        if len(guarded) < ENCODER_COUNT:
            guarded.extend([0.0] * (ENCODER_COUNT - len(guarded)))
        blocked: List[int] = []
        if not state:
            return guarded, blocked, ""
        if not bool(getattr(state, "has_sensor_data", False)):
            return guarded, blocked, ""

        servo_abs = getattr(state, "servo_angles", []) or []
        servo_online = getattr(state, "servo_online", []) or []
        angles = getattr(state, "angles", []) or []
        if not servo_abs or not servo_online:
            return guarded, blocked, ""

        for joint_idx in range(ENCODER_COUNT):
            cfg = self.config[joint_idx] if joint_idx < len(self.config) else {}
            if not bool(cfg.get("enabled", False)):
                continue
            motor_idx = self.primary_motor_for_joint(joint_idx)
            if motor_idx < 0 or motor_idx >= len(servo_abs) or motor_idx >= len(servo_online):
                continue
            if joint_idx >= len(angles) or not bool(servo_online[motor_idx]):
                continue
            sign = 1 if int(cfg.get("pull_sign", 1)) >= 0 else -1
            x1_abs = clamp_motor_abs(int(cfg.get("x1_abs", 0)))
            cur_joint_deg = float(angles[joint_idx])
            target_joint_deg = float(guarded[joint_idx])
            release_cmd = target_joint_deg < (cur_joint_deg - self.release_angle_eps_deg)
            if not release_cmd:
                continue
            abs_now = int(servo_abs[motor_idx])
            hit_boundary = (
                (sign > 0 and abs_now <= (x1_abs + self.release_margin_counts))
                or (sign < 0 and abs_now >= (x1_abs - self.release_margin_counts))
            )
            if hit_boundary:
                guarded[joint_idx] = cur_joint_deg
                blocked.append(joint_idx)

        msg = ""
        if blocked:
            now = time.time()
            if now - self._last_warn_ts >= self.warn_interval_s:
                self._last_warn_ts = now
                joints = ", ".join(f"J{i}" for i in blocked)
                suffix = f" ({source})" if source else ""
                msg = f"Tendon guard blocked release on {joints}{suffix}"
        return guarded, blocked, msg

    @staticmethod
    def primary_motor_for_joint(joint_idx: int) -> int:
        if 0 <= joint_idx < len(JOINT_TO_PRIMARY_MOTOR_INDEX):
            return int(JOINT_TO_PRIMARY_MOTOR_INDEX[joint_idx])
        return -1

    @staticmethod
    def parse_sign_text(text: str) -> int:
        t = str(text or "").strip()
        if t in ("+", "+1", "1"):
            return 1
        if t in ("-", "-1"):
            return -1
        return 0

    @staticmethod
    def sign_text(item: dict) -> str:
        cfg = TendonGuard._normalize_item(item)
        if not cfg["enabled"]:
            return ""
        return "+" if int(cfg["pull_sign"]) >= 0 else "-"

    @staticmethod
    def _normalize_item(item: dict) -> dict:
        try:
            sign_raw = int(item.get("pull_sign", 1))
        except (TypeError, ValueError):
            sign_raw = 1
        try:
            x1 = int(item.get("x1_abs", 0))
        except (TypeError, ValueError):
            x1 = 0
        return {
            "enabled": bool(item.get("enabled", False)),
            "pull_sign": 1 if sign_raw >= 0 else -1,
            "x1_abs": clamp_motor_abs(x1),
        }
