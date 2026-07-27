import json
import math
import shutil
import threading
import time
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

from desktop.paths import CALIB_LOAD_SNAPSHOT_PATH, MOTOR_MID_CALIB_PATH, PROJECT_ROOT
from desktop.core.angle_mapping import (
    get_spec,
    urdf_manual_scaled_mapping_metadata,
)
from desktop.core.calibration_snapshot import (
    CalibrationSnapshotError,
    atomic_write_snapshot,
    merge_snapshot,
    read_snapshot,
    validate_snapshot,
)
from desktop.core.data_models import (
    HandModel,
    JointWorkspaceRange,
)
from desktop.core.protocol import (
    CALIB_WARN_AUTO_MAX_SHORTFALL,
    CALIB_WARN_AUTO_MIN_OFFSET,
    CALIB_WARN_URDF_MANUAL_SPAN_MISMATCH,
    ENCODER_COUNT,
    HAND_PROFILE,
    MOTOR_COUNT,
    DUAL_TENDON_JOINT_INDICES,
    TENDON_LUT_MAX_SAMPLES,
    VALID_MOTOR_MASK,
    ControlMode,
    MOTOR_ABS_CMD_MIN,
    MOTOR_ABS_CMD_MAX,
    SYSTEM_STATE_UNCONTROLLABLE,
    SYSTEM_STATE_IDLE,
    SYSTEM_STATE_PAUSED,
    SYSTEM_STATE_CALIBRATION_RUNNING,
    SYSTEM_STATE_RUN_DIRECT,
    SYSTEM_STATE_RUN_FEEDBACK,
    SYSTEM_STATE_ROPE_INIT_RUNNING,
    SYSTEM_STATE_READY,
    SYSTEM_STATE_RECOVERY_REQUIRED,
    SYSTEM_FAULT_MULTITURN_UNKNOWN,
    ROPE_INIT_STATUS_RUNNING,
    ROPE_INIT_STATUS_SUCCESS,
    ROPE_INIT_STATUS_FAILED,
    ROPE_INIT_REASON_NONE,
    ROPE_INIT_REASON_TIMEOUT,
    ROPE_INIT_REASON_FEEDBACK_STALE,
    ROPE_INIT_REASON_SERVO_OFFLINE,
    ROPE_INIT_REASON_MULTITURN_UNKNOWN,
    ROPE_INIT_REASON_MANUAL_REQUIRED,
    ROPE_INIT_REASON_AUTO_REQUIRED,
    ROPE_INIT_REASON_TARGET_PUBLISH_FAILED,
    ROPE_INIT_REASON_SYSTEM_FAULT,
    ROPE_INIT_REASON_CANCELLED,
    ROPE_INIT_REASON_TASK_START_FAILED,
    directed_encoder_delta,
    manual_encoder_reference_entry_crc,
    tendon_lut_field,
)
from .ui_qt_compat import QtCore, QtGui, QtWidgets, Signal
from .ui_device_check_view import DeviceCheckView
from desktop.core.motor_mapping import (
    MOTOR_TO_JOINT_INDEX,
    clamp_motor_abs,
    motor_abs_after_tendon_action,
    motor_tendon_shorten_abs_sign,
    motor_target_from_servo_feedback,
)
from .ui_teleop_view import TeleopView
from .ui_replay_view import ReplayView
from .ui_control_scope import (
    AUTO_CALIBRATION_JOINT_INDICES,
    FINGER_JOINT_RANGES,
    ControlScopeModel,
    _motor_channel_valid,
)
from .ui_control_target_model import JointTargetModel


MOTOR_TARGET_SCALE_LO = 0
MOTOR_TARGET_SCALE_HI = 4095
MOTOR_SINGLE_TURN_STEPS = 4096
# 当前单圈 raw 与 Flash 快照的环形差达到该值时提示恢复锚点可能失配。
MOTOR_FLASH_RAW_WARN_COUNTS = 256
MOTOR_RELEASE_HALF_TURN_COUNTS = MOTOR_SINGLE_TURN_STEPS // 2
MOTOR_JUMP_RESYNC_THRESH = 2048
MOTOR_SLIDER_WRAP_MARGIN = 512
MOTOR_SLIDER_MAX_TARGET_DELTA = MOTOR_SINGLE_TURN_STEPS
MOTOR_DIR_HINT_CLEAR_TOL = 3
MOTOR_DIR_HINT_CLEAR_STABLE_FRAMES = 4
ROPE_INIT_START_TIMEOUT_S = 3.0
ROPE_INIT_REASON_LABELS = {
    ROPE_INIT_REASON_NONE: "no error",
    ROPE_INIT_REASON_TIMEOUT: "timeout",
    ROPE_INIT_REASON_FEEDBACK_STALE: "feedback stale",
    ROPE_INIT_REASON_SERVO_OFFLINE: "servo offline",
    ROPE_INIT_REASON_MULTITURN_UNKNOWN: "multi-turn unknown",
    ROPE_INIT_REASON_MANUAL_REQUIRED: "Manual endpoints required",
    ROPE_INIT_REASON_AUTO_REQUIRED: "Auto v3 calibration required",
    ROPE_INIT_REASON_TARGET_PUBLISH_FAILED: "target publish failed",
    ROPE_INIT_REASON_SYSTEM_FAULT: "system fault",
    ROPE_INIT_REASON_CANCELLED: "cancelled",
    ROPE_INIT_REASON_TASK_START_FAILED: "task start failed",
}
TEST_ROW_COUNT = 5
TEST_TICK_MS = 10
TEST_DEFAULT_FREQ_HZ = 0.20
TEST_DEFAULT_PHASE_DEG = 0.0
TEST_DEFAULT_HOLD_S = 0.0
FF_TEST_DEFAULT_TARGETS_DEG = "20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40"
FF_TEST_DEFAULT_STEP_S = 1.0
TRI_TEST_DEFAULT_FREQ_HZ = 0.20
JOINT_TARGET_MIN_DEG_BY_INDEX = [get_spec(i).urdf_range_deg[0] for i in range(ENCODER_COUNT)]
JOINT_TARGET_MAX_DEG_BY_INDEX = [get_spec(i).urdf_range_deg[1] for i in range(ENCODER_COUNT)]
try:
    WORKSPACE_MAPPING_VERSION, WORKSPACE_URDF_CRC = urdf_manual_scaled_mapping_metadata()
except (OSError, TypeError, ValueError):
    WORKSPACE_MAPPING_VERSION, WORKSPACE_URDF_CRC = 0, 0

OPEN_POSE_CURRENT_MIN_OFFSET_DEG = 5.0
FIST_POSE_URDF_MAX_OFFSET_DEG = 7.0

class _ModeBody(QtWidgets.QWidget):
    def __init__(self, child: Optional[QtWidgets.QWidget] = None, parent=None):
        """Initialize this control panel component."""
        super().__init__(parent)
        self.child = child

    def update_state(self, state) -> None:
        """Run the update state workflow."""
        updater = getattr(self.child, "update_state", None)
        if callable(updater):
            updater(state)

    def close(self) -> None:
        """Close the proxied child widget safely."""
        closer = getattr(self.child, "close", None)
        if callable(closer):
            closer()
        super().close()


__all__ = [
    "AUTO_CALIBRATION_JOINT_INDICES",
    "CALIB_LOAD_SNAPSHOT_PATH",
    "CALIB_WARN_AUTO_MAX_SHORTFALL",
    "CALIB_WARN_AUTO_MIN_OFFSET",
    "CALIB_WARN_URDF_MANUAL_SPAN_MISMATCH",
    "CalibrationSnapshotError",
    "ControlMode",
    "ControlScopeModel",
    "DUAL_TENDON_JOINT_INDICES",
    "DeviceCheckView",
    "ENCODER_COUNT",
    "FF_TEST_DEFAULT_STEP_S",
    "FF_TEST_DEFAULT_TARGETS_DEG",
    "FINGER_JOINT_RANGES",
    "FIST_POSE_URDF_MAX_OFFSET_DEG",
    "HAND_PROFILE",
    "HandModel",
    "JointTargetModel",
    "JointWorkspaceRange",
    "List",
    "MOTOR_ABS_CMD_MAX",
    "MOTOR_ABS_CMD_MIN",
    "MOTOR_COUNT",
    "MOTOR_DIR_HINT_CLEAR_STABLE_FRAMES",
    "MOTOR_DIR_HINT_CLEAR_TOL",
    "MOTOR_JUMP_RESYNC_THRESH",
    "MOTOR_FLASH_RAW_WARN_COUNTS",
    "MOTOR_MID_CALIB_PATH",
    "MOTOR_RELEASE_HALF_TURN_COUNTS",
    "MOTOR_SINGLE_TURN_STEPS",
    "MOTOR_SLIDER_MAX_TARGET_DELTA",
    "MOTOR_SLIDER_WRAP_MARGIN",
    "MOTOR_TARGET_SCALE_HI",
    "MOTOR_TARGET_SCALE_LO",
    "MOTOR_TO_JOINT_INDEX",
    "OPEN_POSE_CURRENT_MIN_OFFSET_DEG",
    "Optional",
    "PROJECT_ROOT",
    "Path",
    "QtCore",
    "QtGui",
    "QtWidgets",
    "ROPE_INIT_REASON_LABELS",
    "ROPE_INIT_START_TIMEOUT_S",
    "ROPE_INIT_STATUS_FAILED",
    "ROPE_INIT_STATUS_RUNNING",
    "ROPE_INIT_STATUS_SUCCESS",
    "ReplayView",
    "SYSTEM_FAULT_MULTITURN_UNKNOWN",
    "SYSTEM_STATE_CALIBRATION_RUNNING",
    "SYSTEM_STATE_IDLE",
    "SYSTEM_STATE_PAUSED",
    "SYSTEM_STATE_READY",
    "SYSTEM_STATE_RECOVERY_REQUIRED",
    "SYSTEM_STATE_ROPE_INIT_RUNNING",
    "SYSTEM_STATE_RUN_DIRECT",
    "SYSTEM_STATE_RUN_FEEDBACK",
    "SYSTEM_STATE_UNCONTROLLABLE",
    "Signal",
    "TENDON_LUT_MAX_SAMPLES",
    "TEST_DEFAULT_FREQ_HZ",
    "TEST_DEFAULT_HOLD_S",
    "TEST_DEFAULT_PHASE_DEG",
    "TEST_ROW_COUNT",
    "TEST_TICK_MS",
    "TRI_TEST_DEFAULT_FREQ_HZ",
    "TeleopView",
    "Tuple",
    "VALID_MOTOR_MASK",
    "WORKSPACE_MAPPING_VERSION",
    "WORKSPACE_URDF_CRC",
    "_ModeBody",
    "_motor_channel_valid",
    "atomic_write_snapshot",
    "clamp_motor_abs",
    "directed_encoder_delta",
    "json",
    "manual_encoder_reference_entry_crc",
    "math",
    "merge_snapshot",
    "motor_abs_after_tendon_action",
    "motor_target_from_servo_feedback",
    "motor_tendon_shorten_abs_sign",
    "np",
    "read_snapshot",
    "shutil",
    "tendon_lut_field",
    "threading",
    "time",
    "urdf_manual_scaled_mapping_metadata",
    "validate_snapshot",
]
