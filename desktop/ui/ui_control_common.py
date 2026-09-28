"""ui_control_common.py - MMHand Desktop 模块契约。

职责：提供控制面板模块共享的格式化、校验和小型工具函数。
线程：Qt GUI 主线程执行；耗时工作必须委托后台服务，禁止阻塞界面事件循环。
输入：用户操作、HandModel 只读快照以及 HandBackend/服务返回结果。
输出：Qt 控件状态、用户提示和经门面提交的应用命令。
状态所有权：仅拥有本视图控件和瞬时交互状态；设备真值由 core 层拥有。
禁止依赖：禁止直接导入串口通信和底层控制实现，禁止在 UI 中复制协议或安全规则。
"""
import math
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

from desktop.paths import PROJECT_ROOT
from desktop.core.angle_mapping import (
    get_spec,
    urdf_manual_scaled_mapping_metadata,
)
from desktop.core.data_models import (
    HandModel,
    JointWorkspaceRange,
)
from desktop.core.protocol import (
    CALIB_WARN_AUTO_MAX_SHORTFALL,
    CALIB_WARN_AUTO_MIN_OFFSET,
    CALIB_WARN_URDF_MANUAL_SPAN_MISMATCH,
    CALIB_DIAG_REASON_JOINT_STEP_FAILED,
    CALIB_DIAG_PHASE_SEQUENCE,
    ENCODER_COUNT,
    HAND_PROFILE,
    MOTOR_COUNT,
    DUAL_TENDON_JOINT_INDICES,
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
    DIRECT_REJECT_NONE,
    DIRECT_REJECT_STATE,
    DIRECT_REJECT_HARDWARE,
    DIRECT_REJECT_MULTITURN,
    DIRECT_REJECT_EVENT_QUEUE,
    DIRECT_REJECT_STALE,
    DIRECT_REJECT_FAULT,
    SERVO_ROUTE_UNKNOWN,
    SERVO_ROUTE_PROBING,
    SERVO_ROUTE_ONLINE,
    SERVO_ROUTE_OFFLINE,
    SERVO_ROUTE_CONFLICT,
    SERVO_ROUTE_BUS_OVERLOADED,
    directed_encoder_delta,
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
# 当前单圈 raw 与上位机 JSON 锚点的环形差达到该值时提示恢复可能失配。
MOTOR_SAVED_RAW_WARN_COUNTS = 400
MOTOR_RELEASE_HALF_TURN_COUNTS = MOTOR_SINGLE_TURN_STEPS // 2
MOTOR_JUMP_RESYNC_THRESH = 2048
MOTOR_SLIDER_WRAP_MARGIN = 512
MOTOR_SLIDER_MAX_TARGET_DELTA = MOTOR_SINGLE_TURN_STEPS
MOTOR_DIR_HINT_CLEAR_TOL = 3
MOTOR_DIR_HINT_CLEAR_STABLE_FRAMES = 4
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
    "CALIB_WARN_AUTO_MAX_SHORTFALL",
    "CALIB_WARN_AUTO_MIN_OFFSET",
    "CALIB_WARN_URDF_MANUAL_SPAN_MISMATCH",
    "ControlMode",
    "ControlScopeModel",
    "DUAL_TENDON_JOINT_INDICES",
    "DIRECT_REJECT_NONE",
    "DIRECT_REJECT_STATE",
    "DIRECT_REJECT_HARDWARE",
    "DIRECT_REJECT_MULTITURN",
    "DIRECT_REJECT_EVENT_QUEUE",
    "DIRECT_REJECT_STALE",
    "DIRECT_REJECT_FAULT",
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
    "MOTOR_SAVED_RAW_WARN_COUNTS",
    "MOTOR_RELEASE_HALF_TURN_COUNTS",
    "MOTOR_SINGLE_TURN_STEPS",
    "MOTOR_SLIDER_MAX_TARGET_DELTA",
    "MOTOR_SLIDER_WRAP_MARGIN",
    "MOTOR_TARGET_SCALE_HI",
    "MOTOR_TARGET_SCALE_LO",
    "MOTOR_TO_JOINT_INDEX",
    "Optional",
    "PROJECT_ROOT",
    "Path",
    "QtCore",
    "QtGui",
    "QtWidgets",
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
    "TEST_DEFAULT_FREQ_HZ",
    "TEST_DEFAULT_HOLD_S",
    "TEST_DEFAULT_PHASE_DEG",
    "TEST_ROW_COUNT",
    "TEST_TICK_MS",
    "TRI_TEST_DEFAULT_FREQ_HZ",
    "Tuple",
    "VALID_MOTOR_MASK",
    "WORKSPACE_MAPPING_VERSION",
    "WORKSPACE_URDF_CRC",
    "_ModeBody",
    "_motor_channel_valid",
    "clamp_motor_abs",
    "directed_encoder_delta",
    "math",
    "motor_abs_after_tendon_action",
    "motor_target_from_servo_feedback",
    "motor_tendon_shorten_abs_sign",
    "np",
    "urdf_manual_scaled_mapping_metadata",
]
