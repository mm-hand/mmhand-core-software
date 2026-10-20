"""data_models.py - MMHand Desktop 模块契约。

职责：定义上位机共享的设备状态、标定状态和遥测数据模型。
线程：除 comm_layer 的串口收发线程外不主动创建 GUI 线程；共享状态必须按既有锁读取。
输入：P4 字节流、上层类型化命令、配置和持久化数据。
输出：线程安全的 HandModel 快照、协议帧、ACK 和后端操作结果。
状态所有权：通信对象拥有串口与解析缓存；HandModel 的写入只允许在核心同步边界内发生。
禁止依赖：禁止依赖 Qt/UI、app、ROS2 或 Replay 页面，禁止把控件和展示规则带入核心层。
"""
# data_models.py - 灵巧手数据模型（状态与控制合一）

from dataclasses import dataclass, field, fields, is_dataclass
from copy import copy, deepcopy
import math
import types
from typing import List, Optional, Tuple, Union, get_args, get_origin, get_type_hints
import numpy as np
from enum import Enum

from .protocol import (
    ENCODER_COUNT,
    MOTOR_COUNT,
    manual_encoder_reference_entry_crc,
)

# 角度 → 度 转换因子 (14 位编码器 16384 = 360°)
RAW_TO_DEG = 360.0 / 16384.0

# 触觉：每指 3 组传感数据（指尖 5×5=25，两指腹各 4×13=52），contact_forces 形状见下；热力图条带几何仍为 5 段
TACTILE_SENSORS_PER_FINGER = 3
TACTILE_SOURCE_NONE = "NONE"
TACTILE_SOURCE_P4_CAN_RELAY = "P4_CAN_RELAY"
TACTILE_SOURCE_S3_SERIAL = "S3_SERIAL"
TACTILE_DATA_LEVEL_NONE = "NONE"
TACTILE_DATA_LEVEL_SUMMARY = "SUMMARY"
TACTILE_DATA_LEVEL_FULL = "FULL"
TACTILE_TIP_ROWS, TACTILE_TIP_COLS = 5, 5
TACTILE_PAD_ROWS, TACTILE_PAD_COLS = 4, 13
TACTILE_TIP_TAXEL_COUNT = TACTILE_TIP_ROWS * TACTILE_TIP_COLS  # 25
TACTILE_PAD_TAXEL_COUNT = TACTILE_PAD_ROWS * TACTILE_PAD_COLS  # 52
def tactile_segment_shape(seg_index: int) -> Tuple[int, int]:
    """Sensor wire shapes: segment 2 is the tip; 0/1 are finger pads."""
    return (TACTILE_TIP_ROWS, TACTILE_TIP_COLS) if seg_index == 2 else (TACTILE_PAD_ROWS, TACTILE_PAD_COLS)


_SCALAR_STATE_TYPES = {int, float, bool, str, bytes, type(None)}


def _copy_state_value(value):
    """Copy nested telemetry; flat scalar lists use the fast built-in list copy."""
    value_type = type(value)
    if value_type is list:
        for item in value:
            if type(item) not in _SCALAR_STATE_TYPES:
                return [_copy_state_value(item) for item in value]
        return value.copy()
    if value_type is dict:
        for item in value.values():
            if type(item) not in _SCALAR_STATE_TYPES:
                return {key: _copy_state_value(item) for key, item in value.items()}
        return value.copy()
    if isinstance(value, np.ndarray):
        return value.copy()
    if is_dataclass(value) or isinstance(value, (tuple, set)):
        return deepcopy(value)
    # Rare list/dict subclasses keep the generic recursive copy contract.
    if isinstance(value, list):
        return [_copy_state_value(item) for item in value]
    if isinstance(value, dict):
        return {key: _copy_state_value(item) for key, item in value.items()}
    return value


class MotorState(Enum):
    IDLE = 0
    RUNNING = 1
    ERROR = 2
    CALIBRATING = 3


@dataclass
class TactileSensor:
    """单组触觉元：指尖为 (5,5,3)，指腹为 (4,13,3)；末维 Fx,Fy,Fz"""
    contact_forces: np.ndarray  # (5,5,3) 或 (4,13,3)
    resultant: np.ndarray       # (3,)
    raw_contact_forces: Optional[np.ndarray] = None
    raw_unsigned_contact_forces: Optional[np.ndarray] = None

    @property
    def n_rows(self) -> int:
        return self.contact_forces.shape[0]

    @property
    def n_cols(self) -> int:
        return self.contact_forces.shape[1]

    @property
    def magnitude(self) -> float:
        return float(np.linalg.norm(self.resultant))


@dataclass
class FingerTactile:
    """单指 3 组数据：sensors[0] 尖 25，[1][2] 两腹各 52"""
    sensors: List[TactileSensor]


@dataclass
class JointEncoder:
    """
    关节处的角度编码器（每个关节一个传感器）。
    包含：原始读数、校准零点、当前角度、目标角度及故障标志。
    """
    encoder_id: int = 0
    raw_sensor: int = 0       # magnetic encoder original raw for UI Raw column
    raw: int = 0              # 原始数据（14 位，0~16383）
    zero_raw: int = 0         # 矫正的零点值（来自 P4 标定状态）
    target_angle_deg: float = 0.0   # 目标角度（度）
    error: bool = False       # 传感器/读数异常
    angle_error: bool = True  # mapped angle packet validity; not overwritten by health status
    raw_error: bool = True    # raw encoder packet validity; not overwritten by health status
    angle_out_of_workspace: bool = False  # live mapped angle is outside calibrated URDF interval

    @property
    def current_angle_deg(self) -> float:
        """当前角度（度），由 (raw - zero_raw) 换算"""
        return (self.raw - self.zero_raw) * RAW_TO_DEG


def _default_encoders() -> List[JointEncoder]:
    return [JointEncoder(encoder_id=i) for i in range(ENCODER_COUNT)]


@dataclass
class MotorSlot:
    """
    单个电机槽：仅电机状态（来自下位机），共 MOTOR_COUNT 个。
    角度与目标由同索引的 JointEncoder 表示。
    """
    motor_id: int
    error: bool = False
    state: MotorState = MotorState.IDLE


def _default_motors() -> List[MotorSlot]:
    return [MotorSlot(motor_id=i) for i in range(MOTOR_COUNT)]


@dataclass(frozen=True)
class JointWorkspaceRange:
    """Validated host-side view of one joint's V4 Manual/URDF workspace."""

    joint_index: int
    urdf_min_deg: float
    urdf_max_deg: float
    current_min_deg: Optional[float]
    current_max_deg: float
    manual_valid: bool
    auto_valid: bool
    manual_crc_match: bool
    mapping_valid: bool
    ready: bool
    reason: str = ""

    def clamp(self, value: float) -> float:
        if not self.ready or self.current_min_deg is None:
            raise ValueError(self.reason or f"J{self.joint_index:02d} workspace is not ready")
        target = float(value)
        if not math.isfinite(target):
            target = self.current_min_deg
        return max(self.current_min_deg, min(self.current_max_deg, target))

def resolve_joint_workspace_range(
    model: Optional["HandModel"],
    joint_index: int,
    urdf_min_deg: float,
    urdf_max_deg: float,
    expected_mapping_version: int,
    expected_urdf_crc: int,
) -> JointWorkspaceRange:
    """校验 V4 标定有效性，并返回固定的 URDF Min/Max 控制区间。"""

    idx = int(joint_index)
    urdf_min = float(urdf_min_deg)
    urdf_max = float(urdf_max_deg)
    base = dict(
        joint_index=idx,
        urdf_min_deg=urdf_min,
        urdf_max_deg=urdf_max,
        current_min_deg=None,
        current_max_deg=urdf_max,
        manual_valid=False,
        auto_valid=False,
        manual_crc_match=False,
        mapping_valid=False,
        ready=False,
    )
    if model is None or idx < 0 or idx >= ENCODER_COUNT:
        return JointWorkspaceRange(**base, reason="state unavailable")
    if not math.isfinite(urdf_min) or not math.isfinite(urdf_max) or urdf_min >= urdf_max:
        return JointWorkspaceRange(**base, reason="invalid URDF range")

    bit = 1 << idx
    manual_valid = bool(
        getattr(model, "manual_reference_committed", False)
        and (int(getattr(model, "manual_reference_valid_bitmap", 0) or 0) & bit)
    )
    auto_valid = bool(int(getattr(model, "auto_calibration_valid_bitmap", 0) or 0) & bit)
    base["manual_valid"] = manual_valid
    base["auto_valid"] = auto_valid
    if not manual_valid:
        return JointWorkspaceRange(**base, reason="Manual reference missing")
    if not auto_valid:
        return JointWorkspaceRange(**base, reason="Auto calibration missing")

    results = list(getattr(model, "joint_calib_results", []) or [])
    result = results[idx] if idx < len(results) else None
    if not isinstance(result, dict) or not bool(result.get("success", False)):
        return JointWorkspaceRange(**base, reason="v4 calibration result missing")
    if not bool(result.get("hasManualScaledV4Fields", False)):
        return JointWorkspaceRange(**base, reason="v4 calibration fields missing")
    try:
        mapping_version = int(result.get("mappingVersion", 0) or 0)
        mapping_crc = int(result.get("urdfConfigCrc", 0) or 0)
        result_manual_crc = int(result.get("manualReferenceCrc", 0) or 0)
        current_min = float(result.get("currentMinDeg"))
        reported_current_max = float(result.get("currentMaxDeg"))
        angle_min = float(result.get("angleMin"))
        angle_max = float(result.get("angleMax"))
    except (TypeError, ValueError):
        return JointWorkspaceRange(**base, reason="invalid v4 calibration fields")

    manual_min_values = list(
        getattr(model, "manual_reference_min_raw", []) or []
    )
    manual_max_values = list(
        getattr(model, "manual_reference_max_raw", []) or []
    )
    try:
        manual_entry_crc = manual_encoder_reference_entry_crc(
            idx,
            manual_min_values[idx],
            manual_max_values[idx],
        )
    except (IndexError, TypeError, ValueError):
        return JointWorkspaceRange(**base, reason="Manual endpoint data missing")
    manual_crc_match = result_manual_crc == manual_entry_crc
    mapping_valid = (
        int(expected_mapping_version) == 4
        and mapping_version == 4
        and mapping_crc == (int(expected_urdf_crc) & 0xFFFFFFFF)
    )
    base["manual_crc_match"] = manual_crc_match
    base["mapping_valid"] = mapping_valid
    if not mapping_valid:
        return JointWorkspaceRange(**base, reason="URDF mapping version/CRC mismatch")
    if not manual_crc_match:
        return JointWorkspaceRange(**base, reason="Manual CRC mismatch")
    if (
        not math.isfinite(current_min)
        or not math.isfinite(reported_current_max)
        or not math.isfinite(angle_min)
        or not math.isfinite(angle_max)
        or abs(current_min - urdf_min) > 0.01
        or abs(reported_current_max - urdf_max) > 0.01
        or abs(angle_min - current_min) > 0.01
        or abs(angle_max - urdf_max) > 0.01
    ):
        return JointWorkspaceRange(**base, reason="V4 range must equal static URDF Min/Max")

    base["current_min_deg"] = current_min
    base["ready"] = True
    return JointWorkspaceRange(**base)


@dataclass
class HandModel:
    """
    灵巧手统一模型：状态与控制合一。
    - ENCODER_COUNT 个关节角编码器（JointEncoder），单独存放。
    - MOTOR_COUNT 个电机槽（MotorSlot），仅电机状态。
    - 触觉、校准状态、时间戳。
    """
    encoders: List[JointEncoder] = field(default_factory=_default_encoders)
    motors: List[MotorSlot] = field(default_factory=_default_motors)
    tactile_data: List[FingerTactile] = field(default_factory=list)
    tactile_summary_seq: Optional[int] = None
    tactile_full_seq: Optional[int] = None
    tactile_full_summary_seq: Optional[int] = None
    tactile_full_crc: Optional[int] = None
    tactile_full_crc_errors: int = 0
    tactile_full_seq_mismatches: int = 0
    tactile_full_dropped: int = 0
    tactile_source: str = TACTILE_SOURCE_NONE
    tactile_data_level: str = TACTILE_DATA_LEVEL_NONE
    tactile_module_valid_mask: int = 0
    tactile_diag_version: int = 0
    tactile_diag_frame_count: int = 0
    tactile_diag_enabled_mask: int = 0
    tactile_diag_summary_ok_mask: int = 0
    tactile_diag_full_ok_mask: int = 0
    tactile_diag_command_fail_mask: int = 0
    tactile_diag_response_fail_mask: int = 0
    tactile_diag_crc_fail_mask: int = 0
    tactile_diag_status_nonzero_mask: int = 0
    tactile_diag_summary_retry_recovered_mask: int = 0
    tactile_diag_summary_success_count: List[int] = field(default_factory=lambda: [0] * 15)
    tactile_diag_summary_failure_count: List[int] = field(default_factory=lambda: [0] * 15)
    tactile_diag_full_success_count: List[int] = field(default_factory=lambda: [0] * 15)
    tactile_diag_full_failure_count: List[int] = field(default_factory=lambda: [0] * 15)
    tactile_diag_summary_last_fail_reason: List[int] = field(default_factory=lambda: [0] * 15)
    tactile_diag_full_last_fail_reason: List[int] = field(default_factory=lambda: [0] * 15)
    tactile_diag_module_last_status: List[int] = field(default_factory=lambda: [0] * 15)
    tactile_diag_last_rx_ts: float = 0.0
    tactile_online: bool = False
    tactile_last_rx_ts: float = 0.0
    tactile_module_online: List[bool] = field(default_factory=lambda: [False] * (5 * TACTILE_SENSORS_PER_FINGER))
    tactile_module_last_rx_ts: List[float] = field(default_factory=lambda: [0.0] * (5 * TACTILE_SENSORS_PER_FINGER))
    calib_status: str = "IDLE"  # NO_CALIB, IDLE, PENDING, RUNNING, SUCCESS, WARN, FAILED
    # 上位机完成当前状态合并的 Unix 时间，仅用于 UI 新鲜度与 CSV host_time。
    host_rx_time: float = 0.0
    # Process-local receive times; wall timestamps remain available for logs.
    monotonic_received: dict[str, float] = field(default_factory=dict)
    # 当前状态最近一次有效 P4 数据包的 millis() 时间。0 表示尚未收到 P4 时间。
    p4_time_ms: int = 0
    has_p4_time: bool = False
    # 最近一帧 PACKET_TYPE_SERVO_ANGLE 自带的 P4 时间戳；记录舵机 ABS 时
    # 必须使用该值，不能借用随后到达的其他状态包时间。
    servo_feedback_p4_time_ms: int = 0
    servo_raw_p4_time_ms: int = 0
    servo_abs_last_rx_ts: float = 0.0
    # 控制跟踪由 P4 10 ms 控制周期一次性封装，禁止与其他状态包时间拼接。
    has_control_trace: bool = False
    control_trace_sequence: int = 0
    control_trace_target_token: int = 0
    control_trace_p4_time_ms: int = 0
    control_trace_target_accepted_ms: int = 0
    control_trace_encoder_timestamp_ms: int = 0
    control_trace_active_joint_mask: int = 0
    control_trace_valid_joint_mask: int = 0
    control_trace_motor_command_mask: int = 0
    control_trace_target_deg: List[float] = field(default_factory=lambda: [0.0] * ENCODER_COUNT)
    control_trace_actual_deg: List[float] = field(default_factory=lambda: [0.0] * ENCODER_COUNT)
    control_trace_motor_command_abs: List[int] = field(default_factory=lambda: [0] * MOTOR_COUNT)
    control_trace_last_rx_ts: float = 0.0
    control_trace_components: dict = field(default_factory=dict)
    control_trace_detail: dict = field(default_factory=dict)
    # 最近一次协议 ACK，用于把“UI 请求”和“P4 实际处理”分别写入事件日志。
    proto_ack_sequence: int = 0
    proto_ack_cmd: int = 0
    proto_ack_mode: int = 0
    proto_ack_status: int = 0
    proto_ack_p4_time_ms: int = 0
    # ACK 是离散事件；历史独立于 latest-state 邮箱，供事件日志逐条补记。
    proto_ack_history: List[dict] = field(default_factory=list)
    # 数据有效性标志：用于 UI 区分“无数据”和“值为 0”
    has_sensor_data: bool = False
    encoder_mapped_last_rx_ts: float = 0.0
    encoder_mapped_p4_time_ms: int = 0
    # 原始磁编包（PACKET_TYPE_ENCODER_RAW）与映射角包独立：未标定时 raw 仍应可见。
    has_encoder_raw_data: bool = False
    encoder_raw_last_rx_ts: float = 0.0
    encoder_raw_p4_time_ms: int = 0
    servo_telem_p4_time_ms: int = 0
    servo_telem_last_rx_ts: float = 0.0
    has_servo_angle_data: bool = False
    has_servo_raw_data: bool = False
    has_system_status_data: bool = False
    system_status_last_rx_ts: float = 0.0
    system_status_p4_time_ms: int = 0
    system_state: int = 0
    control_enabled: bool = False
    servo_target_owner: int = 0
    control_mode: int = 0
    system_fault_bitmap: int = 0
    state_transition_count: int = 0
    multi_turn_unknown_bitmap: int = 0
    # CMD_RESTORE_MULTITURN 的异步最终结果，不与“已入队”协议 ACK 混淆。
    multiturn_restore_result_token: int = 0
    multiturn_restore_restored_mask: int = 0
    multiturn_restore_rejected_mask: int = 0
    multiturn_restore_result_last_rx_ts: float = 0.0
    multiturn_restore_result_p4_time_ms: int = 0
    has_multiturn_restore_result: bool = False
    # 当前位置 ABS 设零的独立最终结果，QUEUED ACK 不代表硬件执行成功。
    servo_rebase_zero_result_token: int = 0
    servo_rebase_zero_success_mask: int = 0
    servo_rebase_zero_failed_mask: int = 0
    servo_rebase_zero_result_p4_time_ms: int = 0
    servo_rebase_zero_result_last_rx_ts: float = 0.0
    has_servo_rebase_zero_result: bool = False
    direct_target_token: int = 0
    direct_target_requested_mask: int = 0
    direct_target_applied_mask: int = 0
    direct_target_rejected_mask: int = 0
    direct_target_status: int = 0
    direct_target_result_rx_ts: float = 0.0
    has_direct_target_result: bool = False
    # 上位机 JSON 中的可信多圈锚点。该状态不来自 P4 协议，只用于 UI 展示和掉电恢复。
    host_multiturn_valid_mask: int = 0
    host_multiturn_saved_raw_positions: List[int] = field(
        default_factory=lambda: [0] * MOTOR_COUNT
    )
    host_multiturn_status: str = "not loaded"
    manual_reference_valid_bitmap: int = 0
    manual_reference_crc: int = 0
    # P4 的 Manual 表仅驻留 RAM；桌面端 calibration_bundle JSON 才负责跨重启保存。
    manual_reference_revision: int = 0
    manual_reference_committed: bool = False
    manual_reference_commit_status: int = 0
    auto_calibration_valid_bitmap: int = 0
    rope_init_status: int = 0
    rope_init_reason: int = 0
    rope_init_sequence: int = 0
    rope_init_joint_mask: int = 0
    rope_init_timestamp_ms: int = 0
    pending_direct_generation: int = 0
    applied_direct_generation: int = 0
    direct_reject_reason: int = 0
    has_motion_result_status: bool = False
    manual_reference_last_rx_time: float = 0.0
    has_manual_reference_status: bool = False
    manual_reference_min_raw: List[int] = field(default_factory=lambda: [0] * ENCODER_COUNT)
    manual_reference_max_raw: List[int] = field(default_factory=lambda: [0] * ENCODER_COUNT)
    manual_reference_encoder_directions: List[int] = field(default_factory=lambda: [0] * ENCODER_COUNT)
    has_manual_reference_encoder_directions: bool = False
    has_active_scope_status: bool = False
    active_motor_mask: int = 0
    active_joint_mask: int = 0
    require_can_feedback: bool = False
    has_connectivity_status: bool = False
    servo_online_bitmap: int = 0
    # P4 动态扫描得到的唯一可信路由。未收到该包时，上位机不得按旧固定 BUS 继续控制。
    has_servo_route_status: bool = False
    servo_route_controllable_bitmap: int = 0
    servo_route_conflict_bitmap: int = 0
    servo_bus_overload_mask: int = 0
    servo_route_scan_state: int = 0
    servo_route_scan_progress: int = 0
    servo_bus_device_count: List[int] = field(default_factory=lambda: [0] * 4)
    servo_route_bus: List[int] = field(default_factory=lambda: [0xFF] * MOTOR_COUNT)
    servo_route_state: List[int] = field(default_factory=lambda: [0] * MOTOR_COUNT)
    encoder_online_bitmap: int = 0
    encoder_warn_bitmap: int = 0
    encoder_fault_bitmap: int = 0
    has_encoder_health_status: bool = False
    calibration_warning_bitmap: int = 0
    calibration_auto_min_offset_counts: List[int] = field(
        default_factory=lambda: [0] * ENCODER_COUNT
    )
    has_calibration_warning_status: bool = False
    feedback_admission_sequence: int = 0
    feedback_admission_status: int = 0
    feedback_reject_reason: int = 0
    has_feedback_admission_status: bool = False
    feedback_collision_armed_bitmap: int = 0
    feedback_collision_hold_bitmap: int = 0
    feedback_collision_transition_count: int = 0
    has_feedback_collision_status: bool = False
    # hardware ABS 严格越过 ±30715 时触发 Reset Zero 事务。
    servo_turn_overflow_bitmap: int = 0
    servo_reset_zero_request_token: int = 0
    servo_reset_zero_result_token: int = 0
    servo_reset_zero_success_mask: int = 0
    servo_reset_zero_failed_mask: int = 0
    servo_reset_zero_invalidated_joint_mask: int = 0
    servo_reset_zero_status: int = 0
    has_servo_reset_zero_status: bool = False
    can_link_online: bool = False
    motor_abs_limit_valid_bitmap: int = 0
    motor_abs_limit_min: List[int] = field(default_factory=lambda: [0] * MOTOR_COUNT)
    motor_abs_limit_max: List[int] = field(default_factory=lambda: [0] * MOTOR_COUNT)
    # 每台舵机当前可信的 ABS 可达范围，与 ST3215 硬件多圈行程一致。
    servo_logical_range_valid_bitmap: int = 0
    servo_logical_range_last_rx_ts: float = 0.0
    servo_logical_range_p4_time_ms: int = 0
    servo_logical_range_min: List[int] = field(default_factory=lambda: [0] * MOTOR_COUNT)
    servo_logical_range_max: List[int] = field(default_factory=lambda: [0] * MOTOR_COUNT)
    joint_calib_results: List[Optional[dict]] = field(default_factory=lambda: [None] * ENCODER_COUNT)
    calib_diag_valid: bool = False
    calib_diag_reason: int = 0
    calib_diag_phase: int = 0
    calib_diag_joint: int = 0xFF
    calib_diag_motor: int = 0xFF
    calib_diag_value_a: int = 0
    calib_diag_value_b: int = 0
    calib_diag_value_c: int = 0
    calib_diag_bitmap: int = 0
    calib_diag_timestamp_ms: int = 0
    calib_diag_sequence: int = 0
    calib_diag_completed_joint_mask: int = 0
    calib_diag_failed_joint_mask: int = 0
    task_runtime: dict = field(default_factory=dict)
    task_runtime_last_rx_ts: float = 0.0
    # Task Runtime 七包窗口完整性诊断；完整快照和正在接收的分包必须分开记录。
    task_runtime_window_sequence: int = 0
    task_runtime_received_mask: int = 0
    task_runtime_expected_mask: int = 0
    task_runtime_dropped_windows: int = 0
    task_runtime_last_packet_rx_ts: float = 0.0
    # P4 周期上报的产品指纹，用于把运行日志绑定到确切的固件、参数和机器人模型。
    product_fingerprint: dict = field(default_factory=dict)
    has_product_fingerprint: bool = False
    protocol_compatible: bool = False
    # P4 安全规则事件只保留最近 64 条；sequence 用于去重和日志关联。
    latest_safety_event: dict = field(default_factory=dict)
    safety_event_history: List[dict] = field(default_factory=list)
    safety_event_overflow_count: int = 0
    safety_event_last_rx_ts: float = 0.0
    has_safety_event: bool = False

    # 伺服绝对角度来自 SERVO_ANGLE；在线状态统一来自 SYSTEM_STATUS 位图。
    servo_angles: List[int] = field(default_factory=lambda: [0] * MOTOR_COUNT)
    servo_online: List[bool] = field(default_factory=lambda: [False] * MOTOR_COUNT)
    # 当前单圈 raw 来自 100 Hz 流；Flash raw 来自保存结果触发的独立快照。
    servo_raw_positions: List[int] = field(default_factory=lambda: [0] * MOTOR_COUNT)
    servo_raw_online: List[bool] = field(default_factory=lambda: [False] * MOTOR_COUNT)

    # 伺服遥测数据（来自 PACKET_TYPE_SERVO_TELEM）
    servo_speed: List[int] = field(default_factory=lambda: [0] * MOTOR_COUNT)
    servo_load: List[int] = field(default_factory=lambda: [0] * MOTOR_COUNT)
    # ST3215 Present Current 原始有符号值，暂不假定其mA换算比例。
    servo_current: List[int] = field(default_factory=lambda: [0] * MOTOR_COUNT)
    servo_voltage: List[int] = field(default_factory=lambda: [0] * MOTOR_COUNT)
    servo_temperature: List[int] = field(default_factory=lambda: [0] * MOTOR_COUNT)
    servo_telem_online: List[bool] = field(default_factory=lambda: [False] * MOTOR_COUNT)
    has_fault_status_data: bool = False
    fault_status_last_rx_ts: float = 0.0
    # 伺服过载保护锁存状态（来自 PACKET_TYPE_FAULT_STATUS，按 motor channel 位图映射）
    servo_overload_fault: List[bool] = field(default_factory=lambda: [False] * MOTOR_COUNT)
    servo_overload_trigger_load: List[Optional[int]] = field(default_factory=lambda: [None] * MOTOR_COUNT)

    @property
    def angles(self) -> List[float]:
        """ENCODER_COUNT 个关节当前角度（度），来自编码器"""
        return [e.current_angle_deg for e in self.encoders]

    @property
    def encoder_angles_deg(self) -> List[float]:
        """与 angles 一致"""
        return self.angles

    @property
    def target_angles(self) -> List[float]:
        """ENCODER_COUNT 个关节目标角度（度）"""
        return [e.target_angle_deg for e in self.encoders]

    def set_target_angles(self, angles: List[float]) -> None:
        """设置全部编码器的目标角度（度）"""
        for i, a in enumerate(angles):
            if i < len(self.encoders):
                self.encoders[i].target_angle_deg = float(a)

    def received_at(self, field: str) -> float:
        """Monotonic receive time; missing telemetry is never treated as fresh."""
        return self.monotonic_received.get(field, 0.0)

    def copy_state_from(
        self,
        other: "HandModel",
        *,
        field_names: Optional[Tuple[str, ...]] = None,
    ) -> None:
        """Copy telemetry without replacing locally owned targets and encoder zeros.

        Mutable nested data (including tactile NumPy arrays) belongs to each snapshot.
        Encoder/motor objects keep their identity so callers can hold row references.
        ``field_names`` restricts the copy to an explicit subset of top-level fields;
        ``encoders``/``motors`` are only synchronised when named explicitly.
        """
        source = other.__dict__
        target = self.__dict__
        if field_names is None:
            own_encoders = target["encoders"]
            own_motors = target["motors"]
            # Scalar fields are immutable values, so a single bulk reference copy is
            # sufficient; only the known container fields need fresh copies below.
            target.update(source)
            target["encoders"] = own_encoders
            target["motors"] = own_motors
            for name in _HAND_FLAT_CONTAINER_FIELDS:
                value = source[name]
                value_type = type(value)
                if value_type is list or value_type is dict:
                    target[name] = value.copy()
                else:
                    target[name] = _copy_state_value(value)
            for name in _HAND_NESTED_CONTAINER_FIELDS:
                target[name] = _copy_state_value(source[name])
            _sync_encoders(own_encoders, source["encoders"])
            _sync_motors(own_motors, source["motors"])
            return

        requested = tuple(field_names)
        for name in requested:
            if name == "encoders" or name == "motors":
                continue
            if name in _HAND_FLAT_CONTAINER_FIELD_SET:
                value = source[name]
                value_type = type(value)
                if value_type is list or value_type is dict:
                    target[name] = value.copy()
                else:
                    target[name] = _copy_state_value(value)
            elif name in _HAND_NESTED_CONTAINER_FIELD_SET:
                target[name] = _copy_state_value(source[name])
            else:
                target[name] = source[name]
        if "encoders" in requested:
            _sync_encoders(target["encoders"], source["encoders"])
        if "motors" in requested:
            _sync_motors(target["motors"], source["motors"])


def _annotation_is_scalar(annotation) -> bool:
    """True when one list element / dict value annotation is an immutable scalar."""
    if annotation in (int, float, bool, str, bytes) or annotation is type(None):
        return True
    origin = get_origin(annotation)
    if origin is Union or origin is getattr(types, "UnionType", Union):
        return all(_annotation_is_scalar(arg) for arg in get_args(annotation))
    return False


def _classify_hand_state_fields() -> tuple[tuple[str, ...], tuple[str, ...]]:
    """Split HandModel state fields into flat scalar containers and nested ones.

    Flat containers may be duplicated with ``list.copy()``/``dict.copy()`` because
    every element is an immutable scalar; nested containers need the recursive copy.
    Classification is annotation based and falls back to the nested path when a
    field's annotation is unknown, so newly added fields stay isolated by default.
    """
    hints = get_type_hints(HandModel)
    flat: list[str] = []
    nested: list[str] = []
    union_types = (Union, getattr(types, "UnionType", Union))
    for item in fields(HandModel):
        name = item.name
        if name in ("encoders", "motors"):
            continue
        annotation = hints.get(name)
        origin = get_origin(annotation)
        if origin is list:
            args = get_args(annotation)
            (flat if len(args) == 1 and _annotation_is_scalar(args[0]) else nested).append(name)
        elif origin is dict:
            args = get_args(annotation)
            (flat if len(args) == 2 and _annotation_is_scalar(args[1]) else nested).append(name)
        elif origin in union_types:
            # Optional[List[...]] 等联合容器保守走递归拷贝，避免共享可变引用。
            nested.append(name)
        elif annotation in (list, dict, np.ndarray):
            nested.append(name)
    return tuple(flat), tuple(nested)


def _sync_encoders(targets: List[JointEncoder], sources: List[JointEncoder]) -> None:
    """Update encoder objects in place; targets/zeros stay locally owned."""
    for target, source in zip(targets, sources):
        target_dict = target.__dict__
        keep_target_angle_deg = target_dict["target_angle_deg"]
        keep_zero_raw = target_dict["zero_raw"]
        target_dict.update(source.__dict__)
        target_dict["target_angle_deg"] = keep_target_angle_deg
        target_dict["zero_raw"] = keep_zero_raw


def _sync_motors(targets: List[MotorSlot], sources: List[MotorSlot]) -> None:
    """Update motor slot objects in place to preserve row-reference identity."""
    for target, source in zip(targets, sources):
        target.__dict__.update(source.__dict__)


_HAND_FLAT_CONTAINER_FIELDS, _HAND_NESTED_CONTAINER_FIELDS = _classify_hand_state_fields()
_HAND_FLAT_CONTAINER_FIELD_SET = frozenset(_HAND_FLAT_CONTAINER_FIELDS)
_HAND_NESTED_CONTAINER_FIELD_SET = frozenset(_HAND_NESTED_CONTAINER_FIELDS)


def _new_hand_objects(sources) -> List:
    """Shallow-clone simple state dataclasses (all fields are immutable scalars)."""
    result = []
    for source in sources:
        clone = object.__new__(type(source))
        clone.__dict__.update(source.__dict__)
        result.append(clone)
    return result


def clone_hand_model(model: HandModel) -> HandModel:
    """建立跨线程只读快照，集中维护目标、零点和状态字段复制。"""
    snapshot = copy(model)
    source = model.__dict__
    target = snapshot.__dict__
    target["encoders"] = _new_hand_objects(source["encoders"])
    target["motors"] = _new_hand_objects(source["motors"])
    for name in _HAND_FLAT_CONTAINER_FIELDS:
        value = source[name]
        value_type = type(value)
        if value_type is list or value_type is dict:
            target[name] = value.copy()
        else:
            target[name] = _copy_state_value(value)
    for name in _HAND_NESTED_CONTAINER_FIELDS:
        target[name] = _copy_state_value(source[name])
    return snapshot
