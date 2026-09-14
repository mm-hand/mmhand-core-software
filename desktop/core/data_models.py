"""data_models.py - MMHand Desktop 模块契约。

职责：定义上位机共享的设备状态、标定状态和遥测数据模型。
线程：除 comm_layer 的串口收发线程外不主动创建 GUI 线程；共享状态必须按既有锁读取。
输入：P4 字节流、上层类型化命令、配置和持久化数据。
输出：线程安全的 HandModel 快照、协议帧、ACK 和后端操作结果。
状态所有权：通信对象拥有串口与解析缓存；HandModel 的写入只允许在核心同步边界内发生。
禁止依赖：禁止依赖 Qt/UI、app、ROS2 或 Replay 页面，禁止把控件和展示规则带入核心层。
"""
# data_models.py - 灵巧手数据模型（状态与控制合一）

from dataclasses import dataclass, field
import math
from typing import List, Optional, Tuple, NamedTuple
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
# 条带 0/2/4 → sensors[0]/[1]/[2]（尖、中腹、近腹）；关节条带无数据
TACTILE_BAND_SENSOR_INDEX = {0: 0, 2: 1, 4: 2}
TACTILE_BAND_WIDTH = 15
# 自上而下（指尖→指根）：25×15 指尖 +12×15 关节 +16×15 指腹 +12×15 关节 +16×15 指腹
TACTILE_BAND_HEIGHTS = (25, 12, 16, 12, 16)
TACTILE_FINGER_STRIP_ROWS = sum(TACTILE_BAND_HEIGHTS)  # 81
TACTILE_NBANDS = len(TACTILE_BAND_HEIGHTS)
# 条带索引 1、3 为关节位（无触觉矩阵）；热力图填 TACTILE_HEATMAP_STRUCTURE_VALUE 显示浅灰；3D 着色不采用其读数
TACTILE_JOINT_BAND_INDICES = (1, 3)
# 热力图专用：关节条带、拇指近端指腹–掌间连接带等，经 BoundaryNorm 映射为浅灰
TACTILE_HEATMAP_STRUCTURE_VALUE = -1.0
TACTILE_HEATMAP_STRUCTURE_BOUNDS = (-1.01, -0.99)  # 包含 STRUCTURE_VALUE 的区间端点
TACTILE_HEATMAP_GRAY_RGBA = (0.82, 0.82, 0.82, 1.0)  # 间隙/关节等浅灰
# 热力图掌心：填充 PALM_VALUE，色标用较深灰，与浅灰结构区分
TACTILE_HEATMAP_PALM_VALUE = -2.0
TACTILE_HEATMAP_PALM_BOUNDS = (-2.01, -1.99)
TACTILE_HEATMAP_PALM_RGBA = (0.50, 0.50, 0.50, 1.0)  # 掌心深灰（中性灰）
TACTILE_PALM_SIDE = 15 * 4 + 5 * 3  # 75，掌宽
# 食指～小指条带之间的列间隔；拇指近端指腹带与掌左缘之间浅灰列数（仅该节行域内绘制）
TACTILE_FOUR_FINGER_GAP = 5
TACTILE_THUMB_PALM_GAP = 8
# 竖直拇指整体上移的行数（行索引减小，正值=向图像上方移动）
TACTILE_THUMB_VERTICAL_SHIFT_UP = 20
# 四指指根–掌顶之间浅灰间隙行数；掌面竖直有效高度 = TACTILE_PALM_SIDE - 本值
TACTILE_FOUR_FINGER_PALM_GAP = 6
# 掌面由正方形在下半部左右各切除一个直角三角形（直角在左下/右下顶点）而成六边形；
# 每条直角边沿掌底/掌侧占用的格数，实际使用时会限制为不超过掌边长一半以免两角相交
TACTILE_PALM_BOTTOM_CORNER_CUT_LEG = 15

def tactile_band_shape(band_index: int) -> Tuple[int, int]:
    """band 0=指尖 … 4=近端指腹，均为 (高, 15)。"""
    if band_index < 0 or band_index >= TACTILE_NBANDS:
        raise ValueError(f"band_index must be within 0..{TACTILE_NBANDS - 1}")
    return (TACTILE_BAND_HEIGHTS[band_index], TACTILE_BAND_WIDTH)


def tactile_segment_shape(seg_index: int) -> Tuple[int, int]:
    """兼容旧 API：2→指尖元栅格 5×5；0/1→指腹元栅格 4×13。"""
    if seg_index == 2:
        return (TACTILE_TIP_ROWS, TACTILE_TIP_COLS)
    return (TACTILE_PAD_ROWS, TACTILE_PAD_COLS)


def tactile_upsample_tip_grid_to_band0(ch2: np.ndarray) -> np.ndarray:
    """
    指尖逻辑栅格 5×5（或 25 元扁平）→ 热力图指尖条带 25×15：每行重复 ×5、每列重复 ×3。
    """
    g = np.asarray(ch2, dtype=float)
    if g.size == TACTILE_TIP_TAXEL_COUNT and g.shape != (TACTILE_TIP_ROWS, TACTILE_TIP_COLS):
        g = g.reshape(TACTILE_TIP_ROWS, TACTILE_TIP_COLS)
    elif g.shape != (TACTILE_TIP_ROWS, TACTILE_TIP_COLS):
        t = np.zeros((TACTILE_TIP_ROWS, TACTILE_TIP_COLS), dtype=float)
        r0, c0 = min(TACTILE_TIP_ROWS, g.shape[0]), min(TACTILE_TIP_COLS, g.shape[1])
        t[:r0, :c0] = g[:r0, :c0]
        g = t
    return np.repeat(np.repeat(g, 5, axis=0), 3, axis=1)


def tactile_upsample_pad_grid_to_band(ch2: np.ndarray, out_h: int, out_w: int) -> np.ndarray:
    """
    指腹逻辑栅格 4×13（或 52 元扁平）→ 热力图指腹条带 out_h×out_w（通常 16×15）：行每行重复 ×4，列左右各填 1 列对齐 15。
    """
    g = np.asarray(ch2, dtype=float)
    if g.size == TACTILE_PAD_TAXEL_COUNT and g.shape != (TACTILE_PAD_ROWS, TACTILE_PAD_COLS):
        g = g.reshape(TACTILE_PAD_ROWS, TACTILE_PAD_COLS)
    elif g.shape != (TACTILE_PAD_ROWS, TACTILE_PAD_COLS):
        t = np.zeros((TACTILE_PAD_ROWS, TACTILE_PAD_COLS), dtype=float)
        r0, c0 = min(TACTILE_PAD_ROWS, g.shape[0]), min(TACTILE_PAD_COLS, g.shape[1])
        t[:r0, :c0] = g[:r0, :c0]
        g = t
    u = np.repeat(g, 4, axis=0)
    if u.shape[0] > out_h:
        u = u[:out_h, :]
    elif u.shape[0] < out_h:
        u = np.pad(u, ((0, out_h - u.shape[0]), (0, 0)), mode="edge")
    inner_w = min(u.shape[1], out_w)
    pad = max(0, (out_w - inner_w) // 2)
    out = np.zeros((out_h, out_w), dtype=float)
    out[:, pad : pad + inner_w] = u[:, :inner_w]
    return out


def tactile_palm_left_bottom_cut_top_vertex_rc(
    pr: int,
    pc: int,
    palm_h: int,
    palm_w: int,
    leg: Optional[int] = None,
) -> Tuple[int, int]:
    """
    掌面左下裁切的等腰直角三角形在「掌左侧竖直边」上的顶点（非直角顶点、靠近斜边与左缘交点一侧），
    与当前掌面六边形的阶梯边界一致：最上一行被挖行的下标为 pr + palm_h - leg，
    该顶点取为 (row, col) = (pr + palm_h - leg, pc)。
    leg 与裁切函数同样截断到 palm_w//2、palm_h//2。
    """
    if leg is None:
        leg = TACTILE_PALM_BOTTOM_CORNER_CUT_LEG
    leg = min(int(leg), palm_w // 2, palm_h // 2)
    if leg <= 0:
        return pr + palm_h - 1, pc
    r_v = pr + palm_h - leg
    c_v = pc
    return r_v, c_v


class TactileStitchSpec(NamedTuple):
    """整手拼接画布（相对坐标，原点在画布左上角）。"""

    height: int
    width: int
    palm_r: int
    palm_c: int
    palm_h: int
    palm_w: int
    thumb_r: int
    thumb_c: int
    thumb_h: int
    thumb_w: int
    four_r: int
    four_c: int


def build_tactile_stitch_spec() -> TactileStitchSpec:
    """
    布局：掌面 palm_h×palm_w；裁左下/右下两角成六边形。
    竖直拇指：先按顶点对齐定 thumb_r/thumb_c，再整体上移 TACTILE_THUMB_VERTICAL_SHIFT_UP 行。
    thumb_g=0 时对齐后指根右下与裁切顶点 (vr,vc) 重合再上移；thumb_g>0 时最右列与 pc 间留 thumb_g 列（浅灰仅在近端指腹行带绘制，见 ui）。
    四指在掌上沿，指根–掌顶 gv 行浅灰；指间在掌宽内为掌心深灰。
    """
    margin = 8
    palm_r0, palm_c0 = 80, 90
    sw = TACTILE_PALM_SIDE
    gv = TACTILE_FOUR_FINGER_PALM_GAP
    sh = sw - gv
    g = TACTILE_FOUR_FINGER_GAP
    thumb_g = TACTILE_THUMB_PALM_GAP
    four_span = 4 * TACTILE_BAND_WIDTH + 3 * g  # 四指总宽大（含指间 3 个间隙）
    anchor_r = palm_r0 + sh // 2
    anchor_c = palm_c0
    h_strip = TACTILE_FINGER_STRIP_ROWS
    w_strip = TACTILE_BAND_WIDTH
    vr, vc = tactile_palm_left_bottom_cut_top_vertex_rc(palm_r0, palm_c0, sh, sw)
    thumb_r = vr - (h_strip - 1)
    if thumb_g <= 0:
        # 指根右下与裁切顶点同格
        thumb_c = vc - (w_strip - 1)
    else:
        # 指根右下列为 vc - thumb_g - 1，与掌左缘 pc=vc 之间留 thumb_g 列
        thumb_c = vc - w_strip - thumb_g
    thumb_r -= TACTILE_THUMB_VERTICAL_SHIFT_UP
    four_r = palm_r0 - TACTILE_FINGER_STRIP_ROWS - gv
    four_c = palm_c0 + (sw - four_span) // 2
    min_r = min(thumb_r, four_r, palm_r0) - margin
    min_c = min(thumb_c, four_c, palm_c0) - margin
    max_r = max(palm_r0 + sh - 1, anchor_r, four_r + TACTILE_FINGER_STRIP_ROWS - 1) + margin
    max_c = max(palm_c0 + sw - 1, thumb_c + TACTILE_BAND_WIDTH - 1, four_c + four_span - 1) + margin
    H = max_r - min_r + 1
    W = max_c - min_c + 1

    return TactileStitchSpec(
        height=H,
        width=W,
        palm_r=palm_r0 - min_r,
        palm_c=palm_c0 - min_c,
        palm_h=sh,
        palm_w=sw,
        thumb_r=thumb_r - min_r,
        thumb_c=thumb_c - min_c,
        thumb_h=TACTILE_FINGER_STRIP_ROWS,
        thumb_w=TACTILE_BAND_WIDTH,
        four_r=four_r - min_r,
        four_c=four_c - min_c,
    )


TACTILE_STITCH_SPEC = build_tactile_stitch_spec()

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
    """单指 3 组数据：sensors[0] 尖 25，[1][2] 两腹各 52；热力图仍为 5 条带几何（尖/腹上采样嵌入）"""
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
    joint_state_p4_time_ms: Optional[int] = None
    joint_state_host_rx_ns: int = 0
    joint_state_host_rx_monotonic_ns: int = 0
    joint_state_sequence: int = 0
    joint_state_valid: bool = False
    # 当前状态最近一次有效 P4 数据包的 millis() 时间。0 表示尚未收到 P4 时间。
    p4_time_ms: int = 0
    has_p4_time: bool = False
    # 最近一帧 PACKET_TYPE_SERVO_ANGLE 自带的 P4 时间戳；记录舵机 ABS 时
    # 必须使用该值，不能借用随后到达的其他状态包时间。
    servo_feedback_p4_time_ms: int = 0
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
    # 原始磁编包（PACKET_TYPE_ENCODER_RAW）与映射角包独立：未标定时 raw 仍应可见。
    has_encoder_raw_data: bool = False
    encoder_raw_last_rx_ts: float = 0.0
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
    # 每台舵机当前上电会话的 logical ABS 可达范围。该范围由
    # logicalOffset + ST3215 硬件多圈行程得到，单台舵机掉电只改变对应一项。
    servo_logical_range_valid_bitmap: int = 0
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

    def copy_state_from(self, other: "HandModel") -> None:
        """仅复制状态（编码器 raw/error、电机 error/state/伺服遥测），不覆盖 target_angle_deg、zero_raw"""
        self.host_rx_time = float(getattr(other, "host_rx_time", 0.0) or 0.0)
        for name in ("joint_state_p4_time_ms", "joint_state_host_rx_ns",
                     "joint_state_host_rx_monotonic_ns", "joint_state_sequence",
                     "joint_state_valid"):
            setattr(self, name, getattr(other, name, getattr(self, name)))
        self.p4_time_ms = int(getattr(other, "p4_time_ms", 0) or 0)
        self.has_p4_time = bool(getattr(other, "has_p4_time", False))
        self.servo_abs_last_rx_ts = float(getattr(other, "servo_abs_last_rx_ts", 0.0))
        self.servo_feedback_p4_time_ms = int(
            getattr(other, "servo_feedback_p4_time_ms", 0) or 0
        )
        self.has_control_trace = bool(getattr(other, "has_control_trace", False))
        self.control_trace_sequence = int(getattr(other, "control_trace_sequence", 0) or 0)
        self.control_trace_target_token = int(getattr(other, "control_trace_target_token", 0) or 0)
        self.control_trace_p4_time_ms = int(getattr(other, "control_trace_p4_time_ms", 0) or 0)
        self.control_trace_target_accepted_ms = int(
            getattr(other, "control_trace_target_accepted_ms", 0) or 0
        )
        self.control_trace_encoder_timestamp_ms = int(
            getattr(other, "control_trace_encoder_timestamp_ms", 0) or 0
        )
        self.control_trace_active_joint_mask = int(
            getattr(other, "control_trace_active_joint_mask", 0) or 0
        )
        self.control_trace_valid_joint_mask = int(
            getattr(other, "control_trace_valid_joint_mask", 0) or 0
        )
        self.control_trace_motor_command_mask = int(
            getattr(other, "control_trace_motor_command_mask", 0) or 0
        )
        self.control_trace_target_deg = list(
            getattr(other, "control_trace_target_deg", [0.0] * ENCODER_COUNT)
        )
        self.control_trace_actual_deg = list(
            getattr(other, "control_trace_actual_deg", [0.0] * ENCODER_COUNT)
        )
        self.control_trace_motor_command_abs = list(
            getattr(other, "control_trace_motor_command_abs", [0] * MOTOR_COUNT)
        )
        self.control_trace_last_rx_ts = float(
            getattr(other, "control_trace_last_rx_ts", 0.0) or 0.0
        )
        self.proto_ack_sequence = int(getattr(other, "proto_ack_sequence", 0) or 0)
        self.proto_ack_cmd = int(getattr(other, "proto_ack_cmd", 0) or 0)
        self.proto_ack_mode = int(getattr(other, "proto_ack_mode", 0) or 0)
        self.proto_ack_status = int(getattr(other, "proto_ack_status", 0) or 0)
        self.proto_ack_p4_time_ms = int(getattr(other, "proto_ack_p4_time_ms", 0) or 0)
        self.proto_ack_history = [
            dict(item)
            for item in getattr(other, "proto_ack_history", [])
            if isinstance(item, dict)
        ][-64:]
        for i, e in enumerate(self.encoders):
            if i < len(other.encoders):
                e.raw = other.encoders[i].raw
                e.raw_sensor = getattr(other.encoders[i], "raw_sensor", other.encoders[i].raw)
                e.error = other.encoders[i].error
                e.angle_error = bool(getattr(other.encoders[i], "angle_error", e.error))
                e.raw_error = bool(getattr(other.encoders[i], "raw_error", e.error))
                e.angle_out_of_workspace = bool(
                    getattr(other.encoders[i], "angle_out_of_workspace", False)
                )
        for i, m in enumerate(self.motors):
            if i < len(other.motors):
                m.error = other.motors[i].error
                m.state = other.motors[i].state
        # 复制伺服绝对角度与遥测
        self.servo_angles = list(other.servo_angles)
        self.servo_online = list(other.servo_online)
        self.servo_raw_positions = list(other.servo_raw_positions)
        self.servo_raw_online = list(other.servo_raw_online)
        self.servo_speed = list(other.servo_speed)
        self.servo_load = list(other.servo_load)
        self.servo_current = list(other.servo_current)
        self.servo_voltage = list(other.servo_voltage)
        self.servo_temperature = list(other.servo_temperature)
        self.servo_telem_online = list(other.servo_telem_online)
        self.servo_overload_fault = list(other.servo_overload_fault)
        self.servo_overload_trigger_load = list(getattr(other, "servo_overload_trigger_load", [None] * MOTOR_COUNT))
        # 复制数据有效性标志
        self.has_sensor_data = bool(other.has_sensor_data)
        self.has_encoder_raw_data = bool(getattr(other, "has_encoder_raw_data", False))
        self.encoder_raw_last_rx_ts = float(getattr(other, "encoder_raw_last_rx_ts", 0.0) or 0.0)
        self.has_servo_angle_data = bool(other.has_servo_angle_data)
        self.has_servo_raw_data = bool(other.has_servo_raw_data)
        self.has_system_status_data = bool(other.has_system_status_data)
        self.system_status_last_rx_ts = float(getattr(other, "system_status_last_rx_ts", 0.0) or 0.0)
        self.system_status_p4_time_ms = int(
            getattr(other, "system_status_p4_time_ms", 0) or 0
        )
        self.system_state = int(other.system_state)
        self.control_enabled = bool(other.control_enabled)
        self.servo_target_owner = int(other.servo_target_owner)
        self.control_mode = int(other.control_mode)
        self.system_fault_bitmap = int(other.system_fault_bitmap)
        self.state_transition_count = int(other.state_transition_count)
        self.multi_turn_unknown_bitmap = int(getattr(other, "multi_turn_unknown_bitmap", 0))
        self.multiturn_restore_result_token = int(
            getattr(other, "multiturn_restore_result_token", 0) or 0
        )
        self.multiturn_restore_restored_mask = int(
            getattr(other, "multiturn_restore_restored_mask", 0) or 0
        )
        self.multiturn_restore_rejected_mask = int(
            getattr(other, "multiturn_restore_rejected_mask", 0) or 0
        )
        self.multiturn_restore_result_last_rx_ts = float(
            getattr(other, "multiturn_restore_result_last_rx_ts", 0.0) or 0.0
        )
        self.host_multiturn_valid_mask = int(
            getattr(other, "host_multiturn_valid_mask", 0) or 0
        )
        self.host_multiturn_saved_raw_positions = list(
            getattr(
                other,
                "host_multiturn_saved_raw_positions",
                [0] * MOTOR_COUNT,
            )
        )
        self.host_multiturn_status = str(
            getattr(other, "host_multiturn_status", "not loaded")
        )
        self.manual_reference_valid_bitmap = int(getattr(other, "manual_reference_valid_bitmap", 0) or 0)
        self.manual_reference_crc = int(getattr(other, "manual_reference_crc", 0) or 0)
        self.manual_reference_revision = int(getattr(other, "manual_reference_revision", 0) or 0)
        self.manual_reference_committed = bool(getattr(other, "manual_reference_committed", False))
        self.manual_reference_commit_status = int(
            getattr(other, "manual_reference_commit_status", 0) or 0
        )
        self.auto_calibration_valid_bitmap = int(
            getattr(other, "auto_calibration_valid_bitmap", 0) or 0
        )
        self.rope_init_status = int(getattr(other, "rope_init_status", 0) or 0)
        self.rope_init_reason = int(getattr(other, "rope_init_reason", 0) or 0)
        self.rope_init_sequence = int(getattr(other, "rope_init_sequence", 0) or 0)
        self.rope_init_joint_mask = int(getattr(other, "rope_init_joint_mask", 0) or 0)
        self.rope_init_timestamp_ms = int(
            getattr(other, "rope_init_timestamp_ms", 0) or 0
        )
        self.pending_direct_generation = int(
            getattr(other, "pending_direct_generation", 0) or 0
        )
        self.applied_direct_generation = int(
            getattr(other, "applied_direct_generation", 0) or 0
        )
        self.direct_reject_reason = int(getattr(other, "direct_reject_reason", 0) or 0)
        self.has_motion_result_status = bool(
            getattr(other, "has_motion_result_status", False)
        )
        self.feedback_collision_armed_bitmap = int(
            getattr(other, "feedback_collision_armed_bitmap", 0) or 0
        )
        self.feedback_collision_hold_bitmap = int(
            getattr(other, "feedback_collision_hold_bitmap", 0) or 0
        )
        self.feedback_collision_transition_count = int(
            getattr(other, "feedback_collision_transition_count", 0) or 0
        )
        self.has_feedback_collision_status = bool(
            getattr(other, "has_feedback_collision_status", False)
        )
        self.servo_turn_overflow_bitmap = int(
            getattr(other, "servo_turn_overflow_bitmap", 0) or 0
        )
        self.servo_reset_zero_request_token = int(
            getattr(other, "servo_reset_zero_request_token", 0) or 0
        )
        self.servo_reset_zero_result_token = int(
            getattr(other, "servo_reset_zero_result_token", 0) or 0
        )
        self.servo_reset_zero_success_mask = int(
            getattr(other, "servo_reset_zero_success_mask", 0) or 0
        )
        self.servo_reset_zero_failed_mask = int(
            getattr(other, "servo_reset_zero_failed_mask", 0) or 0
        )
        self.servo_reset_zero_invalidated_joint_mask = int(
            getattr(other, "servo_reset_zero_invalidated_joint_mask", 0) or 0
        )
        self.servo_reset_zero_status = int(
            getattr(other, "servo_reset_zero_status", 0) or 0
        )
        self.has_servo_reset_zero_status = bool(
            getattr(other, "has_servo_reset_zero_status", False)
        )
        self.manual_reference_last_rx_time = float(
            getattr(other, "manual_reference_last_rx_time", 0.0) or 0.0
        )
        self.has_manual_reference_status = bool(getattr(other, "has_manual_reference_status", False))
        self.manual_reference_min_raw = list(
            getattr(other, "manual_reference_min_raw", [0] * ENCODER_COUNT)
        )
        self.manual_reference_max_raw = list(
            getattr(other, "manual_reference_max_raw", [0] * ENCODER_COUNT)
        )
        self.manual_reference_encoder_directions = list(
            getattr(other, "manual_reference_encoder_directions", [0] * ENCODER_COUNT)
        )
        self.has_manual_reference_encoder_directions = bool(
            getattr(other, "has_manual_reference_encoder_directions", False)
        )
        self.has_active_scope_status = bool(getattr(other, "has_active_scope_status", False))
        self.active_motor_mask = int(getattr(other, "active_motor_mask", 0) or 0)
        self.active_joint_mask = int(getattr(other, "active_joint_mask", 0) or 0)
        self.require_can_feedback = bool(getattr(other, "require_can_feedback", False))
        self.has_connectivity_status = bool(getattr(other, "has_connectivity_status", False))
        self.servo_online_bitmap = int(getattr(other, "servo_online_bitmap", 0) or 0)
        self.has_servo_route_status = bool(
            getattr(other, "has_servo_route_status", False)
        )
        self.servo_route_controllable_bitmap = int(
            getattr(other, "servo_route_controllable_bitmap", 0) or 0
        )
        self.servo_route_conflict_bitmap = int(
            getattr(other, "servo_route_conflict_bitmap", 0) or 0
        )
        self.servo_bus_overload_mask = int(
            getattr(other, "servo_bus_overload_mask", 0) or 0
        )
        self.servo_route_scan_state = int(
            getattr(other, "servo_route_scan_state", 0) or 0
        )
        self.servo_route_scan_progress = int(
            getattr(other, "servo_route_scan_progress", 0) or 0
        )
        self.servo_bus_device_count = list(
            getattr(other, "servo_bus_device_count", [0] * 4)
        )
        self.servo_route_bus = list(
            getattr(other, "servo_route_bus", [0xFF] * MOTOR_COUNT)
        )
        self.servo_route_state = list(
            getattr(other, "servo_route_state", [0] * MOTOR_COUNT)
        )
        self.encoder_online_bitmap = int(getattr(other, "encoder_online_bitmap", 0) or 0)
        self.encoder_warn_bitmap = int(getattr(other, "encoder_warn_bitmap", 0) or 0)
        self.encoder_fault_bitmap = int(getattr(other, "encoder_fault_bitmap", 0) or 0)
        self.has_encoder_health_status = bool(getattr(other, "has_encoder_health_status", False))
        self.calibration_warning_bitmap = int(
            getattr(other, "calibration_warning_bitmap", 0) or 0
        )
        self.calibration_auto_min_offset_counts = list(
            getattr(other, "calibration_auto_min_offset_counts", [0] * ENCODER_COUNT)
        )
        self.has_calibration_warning_status = bool(
            getattr(other, "has_calibration_warning_status", False)
        )
        self.feedback_admission_sequence = int(
            getattr(other, "feedback_admission_sequence", 0) or 0
        )
        self.feedback_admission_status = int(
            getattr(other, "feedback_admission_status", 0) or 0
        )
        self.feedback_reject_reason = int(
            getattr(other, "feedback_reject_reason", 0) or 0
        )
        self.has_feedback_admission_status = bool(
            getattr(other, "has_feedback_admission_status", False)
        )
        self.can_link_online = bool(getattr(other, "can_link_online", False))
        self.has_fault_status_data = bool(getattr(other, "has_fault_status_data", False))
        self.fault_status_last_rx_ts = float(getattr(other, "fault_status_last_rx_ts", 0.0) or 0.0)
        self.motor_abs_limit_valid_bitmap = int(getattr(other, "motor_abs_limit_valid_bitmap", 0))
        self.motor_abs_limit_min = list(getattr(other, "motor_abs_limit_min", [0] * MOTOR_COUNT))
        self.motor_abs_limit_max = list(getattr(other, "motor_abs_limit_max", [0] * MOTOR_COUNT))
        self.servo_logical_range_valid_bitmap = int(
            getattr(other, "servo_logical_range_valid_bitmap", 0)
        )
        self.servo_logical_range_min = list(
            getattr(other, "servo_logical_range_min", [0] * MOTOR_COUNT)
        )
        self.servo_logical_range_max = list(
            getattr(other, "servo_logical_range_max", [0] * MOTOR_COUNT)
        )
        self.joint_calib_results = [
            dict(item) if isinstance(item, dict) else None
            for item in getattr(other, "joint_calib_results", [None] * ENCODER_COUNT)
        ]
        self.calib_diag_valid = bool(getattr(other, "calib_diag_valid", False))
        self.calib_diag_reason = int(getattr(other, "calib_diag_reason", 0) or 0)
        self.calib_diag_phase = int(getattr(other, "calib_diag_phase", 0) or 0)
        self.calib_diag_joint = int(getattr(other, "calib_diag_joint", 0xFF) or 0)
        self.calib_diag_motor = int(getattr(other, "calib_diag_motor", 0xFF) or 0)
        self.calib_diag_value_a = int(getattr(other, "calib_diag_value_a", 0) or 0)
        self.calib_diag_value_b = int(getattr(other, "calib_diag_value_b", 0) or 0)
        self.calib_diag_value_c = int(getattr(other, "calib_diag_value_c", 0) or 0)
        self.calib_diag_bitmap = int(getattr(other, "calib_diag_bitmap", 0) or 0)
        self.calib_diag_timestamp_ms = int(getattr(other, "calib_diag_timestamp_ms", 0) or 0)
        self.calib_diag_sequence = int(getattr(other, "calib_diag_sequence", 0) or 0)
        self.calib_diag_completed_joint_mask = int(
            getattr(other, "calib_diag_completed_joint_mask", 0) or 0
        )
        self.calib_diag_failed_joint_mask = int(
            getattr(other, "calib_diag_failed_joint_mask", 0) or 0
        )
        self.task_runtime = {
            int(task_id): dict(item)
            for task_id, item in getattr(other, "task_runtime", {}).items()
            if isinstance(item, dict)
        }
        self.task_runtime_last_rx_ts = float(getattr(other, "task_runtime_last_rx_ts", 0.0) or 0.0)
        self.task_runtime_window_sequence = int(
            getattr(other, "task_runtime_window_sequence", 0) or 0
        )
        self.task_runtime_received_mask = int(
            getattr(other, "task_runtime_received_mask", 0) or 0
        )
        self.task_runtime_expected_mask = int(
            getattr(other, "task_runtime_expected_mask", 0) or 0
        )
        self.task_runtime_dropped_windows = int(
            getattr(other, "task_runtime_dropped_windows", 0) or 0
        )
        self.task_runtime_last_packet_rx_ts = float(
            getattr(other, "task_runtime_last_packet_rx_ts", 0.0) or 0.0
        )
        self.product_fingerprint = dict(
            getattr(other, "product_fingerprint", {}) or {}
        )
        self.has_product_fingerprint = bool(
            getattr(other, "has_product_fingerprint", False)
        )
        self.latest_safety_event = dict(
            getattr(other, "latest_safety_event", {}) or {}
        )
        self.safety_event_history = [
            dict(item)
            for item in getattr(other, "safety_event_history", [])
            if isinstance(item, dict)
        ][-64:]
        self.safety_event_overflow_count = int(
            getattr(other, "safety_event_overflow_count", 0) or 0
        )
        self.safety_event_last_rx_ts = float(
            getattr(other, "safety_event_last_rx_ts", 0.0) or 0.0
        )
        self.has_safety_event = bool(
            getattr(other, "has_safety_event", False)
        )
        self.tactile_data = list(getattr(other, "tactile_data", []) or [])
        self.tactile_summary_seq = getattr(other, "tactile_summary_seq", None)
        self.tactile_full_seq = getattr(other, "tactile_full_seq", None)
        self.tactile_full_summary_seq = getattr(other, "tactile_full_summary_seq", None)
        self.tactile_full_crc = getattr(other, "tactile_full_crc", None)
        self.tactile_full_crc_errors = int(getattr(other, "tactile_full_crc_errors", 0))
        self.tactile_full_seq_mismatches = int(getattr(other, "tactile_full_seq_mismatches", 0))
        self.tactile_full_dropped = int(getattr(other, "tactile_full_dropped", 0))
        self.tactile_source = str(getattr(other, "tactile_source", TACTILE_SOURCE_NONE))
        self.tactile_data_level = str(getattr(other, "tactile_data_level", TACTILE_DATA_LEVEL_NONE))
        self.tactile_module_valid_mask = int(getattr(other, "tactile_module_valid_mask", 0) or 0)
        self.tactile_diag_version = int(getattr(other, "tactile_diag_version", 0) or 0)
        self.tactile_diag_frame_count = int(getattr(other, "tactile_diag_frame_count", 0) or 0)
        for name in (
            "tactile_diag_enabled_mask",
            "tactile_diag_summary_ok_mask",
            "tactile_diag_full_ok_mask",
            "tactile_diag_command_fail_mask",
            "tactile_diag_response_fail_mask",
            "tactile_diag_crc_fail_mask",
            "tactile_diag_status_nonzero_mask",
            "tactile_diag_summary_retry_recovered_mask",
        ):
            setattr(self, name, int(getattr(other, name, 0) or 0))
        for name in (
            "tactile_diag_summary_success_count",
            "tactile_diag_summary_failure_count",
            "tactile_diag_full_success_count",
            "tactile_diag_full_failure_count",
            "tactile_diag_summary_last_fail_reason",
            "tactile_diag_full_last_fail_reason",
            "tactile_diag_module_last_status",
        ):
            setattr(self, name, list(getattr(other, name, []) or []))
        self.tactile_diag_last_rx_ts = float(getattr(other, "tactile_diag_last_rx_ts", 0.0) or 0.0)
        self.tactile_online = bool(getattr(other, "tactile_online", False))
        self.tactile_last_rx_ts = float(getattr(other, "tactile_last_rx_ts", 0.0) or 0.0)
        self.tactile_module_online = list(
            getattr(other, "tactile_module_online", [False] * (5 * TACTILE_SENSORS_PER_FINGER))
        )
        self.tactile_module_last_rx_ts = list(
            getattr(other, "tactile_module_last_rx_ts", [0.0] * (5 * TACTILE_SENSORS_PER_FINGER))
        )


def clone_hand_model(model: HandModel) -> HandModel:
    """建立跨线程只读快照，集中维护目标、零点和状态字段复制。"""
    snapshot = HandModel()
    snapshot.copy_state_from(model)
    snapshot.calib_status = str(getattr(model, "calib_status", "IDLE"))
    snapshot.set_target_angles(model.target_angles)
    for index, encoder in enumerate(snapshot.encoders):
        if index < len(getattr(model, "encoders", [])):
            encoder.zero_raw = int(model.encoders[index].zero_raw)
    return snapshot
