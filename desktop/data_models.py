# data_models.py - 灵巧手数据模型（状态与控制合一）

from dataclasses import dataclass, field
from typing import List, Optional, Tuple, NamedTuple
import numpy as np
from enum import Enum

from protocol import ENCODER_COUNT, MOTOR_COUNT

# 角度 → 度 转换因子 (14 位编码器 16384 = 360°)
RAW_TO_DEG = 360.0 / 16384.0

# 触觉：每指 3 组传感数据（指尖 5×5=25，两指腹各 4×13=52），contact_forces 形状见下；热力图条带几何仍为 5 段
TACTILE_SENSORS_PER_FINGER = 3
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

# 兼容旧脚本名（曾用于 4×13 指腹）
TACTILE_ROWS = TACTILE_BAND_HEIGHTS[2]
TACTILE_COLS = TACTILE_BAND_WIDTH


def tactile_band_shape(band_index: int) -> Tuple[int, int]:
    """band 0=指尖 … 4=近端指腹，均为 (高, 15)。"""
    if band_index < 0 or band_index >= TACTILE_NBANDS:
        raise ValueError(f"band_index 需在 0..{TACTILE_NBANDS - 1}")
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


def tactile_apply_palm_hexagon_bottom_corners(
    mat: np.ndarray,
    pr: int,
    pc: int,
    palm_h: int,
    palm_w: int,
    leg: Optional[int] = None,
) -> None:
    """
    在已填充的掌矩形区域上，将左下、右下两角各裁去一个等腰直角三角形（斜边朝掌内侧），
    裁掉格置为 NaN，热力图显示为轮廓外（白）。行向下递增，底行 pr+palm_h-1。
    """
    if leg is None:
        leg = TACTILE_PALM_BOTTOM_CORNER_CUT_LEG
    leg = min(int(leg), palm_w // 2, palm_h // 2)
    if leg <= 0:
        return
    nrows, ncols = mat.shape
    for idx in range(leg):
        r = pr + palm_h - 1 - idx
        if r < 0 or r >= nrows:
            continue
        w = leg - idx
        # 左下角：自下而上每行多裁 1 列，底行共 leg 列
        c0l, c1l = pc, pc + w
        mat[r, max(c0l, 0) : min(c1l, ncols)] = np.nan
        # 右下角
        c0r, c1r = pc + palm_w - w, pc + palm_w
        mat[r, max(c0r, 0) : min(c1r, ncols)] = np.nan


def tactile_palm_left_bottom_cut_top_vertex_rc(
    pr: int,
    pc: int,
    palm_h: int,
    palm_w: int,
    leg: Optional[int] = None,
) -> Tuple[int, int]:
    """
    掌面左下裁切的等腰直角三角形在「掌左侧竖直边」上的顶点（非直角顶点、靠近斜边与左缘交点一侧），
    与 tactile_apply_palm_hexagon_bottom_corners 的阶梯边界一致：最上一行被挖行的下标为 pr + palm_h - leg，
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


def tactile_apply_fingertip_round_cap(mat: np.ndarray, r_top: int, c0: int, W: int) -> None:
    """
    竖条指尖（行 r_top 为条带最上缘、指尖朝向图像上方）：将顶端裁成半圆鼓包，
    圆直径 = W（与手指宽度一致），圆心 local ((W-1)/2, R)、R=W/2，直径边水平贴在指尖带下沿一侧，圆外格置 NaN（白）。
    """
    if W < 2:
        return
    R = W / 2.0
    cx = (W - 1) / 2.0
    r2 = R * R
    nrows, ncols = mat.shape
    cap_h = int(np.ceil(R)) + 1
    for dy in range(cap_h):
        rr = r_top + dy
        if rr < 0 or rr >= nrows:
            break
        for dx in range(W):
            cc = c0 + dx
            if cc < 0 or cc >= ncols:
                continue
            if (dx - cx) ** 2 + (dy - R) ** 2 > r2 + 1e-9:
                mat[rr, cc] = np.nan


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

# 仅传感覆盖层：指根参考点相对条带最下行的向下偏移（行，与拼接格网同一单位）
TACTILE_OVERLAY_FINGER_ROOT_BELOW_UNITS = 10


def tactile_sensors_only_anatomy_points(
    spec: TactileStitchSpec,
) -> Tuple[Tuple[float, float], List[Tuple[float, float]]]:
    """
    「仅传感」示意：掌根点 + 五指（拇…小）参考点，均为 imshow 数据坐标 (x=列, y=行)，行 0 在图像上方。
    掌根取掌矩形底边中点；指根点取各指条带最下行（靠掌/腕侧）再向腕侧偏移固定格数 TACTILE_OVERLAY_FINGER_ROOT_BELOW_UNITS。
    """
    pr, pc, ph, pw = spec.palm_r, spec.palm_c, spec.palm_h, spec.palm_w
    palm_xy = (float(pc + pw // 2), float(pr + ph - 1))

    dr = int(TACTILE_OVERLAY_FINGER_ROOT_BELOW_UNITS)
    h_strip = TACTILE_FINGER_STRIP_ROWS
    step = TACTILE_BAND_WIDTH + TACTILE_FOUR_FINGER_GAP
    base_four_r = spec.four_r + h_strip - 1
    thumb_base_r = spec.thumb_r + h_strip - 1
    thumb_cx = float(spec.thumb_c + TACTILE_BAND_WIDTH // 2)

    finger_pts: List[Tuple[float, float]] = [
        (thumb_cx, float(thumb_base_r + dr)),
    ]
    for fi in range(4):
        c0 = spec.four_c + fi * step
        cx = float(c0 + TACTILE_BAND_WIDTH // 2)
        finger_pts.append((cx, float(base_four_r + dr)))

    H, W = spec.height, spec.width
    def _clamp(x: float, y: float) -> Tuple[float, float]:
        return (
            float(np.clip(x, 0.0, float(W - 1))),
            float(np.clip(y, 0.0, float(H - 1))),
        )

    palm_xy = _clamp(*palm_xy)
    finger_pts = [_clamp(x, y) for x, y in finger_pts]
    return palm_xy, finger_pts


def tactile_sensors_only_finger_chain_polylines(
    spec: TactileStitchSpec,
) -> List[List[Tuple[float, float]]]:
    """
    五指各一条折线顶点（顺序：指尖顶行中心 → 近端第 1 个关节带行中心 → 第 2 个关节带行中心 → 指根参考点），
    坐标同 imshow (列, 行)。关节带对应 TACTILE_BAND_HEIGHTS 中间两段 12 行宽条带的几何中心行。
    """
    dr = int(TACTILE_OVERLAY_FINGER_ROOT_BELOW_UNITS)
    h_strip = TACTILE_FINGER_STRIP_ROWS
    bh = TACTILE_BAND_HEIGHTS
    tip_lr = 0
    j1_lr = bh[0] + bh[1] // 2
    j2_lr = bh[0] + bh[1] + bh[2] + bh[3] // 2
    root_lr = h_strip - 1 + dr
    step = TACTILE_BAND_WIDTH + TACTILE_FOUR_FINGER_GAP
    w = TACTILE_BAND_WIDTH
    H, W = spec.height, spec.width

    def _clamp(x: float, y: float) -> Tuple[float, float]:
        return (
            float(np.clip(x, 0.0, float(W - 1))),
            float(np.clip(y, 0.0, float(H - 1))),
        )

    def _one_strip(sr: int, sc: int) -> List[Tuple[float, float]]:
        cx = float(sc + w // 2)
        pts = [
            (cx, float(sr + tip_lr)),
            (cx, float(sr + j1_lr)),
            (cx, float(sr + j2_lr)),
            (cx, float(sr + root_lr)),
        ]
        return [_clamp(x, y) for x, y in pts]

    polys: List[List[Tuple[float, float]]] = [
        _one_strip(spec.thumb_r, spec.thumb_c),
    ]
    for fi in range(4):
        polys.append(_one_strip(spec.four_r, spec.four_c + fi * step))
    return polys


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
    raw: int = 0              # 原始数据（14 位，0~16383）
    zero_raw: int = 0         # 矫正的零点值（来自 calib_deg.json / 下位机）
    target_angle_deg: float = 0.0   # 目标角度（度）
    error: bool = False       # 传感器/读数异常

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
    calib_status: str = "IDLE"  # NO_CALIB, IDLE, PENDING, SUCCESS, FAILED
    timestamp: float = 0.0
    # 数据有效性标志：用于 UI 区分“无数据”和“值为 0”
    has_sensor_data: bool = False
    has_servo_angle_data: bool = False

    # 伺服绝对角度与在线状态（来自 PACKET_TYPE_SERVO_ANGLE，单位：编码器计数或固件定义的原始值）
    servo_angles: List[int] = field(default_factory=lambda: [0] * MOTOR_COUNT)
    servo_online: List[bool] = field(default_factory=lambda: [False] * MOTOR_COUNT)

    # 伺服遥测数据（来自 PACKET_TYPE_SERVO_TELEM）
    servo_speed: List[int] = field(default_factory=lambda: [0] * MOTOR_COUNT)
    servo_load: List[int] = field(default_factory=lambda: [0] * MOTOR_COUNT)
    servo_voltage: List[int] = field(default_factory=lambda: [0] * MOTOR_COUNT)
    servo_temperature: List[int] = field(default_factory=lambda: [0] * MOTOR_COUNT)
    servo_telem_online: List[bool] = field(default_factory=lambda: [False] * MOTOR_COUNT)
    # 伺服过载保护锁存状态（来自 PACKET_TYPE_FAULT_STATUS，按 motor channel 位图映射）
    servo_overload_fault: List[bool] = field(default_factory=lambda: [False] * MOTOR_COUNT)
    # 关节反绕保护锁存状态（来自 PACKET_TYPE_RELEASE_FAULT，按 joint index 位图映射）
    joint_reverse_release_fault: List[bool] = field(default_factory=lambda: [False] * ENCODER_COUNT)

    # 关节调试信息（来自 PACKET_TYPE_JOINT_DEBUG）
    joint_debug_valid: List[bool] = field(default_factory=lambda: [False] * ENCODER_COUNT)
    joint_debug_target_deg: List[float] = field(default_factory=lambda: [0.0] * ENCODER_COUNT)
    joint_debug_actual_deg: List[float] = field(default_factory=lambda: [0.0] * ENCODER_COUNT)
    joint_debug_loop1_output: List[float] = field(default_factory=lambda: [0.0] * ENCODER_COUNT)
    joint_debug_loop2_actual: List[float] = field(default_factory=lambda: [0.0] * ENCODER_COUNT)
    joint_debug_loop2_output: List[float] = field(default_factory=lambda: [0.0] * ENCODER_COUNT)
    joint_debug_cmd_valid: List[bool] = field(default_factory=lambda: [False] * ENCODER_COUNT)
    joint_debug_cmd_target_pos: List[int] = field(default_factory=lambda: [0] * ENCODER_COUNT)

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

    def set_encoder_zeros(self, zero_raw_list: List[int]) -> None:
        """设置全部编码器的零点值（校准）"""
        for i, z in enumerate(zero_raw_list):
            if i < len(self.encoders):
                self.encoders[i].zero_raw = int(z) & 0xFFFF

    def copy_state_from(self, other: "HandModel") -> None:
        """仅复制状态（编码器 raw/error、电机 error/state/伺服遥测），不覆盖 target_angle_deg、zero_raw"""
        for i, e in enumerate(self.encoders):
            if i < len(other.encoders):
                e.raw = other.encoders[i].raw
                e.error = other.encoders[i].error
        for i, m in enumerate(self.motors):
            if i < len(other.motors):
                m.error = other.motors[i].error
                m.state = other.motors[i].state
        # 复制伺服绝对角度与遥测
        self.servo_angles = list(other.servo_angles)
        self.servo_online = list(other.servo_online)
        self.servo_speed = list(other.servo_speed)
        self.servo_load = list(other.servo_load)
        self.servo_voltage = list(other.servo_voltage)
        self.servo_temperature = list(other.servo_temperature)
        self.servo_telem_online = list(other.servo_telem_online)
        self.servo_overload_fault = list(other.servo_overload_fault)
        self.joint_reverse_release_fault = list(other.joint_reverse_release_fault)
        # 复制关节调试信息
        self.joint_debug_valid = list(other.joint_debug_valid)
        self.joint_debug_target_deg = list(other.joint_debug_target_deg)
        self.joint_debug_actual_deg = list(other.joint_debug_actual_deg)
        self.joint_debug_loop1_output = list(other.joint_debug_loop1_output)
        self.joint_debug_loop2_actual = list(other.joint_debug_loop2_actual)
        self.joint_debug_loop2_output = list(other.joint_debug_loop2_output)
        self.joint_debug_cmd_valid = list(other.joint_debug_cmd_valid)
        self.joint_debug_cmd_target_pos = list(other.joint_debug_cmd_target_pos)
        # 复制数据有效性标志
        self.has_sensor_data = bool(other.has_sensor_data)
        self.has_servo_angle_data = bool(other.has_servo_angle_data)
