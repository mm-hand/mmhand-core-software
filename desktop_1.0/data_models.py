# data_models.py - 灵巧手数据模型（状态与控制合一）

from dataclasses import dataclass, field
from typing import List, Optional
import numpy as np
from enum import Enum

from protocol import ENCODER_COUNT, MOTOR_COUNT

# 角度 → 度 转换因子 (14 位编码器 16384 = 360°)
RAW_TO_DEG = 360.0 / 16384.0

# 触觉传感器布局：每指 3 个传感器，每个传感器 N×M 个触点
TACTILE_SENSORS_PER_FINGER = 3
TACTILE_ROWS = 4
TACTILE_COLS = 4


class MotorState(Enum):
    IDLE = 0
    RUNNING = 1
    ERROR = 2
    CALIBRATING = 3


@dataclass
class TactileSensor:
    """单根手指上的一个触觉传感器"""
    contact_forces: np.ndarray  # (N, M, 3)
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
    """单根手指的触觉数据：3 个触觉传感器"""
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
