"""raw_ik_target.py - raw-IK ROS2 兼容入口模块契约。

职责：为 raw_ik 输入选择复用唯一的双向 ROS2 关节桥实现。
线程：不创建额外线程；生命周期与 Ros2JointTargetBridge 完全一致。
输入：固定 raw-IK 目标 topic、固定 urdf_deg 数据和反馈门控配置。
输出：配置后的共享 ROS2 bridge。
状态所有权：不拥有额外订阅器或设备状态，仅保留兼容配置类型。
禁止依赖：禁止依赖 Qt/UI、直接写串口或复制 ROS2 订阅实现。
"""
from dataclasses import dataclass
from desktop.core.joint_target_adapter import URDF_DEG
from desktop.ros2.joint_targets import Ros2JointTargetBridge

DEFAULT_RAW_IK_TARGET_TOPIC = "/raw_ik_target"


@dataclass(frozen=True)
class Ros2RawIkTargetConfig:
    topic: str = DEFAULT_RAW_IK_TARGET_TOPIC
    auto_start: bool = False
    input_space: str = URDF_DEG


class RawIkTargetBridge(Ros2JointTargetBridge):
    def __init__(self, backend, *, auto_start=False, feedback_mode="armed", output="all"):
        super().__init__(backend, topic=DEFAULT_RAW_IK_TARGET_TOPIC, auto_start=auto_start,
                         input_space=URDF_DEG, feedback_mode=feedback_mode,
                         output=output)


__all__ = ["DEFAULT_RAW_IK_TARGET_TOPIC", "RawIkTargetBridge", "Ros2RawIkTargetConfig"]
