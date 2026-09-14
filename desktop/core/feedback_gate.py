"""feedback_gate.py - HORA 反馈门控模块契约。

职责：按 off/always/armed 模式筛选新鲜 P4 关节反馈并执行 300 ms 重置窗口。
线程：由 ROS2 executor 定时器调用；HandBackend 使用同一把 ROS2 锁串行化访问。
输入：HandModel 实测映射角、主机接收时间及 ROS2 INPUT 授权状态。
输出：可发布的系统时间戳和 21 个 URDF 弧度角，或丢弃当前帧。
状态所有权：拥有冷却截止时间、发布高水位和 ready 状态，不拥有设备状态。
禁止依赖：禁止依赖 Qt、rclpy、串口或直接发送任何硬件命令。
"""
import math
import time


class FeedbackGate:
    def __init__(self, mode="armed", clock=time.monotonic_ns):
        if mode not in ("off", "always", "armed"):
            raise ValueError("feedback mode must be off, always or armed")
        self.mode = mode
        self.clock = clock
        self.ready = False
        self.last_publish_ns = 0
        self.last_stamp_ns = 0
        self.disabled()

    def disabled(self):
        self.ready = False
        self.deadline = max(self.clock(), self.last_publish_ns) + 300_000_000
        self.fresh_after = None

    def next_sample(self, state, requested):
        now = self.clock()
        if self.mode == "off":
            return None
        if self.mode == "armed":
            if not requested or now < self.deadline:
                return None
            if self.fresh_after is None:
                self.fresh_after = now
                return None
            if state.joint_state_host_rx_monotonic_ns <= self.fresh_after:
                return None
        age = now - state.joint_state_host_rx_monotonic_ns
        stamp = state.joint_state_host_rx_ns
        values = state.angles
        if (not state.joint_state_valid or not 0 <= age <= 200_000_000
                or stamp <= self.last_stamp_ns or len(values) != 21
                or any(e.angle_error for e in state.encoders)
                or not all(math.isfinite(x) for x in values)):
            return None
        return stamp, [math.radians(x) for x in values]

    def published(self, stamp):
        self.last_stamp_ns = stamp
        self.last_publish_ns = self.clock()
        self.ready = True
