"""state_reducer.py - P4 状态快照的单写入归并器。

职责：维护一个常驻工作模型，按接收顺序应用路由包，并按节流策略发布新的只读快照。
线程：仅由 P4 RX 线程写入；其他线程只能通过兼容门面读取最后**已发布**快照。
输入：RoutedPacket 序列和一个无持久状态的单包归并函数。
输出：发生可见状态变化时返回新的只读 HandModel 快照，否则返回 ``None``。
状态所有权：本类唯一持有工作模型与最新已发布快照；业务处理函数不得另存可变模型引用。
禁止依赖：禁止打开串口、编码命令、访问 Qt/UI 或直接操作 RX/TX 队列。
"""

from __future__ import annotations

import time
from typing import Callable, Optional

from ..data_models import (
    HandModel,
    MotorState,
    TACTILE_DATA_LEVEL_FULL,
    TACTILE_DATA_LEVEL_SUMMARY,
    TACTILE_SOURCE_P4_CAN_RELAY,
    clone_hand_model,
)
from ..protocol import (
    CMD_PROTOCOL_HELLO,
    COMMANDS_REQUIRING_HANDSHAKE,
    CMD_ACTIVE_SCOPE,
    CMD_CLEAR_CALIBRATION,
    CMD_CLEAR_CALIBRATION_SCOPE,
    CMD_MANUAL_ENCODER_REFERENCE,
    CMD_MANUAL_ENCODER_SINGLE,

    CMD_RESET,
    CMD_RESTORE_MULTITURN,
    CMD_RUN_FEEDBACK,
    CMD_SERVO_RESET_ZERO,
    CMD_START,
    CMD_STOP,
    ENCODER_COUNT,
    MOTOR_COUNT,
    PROTO_ACK_STATUS_OK,
    PROTO_ACK_STATUS_CONTRACT_MISMATCH,
    PROTO_ACK_STATUS_QUEUED,
    SYSTEM_FAULT_CALIBRATION,
    SYSTEM_STATE_CALIBRATION_RUNNING,
    SYSTEM_STATE_IDLE,

    SYSTEM_STATE_READY,
    SYSTEM_STATE_RECOVERY_REQUIRED,
    SYSTEM_STATE_RUN_FEEDBACK,
    SYSTEM_STATE_UNCONTROLLABLE,
    TASK_RUNTIME_NAMES,
)
from ..session_log import log_session
from ..protocol import (
    parse_adaptive_kp_packet,
    parse_control_components_packet,
    parse_control_detail_packet,
)
from .ack_tracker import AckTracker
from .event_channel import EventChannel
from .feedback_history import TimestampedFeedbackHistory
from .packet_router import P4PacketRouter


ModelFinalizer = Callable[[HandModel], None]


# 高频遥测批次不逐批物化快照：工作模型常驻，只有距上次发布达到该间隔才复制一份
# 只读快照，避免 _update_loop 每次消费都做一次全量拷贝。测试可 monkeypatch 为 0
# 关闭节流（发布间隔=0 表示每批都发布），不会掩盖真实节流行为。
STATE_SNAPSHOT_MIN_INTERVAL_S = 0.02

# 控制/标定/安全状态不能等待节流窗口：这些包到达时立即发布快照，保证 ACK、
# 系统状态、事务结果与安全事件的可见性不取决于高频流量的发布相位。
STATE_SNAPSHOT_CRITICAL_PACKET_KINDS = frozenset({
    "protocol_ack",             # ACK（含 HELLO 握手、STOP/RESET、标定命令应答）
    "system_status",            # 系统状态/连接性/标定状态权威来源
    "direct_target_result",     # 直接目标执行最终结果
    "joint_calibration_snapshot",   # 关节标定快照
    "calibration_diagnostic",   # 标定诊断进度
    "manual_reference",         # Manual 参考表回读
    "servo_rebase_zero_result",     # 当前位置设零最终结果
    "multiturn_restore_result",     # 多圈恢复最终结果
    "safety_event",             # 安全规则事件
    "fault_status",             # 过载故障位图
    "servo_route_status",       # 舵机路由/扫描配置
    "product_fingerprint",      # 产品指纹（启动握手依赖）
    "motor_abs_limit",          # ABS 可达范围
    "servo_logical_range",      # 舵机逻辑行程
})


class StateReducer:
    """把增量协议包归并为完整设备快照。"""

    # 包类型 -> 处理函数。新增一种 P4 包只在 _apply_xxx 写好后来这里登记一行。
    PACKET_APPLIERS = {
        "sensor": lambda self, owner, model, payload, time_ms, rx_at: self._apply_sensor(owner, model, payload, time_ms, rx_at),
        "encoder_raw": lambda self, owner, model, payload, time_ms, rx_at: self._apply_encoder_raw(owner, model, payload, time_ms, rx_at),
        "servo_position": lambda self, owner, model, payload, time_ms, rx_at: self._apply_servo_position(owner, model, payload, time_ms, rx_at),
        "multiturn_restore_result": lambda self, owner, model, payload, time_ms, rx_at: self._apply_multiturn_restore_result(owner, model, payload, time_ms, rx_at),
        "servo_rebase_zero_result": lambda self, owner, model, payload, time_ms, rx_at: self._apply_servo_rebase_zero_result(owner, model, payload, time_ms, rx_at),
        "direct_target_result": lambda self, owner, model, payload, time_ms, rx_at: self._apply_direct_target_result(owner, model, payload, time_ms, rx_at),
        "servo_telemetry": lambda self, owner, model, payload, time_ms, rx_at: self._apply_servo_telemetry(owner, model, payload, time_ms, rx_at),
        "fault_status": lambda self, owner, model, payload, time_ms, rx_at: self._apply_fault_status(owner, model, payload, time_ms, rx_at),
        "system_status": lambda self, owner, model, payload, time_ms, rx_at: self._apply_system_status(owner, model, payload, time_ms, rx_at),
        "servo_route_status": lambda self, owner, model, payload, time_ms, rx_at: self._apply_servo_route_status(owner, model, payload, time_ms, rx_at),
        "product_fingerprint": lambda self, owner, model, payload, time_ms, rx_at: self._apply_product_fingerprint(owner, model, payload, time_ms, rx_at),
        "safety_event": lambda self, owner, model, payload, time_ms, rx_at: self._apply_safety_event(owner, model, payload, time_ms, rx_at),
        "motor_abs_limit": lambda self, owner, model, payload, time_ms, rx_at: self._apply_motor_abs_limit(owner, model, payload, time_ms, rx_at),
        "servo_logical_range": lambda self, owner, model, payload, time_ms, rx_at: self._apply_servo_logical_range(owner, model, payload, time_ms, rx_at),
        "manual_reference": lambda self, owner, model, payload, time_ms, rx_at: self._apply_manual_reference(owner, model, payload, time_ms, rx_at),
        "joint_calibration_snapshot": lambda self, owner, model, payload, time_ms, rx_at: self._apply_joint_calibration_snapshot(owner, model, payload, time_ms, rx_at),
        "calibration_diagnostic": lambda self, owner, model, payload, time_ms, rx_at: self._apply_calibration_diagnostic(owner, model, payload, time_ms, rx_at),
        "adaptive_kp": lambda self, owner, model, payload, time_ms, rx_at: self._apply_adaptive_kp(owner, model, payload, time_ms, rx_at),
        "control_trace": lambda self, owner, model, payload, time_ms, rx_at: self._apply_control_trace(owner, model, payload, time_ms, rx_at),
        "task_diagnostic": lambda self, owner, model, payload, time_ms, rx_at: self._apply_task_diagnostic(owner, model, payload, time_ms, rx_at),
        "tactile_summary": lambda self, owner, model, payload, time_ms, rx_at: self._apply_tactile_summary(owner, model, payload, time_ms, rx_at),
        "tactile_full": lambda self, owner, model, payload, time_ms, rx_at: self._apply_tactile_full(owner, model, payload, time_ms, rx_at),
        "protocol_ack": lambda self, owner, model, payload, time_ms, rx_at: self._apply_protocol_ack(owner, model, payload, time_ms, rx_at),
    }

    def __init__(
        self,
        packet_router: Optional[P4PacketRouter] = None,
        ack_tracker: Optional[AckTracker] = None,
        safety_event_channel: Optional[EventChannel[dict]] = None,
    ) -> None:
        # 工作模型只在 RX 线程内原地更新；_last_model 始终指向最近一次**已发布**
        # 的只读快照，供其他线程/测试安全读取，绝不让外部看到可变工作模型。
        self._working: Optional[HandModel] = None
        self._last_model: Optional[HandModel] = None
        self._last_publish_monotonic = 0.0
        self._packet_router = packet_router
        self._ack_tracker = ack_tracker
        self._safety_event_channel = safety_event_channel
        # 高频原始反馈按协议包逐帧留存，不能从批次末尾的 HandModel 反推历史。
        # UI 快照仍保持 latest-value；需要时间对齐的业务显式读取此独立历史。
        self.feedback_history = TimestampedFeedbackHistory()
        # 只有 SYSTEM_STATUS 的包尾时间是当前设备时钟；raw/ABS 保留采集
        # 时间，历史诊断也可回放旧 timestamp，不能拿它们判断设备重启。
        self._feedback_status_clock_ms: Optional[int] = None
        # TASK_DIAG v4 由 P4 在约 35 ms 内分七包发出；先按窗口暂存，
        # 避免 UI 把不同一秒统计窗口的任务行混合展示。
        self._task_runtime_window_sequence: Optional[int] = None
        self._task_runtime_window_packets: dict[int, dict] = {}
        self._task_runtime_dropped_windows = 0
        self._control_parts = {}

    @property
    def last_model(self) -> Optional[HandModel]:
        """返回最近一次已发布快照引用；调用方按只读契约使用。"""
        return self._last_model

    @last_model.setter
    def last_model(self, model: Optional[HandModel]) -> None:
        # 断线模型恢复与组件级测试通过此入口替换快照；下一次 begin 会以该快照
        # 为基线重建工作模型，协议包仍必须经 begin/commit 归并。
        self._last_model = model
        self._working = None

    def reset(self) -> None:
        """连接新设备时清空旧设备状态，避免跨设备沿用在线位图或标定结果。"""
        self._working = None
        self._last_model = None
        self._last_publish_monotonic = 0.0
        self.feedback_history.reset()
        self._feedback_status_clock_ms = None
        self._task_runtime_window_sequence = None
        self._task_runtime_window_packets.clear()
        self._task_runtime_dropped_windows = 0
        self._control_parts.clear()

    def begin(self) -> tuple[HandModel, float]:
        """返回本批数据归并用的常驻工作模型，不做整模型拷贝。"""
        working = self._working
        if working is None:
            working = HandModel()
            previous = self._last_model
            if previous is not None:
                # 外部替换过快照（断线模型/测试基线）时只在这里做一次性继承；
                # 正常在线路径的工作模型常驻，begin 不再复制。
                working.copy_state_from(previous)
            self._working = working
        host_received_at = time.time()
        working.host_rx_time = host_received_at
        return working, host_received_at

    def commit(
        self,
        model: HandModel,
        emitted: bool,
        finalize_model: ModelFinalizer,
        *,
        publish_now: bool = False,
    ) -> Optional[HandModel]:
        """完成在线可见性等统一收尾，并按节流策略发布只读快照。

        ``publish_now`` 表示本批含控制/标定/安全关键包，必须立即发布。
        返回 ``None`` 表示本批状态已被常驻工作模型吸收、无需向状态邮箱入队。
        """
        finalize_model(model)
        if not emitted:
            return None
        now = time.monotonic()
        interval = STATE_SNAPSHOT_MIN_INTERVAL_S
        if (
            not publish_now
            and interval > 0.0
            and (now - self._last_publish_monotonic) < interval
        ):
            # 工作模型继续吸收本批更新；下一批到期后发布的快照自然包含它们。
            return None
        snapshot = clone_hand_model(model)
        self._last_publish_monotonic = now
        self._last_model = snapshot
        return snapshot

    def process_packets(self, owner, packets: list[tuple]) -> Optional[HandModel]:
        """归并一批 P4 包。

        ``owner`` 是暂时保留的兼容钩子，只提供日志、触觉重组和动态 parser。
        HandModel、包路由与 ACK 序号均已由拆出的组件持有，不能写回兼容类字段。
        """
        if self._packet_router is None or self._ack_tracker is None:
            raise RuntimeError("StateReducer requires packet router and ACK tracker")
        if packets:
            owner._heartbeat_enabled = True
        model, _host_received_at = self.begin()
        emitted = False
        publish_now = False

        for routed_packet in self._packet_router.route_many(packets):
            kind, payload, time_ms = routed_packet.kind, routed_packet.payload, routed_packet.p4_time_ms
            publish_now = publish_now or kind in STATE_SNAPSHOT_CRITICAL_PACKET_KINDS
            if time_ms is not None:
                model.p4_time_ms, model.has_p4_time = int(time_ms), True
            if kind == "control_components":
                part = parse_control_components_packet(payload)
                if part is not None:
                    self._cache_control_part(time_ms, part, 'components')
                continue
            # 包类型到处理函数的唯一映射；未登记的类型（文本日志等）直接忽略。
            apply = self.PACKET_APPLIERS.get(kind)
            if apply is not None:
                emitted = apply(self, owner, model, payload, time_ms, _host_received_at) or emitted

        return self.commit(
            model,
            emitted,
            owner._apply_connectivity_visibility,
            publish_now=publish_now,
        )

    def _apply_sensor(self, owner, model, payload, packet_p4_time_ms, _host_received_at):
        """Apply one sensor packet within the RX batch boundary."""
        emitted = False
        angles, errors = owner._parse_protocol_payload("parse_sensor_packet", payload)
        if angles:
            for i in range(min(ENCODER_COUNT, len(angles), len(model.encoders))):
                model.encoders[i].encoder_id = i
                angle_error = errors[i] if i < len(errors) else False
                model.encoders[i].angle_error = angle_error
                if not angle_error:
                    # P4 validFlags 决定映射角是否可信；越出标定空间的真实反馈
                    # 仍必须覆盖旧值，否则 UI 会把陈旧角度误显示为当前角。
                    model.encoders[i].raw = angles[i]
                    model.encoders[i].angle_out_of_workspace = (
                        owner._mapped_angle_out_of_workspace(i, int(angles[i]), model)
                    )
                else:
                    model.encoders[i].angle_out_of_workspace = False
                model.encoders[i].zero_raw = 0
                model.encoders[i].error = angle_error

            for i in range(min(ENCODER_COUNT, len(model.motors))):
                model.motors[i].motor_id = i
                model.motors[i].error = errors[i] if i < len(errors) else False
                model.motors[i].state = (
                    MotorState.ERROR if model.motors[i].error else MotorState.RUNNING
                )

            model.has_sensor_data = True
            model.encoder_mapped_last_rx_ts = _host_received_at
            model.monotonic_received['encoder_mapped_last_rx_ts'] = time.monotonic()
            model.encoder_mapped_p4_time_ms = int(packet_p4_time_ms or 0)
            emitted = True
        return emitted

    def _apply_encoder_raw(self, owner, model, payload, packet_p4_time_ms, _host_received_at):
        """Apply one encoder_raw packet within the RX batch boundary."""
        emitted = False
        raw_values, errors = owner._parse_protocol_payload("parse_sensor_packet", payload)
        if raw_values:
            for i in range(min(ENCODER_COUNT, len(raw_values), len(model.encoders))):
                raw_error = errors[i] if i < len(errors) else False
                model.encoders[i].raw_error = raw_error
                if not raw_error:
                    model.encoders[i].raw_sensor = raw_values[i]
                model.encoders[i].error = raw_error
            model.has_encoder_raw_data = True
            model.encoder_raw_p4_time_ms = int(packet_p4_time_ms or 0)
            model.encoder_raw_last_rx_ts = time.time()
            model.monotonic_received['encoder_raw_last_rx_ts'] = time.monotonic()
            valid_mask = sum(
                1 << index
                for index in range(min(ENCODER_COUNT, len(raw_values)))
                if index >= len(errors) or not errors[index]
            )
            if model.has_connectivity_status:
                valid_mask &= (
                    int(model.encoder_online_bitmap)
                    if model.can_link_online else 0
                )
            else:
                # 尚无权威在线状态的首批帧仍留存，但不能用于拟合；
                # 后到 SYSTEM_STATUS 不反向解释或修补先前的数据帧。
                valid_mask = 0
            if model.has_encoder_health_status:
                valid_mask &= ~int(model.encoder_fault_bitmap)
            if packet_p4_time_ms is not None:
                self.feedback_history.record(
                    "encoder_raw", int(packet_p4_time_ms),
                    tuple(int(value) for value in raw_values),
                    valid_mask, model.encoder_raw_last_rx_ts,
                )
            emitted = True
        return emitted

    def _apply_servo_position(self, owner, model, payload, packet_p4_time_ms, _host_received_at):
        """Apply one merged raw+ABS servo position packet."""
        parsed = owner._parse_protocol_payload("parse_servo_position_packet", payload)
        if parsed is None:
            return False
        raw_positions, angles = parsed
        for i in range(min(MOTOR_COUNT, len(raw_positions))):
            model.servo_raw_positions[i] = int(raw_positions[i])
        for i in range(min(MOTOR_COUNT, len(angles))):
            model.servo_angles[i] = int(angles[i])
        # 两个数组共享同一个 P4 时间戳；CSV 不会误用同批次的 SYSTEM_STATUS。
        if packet_p4_time_ms is not None:
            model.servo_feedback_p4_time_ms = int(packet_p4_time_ms)
            model.servo_raw_p4_time_ms = int(packet_p4_time_ms)
        model.servo_abs_last_rx_ts = time.time()
        model.monotonic_received['servo_abs_last_rx_ts'] = time.monotonic()
        model.has_servo_angle_data = True
        model.has_servo_raw_data = True
        # ABS 没有单路有效位。尚无权威在线状态时，不能把占位 ABS 当成可信反馈。
        valid_mask = (
            int(model.servo_online_bitmap)
            & ~int(model.multi_turn_unknown_bitmap)
            & ((1 << min(MOTOR_COUNT, len(angles))) - 1)
            if model.has_connectivity_status else 0
        )
        if packet_p4_time_ms is not None:
            self.feedback_history.record(
                "servo_abs", int(packet_p4_time_ms),
                tuple(int(value) for value in angles),
                valid_mask, model.servo_abs_last_rx_ts,
            )
        return True

    def _apply_multiturn_restore_result(self, owner, model, payload, packet_p4_time_ms, _host_received_at):
        """Apply one multiturn_restore_result packet within the RX batch boundary."""
        emitted = False
        parsed = owner._parse_protocol_payload(
            "parse_multiturn_restore_result_packet", payload
        )
        if parsed is not None and packet_p4_time_ms is not None:
            token, restored_mask, rejected_mask = parsed
            model.multiturn_restore_result_token = int(token)
            model.multiturn_restore_restored_mask = int(restored_mask)
            model.multiturn_restore_rejected_mask = int(rejected_mask)
            model.multiturn_restore_result_last_rx_ts = time.time()
            model.monotonic_received['multiturn_restore_result_last_rx_ts'] = time.monotonic()
            model.multiturn_restore_result_p4_time_ms = int(packet_p4_time_ms or 0)
            model.has_multiturn_restore_result = True
            emitted = True
        return emitted

    def _apply_servo_rebase_zero_result(self, owner, model, payload, packet_p4_time_ms, _host_received_at):
        """Apply one servo_rebase_zero_result packet within the RX batch boundary."""
        emitted = False
        parsed = owner._parse_protocol_payload(
            "parse_servo_rebase_zero_result_packet", payload
        )
        if parsed is not None:
            token, success_mask, failed_mask = parsed
            previous_identity = (
                int(model.servo_rebase_zero_result_token),
                int(model.servo_rebase_zero_success_mask),
                int(model.servo_rebase_zero_failed_mask),
            )
            result_identity = (
                int(token), int(success_mask), int(failed_mask),
            )
            new_result = (
                not model.has_servo_rebase_zero_result
                or previous_identity != result_identity
            )
            model.servo_rebase_zero_result_token = int(token)
            model.servo_rebase_zero_success_mask = int(success_mask)
            model.servo_rebase_zero_failed_mask = int(failed_mask)
            model.servo_rebase_zero_result_p4_time_ms = int(
                packet_p4_time_ms or 0
            )
            model.servo_rebase_zero_result_last_rx_ts = time.time()
            model.monotonic_received['servo_rebase_zero_result_last_rx_ts'] = time.monotonic()
            model.has_servo_rebase_zero_result = True
            if success_mask and new_result:
                # F0 改变 ABS 的坐标原点；旧、新坐标的
                # 数据不能处于同一可插值时间窗口。状态同步会重发
                # 同一事务结果，单纯回显不能废弃正在采集的窗口。
                self.feedback_history.reset()
            emitted = True
        return emitted

    def _apply_direct_target_result(self, owner, model, payload, packet_p4_time_ms, _host_received_at):
        """Apply one direct_target_result packet within the RX batch boundary."""
        emitted = False
        parsed = owner._parse_protocol_payload(
            "parse_direct_target_result_packet", payload
        )
        if parsed is not None:
            token, requested, applied, rejected, status = parsed
            model.direct_target_token = int(token)
            model.direct_target_requested_mask = int(requested)
            model.direct_target_applied_mask = int(applied)
            model.direct_target_rejected_mask = int(rejected)
            model.direct_target_status = int(status)
            model.direct_target_result_rx_ts = time.time()
            model.monotonic_received['direct_target_result_rx_ts'] = time.monotonic()
            model.has_direct_target_result = True
            emitted = True
        return emitted

    def _apply_servo_telemetry(self, owner, model, payload, packet_p4_time_ms, _host_received_at):
        """Apply one servo_telemetry packet within the RX batch boundary."""
        emitted = False
        parsed = owner._parse_protocol_payload("parse_servo_telem_packet", payload)
        if parsed is not None:
            speeds, loads, currents, volts, temps = parsed
            for i in range(min(MOTOR_COUNT, len(speeds))):
                model.servo_speed[i] = int(speeds[i])
                model.servo_load[i] = int(loads[i])
                model.servo_current[i] = int(currents[i])
                model.servo_voltage[i] = int(volts[i])
                model.servo_temperature[i] = int(temps[i])
            model.servo_telem_p4_time_ms = int(packet_p4_time_ms or 0)
            model.servo_telem_last_rx_ts = time.time()
            model.monotonic_received['servo_telem_last_rx_ts'] = time.monotonic()
            emitted = True
        return emitted

    def _apply_fault_status(self, owner, model, payload, packet_p4_time_ms, _host_received_at):
        """Apply one fault_status packet within the RX batch boundary."""
        emitted = False
        parsed_fault = owner._parse_protocol_payload("parse_fault_status_packet", payload)
        if parsed_fault is not None:
            fault_bitmap, trigger_loads = parsed_fault
            for i in range(MOTOR_COUNT):
                faulted = ((fault_bitmap >> i) & 0x01) != 0
                model.servo_overload_fault[i] = faulted
                if i < len(trigger_loads):
                    model.servo_overload_trigger_load[i] = trigger_loads[i] if faulted else None
            model.has_fault_status_data = True
            model.fault_status_last_rx_ts = time.time()
            model.monotonic_received['fault_status_last_rx_ts'] = time.monotonic()
            emitted = True
        return emitted

    def _apply_system_status(self, owner, model, payload, packet_p4_time_ms, _host_received_at):
        """Apply one system_status packet within the RX batch boundary."""
        emitted = False
        parsed = owner._parse_protocol_payload("parse_system_status_packet", payload)
        if parsed is not None:
            if packet_p4_time_ms is not None:
                status_clock_ms = int(packet_p4_time_ms) & 0xFFFFFFFF
                previous_clock_ms = self._feedback_status_clock_ms
                if previous_clock_ms is not None:
                    signed_delta = (
                        (status_clock_ms - previous_clock_ms + 0x80000000)
                        & 0xFFFFFFFF
                    ) - 0x80000000
                    if signed_delta < 0:
                        # 串口 FIFO 的同类状态包时钟倒退：保守地
                        # 撤销旧 epoch，不能跨自主重启拼接位置曲线。
                        # uint32 的正常自然回卷 signed_delta 仍为正。
                        self.feedback_history.reset()
                self._feedback_status_clock_ms = status_clock_ms
            (
                system_state,
                control_enabled,
                servo_target_owner,
                control_mode,
                calib_status,
                system_fault_bitmap,
                state_transition_count,
                multi_turn_unknown_bitmap,
                active_motor_mask,
                active_joint_mask,
                require_can_feedback,
                has_active_scope_status,
                servo_online_bitmap,
                encoder_online_bitmap,
                can_link_online,
                has_connectivity_status,
                encoder_warn_bitmap,
                encoder_fault_bitmap,
                has_encoder_health_status,
                manual_reference_valid_bitmap,
                manual_reference_crc,
                manual_reference_revision,
                manual_reference_committed,
                has_manual_reference_status,
                auto_calibration_valid_bitmap,
                manual_reference_commit_status,
                has_manual_commit_status,
                rope_init_status,
                rope_init_reason,
                rope_init_sequence,
                rope_init_joint_mask,
                rope_init_timestamp_ms,
                pending_direct_generation,
                applied_direct_generation,
                direct_reject_reason,
                has_motion_result_status,
                servo_turn_overflow_bitmap,
                servo_reset_zero_request_token,
                servo_reset_zero_result_token,
                servo_reset_zero_success_mask,
                servo_reset_zero_failed_mask,
                servo_reset_zero_invalidated_joint_mask,
                servo_reset_zero_status,
                has_servo_reset_zero_status,
                feedback_admission_sequence,
                feedback_admission_status,
                feedback_reject_reason,
                has_feedback_admission_status,
                feedback_collision_armed_bitmap,
                feedback_collision_hold_bitmap,
                feedback_collision_transition_count,
                has_feedback_collision_status,
                calibration_warning_bitmap,
                calibration_auto_min_offset_counts,
                has_calibration_warning_status,
            ) = parsed
            model.system_state = int(system_state)
            model.control_enabled = bool(control_enabled)
            model.servo_target_owner = int(servo_target_owner)
            model.control_mode = int(control_mode)
            model.system_fault_bitmap = int(system_fault_bitmap)
            model.state_transition_count = int(state_transition_count)
            model.multi_turn_unknown_bitmap = int(multi_turn_unknown_bitmap)
            model.manual_reference_valid_bitmap = int(manual_reference_valid_bitmap)
            model.manual_reference_crc = int(manual_reference_crc)
            model.manual_reference_revision = int(manual_reference_revision)
            model.manual_reference_committed = bool(manual_reference_committed)
            model.has_manual_reference_status = bool(has_manual_reference_status)
            if has_manual_commit_status:
                model.auto_calibration_valid_bitmap = int(
                    auto_calibration_valid_bitmap
                )
                model.manual_reference_commit_status = int(
                    manual_reference_commit_status
                )
            if has_motion_result_status:
                model.rope_init_status = int(rope_init_status)
                model.rope_init_reason = int(rope_init_reason)
                model.rope_init_sequence = int(rope_init_sequence)
                model.rope_init_joint_mask = int(rope_init_joint_mask)
                model.rope_init_timestamp_ms = int(rope_init_timestamp_ms)
                model.pending_direct_generation = int(pending_direct_generation)
                model.applied_direct_generation = int(applied_direct_generation)
                model.direct_reject_reason = int(direct_reject_reason)
                model.has_motion_result_status = True
            model.active_motor_mask = int(active_motor_mask)
            model.active_joint_mask = int(active_joint_mask)
            model.require_can_feedback = bool(require_can_feedback)
            model.has_active_scope_status = bool(has_active_scope_status)
            model.servo_online_bitmap = int(servo_online_bitmap)
            # 在线状态只有 SYSTEM_STATUS 一个权威来源；三个展示数组保持
            # 同步，避免 100 Hz 数据包重复携带 27 字节在线标志。
            for i in range(MOTOR_COUNT):
                online = ((int(servo_online_bitmap) >> i) & 0x01) != 0
                model.servo_online[i] = online
                model.servo_raw_online[i] = online
                model.servo_telem_online[i] = online
            model.encoder_online_bitmap = int(encoder_online_bitmap)
            model.encoder_warn_bitmap = int(encoder_warn_bitmap)
            model.encoder_fault_bitmap = int(encoder_fault_bitmap)
            model.can_link_online = bool(can_link_online)
            model.has_connectivity_status = bool(has_connectivity_status)
            model.has_encoder_health_status = bool(has_encoder_health_status)
            model.calibration_warning_bitmap = int(
                calibration_warning_bitmap
            )
            model.calibration_auto_min_offset_counts = list(
                calibration_auto_min_offset_counts
            )
            model.has_calibration_warning_status = bool(
                has_calibration_warning_status
            )
            model.feedback_admission_sequence = int(feedback_admission_sequence)
            model.feedback_admission_status = int(feedback_admission_status)
            model.feedback_reject_reason = int(feedback_reject_reason)
            model.has_feedback_admission_status = bool(
                has_feedback_admission_status
            )
            model.feedback_collision_armed_bitmap = int(
                feedback_collision_armed_bitmap
            )
            model.feedback_collision_hold_bitmap = int(
                feedback_collision_hold_bitmap
            )
            model.feedback_collision_transition_count = int(
                feedback_collision_transition_count
            )
            model.has_feedback_collision_status = bool(
                has_feedback_collision_status
            )
            model.servo_turn_overflow_bitmap = int(
                servo_turn_overflow_bitmap
            )
            model.servo_reset_zero_request_token = int(
                servo_reset_zero_request_token
            )
            model.servo_reset_zero_result_token = int(
                servo_reset_zero_result_token
            )
            model.servo_reset_zero_success_mask = int(
                servo_reset_zero_success_mask
            )
            model.servo_reset_zero_failed_mask = int(
                servo_reset_zero_failed_mask
            )
            model.servo_reset_zero_invalidated_joint_mask = int(
                servo_reset_zero_invalidated_joint_mask
            )
            model.servo_reset_zero_status = int(
                servo_reset_zero_status
            )
            model.has_servo_reset_zero_status = bool(
                has_servo_reset_zero_status
            )
            model.has_system_status_data = True
            model.system_status_last_rx_ts = time.time()
            model.monotonic_received['system_status_last_rx_ts'] = time.monotonic()
            if packet_p4_time_ms is not None:
                model.system_status_p4_time_ms = int(packet_p4_time_ms)
            if calib_status == 0:
                if (
                    model.calib_status in ("PENDING", "RUNNING") and
                    model.system_state != SYSTEM_STATE_CALIBRATION_RUNNING
                ):
                    if (model.system_fault_bitmap & SYSTEM_FAULT_CALIBRATION) != 0:
                        model.calib_status = "FAILED"
                    elif model.system_state in (
                        SYSTEM_STATE_IDLE,
                        SYSTEM_STATE_READY,
                        SYSTEM_STATE_RECOVERY_REQUIRED,
                    ):
                        model.calib_status = "SUCCESS"
                    else:
                        model.calib_status = "IDLE"
                else:
                    model.calib_status = "IDLE"
            elif calib_status == 1:
                model.calib_status = "RUNNING"
            elif calib_status == 2:
                model.calib_status = "SUCCESS"
            elif calib_status == 3:
                model.calib_status = "FAILED"
            elif calib_status == 4:
                model.calib_status = "WARN"
            emitted = True
        return emitted

    def _apply_servo_route_status(self, owner, model, payload, packet_p4_time_ms, _host_received_at):
        """Apply one servo_route_status packet within the RX batch boundary."""
        emitted = False
        parsed = owner._parse_protocol_payload(
            "parse_servo_route_status_packet", payload
        )
        if parsed is not None:
            model.servo_route_controllable_bitmap = int(
                parsed["controllable_bitmap"]
            )
            model.servo_route_conflict_bitmap = int(parsed["conflict_bitmap"])
            model.servo_bus_overload_mask = int(parsed["bus_overload_mask"])
            model.servo_route_scan_state = int(parsed["scan_state"])
            model.servo_route_scan_progress = int(parsed["scan_progress"])
            model.servo_bus_device_count = list(parsed["bus_device_count"])
            model.servo_route_bus = list(parsed["route_bus"])
            model.servo_route_state = list(parsed["route_state"])
            model.has_servo_route_status = True
            emitted = True
        return emitted

    def _apply_product_fingerprint(self, owner, model, payload, packet_p4_time_ms, _host_received_at):
        """Apply one product_fingerprint packet within the RX batch boundary."""
        emitted = False
        parsed = owner._parse_protocol_payload("parse_product_fingerprint_packet", payload)
        if parsed is not None:
            # CRC 在模型中保留整数，写 meta.json 时统一格式化为 8 位十六进制。
            model.product_fingerprint = dict(parsed)
            model.has_product_fingerprint = True
            emitted = True
        return emitted

    def _apply_safety_event(self, owner, model, payload, packet_p4_time_ms, _host_received_at):
        """Apply one safety_event packet within the RX batch boundary."""
        emitted = False
        parsed = owner._parse_protocol_payload(
            "parse_safety_event_packet", payload
        )
        if parsed is not None:
            event = dict(parsed)
            event["device_timestamp_ms"] = int(packet_p4_time_ms or 0)
            # sequence 是 P4 单调事件号；重连或重复帧不得制造第二条证据。
            last_sequence = int(
                model.latest_safety_event.get("sequence", 0)
                if model.latest_safety_event
                else 0
            )
            if int(event["sequence"]) != last_sequence:
                model.safety_event_history.append(event)
                model.safety_event_history = model.safety_event_history[-64:]
            model.latest_safety_event = event
            model.safety_event_overflow_count = int(
                event.get("overflow_count", 0)
            )
            model.safety_event_last_rx_ts = time.time()
            model.monotonic_received['safety_event_last_rx_ts'] = time.monotonic()
            model.has_safety_event = True
            if self._safety_event_channel is not None:
                self._safety_event_channel.publish(dict(event))
            emitted = True
        return emitted

    def _apply_motor_abs_limit(self, owner, model, payload, packet_p4_time_ms, _host_received_at):
        """Apply one motor_abs_limit packet within the RX batch boundary."""
        emitted = False
        parsed = owner._parse_protocol_payload("parse_motor_abs_limit_packet", payload)
        if parsed is not None:
            valid_bitmap, mins, maxs = parsed
            model.motor_abs_limit_valid_bitmap = int(valid_bitmap)
            for i in range(min(MOTOR_COUNT, len(mins), len(maxs))):
                model.motor_abs_limit_min[i] = int(mins[i])
                model.motor_abs_limit_max[i] = int(maxs[i])
            emitted = True
        return emitted

    def _apply_servo_logical_range(self, owner, model, payload, packet_p4_time_ms, _host_received_at):
        """Apply one servo_logical_range packet within the RX batch boundary."""
        emitted = False
        parsed = owner._parse_protocol_payload("parse_servo_logical_range_packet", payload)
        if parsed is not None:
            valid_bitmap, mins, maxs = parsed
            model.servo_logical_range_valid_bitmap = int(valid_bitmap)
            for i in range(min(MOTOR_COUNT, len(mins), len(maxs))):
                model.servo_logical_range_min[i] = int(mins[i])
                model.servo_logical_range_max[i] = int(maxs[i])
            model.servo_logical_range_last_rx_ts = time.time()
            model.monotonic_received['servo_logical_range_last_rx_ts'] = time.monotonic()
            model.servo_logical_range_p4_time_ms = int(packet_p4_time_ms or 0)
            emitted = True
        return emitted

    def _apply_manual_reference(self, owner, model, payload, packet_p4_time_ms, _host_received_at):
        """Apply one manual_reference packet within the RX batch boundary."""
        emitted = False
        parsed = owner._parse_protocol_payload("parse_manual_encoder_reference_packet", payload)
        if parsed is not None:
            model.manual_reference_valid_bitmap = int(parsed["valid_bitmap"])
            model.manual_reference_crc = int(parsed["reference_crc"])
            model.manual_reference_revision = int(parsed["revision"])
            model.manual_reference_committed = bool(parsed["committed"])
            model.auto_calibration_valid_bitmap = int(
                parsed.get("auto_valid_bitmap", 0)
            )
            model.manual_reference_commit_status = int(
                parsed.get("commit_status", 0)
            )
            model.manual_reference_last_rx_time = time.time()
            model.monotonic_received['manual_reference_last_rx_time'] = time.monotonic()
            model.has_manual_reference_status = True
            model.manual_reference_min_raw = list(parsed["min_raw"])
            model.manual_reference_max_raw = list(parsed["max_raw"])
            model.manual_reference_encoder_directions = list(
                parsed.get("encoder_directions", [0] * ENCODER_COUNT)
            )
            model.has_manual_reference_encoder_directions = bool(
                parsed.get("has_encoder_directions", False)
            )
            emitted = True
        return emitted

    def _apply_joint_calibration_snapshot(self, owner, model, payload, packet_p4_time_ms, _host_received_at):
        """Apply one joint_calibration_snapshot packet within the RX batch boundary."""
        emitted = False
        parsed = owner._parse_protocol_payload("parse_joint_calib_snapshot_packet", payload)
        if parsed is not None:
            joint_idx = int(parsed.get("joint", -1))
            if 0 <= joint_idx < ENCODER_COUNT:
                new_result = dict(parsed)
                model.joint_calib_results[joint_idx] = new_result
                emitted = True
        return emitted

    def _apply_calibration_diagnostic(self, owner, model, payload, packet_p4_time_ms, _host_received_at):
        """Apply one calibration_diagnostic packet within the RX batch boundary."""
        emitted = False
        parsed = owner._parse_protocol_payload("parse_calib_diag_packet", payload)
        if parsed is not None:
            owner._log_auto_calib_diag(parsed)
            model.calib_diag_valid = bool(parsed.get("valid", False))
            model.calib_diag_reason = int(parsed.get("reason", 0) or 0)
            model.calib_diag_phase = int(parsed.get("phase", 0) or 0)
            model.calib_diag_joint = int(parsed.get("joint", 0xFF) or 0)
            model.calib_diag_motor = int(parsed.get("motor", 0xFF) or 0)
            model.calib_diag_value_a = int(parsed.get("valueA", 0) or 0)
            model.calib_diag_value_b = int(parsed.get("valueB", 0) or 0)
            model.calib_diag_value_c = int(parsed.get("valueC", 0) or 0)
            model.calib_diag_bitmap = int(parsed.get("bitmap", 0) or 0)
            model.calib_diag_timestamp_ms = int(parsed.get("timestamp_ms", 0) or 0)
            model.calib_diag_sequence = int(parsed.get("sequence", 0) or 0)
            model.calib_diag_completed_joint_mask = int(
                parsed.get("completed_joint_mask", 0) or 0
            )
            model.calib_diag_failed_joint_mask = int(
                parsed.get("failed_joint_mask", 0) or 0
            )
            model.p4_time_ms = model.calib_diag_timestamp_ms
            model.has_p4_time = True
            emitted = True
        return emitted

    def _apply_adaptive_kp(self, owner, model, payload, packet_p4_time_ms, _host_received_at):
        """Apply the 1 Hz per-joint adaptive Kp snapshot."""
        parsed = parse_adaptive_kp_packet(payload)
        if parsed is None:
            return False
        model.adaptive_kp_enabled = bool(parsed["enabled"])
        model.adaptive_kp_scale = list(parsed["scale"])
        model.adaptive_kp_effective = list(parsed["effective"])
        model.adaptive_kp_last_rx_ts = _host_received_at
        model.has_adaptive_kp_data = True
        return True

    def _apply_control_trace(self, owner, model, payload, packet_p4_time_ms, _host_received_at):
        """Apply one control_trace packet within the RX batch boundary."""
        emitted = False
        detail = parse_control_detail_packet(payload)
        if detail is not None:
            self._cache_control_part(packet_p4_time_ms, detail, 'detail')
            return False
        parsed = owner._parse_protocol_payload("parse_control_trace_packet", payload)
        if parsed is not None and packet_p4_time_ms is not None:
            model.has_control_trace = True
            parts = self._control_parts.pop((packet_p4_time_ms, parsed['sequence']), {})
            model.control_trace_components = parts.get('components', {})
            model.control_trace_detail = parts.get('detail', {})
            model.control_trace_sequence = int(parsed["sequence"])
            model.control_trace_target_token = int(parsed["target_token"])
            model.control_trace_p4_time_ms = int(packet_p4_time_ms)
            model.control_trace_target_accepted_ms = int(
                parsed["target_accepted_ms"]
            )
            model.control_trace_encoder_timestamp_ms = int(
                parsed["encoder_timestamp_ms"]
            )
            model.control_trace_active_joint_mask = int(
                parsed["active_joint_mask"]
            )
            model.control_trace_valid_joint_mask = int(
                parsed["valid_joint_mask"]
            )
            model.control_trace_motor_command_mask = int(
                parsed["motor_command_mask"]
            )
            model.control_trace_target_deg = list(parsed["target_deg"])
            model.control_trace_actual_deg = list(parsed["actual_deg"])
            model.control_trace_motor_command_abs = list(
                parsed["motor_command_abs"]
            )
            model.control_trace_last_rx_ts = time.time()
            model.monotonic_received['control_trace_last_rx_ts'] = time.monotonic()
            emitted = True
        return emitted

    def _cache_control_part(self, timestamp, part, kind):
        self._control_parts.setdefault((timestamp, part['sequence']), {})[kind] = part
        while len(self._control_parts) > 32:
            self._control_parts.pop(next(iter(self._control_parts)))

    def _apply_task_diagnostic(self, owner, model, payload, packet_p4_time_ms, _host_received_at):
        """Apply one task_diagnostic packet within the RX batch boundary."""
        emitted = False
        parsed = owner._parse_protocol_payload("parse_task_diag_packet", payload)
        if parsed is not None:
            task_value = parsed.get("task_id")
            task_id = int(task_value) if task_value is not None else -1
            task_count = len(TASK_RUNTIME_NAMES)
            if 0 <= task_id < task_count:
                now = time.time()
                parsed["host_rx_ts"] = now
                window_sequence = int(parsed.get("window_sequence", 0) or 0)
                if self._task_runtime_window_sequence != window_sequence:
                    # 新窗口到达时丢弃上一个未收齐窗口，绝不拼接两组统计值。
                    if (
                        self._task_runtime_window_sequence is not None
                        and 0 < len(self._task_runtime_window_packets) < task_count
                    ):
                        self._task_runtime_dropped_windows += 1
                    self._task_runtime_window_sequence = window_sequence
                    self._task_runtime_window_packets.clear()
                self._task_runtime_window_packets[task_id] = dict(parsed)
                received_mask = 0
                for received_task_id in self._task_runtime_window_packets:
                    received_mask |= 1 << int(received_task_id)
                model.task_runtime_window_sequence = window_sequence
                model.task_runtime_received_mask = received_mask
                model.task_runtime_expected_mask = (1 << task_count) - 1
                model.task_runtime_dropped_windows = self._task_runtime_dropped_windows
                model.task_runtime_last_packet_rx_ts = now
                # 分包进度也发布给诊断 UI，避免“旧表还在但不知道缺了哪一包”。
                emitted = True
                if len(self._task_runtime_window_packets) >= task_count:
                    model.task_runtime = dict(self._task_runtime_window_packets)
                    model.task_runtime_last_rx_ts = now
                    model.p4_time_ms = int(parsed.get("device_timestamp_ms", 0) or 0)
                    model.has_p4_time = True
                    emitted = True
        return emitted

    def _apply_tactile_summary(self, owner, model, payload, packet_p4_time_ms, _host_received_at):
        """Apply one tactile_summary packet within the RX batch boundary."""
        emitted = False
        parsed = owner._parse_protocol_payload("parse_tactile_packet", payload)
        if parsed is not None:
            _seq, groups, module_valid_mask = parsed
            now = time.time()
            stable_groups = owner._stabilize_tactile_groups(groups)
            owner._last_tactile_summary_groups = stable_groups
            owner._last_tactile_summary_seq = int(_seq)
            model.tactile_summary_seq = int(_seq)
            model.tactile_source = TACTILE_SOURCE_P4_CAN_RELAY
            model.tactile_module_valid_mask = int(module_valid_mask)
            model.tactile_online = True
            model.tactile_last_rx_ts = now
            model.tactile_module_online = [
                bool(int(module_valid_mask) & (1 << module_index))
                for module_index in range(15)
            ]
            module_rx_times = list(model.tactile_module_last_rx_ts)
            while len(module_rx_times) < 15:
                module_rx_times.append(0.0)
            for module_index in range(15):
                if int(module_valid_mask) & (1 << module_index):
                    module_rx_times[module_index] = now
            model.tactile_module_last_rx_ts = module_rx_times[:15]
            model.tactile_full_crc_errors = owner._tactile_full_crc_errors
            model.tactile_full_seq_mismatches = owner._tactile_full_seq_mismatches
            model.tactile_full_dropped = owner._tactile_full_dropped
            if owner._summary_groups_all_zero(stable_groups):
                owner._last_tactile_full_ts = 0.0
                model.tactile_data = owner._build_tactile_fingers(stable_groups)
                model.tactile_data_level = TACTILE_DATA_LEVEL_SUMMARY
            elif (
                not model.tactile_data or
                (time.time() - owner._last_tactile_full_ts) > owner._tactile_full_stale_after_s
            ):
                model.tactile_data = owner._build_tactile_fingers(stable_groups)
                model.tactile_data_level = TACTILE_DATA_LEVEL_SUMMARY
            emitted = True
        return emitted

    def _apply_tactile_full(self, owner, model, payload, packet_p4_time_ms, _host_received_at):
        """Apply one tactile_full packet within the RX batch boundary."""
        emitted = False
        parsed = owner._parse_protocol_payload("parse_tactile_full_fragment", payload)
        if parsed is not None:
            seq, frag_idx, frag_count, chunk, summary_seq, full_crc = parsed
            accepted = owner._accept_tactile_full_fragment(
                seq,
                frag_idx,
                frag_count,
                chunk,
                summary_seq,
                full_crc,
            )
            if accepted is not None:
                raw, accepted_summary_seq, accepted_crc = accepted
                if (
                    accepted_summary_seq is not None and
                    owner._last_tactile_summary_seq != accepted_summary_seq
                ):
                    owner._tactile_full_seq_mismatches += 1
                    model.tactile_full_seq_mismatches = owner._tactile_full_seq_mismatches
                    emitted = True
                    return emitted
                model.tactile_data = owner._build_tactile_fingers_from_full(
                    raw,
                    owner._last_tactile_summary_groups,
                )
                model.tactile_source = TACTILE_SOURCE_P4_CAN_RELAY
                model.tactile_data_level = TACTILE_DATA_LEVEL_FULL
                model.tactile_online = True
                model.tactile_last_rx_ts = time.time()
                model.monotonic_received['tactile_last_rx_ts'] = time.monotonic()
                model.tactile_full_seq = int(seq)
                model.tactile_full_summary_seq = (
                    int(accepted_summary_seq)
                    if accepted_summary_seq is not None
                    else None
                )
                model.tactile_full_crc = (
                    int(accepted_crc)
                    if accepted_crc is not None
                    else None
                )
                model.tactile_full_crc_errors = owner._tactile_full_crc_errors
                model.tactile_full_seq_mismatches = owner._tactile_full_seq_mismatches
                model.tactile_full_dropped = owner._tactile_full_dropped
                owner._last_tactile_full_ts = time.time()
                emitted = True
        return emitted

    def _apply_protocol_ack(self, owner, model, payload, packet_p4_time_ms, _host_received_at):
        """Apply one protocol_ack packet within the RX batch boundary."""
        emitted = False
        parsed = owner._parse_protocol_payload("parse_proto_ack", payload)
        if parsed is not None:
            ack_cmd, applied_mode, status = parsed
            if ack_cmd in (
                CMD_CLEAR_CALIBRATION,
                CMD_CLEAR_CALIBRATION_SCOPE,
                CMD_SERVO_RESET_ZERO,
            ):
                owner._auto_calib_logger.log(
                    "ACK",
                    f"cmd=0x{int(ack_cmd):02X} mode={int(applied_mode)} status={int(status)}",
                )
            log_session(
                "PROTO",
                f"ACK cmd=0x{int(ack_cmd):02X} mode={int(applied_mode)} status={int(status)}",
            )
            # ACK 跟踪器是唯一同步边界，避免状态归并器自己操作 Condition。
            ack_received_at = time.time()
            model.proto_ack_sequence = self._ack_tracker.publish(
                ack_cmd,
                applied_mode,
                status,
                ack_received_at,
                packet_p4_time_ms,
            )
            model.proto_ack_cmd = int(ack_cmd)
            model.proto_ack_mode = int(applied_mode)
            model.proto_ack_status = int(status)
            if ack_cmd == CMD_PROTOCOL_HELLO:
                owner._protocol_compatible = int(status) == PROTO_ACK_STATUS_OK
                model.protocol_compatible = owner._protocol_compatible
            elif (ack_cmd in COMMANDS_REQUIRING_HANDSHAKE
                  and int(status) == PROTO_ACK_STATUS_CONTRACT_MISMATCH):
                owner._protocol_compatible = False
                model.protocol_compatible = False
            model.proto_ack_p4_time_ms = int(packet_p4_time_ms or 0)
            if ack_cmd == CMD_RESET and status in (
                PROTO_ACK_STATUS_OK, PROTO_ACK_STATUS_QUEUED
            ):
                self.feedback_history.reset()
            # 模型保留最近历史供 UI 展示；可靠逐条消费仍由 AckTracker 事件通道负责。
            model.proto_ack_history.append({
                "sequence": int(model.proto_ack_sequence),
                "command": int(ack_cmd),
                "mode": int(applied_mode),
                "status": int(status),
                "received_at": float(ack_received_at),
                "received_monotonic": time.monotonic(),
                "p4_time_ms": int(packet_p4_time_ms or 0),
            })
            model.proto_ack_history = model.proto_ack_history[-64:]
            if (
                ack_cmd in (CMD_STOP, CMD_RESET) and
                status in (PROTO_ACK_STATUS_OK, PROTO_ACK_STATUS_QUEUED)
            ):
                model.control_enabled = False
                model.system_state = SYSTEM_STATE_UNCONTROLLABLE
                if model.calib_status in ("PENDING", "RUNNING"):
                    model.calib_status = "IDLE"
            if ack_cmd in (
                CMD_START,
                CMD_STOP,
                CMD_RESET,
                CMD_ACTIVE_SCOPE,
                CMD_RUN_FEEDBACK,
                            CMD_RESTORE_MULTITURN,
                CMD_MANUAL_ENCODER_REFERENCE,
                CMD_MANUAL_ENCODER_SINGLE,
                CMD_CLEAR_CALIBRATION,
                CMD_CLEAR_CALIBRATION_SCOPE,
            ) and status == PROTO_ACK_STATUS_OK:
                model.system_state = int(applied_mode)
                if ack_cmd == CMD_START:
                    model.control_enabled = False
                elif ack_cmd == CMD_STOP:
                    model.control_enabled = False
                    model.system_state = SYSTEM_STATE_UNCONTROLLABLE
                    if model.calib_status in ("PENDING", "RUNNING"):
                        model.calib_status = "IDLE"
                elif ack_cmd == CMD_RESET:
                    model.control_enabled = False
                    model.system_state = SYSTEM_STATE_UNCONTROLLABLE
                    if model.calib_status in ("PENDING", "RUNNING"):
                        model.calib_status = "IDLE"
                elif ack_cmd == CMD_ACTIVE_SCOPE:
                    pass
                elif ack_cmd == CMD_RUN_FEEDBACK:
                    model.control_enabled = int(applied_mode) == SYSTEM_STATE_RUN_FEEDBACK
                elif ack_cmd == CMD_RESTORE_MULTITURN:
                    pass
                elif ack_cmd in (
                    CMD_MANUAL_ENCODER_REFERENCE,
                    CMD_MANUAL_ENCODER_SINGLE,
                ):
                    # Manual 写入后的 Auto 失效范围由 P4 按实际 changedMask 决定。
                    # 单路写入绝不能在上位机把其他 20 路结果和 ABS Limit 全清空；
                    # 后续 AUTO_STATUS/ABS_LIMIT/快照包会提交准确的逐关节状态。
                    model.calib_status = "IDLE"
                elif ack_cmd == CMD_CLEAR_CALIBRATION:
                    model.calib_status = "IDLE"
                    model.motor_abs_limit_valid_bitmap = 0
                    model.joint_calib_results = [None] * ENCODER_COUNT
                    for encoder in model.encoders:
                        encoder.raw = 0
                        encoder.angle_error = True
                        encoder.angle_out_of_workspace = False
                elif ack_cmd == CMD_CLEAR_CALIBRATION_SCOPE:
                    # 局部清除的准确 joint/motor mask 由后续 P4 状态包
                    # 回读，不能像全局清除一样在通信层把 21 路全部归零。
                    pass
                elif ack_cmd == CMD_SERVO_RESET_ZERO:
                    pass
            emitted = True
        return emitted
