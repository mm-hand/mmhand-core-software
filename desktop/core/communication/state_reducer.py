"""state_reducer.py - P4 状态快照的单写入归并器。

职责：复制上一帧 HandModel、按接收顺序应用路由包，并发布新的完整状态快照。
线程：仅由 P4 RX 线程写入；其他线程只能通过兼容门面读取最后快照。
输入：RoutedPacket 序列和一个无持久状态的单包归并函数。
输出：发生有效状态变化时返回新 HandModel，否则返回 ``None``。
状态所有权：本类唯一持有最新 HandModel；业务处理函数不得另存可变模型引用。
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
)
from ..protocol import (
    CMD_ACTIVE_SCOPE,
    CMD_CALIBRATE,
    CMD_CLEAR_CALIBRATION,
    CMD_CLEAR_CALIBRATION_SCOPE,
    CMD_MANUAL_ENCODER_REFERENCE,
    CMD_MANUAL_ENCODER_SINGLE,
    CMD_PAUSE,
    CMD_RESET,
    CMD_RESTORE_MULTITURN,
    CMD_RUN_FEEDBACK,
    CMD_SERVO_RESET_ZERO,
    CMD_START,
    CMD_STOP,
    ENCODER_COUNT,
    MOTOR_COUNT,
    PROTO_ACK_STATUS_OK,
    PROTO_ACK_STATUS_QUEUED,
    SYSTEM_FAULT_CALIBRATION,
    SYSTEM_STATE_CALIBRATION_RUNNING,
    SYSTEM_STATE_IDLE,
    SYSTEM_STATE_PAUSED,
    SYSTEM_STATE_READY,
    SYSTEM_STATE_RECOVERY_REQUIRED,
    SYSTEM_STATE_RUN_FEEDBACK,
    SYSTEM_STATE_UNCONTROLLABLE,
    TASK_RUNTIME_NAMES,
)
from ..session_log import log_session
from .ack_tracker import AckTracker
from .event_channel import EventChannel
from .feedback_history import TimestampedFeedbackHistory
from .packet_router import P4PacketRouter


ModelFinalizer = Callable[[HandModel], None]


class StateReducer:
    """把增量协议包归并为完整设备快照。"""

    def __init__(
        self,
        packet_router: Optional[P4PacketRouter] = None,
        ack_tracker: Optional[AckTracker] = None,
        safety_event_channel: Optional[EventChannel[dict]] = None,
    ) -> None:
        self._last_model: Optional[HandModel] = None
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

    @property
    def last_model(self) -> Optional[HandModel]:
        """返回最近快照引用；兼容层对外发布前仍应遵守原有只读约定。"""
        return self._last_model

    @last_model.setter
    def last_model(self, model: Optional[HandModel]) -> None:
        # 断线模型恢复与组件级测试通过此入口替换快照；协议包仍必须经 begin/commit 归并。
        self._last_model = model

    def reset(self) -> None:
        """连接新设备时清空旧设备状态，避免跨设备沿用在线位图或标定结果。"""
        self._last_model = None
        self.feedback_history.reset()
        self._feedback_status_clock_ms = None
        self._task_runtime_window_sequence = None
        self._task_runtime_window_packets.clear()
        self._task_runtime_dropped_windows = 0

    def begin(self) -> tuple[HandModel, float]:
        """创建本批数据的工作副本，并继承不能被单包覆盖的通信有效位。"""
        model = HandModel()
        if self._last_model is not None:
            model.copy_state_from(self._last_model)
            model.calib_status = self._last_model.calib_status
            model.has_sensor_data = bool(self._last_model.has_sensor_data)
            model.has_encoder_raw_data = bool(
                getattr(self._last_model, "has_encoder_raw_data", False)
            )
            model.encoder_raw_last_rx_ts = float(
                getattr(self._last_model, "encoder_raw_last_rx_ts", 0.0) or 0.0
            )
            model.has_servo_angle_data = bool(self._last_model.has_servo_angle_data)
            model.has_servo_raw_data = bool(self._last_model.has_servo_raw_data)
        host_received_at = time.time()
        model.host_rx_time = host_received_at
        return model, host_received_at

    def commit(
        self,
        model: HandModel,
        emitted: bool,
        finalize_model: ModelFinalizer,
    ) -> Optional[HandModel]:
        """完成在线可见性等统一收尾，并原子替换最新模型引用。"""
        finalize_model(model)
        self._last_model = model
        return model if emitted else None

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

        for routed_packet in self._packet_router.route_many(packets):
            packet_kind = routed_packet.kind
            payload = routed_packet.payload
            packet_p4_time_ms = routed_packet.p4_time_ms
            if packet_p4_time_ms is not None:
                model.p4_time_ms = int(packet_p4_time_ms)
                model.has_p4_time = True
            # 磁编码器映射角：更新负反馈使用的实时关节角及工作空间提示。
            if packet_kind == "sensor":
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
                    emitted = True

            # 磁编码器原始值：未自动标定时同样有效，Device Check 依赖该数据。
            elif packet_kind == "encoder_raw":
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

            # 舵机 logical ABS：控制、标定和多圈恢复共同使用的连续位置反馈。
            elif packet_kind == "servo_abs":
                parsed = owner._parse_protocol_payload("parse_servo_angle_packet", payload)
                if parsed is not None:
                    for i in range(min(MOTOR_COUNT, len(parsed))):
                        model.servo_angles[i] = int(parsed[i])
                    # 舵机 ABS 包与固件反馈快照共用该 P4 时间戳。单独保存后，
                    # CSV 不会误用同批次中随后到达的 SYSTEM_STATUS 等包时间。
                    if packet_p4_time_ms is not None:
                        model.servo_feedback_p4_time_ms = int(packet_p4_time_ms)
                    model.servo_abs_last_rx_ts = time.time()
                    model.has_servo_angle_data = True
                    # ABS 包没有单路有效位。尚无权威在线状态时，不能把
                    # 默认 multi_turn_unknown=0 和占位 ABS 误当成可信反馈。
                    valid_mask = (
                        int(model.servo_online_bitmap)
                        & ~int(model.multi_turn_unknown_bitmap)
                        & ((1 << min(MOTOR_COUNT, len(parsed))) - 1)
                        if model.has_connectivity_status else 0
                    )
                    if packet_p4_time_ms is not None:
                        self.feedback_history.record(
                            "servo_abs", int(packet_p4_time_ms),
                            tuple(int(value) for value in parsed),
                            valid_mask, model.servo_abs_last_rx_ts,
                        )
                    emitted = True

            # 100 Hz 舵机单圈 raw；跨重启多圈锚点只由桌面 JSON 持有。
            elif packet_kind == "servo_raw":
                parsed = owner._parse_protocol_payload("parse_servo_raw_packet", payload)
                if parsed is not None:
                    for i in range(min(MOTOR_COUNT, len(parsed))):
                        model.servo_raw_positions[i] = int(parsed[i])
                    if packet_p4_time_ms is not None:
                        model.servo_raw_p4_time_ms = int(packet_p4_time_ms)
                    model.has_servo_raw_data = True
                    emitted = True

            elif packet_kind == "multiturn_restore_result":
                parsed = owner._parse_protocol_payload(
                    "parse_multiturn_restore_result_packet", payload
                )
                if parsed is not None:
                    token, restored_mask, rejected_mask = parsed
                    model.multiturn_restore_result_token = int(token)
                    model.multiturn_restore_restored_mask = int(restored_mask)
                    model.multiturn_restore_rejected_mask = int(rejected_mask)
                    model.multiturn_restore_result_last_rx_ts = time.time()
                    emitted = True

            elif packet_kind == "servo_rebase_zero_result":
                parsed = owner._parse_protocol_payload(
                    "parse_servo_rebase_zero_result_packet", payload
                )
                if parsed is not None:
                    token, success_mask, failed_mask, offsets = parsed
                    previous_identity = (
                        int(model.servo_rebase_zero_result_token),
                        int(model.servo_rebase_zero_success_mask),
                        int(model.servo_rebase_zero_failed_mask),
                        tuple(model.servo_rebase_zero_logical_offsets),
                    )
                    result_identity = (
                        int(token), int(success_mask), int(failed_mask), tuple(offsets),
                    )
                    new_result = (
                        not model.has_servo_rebase_zero_result
                        or previous_identity != result_identity
                    )
                    model.servo_rebase_zero_result_token = int(token)
                    model.servo_rebase_zero_success_mask = int(success_mask)
                    model.servo_rebase_zero_failed_mask = int(failed_mask)
                    model.servo_rebase_zero_logical_offsets = list(offsets)
                    model.servo_rebase_zero_result_p4_time_ms = int(
                        packet_p4_time_ms or 0
                    )
                    model.servo_rebase_zero_result_last_rx_ts = time.time()
                    model.has_servo_rebase_zero_result = True
                    if success_mask and new_result:
                        # F0 改变 logical ABS 的坐标原点；旧、新 offset 的
                        # 数据不能处于同一可插值时间窗口。状态同步会重发
                        # 同一事务结果，单纯回显不能废弃正在采集的窗口。
                        self.feedback_history.reset()
                    emitted = True

            elif packet_kind == "direct_target_result":
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
                    model.has_direct_target_result = True
                    emitted = True

            # 舵机遥测：速度、负载、电流、电压和温度保持同一帧原子更新。
            elif packet_kind == "servo_telemetry":
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
                    emitted = True

            # 故障包只描述每台舵机的过载锁存，不在通信层自行清除故障。
            elif packet_kind == "fault_status":
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
                    emitted = True

            # 系统状态是 P4 权威快照；UI 的 READY、owner、碰撞和恢复状态均来自此包。
            elif packet_kind == "system_status":
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

            # 固件/结构指纹用于运行数据复现，通信层只保存原值，不判断产品兼容性。
            elif packet_kind == "servo_route_status":
                # 动态路由是 Motor Channels 显示和控制门禁的唯一 BUS 数据源。
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

            elif packet_kind == "product_fingerprint":
                parsed = owner._parse_protocol_payload("parse_product_fingerprint_packet", payload)
                if parsed is not None:
                    # CRC 在模型中保留整数，写 meta.json 时统一格式化为 8 位十六进制。
                    model.product_fingerprint = dict(parsed)
                    model.has_product_fingerprint = True
                    emitted = True

            elif packet_kind == "safety_event":
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
                    model.has_safety_event = True
                    if self._safety_event_channel is not None:
                        self._safety_event_channel.publish(dict(event))
                    emitted = True

            # 标定得到的电机 ABS 诊断范围必须整包替换，不能逐项混用旧值。
            elif packet_kind == "motor_abs_limit":
                parsed = owner._parse_protocol_payload("parse_motor_abs_limit_packet", payload)
                if parsed is not None:
                    valid_bitmap, mins, maxs = parsed
                    model.motor_abs_limit_valid_bitmap = int(valid_bitmap)
                    for i in range(min(MOTOR_COUNT, len(mins), len(maxs))):
                        model.motor_abs_limit_min[i] = int(mins[i])
                        model.motor_abs_limit_max[i] = int(maxs[i])
                    emitted = True

            # 热插拔恢复后的 logical 行程由 P4 计算，上位机只展示与记录。
            elif packet_kind == "servo_logical_range":
                parsed = owner._parse_protocol_payload("parse_servo_logical_range_packet", payload)
                if parsed is not None:
                    valid_bitmap, mins, maxs = parsed
                    model.servo_logical_range_valid_bitmap = int(valid_bitmap)
                    for i in range(min(MOTOR_COUNT, len(mins), len(maxs))):
                        model.servo_logical_range_min[i] = int(mins[i])
                        model.servo_logical_range_max[i] = int(maxs[i])
                    model.servo_logical_range_last_rx_ts = time.time()
                    model.servo_logical_range_p4_time_ms = int(packet_p4_time_ms or 0)
                    emitted = True

            # Manual Min/Max 是 Auto 标定前置基准，跨圈方向由独立 direction 字段表达。
            elif packet_kind == "manual_reference":
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

            # 单关节快照可分批到达，必须仅覆盖对应 Jxx，不能清空其他成功关节。
            elif packet_kind == "joint_calibration_snapshot":
                parsed = owner._parse_protocol_payload("parse_joint_calib_snapshot_packet", payload)
                if parsed is not None:
                    joint_idx = int(parsed.get("joint", -1))
                    if 0 <= joint_idx < ENCODER_COUNT:
                        new_result = dict(parsed)
                        model.joint_calib_results[joint_idx] = new_result
                        emitted = True

            # 标定诊断保留原始 reason/phase/value，弹窗文案由 UI 层负责。
            elif packet_kind == "calibration_diagnostic":
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

            # Target/Actual 来自同一 10 ms 控制周期，包尾时间由路由器单独提供。
            elif packet_kind == "control_trace":
                parsed = owner._parse_protocol_payload("parse_control_trace_packet", payload)
                if parsed is not None and packet_p4_time_ms is not None:
                    model.has_control_trace = True
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
                    emitted = True

            # RTOS 任务诊断按 task_id 更新，便于观察周期、执行时间和栈余量。
            elif packet_kind == "task_diagnostic":
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

            # 汇总触觉持续刷新在线状态；全量触觉新鲜时不允许汇总包覆盖细粒度矩阵。
            elif packet_kind == "tactile_summary":
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

            # 全量触觉分片需先完成序号、片数和 CRC 闭环，再发布完整一帧。
            elif packet_kind == "tactile_full":
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
                            continue
                        model.tactile_data = owner._build_tactile_fingers_from_full(
                            raw,
                            owner._last_tactile_summary_groups,
                        )
                        model.tactile_source = TACTILE_SOURCE_P4_CAN_RELAY
                        model.tactile_data_level = TACTILE_DATA_LEVEL_FULL
                        model.tactile_online = True
                        model.tactile_last_rx_ts = time.time()
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

            # 通用 ACK 同时唤醒等待线程并做有限的乐观状态更新；后续状态包仍是权威值。
            elif packet_kind == "protocol_ack":
                parsed = owner._parse_protocol_payload("parse_proto_ack", payload)
                if parsed is not None:
                    ack_cmd, applied_mode, status = parsed
                    if ack_cmd in (
                        CMD_CALIBRATE,
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
                        CMD_CALIBRATE,
                        CMD_ACTIVE_SCOPE,
                        CMD_RUN_FEEDBACK,
                        CMD_PAUSE,
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
                        elif ack_cmd == CMD_CALIBRATE:
                            model.control_enabled = False
                            model.system_state = SYSTEM_STATE_CALIBRATION_RUNNING
                        elif ack_cmd == CMD_ACTIVE_SCOPE:
                            pass
                        elif ack_cmd == CMD_RUN_FEEDBACK:
                            model.control_enabled = int(applied_mode) == SYSTEM_STATE_RUN_FEEDBACK
                        elif ack_cmd == CMD_PAUSE:
                            model.control_enabled = False
                            model.system_state = SYSTEM_STATE_PAUSED
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

        return self.commit(
            model,
            emitted,
            owner._apply_connectivity_visibility,
        )
