"""ui_control_state.py - MMHand Desktop 模块契约。

职责：根据 HandModel 快照刷新控制状态、门禁和提示。
线程：Qt GUI 主线程执行；耗时工作必须委托后台服务，禁止阻塞界面事件循环。
输入：用户操作、HandModel 只读快照以及 HandBackend/服务返回结果。
输出：Qt 控件状态、用户提示和经门面提交的应用命令。
状态所有权：仅拥有本视图控件和瞬时交互状态；设备真值由 core 层拥有。
禁止依赖：禁止直接导入串口通信和底层控制实现，禁止在 UI 中复制协议或安全规则。
"""

import math
import time

from .ui_control_common import (
    ControlMode,
    ControlScopeModel,
    DIRECT_REJECT_EVENT_QUEUE,
    DIRECT_REJECT_FAULT,
    DIRECT_REJECT_HARDWARE,
    DIRECT_REJECT_MULTITURN,
    DIRECT_REJECT_NONE,
    DIRECT_REJECT_STALE,
    DIRECT_REJECT_STATE,
    ENCODER_COUNT,
    HandModel,
    JointWorkspaceRange,
    List,
    MOTOR_ABS_CMD_MAX,
    MOTOR_ABS_CMD_MIN,
    MOTOR_COUNT,
    MOTOR_DIR_HINT_CLEAR_STABLE_FRAMES,
    MOTOR_DIR_HINT_CLEAR_TOL,
    MOTOR_JUMP_RESYNC_THRESH,
    MOTOR_FLASH_RAW_WARN_COUNTS,
    MOTOR_SINGLE_TURN_STEPS,
    MOTOR_SLIDER_MAX_TARGET_DELTA,
    MOTOR_SLIDER_WRAP_MARGIN,
    MOTOR_TARGET_SCALE_HI,
    MOTOR_TARGET_SCALE_LO,
    MOTOR_TO_JOINT_INDEX,
    Optional,
    QtGui,
    QtWidgets,
    SERVO_ROUTE_UNKNOWN,
    SERVO_ROUTE_PROBING,
    SERVO_ROUTE_ONLINE,
    SERVO_ROUTE_OFFLINE,
    SERVO_ROUTE_CONFLICT,
    SERVO_ROUTE_BUS_OVERLOADED,
    SYSTEM_STATE_CALIBRATION_RUNNING,
    SYSTEM_STATE_IDLE,
    SYSTEM_STATE_READY,
    SYSTEM_STATE_RECOVERY_REQUIRED,
    SYSTEM_STATE_RUN_FEEDBACK,
    Tuple,
    VALID_MOTOR_MASK,
    clamp_motor_abs,
    motor_target_from_servo_feedback,
    motor_tendon_shorten_abs_sign,
)
from .ui_dialogs import show_scrollable_message
from desktop.core.safety_rules import explain_direct_reject


class ControlPanelStateMixin:
    _ENCODER_ALL_MASK = (1 << ENCODER_COUNT) - 1
    _ENCODER_COMMON_ZERO_CONFIRM_S = 0.20
    _ENCODER_COMMON_ZERO_RECOVERY_FRAMES = 3
    _ROS2_TARGET_STALE_AFTER_S = 0.50
    _ENCODER_COMMON_BUS_MESSAGE = (
        "检测到21路磁编Raw持续为0x0000，但固件仍报告全部在线：疑似公共SPI MISO被拉低。"
        "优先检查或更换图三末端接回S3的磁编FPC，并检查连接器MISO与GND是否短路。"
    )

    def update_state(self, state) -> None:
        """Run the update state workflow."""
        self._state = state
        # active scope、RESET、Replay 等路径会在后端撤销 RAW IK 授权；下一帧状态到达时
        # 同步清除旧 ROS2 目标，避免用户把历史值误认为仍在实时下发。
        raw_ik_enabled_getter = getattr(self.backend, "raw_ik_output_enabled", None)
        raw_ik_enabled = bool(raw_ik_enabled_getter()) if callable(raw_ik_enabled_getter) else False
        if any(value is not None for value in self._ros2_target_values) and not raw_ik_enabled:
            self.clear_ros2_joint_targets()
        self._update_encoder_common_bus_guard(state)
        unknown_bitmap = int(getattr(state, "multi_turn_unknown_bitmap", 0) or 0) & VALID_MOTOR_MASK
        if unknown_bitmap == 0:
            self.reset_turn_unknown_popup_episode()
        overflow_bitmap = int(
            getattr(state, "servo_turn_overflow_bitmap", 0) or 0
        ) & VALID_MOTOR_MASK
        if overflow_bitmap == 0:
            self.reset_servo_turn_overflow_popup_episode()
        else:
            self._show_servo_turn_overflow_guide(overflow_bitmap)
        self._handle_firmware_epoch_change(state)
        self._handle_servo_route_status(state)
        self._handle_connectivity_transitions(state)
        self._handle_direct_command_result(state)
        self._update_p4_rope_init_progress(state)
        calib_status = str(getattr(state, "calib_status", "IDLE"))
        active_calib_statuses = {"PENDING", "RUNNING"}
        partial_completed_mask = (
            self._partial_calibration_completed_joint_mask(state)
            if calib_status == "FAILED"
            else 0
        )
        calib_finished_success = (
            (
                calib_status in {"SUCCESS", "WARN"}
                or partial_completed_mask != 0
            )
            and self._last_calib_status in active_calib_statuses
        )
        if calib_finished_success:
            self._joint_target_initialized = False
        joint_calib_valid_mask = self._joint_calib_valid_mask(state)
        if self._last_joint_calib_valid_mask is not None and joint_calib_valid_mask != self._last_joint_calib_valid_mask:
            self._joint_target_initialized = False
            if joint_calib_valid_mask == 0:
                self._joint_target_user_set = [False] * ENCODER_COUNT
        self._last_joint_calib_valid_mask = joint_calib_valid_mask
        # FAILED 状态包可能先于最终标定诊断和成功关节快照到达。
        # 此处不能提前清掉 pending mask，否则部分成功结果无法合并。
        if (
            calib_status == "IDLE"
            and self._last_calib_status
            in (active_calib_statuses | {"SUCCESS", "WARN", "FAILED"})
        ):
            self._pending_calib_joint_mask = 0
            self._pending_calib_motor_mask = 0
            self._pending_calib_result_baselines = {}
        self._last_calib_status = calib_status
        self._update_connection_lights()
        self._refresh_nvs_control_button()
        self._check_feedback_abs_limit_violation()
        self._maybe_show_calibration_assembly_warning(state)
        self._maybe_save_calibration_snapshot(state)
        if self._app_mode != "manual":
            updater = getattr(self._body_widget, "update_state", None)
            if callable(updater):
                updater(state)
            return
        if self.device_check_view is not None:
            self.device_check_view.update_state(state)
        self._refresh_joint_feedback_view(
            sync_target=(not self._joint_target_initialized) or calib_finished_success
        )
        self._refresh_motor_feedback_view(sync_target=not self._motor_target_initialized)
        self._update_manual_action_buttons()

    @classmethod
    def _encoder_common_zero_snapshot(cls, state) -> bool:
        """判断当前新鲜Raw快照是否符合“公共MISO被拉低”的假在线特征。"""
        if state is None or not bool(getattr(state, "has_encoder_raw_data", False)):
            return False
        if not bool(getattr(state, "has_encoder_health_status", False)):
            return False
        online_mask = int(getattr(state, "encoder_online_bitmap", 0) or 0)
        fault_mask = int(getattr(state, "encoder_fault_bitmap", 0) or 0)
        if (online_mask & cls._ENCODER_ALL_MASK) != cls._ENCODER_ALL_MASK or fault_mask != 0:
            return False
        encoders = list(getattr(state, "encoders", []) or [])
        if len(encoders) < ENCODER_COUNT:
            return False
        for encoder in encoders[:ENCODER_COUNT]:
            if bool(getattr(encoder, "raw_error", getattr(encoder, "error", True))):
                return False
            if int(getattr(encoder, "raw_sensor", 0) or 0) != 0:
                return False
        return True

    def _apply_encoder_common_bus_fault_overlay(self, state) -> None:
        """只覆盖上位机本地健康位图，不改变下位机状态或通信协议。"""
        if state is None or not self._encoder_common_bus_fault:
            return
        state.encoder_online_bitmap = int(
            getattr(state, "encoder_online_bitmap", 0) or 0
        ) & ~self._ENCODER_ALL_MASK
        state.encoder_warn_bitmap = int(
            getattr(state, "encoder_warn_bitmap", 0) or 0
        ) & ~self._ENCODER_ALL_MASK
        state.encoder_fault_bitmap = int(
            getattr(state, "encoder_fault_bitmap", 0) or 0
        ) | self._ENCODER_ALL_MASK

    def _update_encoder_common_bus_guard(self, state) -> None:
        """对21路全零假在线做200ms确认，并在恢复后连续3帧解除。"""
        if state is None:
            return
        sample_ts = float(getattr(state, "encoder_raw_last_rx_ts", 0.0) or 0.0)
        is_new_raw_sample = (
            sample_ts > 0.0 and
            sample_ts != self._encoder_common_zero_last_sample_ts
        )
        if is_new_raw_sample:
            self._encoder_common_zero_last_sample_ts = sample_ts
            zero_snapshot = self._encoder_common_zero_snapshot(state)
            encoders = list(getattr(state, "encoders", []) or [])
            valid_nonzero_snapshot = (
                len(encoders) >= ENCODER_COUNT and
                all(
                    not bool(getattr(encoder, "raw_error", getattr(encoder, "error", True)))
                    for encoder in encoders[:ENCODER_COUNT]
                ) and
                any(
                    int(getattr(encoder, "raw_sensor", 0) or 0) != 0
                    for encoder in encoders[:ENCODER_COUNT]
                )
            )

            if not self._encoder_common_bus_fault:
                self._encoder_common_zero_recovery_frames = 0
                if zero_snapshot:
                    if self._encoder_common_zero_first_ts <= 0.0:
                        self._encoder_common_zero_first_ts = sample_ts
                    elif (
                        sample_ts - self._encoder_common_zero_first_ts
                        >= self._ENCODER_COMMON_ZERO_CONFIRM_S
                    ):
                        self._encoder_common_bus_fault = True
                        self.context.emit_action(self._ENCODER_COMMON_BUS_MESSAGE)
                        self._set_status(self._ENCODER_COMMON_BUS_MESSAGE)
                else:
                    self._encoder_common_zero_first_ts = 0.0
            elif valid_nonzero_snapshot:
                self._encoder_common_zero_recovery_frames += 1
                if (
                    self._encoder_common_zero_recovery_frames
                    >= self._ENCODER_COMMON_ZERO_RECOVERY_FRAMES
                ):
                    self._encoder_common_bus_fault = False
                    self._encoder_common_zero_first_ts = 0.0
                    self._encoder_common_zero_recovery_frames = 0
                    message = "磁编公共SPI全零异常已解除；请转动关节确认Raw值连续变化。"
                    self.context.emit_action(message)
                    self._set_status(message)
            else:
                self._encoder_common_zero_recovery_frames = 0

        # 后续状态包会再次带回固件的全在线位图，因此故障锁存期间每次刷新都要覆盖。
        self._apply_encoder_common_bus_fault_overlay(state)

    def _handle_direct_command_result(self, state) -> None:
        """把“已提交串口帧”和“P4 已接受执行”区分为两个阶段。"""
        if state is None or not bool(
            getattr(state, "has_motion_result_status", False)
        ):
            return
        generation = int(
            getattr(state, "pending_direct_generation", 0) or 0
        ) & 0xFFFFFFFF
        applied_generation = int(
            getattr(state, "applied_direct_generation", 0) or 0
        ) & 0xFFFFFFFF
        reject_reason = int(
            getattr(state, "direct_reject_reason", DIRECT_REJECT_NONE) or 0
        )

        # 首帧只建立设备当前代次基线，避免重连时把上一次会话的结果重新弹出。
        if self._last_reported_direct_generation is None:
            self._last_reported_direct_generation = generation
            return
        if generation == self._last_reported_direct_generation:
            return

        if generation != 0 and applied_generation == generation:
            self._last_reported_direct_generation = generation
            self.context.emit_action(
                f"Direct command executed by P4: generation={generation}"
            )
            return
        if reject_reason == DIRECT_REJECT_NONE:
            # 状态包可能落在“已入队、状态机尚未仲裁”的短窗口，继续等待下一帧。
            return

        self._last_reported_direct_generation = generation
        reason_text = explain_direct_reject(reject_reason)
        message = (
            f"Direct command rejected by P4: generation={generation}, "
            f"reason={reason_text}"
        )
        self.context.emit_action(message)
        self._set_status(message)

    def _reset_motor_target_cache_for_device_epoch(self, reason: str) -> None:
        """Reset the motor target cache for device epoch state."""
        self._motor_target_initialized = False
        self._motor_abs_pos = [None] * MOTOR_COUNT
        self._motor_session_origin_abs = [None] * MOTOR_COUNT
        self._motor_last_slider = [None] * MOTOR_COUNT
        self._motor_last_sent_abs = [None] * MOTOR_COUNT
        self._last_reported_direct_generation = None
        self._motor_target_user_set = [False] * MOTOR_COUNT
        self._clear_run_states()
        self.context.emit_action(reason)

    def _handle_firmware_epoch_change(self, state) -> None:
        """Handle handle firmware epoch change behavior for the control panel."""
        if state is None or not bool(getattr(state, "has_system_status_data", False)):
            return
        try:
            transition_count = int(getattr(state, "state_transition_count", 0))
        except (TypeError, ValueError):
            transition_count = 0
        try:
            unknown_bitmap = int(getattr(state, "multi_turn_unknown_bitmap", 0) or 0)
        except (TypeError, ValueError):
            unknown_bitmap = 0

        if self._last_state_transition_count is not None and transition_count < self._last_state_transition_count:
            self._reset_motor_target_cache_for_device_epoch(
                "Firmware reboot detected: motor targets re-synced to feedback"
            )
        elif self._last_multi_turn_unknown_bitmap is not None and unknown_bitmap != self._last_multi_turn_unknown_bitmap:
            self._reset_motor_target_cache_for_device_epoch(
                "Turn status changed: motor targets re-synced to feedback"
            )

        self._last_state_transition_count = transition_count
        self._last_multi_turn_unknown_bitmap = unknown_bitmap

    def _handle_connectivity_transitions(self, state) -> None:
        """Handle handle connectivity transitions behavior for the control panel."""
        if state is None:
            return
        if bool(getattr(state, "has_connectivity_status", False)):
            servo_mask = int(getattr(state, "servo_online_bitmap", 0) or 0) & VALID_MOTOR_MASK
            encoder_mask = int(getattr(state, "encoder_online_bitmap", 0) or 0) & ((1 << ENCODER_COUNT) - 1)
            encoder_warn_mask = int(getattr(state, "encoder_warn_bitmap", 0) or 0) & ((1 << ENCODER_COUNT) - 1)
            encoder_fault_mask = int(getattr(state, "encoder_fault_bitmap", 0) or 0) & ((1 << ENCODER_COUNT) - 1)
        else:
            servo_mask = 0
            for idx, online in enumerate(getattr(state, "servo_online", [])[:MOTOR_COUNT]):
                if online:
                    servo_mask |= (1 << idx)
            encoder_mask = 0
            encoder_warn_mask = 0
            encoder_fault_mask = 0
            if bool(getattr(state, "has_sensor_data", False)):
                for idx, encoder in enumerate(getattr(state, "encoders", [])[:ENCODER_COUNT]):
                    if not bool(getattr(encoder, "error", False)):
                        encoder_mask |= (1 << idx)

        if self._last_servo_online_bitmap is not None and servo_mask != self._last_servo_online_bitmap:
            newly_offline = self._last_servo_online_bitmap & ~servo_mask
            newly_online = servo_mask & ~self._last_servo_online_bitmap
            if newly_offline:
                message = f"Servo offline: {self._format_motor_mask(newly_offline)}, stale values hidden and targets cleared"
                self.context.emit_action(message)
                self._set_status(message)
            if newly_online:
                unknown = int(getattr(state, "multi_turn_unknown_bitmap", 0) or 0)
                trusted = newly_online & ~unknown
                untrusted = newly_online & unknown
                parts: List[str] = []
                if trusted:
                    parts.append(f"trusted {self._format_motor_mask(trusted)}")
                if untrusted:
                    parts.append(f"TURN UNKNOWN {self._format_motor_mask(untrusted)}")
                message = "Servo online: " + "; ".join(parts)
                self.context.emit_action(message)
                self._set_status(message)
            self._reset_motor_target_cache_for_device_epoch("Servo connectivity changed: motor targets cleared")

        if self._last_encoder_online_bitmap is not None and encoder_mask != self._last_encoder_online_bitmap:
            newly_offline = self._last_encoder_online_bitmap & ~encoder_mask
            newly_online = encoder_mask & ~self._last_encoder_online_bitmap
            parts: List[str] = []
            if newly_offline:
                parts.append(f"offline {self._format_joint_indices(ControlScopeModel.ordered_joint_indices_from_mask(newly_offline))}, stale values hidden")
            if newly_online:
                parts.append(f"online {self._format_joint_indices(ControlScopeModel.ordered_joint_indices_from_mask(newly_online))}")
            if parts:
                message = "Encoder status: " + "; ".join(parts)
                self.context.emit_action(message)
                self._set_status(message)

        if bool(getattr(state, "has_encoder_health_status", False)):
            if self._last_encoder_warn_bitmap is not None and encoder_warn_mask != self._last_encoder_warn_bitmap:
                new_warn = encoder_warn_mask & ~self._last_encoder_warn_bitmap
                cleared_warn = self._last_encoder_warn_bitmap & ~encoder_warn_mask
                parts: List[str] = []
                if new_warn:
                    parts.append(f"warn {self._format_joint_indices(ControlScopeModel.ordered_joint_indices_from_mask(new_warn))}")
                if cleared_warn:
                    parts.append(f"warn cleared {self._format_joint_indices(ControlScopeModel.ordered_joint_indices_from_mask(cleared_warn))}")
                if parts:
                    message = "Encoder health: " + "; ".join(parts)
                    self.context.emit_action(message)
                    self._set_status(message)
            if self._last_encoder_fault_bitmap is not None and encoder_fault_mask != self._last_encoder_fault_bitmap:
                new_fault = encoder_fault_mask & ~self._last_encoder_fault_bitmap
                cleared_fault = self._last_encoder_fault_bitmap & ~encoder_fault_mask
                parts = []
                if new_fault:
                    parts.append(f"fault {self._format_joint_indices(ControlScopeModel.ordered_joint_indices_from_mask(new_fault))}")
                if cleared_fault:
                    parts.append(f"fault cleared {self._format_joint_indices(ControlScopeModel.ordered_joint_indices_from_mask(cleared_fault))}")
                if parts:
                    message = "Encoder health: " + "; ".join(parts)
                    self.context.emit_action(message)
                    self._set_status(message)

        self._last_servo_online_bitmap = servo_mask
        self._last_encoder_online_bitmap = encoder_mask
        self._last_encoder_warn_bitmap = encoder_warn_mask
        self._last_encoder_fault_bitmap = encoder_fault_mask

    def _handle_servo_route_status(self, state) -> None:
        """在扫描提交后一次性提示重复 ID 或单 BUS 超过十台的布线错误。"""
        if not bool(getattr(state, "has_servo_route_status", False)):
            return
        conflict_mask = int(getattr(state, "servo_route_conflict_bitmap", 0) or 0)
        overload_mask = int(getattr(state, "servo_bus_overload_mask", 0) or 0)
        if conflict_mask == 0 and overload_mask == 0:
            self._last_servo_route_warning_signature = None
            return
        counts = tuple(int(value) for value in getattr(state, "servo_bus_device_count", [])[:4])
        signature = (conflict_mask, overload_mask, counts)
        if signature == self._last_servo_route_warning_signature:
            return
        self._last_servo_route_warning_signature = signature

        lines = ["P4 已停止存在布线冲突的舵机路由："]
        if conflict_mask:
            lines.append(
                "- 重复 ID：" + self._format_motor_mask(conflict_mask)
                + "；同一个 ID 只能出现在一条 BUS 上。"
            )
        for bus in range(min(4, len(counts))):
            if (overload_mask & (1 << bus)) != 0:
                lines.append(
                    f"- BUS{bus} 检测到 {counts[bus]} 个响应 ID，最多允许 10 个；"
                    "请把部分舵机移动到其他 BUS。"
                )
        message = "\n".join(lines)
        self.context.emit_action(message.replace("\n", " "))
        self._set_status("Servo route blocked: check duplicate IDs or BUS capacity")
        show_scrollable_message(
            self,
            "舵机 BUS 路由异常",
            message,
            severity="warning",
        )

    def clear_live_state(self) -> None:
        self.stop_all_tests("stopped on disconnect", update_status=False)
        self._reset_p4_rope_init_progress()
        self._state = None
        self._startup_device_check_epoch += 1
        self._startup_device_check_running = False
        self._startup_missing_warning_shown = False
        if self.device_check_view is not None:
            self.device_check_view.clear_state()
        self._update_connection_lights()
        self._refresh_nvs_control_button()
        self._joint_target_initialized = False
        self._motor_target_initialized = False
        self._last_calib_status = None
        self._calibration_warning_popup_mask = 0
        self._motor_abs_pos = [None] * MOTOR_COUNT
        self._motor_session_origin_abs = [None] * MOTOR_COUNT
        self._motor_dir_hint = [0] * MOTOR_COUNT
        self._motor_dir_hint_pending_clear = [False] * MOTOR_COUNT
        self._motor_dir_hint_stable_count = [0] * MOTOR_COUNT
        self._motor_last_slider = [None] * MOTOR_COUNT
        self._motor_slider_dragging = [False] * MOTOR_COUNT
        self._motor_last_sent_abs = [None] * MOTOR_COUNT
        self._last_reported_direct_generation = None
        self._last_state_transition_count = None
        self._last_multi_turn_unknown_bitmap = None
        self.reset_turn_unknown_popup_episode()
        self._last_servo_online_bitmap = None
        self._last_servo_route_warning_signature = None
        self._last_encoder_online_bitmap = None
        self._last_encoder_warn_bitmap = None
        self._last_encoder_fault_bitmap = None
        self._encoder_common_zero_first_ts = 0.0
        self._encoder_common_zero_last_sample_ts = 0.0
        self._encoder_common_zero_recovery_frames = 0
        self._encoder_common_bus_fault = False
        self._joint_target_user_set = [False] * ENCODER_COUNT
        self._motor_target_user_set = [False] * MOTOR_COUNT
        self.clear_ros2_joint_targets()
        self._clear_run_states()
        self._updating = True
        try:
            for item in self.joint_current_items + self.joint_encoder_items + self.joint_abs_limit_items:
                item.setText("-")
                item.setToolTip("")
            for item in (
                self.motor_turn_items
                + self.motor_raw_items
                + self.motor_flash_raw_items
                + self.motor_abs_items
                + self.motor_load_items
                + self.motor_joint_items
                + self.motor_bus_items
                + self.motor_route_items
            ):
                if item is not None:
                    item.setText("-")
                    item.setToolTip("")
                    item.setForeground(QtGui.QBrush(QtGui.QColor("#000000")))
            for i in range(ENCODER_COUNT):
                self._set_joint_encoder_dot(i, None)
        finally:
            self._updating = False

    def update_ros2_joint_targets(self, payload) -> None:
        """接收 ROS2 最终目标并刷新 Joint Channels。

        该函数只在 Qt 主线程调用。被拒绝的帧不会覆盖最近一次成功下发的目标，
        防止把未执行的输入误显示为 P4 已接收命令。
        """
        if not isinstance(payload, dict):
            return
        source = str(payload.get("source", "") or "")
        if not source.upper().startswith("ROS2:"):
            return

        self._ros2_target_last_sent = bool(payload.get("sent", False))
        self._ros2_target_message = str(payload.get("message", "") or "")
        if not self._ros2_target_last_sent:
            self._refresh_ros2_target_view()
            return

        try:
            output_values = [float(value) for value in payload.get("output_targets", [])]
        except (TypeError, ValueError):
            return
        if len(output_values) != ENCODER_COUNT or not all(
            math.isfinite(value) for value in output_values
        ):
            return

        try:
            input_values = [float(value) for value in payload.get("input_targets", [])]
        except (TypeError, ValueError):
            input_values = []
        if len(input_values) != ENCODER_COUNT or not all(
            math.isfinite(value) for value in input_values
        ):
            input_values = list(output_values)

        valid_mask = (1 << ENCODER_COUNT) - 1
        command_mask = payload.get("command_joint_mask")
        if command_mask is None:
            command_mask = int(getattr(self._state, "active_joint_mask", 0) or 0)
        try:
            self._ros2_target_command_mask = int(command_mask) & valid_mask
        except (TypeError, ValueError):
            self._ros2_target_command_mask = 0

        clamped_mask = 0
        for joint_idx in payload.get("clamped", []) or []:
            try:
                idx = int(joint_idx)
            except (TypeError, ValueError):
                continue
            if 0 <= idx < ENCODER_COUNT:
                clamped_mask |= 1 << idx

        self._ros2_target_values = output_values
        self._ros2_target_input_values = input_values
        self._ros2_target_clamped_mask = clamped_mask
        self._ros2_target_updated_at = time.monotonic()
        self._refresh_ros2_target_view()

    def clear_ros2_joint_targets(self) -> None:
        """清除 ROS2 目标快照，避免停机或断连后继续显示旧命令。"""
        self._ros2_target_values = [None] * ENCODER_COUNT
        self._ros2_target_input_values = [None] * ENCODER_COUNT
        self._ros2_target_command_mask = 0
        self._ros2_target_clamped_mask = 0
        self._ros2_target_last_sent = False
        self._ros2_target_message = ""
        self._ros2_target_updated_at = 0.0
        self._refresh_ros2_target_view()

    def _refresh_ros2_target_view(self) -> None:
        """根据发送范围、钳位状态和新鲜度刷新21路 ROS2 Target。"""
        items = getattr(self, "joint_ros2_target_items", [])
        if not items:
            return
        age_s = (
            time.monotonic() - self._ros2_target_updated_at
            if self._ros2_target_updated_at > 0.0
            else float("inf")
        )
        stale = age_s >= self._ROS2_TARGET_STALE_AFTER_S
        for joint_idx in range(min(ENCODER_COUNT, len(items))):
            item = items[joint_idx]
            if item is None:
                continue
            value = self._ros2_target_values[joint_idx]
            if value is None:
                self._set_item_text_if_changed(item, "-")
                item.setForeground(QtGui.QBrush(QtGui.QColor("#9aa0a6")))
                self._set_item_tooltip_if_changed(
                    item,
                    self._ros2_target_message or "No successfully sent ROS2 target",
                )
                continue

            in_command_scope = (
                self._ros2_target_command_mask & (1 << joint_idx)
            ) != 0
            clamped = (self._ros2_target_clamped_mask & (1 << joint_idx)) != 0
            self._set_item_text_if_changed(item, f"{value:.2f}")
            if not in_command_scope:
                color = "#9aa0a6"
                tooltip = (
                    f"ROS2 value received, but J{joint_idx:02d} was outside "
                    "the command scope and was not sent"
                )
            elif not self._ros2_target_last_sent:
                color = "#9aa0a6"
                tooltip = self._ros2_target_message or "Latest ROS2 target was rejected"
            elif stale:
                color = "#9aa0a6"
                tooltip = (
                    "Last ROS2 target is stale; no fresh frame has been sent "
                    f"for {age_s * 1000.0:.0f} ms"
                )
            elif clamped:
                color = "#d97706"
                raw_value = self._ros2_target_input_values[joint_idx]
                tooltip = (
                    f"ROS2 target clamped from {raw_value:.2f} deg to "
                    f"{value:.2f} deg before sending"
                )
            else:
                color = "#1565c0"
                tooltip = "Latest ROS2 target successfully sent to P4"
            item.setForeground(QtGui.QBrush(QtGui.QColor(color)))
            self._set_item_tooltip_if_changed(item, tooltip)

    def stop_all_tests(self, reason: str = "stopped", update_status: bool = True) -> None:
        """停止全部由上位机生成目标轨迹的测试。"""
        self._on_stop_test(reason=reason, update_status=update_status)
        self._on_stop_ff_test(reason=reason, update_status=update_status)
        self._on_stop_tri_test(reason=reason, update_status=update_status)

    def _guard_joint_command_started(self) -> bool:
        """Check whether the joint command started guard passes."""
        self.backend.set_control_mode(ControlMode.JOINT_ANGLE)
        system_state = getattr(self._state, "system_state", None) if self._state is not None else None
        if system_state in (
            SYSTEM_STATE_IDLE,
            SYSTEM_STATE_READY,
            SYSTEM_STATE_RECOVERY_REQUIRED,
        ):
            if not self._apply_active_scope_for_current_mode(force_joint=True):
                return False
        if not self.feedback_command_allowed(show_status=True):
            return False
        # ACTIVE_SCOPE may have just moved P4 from IDLE to READY, but this UI
        # callback still holds the preceding status packet. Let P4 validate
        # RUN_FEEDBACK instead of falsely requiring a later UI action to
        # refresh the desktop-side state cache.
        if system_state not in (
            SYSTEM_STATE_IDLE,
            SYSTEM_STATE_READY,
            SYSTEM_STATE_RECOVERY_REQUIRED,
            SYSTEM_STATE_RUN_FEEDBACK,
        ):
            if self._estop_latched:
                self._set_status("Command blocked: press ENABLE after E-STOP")
            else:
                self._set_status("Joint command blocked: controller is not READY")
            return False
        self.backend.set_pid_control(True)
        if self.backend.is_started():
            return True
        if self._estop_latched:
            self._set_status("Command blocked: press ENABLE after E-STOP")
            return False
        self._set_status("Joint command blocked: press ENABLE first")
        return False

    @staticmethod
    def _slider_from_abs_pos(abs_pos: int) -> int:
        """Handle slider from abs pos behavior for the control panel."""
        return int(abs_pos) % MOTOR_SINGLE_TURN_STEPS

    @staticmethod
    def _single_turn_diff(a: int, b: int) -> int:
        """Handle single turn diff behavior for the control panel."""
        diff = abs(int(a) - int(b)) % MOTOR_SINGLE_TURN_STEPS
        return min(diff, MOTOR_SINGLE_TURN_STEPS - diff)

    def _motor_display_valid(self, idx: int, state: Optional[HandModel] = None) -> bool:
        """Compute or update motor display valid data."""
        state = self._state if state is None else state
        if idx < 0 or idx >= MOTOR_COUNT or state is None:
            return False
        if bool(getattr(state, "has_connectivity_status", False)):
            return (int(getattr(state, "servo_online_bitmap", 0) or 0) & (1 << idx)) != 0
        if idx < len(getattr(state, "servo_online", []) or []) and getattr(state, "servo_online", [])[idx]:
            return True
        if idx < len(getattr(state, "servo_raw_online", []) or []) and getattr(state, "servo_raw_online", [])[idx]:
            return True
        if idx < len(getattr(state, "servo_telem_online", []) or []) and getattr(state, "servo_telem_online", [])[idx]:
            return True
        return False

    def _motor_route_controllable(self, idx: int, state: Optional[HandModel] = None) -> bool:
        """仅认可 P4 动态扫描已提交且未冲突、未超容量的唯一 BUS 路由。"""
        state = self._state if state is None else state
        if idx < 0 or idx >= MOTOR_COUNT or state is None:
            return False
        if not bool(getattr(state, "has_servo_route_status", False)):
            return False
        mask = int(getattr(state, "servo_route_controllable_bitmap", 0) or 0)
        return (mask & (1 << idx)) != 0

    def _active_scope_unroutable_motor_indices(self, state=None) -> List[int]:
        """返回当前作用域内没有唯一可控 BUS 路由的舵机。"""
        state = self._state if state is None else state
        active_mask = self._active_motor_mask_from_ui() & VALID_MOTOR_MASK
        return [
            idx for idx in range(MOTOR_COUNT)
            if (active_mask & (1 << idx)) != 0
            and not self._motor_route_controllable(idx, state)
        ]

    def _refresh_motor_route_item(self, motor_idx: int, state) -> None:
        """刷新单台舵机的动态 BUS 与路由状态，不推断或回退到固定映射。"""
        if motor_idx >= len(self.motor_bus_items) or motor_idx >= len(self.motor_route_items):
            return
        bus_item = self.motor_bus_items[motor_idx]
        route_item = self.motor_route_items[motor_idx]
        if bus_item is None or route_item is None:
            return
        bus_item.setText("-")
        route_item.setForeground(QtGui.QBrush(QtGui.QColor("#6b7280")))
        if not bool(getattr(state, "has_servo_route_status", False)):
            route_item.setText("Waiting")
            route_item.setToolTip("Waiting for P4 dynamic servo route status")
            return
        route_buses = list(getattr(state, "servo_route_bus", []) or [])
        route_states = list(getattr(state, "servo_route_state", []) or [])
        route_bus = int(route_buses[motor_idx]) if motor_idx < len(route_buses) else 0xFF
        route_state = int(route_states[motor_idx]) if motor_idx < len(route_states) else SERVO_ROUTE_UNKNOWN
        if 0 <= route_bus < 4:
            bus_item.setText(f"BUS{route_bus}")
        labels = {
            SERVO_ROUTE_UNKNOWN: "Unknown",
            SERVO_ROUTE_PROBING: "Scanning",
            SERVO_ROUTE_ONLINE: "Online",
            SERVO_ROUTE_OFFLINE: "Offline",
            SERVO_ROUTE_CONFLICT: "ID Conflict",
            SERVO_ROUTE_BUS_OVERLOADED: "BUS Full",
        }
        route_item.setText(labels.get(route_state, f"State {route_state}"))
        if route_state == SERVO_ROUTE_ONLINE:
            color = "#2e7d32"
            tip = "Unique dynamic route is online and controllable"
        elif route_state == SERVO_ROUTE_PROBING:
            color = "#b26a00"
            progress = int(getattr(state, "servo_route_scan_progress", 0) or 0)
            tip = f"P4 is scanning servo routes: {progress}%"
        elif route_state == SERVO_ROUTE_CONFLICT:
            color = "#c62828"
            tip = "This servo ID was found on multiple BUSes; control is blocked"
        elif route_state == SERVO_ROUTE_BUS_OVERLOADED:
            color = "#c62828"
            tip = "This BUS has more than 10 servos; move servos to another BUS"
        else:
            color = "#6b7280"
            tip = "No controllable dynamic route"
        route_item.setForeground(QtGui.QBrush(QtGui.QColor(color)))
        route_item.setToolTip(tip)
        bus_item.setToolTip(tip)

    def _active_scope_offline_motor_indices(self, state=None) -> List[int]:
        """返回当前作用域中未在线的舵机；直控门禁不得依赖磁编码器。"""
        state = self._state if state is None else state
        active_mask = self._active_motor_mask_from_ui() & VALID_MOTOR_MASK
        return [
            idx for idx in range(MOTOR_COUNT)
            if (active_mask & (1 << idx)) != 0
            and not self._motor_display_valid(idx, state)
        ]

    def _active_scope_offline_encoder_indices(self, state=None) -> List[int]:
        """返回当前负反馈作用域中离线/无有效 raw 的磁编码器。"""
        state = self._state if state is None else state
        missing: List[int] = []
        for idx in self._active_scope_joint_indices():
            status = self._joint_encoder_status(idx, state)
            if status == "fault":
                missing.append(idx)
                continue
            # 旧固件没有 encoder health 位图时，以 raw 数据有效性兜底。
            if status is None and not self._joint_raw_feedback_valid(idx, state):
                missing.append(idx)
        return missing

    def _read_motor_target_slider_int(self, idx: int) -> int:
        """Read the motor target slider int value from the UI."""
        if 0 <= idx < len(self.motor_sliders):
            return max(MOTOR_TARGET_SCALE_LO, min(MOTOR_TARGET_SCALE_HI, int(self.motor_sliders[idx].value())))
        if 0 <= idx < len(self.motor_target_spins):
            return self._slider_from_abs_pos(int(self.motor_target_spins[idx].value()))
        return MOTOR_TARGET_SCALE_LO

    def _session_abs_from_slider_locked_turn(self, idx: int, slider_value: int, drag_sign: int = 0) -> Optional[int]:
        """Handle session abs from slider locked turn behavior for the control panel."""
        if idx < 0 or idx >= MOTOR_COUNT:
            return None
        slider = max(MOTOR_TARGET_SCALE_LO, min(MOTOR_TARGET_SCALE_HI, int(slider_value)))
        current = self._motor_abs_pos[idx]
        if current is None:
            current = 0
        hint = int(self._motor_dir_hint[idx])
        step = MOTOR_SINGLE_TURN_STEPS
        if hint > 0 or drag_sign > 0:
            k = int((int(current) - slider) // step)
            candidate = slider + k * step
            if candidate < int(current):
                candidate += step
        elif hint < 0 or drag_sign < 0:
            k = int((int(current) - slider) // step)
            candidate = slider + k * step
            if candidate > int(current):
                candidate -= step
        else:
            k = int(round((int(current) - slider) / float(step)))
            candidate = slider + k * step
        return clamp_motor_abs(candidate)

    def _motor_resync_one_channel(self, idx: int) -> None:
        dev_abs = self._motor_feedback_abs(idx)
        if dev_abs is None:
            return
        self._motor_session_origin_abs[idx] = dev_abs
        self._motor_abs_pos[idx] = 0
        self._motor_last_slider[idx] = 0

    def _sync_motor_targets_to_feedback(self) -> bool:
        """Synchronize the motor targets to feedback state."""
        state = self._state
        if state is None or not (
            bool(getattr(state, "has_servo_angle_data", False)) or bool(getattr(state, "has_servo_raw_data", False))
        ):
            return False
        self._refresh_motor_feedback_view(sync_target=True)
        return any(origin is not None for origin in self._motor_session_origin_abs)

    def _motor_feedback_abs(self, idx: int) -> Optional[int]:
        """Compute or update motor feedback abs data."""
        state = self._state
        if idx < 0 or idx >= MOTOR_COUNT or state is None:
            return None
        if not self._motor_display_valid(idx, state):
            return None
        if bool(getattr(state, "has_servo_angle_data", False)):
            if idx < len(getattr(state, "servo_online", [])) and state.servo_online[idx] and idx < len(getattr(state, "servo_angles", [])):
                return clamp_motor_abs(int(state.servo_angles[idx]))
        if bool(getattr(state, "has_servo_raw_data", False)):
            if idx < len(getattr(state, "servo_raw_online", [])) and state.servo_raw_online[idx] and idx < len(getattr(state, "servo_raw_positions", [])):
                return clamp_motor_abs(motor_target_from_servo_feedback(state.servo_raw_positions[idx]))
        return None

    def _motor_raw_single_feedback(self, idx: int) -> Optional[int]:
        """Compute or update motor raw single feedback data."""
        state = self._state
        if idx < 0 or idx >= MOTOR_COUNT or state is None:
            return None
        if not self._motor_display_valid(idx, state):
            return None
        if not bool(getattr(state, "has_servo_raw_data", False)):
            return None
        if idx >= len(getattr(state, "servo_raw_online", [])) or not state.servo_raw_online[idx]:
            return None
        if idx >= len(getattr(state, "servo_raw_positions", [])):
            return None
        return motor_target_from_servo_feedback(state.servo_raw_positions[idx])



    @staticmethod
    def _motor_turn_from_abs(value: int) -> int:
        """Compute or update motor turn from abs data."""
        raw = int(value) % MOTOR_SINGLE_TURN_STEPS
        return (int(value) - raw) // MOTOR_SINGLE_TURN_STEPS

    def _current_motor_hold_targets_from_feedback(self) -> List[int]:
        """Handle current motor hold targets from feedback behavior for the control panel."""
        state = self._state
        targets: List[int] = []
        for i in range(MOTOR_COUNT):
            target: Optional[int] = None
            if state is not None and self._motor_display_valid(i, state) and bool(getattr(state, "has_servo_angle_data", False)):
                if i < len(getattr(state, "servo_online", [])) and state.servo_online[i] and i < len(getattr(state, "servo_angles", [])):
                    target = clamp_motor_abs(int(state.servo_angles[i]))
            if target is None:
                raw_single = self._motor_raw_single_feedback(i)
                if raw_single is not None:
                    target = int(raw_single)
            if (
                target is None and
                state is not None and
                self._motor_display_valid(i, state) and
                i < len(self._motor_last_sent_abs) and
                self._motor_last_sent_abs[i] is not None
            ):
                target = clamp_motor_abs(int(self._motor_last_sent_abs[i]))
            targets.append(0 if target is None else clamp_motor_abs(int(target)))
        return targets

    def _motor_abs_for_command_packet(self, idx: int, state: Optional[HandModel], *, ignore_last_sent: bool = False) -> int:
        relative_target = self._motor_abs_pos[idx]
        origin = self._motor_session_origin_abs[idx]
        if origin is None and state is not None and self._motor_display_valid(idx, state):
            if bool(getattr(state, "has_servo_angle_data", False)):
                if idx < len(getattr(state, "servo_online", [])) and state.servo_online[idx] and idx < len(state.servo_angles):
                    origin = clamp_motor_abs(int(state.servo_angles[idx]))
            elif bool(getattr(state, "has_servo_raw_data", False)):
                if idx < len(getattr(state, "servo_raw_online", [])) and state.servo_raw_online[idx] and idx < len(state.servo_raw_positions):
                    origin = clamp_motor_abs(motor_target_from_servo_feedback(state.servo_raw_positions[idx]))
        if relative_target is None:
            relative_target = 0
            self._motor_abs_pos[idx] = relative_target
        if origin is not None:
            self._motor_session_origin_abs[idx] = origin
            sign = motor_tendon_shorten_abs_sign(idx)
            return clamp_motor_abs(int(origin) + sign * int(relative_target))
        if not ignore_last_sent and self._motor_last_sent_abs[idx] is not None:
            return clamp_motor_abs(int(self._motor_last_sent_abs[idx]))
        return 0

    def _build_motor_abs_command_list(self) -> List[int]:
        """Build the motor abs command list UI section."""
        return [self._motor_abs_for_command_packet(i, self._state) for i in range(MOTOR_COUNT)]

    def _seed_motor_last_sent_after_sync(self, state: HandModel) -> None:
        """Handle seed motor last sent after sync behavior for the control panel."""
        for i in range(MOTOR_COUNT):
            self._motor_last_sent_abs[i] = self._motor_abs_for_command_packet(i, state, ignore_last_sent=True)

    def _motor_command_debug_summary(self, trigger: str, targets: List[int]) -> str:
        """Compute or update motor command debug summary data."""
        idx = self._motor_index_from_trigger(trigger)
        if idx is None:
            idx = 0
        relative = self._motor_abs_pos[idx] if 0 <= idx < len(self._motor_abs_pos) else None
        origin = self._motor_session_origin_abs[idx] if 0 <= idx < len(self._motor_session_origin_abs) else None
        sent = targets[idx] if 0 <= idx < len(targets) else None
        sign = motor_tendon_shorten_abs_sign(idx)
        return (
            f"M{idx:02d} rel={relative if relative is not None else '-'} "
            f"sign={sign:+d} origin={origin if origin is not None else '-'} "
            f"sent={sent if sent is not None else '-'}"
        )

    def _motor_index_from_trigger(self, trigger: str) -> Optional[int]:
        """Compute or update motor index from trigger data."""
        marker = "M"
        pos = str(trigger).rfind(marker)
        if pos >= 0:
            digits = []
            for ch in str(trigger)[pos + 1:]:
                if ch.isdigit():
                    digits.append(ch)
                else:
                    break
            if digits:
                return max(0, min(MOTOR_COUNT - 1, int("".join(digits))))
        return None

    def _send_motor_abs_from_ui(self, trigger: str) -> bool:
        """Handle send motor abs from ui behavior for the control panel."""
        if not self._interaction_enabled:
            return False
        if not self.context.is_comm_connected():
            raise RuntimeError("Serial port is not connected")
        self.backend.set_control_mode(ControlMode.DIRECT_MOTOR)
        self.backend.set_pid_control(False)
        if self._estop_latched and not self.backend.is_started():
            self._set_status("Motor command blocked: press ENABLE after E-STOP")
            return False
        if not self.backend.is_started():
            self._set_status("Motor command blocked: waiting for controllable IDLE after ENABLE")
            return False
        state = self._state
        if state is None or not (
            bool(getattr(state, "has_servo_angle_data", False)) or bool(getattr(state, "has_servo_raw_data", False))
        ):
            self._set_status("Motor command blocked: waiting for motor feedback")
            return False
        unroutable = self._active_scope_unroutable_motor_indices(state)
        if unroutable:
            self._set_status(
                "Motor command blocked: no unique controllable BUS route ("
                + self._format_motor_list(unroutable) + ")"
            )
            return False
        unknown_mask = self._active_scope_multiturn_unknown_mask()
        if unknown_mask != 0:
            self._show_turn_recovery_guide(unknown_mask, f"Motor command ({trigger})")
            return False
        for idx in range(MOTOR_COUNT):
            if self._motor_session_origin_abs[idx] is None:
                self._motor_resync_one_channel(idx)
            if self._motor_abs_pos[idx] is None:
                self._motor_abs_pos[idx] = 0
        targets = self._build_motor_abs_command_list()
        self.backend.set_motor_positions_absolute(targets)
        self._motor_last_sent_abs = list(targets)
        self.context.emit_action(f"Motor absolute positions sent ({trigger}): {self._motor_command_debug_summary(trigger, targets)}")
        return True

    def _send_single_motor_abs_from_ui(self, idx: int, trigger: str) -> Optional[int]:
        """Handle send single motor abs from ui behavior for the control panel."""
        if idx < 0 or idx >= MOTOR_COUNT:
            return None
        if not self._interaction_enabled:
            return None
        if not self.context.is_comm_connected():
            raise RuntimeError("Serial port is not connected")
        self.backend.set_control_mode(ControlMode.DIRECT_MOTOR)
        self.backend.set_pid_control(False)
        if self._estop_latched and not self.backend.is_started():
            self._set_status("Motor command blocked: press ENABLE after E-STOP")
            return None
        if not self.backend.is_started():
            self._set_status("Motor command blocked: waiting for controllable IDLE after ENABLE")
            return None
        state = self._state
        if state is None or not (
            bool(getattr(state, "has_servo_angle_data", False)) or bool(getattr(state, "has_servo_raw_data", False))
        ):
            self._set_status("Motor command blocked: waiting for motor feedback")
            return None
        if not self._motor_route_controllable(idx, state):
            self._set_status(
                f"Motor command blocked: M{idx:02d} has no unique controllable BUS route"
            )
            return None
        unknown_mask = self._active_scope_multiturn_unknown_mask()
        if unknown_mask != 0:
            self._show_turn_recovery_guide(unknown_mask, f"Motor command ({trigger})")
            return None
        if self._motor_session_origin_abs[idx] is None:
            self._motor_resync_one_channel(idx)
        if self._motor_abs_pos[idx] is None:
            self._motor_abs_pos[idx] = 0
        targets = self._build_motor_abs_command_list()
        target_abs = int(targets[idx])
        self.backend.set_single_motor_position_absolute(idx, target_abs)
        if 0 <= idx < len(self._motor_last_sent_abs):
            self._motor_last_sent_abs[idx] = target_abs
        # send_command 只说明帧已进入发送队列，不能提前声称下位机已经执行。
        # 最终结果由 _handle_direct_command_result 根据 generation 异步确认。
        self.context.emit_action(
            f"Single motor absolute position submitted ({trigger}): M{idx:02d}={target_abs}"
        )
        return target_abs

    def _clear_motor_target_after_single_send(self, idx: int, sent_abs: int) -> None:
        """Clear the motor target after single send state."""
        if idx < 0 or idx >= MOTOR_COUNT:
            return
        self._motor_session_origin_abs[idx] = int(sent_abs)
        self._motor_abs_pos[idx] = 0
        self._motor_last_slider[idx] = 0
        self._motor_target_user_set[idx] = False
        self._set_motor_target_widget(idx, 0)

    def _check_feedback_abs_limit_violation(self) -> None:
        """Handle check feedback abs limit violation behavior for the control panel."""
        state = self._state
        if state is None or getattr(state, "system_state", None) != SYSTEM_STATE_RUN_FEEDBACK:
            self._feedback_abs_limit_popup_episode_active = False
            return
        valid_bitmap = int(getattr(state, "motor_abs_limit_valid_bitmap", 0))
        locked_bitmap = int(getattr(state, "motor_abs_limit_locked_bitmap", 0))
        self._abs_limit_warning_unlocked_mask &= valid_bitmap
        monitor_mask = self._active_motor_mask_from_ui()
        if monitor_mask == 0:
            monitor_mask = valid_bitmap
        monitor_mask &= valid_bitmap & locked_bitmap
        monitor_mask &= ~int(getattr(self, "_abs_limit_warning_unlocked_mask", 0) or 0)
        if monitor_mask == 0:
            self._feedback_abs_limit_popup_episode_active = False
            return
        mins = getattr(state, "motor_abs_limit_min", [])
        maxs = getattr(state, "motor_abs_limit_max", [])
        hits: List[Tuple[int, int, int, int]] = []
        for idx in range(min(MOTOR_COUNT, len(mins), len(maxs))):
            if ((monitor_mask >> idx) & 0x01) == 0:
                continue
            feedback_abs = self._motor_feedback_abs(idx)
            if feedback_abs is None:
                continue
            lo = int(mins[idx])
            hi = int(maxs[idx])
            if lo > hi:
                lo, hi = hi, lo
            if feedback_abs < lo or feedback_abs > hi:
                hits.append((idx, int(feedback_abs), lo, hi))
        if hits and not self._feedback_abs_limit_popup_episode_active:
            self._feedback_abs_limit_popup_episode_active = True
            self._show_feedback_abs_limit_violation(hits)
        elif not hits:
            self._feedback_abs_limit_popup_episode_active = False

    def _show_feedback_abs_limit_violation(self, hits: List[Tuple[int, int, int, int]]) -> None:
        """Show the feedback abs limit violation UI message."""
        preview = ", ".join(
            f"M{idx:02d}: abs={feedback_abs} [{lo},{hi}]"
            for idx, feedback_abs, lo, hi in hits[:4]
        )
        suffix = "" if len(hits) <= 4 else f" ... +{len(hits) - 4}"
        self._set_status(f"Feedback ABS out of limit: {preview}{suffix}")
        self._warn(
            "Feedback ABS Limit (default unlocked)",
            "RUN_FEEDBACK servo feedback ABS is outside the calibrated 5-turn locked window.\n"
            "Feedback limits are unlocked by default; use LOCK only when you intentionally need this guard.\n"
            f"{preview}{suffix}",
        )

    def _send_motor_sweep_from_ui(self, trigger: str) -> None:
        """Handle send motor sweep from ui behavior for the control panel."""
        if not self._interaction_enabled:
            return
        if not self.context.is_comm_connected():
            raise RuntimeError("Serial port is not connected")
        self.backend.set_control_mode(ControlMode.DIRECT_MOTOR)
        self.backend.set_pid_control(False)
        if self._estop_latched and not self.backend.is_started():
            self._set_status("Motor command blocked: press ENABLE after E-STOP")
            return
        if not self.backend.is_started():
            self._set_status("Motor command blocked: waiting for controllable IDLE after ENABLE")
            return
        state = self._state
        if state is None or not (
            bool(getattr(state, "has_servo_angle_data", False)) or bool(getattr(state, "has_servo_raw_data", False))
        ):
            self._set_status("Motor command blocked: waiting for motor feedback")
            return
        unroutable = self._active_scope_unroutable_motor_indices(state)
        if unroutable:
            self._set_status(
                "Motor sweep blocked: no unique controllable BUS route ("
                + self._format_motor_list(unroutable) + ")"
            )
            return
        unknown_mask = self._active_scope_multiturn_unknown_mask()
        if unknown_mask != 0:
            self._show_turn_recovery_guide(unknown_mask, f"Motor sweep ({trigger})")
            return
        targets = self.motor_sweep_targets()
        self.backend.set_motor_positions_raw_sweep(targets)
        self.context.emit_action(f"Motor sweep positions sent ({trigger})")

    def _sync_motor_abs_from_slider_before_send(self, idx: int) -> None:
        """Synchronize the motor abs from slider before send state."""
        if idx < 0 or idx >= MOTOR_COUNT:
            return
        if self._motor_session_origin_abs[idx] is None:
            self._motor_resync_one_channel(idx)
        if self._motor_abs_pos[idx] is None:
            self._motor_abs_pos[idx] = 0
        cur = self._read_motor_target_slider_int(idx)
        mapped = self._session_abs_from_slider_locked_turn(idx, cur, 0)
        if mapped is not None:
            self._motor_abs_pos[idx] = mapped
            self._motor_last_slider[idx] = cur

    def _on_motor_slider_press(self, idx: int) -> None:
        """Handle the motor slider press event."""
        if 0 <= idx < MOTOR_COUNT:
            self._motor_slider_dragging[idx] = True

    def _on_motor_slider_release(self, idx: int) -> None:
        """Handle the motor slider release event."""
        if 0 <= idx < MOTOR_COUNT:
            self._motor_slider_dragging[idx] = False

    def _mark_joint_target_user_set(self, idx: int) -> None:
        """Handle mark joint target user set behavior for the control panel."""
        if 0 <= idx < len(self._joint_target_user_set):
            self._joint_target_user_set[idx] = True

    def _mark_motor_target_user_set(self, idx: int) -> None:
        """Handle mark motor target user set behavior for the control panel."""
        if 0 <= idx < len(self._motor_target_user_set):
            self._motor_target_user_set[idx] = True

    def _joint_default_target_max_deg(self, idx: int) -> float:
        """Compute or update joint default target max deg data."""
        return self._joint_target_model.default_max(idx)

    def _joint_default_target_min_deg(self, idx: int) -> float:
        """Compute or update joint default target min deg data."""
        return self._joint_target_model.default_min(idx)

    def _joint_workspace_range(
        self,
        idx: int,
        state: Optional[HandModel] = None,
    ) -> JointWorkspaceRange:
        """Compute or update joint workspace range data."""
        state = self._state if state is None else state
        return self._joint_target_model.workspace(idx, state)

    def _joint_calibrated_target_min_deg(self, idx: int, state: Optional[HandModel] = None) -> Optional[float]:
        """Compute or update joint calibrated target min deg data."""
        state = self._state if state is None else state
        return self._joint_target_model.calibrated_min(idx, state)

    def _joint_target_max_deg(self, idx: int) -> float:
        # The upper target boundary is always the static URDF Max.
        """Compute or update joint target max deg data."""
        return self._joint_target_model.target_max(idx)

    def _joint_target_min_deg(self, idx: int, state: Optional[HandModel] = None) -> float:
        """Compute or update joint target min deg data."""
        state = self._state if state is None else state
        return self._joint_target_model.target_min(idx, state)

    def _clamp_joint_target_deg(self, idx: int, value: float) -> float:
        """Handle clamp joint target deg behavior for the control panel."""
        return self._joint_target_model.clamp(idx, value, self._state)

    def _refresh_joint_target_range(self, idx: int, state: Optional[HandModel]) -> None:
        """Refresh the joint target range display or controls."""
        if idx < 0 or idx >= ENCODER_COUNT:
            return
        workspace = self._joint_workspace_range(idx, state)
        lo = workspace.current_min_deg if workspace.ready else self._joint_default_target_min_deg(idx)
        hi = self._joint_target_max_deg(idx)
        if idx < len(self.joint_min_items):
            item = self.joint_min_items[idx]
            if item is not None:
                self._set_item_text_if_changed(item, f"{lo:.2f}" if workspace.ready else "-")
                self._set_item_tooltip_if_changed(
                    item,
                    "Dynamic Current Min from P4 v3 calibration" if workspace.ready else workspace.reason,
                )
        if idx < len(self.joint_max_items):
            item = self.joint_max_items[idx]
            if item is not None:
                self._set_item_text_if_changed(item, f"{hi:.2f}")
                self._set_item_tooltip_if_changed(item, "Static URDF Max")
        if idx < len(self.joint_target_spins):
            spin = self.joint_target_spins[idx]
            if spin is not None:
                self._set_spin_range_if_changed(spin, lo, hi)
        if idx < len(self.joint_sliders):
            slider = self.joint_sliders[idx]
            if slider is not None:
                self._set_slider_range_if_changed(
                    slider,
                    int(round(lo * 100.0)),
                    int(round(hi * 100.0)),
                )
        controls_enabled = bool(self._interaction_enabled and workspace.ready)
        for widgets in (self.joint_target_spins, self.joint_sliders, self.joint_send_buttons):
            if idx < len(widgets) and widgets[idx] is not None:
                widgets[idx].setEnabled(controls_enabled)

    def _joint_angle_feedback_valid(self, idx: int, state=None) -> bool:
        """Compute or update joint angle feedback valid data."""
        state = self._state if state is None else state
        if state is None or not bool(getattr(state, "has_sensor_data", False)):
            return False
        angles = getattr(state, "angles", []) or []
        if idx < 0 or idx >= len(angles):
            return False
        encoders = getattr(state, "encoders", []) or []
        if idx < len(encoders):
            return not bool(getattr(encoders[idx], "angle_error", getattr(encoders[idx], "error", False)))
        return True

    def _joint_raw_feedback_valid(self, idx: int, state=None) -> bool:
        """Compute or update joint raw feedback valid data."""
        state = self._state if state is None else state
        if state is None or not bool(getattr(state, "has_encoder_raw_data", False)):
            return False
        encoders = getattr(state, "encoders", []) or []
        if idx < 0 or idx >= len(encoders):
            return False
        return not bool(getattr(encoders[idx], "raw_error", getattr(encoders[idx], "error", False)))

    def _joint_encoder_status(self, idx: int, state: Optional[HandModel] = None) -> Optional[str]:
        """Compute or update joint encoder status data."""
        state = self._state if state is None else state
        if idx < 0 or idx >= ENCODER_COUNT or state is None:
            return None
        if bool(getattr(state, "has_encoder_health_status", False)):
            bit = 1 << idx
            online = (int(getattr(state, "encoder_online_bitmap", 0) or 0) & bit) != 0
            warn = (int(getattr(state, "encoder_warn_bitmap", 0) or 0) & bit) != 0
            fault = (int(getattr(state, "encoder_fault_bitmap", 0) or 0) & bit) != 0
            if fault or not online:
                return "fault"
            if warn:
                return "warn"
            return "ok"
        if not bool(getattr(state, "has_sensor_data", False)):
            return None
        encoders = getattr(state, "encoders", []) or []
        if idx >= len(encoders):
            return None
        if bool(getattr(encoders[idx], "angle_error", getattr(encoders[idx], "error", False))):
            return "fault"
        return "ok"

    def _joint_encoder_display_valid(self, idx: int, state: Optional[HandModel] = None) -> bool:
        """Compute or update joint encoder display valid data."""
        return self._joint_encoder_status(idx, state) in ("ok", "warn")

    def _joint_angle_display_calibrated(self, idx: int, state: Optional[HandModel] = None) -> bool:
        """Compute or update joint angle display calibrated data."""
        return self._joint_workspace_range(idx, state).ready

    def _joint_calib_valid_mask(self, state: Optional[HandModel]) -> int:
        """Compute or update joint calib valid mask data."""
        if state is None:
            return 0
        mask = 0
        for idx in range(ENCODER_COUNT):
            if self._joint_workspace_range(idx, state).ready:
                mask |= (1 << idx)
        return mask

    def _joint_feedback_target(self, idx: int) -> float:
        """Compute or update joint feedback target data."""
        state = self._state
        if (
            self._joint_angle_display_calibrated(idx, state) and
            self._joint_encoder_display_valid(idx, state) and
            self._joint_angle_feedback_valid(idx, state)
        ):
            return self._clamp_joint_target_deg(idx, float(state.angles[idx]))
        if idx < len(self.joint_target_spins):
            return self._clamp_joint_target_deg(idx, float(self.joint_target_spins[idx].value()))
        return self._joint_target_min_deg(idx)

    def _joint_angle_batch_suspicious_zero(self, state: Optional[HandModel]) -> bool:
        """Compute or update joint angle batch suspicious zero data."""
        if state is None or not bool(getattr(state, "has_sensor_data", False)):
            return False
        angles = list(getattr(state, "angles", []) or [])
        if len(angles) < ENCODER_COUNT:
            return False
        if any(abs(float(value)) > 0.001 for value in angles[:ENCODER_COUNT]):
            return False
        encoders = list(getattr(state, "encoders", []) or [])
        raw_online = any(
            int(getattr(encoders[i], "raw_sensor", 0) or 0) not in (0, 0xFFFF)
            for i in range(min(ENCODER_COUNT, len(encoders)))
        )
        if not raw_online:
            return False
        calib_active = str(getattr(state, "calib_status", "") or "") in ("PENDING", "RUNNING")
        system_calib = int(getattr(state, "system_state", -1) or -1) == SYSTEM_STATE_CALIBRATION_RUNNING
        return bool(self._joint_target_initialized or calib_active or system_calib)

    def _motor_feedback_target(self, idx: int) -> int:
        """Compute or update motor feedback target data."""
        state = self._state
        if state is not None and self._motor_display_valid(idx, state):
            if bool(getattr(state, "has_servo_raw_data", False)) and idx < len(getattr(state, "servo_raw_positions", [])):
                return motor_target_from_servo_feedback(state.servo_raw_positions[idx])
            if bool(getattr(state, "has_servo_angle_data", False)) and idx < len(getattr(state, "servo_angles", [])):
                return self._slider_from_abs_pos(int(state.servo_angles[idx]))
        return self._read_motor_target_slider_int(idx)

    def _refresh_joint_feedback_view(self, sync_target: bool = False) -> None:
        """Refresh the joint feedback view display or controls."""
        state = self._state
        joint_has_data = bool(state is not None and getattr(state, "has_sensor_data", False))
        suspicious_zero_angles = self._joint_angle_batch_suspicious_zero(state)
        table = getattr(self, "joint_table", None)
        if table is not None:
            table.setUpdatesEnabled(False)
        self._updating = True
        try:
            self._refresh_ros2_target_view()
            for i in range(ENCODER_COUNT):
                self._refresh_joint_target_range(i, state)
                self._refresh_joint_abs_limit_item(i, state)
                encoder_status = self._joint_encoder_status(i, state)
                display_valid = encoder_status in ("ok", "warn")
                angle_feedback_valid = self._joint_angle_feedback_valid(i, state)
                raw_feedback_valid = self._joint_raw_feedback_valid(i, state)
                angle_calibrated = self._joint_angle_display_calibrated(i, state)
                self._set_joint_encoder_dot(i, encoder_status)
                if (
                    display_valid and
                    angle_feedback_valid and
                    joint_has_data and
                    i < len(getattr(state, "angles", []))
                ):
                    if angle_calibrated and not suspicious_zero_angles:
                        current_item = self.joint_current_items[i]
                        current_deg = float(state.angles[i])
                        self._set_item_text_if_changed(current_item, f"{current_deg:.2f}")
                        out_of_workspace = bool(
                            i < len(getattr(state, "encoders", [])) and
                            getattr(state.encoders[i], "angle_out_of_workspace", False)
                        )
                        if out_of_workspace:
                            current_item.setForeground(QtGui.QBrush(QtGui.QColor("#d89b00")))
                            self._set_item_tooltip_if_changed(
                                current_item,
                                "Live joint angle is outside the calibrated URDF workspace.",
                            )
                        else:
                            current_item.setForeground(QtGui.QBrush(QtGui.QColor("#000000")))
                            self._set_item_tooltip_if_changed(current_item, "Live calibrated joint angle")
                    elif not angle_calibrated:
                        current_item = self.joint_current_items[i]
                        self._set_item_text_if_changed(current_item, "0.00")
                        current_item.setForeground(QtGui.QBrush(QtGui.QColor("#9aa0a6")))
                        self._set_item_tooltip_if_changed(current_item, "Joint is not calibrated")
                else:
                    current_item = self.joint_current_items[i]
                    self._set_item_text_if_changed(current_item, "-")
                    current_item.setForeground(QtGui.QBrush(QtGui.QColor("#9aa0a6")))
                    self._set_item_tooltip_if_changed(current_item, "No valid mapped-angle feedback")
                if (
                    display_valid and
                    raw_feedback_valid and
                    i < len(getattr(state, "encoders", []))
                ):
                    self._set_item_text_if_changed(
                        self.joint_encoder_items[i],
                        str(getattr(state.encoders[i], "raw_sensor", getattr(state.encoders[i], "raw", "-"))),
                    )
                else:
                    self._set_item_text_if_changed(self.joint_encoder_items[i], "-")
            if joint_has_data and sync_target and not suspicious_zero_angles:
                for i in range(min(ENCODER_COUNT, len(getattr(state, "angles", [])))):
                    if not self._joint_encoder_display_valid(i, state):
                        continue
                    if not self._joint_angle_feedback_valid(i, state):
                        continue
                    user_set = i < len(self._joint_target_user_set) and self._joint_target_user_set[i]
                    if user_set:
                        continue
                    if self._joint_angle_display_calibrated(i, state):
                        target = self._clamp_joint_target_deg(i, float(state.angles[i]))
                    else:
                        target = 0.0
                    self._set_spin_value_if_changed(self.joint_target_spins[i], target, tol=0.0005)
                    self._set_slider_value_if_changed(self.joint_sliders[i], int(round(target * 100.0)))
                    if i < len(self._joint_target_user_set):
                        self._joint_target_user_set[i] = False
                self._joint_target_initialized = True
            self._refresh_test_workspace_ranges()
        finally:
            self._updating = False
            if table is not None:
                table.setUpdatesEnabled(True)

    def _refresh_joint_abs_limit_item(self, joint_idx: int, state: Optional[HandModel]) -> None:
        """Refresh the joint abs limit item display or controls."""
        if joint_idx < 0 or joint_idx >= len(self.joint_abs_limit_items):
            return
        item = self.joint_abs_limit_items[joint_idx]
        if item is None or state is None:
            if item is not None:
                item.setText("-")
                item.setToolTip("")
            return

        valid_bitmap = int(getattr(state, "motor_abs_limit_valid_bitmap", 0) or 0)
        locked_bitmap = int(getattr(state, "motor_abs_limit_locked_bitmap", 0) or 0)
        mins = getattr(state, "motor_abs_limit_min", []) or []
        maxs = getattr(state, "motor_abs_limit_max", []) or []
        summaries: List[str] = []
        details: List[str] = []
        motors = self._motors_by_joint[joint_idx] if joint_idx < len(self._motors_by_joint) else []
        for motor_idx in motors:
            if motor_idx >= len(mins) or motor_idx >= len(maxs):
                continue
            if ((valid_bitmap >> motor_idx) & 0x01) == 0:
                continue
            lo = int(mins[motor_idx])
            hi = int(maxs[motor_idx])
            if lo > hi:
                lo, hi = hi, lo
            state_text = "LOCK" if ((locked_bitmap >> motor_idx) & 0x01) != 0 else "UNLOCK"
            summaries.append(f"M{motor_idx:02d} {state_text[0]} [{lo},{hi}]")
            details.append(f"M{motor_idx:02d}: {state_text}, 5-turn ABS range [{lo}, {hi}]")

        item.setText(" / ".join(summaries) if summaries else "-")
        item.setToolTip("\n".join(details))

    def _refresh_motor_joint_feedback_item(self, motor_idx: int, state: Optional[HandModel]) -> None:
        """Refresh the motor joint feedback item display or controls."""
        if motor_idx < 0 or motor_idx >= len(self.motor_joint_items):
            return
        item = self.motor_joint_items[motor_idx]
        if item is None or state is None:
            return
        joint_idx = MOTOR_TO_JOINT_INDEX[motor_idx] if motor_idx < len(MOTOR_TO_JOINT_INDEX) else -1
        if (
            0 <= joint_idx < len(getattr(state, "angles", []) or []) and
            self._joint_encoder_display_valid(joint_idx, state)
        ):
            item.setText(f"{float(state.angles[joint_idx]):.2f}")
            item.setToolTip("")
        else:
            item.setText("-")
            item.setToolTip("Joint encoder feedback unavailable")

    def _hide_offline_motor_feedback_items(self, motor_idx: int) -> None:
        tip = f"M{motor_idx:02d} offline: stale feedback hidden"
        for items in (
            self.motor_turn_items,
            self.motor_raw_items,
            self.motor_flash_raw_items,
            self.motor_abs_items,
            self.motor_load_items,
        ):
            if motor_idx < len(items):
                item = items[motor_idx]
                if item is not None:
                    item.setText("-")
                    item.setForeground(QtGui.QBrush(QtGui.QColor("#9aa0a6")))
                    item.setToolTip(tip)

    def _refresh_motor_feedback_view(self, sync_target: bool = False) -> None:
        state = self._state
        if state is None:
            return
        motor_has_abs = bool(getattr(state, "has_servo_angle_data", False))
        motor_has_raw = bool(getattr(state, "has_servo_raw_data", False))
        table = getattr(self, "motor_table", None)
        if table is not None:
            table.setUpdatesEnabled(False)
        self._updating = True
        try:
            unknown_mask = int(getattr(state, "multi_turn_unknown_bitmap", 0) or 0)
            overflow_mask = int(
                getattr(state, "servo_turn_overflow_bitmap", 0) or 0
            )
            overload_flags = getattr(state, "servo_overload_fault", []) or []
            overload_loads = getattr(state, "servo_overload_trigger_load", []) or []
            for i in range(MOTOR_COUNT):
                self._refresh_motor_route_item(i, state)
                motor_online = self._motor_display_valid(i, state)
                if not motor_online:
                    self._hide_offline_motor_feedback_items(i)
                    self._refresh_motor_joint_feedback_item(i, state)
                    self._motor_session_origin_abs[i] = None
                    self._motor_abs_pos[i] = None
                    self._motor_last_slider[i] = None
                    self._motor_last_sent_abs[i] = None
                    self._motor_target_user_set[i] = False
                    continue
                dev_abs = self._motor_feedback_abs(i)
                turn_unknown = ((unknown_mask & (1 << i)) != 0)
                turn_overflow = ((overflow_mask & (1 << i)) != 0)
                if i < len(self.motor_turn_items):
                    turn_item = self.motor_turn_items[i]
                    turn_item.setForeground(QtGui.QBrush(QtGui.QColor("#000000")))
                    turn_item.setToolTip("P4 logical turn count")
                    turn_item.setText(str(self._motor_turn_from_abs(dev_abs)) if dev_abs is not None and not turn_unknown else "-")
                    if turn_unknown or turn_overflow:
                        turn_item.setForeground(QtGui.QBrush(QtGui.QColor("#c62828")))
                        turn_item.setToolTip(
                            "Hardware ABS exceeds ±30715: use Motor Channels / Reset Zero."
                            if turn_overflow
                            else
                            "Turn UNKNOWN: P4 cannot trust the multi-turn position. "
                            "Use New Device Reset or restore multi-turn before control."
                        )
                raw_single = self._motor_raw_single_feedback(i)
                if i < len(self.motor_raw_items):
                    raw_item = self.motor_raw_items[i]
                    raw_item.setText(f"{raw_single:4d}" if raw_single is not None else "-")
                    raw_item.setForeground(QtGui.QBrush(QtGui.QColor("#000000")))
                    raw_item.setToolTip("Single-turn raw position, 0..4095")
                flash_raw = self._motor_flash_raw_feedback(i)
                if (
                    i < len(self.motor_flash_raw_items)
                    and self.motor_flash_raw_items[i] is not None
                ):
                    flash_item = self.motor_flash_raw_items[i]
                    flash_item.setText(
                        f"{int(flash_raw):4d}" if flash_raw is not None else "-"
                    )
                    flash_item.setForeground(
                        QtGui.QBrush(QtGui.QColor("#000000"))
                    )
                    flash_item.setToolTip(
                        "P4 最近一次成功写入 Flash 的单圈 raw"
                        if flash_raw is not None
                        else "该舵机没有有效的多圈 Flash 快照"
                    )
                if i < len(self.motor_abs_items):
                    abs_item = self.motor_abs_items[i]
                    abs_item.setText(str(int(dev_abs)) if dev_abs is not None else "-")
                    abs_item.setForeground(QtGui.QBrush(QtGui.QColor("#000000")))
                    abs_item.setToolTip("P4 logical ABS used by control")
                    if turn_unknown or turn_overflow:
                        abs_item.setForeground(QtGui.QBrush(QtGui.QColor("#c62828")))
                        abs_item.setToolTip(
                            "Hardware ABS exceeds ±30715; P4 has stopped this servo. "
                            "Use Reset Zero and recalibrate."
                            if turn_overflow
                            else
                            "Turn UNKNOWN: P4 cannot trust this logical ABS. "
                            "Do not control until multi-turn is recovered."
                        )
                    range_mask = int(
                        getattr(state, "servo_logical_range_valid_bitmap", 0)
                        or 0
                    )
                    range_mins = list(
                        getattr(state, "servo_logical_range_min", []) or []
                    )
                    range_maxs = list(
                        getattr(state, "servo_logical_range_max", []) or []
                    )
                    if (
                        (range_mask & (1 << i)) != 0
                        and i < len(range_mins)
                        and i < len(range_maxs)
                    ):
                        range_tip = (
                            f"Current-session logical range: "
                            f"[{int(range_mins[i])}, {int(range_maxs[i])}]"
                        )
                        current_tip = abs_item.toolTip()
                        abs_item.setToolTip(
                            f"{current_tip}\n{range_tip}"
                            if current_tip else range_tip
                        )
                if raw_single is not None and flash_raw is not None:
                    flash_diff = self._single_turn_diff(
                        int(raw_single), int(flash_raw)
                    )
                    if flash_diff >= MOTOR_FLASH_RAW_WARN_COUNTS:
                        tip = (
                            f"当前 raw 与 Flash 快照偏差过大："
                            f"RawNow={int(raw_single)}, FlashRaw={int(flash_raw)}, "
                            f"环形差值={flash_diff} counts（阈值 "
                            f"{MOTOR_FLASH_RAW_WARN_COUNTS}）。"
                        )
                        if i < len(self.motor_raw_items):
                            self.motor_raw_items[i].setForeground(QtGui.QBrush(QtGui.QColor("#c62828")))
                            self.motor_raw_items[i].setToolTip(tip)
                        if (
                            i < len(self.motor_flash_raw_items)
                            and self.motor_flash_raw_items[i] is not None
                        ):
                            self.motor_flash_raw_items[i].setForeground(
                                QtGui.QBrush(QtGui.QColor("#c62828"))
                            )
                            self.motor_flash_raw_items[i].setToolTip(tip)
                if i < len(self.motor_load_items) and self.motor_load_items[i] is not None:
                    item = self.motor_load_items[i]
                    load_text = "-"
                    loads = getattr(state, "servo_load", [])
                    online = getattr(state, "servo_telem_online", [])
                    overload_active = i < len(overload_flags) and bool(overload_flags[i])
                    if overload_active:
                        trigger_load = "?"
                        if i < len(overload_loads) and overload_loads[i] is not None:
                            try:
                                trigger_load = str(int(overload_loads[i]))
                            except Exception:
                                trigger_load = str(overload_loads[i])
                        load_text = f"OL {trigger_load}"
                        item.setForeground(QtGui.QBrush(QtGui.QColor("#c62828")))
                        item.setToolTip(f"M{i:02d} overload trigger load={trigger_load}")
                    else:
                        item.setForeground(QtGui.QBrush(QtGui.QColor("#000000")))
                        item.setToolTip("")
                    if i < len(loads) and i < len(online) and online[i]:
                        if not overload_active:
                            load_text = str(int(loads[i]))
                    item.setText(load_text)
                self._refresh_motor_joint_feedback_item(i, state)
            if (motor_has_raw or motor_has_abs) and sync_target:
                for i in range(MOTOR_COUNT):
                    if not self._motor_display_valid(i, state):
                        continue
                    dev_abs = self._motor_feedback_abs(i)
                    if dev_abs is None:
                        continue
                    self._motor_session_origin_abs[i] = dev_abs
                    self._motor_abs_pos[i] = 0
                    self._set_spin_value_if_changed(self.motor_target_spins[i], 0)
                    self._set_slider_value_if_changed(self.motor_sliders[i], 0)
                    self._motor_last_slider[i] = 0
                    self._motor_target_user_set[i] = False
                self._motor_target_initialized = True
                self._seed_motor_last_sent_after_sync(state)
        finally:
            self._updating = False
            if table is not None:
                table.setUpdatesEnabled(True)
        for i in range(MOTOR_COUNT):
            if not self._motor_display_valid(i, state):
                continue
            cur_single = self._motor_feedback_target(i)
            tgt_single = self._read_motor_target_slider_int(i)
            if self._single_turn_diff(cur_single, tgt_single) <= MOTOR_DIR_HINT_CLEAR_TOL:
                if self._motor_dir_hint_pending_clear[i]:
                    self._motor_dir_hint_stable_count[i] += 1
                if self._motor_dir_hint_stable_count[i] >= MOTOR_DIR_HINT_CLEAR_STABLE_FRAMES:
                    self._motor_dir_hint[i] = 0
                    self._motor_dir_hint_pending_clear[i] = False
                    self._motor_dir_hint_stable_count[i] = 0

    def _set_joint_encoder_dot(self, joint_index: int, status: Optional[object]) -> None:
        """Update the joint encoder dot value or state."""
        if joint_index < 0 or joint_index >= len(self.joint_status_items):
            return
        item = self.joint_status_items[joint_index]
        if status is True or status == "ok":
            item.setForeground(QtGui.QBrush(QtGui.QColor("#19a55a")))
            item.setToolTip("Encoder OK")
        elif status == "warn":
            item.setForeground(QtGui.QBrush(QtGui.QColor("#d89b00")))
            item.setToolTip("Encoder WARN: transient CAN/encoder bad frame, using last trusted value")
        elif status is False or status == "fault":
            item.setForeground(QtGui.QBrush(QtGui.QColor("#d84343")))
            if self._encoder_common_bus_fault:
                item.setToolTip(self._ENCODER_COMMON_BUS_MESSAGE)
            else:
                item.setToolTip("Encoder FAULT/OFFLINE: stable feedback is unavailable")
        else:
            item.setForeground(QtGui.QBrush(QtGui.QColor("#9aa0a6")))
            item.setToolTip("Encoder state unknown")

    def _clear_run_states(self) -> None:
        """Clear the run states state."""
        self._joint_run_states = [False] * ENCODER_COUNT
        self._motor_run_states = [False] * MOTOR_COUNT
        for btn in self.joint_send_buttons + self.motor_send_buttons:
            btn.setText("Send")

    def _set_row_running(self, is_motor: bool, idx: int, running: bool) -> None:
        """Update the row running value or state."""
        states = self._motor_run_states if is_motor else self._joint_run_states
        buttons = self.motor_send_buttons if is_motor else self.joint_send_buttons
        if 0 <= idx < len(states):
            states[idx] = bool(running)
        if 0 <= idx < len(buttons):
            buttons[idx].setText("Send")

    def _set_motor_target_widget(self, idx: int, value: int) -> None:
        """Update the motor target widget value or state."""
        self._updating = True
        try:
            self._set_spin_value_if_changed(self.motor_target_spins[idx], int(value))
            self._set_slider_value_if_changed(self.motor_sliders[idx], self._slider_from_abs_pos(int(value)))
        finally:
            self._updating = False

    def _set_motor_relative_target_widget(self, idx: int, relative_target: int) -> None:
        """Update the motor relative target widget value or state."""
        self._updating = True
        try:
            target = clamp_motor_abs(int(relative_target))
            self._set_spin_value_if_changed(self.motor_target_spins[idx], target)
            self._set_slider_value_if_changed(self.motor_sliders[idx], self._slider_from_abs_pos(target))
        finally:
            self._updating = False

    def _restore_motor_slider_to_relative_target(self, idx: int) -> None:
        """Handle restore motor slider to relative target behavior for the control panel."""
        if idx < 0 or idx >= len(self.motor_sliders):
            return
        current = self._motor_abs_pos[idx]
        if current is None:
            current = 0
        self._motor_last_slider[idx] = self._slider_from_abs_pos(int(current))
        self._set_motor_relative_target_widget(idx, int(current))

    def _on_joint_slider_change(self, idx: int, value: float) -> None:
        """Handle the joint slider change event."""
        self._mark_joint_target_user_set(idx)
        if not self._guard_joint_command_started():
            return
        try:
            targets = self._single_joint_command_targets(idx)
            self.context.send_joint_targets(
                targets,
                source=f"Slider J{idx}",
                live=True,
                command_joint_mask=(1 << idx),
            )
        except Exception as exc:
            self._set_status(f"Joint slider send failed: {exc}")

    def _on_motor_slider_change(self, idx: int, value: int) -> None:
        """Handle the motor slider change event."""
        state = self._state
        if state is None or not (
            bool(getattr(state, "has_servo_angle_data", False)) or bool(getattr(state, "has_servo_raw_data", False))
        ):
            self._set_status("Slider ignored: waiting for motor feedback")
            return
        if self._motor_session_origin_abs[idx] is None:
            self._motor_resync_one_channel(idx)
        if self._motor_abs_pos[idx] is None:
            self._motor_abs_pos[idx] = 0
        new_single = int(value)
        last = self._motor_last_slider[idx]
        drag_sign = 0
        if last is not None:
            diff = new_single - int(last)
            last_single = int(last)
            if (
                last_single >= MOTOR_TARGET_SCALE_HI - MOTOR_SLIDER_WRAP_MARGIN
                and new_single <= MOTOR_TARGET_SCALE_LO + MOTOR_SLIDER_WRAP_MARGIN
            ):
                drag_sign = 1
            elif (
                last_single <= MOTOR_TARGET_SCALE_LO + MOTOR_SLIDER_WRAP_MARGIN
                and new_single >= MOTOR_TARGET_SCALE_HI - MOTOR_SLIDER_WRAP_MARGIN
            ):
                drag_sign = -1
            elif abs(diff) > MOTOR_JUMP_RESYNC_THRESH:
                self._restore_motor_slider_to_relative_target(idx)
                self._set_status(
                    f"Motor slider jump blocked: M{idx:02d} slider {last_single} -> {new_single}"
                )
                return
            else:
                drag_sign = 1 if diff > 0 else (-1 if diff < 0 else 0)
        mapped = self._session_abs_from_slider_locked_turn(idx, new_single, drag_sign)
        if mapped is None:
            return
        current_target = int(self._motor_abs_pos[idx] or 0)
        if abs(int(mapped) - current_target) > MOTOR_SLIDER_MAX_TARGET_DELTA:
            self._restore_motor_slider_to_relative_target(idx)
            self._set_status(
                f"Motor slider jump blocked: M{idx:02d} {current_target} -> {int(mapped)}"
            )
            return
        if last is not None and self._single_turn_diff(new_single, int(last)) > MOTOR_JUMP_RESYNC_THRESH:
            if self._motor_slider_dragging[idx]:
                self._restore_motor_slider_to_relative_target(idx)
                return
            self._motor_resync_one_channel(idx)
            return
        self._motor_abs_pos[idx] = mapped
        self._motor_last_slider[idx] = new_single
        self._mark_motor_target_user_set(idx)
        self._set_motor_relative_target_widget(idx, mapped)
        try:
            self._send_single_motor_abs_from_ui(idx, f"Slider M{idx}")
        except Exception as exc:
            self._set_status(f"Motor slider send failed: {exc}")

    def _step_motor_target(self, idx: int, delta: int) -> None:
        """Handle step motor target behavior for the control panel."""
        if not self._interaction_enabled:
            return
        step_dir = 1 if int(delta) > 0 else (-1 if int(delta) < 0 else 0)
        if step_dir == 0:
            return
        if self._motor_session_origin_abs[idx] is None:
            self._motor_resync_one_channel(idx)
        if self._motor_abs_pos[idx] is None:
            self._motor_abs_pos[idx] = 0
        cur_sl = self._read_motor_target_slider_int(idx)
        if step_dir > 0:
            new_sl = MOTOR_TARGET_SCALE_LO if cur_sl >= MOTOR_TARGET_SCALE_HI else cur_sl + 1
        elif step_dir < 0:
            new_sl = MOTOR_TARGET_SCALE_HI if cur_sl <= MOTOR_TARGET_SCALE_LO else cur_sl - 1
        self._motor_abs_pos[idx] = clamp_motor_abs(int(self._motor_abs_pos[idx]) + step_dir)
        self._motor_dir_hint[idx] = step_dir
        self._motor_dir_hint_pending_clear[idx] = True
        self._motor_dir_hint_stable_count[idx] = 0
        self._motor_last_slider[idx] = new_sl
        self._mark_motor_target_user_set(idx)
        self._set_motor_relative_target_widget(idx, self._motor_abs_pos[idx])
        self._send_single_motor_abs_from_ui(idx, f"{'Step+' if delta > 0 else 'Step-'} M{idx}")


    def _motor_flash_raw_feedback(self, idx: int) -> Optional[int]:
        """仅返回有效 NVS 快照中的单圈 raw；无快照的通道显示为 '-'。"""
        state = self._state
        if idx < 0 or idx >= MOTOR_COUNT or state is None:
            return None
        if not self._motor_display_valid(idx, state):
            return None
        if not bool(getattr(state, "has_servo_flash_raw_data", False)):
            return None
        valid_mask = int(
            getattr(state, "multiturn_nvs_save_valid_mask", 0) or 0
        )
        if (valid_mask & (1 << idx)) == 0:
            return None
        flash_values = getattr(state, "servo_flash_raw_positions", []) or []
        if idx < len(flash_values):
            return int(flash_values[idx]) % MOTOR_SINGLE_TURN_STEPS
        return None
