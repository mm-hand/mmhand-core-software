"""ui_control_commands.py - MMHand Desktop 模块契约。

职责：实现控制面板的 Enable、Stop、E-STOP 和目标发送命令。
线程：Qt GUI 主线程执行；耗时工作必须委托后台服务，禁止阻塞界面事件循环。
输入：用户操作、HandModel 只读快照以及 HandBackend/服务返回结果。
输出：Qt 控件状态、用户提示和经门面提交的应用命令。
状态所有权：仅拥有本视图控件和瞬时交互状态；设备真值由 core 层拥有。
禁止依赖：禁止直接导入串口通信和底层控制实现，禁止在 UI 中复制协议或安全规则。
"""

from .ui_control_common import (
    ControlMode,
    ENCODER_COUNT,
    SYSTEM_FAULT_MULTITURN_UNKNOWN,
    SYSTEM_STATE_PAUSED,
    SYSTEM_STATE_UNCONTROLLABLE,
    VALID_MOTOR_MASK,
)
from .ui_dialogs import show_scrollable_message


class ControlCommandController:
    def start_controller(self) -> None:
        """Start the controller workflow."""
        if bool(getattr(self, "_startup_device_check_running", False)):
            self._set_status("ENABLE blocked: restoring P4 calibration state")
            return
        if self._app_mode == "manual" and self._manual_submode() == "device_check":
            self._set_status("ENABLE blocked: switch to Joint or Motor Channels after Device Check")
            return
        if not self.context.is_comm_connected():
            if not self.context.connect_selected_port():
                return
        mode = self._manual_submode()
        state = self._state
        scan_state = int(getattr(state, "servo_route_scan_state", 0) or 0) if state is not None else 0
        if scan_state == 1:
            self._set_status("ENABLE blocked: wait for P4 servo route scan to finish")
            return
        missing_motors = self._active_scope_offline_motor_indices(state)
        if missing_motors:
            self._set_status(
                "ENABLE blocked: selected servo offline ("
                + self._format_motor_list(missing_motors)
                + ")"
            )
            return
        unroutable_motors = self._active_scope_unroutable_motor_indices(state)
        if unroutable_motors:
            scan_state = int(getattr(state, "servo_route_scan_state", 0) or 0) if state is not None else 0
            reason = "P4 route scan is running" if scan_state else "duplicate ID, BUS capacity, or missing route"
            self._set_status(
                "ENABLE blocked: selected servo has no unique controllable BUS route ("
                + self._format_motor_list(unroutable_motors)
                + f"); {reason}"
            )
            return
        if mode != "motor":
            missing_encoders = self._active_scope_offline_encoder_indices(state)
            if missing_encoders:
                self._set_status(
                    "ENABLE blocked: selected encoder offline ("
                    + self._format_joint_list(missing_encoders)
                    + ")"
                )
                return
        # 两种控制模式都依赖可信的舵机多圈坐标。未知时保持禁用；
        # 上位机先尝试 JSON 最近圈恢复，仍失败则引导用户勾选关节重新标定。
        unknown_mask = self._active_scope_multiturn_unknown_mask()
        if unknown_mask != 0:
            self._show_turn_recovery_guide(unknown_mask, "ENABLE")
            return
        # ENABLE 是用户解除 E-STOP 的明确动作。上位机重启会丢失本地
        # _estop_latched，但 P4 仍可能保留 UNCONTROLLABLE/E-STOP；因此
        # 必须同时信任下位机状态，并在重发 active scope 前完成 RESET。
        system_state = (
            getattr(state, "system_state", None) if state is not None else None
        )
        resume_allowed = (bool(getattr(self, "_pause_resume_allowed", False))
                          and getattr(self, "_pause_resume_scope", None)
                          == (mode, self._active_motor_mask_from_ui()))
        if (self._estop_latched or system_state == SYSTEM_STATE_UNCONTROLLABLE
                or (system_state == SYSTEM_STATE_PAUSED and not resume_allowed)):
            self.backend.reset()
            self._estop_latched = False
        if not self._apply_active_scope_for_current_mode():
            self._set_status("Enable blocked: active scope was not accepted by lower controller")
            return
        if system_state == SYSTEM_STATE_PAUSED and resume_allowed:
            self._estop_latched = False
            self.backend.start()
            self.context.emit_action("Resume")
            self._set_status("Resume requested: continuing paused target")
            return

        if mode != "motor":
            # 关节负反馈/遥操启动时不能先发舵机 ABS 直控命令。
            # ABS 命令会让 P4 立即进入 RUN_DIRECT，紧接着的负反馈
            # START/RUN 会因模式门禁被拒绝，表现为 Send 成功但舵机不转。
            self.backend.set_control_mode(ControlMode.JOINT_ANGLE)
            self._estop_latched = False
            self.backend.start()
            self.context.emit_action("Start joint feedback controller")
            self._set_status("Enable requested: waiting for joint READY")
            return

        ok, message, targets = self.backend.enable_direct_hold(self._active_motor_mask_from_ui())
        if not ok:
            self._set_status(f"Enable blocked: {message}")
            return
        for i, target in enumerate(targets):
            if self._active_motor_mask_from_ui() & (1 << i):
                self._clear_motor_target_after_single_send(i, target)
        self._motor_last_sent_abs = list(targets)
        self._estop_latched = False
        self.context.emit_action("Start direct motor hold from current feedback")
        self._set_status(message)

    def toggle_ros2_output(self, enabled: bool) -> None:
        """只切换外部 ROS2 输出授权，不改变普通负反馈或当前控制目标。"""
        ok, message = self.backend.set_ros2_output_enabled(bool(enabled))
        actual = bool(self.backend.ros2_output_enabled())
        self.ros2_output_btn.blockSignals(True)
        self.ros2_output_btn.setChecked(actual)
        self.ros2_output_btn.setText(
            "ROS2 INPUT ON" if actual else "ROS2 INPUT"
        )
        self.ros2_output_btn.blockSignals(False)
        if not actual:
            self.clear_ros2_joint_targets()
        self.context.emit_action(
            "ROS2 input enabled" if actual else "ROS2 input disabled"
        )
        self._set_status(message if ok else f"ROS2 input enable blocked: {message}")

    def stop_controller(self) -> None:
        """Stop the controller workflow."""
        if self._estop_in_progress:
            return
        self._estop_in_progress = True
        try:
            self._pause_resume_allowed = False
            self._estop_latched = True
            # STOP is safety-critical. Enqueue it before CSV recorder cleanup
            # or any other UI-side cancellation work.
            try:
                self.backend.stop()
            except Exception:
                pass
            self.context.cancel_replay("E-STOP")
            self.stop_all_tests("stopped by E-STOP", update_status=False)
            self._clear_run_states()
            self.clear_ros2_joint_targets()
            self.context.emit_action("Stop")
        finally:
            self._estop_in_progress = False

    def pause_controller(self) -> None:
        """Run the pause controller workflow."""
        self.context.cancel_replay("Pause")
        try:
            self.backend.pause()
            self._pause_resume_allowed = True
            self._pause_resume_scope = (self._manual_submode(), self._active_motor_mask_from_ui())
        except Exception:
            pass
        self.clear_ros2_joint_targets()
        self.context.emit_action("Pause")
        self._set_status("Pause requested: target cache kept for resume")

    def clear_fault_with_confirm(self) -> None:
        """Clear the fault with confirm state."""
        state = self._state
        if state is None:
            try:
                state = self.context.get_state()
            except Exception:
                state = None

        summary = None
        root = getattr(self.context, "root", None)
        builder = getattr(root, "_build_fault_summary", None)
        if callable(builder) and state is not None:
            try:
                summary = builder(state)
            except Exception:
                summary = None

        if summary and summary.get("active"):
            body_builder = getattr(root, "_fault_body_text", None)
            if callable(body_builder):
                detail = body_builder(summary)
            else:
                detail = "\n".join(summary.get("lines") or [])
            try:
                unknown_mask = int(getattr(state, "multi_turn_unknown_bitmap", 0) or 0) & VALID_MOTOR_MASK
            except Exception:
                unknown_mask = 0
            try:
                fault_bitmap = int(summary.get("bitmap", 0) or 0)
            except Exception:
                fault_bitmap = 0
            if unknown_mask != 0 or (fault_bitmap & SYSTEM_FAULT_MULTITURN_UNKNOWN) != 0:
                guide_mask = unknown_mask or self._active_scope_multiturn_unknown_mask() or VALID_MOTOR_MASK
                detail = (
                    f"{detail}\n\n"
                    "Turn UNKNOWN recovery guide:\n"
                    f"{self._turn_recovery_guide_text(guide_mask, 'Clear Fault')}"
                )
            text = (
                "当前仍检测到故障：\n\n"
                f"{detail}\n\n"
                "请先确认故障原因已经排除。\n"
                "清除故障会让下位机回到不可控状态，之后需要重新按 ENABLE。"
            )
            title = "Clear Fault"
            accept_text = "清除故障"
        else:
            text = (
                "确认清除当前故障/保护状态？\n\n"
                "这会发送 reset 指令，让下位机回到不可控状态，之后需要重新按 ENABLE。"
            )
            title = "Clear Fault"
            accept_text = "清除"
        confirmed = show_scrollable_message(
            self,
            title,
            text,
            severity=(
                "warning"
                if summary and summary.get("active")
                else "question"
            ),
            accept_text=accept_text,
            reject_text="取消",
        )
        if not confirmed:
            self._set_status("Clear Fault cancelled")
            return

        self.reset_controller()
        notifier = getattr(root, "_on_clear_fault_confirmed", None)
        if callable(notifier):
            notifier()

    def reset_controller(self) -> None:
        """Reset the controller state."""
        # RESET/Clear Fault 会清除 P4 的锁存 E-STOP；控制仍保持关闭，
        # 用户下一次按 ENABLE 时再按当前模式重新取得控制权。
        self._estop_latched = False
        self.backend.reset()
        self._clear_run_states()
        self.context.emit_action("Clear Fault")

    def feedback_command_allowed(self, source: str = "", show_status: bool = True) -> bool:
        """Return whether feedback command allowed passes."""
        if self._app_mode != "manual":
            return True
        if bool(getattr(self, "_encoder_common_bus_fault", False)):
            if show_status:
                self._set_status(
                    "Feedback blocked: 21路磁编Raw全零，疑似图三末端至S3的公共SPI FPC短路"
                )
            return False
        active_mask = self._active_motor_mask_from_ui() & VALID_MOTOR_MASK
        if active_mask == 0:
            if show_status:
                self._set_status("Feedback blocked: no active motor selected")
            return False
        unroutable = self._active_scope_unroutable_motor_indices(self._state)
        if unroutable:
            if show_status:
                self._set_status(
                    "Feedback blocked: no unique controllable BUS route ("
                    + self._format_motor_list(unroutable)
                    + ")"
                )
            return False
        unknown_mask = self._active_scope_multiturn_unknown_mask()
        if unknown_mask != 0:
            if show_status:
                self._show_turn_recovery_guide(unknown_mask, str(source or "Feedback"))
            return False
        missing_mask = self._active_scope_abs_limit_missing_mask()
        if missing_mask != 0:
            if show_status:
                label = self._active_scope_label_from_ui() or str(source or "Feedback")
                self._set_status(
                    f"{label} blocked: calibrate active motors first ({self._format_motor_mask(missing_mask)})"
                )
            return False
        missing_joints = self._active_scope_joint_calib_missing_indices()
        if missing_joints:
            if show_status:
                self._set_status(
                    "Feedback blocked: joint calibration not loaded "
                    f"({self._format_joint_indices(missing_joints)}); load a calibration file or recalibrate"
                )
            return False
        mismatch = self._active_scope_status_mismatch()
        if mismatch:
            if show_status:
                self._set_status(f"Feedback blocked: active scope mismatch ({mismatch})")
            return False
        return True

