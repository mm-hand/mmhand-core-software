"""ControlCommandController methods for the modular control panel."""

from .ui_control_common import (
    ControlMode,
    ENCODER_COUNT,
    SYSTEM_FAULT_MULTITURN_UNKNOWN,
    SYSTEM_STATE_PAUSED,
    VALID_MOTOR_MASK,
)
from .ui_dialogs import show_scrollable_message


class ControlCommandController:
    def start_controller(self) -> None:
        """Start the controller workflow."""
        if self._app_mode == "manual" and self._manual_submode() == "device_check":
            self._set_status("ENABLE blocked: switch to Joint or Motor Channels after Device Check")
            return
        if not self.context.is_comm_connected():
            if not self.context.connect_selected_port():
                self._update_connection_lights()
                return
            self._update_connection_lights()
        if self._manual_submode() != "motor":
            missing_workspace = self._active_scope_joint_calib_missing_indices()
            if missing_workspace:
                self._set_status(
                    "ENABLE blocked: active workspace is not ready ("
                    + self._format_joint_indices(missing_workspace)
                    + ")"
                )
                return
        if not self._apply_active_scope_for_current_mode():
            self._set_status("Enable blocked: active scope was not accepted by lower controller")
            return
        state = self._state
        if state is not None and getattr(state, "system_state", None) == SYSTEM_STATE_PAUSED:
            self._estop_latched = False
            self.backend.start()
            self.context.emit_action("Resume")
            self._set_status("Resume requested: continuing paused target")
            return
        unknown_mask = self._active_scope_multiturn_unknown_mask()
        if unknown_mask != 0:
            self._show_turn_recovery_guide(unknown_mask, "ENABLE")
            return

        if self._manual_submode() != "motor":
            # 关节负反馈/遥操启动时不能先发舵机 ABS 直控命令。
            # ABS 命令会让 P4 立即进入 RUN_DIRECT，紧接着的负反馈
            # START/RUN 会因模式门禁被拒绝，表现为 Send 成功但舵机不转。
            self.backend.set_control_mode(ControlMode.JOINT_ANGLE)
            self.backend.set_pid_control(True)
            self._estop_latched = False
            self.backend.start()
            self.context.emit_action("Start joint feedback controller")
            self._set_status("Enable requested: waiting for joint READY")
            return

        if not self._sync_motor_targets_to_feedback():
            self._set_status("Enable waits for fresh motor feedback")
            return
        targets = self._build_motor_abs_command_list()
        self.backend.set_control_mode(ControlMode.DIRECT_MOTOR)
        self.backend.set_pid_control(False)
        self.backend.set_motor_positions_absolute(targets)
        self._motor_last_sent_abs = list(targets)
        self._estop_latched = False
        # 绝对位置直控命令自带 RUN_DIRECT 请求，不再叠加 START 事件。
        self.context.emit_action("Start direct motor hold from current feedback")
        self._set_status("Direct motor hold requested")

    def toggle_raw_ik_output(self, enabled: bool) -> None:
        """只切换遥操授权，不改变普通负反馈 Enable 或当前控制目标。"""
        ok, message = self.backend.set_raw_ik_output_enabled(bool(enabled))
        actual = bool(self.backend.raw_ik_output_enabled())
        self.raw_ik_enable_btn.blockSignals(True)
        self.raw_ik_enable_btn.setChecked(actual)
        self.raw_ik_enable_btn.setText("RAW IK ON" if actual else "RAW IK ENABLE")
        self.raw_ik_enable_btn.blockSignals(False)
        self.context.emit_action(
            "RAW IK output enabled" if actual else "RAW IK output disabled"
        )
        self._set_status(message if ok else f"RAW IK enable blocked: {message}")

    def stop_controller(self) -> None:
        """Stop the controller workflow."""
        if self._estop_in_progress:
            return
        self._estop_in_progress = True
        try:
            self._update_connection_lights()
            self._estop_latched = True
            # STOP is safety-critical. Enqueue it before recorder/HDF5 cleanup
            # or any other UI-side cancellation work.
            try:
                self.backend.stop()
            except Exception:
                pass
            self.context.cancel_replay("E-STOP")
            self.stop_all_tests("stopped by E-STOP", update_status=False)
            self._clear_run_states()
            self.context.emit_action("Stop")
        finally:
            self._estop_in_progress = False

    def pause_controller(self) -> None:
        """Run the pause controller workflow."""
        self.context.cancel_replay("Pause")
        try:
            self.backend.pause()
        except Exception:
            pass
        self.context.emit_action("Pause")
        self._set_status("Pause requested: target cache kept for resume")

    def safe_close_controller(self) -> None:
        """Run the safe close controller workflow."""
        self.stop_all_tests("safe close", update_status=False)
        try:
            self.backend.hold_current_motor_positions(delay_s=0.25)
        except Exception:
            pass
        self.context.disconnect_current_port("Safe closed: holding current motor targets, then STOP", stop_lower=True)

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
        self._estop_latched = True
        self._reset_p4_rope_init_progress()
        self.backend.reset()
        self._clear_run_states()
        self.context.emit_action("Clear Fault")

    def feedback_command_allowed(self, source: str = "", show_status: bool = True) -> bool:
        """Return whether feedback command allowed passes."""
        if self._app_mode not in ("manual", "teleop"):
            return True
        active_mask = self._active_motor_mask_from_ui() & VALID_MOTOR_MASK
        if active_mask == 0:
            if show_status:
                self._set_status("Feedback blocked: no active motor selected")
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
                    f"({self._format_joint_indices(missing_joints)}); press Calib Load or recalibrate"
                )
            return False
        mismatch = self._active_scope_status_mismatch()
        if mismatch:
            if show_status:
                self._set_status(f"Feedback blocked: active scope mismatch ({mismatch})")
            return False
        return True

    def set_abs_limit_lock(self, locked: bool) -> None:
        """Run the set abs limit lock workflow."""
        if self._app_mode == "manual" and self._manual_submode() not in ("joint", "test"):
            self._set_status("ABS limit lock is only available in Joint/Test Channels")
            return
        if not self.context.is_comm_connected():
            self._set_status("ABS limit lock blocked: serial port is not connected")
            return
        mask = self._active_scope_abs_limit_valid_mask()
        if mask == 0:
            self._set_status("ABS limit lock blocked: no calibrated ABS limit in current control set")
            return
        try:
            ok = self.backend.set_motor_abs_limit_lock(mask, bool(locked))
        except Exception as exc:
            self._set_status(f"ABS limit lock failed: {exc}")
            return
        action = "locked" if locked else "unlocked"
        self.context.emit_action(f"ABS limits {action}: mask=0x{mask:08X}")
        self._set_status(f"ABS limits {action}" if ok else f"ABS limits {action} ACK timeout")
        if ok and self._state is not None:
            if locked:
                self._abs_limit_warning_unlocked_mask &= ~mask
            else:
                self._abs_limit_warning_unlocked_mask |= mask
            self._feedback_abs_limit_popup_episode_active = False
            current = int(getattr(self._state, "motor_abs_limit_locked_bitmap", 0) or 0)
            self._state.motor_abs_limit_locked_bitmap = (current | mask) if locked else (current & ~mask)
            self._refresh_abs_limit_lock_toggle_button()
            for idx in range(ENCODER_COUNT):
                self._refresh_joint_abs_limit_item(idx, self._state)

    def toggle_multiturn_nvs(self) -> None:
        """Toggle the multiturn nvs setting."""
        if not self.context.is_comm_connected():
            self._set_status("Flash control blocked: serial port is not connected")
            return
        state = self._state or self.backend.get_current_state()
        current = bool(getattr(state, "multiturn_nvs_enabled", True))
        target = not current
        try:
            ok = self.backend.set_multiturn_nvs_enabled(target)
        except Exception as exc:
            self._set_status(f"Flash control failed: {exc}")
            return
        if ok:
            if target:
                self._set_status("Flash multi-turn save enabled")
            else:
                self._set_status("Flash multi-turn save disabled; lower P4 snapshots cleared")
        else:
            self._set_status("Flash control ACK timeout")

    def _refresh_nvs_control_button(self) -> None:
        """Refresh the nvs control button display or controls."""
        btn = getattr(self, "nvs_control_btn", None)
        if btn is None:
            return
        state = self._state
        enabled = bool(getattr(state, "multiturn_nvs_enabled", True)) if state is not None else True
        snapshot = bool(getattr(state, "multiturn_nvs_snapshot_valid", False)) if state is not None else False
        btn.setText("Flash ON" if enabled else "Flash OFF")
        if enabled:
            btn.setStyleSheet("font-weight:700;background:#e8f5e9;color:#1b5e20;")
            btn.setToolTip(
                "Flash multi-turn save/boot restore enabled; "
                + ("snapshot valid" if snapshot else "no snapshot yet")
            )
        else:
            btn.setStyleSheet("font-weight:700;background:#ffebee;color:#c62828;")
            btn.setToolTip("P4 Flash save disabled; lower snapshots cleared; click to enable future saves")
