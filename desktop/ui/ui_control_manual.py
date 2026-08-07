"""ui_control_manual.py - MMHand Desktop 模块契约。

职责：实现直控模式下的舵机目标编辑和发送交互。
线程：Qt GUI 主线程执行；耗时工作必须委托后台服务，禁止阻塞界面事件循环。
输入：用户操作、HandModel 只读快照以及 HandBackend/服务返回结果。
输出：Qt 控件状态、用户提示和经门面提交的应用命令。
状态所有权：仅拥有本视图控件和瞬时交互状态；设备真值由 core 层拥有。
禁止依赖：禁止直接导入串口通信和底层控制实现，禁止在 UI 中复制协议或安全规则。
"""

from desktop.core.angle_mapping import get_spec

from .ui_control_common import (
    CALIB_LOAD_SNAPSHOT_PATH,
    ControlMode,
    ENCODER_COUNT,
    FIST_POSE_URDF_MAX_OFFSET_DEG,
    List,
    MOTOR_COUNT,
    MOTOR_RELEASE_HALF_TURN_COUNTS,
    OPEN_POSE_CURRENT_MIN_OFFSET_DEG,
    Optional,
    ReplayView,
    SYSTEM_STATE_CALIBRATION_RUNNING,
    SYSTEM_STATE_IDLE,
    SYSTEM_STATE_READY,
    SYSTEM_STATE_RECOVERY_REQUIRED,
    SYSTEM_STATE_ROPE_INIT_RUNNING,
    SYSTEM_STATE_RUN_DIRECT,
    SYSTEM_STATE_RUN_FEEDBACK,
    SYSTEM_STATE_UNCONTROLLABLE,
    Tuple,
    VALID_MOTOR_MASK,
    clamp_motor_abs,
    motor_abs_after_tendon_action,
)


class ControlPanelManualMixin:
    def _manual_submode(self) -> str:
        """Handle manual submode behavior for the control panel."""
        if not hasattr(self, "tabs"):
            return "joint"
        if self.tabs is None:
            return "joint"
        widget = self.tabs.currentWidget()
        if widget is getattr(self, "device_check_view", None):
            return "device_check"
        if widget is getattr(self, "motor_page", None):
            return "motor"
        if widget is getattr(self, "test_panel", None):
            return "test"
        return "joint"

    def _on_manual_tab_changed(self) -> None:
        """Handle the manual tab changed event."""
        previous = getattr(self, "_last_control_mode", "joint")
        current = self._manual_submode()
        if previous == "test" and current != "test":
            self.stop_all_tests("stopped on mode switch", update_status=False)
        if previous != "motor" and current == "motor":
            state = self._state
            if state is not None and getattr(state, "system_state", None) == SYSTEM_STATE_READY:
                self._warn(
                    "Leaving READY",
                    "Motor direct control does not use the calibrated joint workspace check. "
                    "The lower controller will switch back to IDLE; return to Joint Channels to enter READY again.",
                )
        if current == "motor":
            self._refresh_motor_feedback_view(sync_target=True)
        elif current == "test":
            pass
        elif current == "device_check":
            if self.device_check_view is not None:
                self.device_check_view.update_state(self._state)
        else:
            self._refresh_joint_feedback_view(sync_target=True)
        self._last_control_mode = current
        self._apply_control_mode_policy()
        self._apply_active_scope_for_current_mode()
        self._update_manual_action_buttons()

    def _on_device_check_operation_finished(
        self, _name: str, _ok: bool, _message: str
    ) -> None:
        """Publish Device Check readback into the parent panel immediately."""
        try:
            state = self.backend.get_state_snapshot()
        except Exception:
            return
        if state is None:
            return
        self._state = state
        self._update_manual_action_buttons()

    def _apply_interaction_enabled(self, enabled: bool) -> None:
        """Apply the interaction enabled selection or policy."""
        self._interaction_enabled = bool(enabled)
        if getattr(self, "manual_check", None) is not None:
            self.manual_check.blockSignals(True)
            self.manual_check.setChecked(self._interaction_enabled)
            self.manual_check.blockSignals(False)
        for widget in self._control_widgets:
            try:
                widget.setEnabled(self._interaction_enabled)
            except Exception:
                pass
        if getattr(self, "joint_target_spins", None):
            for idx in range(min(ENCODER_COUNT, len(self.joint_target_spins))):
                self._refresh_joint_target_range(idx, self._state)
        self._update_manual_action_buttons()
        self._apply_control_mode_policy()
        self._set_status(f"Control Enable: {'ON' if self._interaction_enabled else 'OFF'}")
        if not self._interaction_enabled:
            self.stop_all_tests("stopped: Control Enable OFF", update_status=False)

    def _apply_control_mode_policy(self) -> None:
        """Apply the control mode policy selection or policy."""
        mode = self._manual_submode() if self._app_mode == "manual" else "joint"
        if mode == "device_check":
            self.backend.set_pid_control(False)
        elif mode == "motor":
            self.backend.set_control_mode(ControlMode.DIRECT_MOTOR)
            self.backend.set_pid_control(False)
        else:
            self.backend.set_control_mode(ControlMode.JOINT_ANGLE)
            self.backend.set_pid_control(bool(self._interaction_enabled))

    def _update_manual_action_buttons(self) -> None:
        mode = self._manual_submode() if self._app_mode == "manual" else "joint"
        interaction = bool(self._interaction_enabled)
        system_state = getattr(self._state, "system_state", None) if self._state is not None else None
        active_joints = self._active_scope_joint_indices()
        joint_workspace_ready = bool(active_joints) and not self._active_scope_joint_calib_missing_indices()
        active_motor_mask = self._active_motor_mask_from_ui() & VALID_MOTOR_MASK
        motor_online_ready = (
            active_motor_mask != 0 and
            not self._active_scope_offline_motor_indices(self._state) and
            not self._active_scope_unroutable_motor_indices(self._state)
        )
        encoder_online_ready = (
            bool(active_joints) and
            not self._active_scope_offline_encoder_indices(self._state)
        )
        if getattr(self, "manual_check", None) is not None:
            self.manual_check.setVisible(mode != "device_check")
            self.manual_check.setEnabled(mode != "device_check")
        if getattr(self, "open_btn", None) is not None:
            self.open_btn.setVisible(mode in ("joint", "test"))
            self.open_btn.setEnabled(mode in ("joint", "test") and joint_workspace_ready)
        if getattr(self, "release_half_turn_btn", None) is not None:
            self.release_half_turn_btn.setVisible(mode == "motor")
            self.release_half_turn_btn.setEnabled(interaction and mode == "motor")
        if getattr(self, "tighten_half_turn_btn", None) is not None:
            self.tighten_half_turn_btn.setVisible(mode == "motor")
            self.tighten_half_turn_btn.setEnabled(interaction and mode == "motor")
        if getattr(self, "calib_load_btn", None) is not None:
            can_load = (
                bool(self.context.is_comm_connected()) and
                CALIB_LOAD_SNAPSHOT_PATH.exists() and
                system_state not in (SYSTEM_STATE_RUN_DIRECT, SYSTEM_STATE_RUN_FEEDBACK)
            )
            self.calib_load_btn.setEnabled(can_load)
        if getattr(self, "calib_import_btn", None) is not None:
            self.calib_import_btn.setEnabled(
                system_state not in (
                    SYSTEM_STATE_RUN_DIRECT,
                    SYSTEM_STATE_RUN_FEEDBACK,
                    SYSTEM_STATE_CALIBRATION_RUNNING,
                )
            )
        if getattr(self, "calib_export_btn", None) is not None:
            self.calib_export_btn.setEnabled(CALIB_LOAD_SNAPSHOT_PATH.exists())
        if getattr(self, "new_device_reset_btn", None) is not None:
            can_reset_device, reset_tip = self._new_device_reset_action_state(
                self._state
            )
            self.new_device_reset_btn.setEnabled(can_reset_device)
            self.new_device_reset_btn.setToolTip(reset_tip)
        if getattr(self, "reset_zero_btn", None) is not None:
            state = self._state
            overflow_mask = (
                int(getattr(state, "servo_turn_overflow_bitmap", 0) or 0)
                & VALID_MOTOR_MASK
                if state is not None
                else 0
            )
            reset_running = (
                int(getattr(state, "servo_reset_zero_status", 0) or 0) == 1
                if state is not None
                else False
            )
            flash_enabled = bool(
                getattr(state, "multiturn_nvs_enabled", True)
            ) if state is not None else False
            safe_state = system_state not in (
                SYSTEM_STATE_RUN_DIRECT,
                SYSTEM_STATE_RUN_FEEDBACK,
                SYSTEM_STATE_CALIBRATION_RUNNING,
                SYSTEM_STATE_ROPE_INIT_RUNNING,
            )
            can_reset_zero = (
                mode == "motor"
                and bool(self.context.is_comm_connected())
                and overflow_mask != 0
                and flash_enabled
                and safe_state
                and not reset_running
            )
            self.reset_zero_btn.setVisible(mode == "motor")
            self.reset_zero_btn.setEnabled(can_reset_zero)
            if overflow_mask:
                self.reset_zero_btn.setToolTip(
                    "Reset overflow servos: "
                    + self._format_motor_mask(overflow_mask, limit=MOTOR_COUNT)
                )
            else:
                self.reset_zero_btn.setToolTip(
                    "No servo hardware ABS strictly exceeds ±30715"
                )
        if getattr(self, "fist_btn", None) is not None:
            self.fist_btn.setVisible(mode in ("joint", "test"))
            self.fist_btn.setEnabled(
                mode in ("joint", "test") and joint_workspace_ready
            )
        if getattr(self, "rope_init_btn", None) is not None:
            self.rope_init_btn.setEnabled(
                interaction and
                mode == "joint" and
                joint_workspace_ready and
                self._p4_rope_init_phase == "idle"
            )
        if getattr(self, "calib_btn", None) is not None:
            connected = bool(self.context.is_comm_connected())
            task_busy = system_state in (
                SYSTEM_STATE_CALIBRATION_RUNNING,
                SYSTEM_STATE_ROPE_INIT_RUNNING,
            )
            # Do not permanently grey this action from a cached Manual packet.
            # calibrate() performs a fresh P4 query and reports the exact blocker.
            self.calib_btn.setEnabled(connected and mode == "joint" and not task_busy)
            missing_manual = self._active_scope_manual_missing_indices()
            if missing_manual:
                self.calib_btn.setToolTip(
                    "Click to refresh Manual endpoints from P4 and validate: "
                    + self._format_joint_indices(missing_manual)
                )
            elif system_state not in (
                SYSTEM_STATE_UNCONTROLLABLE,
                SYSTEM_STATE_IDLE,
                SYSTEM_STATE_READY,
                SYSTEM_STATE_RECOVERY_REQUIRED,
            ):
                self.calib_btn.setToolTip(
                    "Calibration requires UNCONTROLLABLE, IDLE, READY, or RECOVERY; "
                    "click to see the current-state blocker"
                )
            else:
                self.calib_btn.setToolTip(
                    "Start automatic calibration for the selected joints; "
                    "Control Enable is not a calibration prerequisite"
                )
        if getattr(self, "enable_btn", None) is not None:
            if mode == "motor":
                enable_ready = motor_online_ready
                missing = self._active_scope_offline_motor_indices(self._state)
                unroutable = self._active_scope_unroutable_motor_indices(self._state)
                if missing:
                    tooltip = "Direct Enable blocked: offline " + self._format_motor_list(missing)
                elif unroutable:
                    tooltip = (
                        "Direct Enable blocked: no unique controllable BUS route "
                        + self._format_motor_list(unroutable)
                    )
                else:
                    tooltip = "Direct Enable requires selected servos online with unique BUS routes"
            else:
                # 负反馈 ENABLE 检查所选磁编及对应舵机在线，但不依赖 Auto/快照。
                # 动态工作空间仍在真正发送关节目标时执行最终保护。
                enable_ready = motor_online_ready and encoder_online_ready
                missing_motors = self._active_scope_offline_motor_indices(self._state)
                unroutable = self._active_scope_unroutable_motor_indices(self._state)
                missing_encoders = self._active_scope_offline_encoder_indices(self._state)
                if missing_motors:
                    tooltip = "Feedback Enable blocked: servo offline " + self._format_motor_list(missing_motors)
                elif unroutable:
                    tooltip = (
                        "Feedback Enable blocked: no unique controllable BUS route "
                        + self._format_motor_list(unroutable)
                    )
                elif missing_encoders:
                    tooltip = "Feedback Enable blocked: encoder offline " + self._format_joint_list(missing_encoders)
                else:
                    tooltip = (
                        "Feedback Enable requires selected servos and encoders online; "
                        "calibration safety is checked when a joint target is sent"
                    )
            self.enable_btn.setEnabled(mode != "device_check" and enable_ready)
            self.enable_btn.setToolTip(tooltip)
        if getattr(self, "raw_ik_enable_btn", None) is not None:
            # UI 必须显示 backend 的真实授权状态。E-STOP、Pause、断线或
            # active scope 改变后 backend 会撤权，按钮会在下一次刷新时复位。
            ros2_available = bool(self.context.ros2_enabled)
            raw_ik_enabled = bool(self.backend.raw_ik_output_enabled())
            self.raw_ik_enable_btn.blockSignals(True)
            self.raw_ik_enable_btn.setChecked(raw_ik_enabled)
            self.raw_ik_enable_btn.setText(
                "RAW IK ON" if raw_ik_enabled else "RAW IK ENABLE"
            )
            self.raw_ik_enable_btn.blockSignals(False)
            # ROS2 bridge 成功启动后才显示遥操授权按钮；普通控制和 Replay
            # 模式未接入 ROS2 时保持隐藏，避免把不可用入口暴露给用户。
            self.raw_ik_enable_btn.setVisible(ros2_available)
            self.raw_ik_enable_btn.setToolTip(
                "Authorize ROS2 raw IK/joint target output"
            )
            self.raw_ik_enable_btn.setEnabled(
                ros2_available and
                mode != "device_check" and
                (
                    raw_ik_enabled or
                    (
                        bool(self.context.is_comm_connected()) and
                        bool(self.backend.is_started()) and
                        joint_workspace_ready
                    )
                )
            )
        if getattr(self, "mid_calib_btn", None) is not None:
            self.mid_calib_btn.setEnabled(bool(self.context.is_comm_connected()))
        has_active_motor = (self._active_motor_mask_from_ui() & VALID_MOTOR_MASK) != 0
        abs_limit_mode = mode in ("joint", "test")
        if getattr(self, "abs_limit_lock_toggle_btn", None) is not None:
            self._refresh_abs_limit_lock_toggle_button()
            self.abs_limit_lock_toggle_btn.setEnabled(
                abs_limit_mode and bool(self.context.is_comm_connected()) and has_active_motor
            )
        if getattr(self, "pause_btn", None) is not None:
            self.pause_btn.setEnabled(system_state in (SYSTEM_STATE_RUN_DIRECT, SYSTEM_STATE_RUN_FEEDBACK))
        self._set_test_button_states()

    def sync_app_mode(self, mode_name: str) -> None:
        """Run the sync app mode workflow."""
        mode = str(mode_name or "").lower()
        if "teleop" in mode:
            self._build_teleop_body()
            self.backend.set_control_mode(ControlMode.JOINT_ANGLE)
            self.backend.set_pid_control(True)
        elif "replay" in mode:
            self._clear_body()
            self._app_mode = "replay"
            self._replay_view = ReplayView(self.context)
            self._body_widget = self._replay_view
            self.body_layout.addWidget(self._body_widget)
            self.backend.set_control_mode(ControlMode.JOINT_ANGLE)
            self.backend.set_pid_control(True)
        else:
            self._build_manual_body()
            if self._state is not None:
                self.update_state(self._state)

    def _joint_spin_changed(self, idx: int, value: float) -> None:
        """Compute or update joint spin changed data."""
        if self._updating:
            return
        self._mark_joint_target_user_set(idx)
        clamped = self._clamp_joint_target_deg(idx, float(value))
        self.joint_sliders[idx].blockSignals(True)
        self.joint_sliders[idx].setValue(int(round(clamped * 100.0)))
        self.joint_sliders[idx].blockSignals(False)

    def _joint_slider_changed(self, idx: int, value: int) -> None:
        """Compute or update joint slider changed data."""
        if self._updating:
            return
        self._mark_joint_target_user_set(idx)
        target = self._clamp_joint_target_deg(idx, float(value) / 100.0)
        self.joint_target_spins[idx].blockSignals(True)
        self.joint_target_spins[idx].setValue(target)
        self.joint_target_spins[idx].blockSignals(False)
        if self._interaction_enabled:
            self._on_joint_slider_change(idx, target)

    def _motor_spin_changed(self, idx: int, value: int) -> None:
        """Compute or update motor spin changed data."""
        if self._updating:
            return
        relative_target = clamp_motor_abs(int(value))
        single = self._slider_from_abs_pos(relative_target)
        self.motor_sliders[idx].blockSignals(True)
        self.motor_sliders[idx].setValue(single)
        self.motor_sliders[idx].blockSignals(False)
        if 0 <= idx < MOTOR_COUNT:
            if self._motor_session_origin_abs[idx] is None:
                self._motor_resync_one_channel(idx)
            self._motor_abs_pos[idx] = relative_target
            self._motor_last_slider[idx] = single

    def _motor_slider_changed(self, idx: int, value: int) -> None:
        """Compute or update motor slider changed data."""
        if self._updating:
            return
        if self._interaction_enabled:
            self._on_motor_slider_change(idx, int(value))
        else:
            self.motor_target_spins[idx].blockSignals(True)
            self.motor_target_spins[idx].setValue(int(value))
            self.motor_target_spins[idx].blockSignals(False)

    def resolved_joint_targets(self) -> List[float]:
        """读取全部编辑框目标，供整手姿态、Replay 预览和测试功能复用。"""
        targets = []
        for i in range(ENCODER_COUNT):
            if i < len(self._joint_target_user_set) and self._joint_target_user_set[i]:
                raw = float(self.joint_target_spins[i].value())
            else:
                raw = self._joint_feedback_target(i)
            value = self._clamp_joint_target_deg(i, raw)
            targets.append(value)
        return targets

    def _single_joint_command_targets(self, command_joint: int) -> List[float]:
        """构造单行命令：目标关节取编辑值，其余关节保持实时反馈角。

        P4 的角度协议固定发送 21 路，因此“Run J03”不能直接复用所有曾被
        编辑过的输入框，否则 J02 的历史目标会被整包重发。反馈无效的关节
        才回退到上一次目标，避免用无效角度生成突跳命令。
        """
        state = self._state
        state_targets = list(getattr(state, "target_angles", []) or []) if state is not None else []
        targets: List[float] = []
        for i in range(ENCODER_COUNT):
            if i == command_joint:
                raw = float(self.joint_target_spins[i].value())
            elif (
                state is not None and
                self._joint_angle_display_calibrated(i, state) and
                self._joint_encoder_display_valid(i, state) and
                self._joint_angle_feedback_valid(i, state) and
                i < len(getattr(state, "angles", []))
            ):
                raw = float(state.angles[i])
            elif i < len(state_targets):
                raw = float(state_targets[i])
            else:
                raw = float(self.joint_target_spins[i].value())
            targets.append(self._clamp_joint_target_deg(i, raw))
        return targets

    def resolved_motor_targets(self) -> List[int]:
        """Return the resolved motor targets values."""
        state = self._state
        return [self._motor_abs_for_command_packet(i, state) for i in range(MOTOR_COUNT)]

    def motor_sweep_targets(self) -> List[int]:
        """Run the motor sweep targets workflow."""
        return [self._read_motor_target_slider_int(i) for i in range(MOTOR_COUNT)]

    def send_joint_row(self, idx: int) -> None:
        """Send the joint row command to the backend."""
        if not self._interaction_enabled:
            return
        self._mark_joint_target_user_set(idx)
        if not self._guard_joint_command_started():
            return
        targets = self._single_joint_command_targets(idx)
        self.context.send_joint_targets(
            targets,
            source=f"Run J{idx}",
            live=False,
            command_joint_mask=(1 << idx),
        )
        self._set_row_running(False, idx, True)
        self._set_status(f"Joint command sent (J{idx})")

    def send_motor_row(self, idx: int) -> None:
        """Send the motor row command to the backend."""
        if not self._interaction_enabled:
            return
        self._mark_motor_target_user_set(idx)
        self._sync_motor_abs_from_slider_before_send(idx)
        sent_abs = self._send_single_motor_abs_from_ui(idx, f"Run M{idx}")
        if sent_abs is not None:
            self._set_row_running(True, idx, True)
            self._clear_motor_target_after_single_send(idx, sent_abs)
            if self._active_scope_multiturn_unknown_mask() == 0:
                self._set_status(f"Motor absolute command sent (M{idx})")


    def release_all_motors_half_turn(self) -> None:
        """Run the release all motors half turn workflow."""
        if not self._interaction_enabled:
            return
        if self._manual_submode() != "motor":
            self._set_status("Release Half Turn is only available in Motor Channels")
            return
        try:
            if not self.context.is_comm_connected():
                raise RuntimeError("Serial port is not connected")
            self.backend.set_control_mode(ControlMode.DIRECT_MOTOR)
            self.backend.set_pid_control(False)
            if self._estop_latched and not self.backend.is_started():
                self._set_status("Release blocked: press ENABLE after E-STOP")
                return
            if not self.backend.is_started():
                self._set_status("Release blocked: waiting for controllable IDLE after ENABLE")
                return
            unknown_mask = self._active_scope_multiturn_unknown_mask()
            if unknown_mask != 0:
                self._show_turn_recovery_guide(unknown_mask, "Release Half Turn")
                return
            state = self._state
            if state is None or not bool(getattr(state, "has_servo_angle_data", False)):
                self._set_status("Release blocked: waiting for motor ABS feedback")
                return
            unroutable = self._active_scope_unroutable_motor_indices(state)
            if unroutable:
                self._set_status(
                    "Release blocked: no unique controllable BUS route "
                    + self._format_motor_list(unroutable)
                )
                return

            active_mask = self._active_motor_mask_from_ui() & int(VALID_MOTOR_MASK) & ((1 << MOTOR_COUNT) - 1)
            if active_mask == 0:
                self._set_status("Release blocked: no active motor selected")
                return
            targets = self._current_motor_hold_targets_from_feedback()
            current_abs: List[Optional[int]] = [None] * MOTOR_COUNT
            missing: List[str] = []
            for idx in range(MOTOR_COUNT):
                if ((active_mask >> idx) & 0x01) == 0:
                    continue
                if (
                    idx >= len(getattr(state, "servo_online", [])) or
                    not state.servo_online[idx] or
                    idx >= len(getattr(state, "servo_angles", []))
                ):
                    missing.append(f"M{idx:02d}")
                    continue
                current = clamp_motor_abs(int(state.servo_angles[idx]))
                current_abs[idx] = current
                targets[idx] = motor_abs_after_tendon_action(current, idx, -MOTOR_RELEASE_HALF_TURN_COUNTS)

            if missing:
                self._set_status(f"Release blocked: waiting ABS feedback {','.join(missing[:4])}")
                return

            self.backend.set_motor_positions_absolute(targets)
            self._motor_last_sent_abs = list(targets)
            for idx in range(MOTOR_COUNT):
                if ((active_mask >> idx) & 0x01) == 0:
                    continue
                self._motor_session_origin_abs[idx] = int(targets[idx])
                self._motor_abs_pos[idx] = 0
                self._motor_last_slider[idx] = 0
                self._motor_target_user_set[idx] = False
                if idx < len(self.motor_target_spins) and idx < len(self.motor_sliders):
                    self._set_motor_target_widget(idx, 0)

            preview_items = []
            for idx in range(MOTOR_COUNT):
                if ((active_mask >> idx) & 0x01) == 0:
                    continue
                if current_abs[idx] is not None:
                    preview_items.append(f"M{idx:02d}:{current_abs[idx]}->{targets[idx]}")
                if len(preview_items) >= 4:
                    break
            active_count = bin(active_mask).count("1")
            suffix = "" if active_count <= 4 else f" ... +{active_count - 4}"
            preview = ", ".join(preview_items)
            self.context.emit_action(f"Release Half Turn sent: {preview}{suffix}")
            self._set_status(f"Released {active_count} active motor(s) by 1/2 turn ({MOTOR_RELEASE_HALF_TURN_COUNTS} counts)")
        except Exception as exc:
            self._error("Release Failed", f"{type(exc).__name__}: {exc}")

    def tighten_all_motors_half_turn(self) -> None:
        """Run the tighten all motors half turn workflow."""
        if not self._interaction_enabled:
            return
        if self._manual_submode() != "motor":
            self._set_status("Tighten Half Turn is only available in Motor Channels")
            return
        try:
            if not self.context.is_comm_connected():
                raise RuntimeError("Serial port is not connected")
            self.backend.set_control_mode(ControlMode.DIRECT_MOTOR)
            self.backend.set_pid_control(False)
            if self._estop_latched and not self.backend.is_started():
                self._set_status("Tighten blocked: press ENABLE after E-STOP")
                return
            if not self.backend.is_started():
                self._set_status("Tighten blocked: waiting for controllable IDLE after ENABLE")
                return
            unknown_mask = self._active_scope_multiturn_unknown_mask()
            if unknown_mask != 0:
                self._show_turn_recovery_guide(unknown_mask, "Tighten Half Turn")
                return
            state = self._state
            if state is None or not bool(getattr(state, "has_servo_angle_data", False)):
                self._set_status("Tighten blocked: waiting for motor ABS feedback")
                return
            unroutable = self._active_scope_unroutable_motor_indices(state)
            if unroutable:
                self._set_status(
                    "Tighten blocked: no unique controllable BUS route "
                    + self._format_motor_list(unroutable)
                )
                return

            active_mask = self._active_motor_mask_from_ui() & int(VALID_MOTOR_MASK) & ((1 << MOTOR_COUNT) - 1)
            if active_mask == 0:
                self._set_status("Tighten blocked: no active motor selected")
                return
            targets = self._current_motor_hold_targets_from_feedback()
            current_abs: List[Optional[int]] = [None] * MOTOR_COUNT
            missing: List[str] = []
            for idx in range(MOTOR_COUNT):
                if ((active_mask >> idx) & 0x01) == 0:
                    continue
                if (
                    idx >= len(getattr(state, "servo_online", [])) or
                    not state.servo_online[idx] or
                    idx >= len(getattr(state, "servo_angles", []))
                ):
                    missing.append(f"M{idx:02d}")
                    continue
                current = clamp_motor_abs(int(state.servo_angles[idx]))
                current_abs[idx] = current
                targets[idx] = motor_abs_after_tendon_action(current, idx, MOTOR_RELEASE_HALF_TURN_COUNTS)

            if missing:
                self._set_status(f"Tighten blocked: waiting ABS feedback {','.join(missing[:4])}")
                return

            self.backend.set_motor_positions_absolute(targets)
            self._motor_last_sent_abs = list(targets)
            for idx in range(MOTOR_COUNT):
                if ((active_mask >> idx) & 0x01) == 0:
                    continue
                self._motor_session_origin_abs[idx] = int(targets[idx])
                self._motor_abs_pos[idx] = 0
                self._motor_last_slider[idx] = 0
                self._motor_target_user_set[idx] = False
                if idx < len(self.motor_target_spins) and idx < len(self.motor_sliders):
                    self._set_motor_target_widget(idx, 0)

            preview_items = []
            for idx in range(MOTOR_COUNT):
                if ((active_mask >> idx) & 0x01) == 0:
                    continue
                if current_abs[idx] is not None:
                    preview_items.append(f"M{idx:02d}:{current_abs[idx]}->{targets[idx]}")
                if len(preview_items) >= 4:
                    break
            active_count = bin(active_mask).count("1")
            suffix = "" if active_count <= 4 else f" ... +{active_count - 4}"
            preview = ", ".join(preview_items)
            self.context.emit_action(f"Tighten Half Turn sent: {preview}{suffix}")
            self._set_status(f"Tightened {active_count} active motor(s) by 1/2 turn ({MOTOR_RELEASE_HALF_TURN_COUNTS} counts)")
        except Exception as exc:
            self._error("Tighten Failed", f"{type(exc).__name__}: {exc}")

    def apply_preset(self, angles: List[float], joint_indices: Optional[List[int]] = None) -> None:
        """Apply the preset command to the UI or backend."""
        clamped_angles = list(angles[:ENCODER_COUNT])
        if len(clamped_angles) < ENCODER_COUNT:
            clamped_angles.extend([0.0] * (ENCODER_COUNT - len(clamped_angles)))
        indices = range(ENCODER_COUNT) if joint_indices is None else joint_indices
        self._updating = True
        try:
            for i in indices:
                if i < 0 or i >= ENCODER_COUNT:
                    continue
                value = clamped_angles[i]
                value = self._clamp_joint_target_deg(i, float(value))
                clamped_angles[i] = value
                self.joint_target_spins[i].setValue(value)
                self.joint_sliders[i].setValue(int(round(value * 100.0)))
                self._joint_target_user_set[i] = True
        finally:
            self._updating = False
        self.context.draw_pose_angles(clamped_angles)

    def _build_current_min_open_pose_targets(self) -> Optional[Tuple[List[float], List[int]]]:
        joints = self._active_scope_joint_indices()
        if not joints:
            self._set_status("Open Pose blocked: no active joints selected")
            return None
        targets = self.resolved_joint_targets()
        missing: List[str] = []
        for joint in joints:
            workspace = self._joint_workspace_range(joint, self._state)
            if not workspace.ready:
                missing.append(f"J{joint:02d} {workspace.reason}")
                continue
            targets[joint] = workspace.open_pose_target(OPEN_POSE_CURRENT_MIN_OFFSET_DEG)
        if missing:
            detail = "; ".join(missing[:4])
            if len(missing) > 4:
                detail += f"; +{len(missing) - 4} more"
            self._set_status(f"Open Pose blocked: {detail}")
            return None
        return targets, joints

    def _joint_uses_dual_motors(self, joint: int) -> bool:
        """Return whether a joint is driven by two motor channels."""
        try:
            return len(get_spec(joint).motor_indices) >= 2
        except (KeyError, TypeError, ValueError):
            return False

    def _build_open_pose_targets(self) -> Optional[Tuple[List[float], List[int]]]:
        """Build open pose targets from each joint's active workspace."""
        joints = self._active_scope_joint_indices()
        if not joints:
            self._set_status("Open Pose blocked: no active joints selected")
            return None
        targets = self.resolved_joint_targets()
        missing: List[str] = []
        for joint in joints:
            workspace = self._joint_workspace_range(joint, self._state)
            if not workspace.ready or workspace.current_min_deg is None:
                missing.append(f"J{joint:02d} {workspace.reason}")
                continue
            if self._joint_uses_dual_motors(joint):
                # 双腱关节在 Current Min 附近容易与预紧/防松绳约束打架，
                # Open Pose 改用动态工作空间中点，给两根腱都留下调节余量。
                targets[joint] = workspace.clamp(
                    (workspace.current_min_deg + workspace.current_max_deg) * 0.5
                )
            else:
                targets[joint] = workspace.open_pose_target(OPEN_POSE_CURRENT_MIN_OFFSET_DEG)
        if missing:
            detail = "; ".join(missing[:4])
            if len(missing) > 4:
                detail += f"; +{len(missing) - 4} more"
            self._set_status(f"Open Pose blocked: {detail}")
            return None
        return targets, joints

    def apply_open_pose(self) -> None:
        """Apply the open pose command to the UI or backend."""
        pose = self._build_open_pose_targets()
        if pose is None:
            return
        targets, joints = pose
        self._apply_and_send_joint_pose(
            targets,
            joints,
            "Open Pose",
            "dual Current Min/Max midpoint; single Current Min + "
            f"{OPEN_POSE_CURRENT_MIN_OFFSET_DEG:.0f} deg",
        )


    def apply_fist_pose(self) -> None:
        """Apply the fist pose command to the UI or backend."""
        joints = self._active_scope_joint_indices()
        if not joints:
            self._set_status("Fist Pose blocked: no active joints selected")
            return
        angles = self.resolved_joint_targets()
        for i in joints:
            max_deg = self._joint_target_max_deg(i)
            target = max(
                self._joint_target_min_deg(i),
                max_deg - FIST_POSE_URDF_MAX_OFFSET_DEG,
            )
            angles[i] = self._clamp_joint_target_deg(i, target)
        self._apply_and_send_joint_pose(
            angles,
            joints,
            "Fist Pose",
            f"URDF Max - {FIST_POSE_URDF_MAX_OFFSET_DEG:.0f} deg",
        )

    def _apply_and_send_joint_pose(
        self,
        targets: List[float],
        joints: List[int],
        source: str,
        detail: str,
    ) -> None:
        """Reflect a preset in the controls and submit the same target to P4."""
        self.apply_preset(targets, joint_indices=joints)
        if not self._guard_joint_command_started():
            self._set_status(f"{source} blocked: feedback controller is not ready")
            return
        try:
            result = self.context.send_joint_targets(targets, source=source, live=False)
        except Exception as exc:
            self._set_status(f"{source} send failed: {type(exc).__name__}: {exc}")
            return
        if not bool(getattr(result, "sent", result)):
            message = str(getattr(result, "message", "") or "P4 rejected the joint target")
            self._set_status(f"{source} blocked: {message}")
            return
        self._set_status(f"{source} sent: {detail}")
