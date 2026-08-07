"""ui_control_calibration.py - MMHand Desktop 模块契约。

职责：实现控制面板中的标定、Rope Init 和恢复交互。
线程：Qt GUI 主线程执行；耗时工作必须委托后台服务，禁止阻塞界面事件循环。
输入：用户操作、HandModel 只读快照以及 HandBackend/服务返回结果。
输出：Qt 控件状态、用户提示和经门面提交的应用命令。
状态所有权：仅拥有本视图控件和瞬时交互状态；设备真值由 core 层拥有。
禁止依赖：禁止直接导入串口通信和底层控制实现，禁止在 UI 中复制协议或安全规则。
"""

from .ui_control_common import (
    AUTO_CALIBRATION_JOINT_INDICES,
    CALIB_LOAD_SNAPSHOT_PATH,
    CALIB_WARN_AUTO_MAX_SHORTFALL,
    CALIB_WARN_AUTO_MIN_OFFSET,
    CALIB_WARN_URDF_MANUAL_SPAN_MISMATCH,
    CALIB_DIAG_REASON_JOINT_STEP_FAILED,
    CALIB_DIAG_PHASE_SEQUENCE,
    CalibrationSnapshotError,
    ControlMode,
    ControlScopeModel,
    DUAL_TENDON_JOINT_INDICES,
    ENCODER_COUNT,
    FINGER_JOINT_RANGES,
    HAND_PROFILE,
    List,
    MOTOR_COUNT,
    MOTOR_MID_CALIB_PATH,
    MOTOR_TO_JOINT_INDEX,
    Optional,
    PROJECT_ROOT,
    Path,
    QtWidgets,
    ROPE_INIT_REASON_LABELS,
    ROPE_INIT_START_TIMEOUT_S,
    ROPE_INIT_STATUS_FAILED,
    ROPE_INIT_STATUS_RUNNING,
    ROPE_INIT_STATUS_SUCCESS,
    SYSTEM_STATE_CALIBRATION_RUNNING,
    SYSTEM_STATE_IDLE,
    SYSTEM_STATE_READY,
    SYSTEM_STATE_RECOVERY_REQUIRED,
    SYSTEM_STATE_ROPE_INIT_RUNNING,
    SYSTEM_STATE_RUN_DIRECT,
    SYSTEM_STATE_RUN_FEEDBACK,
    SYSTEM_STATE_UNCONTROLLABLE,
    TENDON_LUT_MAX_SAMPLES,
    Tuple,
    VALID_MOTOR_MASK,
    _motor_channel_valid,
    atomic_write_snapshot,
    directed_encoder_delta,
    json,
    manual_encoder_reference_entry_crc,
    merge_snapshot,
    read_snapshot,
    shutil,
    tendon_lut_field,
    threading,
    time,
    urdf_manual_scaled_mapping_metadata,
    validate_snapshot,
)


class CalibrationWorkflowController:
    def update_rope_init_progress(self, state) -> None:
        """Update Rope Init progress from the latest P4 state packet."""
        self._update_p4_rope_init_progress(state)

    def show_calibration_assembly_warning_if_needed(self, state) -> None:
        """Show the calibration assembly warning popup once per warning episode."""
        self._maybe_show_calibration_assembly_warning(state)

    def _active_scope_joint_indices(self) -> List[int]:
        """Return the active scope joint indices state."""
        if self._app_mode == "manual":
            joint_mask = int(self._selected_joint_mask) & ((1 << ENCODER_COUNT) - 1)
            return ControlScopeModel.ordered_joint_indices_from_mask(joint_mask)
        data = self._current_active_scope_tuple()
        if data is not None:
            _motor_mask, joint_mask, _require_can = data
            return ControlScopeModel.ordered_joint_indices_from_mask(int(joint_mask))
        return list(AUTO_CALIBRATION_JOINT_INDICES)

    def rope_init_controller(self) -> None:
        """Run the rope init controller workflow."""
        if self._p4_rope_init_phase != "idle":
            self._set_status("Rope Init is already queued or running on P4")
            return
        if self._manual_submode() != "joint":
            self._set_status("Rope Init is only available in Joint Channels")
            return
        if not self.context.is_comm_connected():
            if not self.context.connect_selected_port():
                return
        if not self.backend.is_started():
            self._set_status("Rope Init blocked: press ENABLE to enter IDLE first")
            return
        state = self._state
        system_state = getattr(state, "system_state", None)
        if system_state not in (
            SYSTEM_STATE_IDLE,
            SYSTEM_STATE_READY,
            SYSTEM_STATE_RECOVERY_REQUIRED,
        ):
            self._set_status("Rope Init blocked: wait for IDLE/READY/RECOVERY")
            return
        unknown_mask = self._active_scope_multiturn_unknown_mask()
        if unknown_mask != 0:
            self._show_turn_recovery_guide(unknown_mask, "Rope Init")
            return

        if not self._apply_active_scope_for_current_mode():
            self._set_status("Rope Init blocked: active scope was not accepted")
            return
        active_joint_mask = int(self._active_joint_mask_from_ui()) & (
            (1 << ENCODER_COUNT) - 1
        )
        if active_joint_mask == 0:
            self._set_status("Rope Init blocked: no active joint selected")
            return
        transition_base = int(
            getattr(state, "state_transition_count", 0) or 0
        )
        sequence_base = int(getattr(state, "rope_init_sequence", 0) or 0)
        if not self.backend.rope_init(active_joint_mask):
            self._set_status(
                "Rope Init rejected: check Manual/v3 calibration, state, and Turn"
            )
            return
        self._p4_rope_init_phase = "queued"
        self._p4_rope_init_joint_mask = active_joint_mask
        self._p4_rope_init_requested_at = time.monotonic()
        self._p4_rope_init_transition_base = transition_base
        self._p4_rope_init_sequence_base = sequence_base
        self.rope_init_btn.setText("Rope Init...")
        self.context.emit_action(
            f"Rope Init queued on P4: joint_mask=0x{active_joint_mask:08X}"
        )
        self._set_status(
            f"Rope Init queued on P4: joint_mask=0x{active_joint_mask:08X}"
        )
        self._update_manual_action_buttons()

    def _reset_p4_rope_init_progress(self) -> None:
        """Reset the p4 rope init progress state."""
        self._p4_rope_init_phase = "idle"
        self._p4_rope_init_joint_mask = 0
        self._p4_rope_init_requested_at = 0.0
        self._p4_rope_init_transition_base = 0
        self._p4_rope_init_sequence_base = 0
        button = getattr(self, "rope_init_btn", None)
        if button is not None:
            button.setText("Rope Init")

    def _finish_p4_rope_init_progress(self, success: bool, detail: str) -> None:
        """Handle finish p4 rope init progress behavior for the control panel."""
        joint_mask = self._p4_rope_init_joint_mask
        result = "complete" if success else "failed"
        message = (
            f"Rope Init {result} on P4: joint_mask=0x{joint_mask:08X}; {detail}"
        )
        self._reset_p4_rope_init_progress()
        self.context.emit_action(message)
        self._update_manual_action_buttons()

    def _update_p4_rope_init_progress(self, state) -> None:
        """Handle update p4 rope init progress behavior for the control panel."""
        if self._p4_rope_init_phase == "idle" or state is None:
            return
        system_state = int(getattr(state, "system_state", -1) or 0)
        transition_count = int(getattr(state, "state_transition_count", 0) or 0)
        transition_delta = max(
            0, transition_count - self._p4_rope_init_transition_base
        )
        elapsed = max(0.0, time.monotonic() - self._p4_rope_init_requested_at)

        if bool(getattr(state, "has_motion_result_status", False)):
            rope_sequence = int(getattr(state, "rope_init_sequence", 0) or 0)
            rope_status = int(getattr(state, "rope_init_status", 0) or 0)
            rope_reason = int(getattr(state, "rope_init_reason", 0) or 0)
            if rope_sequence > self._p4_rope_init_sequence_base:
                if rope_status == ROPE_INIT_STATUS_RUNNING:
                    if self._p4_rope_init_phase != "running":
                        self._p4_rope_init_phase = "running"
                        self.context.emit_action(
                            "Rope Init started on P4: "
                            f"joint_mask=0x{self._p4_rope_init_joint_mask:08X}"
                        )
                        self._update_manual_action_buttons()
                    self._set_status(f"Rope Init running on P4: {elapsed:.1f} s")
                    return
                if rope_status == ROPE_INIT_STATUS_SUCCESS:
                    self._finish_p4_rope_init_progress(
                        True, "P4 reported SUCCESS"
                    )
                    return
                if rope_status == ROPE_INIT_STATUS_FAILED:
                    reason_label = ROPE_INIT_REASON_LABELS.get(
                        rope_reason, f"unknown reason {rope_reason}"
                    )
                    self._finish_p4_rope_init_progress(
                        False, f"P4 reported FAILED: {reason_label}"
                    )
                    return

        if system_state == SYSTEM_STATE_ROPE_INIT_RUNNING:
            if self._p4_rope_init_phase != "running":
                self._p4_rope_init_phase = "running"
                self.context.emit_action(
                    "Rope Init started on P4: "
                    f"joint_mask=0x{self._p4_rope_init_joint_mask:08X}"
                )
                self._update_manual_action_buttons()
            self._set_status(f"Rope Init running on P4: {elapsed:.1f} s")
            return

        task_has_finished = (
            self._p4_rope_init_phase == "running" or transition_delta >= 2
        )
        if task_has_finished:
            if system_state == SYSTEM_STATE_READY:
                self._finish_p4_rope_init_progress(True, "READY confirmed")
            elif system_state in (SYSTEM_STATE_IDLE, SYSTEM_STATE_UNCONTROLLABLE):
                self._finish_p4_rope_init_progress(
                    False,
                    f"final_state={system_state}; active joints did not reach READY",
                )
            return

        if elapsed >= ROPE_INIT_START_TIMEOUT_S:
            self._finish_p4_rope_init_progress(
                False,
                f"P4 did not enter ROPE_INIT within {ROPE_INIT_START_TIMEOUT_S:.1f} s",
            )
            return
        self._set_status(f"Rope Init queued on P4: waiting to start ({elapsed:.1f} s)")

    def calibrate(self) -> None:
        """Start the active-scope calibration workflow."""
        if self._manual_submode() != "joint":
            self._set_status("Calibrate is only available in Joint Channels")
            return
        if not bool(self.context.is_comm_connected()):
            self._warn("Calibration unavailable", "P4 is not connected.")
            self._set_status("Calibrate blocked: P4 is not connected")
            return

        try:
            latest_state = self.backend.get_state_snapshot()
        except Exception:
            latest_state = None
        if latest_state is not None:
            self._state = latest_state

        missing_manual = self._active_scope_manual_missing_indices()
        if missing_manual:
            self._set_status(
                "Calibrate: refreshing active-scope Manual endpoints from P4..."
            )
            try:
                refreshed = bool(self.backend.query_manual_encoder_references())
            except Exception:
                refreshed = False
            if refreshed:
                try:
                    latest_state = self.backend.get_state_snapshot()
                except Exception:
                    latest_state = None
                if latest_state is not None:
                    self._state = latest_state
                    if self.device_check_view is not None:
                        self.device_check_view.update_state(latest_state)
                missing_manual = self._active_scope_manual_missing_indices()
        if missing_manual:
            self._warn(
                "Manual Required",
                "Active joints are missing persisted Manual endpoints: "
                + self._format_joint_indices(missing_manual),
            )
            self._set_status("Calibrate blocked: active scope Manual is incomplete")
            self._update_manual_action_buttons()
            return

        system_state = getattr(self._state, "system_state", None)
        if system_state not in (
            SYSTEM_STATE_UNCONTROLLABLE,
            SYSTEM_STATE_IDLE,
            SYSTEM_STATE_READY,
            SYSTEM_STATE_RECOVERY_REQUIRED,
        ):
            self._warn(
                "Calibration unavailable",
                "Stop the active motion first. Calibration requires P4 state "
                f"UNCONTROLLABLE, IDLE, READY, or RECOVERY; current state={system_state}.",
            )
            self._set_status(
                f"Calibrate blocked: P4 system_state={system_state}"
            )
            return
        if not self._apply_active_scope_for_current_mode():
            self._set_status("Calibrate blocked: active scope was not accepted by lower controller")
            return
        self._start_calibration_now()

    def _start_calibration_now(self) -> None:
        """Handle start calibration now behavior for the control panel."""
        self._abs_limit_warning_unlocked_mask = 0
        self._feedback_abs_limit_popup_episode_active = False
        active_joint_mask = int(self._active_joint_mask_from_ui()) & ((1 << ENCODER_COUNT) - 1)
        active_motor_mask = int(self._active_motor_mask_from_ui()) & VALID_MOTOR_MASK
        self._pending_calib_joint_mask = active_joint_mask
        self._pending_calib_motor_mask = active_motor_mask
        current_results = list(
            getattr(self._state, "joint_calib_results", []) or []
        )
        self._pending_calib_result_baselines = {
            joint_idx: self._joint_calib_result_signature(
                current_results[joint_idx]
                if joint_idx < len(current_results)
                else None
            )
            for joint_idx in ControlScopeModel.ordered_joint_indices_from_mask(
                active_joint_mask
            )
        }
        for joint_idx in ControlScopeModel.ordered_joint_indices_from_mask(active_joint_mask):
            if 0 <= joint_idx < len(self._joint_target_user_set):
                self._joint_target_user_set[joint_idx] = False
        self._joint_target_initialized = False
        self.context.start_calibration()

    def _active_scope_abs_limit_valid_mask(self) -> int:
        """Return the active scope abs limit valid mask state."""
        state = self._state
        if state is None:
            return 0
        return self._active_motor_mask_from_ui() & int(getattr(state, "motor_abs_limit_valid_bitmap", 0))

    def _abs_limit_toggle_target_locked(self) -> bool:
        """Handle abs limit toggle target locked behavior for the control panel."""
        mask = self._active_scope_abs_limit_valid_mask()
        if mask == 0 or self._state is None:
            return True
        locked_bitmap = int(getattr(self._state, "motor_abs_limit_locked_bitmap", 0) or 0)
        locked_bitmap &= ~int(getattr(self, "_abs_limit_warning_unlocked_mask", 0) or 0)
        all_locked = (locked_bitmap & mask) == mask
        return not all_locked

    def _refresh_abs_limit_lock_toggle_button(self) -> None:
        """Refresh the abs limit lock toggle button display or controls."""
        button = getattr(self, "abs_limit_lock_toggle_btn", None)
        if button is None:
            return
        target_locked = self._abs_limit_toggle_target_locked()
        button.setText("LOCK" if target_locked else "UNLOCK")
        if target_locked:
            button.setToolTip(
                "Lock the calibrated 5-turn ABS travel limits for the selected feedback joints"
            )
        else:
            button.setToolTip(
                "Unlock the calibrated 5-turn ABS travel limits for the selected feedback joints"
            )

    def toggle_abs_limit_lock(self) -> None:
        """Toggle the abs limit lock setting."""
        self.set_abs_limit_lock(self._abs_limit_toggle_target_locked())

    def _active_scope_abs_limit_missing_mask(self) -> int:
        """Return the active scope abs limit missing mask state."""
        active_mask = self._active_motor_mask_from_ui() & VALID_MOTOR_MASK
        if active_mask == 0:
            return 0
        state = self._state
        if state is None:
            return active_mask
        valid_bitmap = int(getattr(state, "motor_abs_limit_valid_bitmap", 0) or 0)
        return active_mask & ~valid_bitmap

    def _active_scope_feedback_calibrated(self) -> bool:
        """Return the active scope feedback calibrated state."""
        active_mask = self._active_motor_mask_from_ui() & VALID_MOTOR_MASK
        if active_mask == 0 or self._active_scope_abs_limit_missing_mask() != 0:
            return False
        mode = self._manual_submode() if self._app_mode == "manual" else "joint"
        if mode != "motor" and self._active_scope_joint_calib_missing_indices():
            return False
        return True

    def _format_motor_mask(self, mask: int, limit: int = 6) -> str:
        """Format the motor mask for display."""
        indices = [
            idx for idx in range(MOTOR_COUNT)
            if _motor_channel_valid(idx) and (int(mask) & (1 << idx)) != 0
        ]
        if not indices:
            return "none"
        text = ",".join(f"M{idx:02d}" for idx in indices[:limit])
        if len(indices) > limit:
            text += f"...(+{len(indices) - limit})"
        return text

    def _turn_recovery_guide_text(self, unknown_mask: int, source: str = "") -> str:
        motors = self._format_motor_mask(unknown_mask)
        source_text = f"{source}\n\n" if source else ""
        return (
            f"{source_text}"
            "当前存在 Turn UNKNOWN，多圈位置不可信，系统已阻止相关关节继续控制。\n\n"
            f"受影响电机：{motors}\n"
            f"unknown mask：0x{int(unknown_mask):08X}\n\n"
            "为什么会这样：\n"
            "- 舵机单独断电后，P4 保存的多圈状态可能与重新上电后的 raw 不匹配。\n"
            "- 环形 raw 差值超过 256 counts 时，P4 会拒绝自动恢复。\n"
            "- 机构被手动转动、换舵机或改映射后，旧 LUT 也不能继续使用。\n\n"
            "恢复步骤：\n"
            "1. 停止发送运动命令，必要时点击 E-STOP。\n"
            "2. 确认舵机电源、总线和 ID 映射正常，机构不再被手动转动。\n"
            "3. 点击 New Device Reset；程序只重建在线 Unknown 舵机的 turn0。\n"
            "4. 界面会自动勾选受影响关节，按提示点击 Calibrate 做单关节标定。\n"
            "5. 双电机关节会由两台舵机共同重新标定 LUT，其他关节数据保留。\n"
            "6. 确认 TURN KNOWN、CALIB SUCCESS、OL OK 后再恢复控制。"
        )

    def turn_unknown_popup_seen(self) -> bool:
        """Run the turn unknown popup seen workflow."""
        return bool(self._turn_recovery_popup_episode_active)

    def mark_turn_unknown_popup_seen(self, unknown_mask: int) -> None:
        """Mark the turn unknown popup seen state."""
        unknown_mask = int(unknown_mask) & VALID_MOTOR_MASK
        if unknown_mask == 0:
            return
        self._turn_recovery_popup_episode_active = True
        self._turn_recovery_popup_episode_mask |= unknown_mask

    def reset_turn_unknown_popup_episode(self) -> None:
        """Reset the turn unknown popup episode state."""
        self._turn_recovery_popup_episode_active = False
        self._turn_recovery_popup_episode_mask = 0

    def reset_servo_turn_overflow_popup_episode(self) -> None:
        """越界全部解除后，允许下一次硬件圈数越界再次弹窗。"""
        self._servo_turn_overflow_popup_episode_active = False
        self._servo_turn_overflow_popup_episode_mask = 0

    def _show_servo_turn_overflow_guide(
        self, overflow_mask: int, source: str = "Servo feedback"
    ) -> None:
        """每次越界事件只提示一次，引导用户到 Motor Channels 执行 Reset Zero。"""
        overflow_mask = int(overflow_mask) & VALID_MOTOR_MASK
        if overflow_mask == 0:
            self.reset_servo_turn_overflow_popup_episode()
            return
        new_mask = overflow_mask & ~int(
            self._servo_turn_overflow_popup_episode_mask
        )
        if self._servo_turn_overflow_popup_episode_active and new_mask == 0:
            return
        self._servo_turn_overflow_popup_episode_active = True
        self._servo_turn_overflow_popup_episode_mask |= overflow_mask
        motor_text = self._format_motor_mask(overflow_mask, limit=MOTOR_COUNT)
        self.context.emit_action(
            f"{source}: servo hardware ABS overflow mask=0x{overflow_mask:08X}"
        )
        self._warn(
            "Servo Turn Overflow",
            (
                "检测到舵机内部硬件 ABS 严格越过 Reset Zero 阈值 "
                "[-30715, +30715]。\n"
                "底层命令硬边界仍为 [-30719, +30719]，当前保护会提前触发。\n\n"
                f"越界舵机：{motor_text}\n\n"
                "P4 已停止这些舵机的目标输出。请确认机构和绳路安全，"
                "打开 Motor Channels，保持 Flash ON，然后点击 Reset Zero。\n"
                "Reset Zero 只处理越界舵机；完成后对应关节的 Auto/LUT/"
                "ABS Limit 会失效，必须按界面自动勾选范围重新标定。"
            ),
        )

    def _show_turn_recovery_guide(self, unknown_mask: int, source: str = "") -> None:
        """Show the turn recovery guide UI message."""
        unknown_mask = int(unknown_mask) & VALID_MOTOR_MASK
        if unknown_mask == 0:
            return
        self.context.emit_action(
            f"{source or 'Command'} blocked: Turn UNKNOWN unknown=0x{unknown_mask:08X}"
        )
        self._set_status(
            f"{source or 'Command'} blocked: Turn UNKNOWN {self._format_motor_mask(unknown_mask)}; run New Device Reset first"
        )
        if self.turn_unknown_popup_seen():
            return
        self.mark_turn_unknown_popup_seen(unknown_mask)
        self._warn(
            "Turn UNKNOWN",
            self._turn_recovery_guide_text(unknown_mask, source),
        )

    def _format_joint_indices(self, indices: List[int], limit: int = 6) -> str:
        """Format the joint indices for display."""
        if not indices:
            return "none"
        text = ",".join(f"J{idx:02d}" for idx in indices[:limit])
        if len(indices) > limit:
            text += f"...(+{len(indices) - limit})"
        return text

    def _active_scope_joint_calib_missing_indices(self) -> List[int]:
        """Return the active scope joint calib missing indices state."""
        state = self._state
        active_joints = self._active_scope_joint_indices()
        if state is None:
            return active_joints
        missing: List[int] = []
        for joint_idx in active_joints:
            if not self._joint_workspace_range(joint_idx, state).ready:
                missing.append(joint_idx)
        return missing

    def _active_scope_manual_missing_indices(self) -> List[int]:
        """Return the active scope manual missing indices state."""
        active_joints = self._active_scope_joint_indices()
        state = self._state
        if state is None or not bool(getattr(state, "manual_reference_nvs_valid", False)):
            return active_joints
        valid_mask = int(getattr(state, "manual_reference_valid_bitmap", 0) or 0)
        min_values = list(getattr(state, "manual_reference_min_raw", []) or [])
        max_values = list(getattr(state, "manual_reference_max_raw", []) or [])
        directions = list(
            getattr(state, "manual_reference_encoder_directions", []) or []
        )
        has_directions = bool(
            getattr(state, "has_manual_reference_encoder_directions", False)
        )
        missing: List[int] = []
        for joint in active_joints:
            if (valid_mask & (1 << joint)) == 0:
                missing.append(joint)
                continue
            if (
                not has_directions
                or joint >= len(min_values)
                or joint >= len(max_values)
                or joint >= len(directions)
            ):
                missing.append(joint)
                continue
            span = directed_encoder_delta(
                int(min_values[joint]),
                int(max_values[joint]),
                int(directions[joint]),
            )
            if span is None or span <= 0:
                missing.append(joint)
        return missing

    def _active_scope_status_mismatch(self) -> str:
        """Return the active scope status mismatch state."""
        state = self._state
        if state is None or not bool(getattr(state, "has_active_scope_status", False)):
            return ""
        data = self._current_active_scope_tuple()
        if data is None:
            return ""
        expected_motor, expected_joint, expected_can = data
        actual_motor = int(getattr(state, "active_motor_mask", 0) or 0) & VALID_MOTOR_MASK
        actual_joint = int(getattr(state, "active_joint_mask", 0) or 0) & ((1 << ENCODER_COUNT) - 1)
        actual_can = bool(getattr(state, "require_can_feedback", False))
        expected_motor = int(expected_motor) & VALID_MOTOR_MASK
        expected_joint = int(expected_joint) & ((1 << ENCODER_COUNT) - 1)
        expected_can = bool(expected_can)
        parts: List[str] = []
        if actual_motor != expected_motor:
            parts.append(f"motor P4=0x{actual_motor:08X} UI=0x{expected_motor:08X}")
        if actual_joint != expected_joint:
            parts.append(f"joint P4=0x{actual_joint:08X} UI=0x{expected_joint:08X}")
        if actual_can != expected_can:
            parts.append(f"can P4={int(actual_can)} UI={int(expected_can)}")
        return "; ".join(parts)

    def _on_active_scope_changed(self) -> None:
        """Handle the active scope changed event."""
        if getattr(self, "scope_combo", None) is not None:
            self._scope_selection_index = int(self.scope_combo.currentIndex())
        self._apply_active_scope_for_current_mode()

    def _sync_motor_selection_from_joint_selection(self) -> None:
        """Synchronize the motor selection from joint selection state."""
        motor_mask = self._motor_mask_for_joint_mask(self._selected_joint_mask)
        self._selected_motor_mask = int(motor_mask) & VALID_MOTOR_MASK
        self._scope_update_guard = True
        try:
            for motor_idx, check in enumerate(getattr(self, "motor_use_checks", [])):
                if check is None:
                    continue
                check.blockSignals(True)
                check.setChecked((self._selected_motor_mask & (1 << motor_idx)) != 0)
                check.blockSignals(False)
        finally:
            self._scope_update_guard = False

    def _sync_scope_group_checks(self) -> None:
        """Synchronize the scope group checks state."""
        if bool(getattr(self, "_scope_update_guard", False)):
            return
        self._scope_update_guard = True
        try:
            for check in getattr(self, "joint_group_use_checks", []):
                text = str(check.text())
                target = next((item for item in FINGER_JOINT_RANGES if item[0] == text), None)
                if target is None:
                    continue
                label, start, end = target
                group_mask = ControlScopeModel.joint_group_mask(label, start, end)
                check.blockSignals(True)
                check.setChecked((self._selected_joint_mask & group_mask) == group_mask)
                check.blockSignals(False)

            motor_rows, group_ranges = ControlScopeModel.motor_rows_by_finger_group()
            for check, (start, end, _label) in zip(getattr(self, "motor_group_use_checks", []), group_ranges):
                group_mask = 0
                for row in range(start, end + 1):
                    if 0 <= row < len(motor_rows):
                        motor_idx = int(motor_rows[row])
                        if _motor_channel_valid(motor_idx):
                            group_mask |= (1 << motor_idx)
                check.blockSignals(True)
                check.setChecked(group_mask != 0 and (self._selected_motor_mask & group_mask) == group_mask)
                check.blockSignals(False)
        finally:
            self._scope_update_guard = False

    def _on_joint_use_changed(self, joint_idx: int, checked: bool) -> None:
        """Handle the joint use changed event."""
        if bool(getattr(self, "_scope_update_guard", False)):
            return
        if 0 <= int(joint_idx) < ENCODER_COUNT:
            bit = 1 << int(joint_idx)
            if checked:
                self._selected_joint_mask |= bit
            else:
                self._selected_joint_mask &= ~bit
        self._sync_motor_selection_from_joint_selection()
        self._sync_scope_group_checks()
        self._apply_active_scope_for_current_mode()
        self._update_manual_action_buttons()

    def _on_joint_group_use_changed(self, start: int, end: int, checked: bool) -> None:
        """Handle the joint group use changed event."""
        if bool(getattr(self, "_scope_update_guard", False)):
            return
        self._scope_update_guard = True
        try:
            for joint_idx in range(int(start), int(end) + 1):
                if 0 <= joint_idx < ENCODER_COUNT:
                    bit = 1 << joint_idx
                    if checked:
                        self._selected_joint_mask |= bit
                    else:
                        self._selected_joint_mask &= ~bit
                    check = self.joint_use_checks[joint_idx] if joint_idx < len(self.joint_use_checks) else None
                    if check is not None:
                        check.blockSignals(True)
                        check.setChecked(bool(checked))
                        check.blockSignals(False)
        finally:
            self._scope_update_guard = False
        self._sync_motor_selection_from_joint_selection()
        self._sync_scope_group_checks()
        self._apply_active_scope_for_current_mode()
        self._update_manual_action_buttons()

    def _on_motor_use_changed(self, motor_idx: int, checked: bool) -> None:
        """Handle the motor use changed event."""
        if bool(getattr(self, "_scope_update_guard", False)):
            return
        if _motor_channel_valid(int(motor_idx)):
            bit = 1 << int(motor_idx)
            if checked:
                self._selected_motor_mask |= bit
            else:
                self._selected_motor_mask &= ~bit
        self._sync_scope_group_checks()
        self._apply_active_scope_for_current_mode()
        self._update_manual_action_buttons()

    def _on_motor_group_use_changed(self, motor_indices: List[int], checked: bool) -> None:
        """Handle the motor group use changed event."""
        if bool(getattr(self, "_scope_update_guard", False)):
            return
        self._scope_update_guard = True
        try:
            for motor_idx in motor_indices:
                motor_idx = int(motor_idx)
                if not _motor_channel_valid(motor_idx):
                    continue
                bit = 1 << motor_idx
                if checked:
                    self._selected_motor_mask |= bit
                else:
                    self._selected_motor_mask &= ~bit
                check = self.motor_use_checks[motor_idx] if motor_idx < len(self.motor_use_checks) else None
                if check is not None:
                    check.blockSignals(True)
                    check.setChecked(bool(checked))
                    check.blockSignals(False)
        finally:
            self._scope_update_guard = False
        self._sync_scope_group_checks()
        self._apply_active_scope_for_current_mode()
        self._update_manual_action_buttons()

    def _teleop_selected_joint_mask_value(self) -> int:
        """Handle teleop selected joint mask value behavior for the control panel."""
        return int(getattr(self, "_teleop_selected_joint_mask", 0)) & ((1 << ENCODER_COUNT) - 1)

    def _sync_teleop_group_checks(self) -> None:
        """Synchronize the teleop group checks state."""
        if bool(getattr(self, "_scope_update_guard", False)):
            return
        self._scope_update_guard = True
        try:
            hand_check = getattr(self, "teleop_hand_use_check", None)
            if hand_check is not None:
                all_mask = (1 << ENCODER_COUNT) - 1
                hand_check.blockSignals(True)
                hand_check.setChecked((self._teleop_selected_joint_mask & all_mask) == all_mask)
                hand_check.blockSignals(False)
            for check in getattr(self, "teleop_group_use_checks", []):
                text = str(check.text())
                target = next((item for item in FINGER_JOINT_RANGES if item[0] == text), None)
                if target is None:
                    continue
                label, start, end = target
                group_mask = ControlScopeModel.joint_group_mask(label, start, end)
                check.blockSignals(True)
                check.setChecked((self._teleop_selected_joint_mask & group_mask) == group_mask)
                check.blockSignals(False)
        finally:
            self._scope_update_guard = False

    def _on_teleop_hand_use_changed(self, checked: bool) -> None:
        """Handle the teleop hand use changed event."""
        if bool(getattr(self, "_scope_update_guard", False)):
            return
        self._scope_update_guard = True
        try:
            all_mask = (1 << ENCODER_COUNT) - 1
            self._teleop_selected_joint_mask = all_mask if checked else 0
            for joint_idx, check in enumerate(getattr(self, "teleop_joint_use_checks", [])):
                if check is not None:
                    check.blockSignals(True)
                    check.setChecked(bool(checked))
                    check.blockSignals(False)
            for check in getattr(self, "teleop_group_use_checks", []):
                check.blockSignals(True)
                check.setChecked(bool(checked))
                check.blockSignals(False)
        finally:
            self._scope_update_guard = False
        self._teleop_selected_motor_mask = self._motor_mask_for_joint_mask(self._teleop_selected_joint_mask)
        self._apply_active_scope_for_current_mode()

    def _on_teleop_joint_use_changed(self, joint_idx: int, checked: bool) -> None:
        """Handle the teleop joint use changed event."""
        if bool(getattr(self, "_scope_update_guard", False)):
            return
        if 0 <= int(joint_idx) < ENCODER_COUNT:
            bit = 1 << int(joint_idx)
            if checked:
                self._teleop_selected_joint_mask |= bit
            else:
                self._teleop_selected_joint_mask &= ~bit
        self._teleop_selected_motor_mask = self._motor_mask_for_joint_mask(self._teleop_selected_joint_mask)
        self._sync_teleop_group_checks()
        self._apply_active_scope_for_current_mode()

    def _on_teleop_joint_group_use_changed(self, start: int, end: int, checked: bool) -> None:
        """Handle the teleop joint group use changed event."""
        if bool(getattr(self, "_scope_update_guard", False)):
            return
        self._scope_update_guard = True
        try:
            for joint_idx in range(int(start), int(end) + 1):
                if 0 <= joint_idx < ENCODER_COUNT:
                    bit = 1 << joint_idx
                    if checked:
                        self._teleop_selected_joint_mask |= bit
                    else:
                        self._teleop_selected_joint_mask &= ~bit
                    check = (
                        self.teleop_joint_use_checks[joint_idx]
                        if joint_idx < len(self.teleop_joint_use_checks)
                        else None
                    )
                    if check is not None:
                        check.blockSignals(True)
                        check.setChecked(bool(checked))
                        check.blockSignals(False)
        finally:
            self._scope_update_guard = False
        self._teleop_selected_motor_mask = self._motor_mask_for_joint_mask(self._teleop_selected_joint_mask)
        self._sync_teleop_group_checks()
        self._apply_active_scope_for_current_mode()

    def _current_active_scope_tuple(self) -> Optional[Tuple[int, int, bool]]:
        """Handle current active scope tuple behavior for the control panel."""
        if self._app_mode == "manual":
            mode = self._manual_submode()
            if mode == "motor":
                return int(self._selected_motor_mask) & VALID_MOTOR_MASK, 0, False
            joint_mask = int(self._selected_joint_mask) & ((1 << ENCODER_COUNT) - 1)
            motor_mask = self._motor_mask_for_joint_mask(joint_mask)
            return motor_mask, joint_mask, True

        if self._app_mode == "teleop":
            joint_mask = self._teleop_selected_joint_mask_value()
            motor_mask = self._motor_mask_for_joint_mask(joint_mask)
            self._teleop_selected_motor_mask = int(motor_mask) & VALID_MOTOR_MASK
            return int(motor_mask), int(joint_mask), True

        if self._app_mode == "replay" and self._state is not None:
            return (
                int(getattr(self._state, "active_motor_mask", 0) or 0),
                int(getattr(self._state, "active_joint_mask", 0) or 0),
                True,
            )

        data = self.scope_combo.currentData() if getattr(self, "scope_combo", None) is not None else None
        if not data:
            return None
        try:
            return int(data[0]), int(data[1]), bool(data[2])
        except Exception:
            return None

    def _active_motor_mask_from_ui(self) -> int:
        """Return the active motor mask from ui state."""
        data = self._current_active_scope_tuple()
        if data is None:
            return 0
        try:
            return int(data[0])
        except Exception:
            return 0

    def _active_motor_indices_from_ui(self) -> List[int]:
        """Return the active motor indices from ui state."""
        mask = self._active_motor_mask_from_ui()
        return [
            idx for idx in range(MOTOR_COUNT)
            if _motor_channel_valid(idx) and (mask & (1 << idx)) != 0
        ]

    def _active_scope_label_from_ui(self) -> str:
        """Return the active scope label from ui state."""
        if self._app_mode == "manual":
            mode = self._manual_submode()
            if mode == "motor":
                indices = self._active_motor_indices_from_ui()
                if len(indices) == 1:
                    return f"Manual Motor M{indices[0]:02d}"
                if indices:
                    return f"Manual Motors {len(indices)} selected"
                return "Manual Motors none"
            joints = self._active_scope_joint_indices()
            if len(joints) == 1:
                return f"Manual Joint J{joints[0]:02d}"
            if joints:
                return f"Manual Joints {len(joints)} selected"
            return "Manual Joints none"
        if self._app_mode == "teleop":
            joints = self._active_scope_joint_indices()
            if len(joints) == 1:
                return f"Teleop Joint J{joints[0]:02d}"
            if joints:
                return f"Teleop Joints {len(joints)} selected"
            return "Teleop Joints none"
        combo = getattr(self, "scope_combo", None)
        if combo is not None:
            try:
                return str(combo.currentText())
            except Exception:
                return ""
        return ""

    def _active_joint_mask_from_ui(self) -> int:
        """Return the active joint mask from ui state."""
        data = self._current_active_scope_tuple()
        if data is None:
            return 0
        try:
            return int(data[1])
        except Exception:
            return 0

    def _active_scope_multiturn_unknown_mask(self) -> int:
        """Return the active scope multiturn unknown mask state."""
        state = self._state
        if state is None:
            return 0
        try:
            unknown = int(getattr(state, "multi_turn_unknown_bitmap", 0))
        except Exception:
            unknown = 0
        return unknown & self._active_motor_mask_from_ui()

    def record_mid_calibration(self) -> None:
        """Record the mid calibration data."""
        state = self._state
        if state is None or not bool(getattr(state, "has_servo_angle_data", False)):
            self._set_status("Mid Calib blocked: waiting for servo ABS feedback")
            return

        motor_indices = self._active_motor_indices_from_ui()
        if not motor_indices:
            self._set_status("Mid Calib blocked: active scope has no motors")
            return

        servo_abs = getattr(state, "servo_angles", []) or []
        servo_raw = getattr(state, "servo_raw_positions", []) or []
        servo_online = getattr(state, "servo_online", []) or []
        raw_online = getattr(state, "servo_raw_online", []) or []
        unknown_mask = int(getattr(state, "multi_turn_unknown_bitmap", 0) or 0)

        records = []
        missing = []
        for motor_idx in motor_indices:
            if motor_idx >= len(servo_abs):
                missing.append(f"M{motor_idx:02d} abs")
                continue
            if motor_idx >= len(servo_online) or not bool(servo_online[motor_idx]):
                missing.append(f"M{motor_idx:02d} online")
                continue
            joint_idx = MOTOR_TO_JOINT_INDEX[motor_idx] if motor_idx < len(MOTOR_TO_JOINT_INDEX) else -1
            raw_value = None
            raw_ok = motor_idx < len(servo_raw) and motor_idx < len(raw_online) and bool(raw_online[motor_idx])
            if raw_ok:
                raw_value = int(servo_raw[motor_idx])
            records.append({
                "motor": int(motor_idx),
                "motor_label": f"M{motor_idx:02d}",
                "joint": int(joint_idx) if int(joint_idx) >= 0 else None,
                "joint_label": f"J{int(joint_idx):02d}" if int(joint_idx) >= 0 else None,
                "abs": int(servo_abs[motor_idx]),
                "raw": raw_value,
                "multi_turn_known": ((unknown_mask & (1 << motor_idx)) == 0),
            })

        if not records:
            self._set_status(f"Mid Calib blocked: {', '.join(missing[:4])}")
            return

        payload = {
            "comment": "Motor middle calibration. Record motor ABS when the tendon/rope is unwrapped; intended for later side-sway joint calibration.",
            "updated_at_host_time": time.time(),
            "hand_profile": HAND_PROFILE,
            "active_scope": {
                "label": self._active_scope_label_from_ui(),
                "motor_mask": f"0x{self._active_motor_mask_from_ui():08X}",
                "joint_mask": f"0x{self._active_joint_mask_from_ui():08X}",
            },
            "motors": records,
            "by_motor": {item["motor_label"]: item for item in records},
        }
        try:
            MOTOR_MID_CALIB_PATH.parent.mkdir(parents=True, exist_ok=True)
            MOTOR_MID_CALIB_PATH.write_text(
                json.dumps(payload, indent=2, ensure_ascii=False),
                encoding="utf-8",
            )
        except OSError as exc:
            self._set_status(f"Mid Calib save failed: {exc}")
            return

        self.context.emit_action(
            "Mid Calib saved: " +
            ",".join(f"{item['motor_label']}={item['abs']}" for item in records)
        )
        suffix = f"; skipped {', '.join(missing[:3])}" if missing else ""
        self._set_status(f"Mid Calib saved {len(records)} motor ABS to {MOTOR_MID_CALIB_PATH.name}{suffix}")

    @staticmethod
    def _format_joint_list(indices: List[int]) -> str:
        """Format the joint list for display."""
        return ",".join(f"J{idx:02d}" for idx in indices)

    @staticmethod
    def _format_motor_list(indices: List[int]) -> str:
        """Format the motor list for display."""
        return ",".join(f"M{idx:02d}" for idx in indices)

    @staticmethod
    def _snapshot_mask_value(value) -> int:
        """Handle snapshot mask value behavior for the control panel."""
        try:
            if isinstance(value, str):
                return int(value, 0)
            return int(value)
        except (TypeError, ValueError):
            return 0

    @staticmethod
    def _joint_calib_result_has_valid_tendon_lut(
        item: Optional[dict], joint_idx: int
    ) -> bool:
        """校验单/双电机关节共用的标定 LUT，并核对 LUT 行与实际电机映射。"""
        if not isinstance(item, dict) or not bool(item.get("success", False)):
            return False
        if not bool(tendon_lut_field(item, "tendonLutValid", False)):
            return False
        try:
            count = int(tendon_lut_field(item, "tendonLutCount", 0))
        except (TypeError, ValueError):
            return False
        if count < 3 or count > TENDON_LUT_MAX_SAMPLES:
            return False
        expected_count = 2 if int(joint_idx) in DUAL_TENDON_JOINT_INDICES else 1
        expected_motors = [
            motor_idx for motor_idx, mapped_joint in enumerate(MOTOR_TO_JOINT_INDEX)
            if int(mapped_joint) == int(joint_idx)
        ]
        if len(expected_motors) != expected_count:
            return False
        motors = tendon_lut_field(item, "tendonLutMotor", [])
        if not isinstance(motors, list) or len(motors) < 2:
            return False
        try:
            if any(int(motors[i]) != expected_motors[i] for i in range(expected_count)):
                return False
            if expected_count == 1 and int(motors[1]) != 0xFF:
                return False
        except (TypeError, ValueError):
            return False
        motor_abs = tendon_lut_field(item, "tendonLutMotorAbs", [])
        if not isinstance(motor_abs, list) or len(motor_abs) < 2:
            return False
        for row in motor_abs[:expected_count]:
            if not isinstance(row, list) or len(row) < count:
                return False
        return True

    def _valid_joint_calib_results_for_mask(self, state, joint_mask: int) -> List[Optional[dict]]:
        """Handle valid joint calib results for mask behavior for the control panel."""
        source = getattr(state, "joint_calib_results", []) or []
        results: List[Optional[dict]] = [None] * ENCODER_COUNT
        mask = int(joint_mask) & ((1 << ENCODER_COUNT) - 1)
        for joint_idx in range(ENCODER_COUNT):
            if (mask & (1 << joint_idx)) == 0:
                continue
            if joint_idx >= len(source):
                continue
            item = source[joint_idx]
            if not isinstance(item, dict) or not bool(item.get("success", False)):
                continue
            results[joint_idx] = dict(item)
        return results

    def _partial_calibration_completed_joint_mask(self, state) -> int:
        """读取 P4 的部分标定最终汇总，只返回本轮真正完成的 Jxx。"""
        if state is None or not bool(getattr(state, "calib_diag_valid", False)):
            return 0
        if (
            int(getattr(state, "calib_diag_reason", 0) or 0)
            != CALIB_DIAG_REASON_JOINT_STEP_FAILED
            or int(getattr(state, "calib_diag_phase", 0) or 0)
            != CALIB_DIAG_PHASE_SEQUENCE
            or int(getattr(state, "calib_diag_motor", 0xFF)) != 0xFF
        ):
            return 0
        completed_mask = int(getattr(state, "calib_diag_value_a", 0) or 0)
        failed_mask = int(getattr(state, "calib_diag_value_b", 0) or 0)
        reported_failed_mask = int(getattr(state, "calib_diag_bitmap", 0) or 0)
        if failed_mask == 0 or failed_mask != reported_failed_mask:
            return 0
        valid_joint_mask = (1 << ENCODER_COUNT) - 1
        return completed_mask & valid_joint_mask & ~failed_mask

    @staticmethod
    def _joint_calib_result_signature(item: Optional[dict]) -> Optional[str]:
        """生成稳定签名，用于区分本轮新结果与 P4 中原有的旧结果。"""
        if not isinstance(item, dict) or not bool(item.get("success", False)):
            return None
        return json.dumps(item, sort_keys=True, ensure_ascii=False)

    def _incremental_calibration_completed_joint_mask(self, state) -> int:
        """返回本轮已经产生新成功结果、但不依赖整轮状态的 Jxx 掩码。"""
        pending_mask = int(self._pending_calib_joint_mask) & (
            (1 << ENCODER_COUNT) - 1
        )
        if state is None or pending_mask == 0:
            return 0
        results = list(getattr(state, "joint_calib_results", []) or [])
        baselines = getattr(self, "_pending_calib_result_baselines", {}) or {}
        completed_mask = 0
        for joint_idx in ControlScopeModel.ordered_joint_indices_from_mask(
            pending_mask
        ):
            item = results[joint_idx] if joint_idx < len(results) else None
            signature = CalibrationWorkflowController._joint_calib_result_signature(
                item
            )
            if signature is not None and signature != baselines.get(joint_idx):
                completed_mask |= 1 << joint_idx
        return completed_mask

    def _maybe_save_calibration_snapshot(self, state) -> None:
        if state is None:
            return
        calib_status = str(getattr(state, "calib_status", "IDLE"))
        if calib_status not in {"PENDING", "RUNNING", "SUCCESS", "WARN", "FAILED"}:
            return
        valid_bitmap = int(getattr(state, "motor_abs_limit_valid_bitmap", 0) or 0)
        if valid_bitmap == 0:
            return
        pending_joint_mask = int(self._pending_calib_joint_mask)
        pending_joint_mask &= (1 << ENCODER_COUNT) - 1
        incremental_mask = self._incremental_calibration_completed_joint_mask(
            state
        )
        if calib_status in {"PENDING", "RUNNING"}:
            active_joint_mask = incremental_mask
        elif calib_status == "FAILED":
            # 整轮失败不等于每个关节都失败。只合并 P4 最终汇总中明确完成的关节，
            # 同时接纳已经逐关节上报的新结果；失败关节及旧快照保持不变。
            active_joint_mask = pending_joint_mask & (
                self._partial_calibration_completed_joint_mask(state)
                | incremental_mask
            )
        else:
            active_joint_mask = pending_joint_mask
        if active_joint_mask == 0:
            return
        # 只更新目标关节直接使用的舵机。工作空间辅助电机不属于本轮标定，
        # 不能借由 pending motor mask 覆盖它们已有的 ABS Limit。
        active_motor_mask = self._motor_mask_for_joint_mask(active_joint_mask)
        required_motor_mask = active_motor_mask & VALID_MOTOR_MASK
        if required_motor_mask != 0 and (valid_bitmap & required_motor_mask) != required_motor_mask:
            missing_motors = [
                idx for idx in range(MOTOR_COUNT)
                if (required_motor_mask & (1 << idx)) and not (valid_bitmap & (1 << idx))
            ]
            if missing_motors:
                self._set_status(
                    "Calib snapshot waiting motor limits: " +
                    self._format_motor_list(missing_motors[:8])
                )
            return

        joint_results = self._valid_joint_calib_results_for_mask(state, active_joint_mask)
        if not any(item is not None for item in joint_results):
            return
        required_joints = [
            idx for idx in range(ENCODER_COUNT)
            if (active_joint_mask & (1 << idx)) != 0
        ]
        missing_joints = [idx for idx in required_joints if not joint_results[idx]]
        if missing_joints:
            self._set_status(
                "Calib snapshot waiting joint results: " +
                self._format_joint_list(missing_joints[:8])
            )
            return
        missing_tendon_lut = [
            idx for idx in required_joints
            if not self._joint_calib_result_has_valid_tendon_lut(
                joint_results[idx], idx
            )
        ]
        if missing_tendon_lut:
            self._set_status(
                "Calib snapshot waiting tendon LUT: " +
                self._format_joint_list(missing_tendon_lut[:8])
            )
            return

        manual_valid_mask = int(
            getattr(state, "manual_reference_valid_bitmap", 0) or 0
        )
        manual_min_values = list(
            getattr(state, "manual_reference_min_raw", []) or []
        )
        manual_max_values = list(
            getattr(state, "manual_reference_max_raw", []) or []
        )
        if (manual_valid_mask & active_joint_mask) != active_joint_mask:
            missing_mask = active_joint_mask & ~manual_valid_mask
            self._set_status(
                "Calib snapshot save blocked: Manual missing " +
                self._format_joint_list(
                    ControlScopeModel.ordered_joint_indices_from_mask(missing_mask)
                )
            )
            return
        if (
            len(manual_min_values) < ENCODER_COUNT
            or len(manual_max_values) < ENCODER_COUNT
        ):
            self._set_status(
                "Calib snapshot save blocked: Manual endpoint readback incomplete"
            )
            return
        incompatible_v3 = [
            idx for idx in required_joints
            if int(joint_results[idx].get("mappingVersion", 0) or 0) != 3 or
            int(joint_results[idx].get("manualReferenceCrc", 0) or 0)
            != manual_encoder_reference_entry_crc(
                idx,
                manual_min_values[idx],
                manual_max_values[idx],
            )
        ]
        if incompatible_v3:
            self._set_status(
                "Calib snapshot save blocked: v3/Manual CRC mismatch " +
                self._format_joint_list(incompatible_v3)
            )
            return

        mins = [int(v) for v in list(getattr(state, "motor_abs_limit_min", []) or [])[:MOTOR_COUNT]]
        maxs = [int(v) for v in list(getattr(state, "motor_abs_limit_max", []) or [])[:MOTOR_COUNT]]
        if len(mins) != MOTOR_COUNT or len(maxs) != MOTOR_COUNT:
            return

        try:
            mapping_version, urdf_config_crc = (
                urdf_manual_scaled_mapping_metadata()
            )
        except (OSError, TypeError, ValueError) as exc:
            self._set_status(f"Calib snapshot save blocked: invalid URDF mapping: {exc}")
            return

        payload = {
            "saved_at_host_time": time.time(),
            "hand_profile": HAND_PROFILE,
            "feedback_mapping_version": mapping_version,
            "manual_scaled_mapping_version": mapping_version,
            "urdf_config_crc": f"0x{urdf_config_crc:08X}",
            "manual_reference_crc": (
                f"0x{int(getattr(state, 'manual_reference_crc', 0) or 0):08X}"
            ),
            "active_joint_mask": f"0x{active_joint_mask:08X}",
            "active_motor_mask": f"0x{active_motor_mask:08X}",
            "motor_abs_limit_valid_bitmap": f"0x{valid_bitmap:08X}",
            "motor_abs_limit_locked_bitmap": (
                f"0x{int(getattr(state, 'motor_abs_limit_locked_bitmap', 0) or 0):08X}"
            ),
            "motor_abs_limit_min": mins,
            "motor_abs_limit_max": maxs,
            "manual_reference_valid_bitmap": f"0x{manual_valid_mask:08X}",
            "manual_reference_min_raw": [
                int(value) for value in manual_min_values[:ENCODER_COUNT]
            ],
            "manual_reference_max_raw": [
                int(value) for value in manual_max_values[:ENCODER_COUNT]
            ],
            "source": {
                "transport": "P4",
                "identity_contract": "hand_profile+urdf_crc+manual_entry_crc",
            },
            "joint_calib_results": joint_results,
        }

        previous_payload = None
        if CALIB_LOAD_SNAPSHOT_PATH.exists():
            try:
                previous_payload = read_snapshot(CALIB_LOAD_SNAPSHOT_PATH)
            except CalibrationSnapshotError as exc:
                self._set_status(
                    f"Calib snapshot save blocked: existing V5 snapshot invalid: {exc}"
                )
                return

        try:
            payload = merge_snapshot(
                previous_payload,
                payload,
                updated_joint_mask=active_joint_mask,
            )
        except CalibrationSnapshotError as exc:
            self._set_status(f"Calib snapshot merge failed: {exc}")
            return

        signature = json.dumps(
            {
                "active_joint_mask": payload["active_joint_mask"],
                "active_motor_mask": payload["active_motor_mask"],
                "valid": payload["motor_abs_limit_valid_bitmap"],
                "locked": payload["motor_abs_limit_locked_bitmap"],
                "mins": payload["motor_abs_limit_min"],
                "maxs": payload["motor_abs_limit_max"],
                "joints": payload["joint_calib_results"],
            },
            sort_keys=True,
            ensure_ascii=False,
        )
        if signature == self._last_calib_snapshot_signature:
            if calib_status in {"SUCCESS", "WARN"}:
                self._pending_calib_joint_mask = 0
                self._pending_calib_motor_mask = 0
                self._pending_calib_result_baselines = {}
            return
        try:
            payload = atomic_write_snapshot(CALIB_LOAD_SNAPSHOT_PATH, payload)
        except (OSError, CalibrationSnapshotError) as exc:
            self._set_status(f"Calib snapshot save failed: {exc}")
            return
        self._last_calib_snapshot_signature = signature
        # 更新已落盘 Jxx 的基线；RUNNING 时继续等待其余关节，不能清空整轮 mask。
        baselines = getattr(self, "_pending_calib_result_baselines", None)
        if not isinstance(baselines, dict):
            baselines = {}
            self._pending_calib_result_baselines = baselines
        for joint_idx in ControlScopeModel.ordered_joint_indices_from_mask(
            active_joint_mask
        ):
            baselines[joint_idx] = (
                self._joint_calib_result_signature(joint_results[joint_idx])
            )
        if calib_status in {"SUCCESS", "WARN"}:
            self._pending_calib_joint_mask = 0
            self._pending_calib_motor_mask = 0
            self._pending_calib_result_baselines = {}
        self.context.emit_action(f"Calib snapshot saved: {CALIB_LOAD_SNAPSHOT_PATH.name}")
        self._set_status(f"Calib snapshot saved to {CALIB_LOAD_SNAPSHOT_PATH.name}")
        self._update_manual_action_buttons()



    def _validate_calibration_snapshot_for_load(self, snapshot: dict) -> Optional[str]:
        try:
            validate_snapshot(snapshot)
        except CalibrationSnapshotError as exc:
            return str(exc)
        snapshot_joint_mask = self._snapshot_mask_value(snapshot.get("active_joint_mask"))
        snapshot_motor_mask = self._snapshot_mask_value(snapshot.get("active_motor_mask")) & VALID_MOTOR_MASK
        valid_motor_mask = self._snapshot_mask_value(
            snapshot.get("motor_abs_limit_valid_bitmap")
        ) & VALID_MOTOR_MASK

        joint_results = snapshot.get("joint_calib_results", [])
        if len(joint_results) != ENCODER_COUNT:
            return f"invalid joint calibration count: {len(joint_results)}"
        if snapshot_joint_mask == 0:
            for joint_idx, item in enumerate(joint_results):
                if isinstance(item, dict) and bool(item.get("success", False)):
                    snapshot_joint_mask |= (1 << joint_idx)
        missing_joints = [
            joint_idx for joint_idx in range(ENCODER_COUNT)
            if (snapshot_joint_mask & (1 << joint_idx)) and (
                not isinstance(joint_results[joint_idx], dict) or
                not bool(joint_results[joint_idx].get("success", False))
            )
        ]
        if missing_joints:
            return (
                "snapshot missing active joint calibration: " +
                self._format_joint_list(missing_joints)
            )
        try:
            expected_version, expected_crc = (
                urdf_manual_scaled_mapping_metadata()
            )
        except (OSError, TypeError, ValueError) as exc:
            return f"current manual-scaled URDF mapping is invalid: {exc}"
        incompatible_mapping = [
            joint_idx for joint_idx in range(ENCODER_COUNT)
            if (snapshot_joint_mask & (1 << joint_idx)) and (
                int(joint_results[joint_idx].get("mappingVersion", 0) or 0)
                != expected_version or
                int(joint_results[joint_idx].get("urdfConfigCrc", 0) or 0)
                != expected_crc
            )
        ]
        if incompatible_mapping:
            return (
                "snapshot mapping/URDF contract mismatch; recalibration required: " +
                self._format_joint_list(incompatible_mapping)
            )
        missing_tendon_lut = [
            joint_idx for joint_idx in range(ENCODER_COUNT)
            if (snapshot_joint_mask & (1 << joint_idx)) and
            not self._joint_calib_result_has_valid_tendon_lut(
                joint_results[joint_idx], joint_idx
            )
        ]
        if missing_tendon_lut:
            return (
                "snapshot missing tendon LUT; recalibrate with the current firmware: " +
                self._format_joint_list(missing_tendon_lut)
            )
        missing_motors = [
            motor_idx for motor_idx in range(MOTOR_COUNT)
            if (snapshot_motor_mask & (1 << motor_idx)) and
            not (valid_motor_mask & (1 << motor_idx))
        ]
        if missing_motors:
            return (
                "snapshot missing active motor ABS limits: " +
                self._format_motor_list(missing_motors)
            )
        return None

    def _latest_new_device_reset_state(self):
        """读取 New Device Reset 所需的最新状态，避免使用 UI 缓存旧帧。"""
        state = self._state
        getter = getattr(self.backend, "get_state_snapshot", None)
        if callable(getter):
            try:
                latest = getter()
                if latest is not None:
                    state = latest
                    self._state = latest
            except Exception:
                # 串口刚断开时快照可能读取失败，后续门禁仍会给出明确提示。
                pass
        return state

    def _new_device_reset_action_state(self, state=None):
        """统一按钮与点击处理的门禁，返回 (可执行, 原因提示)。"""
        if not self.context.is_comm_connected():
            return False, "P4 未连接，无法执行 New Device Reset"
        if state is None:
            state = self._state
        unknown_mask = (
            int(getattr(state, "multi_turn_unknown_bitmap", 0) or 0)
            & VALID_MOTOR_MASK
        )
        if unknown_mask == 0:
            return False, "全部舵机圈数可信，禁止执行 New Device Reset"
        system_state = getattr(state, "system_state", None) if state is not None else None
        if system_state in (
            SYSTEM_STATE_CALIBRATION_RUNNING,
            SYSTEM_STATE_ROPE_INIT_RUNNING,
        ):
            return False, "标定或 Rope Init 正在运行，请等待任务结束"
        if state is None or not bool(getattr(state, "has_servo_raw_data", False)):
            return True, "等待舵机 Raw 反馈；点击后可查看具体缺失原因"
        return True, "修复在线 Turn Unknown 舵机；运行中的控制会先安全停止"

    def new_device_reset(self) -> None:
        self.context.emit_action("New Device Reset clicked")
        self._set_status("New Device Reset checking...")
        state = self._latest_new_device_reset_state()
        can_reset, blocked_reason = self._new_device_reset_action_state(state)
        if not can_reset:
            self._warn("New Device Reset", blocked_reason)
            self._set_status(f"New Device Reset blocked: {blocked_reason}")
            self.context.emit_action(f"New Device Reset blocked: {blocked_reason}")
            return
        multi_turn_unknown_mask = (
            int(getattr(state, "multi_turn_unknown_bitmap", 0) or 0)
            & VALID_MOTOR_MASK
        )
        if state is None or not bool(getattr(state, "has_servo_raw_data", False)):
            self._warn(
                "New Device Reset",
                "尚未收到舵机 raw 反馈，无法建立 turn0 基准。",
            )
            self._set_status("New Device Reset blocked: waiting for servo raw feedback")
            return

        raw_online = list(getattr(state, "servo_raw_online", []) or [])
        repair_motor_mask = 0
        for motor_idx in range(min(MOTOR_COUNT, len(raw_online))):
            if (
                multi_turn_unknown_mask & (1 << motor_idx)
                and bool(raw_online[motor_idx])
            ):
                repair_motor_mask |= 1 << motor_idx
        if repair_motor_mask == 0:
            self._warn(
                "New Device Reset",
                "Turn Unknown 舵机当前不在线，请先恢复舵机供电和总线通信。",
            )
            return

        affected_joint_mask = 0
        for motor_idx, joint_idx in enumerate(
            MOTOR_TO_JOINT_INDEX[:MOTOR_COUNT]
        ):
            if (
                repair_motor_mask & (1 << motor_idx)
                and 0 <= int(joint_idx) < ENCODER_COUNT
            ):
                affected_joint_mask |= 1 << int(joint_idx)
        repair_motors = self._format_motor_mask(repair_motor_mask, limit=MOTOR_COUNT)
        repair_joints = self._format_joint_indices(
            ControlScopeModel.ordered_joint_indices_from_mask(
                affected_joint_mask
            ),
            limit=ENCODER_COUNT,
        )

        reply = QtWidgets.QMessageBox.question(
            self,
            "New Device Reset",
            (
                "将只修复当前在线且 Turn Unknown 的舵机。\n\n"
                f"故障舵机：{repair_motors}\n"
                f"需要重新标定的关节：{repair_joints}\n\n"
                "本次操作会：\n"
                "- 停止当前控制输出\n"
                "- 仅将上述故障舵机的当前 raw 设为 turn0\n"
                "- 仅清除其所属关节的 Auto/LUT/ABS Limit\n"
                "- 保留其他关节的多圈和标定数据\n\n"
                "双电机关节即使只坏一台舵机，也必须整关节重新标定。\n"
                "完成后界面会自动勾选受影响关节，请再点击 Calibrate。"
            ),
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply != QtWidgets.QMessageBox.Yes:
            self._set_status("New Device Reset cancelled")
            return

        self._set_status("New Device Reset running...")
        ok, message, restored_motor_mask, invalidated_joint_mask = (
            self.backend.reset_multiturn_baseline_for_new_device()
        )
        if ok:
            # P4 以“关节”为标定失效单位：双电机关节即使只恢复一台舵机，
            # 两台舵机共同使用的 LUT/ABS Limit 也必须一起清除。本地状态和
            # Calib Load 快照必须使用完整关节电机范围，不能留下半份旧数据。
            invalidated_motor_mask = self._motor_mask_for_joint_mask(
                invalidated_joint_mask
            )
            self._joint_target_initialized = False
            self._reset_motor_target_cache_for_device_epoch(
                "New Device Reset: motor targets cleared"
            )
            self._invalidate_local_calibration_scope(
                invalidated_motor_mask, invalidated_joint_mask
            )
            archive_message = self._backup_and_invalidate_calib_snapshot_scope(
                invalidated_motor_mask, invalidated_joint_mask
            )
            self._select_single_joint_recalibration_scope(
                invalidated_joint_mask
            )
            self._last_multi_turn_unknown_bitmap = None
            self.reset_turn_unknown_popup_episode()
            joint_text = self._format_joint_indices(
                ControlScopeModel.ordered_joint_indices_from_mask(
                    invalidated_joint_mask
                ),
                limit=ENCODER_COUNT,
            )
            final_message = (
                f"{message}; {archive_message}; "
                f"single-joint recalibration required: {joint_text}"
            )
            self._set_status(f"New Device Reset success: {final_message}")
            self.context.emit_action(f"New Device Reset success: {final_message}")
            QtWidgets.QMessageBox.information(
                self,
                "New Device Reset 完成",
                (
                    "局部多圈基准已经重建。\n\n"
                    f"已自动切换到 Joint Channels 并只勾选：{joint_text}\n"
                    "请确认对应手指绳路和运动空间安全，然后点击 Calibrate。\n\n"
                    "标定不要求已有全手快照：只要所选关节 Manual 完整、"
                    "舵机在线且 Turn Known，也可以重新勾选全手后直接标定。\n\n"
                    "标定完成并显示 TURN KNOWN / CALIB SUCCESS 后，"
                    "再恢复正常控制。"
                ),
            )
        else:
            self._error("New Device Reset", f"重置失败：{message}")
            self._set_status(f"New Device Reset failed: {message}")
        self._update_manual_action_buttons()

    def reset_overflow_servo_zero(self) -> None:
        """执行硬件 RESET、Flash 通道替换，并同步失效受影响关节的标定。"""
        state = self._state
        overflow_mask = (
            int(getattr(state, "servo_turn_overflow_bitmap", 0) or 0)
            & VALID_MOTOR_MASK
            if state is not None
            else 0
        )
        if overflow_mask == 0:
            self._warn("Reset Zero", "当前没有硬件 ABS 越过 ±30715 的舵机。")
            return
        motor_text = self._format_motor_mask(overflow_mask, limit=MOTOR_COUNT)
        reply = QtWidgets.QMessageBox.question(
            self,
            "Reset Zero",
            (
                f"将重置以下越界舵机的内部圈数：{motor_text}\n\n"
                "该操作会：\n"
                "1. 停止控制并关闭目标舵机扭矩；\n"
                "2. 向舵机发送 RESET(0x0A)，将硬件圈数清零；\n"
                "3. 用复位后 raw 仅替换这些舵机的 P4 Flash 通道；\n"
                "4. 清除其所属关节的 Auto/LUT/ABS Limit。\n\n"
                "双电机关节只要一台舵机被重置，整个关节都必须重新标定。"
            ),
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply != QtWidgets.QMessageBox.Yes:
            return

        self._set_status(f"Reset Zero running: {motor_text}")
        ok, message, success_mask, failed_mask, joint_mask = (
            self.backend.reset_overflow_servo_turns()
        )

        # 只要硬件 RESET 已成功，就不能再保留该关节旧标定；即使 Flash 写入失败，
        # 本地快照也必须失效，防止下次 Calib Load 把旧 LUT 重新灌回 P4。
        if success_mask:
            if joint_mask == 0:
                for motor_idx, joint_idx in enumerate(
                    MOTOR_TO_JOINT_INDEX[:MOTOR_COUNT]
                ):
                    if (
                        success_mask & (1 << motor_idx)
                        and 0 <= int(joint_idx) < ENCODER_COUNT
                    ):
                        joint_mask |= 1 << int(joint_idx)
            invalidated_motor_mask = self._motor_mask_for_joint_mask(joint_mask)
            self._joint_target_initialized = False
            self._reset_motor_target_cache_for_device_epoch(
                "Reset Zero: motor targets cleared"
            )
            self._invalidate_local_calibration_scope(
                invalidated_motor_mask, joint_mask
            )
            archive_message = self._backup_and_invalidate_calib_snapshot_scope(
                invalidated_motor_mask,
                joint_mask,
                reason="reset_zero",
            )
            self._select_single_joint_recalibration_scope(joint_mask)
        else:
            archive_message = "no hardware reset completed"

        if ok:
            self.reset_servo_turn_overflow_popup_episode()
            joint_text = self._format_joint_indices(
                ControlScopeModel.ordered_joint_indices_from_mask(joint_mask),
                limit=ENCODER_COUNT,
            )
            final_message = (
                f"{message}; {archive_message}; recalibrate {joint_text}"
            )
            self._set_status(f"Reset Zero success: {final_message}")
            self.context.emit_action(f"Reset Zero success: {final_message}")
            QtWidgets.QMessageBox.information(
                self,
                "Reset Zero completed",
                (
                    f"{message}\n\n"
                    f"受影响关节：{joint_text}\n"
                    "界面已自动切换并勾选这些关节，请重新执行 Calibrate。"
                ),
            )
        else:
            failed_text = self._format_motor_mask(
                failed_mask, limit=MOTOR_COUNT
            )
            self._error(
                "Reset Zero",
                (
                    f"{message}\n"
                    f"失败/未完成舵机：{failed_text}\n"
                    f"{archive_message}\n\n"
                    "如部分舵机已完成硬件 RESET，其所属关节仍必须重新标定。"
                ),
            )
            self._set_status(f"Reset Zero failed: {message}")
        self._update_manual_action_buttons()

    def load_calibration_snapshot(self) -> None:
        if not CALIB_LOAD_SNAPSHOT_PATH.exists():
            self._warn("Calib Load", "没有找到可加载的标定快照，请先完成 Calibrate。")
            return
        try:
            snapshot = read_snapshot(CALIB_LOAD_SNAPSHOT_PATH)
        except CalibrationSnapshotError as exc:
            self._error("Calib Load", f"读取标定快照失败：{exc}")
            return
        if str(snapshot.get("hand_profile", "")) != HAND_PROFILE:
            self._error(
                "Calib Load",
                f"标定快照 hand_profile 不一致：{snapshot.get('hand_profile')} != {HAND_PROFILE}",
            )
            return
        snapshot_error = self._validate_calibration_snapshot_for_load(snapshot)
        if snapshot_error:
            self._error("Calib Load", snapshot_error)
            self._set_status(f"Calib Load blocked: {snapshot_error}")
            return
        state = self._state
        system_state = getattr(state, "system_state", None) if state is not None else None
        if system_state in (SYSTEM_STATE_RUN_DIRECT, SYSTEM_STATE_RUN_FEEDBACK):
            self._warn("Calib Load", "控制正在运行，请先 E-STOP 或 Pause 后再加载。")
            return
        reply = QtWidgets.QMessageBox.question(
            self,
            "Calib Load",
            (
                "确认加载最近保存的标定快照？\n\n"
                "仅当机械、舵机 ID、映射和标定姿态没有变化时才应使用。"
            ),
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply != QtWidgets.QMessageBox.Yes:
            return
        self._set_status("Calib Load sending snapshot...")
        ok, message = self.backend.load_calibration_snapshot(snapshot)
        if ok:
            self._joint_target_initialized = False
            self._motor_target_initialized = False
            mode = self._manual_submode() if self._app_mode == "manual" else "joint"
            scope_ok = False
            if mode != "device_check":
                scope_ok = self._apply_active_scope_for_current_mode()
            ready_recheck = False
            current_state = getattr(self._state, "system_state", None) if self._state is not None else None
            if (
                scope_ok and
                mode in ("joint", "test") and
                current_state in (
                    SYSTEM_STATE_UNCONTROLLABLE,
                    SYSTEM_STATE_IDLE,
                    SYSTEM_STATE_READY,
                    SYSTEM_STATE_RECOVERY_REQUIRED,
                )
            ):
                self.backend.start()
                ready_recheck = True
            if mode == "device_check":
                suffix = "; snapshot restored; switch to Joint Channels to apply active scope"
            elif scope_ok:
                suffix = "; active scope applied"
                if ready_recheck:
                    suffix += "; READY recheck requested"
            else:
                suffix = "; active scope not applied"
            self._set_status(f"Calib Load success: {message}{suffix}")
            self.context.emit_action(f"Calib Load success: {message}{suffix}")
        else:
            self._error("Calib Load", f"加载失败：{message}")
            self._set_status(f"Calib Load failed: {message}")

    def _apply_active_scope_for_current_mode(self, force_joint: bool = False) -> bool:
        """Apply the active scope for current mode selection or policy."""
        if self._app_mode == "manual" and self._manual_submode() == "device_check":
            self.backend.set_pid_control(False)
            return True
        data = self._current_active_scope_tuple()
        if force_joint and self._app_mode == "manual":
            joint_mask = int(self._selected_joint_mask) & ((1 << ENCODER_COUNT) - 1)
            data = (
                self._motor_mask_for_joint_mask(joint_mask),
                joint_mask,
                True,
            )
        if data is None:
            return False
        motor_mask, joint_mask, require_can = data
        if int(motor_mask) == 0 and int(joint_mask) == 0:
            self._set_status("Active scope blocked: no channel selected")
            return False
        if not self.context.is_comm_connected():
            self._set_status("Active scope will apply after connect")
            return False
        try:
            mode = "joint" if force_joint else (
                self._manual_submode() if self._app_mode == "manual" else "joint"
            )
            if self._app_mode in ("manual", "teleop"):
                if mode == "motor":
                    self.backend.set_control_mode(ControlMode.DIRECT_MOTOR)
                    self.backend.set_pid_control(False)
                    require_can = False
                else:
                    self.backend.set_control_mode(ControlMode.JOINT_ANGLE)
                    self.backend.set_pid_control(bool(self._interaction_enabled or self._app_mode == "teleop"))
                    require_can = True
            ok = self.backend.set_active_scope(int(motor_mask), int(joint_mask), bool(require_can))
            if ok and self._state is not None and bool(getattr(self._state, "has_active_scope_status", False)):
                self._state.active_motor_mask = int(motor_mask) & VALID_MOTOR_MASK
                self._state.active_joint_mask = int(joint_mask) & ((1 << ENCODER_COUNT) - 1)
                self._state.require_can_feedback = bool(require_can)
        except Exception as exc:
            self._set_status(f"Active scope failed: {exc}")
            return False
        self._set_status("Active scope applied" if ok else "Active scope ACK timeout")
        return bool(ok)

    def begin_startup_device_check(self) -> None:
        """Begin the startup device check workflow."""
        if self._startup_device_check_running or not self.context.is_comm_connected():
            return
        self._startup_device_check_running = True
        self._startup_device_check_epoch += 1
        epoch = self._startup_device_check_epoch

        def query_worker() -> None:
            """Run the query worker workflow."""
            try:
                ok = bool(self.backend.query_manual_encoder_references())
                message = "Manual endpoints received" if ok else "Manual query ACK/readback timeout"
            except Exception as exc:
                ok = False
                message = f"{type(exc).__name__}: {exc}"
            self.startup_device_check_finished.emit(epoch, "query", ok, message)

        threading.Thread(
            target=query_worker,
            name="mmhand-startup-device-check",
            daemon=True,
        ).start()

    def _on_startup_device_check_finished(
        self,
        epoch: int,
        phase: str,
        ok: bool,
        message: str,
    ) -> None:
        if int(epoch) != self._startup_device_check_epoch or not self.context.is_comm_connected():
            return
        if phase == "load":
            self._startup_device_check_running = False
            if ok:
                self._joint_target_initialized = False
                self._motor_target_initialized = False
                self._set_status(f"Startup Calib Load success: {message}; switch to Joint Channels to apply scope")
                self.context.emit_action(f"Startup Calib Load success: {message}")
            else:
                self._set_status(f"Startup Calib Load skipped: {message}; automatic calibration required")
            return

        if not ok:
            self._startup_device_check_running = False
            self._set_status(f"Device Check failed: {message}")
            return

        try:
            state = self.backend.get_state_snapshot()
        except Exception:
            state = self._state
        if state is not None:
            self._state = state
            if self.device_check_view is not None:
                self.device_check_view.update_state(state)

        active_joint_mask = int(self._active_joint_mask_from_ui()) & ((1 << ENCODER_COUNT) - 1)
        manual_mask = int(getattr(state, "manual_reference_valid_bitmap", 0) or 0) if state is not None else 0
        nvs_valid = bool(getattr(state, "manual_reference_nvs_valid", False)) if state is not None else False
        missing_mask = active_joint_mask if not nvs_valid else active_joint_mask & ~manual_mask
        if missing_mask:
            self._startup_device_check_running = False
            missing = ControlScopeModel.ordered_joint_indices_from_mask(
                missing_mask
            )
            text = self._format_joint_indices(missing)
            self._set_status(f"Device Check: active scope Manual missing ({text})")
            if not self._startup_missing_warning_shown:
                self._startup_missing_warning_shown = True
                self._warn(
                    "Manual Required",
                    "Current control set is missing persisted Manual Min/Max: "
                    + text
                    + ". Enter them in Device Check before calibration or control.",
                )
            return

        if not CALIB_LOAD_SNAPSHOT_PATH.exists():
            self._startup_device_check_running = False
            self._set_status("Device Check passed: no calibration snapshot; run automatic calibration")
            return
        try:
            snapshot = read_snapshot(CALIB_LOAD_SNAPSHOT_PATH)
        except CalibrationSnapshotError as exc:
            self._startup_device_check_running = False
            self._set_status(f"Startup Calib Load skipped: invalid snapshot ({exc})")
            return
        if str(snapshot.get("hand_profile", "")) != HAND_PROFILE:
            self._startup_device_check_running = False
            self._set_status("Startup Calib Load skipped: hand profile mismatch")
            return
        snapshot_error = self._validate_calibration_snapshot_for_load(snapshot)
        if snapshot_error:
            self._startup_device_check_running = False
            self._set_status(f"Startup Calib Load skipped: {snapshot_error}")
            return
        system_state = getattr(state, "system_state", None) if state is not None else None
        if system_state in (SYSTEM_STATE_RUN_DIRECT, SYSTEM_STATE_RUN_FEEDBACK, SYSTEM_STATE_CALIBRATION_RUNNING):
            self._startup_device_check_running = False
            self._set_status("Startup Calib Load skipped: lower controller is moving")
            return

        def load_worker() -> None:
            try:
                loaded, load_message = self.backend.load_calibration_snapshot(snapshot)
            except Exception as exc:
                loaded = False
                load_message = f"{type(exc).__name__}: {exc}"
            self.startup_device_check_finished.emit(
                int(epoch),
                "load",
                bool(loaded),
                str(load_message),
            )

        threading.Thread(
            target=load_worker,
            name="mmhand-startup-calib-load",
            daemon=True,
        ).start()

    def _maybe_show_calibration_assembly_warning(self, state) -> None:
        """Handle maybe show calibration assembly warning behavior for the control panel."""
        if not bool(getattr(state, "has_calibration_warning_status", False)):
            return
        mask = int(getattr(state, "calibration_warning_bitmap", 0) or 0)
        mask &= (1 << ENCODER_COUNT) - 1
        if mask == 0:
            self._calibration_warning_popup_mask = 0
            return
        if str(getattr(state, "calib_status", "IDLE") or "IDLE") != "WARN":
            return
        new_mask = mask & ~self._calibration_warning_popup_mask
        if new_mask == 0:
            return
        min_offsets = list(
            getattr(state, "calibration_auto_min_offset_counts", []) or []
        )
        results = list(getattr(state, "joint_calib_results", []) or [])
        joint_sections = []
        event_items = []
        for joint in range(ENCODER_COUNT):
            if (new_mask & (1 << joint)) == 0:
                continue
            result = results[joint] if joint < len(results) else None
            if not isinstance(result, dict):
                # SYSTEM_STATUS may arrive before this joint's snapshot.
                return

            reason = int(result.get("assemblyWarningReason", 0) or 0)
            min_offset = (
                int(min_offsets[joint]) if joint < len(min_offsets) else 0
            )
            max_shortfall = 0
            auto_span = 0
            manual_span = 0
            urdf_span = 0
            span_difference = 0
            details = []
            if bool(result.get("hasAssemblyWarningFields", False)):
                min_offset = int(result.get("autoMinOffsetCounts", 0) or 0)
                max_shortfall = int(
                    result.get("autoMaxShortfallCounts", 0) or 0
                )
                auto_span = int(result.get("assemblyAutoSpanCounts", 0) or 0)
                has_span_comparison = bool(
                    result.get("hasAssemblySpanComparisonFields", False)
                )
                manual_span = int(
                    result.get("assemblyManualSpanCounts", 0) or 0
                )
                urdf_span = int(
                    result.get("assemblyUrdfSpanCounts", 0) or 0
                )
                span_difference = int(
                    result.get("assemblySpanDifferenceCounts", 0) or 0
                )
                if reason & CALIB_WARN_AUTO_MIN_OFFSET:
                    details.append(
                        f"Auto Min 比 Manual Min 正向内缩 {min_offset} counts，"
                        "达到或超过 350 counts"
                    )
                if reason & CALIB_WARN_AUTO_MAX_SHORTFALL:
                    details.append(
                        f"Auto Max 距 Manual Max 仍差 {max_shortfall} counts，"
                        "达到或超过 300 counts"
                    )
                if reason & CALIB_WARN_URDF_MANUAL_SPAN_MISMATCH:
                    if has_span_comparison:
                        details.append(
                            f"Manual Span 为 {manual_span} counts，URDF 理论磁编 "
                            f"Span 为 {urdf_span} counts，差值 {span_difference} "
                            "counts，超过 360 counts"
                        )
                    else:
                        details.append(
                            f"旧固件报告 Auto Span 范围警告：{auto_span} counts"
                        )
            else:
                # Compatibility with the first WARN-capable firmware, which
                # only reported the Auto Min offset in SYSTEM_STATUS.
                reason = CALIB_WARN_AUTO_MIN_OFFSET
                details.append(
                    f"Auto Min 比 Manual Min 正向内缩 {min_offset} counts，"
                    "达到或超过 350 counts"
                )

            if not details:
                details.append("P4 报告装配警告，但未提供可识别的原因位")
            joint_sections.append(
                f"J{joint:02d}：\n" + "\n".join(f"- {item}" for item in details)
            )
            event_items.append(
                f"J{joint:02d}:reason=0x{reason:02X},"
                f"minOffset={min_offset},maxShortfall={max_shortfall},"
                f"autoSpan={auto_span},manualSpan={manual_span},"
                f"urdfSpan={urdf_span},spanDifference={span_difference}"
            )

        self._calibration_warning_popup_mask |= new_mask
        detail = "\n\n".join(joint_sections)
        message = (
            "标定已成功保存，但检测到可能的手指装配偏差：\n\n"
            f"{detail}\n\n"
            "该提示不属于故障，不会阻止 READY 或负反馈控制。\n"
            "建议检查对应手指的绳路、装配位置和机械端点。"
        )
        self.context.emit_action(
            f"Calibration assembly warning mask=0x{new_mask:08X}: " +
            ", ".join(event_items)
        )
        self._warn("标定装配警告", message)


    def _invalidate_local_calibration_scope(
        self, motor_mask: int, joint_mask: int
    ) -> None:
        """同步局部失效结果，不清除其他关节在 UI 中的标定和限位。"""
        motor_mask = int(motor_mask) & VALID_MOTOR_MASK
        joint_mask = int(joint_mask) & ((1 << ENCODER_COUNT) - 1)
        state = self._state
        if state is not None:
            try:
                state.motor_abs_limit_valid_bitmap = (
                    int(getattr(state, "motor_abs_limit_valid_bitmap", 0) or 0)
                    & ~motor_mask
                )
                state.motor_abs_limit_locked_bitmap = (
                    int(getattr(state, "motor_abs_limit_locked_bitmap", 0) or 0)
                    & ~motor_mask
                )
                mins = list(getattr(state, "motor_abs_limit_min", []) or [])
                maxs = list(getattr(state, "motor_abs_limit_max", []) or [])
                while len(mins) < MOTOR_COUNT:
                    mins.append(0)
                while len(maxs) < MOTOR_COUNT:
                    maxs.append(0)
                for motor_idx in range(MOTOR_COUNT):
                    if motor_mask & (1 << motor_idx):
                        mins[motor_idx] = 0
                        maxs[motor_idx] = 0
                state.motor_abs_limit_min = mins[:MOTOR_COUNT]
                state.motor_abs_limit_max = maxs[:MOTOR_COUNT]

                results = list(getattr(state, "joint_calib_results", []) or [])
                while len(results) < ENCODER_COUNT:
                    results.append(None)
                for joint_idx in range(ENCODER_COUNT):
                    if joint_mask & (1 << joint_idx):
                        results[joint_idx] = None
                state.joint_calib_results = results[:ENCODER_COUNT]
                if not any(
                    isinstance(item, dict) and bool(item.get("success", False))
                    for item in state.joint_calib_results
                ):
                    state.calib_status = "IDLE"
            except Exception:
                pass
        self._last_calib_snapshot_signature = None
        self._abs_limit_warning_unlocked_mask &= ~motor_mask
        for joint_idx in ControlScopeModel.ordered_joint_indices_from_mask(
            joint_mask
        ):
            self._refresh_joint_abs_limit_item(joint_idx, state)
            if (
                joint_idx < len(self.joint_max_items)
                and self.joint_max_items[joint_idx] is not None
            ):
                self._refresh_joint_target_range(joint_idx, state)

    def _backup_and_invalidate_calib_snapshot_scope(
        self,
        motor_mask: int,
        joint_mask: int,
        reason: str = "new_device",
    ) -> str:
        """备份 Calib Load 文件，并只删除受影响关节的数据。"""
        if not CALIB_LOAD_SNAPSHOT_PATH.exists():
            return "no Calib Load snapshot"
        reason_tag = (
            "reset_zero" if str(reason).strip().lower() == "reset_zero"
            else "new_device"
        )
        stamp = time.strftime("%Y%m%d_%H%M%S")
        archive_path = CALIB_LOAD_SNAPSHOT_PATH.with_name(
            f"{CALIB_LOAD_SNAPSHOT_PATH.stem}.before_scoped_{reason_tag}_"
            f"{stamp}{CALIB_LOAD_SNAPSHOT_PATH.suffix}"
        )
        try:
            shutil.copy2(CALIB_LOAD_SNAPSHOT_PATH, archive_path)
            payload = read_snapshot(CALIB_LOAD_SNAPSHOT_PATH)
            results = list(payload.get("joint_calib_results", []) or [])
            while len(results) < ENCODER_COUNT:
                results.append(None)
            for joint_idx in range(ENCODER_COUNT):
                if joint_mask & (1 << joint_idx):
                    results[joint_idx] = None
            payload["joint_calib_results"] = results[:ENCODER_COUNT]

            valid_mask = self._snapshot_mask_value(
                payload.get("motor_abs_limit_valid_bitmap")
            ) & ~int(motor_mask)
            locked_mask = self._snapshot_mask_value(
                payload.get("motor_abs_limit_locked_bitmap")
            ) & ~int(motor_mask)
            payload["motor_abs_limit_valid_bitmap"] = f"0x{valid_mask:08X}"
            payload["motor_abs_limit_locked_bitmap"] = f"0x{locked_mask:08X}"

            mins = list(payload.get("motor_abs_limit_min", []) or [])
            maxs = list(payload.get("motor_abs_limit_max", []) or [])
            while len(mins) < MOTOR_COUNT:
                mins.append(0)
            while len(maxs) < MOTOR_COUNT:
                maxs.append(0)
            for motor_idx in range(MOTOR_COUNT):
                if motor_mask & (1 << motor_idx):
                    mins[motor_idx] = 0
                    maxs[motor_idx] = 0
            payload["motor_abs_limit_min"] = mins[:MOTOR_COUNT]
            payload["motor_abs_limit_max"] = maxs[:MOTOR_COUNT]

            active_joint_mask = (
                self._snapshot_mask_value(payload.get("active_joint_mask"))
                & ~int(joint_mask)
            )
            active_motor_mask = (
                self._snapshot_mask_value(payload.get("active_motor_mask"))
                & ~int(motor_mask)
            )
            payload["active_joint_mask"] = f"0x{active_joint_mask:08X}"
            payload["active_motor_mask"] = f"0x{active_motor_mask:08X}"
            payload[f"scoped_{reason_tag}_invalidated_at"] = time.time()
            payload[f"scoped_{reason_tag}_motor_mask"] = (
                f"0x{int(motor_mask):08X}"
            )
            payload[f"scoped_{reason_tag}_joint_mask"] = (
                f"0x{int(joint_mask):08X}"
            )

            if active_joint_mask == 0:
                CALIB_LOAD_SNAPSHOT_PATH.unlink()
                result = (
                    f"snapshot backed up as {archive_path.name}; "
                    "no unaffected calibration remains"
                )
            else:
                atomic_write_snapshot(
                    CALIB_LOAD_SNAPSHOT_PATH,
                    payload,
                    backup=False,
                )
                result = (
                    f"snapshot backed up as {archive_path.name}; "
                    "unaffected joints preserved"
                )
        except (OSError, ValueError, TypeError, CalibrationSnapshotError) as exc:
            self.context.emit_action(f"Scoped Calib snapshot update failed: {exc}")
            return f"Calib snapshot update failed: {exc}"
        self._last_calib_snapshot_signature = None
        self.context.emit_action(result)
        return result

    def _select_single_joint_recalibration_scope(self, joint_mask: int) -> None:
        """New Device 后只勾选受影响关节，等待用户确认后点击 Calibrate。"""
        joint_mask = int(joint_mask) & ((1 << ENCODER_COUNT) - 1)
        self._selected_joint_mask = joint_mask
        self._scope_update_guard = True
        try:
            for joint_idx, check in enumerate(self.joint_use_checks):
                if check is None:
                    continue
                check.blockSignals(True)
                check.setChecked((joint_mask & (1 << joint_idx)) != 0)
                check.blockSignals(False)
        finally:
            self._scope_update_guard = False
        self._sync_motor_selection_from_joint_selection()
        self._sync_scope_group_checks()
        if getattr(self, "tabs", None) is not None and self.tabs.count() > 1:
            self.tabs.setCurrentIndex(1)
        self._apply_active_scope_for_current_mode()

    def export_calibration_snapshot(self) -> None:
        """导出完整 V5 标定包，供同一只手迁移到另一台电脑。"""
        if not CALIB_LOAD_SNAPSHOT_PATH.exists():
            self._warn("Export Calib", "没有可导出的 V5 标定快照。")
            return
        try:
            snapshot = read_snapshot(CALIB_LOAD_SNAPSHOT_PATH)
        except CalibrationSnapshotError as exc:
            self._error("Export Calib", f"当前快照无效：{exc}")
            return
        default_name = (
            f"mmhand_calibration_{time.strftime('%Y%m%d_%H%M%S')}.mmcal.json"
        )
        target, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Export MMHand Calibration",
            str(PROJECT_ROOT / default_name),
            "MMHand Calibration (*.mmcal.json);;JSON (*.json)",
        )
        if not target:
            return
        try:
            atomic_write_snapshot(Path(target), snapshot, backup=False)
        except (OSError, CalibrationSnapshotError) as exc:
            self._error("Export Calib", f"导出失败：{exc}")
            return
        self.context.emit_action(f"Calib snapshot exported: {target}")
        self._set_status(f"Calib snapshot exported: {Path(target).name}")

    def import_calibration_snapshot(self) -> None:
        """导入便携快照并激活到本机；导入本身不会发送运动指令。"""
        source, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Import MMHand Calibration",
            str(PROJECT_ROOT),
            "MMHand Calibration (*.mmcal.json);;JSON (*.json)",
        )
        if not source:
            return
        try:
            snapshot = read_snapshot(Path(source))
        except CalibrationSnapshotError as exc:
            self._error("Import Calib", f"拒绝导入：{exc}")
            return
        if str(snapshot.get("hand_profile", "")) != HAND_PROFILE:
            self._error(
                "Import Calib",
                f"hand_profile 不匹配：{snapshot.get('hand_profile')} != {HAND_PROFILE}",
            )
            return
        snapshot_error = self._validate_calibration_snapshot_for_load(snapshot)
        if snapshot_error:
            self._error("Import Calib", f"拒绝导入：{snapshot_error}")
            return
        try:
            atomic_write_snapshot(CALIB_LOAD_SNAPSHOT_PATH, snapshot)
        except (OSError, CalibrationSnapshotError) as exc:
            self._error("Import Calib", f"激活快照失败：{exc}")
            return
        self._last_calib_snapshot_signature = None
        self.context.emit_action(f"Calib snapshot imported: {source}")
        self._set_status(
            f"Calib snapshot imported: {Path(source).name}; press Calib Load to restore"
        )
        self._update_manual_action_buttons()
