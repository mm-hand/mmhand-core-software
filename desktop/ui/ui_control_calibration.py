"""ui_control_calibration.py - MMHand Desktop 模块契约。

职责：实现控制面板中的标定与恢复交互。
线程：Qt GUI 主线程执行；耗时工作必须委托后台服务，禁止阻塞界面事件循环。
输入：用户操作、HandModel 只读快照以及 HandBackend/服务返回结果。
输出：Qt 控件状态、用户提示和经门面提交的应用命令。
状态所有权：仅拥有本视图控件和瞬时交互状态；设备真值由 core 层拥有。
禁止依赖：禁止直接导入串口通信和底层控制实现，禁止在 UI 中复制协议或安全规则。
"""

from .ui_control_common import (
    AUTO_CALIBRATION_JOINT_INDICES,
    CALIB_DIAG_REASON_JOINT_STEP_FAILED,
    CALIB_DIAG_PHASE_SEQUENCE,
    ControlMode,
    ControlScopeModel,
    ENCODER_COUNT,
    FINGER_JOINT_RANGES,
    List,
    MOTOR_COUNT,
    Optional,
    Path,
    QtWidgets,
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
    _motor_channel_valid,
    directed_encoder_delta,
)
from desktop.core.calibration_diagnostics import (
    format_calibration_batch_summary,
    format_calibration_warning,
)


class CalibrationWorkflowController:
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
        # 每次标定都先把目标关节对应舵机的硬件圈数归零。四指侧摆关节还
        # 依赖其他双电机关节让出机械空间，因此在线但 Turn Unknown 的辅助
        # 双电机也必须成对归零；否则 P4 会在工作空间预检阶段安全拒绝。
        active_joint_mask = int(self._active_joint_mask_from_ui()) & (
            (1 << ENCODER_COUNT) - 1
        )
        active_motor_mask = int(self._active_motor_mask_from_ui()) & VALID_MOTOR_MASK
        servo_online_mask = int(
            getattr(self._state, "servo_online_bitmap", 0) or 0
        ) & VALID_MOTOR_MASK
        unknown_mask = int(
            getattr(self._state, "multi_turn_unknown_bitmap", 0) or 0
        ) & VALID_MOTOR_MASK
        (
            helper_joint_mask,
            helper_motor_mask,
            partial_helper_joint_mask,
            partial_helper_motor_mask,
        ) = ControlScopeModel.calibration_workspace_helper_reset_scope(
            active_joint_mask,
            servo_online_mask,
            unknown_mask,
        )
        if partial_helper_motor_mask:
            helper_text = self._format_joint_indices(
                ControlScopeModel.ordered_joint_indices_from_mask(
                    partial_helper_joint_mask
                )
            )
            motor_text = self._format_motor_mask(partial_helper_motor_mask)
            self._warn(
                "Calibration workspace unavailable",
                "Workspace helper dual-motor joint is only partially online: "
                f"{helper_text}; motors={motor_text}.\n"
                "Reconnect both motors of the helper joint before calibration.",
            )
            self._set_status(
                "Calibrate blocked: workspace helper dual-motor pair is partially online"
            )
            return

        reset_motor_mask = (active_motor_mask | helper_motor_mask) & VALID_MOTOR_MASK
        self.context.emit_action(
            "CALIB_RESET_SCOPE "
            f"selected_joints=0x{active_joint_mask:08X} "
            f"selected_motors=0x{active_motor_mask:08X}"
        )
        self.context.emit_action(
            "CALIB_RESET_HELPERS "
            f"joints=0x{helper_joint_mask:08X} "
            f"motors=0x{helper_motor_mask:08X}"
        )
        self.context.emit_action(
            f"CALIB_RESET_FINAL motors=0x{reset_motor_mask:08X}"
        )
        helper_count = int(helper_motor_mask).bit_count()
        self._set_status(
            "Calibrate: resetting selected motors"
            + (
                f" and {helper_count} workspace helper motor(s)..."
                if helper_count
                else "..."
            )
        )
        self._calibration_batch_active = True
        self._calibration_batch_alerts = []
        self._calibration_warning_popup_mask = 0
        accepted = self.backend.begin_calibration_workflow(
            joint_mask=active_joint_mask,
            motor_mask=active_motor_mask,
            reset_motor_mask=reset_motor_mask,
        )
        if not accepted:
            self._calibration_batch_active = False
            self._set_status("Calibrate blocked: another calibration transaction is running")
            return
        for joint_idx in ControlScopeModel.ordered_joint_indices_from_mask(active_joint_mask):
            if 0 <= joint_idx < len(self._joint_target_user_set):
                self._joint_target_user_set[joint_idx] = False
        self._joint_target_initialized = False
        self._set_status("Calibrate: Reset Zero and startup transaction queued")

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
            "Turn UNKNOWN is active. Multi-turn positions are untrusted, so control of the affected joints is blocked.\n\n"
            f"Affected motors: {motors}\n"
            f"Unknown mask: 0x{int(unknown_mask):08X}\n\n"
            "Why this can happen:\n"
            "- After an individual servo power cycle, the P4 multi-turn state may not match the new raw position.\n"
            "- The desktop attempts nearest-turn recovery from the last SavedRaw/logical ABS pair.\n"
            "- A manually moved mechanism, replaced servo, or changed mapping makes the old LUT unsafe.\n\n"
            "Recovery steps:\n"
            "1. Stop motion commands; use E-STOP if necessary.\n"
            "2. Check servo power, buses, and ID mapping, and stop moving the mechanism by hand.\n"
            "3. Wait for JSON nearest-turn recovery. If Turn UNKNOWN remains, select the affected joints.\n"
            "4. Press Calibrate. It invalidates the selected JSON anchors, resets hardware turn counts, and starts calibration.\n"
            "5. A dual-motor joint recalibrates both motors together; unrelated joint data is preserved.\n"
            "6. Resume control only after Multi-turn: Known, Calibration: Success, and Overload: Normal."
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
        """每次越界事件只提示一次，引导用户重新标定受影响关节。"""
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
                "Servo hardware ABS exceeded the strict turn-count safety range "
                "[-30715, +30715].\n"
                "The command hard limit remains [-30719, +30719], so this protection triggers first.\n\n"
                f"Affected motors: {motor_text}\n\n"
                "P4 stopped target output for these motors. Confirm that the mechanism and tendon routing are safe, "
                "select their joints in Joint Channels, and press Calibrate.\n"
                "Calibration clears the selected JSON anchors and hardware turn counts, invalidates the related "
                "Auto/LUT/ABS Limit data, and then builds a new calibration."
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
            f"{source or 'Command'} blocked: Turn UNKNOWN {self._format_motor_mask(unknown_mask)}; wait for JSON recovery or recalibrate affected joints"
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
        if state is None or not bool(getattr(state, "manual_reference_committed", False)):
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

    def _current_active_scope_tuple(self) -> Optional[Tuple[int, int, bool]]:
        """Handle current active scope tuple behavior for the control panel."""
        if self._app_mode == "manual":
            mode = self._manual_submode()
            if mode == "motor":
                return int(self._selected_motor_mask) & VALID_MOTOR_MASK, 0, False
            joint_mask = int(self._selected_joint_mask) & ((1 << ENCODER_COUNT) - 1)
            motor_mask = self._motor_mask_for_joint_mask(joint_mask)
            return motor_mask, joint_mask, True

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

    @staticmethod
    def _format_joint_list(indices: List[int]) -> str:
        """Format the joint list for display."""
        return ",".join(f"J{idx:02d}" for idx in indices)

    @staticmethod
    def _format_motor_list(indices: List[int]) -> str:
        """Format the motor list for display."""
        return ",".join(f"M{idx:02d}" for idx in indices)

    def load_calibration_from_file(self) -> None:
        """选择标定 JSON；验证、激活和下发统一交给核心工作流。"""
        if not self.context.is_comm_connected():
            self._warn("Load Calibration", "Connect to P4 before loading a calibration file.")
            return
        state = self._state
        system_state = getattr(state, "system_state", None) if state is not None else None
        if system_state in (
            SYSTEM_STATE_RUN_DIRECT,
            SYSTEM_STATE_RUN_FEEDBACK,
            SYSTEM_STATE_CALIBRATION_RUNNING,
            SYSTEM_STATE_ROPE_INIT_RUNNING,
        ):
            self._warn("Load Calibration", "A control or calibration task is running. Use E-STOP or Pause before loading.")
            return

        # 默认定位到自动标定生成的本机快照目录；文件对话框仍允许用户浏览任意目录。
        source, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Load MMHand Calibration",
            str(self.backend.calibration_snapshot_path.parent),
            "MMHand Calibration (*.mmcal.json);;JSON (*.json)",
        )
        if not source:
            return
        source_path = Path(source)
        try:
            snapshot = self.backend.inspect_calibration_snapshot(source_path)
        except Exception as exc:
            self._error("Load Calibration", f"Load rejected: {exc}")
            return
        reply = QtWidgets.QMessageBox.question(
            self,
            "Load Calibration",
            (
                f"Load this calibration file?\n\nFile: {source_path.name}\n\n"
                "Use it only if the mechanism, servo IDs, mapping, and calibration pose have not changed."
            ),
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply != QtWidgets.QMessageBox.Yes:
            return
        if not self.backend.begin_calibration_import(snapshot, str(source_path)):
            self._set_status("Load Calibration blocked: another calibration transaction is running")
            return
        self._set_status("Load Calibration queued...")

    def _apply_active_scope_for_current_mode(self, force_joint: bool = False) -> bool:
        """Apply the active scope for current mode selection or policy."""
        if self._app_mode == "manual" and self._manual_submode() == "device_check":
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
            if self._app_mode == "manual":
                if mode == "motor":
                    self.backend.set_control_mode(ControlMode.DIRECT_MOTOR)
                    require_can = False
                else:
                    self.backend.set_control_mode(ControlMode.JOINT_ANGLE)
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
        """请求核心工作流完成启动期 Manual、多圈和 Auto/LUT 恢复。"""
        if self._startup_device_check_running or not self.context.is_comm_connected():
            return
        self._startup_device_check_running = True
        active_joint_mask = int(self._active_joint_mask_from_ui()) & (
            (1 << ENCODER_COUNT) - 1
        )
        if not self.backend.begin_startup_calibration_restore(active_joint_mask):
            self._startup_device_check_running = False
            self._set_status(
                "Startup Calib Load blocked: another calibration transaction is running"
            )

    def _on_calibration_workflow_event(self, event) -> None:
        """在 Qt 主线程消费核心标定事务事件。"""
        phase = str(getattr(event, "phase", ""))
        ok = bool(getattr(event, "ok", False))
        message = str(getattr(event, "message", ""))
        motor_mask = int(getattr(event, "motor_mask", 0) or 0) & VALID_MOTOR_MASK
        joint_mask = int(getattr(event, "joint_mask", 0) or 0) & (
            (1 << ENCODER_COUNT) - 1
        )

        if phase == "prepare":
            if motor_mask:
                invalidated_joints = ControlScopeModel.joint_mask_for_motor_mask(
                    motor_mask
                )
                self._joint_target_initialized = False
                self._reset_motor_target_cache_for_device_epoch(
                    "Calibrate: motor turn counters reset"
                )
                self._invalidate_local_calibration_scope(
                    motor_mask, invalidated_joints
                )
            if ok:
                self._set_status(f"Calibrate started: {message}")
            else:
                self._calibration_batch_active = False
                failed_mask = int(
                    getattr(event, "failed_motor_mask", 0) or 0
                ) & VALID_MOTOR_MASK
                self._error(
                    "Calibration preparation failed",
                    f"{message}\nfailed motors: {self._format_motor_mask(failed_mask)}",
                )
                self._set_status(f"Calibrate blocked: {message}")
            return

        if phase == "phase_complete":
            details = dict(getattr(event, "details", {}) or {})
            summary = details.get("warning_summary")
            if isinstance(summary, dict):
                self._remember_calibration_batch_alert(summary, details)
            index = int(details.get("phase_index", 0)) + 1
            count = int(details.get("phase_count", 1))
            status = str(details.get("status", ""))
            suffix = "; continuing automatically" if details.get("has_next_phase") else ""
            self._set_status(
                f"Calibration phase {index}/{count} completed: {status}{suffix}"
            )
            return

        if phase == "calibration_complete":
            details = dict(getattr(event, "details", {}) or {})
            for record in details.get("records", []):
                if isinstance(record, dict) and isinstance(
                    record.get("warning_summary"), dict
                ):
                    self._remember_calibration_batch_alert(
                        record["warning_summary"], record
                    )
            was_active = self._calibration_batch_active
            self._calibration_batch_active = False
            records = list(details.get("records", []))
            records.extend(self._calibration_batch_alerts)
            self._calibration_batch_alerts = []
            if ok:
                self._set_status("Calibration completed")
                summary = format_calibration_batch_summary(records)
                if summary["active"]:
                    self._warn(summary["title"], summary["message"])
            else:
                self._set_status(f"Calibration failed: {message}")
                # Firmware calibration failures are shown by the central system-fault
                # dialog with the structured reason. Local persistence failures still
                # need a workflow-owned dialog.
                if was_active and details.get("terminal_status") != "FAILED":
                    self._error("Calibration failed", message)
            return

        if phase == "snapshot_saved":
            self._set_status(f"Calib snapshot saved: {message}")
            self.context.emit_action(
                f"Calib snapshot saved joints=0x{joint_mask:08X} "
                f"motors=0x{motor_mask:08X}: {message}"
            )
            self._update_manual_action_buttons()
            return
        if phase == "snapshot_error":
            self._set_status(f"Calib snapshot save blocked: {message}")
            self.context.emit_action(f"Calib snapshot save blocked: {message}")
            return

        if phase.startswith("startup"):
            self._startup_device_check_running = False
            if phase == "startup_no_snapshot":
                if joint_mask:
                    text = self._format_joint_indices(
                        ControlScopeModel.ordered_joint_indices_from_mask(
                            joint_mask
                        )
                    )
                    self._set_status(
                        f"Device Check: active scope Manual missing ({text})"
                    )
                    if not self._startup_missing_warning_shown:
                        self._startup_missing_warning_shown = True
                        self._warn(
                            "Manual Required",
                            "Current control set is missing persisted Manual Min/Max: "
                            + text
                            + ". Enter them in Device Check before calibration or control.",
                        )
                else:
                    self._set_status(
                        "Device Check passed: no calibration snapshot; "
                        "run automatic calibration"
                    )
                return
            if phase == "startup_manual_only":
                self._set_status(f"Startup Manual restore success: {message}")
                self.context.emit_action(
                    f"Startup Manual restore success: {message}"
                )
                return
            if ok:
                self._joint_target_initialized = False
                self._motor_target_initialized = False
                self._set_status(
                    f"Startup Calib Load success: {message}; "
                    "switch to Joint Channels to apply scope"
                )
                self.context.emit_action(
                    f"Startup Calib Load success: {message}"
                )
            else:
                self._set_status(
                    f"Startup Calib Load failed: {message}; "
                    "automatic calibration required"
                )
            return

        if phase == "import":
            if not ok:
                self._error("Load Calibration", f"Load failed: {message}")
                self._set_status(f"Load Calibration failed: {message}")
                return
            self._joint_target_initialized = False
            self._motor_target_initialized = False
            mode = self._manual_submode() if self._app_mode == "manual" else "joint"
            scope_ok = mode != "device_check" and self._apply_active_scope_for_current_mode()
            ready_recheck = False
            current_state = (
                getattr(self._state, "system_state", None)
                if self._state is not None
                else None
            )
            if (
                scope_ok
                and mode in ("joint", "test")
                and current_state
                in (
                    SYSTEM_STATE_UNCONTROLLABLE,
                    SYSTEM_STATE_IDLE,
                    SYSTEM_STATE_READY,
                    SYSTEM_STATE_RECOVERY_REQUIRED,
                )
            ):
                self.backend.start()
                ready_recheck = True
            if mode == "device_check":
                suffix = "; switch to Joint Channels to apply active scope"
            elif scope_ok:
                suffix = "; active scope applied"
                if ready_recheck:
                    suffix += "; READY recheck requested"
            else:
                suffix = "; active scope not applied"
            self._set_status(f"Load Calibration success: {message}{suffix}")
            source = str(getattr(event, "details", {}).get("source", ""))
            self.context.emit_action(
                f"Calibration file loaded: {source}; {message}{suffix}"
            )
            self._update_manual_action_buttons()

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
        summary = format_calibration_warning(state, new_mask)
        if not summary.get("ready"):
            return
        self._calibration_warning_popup_mask |= new_mask
        self.context.emit_action(
            f"Calibration assembly warning mask=0x{new_mask:08X}: " +
            str(summary.get("event_detail", ""))
        )
        if bool(getattr(self, "_calibration_batch_active", False)):
            self._remember_calibration_batch_alert(summary)
            return
        self._warn(summary["title"], summary["message"])

    def _remember_calibration_batch_alert(self, summary: dict, details=None) -> None:
        record = dict(details or {})
        record["warning_summary"] = dict(summary)
        key = str(summary.get("event_detail") or summary.get("message") or "")
        if not key:
            return
        for item in self._calibration_batch_alerts:
            item_summary = item.get("warning_summary", {})
            item_key = str(
                item_summary.get("event_detail") or item_summary.get("message") or ""
            )
            if item_key == key:
                if details:
                    item.update(record)
                return
        self._calibration_batch_alerts.append(record)


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
        for joint_idx in ControlScopeModel.ordered_joint_indices_from_mask(
            joint_mask
        ):
            self._refresh_joint_abs_limit_item(joint_idx, state)
            if (
                joint_idx < len(self.joint_max_items)
                and self.joint_max_items[joint_idx] is not None
            ):
                self._refresh_joint_target_range(joint_idx, state)
