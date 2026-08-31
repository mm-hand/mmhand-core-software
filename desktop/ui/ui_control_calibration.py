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
    MOTOR_TO_JOINT_INDEX,
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
    TENDON_LUT_MAX_SAMPLES,
    Tuple,
    VALID_MOTOR_MASK,
    _motor_channel_valid,
    atomic_write_snapshot,
    directed_encoder_delta,
    invalidate_calibration_snapshot_scope,
    json,
    manual_encoder_reference_entry_crc,
    merge_snapshot,
    read_snapshot,
    snapshot_write_transaction,
    shutil,
    tendon_lut_field,
    threading,
    time,
    urdf_manual_scaled_mapping_metadata,
    validate_snapshot,
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
        ok, message, reset_mask, failed_mask = (
            self.backend.prepare_calibration_turn_zero(reset_motor_mask)
        )
        if reset_mask:
            # 事务可能出现部分成功；只失效真正改变过硬件 turn0 的舵机及其
            # 关节，既不能漏掉辅助电机，也不能误删未执行复位的标定结果。
            invalidated_joint_mask = ControlScopeModel.joint_mask_for_motor_mask(
                reset_mask
            )
            self._joint_target_initialized = False
            self._reset_motor_target_cache_for_device_epoch(
                "Calibrate: motor turn counters reset"
            )
            self._invalidate_local_calibration_scope(
                reset_mask, invalidated_joint_mask
            )
            self._backup_and_invalidate_calib_snapshot_scope(
                reset_mask,
                invalidated_joint_mask,
                reason="calibration_turn_zero",
            )
        if not ok:
            self._error(
                "Calibration turn reset failed",
                f"{message}\nfailed motors: {self._format_motor_mask(failed_mask)}",
            )
            self._set_status(f"Calibrate blocked: {message}")
            return
        # 维护 RESET 会释放 owner；重新提交同一 active scope 后再启动标定。
        if not self._apply_active_scope_for_current_mode():
            self._set_status("Calibrate blocked: active scope reapply failed after turn reset")
            return
        self._start_calibration_now()

    def _start_calibration_now(self) -> None:
        """Handle start calibration now behavior for the control panel."""
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
            "当前存在 Turn UNKNOWN，多圈位置不可信，系统已阻止相关关节继续控制。\n\n"
            f"受影响电机：{motors}\n"
            f"unknown mask：0x{int(unknown_mask):08X}\n\n"
            "为什么会这样：\n"
            "- 舵机单独断电后，P4 保存的多圈状态可能与重新上电后的 raw 不匹配。\n"
            "- 上位机会用最近一次 SavedRaw/logical ABS 展开到当前 raw 最近的圈数。\n"
            "- 机构被手动转动、换舵机或改映射后，旧 LUT 也不能继续使用。\n\n"
            "恢复步骤：\n"
            "1. 停止发送运动命令，必要时点击 E-STOP。\n"
            "2. 确认舵机电源、总线和 ID 映射正常，机构不再被手动转动。\n"
            "3. 先等待上位机 JSON 最近圈恢复；若 Turn UNKNOWN 仍未清除，勾选受影响关节。\n"
            "4. 点击 Calibrate；程序会先清除所选舵机 JSON 锚点并下发硬件圈数复位，再开始标定。\n"
            "5. 双电机关节会由两台舵机共同重新标定 LUT，其他关节数据保留。\n"
            "6. 确认 Multi-turn: Known、Calibration: Success、Overload: Normal 后再恢复控制。"
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
                "检测到舵机内部硬件 ABS 严格越过圈数安全阈值 "
                "[-30715, +30715]。\n"
                "底层命令硬边界仍为 [-30719, +30719]，当前保护会提前触发。\n\n"
                f"越界舵机：{motor_text}\n\n"
                "P4 已停止这些舵机的目标输出。请确认机构和绳路安全，"
                "在 Joint Channels 勾选越界舵机所属关节并点击 Calibrate。\n"
                "Calibrate 会先清除所选舵机的 JSON 锚点和硬件圈数，"
                "同时失效对应 Auto/LUT/ABS Limit，再建立新标定。"
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
        completed_mask = int(
            getattr(state, "calib_diag_completed_joint_mask", 0) or 0
        )
        failed_mask = int(getattr(state, "calib_diag_failed_joint_mask", 0) or 0)
        if completed_mask or failed_mask:
            valid_joint_mask = (1 << ENCODER_COUNT) - 1
            return completed_mask & valid_joint_mask & ~failed_mask
        return 0

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
        incompatible_v4 = [
            idx for idx in required_joints
            if int(joint_results[idx].get("mappingVersion", 0) or 0) != 4 or
            int(joint_results[idx].get("manualReferenceCrc", 0) or 0)
            != manual_encoder_reference_entry_crc(
                idx,
                manual_min_values[idx],
                manual_max_values[idx],
            )
        ]
        if incompatible_v4:
            self._set_status(
                "Calib snapshot save blocked: v4/Manual CRC mismatch " +
                self._format_joint_list(incompatible_v4)
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

        try:
            # 标定结果和后台多圈锚点都会修改同一 JSON；锁必须覆盖读取、合并和替换，
            # 单独原子替换无法阻止旧读取结果覆盖刚写入的锚点位。
            with snapshot_write_transaction():
                previous_payload = (
                    read_snapshot(CALIB_LOAD_SNAPSHOT_PATH)
                    if CALIB_LOAD_SNAPSHOT_PATH.exists()
                    else None
                )
                payload = merge_snapshot(
                    previous_payload,
                    payload,
                    updated_joint_mask=active_joint_mask,
                )
                signature = json.dumps(
                    {
                        "active_joint_mask": payload["active_joint_mask"],
                        "active_motor_mask": payload["active_motor_mask"],
                        "valid": payload["motor_abs_limit_valid_bitmap"],
                        "mins": payload["motor_abs_limit_min"],
                        "maxs": payload["motor_abs_limit_max"],
                        "joints": payload["joint_calib_results"],
                    },
                    sort_keys=True,
                    ensure_ascii=False,
                )
                snapshot_changed = signature != self._last_calib_snapshot_signature
                if snapshot_changed:
                    payload = atomic_write_snapshot(CALIB_LOAD_SNAPSHOT_PATH, payload)
        except CalibrationSnapshotError as exc:
            self._set_status(f"Calib snapshot save blocked: {exc}")
            return
        except OSError as exc:
            self._set_status(f"Calib snapshot save failed: {exc}")
            return

        if not snapshot_changed:
            if calib_status in {"SUCCESS", "WARN"}:
                self._pending_calib_joint_mask = 0
                self._pending_calib_motor_mask = 0
                self._pending_calib_result_baselines = {}
            return
        request_anchor_capture = getattr(
            self.backend, "request_multiturn_anchor_capture", None
        )
        if callable(request_anchor_capture):
            request_anchor_capture(active_motor_mask)
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


    def load_calibration_from_file(self) -> None:
        """选择标定 JSON，校验后激活为本机快照并立即恢复到 P4。"""
        if not self.context.is_comm_connected():
            self._warn("Load Calibration", "请先连接 P4，再加载标定文件。")
            return
        state = self._state
        system_state = getattr(state, "system_state", None) if state is not None else None
        if system_state in (
            SYSTEM_STATE_RUN_DIRECT,
            SYSTEM_STATE_RUN_FEEDBACK,
            SYSTEM_STATE_CALIBRATION_RUNNING,
            SYSTEM_STATE_ROPE_INIT_RUNNING,
        ):
            self._warn("Load Calibration", "控制或标定任务正在运行，请先 E-STOP 或 Pause 后再加载。")
            return

        # 默认定位到自动标定生成的本机快照目录；文件对话框仍允许用户浏览任意目录。
        source, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Load MMHand Calibration",
            str(CALIB_LOAD_SNAPSHOT_PATH.parent),
            "MMHand Calibration (*.mmcal.json);;JSON (*.json)",
        )
        if not source:
            return
        source_path = Path(source)
        try:
            snapshot = read_snapshot(source_path)
        except CalibrationSnapshotError as exc:
            self._error("Load Calibration", f"拒绝加载：{exc}")
            return
        if str(snapshot.get("hand_profile", "")) != HAND_PROFILE:
            self._error(
                "Load Calibration",
                f"标定快照 hand_profile 不一致：{snapshot.get('hand_profile')} != {HAND_PROFILE}",
            )
            return
        snapshot_error = self._validate_calibration_snapshot_for_load(snapshot)
        if snapshot_error:
            self._error("Load Calibration", snapshot_error)
            self._set_status(f"Load Calibration blocked: {snapshot_error}")
            return
        reply = QtWidgets.QMessageBox.question(
            self,
            "Load Calibration",
            (
                f"确认加载标定文件？\n\n文件：{source_path.name}\n\n"
                "仅当机械、舵机 ID、映射和标定姿态没有变化时才应使用。"
            ),
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply != QtWidgets.QMessageBox.Yes:
            return
        # 仅在文件已通过完整校验且用户确认后才替换本机快照，避免错误 JSON
        # 覆盖自动标定的有效结果。atomic_write_snapshot 会保留可恢复备份。
        try:
            with snapshot_write_transaction():
                atomic_write_snapshot(CALIB_LOAD_SNAPSHOT_PATH, snapshot)
        except (OSError, CalibrationSnapshotError) as exc:
            self._error("Load Calibration", f"激活本机快照失败：{exc}")
            return
        self._last_calib_snapshot_signature = None
        self._set_status("Load Calibration sending snapshot to P4...")
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
            self._set_status(f"Load Calibration success: {message}{suffix}")
            self.context.emit_action(
                f"Calibration file loaded: {source_path}; {message}{suffix}"
            )
        else:
            self._error("Load Calibration", f"加载失败：{message}")
            self._set_status(f"Load Calibration failed: {message}")
        self._update_manual_action_buttons()

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

        if phase == "recovery":
            self._startup_device_check_running = False
            self._set_status(
                f"Startup multi-turn recovery incomplete: {message}; automatic calibration required"
            )
            self.context.emit_action(f"Startup multi-turn recovery incomplete: {message}")
            return

        if phase == "manual":
            self._startup_device_check_running = False
            if ok:
                self._set_status(f"Startup Manual restore success: {message}")
                self.context.emit_action(f"Startup Manual restore success: {message}")
            else:
                self._set_status(f"Startup Manual restore failed: {message}")
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

        if not CALIB_LOAD_SNAPSHOT_PATH.exists():
            self._startup_device_check_running = False
            active_joint_mask = int(self._active_joint_mask_from_ui()) & (
                (1 << ENCODER_COUNT) - 1
            )
            manual_mask = int(
                getattr(state, "manual_reference_valid_bitmap", 0) or 0
            ) if state is not None else 0
            committed = bool(
                getattr(state, "manual_reference_committed", False)
            ) if state is not None else False
            missing_mask = (
                active_joint_mask
                if not committed
                else active_joint_mask & ~manual_mask
            )
            if missing_mask:
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
            else:
                self._set_status(
                    "Device Check passed: no calibration snapshot; run automatic calibration"
                )
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
        system_state = getattr(state, "system_state", None) if state is not None else None
        if system_state in (SYSTEM_STATE_RUN_DIRECT, SYSTEM_STATE_RUN_FEEDBACK, SYSTEM_STATE_CALIBRATION_RUNNING):
            self._startup_device_check_running = False
            self._set_status("Startup Calib Load skipped: lower controller is moving")
            return

        def load_worker() -> None:
            # P4 断电后 Manual RAM 必为空。先恢复 Manual，再校验和加载依赖它
            # 的 Auto/LUT；一次瞬时 ACK 丢失允许重试，但每次都必须完成读回。
            manual_ok = False
            manual_message = "Manual restore was not attempted"
            for attempt in range(2):
                try:
                    manual_ok, manual_message = (
                        self.backend.restore_manual_references_from_snapshot(snapshot)
                    )
                except Exception as exc:
                    manual_ok = False
                    manual_message = f"{type(exc).__name__}: {exc}"
                if manual_ok:
                    break
                if attempt == 0:
                    time.sleep(0.20)
            if not manual_ok:
                self.startup_device_check_finished.emit(
                    int(epoch), "manual", False, str(manual_message)
                )
                return

            try:
                active_motor_mask = int(
                    str(snapshot.get("active_motor_mask", "0")), 0
                ) & VALID_MOTOR_MASK
            except (TypeError, ValueError):
                active_motor_mask = 0
            try:
                recovery_ok, recovery_message = self.backend.wait_for_multiturn_recovery(
                    active_motor_mask
                )
            except Exception as exc:
                recovery_ok = False
                recovery_message = f"{type(exc).__name__}: {exc}"
            if not recovery_ok:
                self.startup_device_check_finished.emit(
                    int(epoch), "recovery", False, str(recovery_message)
                )
                return

            snapshot_error = self._validate_calibration_snapshot_for_load(snapshot)
            if snapshot_error:
                self.startup_device_check_finished.emit(
                    int(epoch),
                    "manual",
                    True,
                    f"{manual_message}; Auto/LUT skipped: {snapshot_error}",
                )
                return
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
        reason: str = "calibration_turn_zero",
    ) -> str:
        """调用核心快照事务，失效 Auto/LUT 但始终保留 Manual 端点。"""
        try:
            result = invalidate_calibration_snapshot_scope(
                CALIB_LOAD_SNAPSHOT_PATH,
                motor_mask=motor_mask,
                joint_mask=joint_mask,
                reason=reason,
            )
        except (OSError, ValueError, TypeError, CalibrationSnapshotError) as exc:
            self.context.emit_action(f"Scoped Calib snapshot update failed: {exc}")
            return f"Calib snapshot update failed: {exc}"
        self._last_calib_snapshot_signature = None
        self.context.emit_action(result)
        return result
