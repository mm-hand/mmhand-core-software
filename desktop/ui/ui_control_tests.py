"""ui_control_tests.py - MMHand Desktop 模块契约。

职责：实现测试模式的阶跃、扫频、前馈和停止操作。
线程：Qt GUI 主线程执行；耗时工作必须委托后台服务，禁止阻塞界面事件循环。
输入：用户操作、HandModel 只读快照以及 HandBackend/服务返回结果。
输出：Qt 控件状态、用户提示和经门面提交的应用命令。
状态所有权：仅拥有本视图控件和瞬时交互状态；设备真值由 core 层拥有。
禁止依赖：禁止直接导入串口通信和底层控制实现，禁止在 UI 中复制协议或安全规则。
"""

from .ui_control_common import (
    ENCODER_COUNT,
    List,
    Optional,
    QtWidgets,
    TEST_ROW_COUNT,
    Tuple,
    math,
    np,
    time,
)


class ManualTestPanelMixin:
    def _normalize_test_joint_value(self, text: str) -> str:
        """Handle normalize test joint value behavior for the control panel."""
        text = str(text or "").strip()
        if text.lower() == "none" or not text:
            return "None"
        if text.upper().startswith("J"):
            text = text[1:]
        try:
            idx = int(text)
        except ValueError:
            return "None"
        return f"J{idx:02d}" if 0 <= idx < ENCODER_COUNT else "None"

    def _joint_index_from_combo(self, combo: QtWidgets.QComboBox) -> Optional[int]:
        """Compute or update joint index from combo data."""
        text = self._normalize_test_joint_value(combo.currentText())
        combo.setCurrentText(text)
        if text == "None":
            return None
        return int(text[1:])

    def _on_test_joint_changed(self, row_idx: int) -> None:
        """Handle the test joint changed event."""
        current = self._normalize_test_joint_value(self.test_joint_combos[row_idx].currentText())
        self.test_joint_combos[row_idx].setCurrentText(current)
        if current == "None":
            return
        for i, combo in enumerate(self.test_joint_combos):
            if i != row_idx and self._normalize_test_joint_value(combo.currentText()) == current:
                self.test_joint_combos[row_idx].setCurrentText("None")
                self._set_status(f"Test config duplicate joint: {current} already selected")
                return
        joint_idx = int(current[1:])
        workspace = self._joint_workspace_range(joint_idx, self._state)
        lo = workspace.current_min_deg if workspace.ready else self._joint_default_target_min_deg(joint_idx)
        hi = self._joint_target_max_deg(joint_idx)
        self._set_spin_range_if_changed(self.test_min_spins[row_idx], lo, hi)
        self._set_spin_range_if_changed(self.test_max_spins[row_idx], lo, hi)

    def _refresh_feedforward_test_workspace(self) -> None:
        """Refresh the feedforward test joint selector."""
        combo = getattr(self, "ff_joint_combo", None)
        if combo is None:
            return
        self._joint_index_from_combo(combo)

    def _refresh_test_workspace_ranges(self) -> None:
        for row_idx in range(len(getattr(self, "test_joint_combos", []))):
            self._on_test_joint_changed(row_idx)
        self._refresh_single_test_workspace("feedforward")
        self._refresh_single_test_workspace("triangle")

    def _test_workspace_ready(self, joint_idx: int, title: str, show_message: bool) -> bool:
        """Run or update the workspace ready test workflow."""
        workspace = self._joint_workspace_range(joint_idx, self._state)
        if workspace.ready:
            return True
        if show_message:
            self._warn(title, f"J{joint_idx:02d} workspace unavailable: {workspace.reason}")
        return False

    def _validate_test_rows(self, show_message: bool = True) -> Optional[List[Tuple[int, float, float, float, float]]]:
        """Validate the test rows input."""
        active = []
        seen = set()
        for row in range(TEST_ROW_COUNT):
            joint_idx = self._joint_index_from_combo(self.test_joint_combos[row])
            if joint_idx is None:
                continue
            if joint_idx in seen:
                if show_message:
                    self._warn("Test Config", f"Test row {row + 1}: duplicated J{joint_idx:02d}")
                return None
            seen.add(joint_idx)
            if not self._test_workspace_ready(joint_idx, "Test Config", show_message):
                return None
            lo = self._clamp_joint_target_deg(joint_idx, self.test_min_spins[row].value())
            hi = self._clamp_joint_target_deg(joint_idx, self.test_max_spins[row].value())
            if hi < lo:
                lo, hi = hi, lo
            freq = float(self.test_freq_spins[row].value())
            phase = float(self.test_phase_spins[row].value())
            if not (np.isfinite(freq) and freq > 0.0 and np.isfinite(phase) and 0.0 <= phase <= 360.0):
                if show_message:
                    self._warn("Test Config", f"Test row {row + 1}: invalid frequency or phase")
                return None
            active.append((joint_idx, lo, hi, freq, phase))
        if not active and show_message:
            self._warn("Test Config", "Select at least one valid joint (not None).")
        return active or None

    def _set_test_button_states(self) -> None:
        any_running = (
            self._test_running
            or self._ff_test_running
            or self._tri_test_running
        )
        controller_ready = self._controller_interaction_ready()
        for btn, running in [
            (getattr(self, "test_stop_btn", None), self._test_running),
            (getattr(self, "ff_stop_btn", None), self._ff_test_running),
            (getattr(self, "tri_stop_btn", None), self._tri_test_running),
        ]:
            if btn is not None:
                try:
                    btn.setEnabled(running)
                except RuntimeError:
                    pass
        for btn, running in [
            (getattr(self, "test_start_btn", None), self._test_running),
            (getattr(self, "ff_start_btn", None), self._ff_test_running),
            (getattr(self, "tri_start_btn", None), self._tri_test_running),
        ]:
            if btn is not None:
                try:
                    btn.setEnabled(controller_ready and not any_running and not running)
                except RuntimeError:
                    pass

    def _start_test_recording(self, reason: str) -> bool:
        """Handle start test recording behavior for the control panel."""
        try:
            self.context.start_recording_for_test(reason)
            return True
        except Exception as exc:
            self._error("Start Test Failed", f"Cannot start recording: {exc}")
            return False

    def _on_start_test(self) -> None:
        """Handle the start test event."""
        if not self._controller_interaction_ready():
            self._set_status("Start Test blocked: press ENABLE and wait for P4 to become controllable")
            return
        if self._manual_submode() != "test" and hasattr(self, "tabs"):
            self.tabs.setCurrentWidget(self.test_panel)
        active = self._validate_test_rows(True)
        if not active:
            return
        self.stop_all_tests("restarted by sine test", update_status=False)
        if not self._guard_joint_command_started():
            return
        if not self._start_test_recording("test"):
            return
        self._test_active_rows = active
        self._test_hold_s = max(0.0, float(self.test_hold_spin.value()))
        self._test_running = True
        self._test_t0 = time.time()
        self._set_test_button_states()
        self.context.emit_action("Test started")
        self._test_tick()
        self._test_timer.start()

    def _on_stop_test(self, reason: str = "stopped by test", update_status: bool = True) -> None:
        """Handle the stop test event."""
        was_running = self._test_running
        self._test_running = False
        self._test_timer.stop()
        self._test_active_rows = []
        self._set_test_button_states()
        if was_running:
            self.context.stop_recording_for_test(reason)
            if update_status:
                self.context.emit_action(f"Test stopped ({reason})")

    def _parse_ff_targets_text(self, raw: str) -> Optional[List[float]]:
        """Parse the ff targets text input."""
        targets = []
        for token in str(raw or "").replace(",", " ").split():
            try:
                targets.append(float(token))
            except ValueError:
                continue
        return targets or None

    def _validate_ff_test_config(self, show_message: bool = True) -> Optional[Tuple[int, List[float], float]]:
        """Validate the ff test config input."""
        joint_idx = self._joint_index_from_combo(self.ff_joint_combo)
        if joint_idx is None:
            if show_message:
                self._warn("Feedforward Test", "Select one valid joint.")
            return None
        if not self._test_workspace_ready(joint_idx, "Feedforward Test", show_message):
            return None
        targets = self._parse_ff_targets_text(self.ff_targets_edit.text())
        if not targets:
            if show_message:
                self._warn("Feedforward Test", "Targets(deg) must contain at least one valid number.")
            return None
        clamped = [self._clamp_joint_target_deg(joint_idx, value) for value in targets]
        step_s = float(self.ff_step_spin.value())
        if not np.isfinite(step_s) or step_s <= 0:
            if show_message:
                self._warn("Feedforward Test", "Step Time(s) must be finite and > 0.")
            return None
        self.ff_targets_edit.setText(", ".join(f"{v:.2f}" for v in clamped))
        return joint_idx, clamped, step_s

    def _on_start_ff_test(self) -> None:
        """Handle the start ff test event."""
        if not self._controller_interaction_ready():
            self._set_status("Feedforward Test blocked: press ENABLE and wait for P4 to become controllable")
            return
        if self._manual_submode() != "test" and hasattr(self, "tabs"):
            self.tabs.setCurrentWidget(self.test_panel)
        validated = self._validate_ff_test_config(True)
        if validated is None:
            return
        self.stop_all_tests("restarted by feedforward test", update_status=False)
        if not self._guard_joint_command_started():
            return
        if not self._start_test_recording("feedforward test"):
            return
        self._ff_test_joint_idx, self._ff_test_targets, self._ff_test_step_s = validated
        self._ff_test_idx = 0
        self._ff_test_t_accum = 0.0
        self._ff_test_last_ts = time.time()
        self._ff_test_running = True
        self._set_test_button_states()
        self.context.emit_action(f"Feedforward Test started on J{self._ff_test_joint_idx:02d}")
        self._ff_test_tick()
        self._ff_test_timer.start()

    def _on_stop_ff_test(self, reason: str = "stopped by feedforward test", update_status: bool = True) -> None:
        """Handle the stop ff test event."""
        was_running = self._ff_test_running
        self._ff_test_running = False
        self._ff_test_timer.stop()
        self._ff_test_targets = []
        self._ff_test_idx = 0
        self._ff_test_t_accum = 0.0
        self._ff_test_last_ts = 0.0
        self._set_test_button_states()
        if was_running:
            self.context.stop_recording_for_test(reason)
            if update_status:
                self.context.emit_action(f"Feedforward Test stopped ({reason})")

    def _test_tick(self) -> None:
        """Run or update the tick test workflow."""
        if not self._test_running:
            return
        if self._manual_submode() != "test":
            self._on_stop_test("stopped on mode switch", update_status=False)
            return
        if not self.backend.is_started():
            self._on_stop_test("stopped: P4 controller is no longer controllable")
            return
        elapsed = max(0.0, time.time() - self._test_t0)
        angles = self.resolved_joint_targets()
        for joint_idx, lo, hi, freq, phase_deg in self._test_active_rows:
            center = 0.5 * (lo + hi)
            amp = 0.5 * (hi - lo)
            phase = math.radians(phase_deg)
            if elapsed < self._test_hold_s:
                target = center + amp * math.sin(phase)
            else:
                target = center + amp * math.sin(2.0 * math.pi * freq * (elapsed - self._test_hold_s) + phase)
            angles[joint_idx] = self._clamp_joint_target_deg(joint_idx, target)
            self._set_joint_target_widget(joint_idx, angles[joint_idx])
            self._joint_target_user_set[joint_idx] = True
        try:
            self.context.send_joint_targets(angles, source="Test", live=True)
        except Exception as exc:
            self._on_stop_test(f"stopped by send error: {exc}")

    def _ff_test_tick(self) -> None:
        """Advance the feedforward test timer workflow."""
        if not self._ff_test_running:
            return
        if self._manual_submode() != "test":
            self._on_stop_ff_test("stopped on mode switch", update_status=False)
            return
        if not self.backend.is_started():
            self._on_stop_ff_test("stopped: P4 controller is no longer controllable")
            return
        if not self._ff_test_targets:
            self._on_stop_ff_test("stopped: no feedforward targets")
            return
        now = time.time()
        dt = 0.0 if self._ff_test_last_ts <= 0.0 else max(0.0, now - self._ff_test_last_ts)
        self._ff_test_last_ts = now
        self._ff_test_t_accum += dt
        while self._ff_test_t_accum >= self._ff_test_step_s and self._ff_test_idx < len(self._ff_test_targets) - 1:
            self._ff_test_t_accum -= self._ff_test_step_s
            self._ff_test_idx += 1
        target = self._clamp_joint_target_deg(self._ff_test_joint_idx, self._ff_test_targets[self._ff_test_idx])
        angles = self.resolved_joint_targets()
        angles[self._ff_test_joint_idx] = target
        self._set_joint_target_widget(self._ff_test_joint_idx, target)
        self._joint_target_user_set[self._ff_test_joint_idx] = True
        try:
            self.context.send_joint_targets(angles, source="Feedforward Test", live=True)
        except Exception as exc:
            self._on_stop_ff_test(f"stopped by send error: {exc}")

    def _set_joint_target_widget(self, idx: int, value: float) -> None:
        """Update the joint target widget value or state."""
        self._updating = True
        try:
            self.joint_target_spins[idx].setValue(float(value))
            self.joint_sliders[idx].setValue(int(round(float(value) * 100.0)))
        finally:
            self._updating = False


    def _refresh_single_test_workspace(self, test_kind: str) -> None:
        if test_kind == "triangle":
            combo = getattr(self, "tri_joint_combo", None)
            min_spin = getattr(self, "tri_min_spin", None)
            max_spin = getattr(self, "tri_max_spin", None)
        else:
            combo = getattr(self, "ff_joint_combo", None)
            min_spin = None
            max_spin = None
        if combo is None:
            return
        joint_idx = self._joint_index_from_combo(combo)
        if joint_idx is None:
            return
        workspace = self._joint_workspace_range(joint_idx, self._state)
        if min_spin is not None and max_spin is not None:
            lo = workspace.current_min_deg if workspace.ready else self._joint_default_target_min_deg(joint_idx)
            hi = self._joint_target_max_deg(joint_idx)
            self._set_spin_range_if_changed(min_spin, lo, hi)
            self._set_spin_range_if_changed(max_spin, lo, hi)

    def _validate_tri_test_config(self, show_message: bool = True) -> Optional[Tuple[int, float, float, float]]:
        joint_idx = self._joint_index_from_combo(self.tri_joint_combo)
        if joint_idx is None:
            if show_message:
                self._warn("Triangle Wave Test", "Select one valid joint.")
            return None
        if not self._test_workspace_ready(joint_idx, "Triangle Wave Test", show_message):
            return None
        lo = self._clamp_joint_target_deg(joint_idx, self.tri_min_spin.value())
        hi = self._clamp_joint_target_deg(joint_idx, self.tri_max_spin.value())
        if hi < lo:
            lo, hi = hi, lo
        freq = float(self.tri_freq_spin.value())
        if not np.isfinite(freq) or freq <= 0:
            if show_message:
                self._warn("Triangle Wave Test", "Frequency must be finite and > 0.")
            return None
        return joint_idx, lo, hi, freq

    def _on_start_tri_test(self) -> None:
        if not self._controller_interaction_ready():
            self._set_status("Triangle Wave Test blocked: press ENABLE and wait for P4 to become controllable")
            return
        if self._manual_submode() != "test" and hasattr(self, "tabs"):
            self.tabs.setCurrentWidget(self.test_panel)
        validated = self._validate_tri_test_config(True)
        if validated is None:
            return
        self.stop_all_tests("restarted by triangle test", update_status=False)
        if not self._guard_joint_command_started():
            return
        if not self._start_test_recording("triangle test"):
            return
        self._tri_test_joint_idx, self._tri_test_min_deg, self._tri_test_max_deg, self._tri_test_freq_hz = validated
        self._tri_test_t0 = time.time()
        self._tri_test_running = True
        self._set_test_button_states()
        self.context.emit_action(f"Triangle Wave Test started on J{self._tri_test_joint_idx:02d}")
        self._tri_test_tick()
        self._tri_test_timer.start()

    def _on_stop_tri_test(self, reason: str = "stopped by triangle test", update_status: bool = True) -> None:
        was_running = self._tri_test_running
        self._tri_test_running = False
        self._tri_test_timer.stop()
        self._tri_test_t0 = 0.0
        self._set_test_button_states()
        if was_running:
            self.context.stop_recording_for_test(reason)
            if update_status:
                self.context.emit_action(f"Triangle Wave Test stopped ({reason})")

    def _tri_test_tick(self) -> None:
        if not self._tri_test_running:
            return
        if self._manual_submode() != "test":
            self._on_stop_tri_test("stopped on mode switch", update_status=False)
            return
        if not self.backend.is_started():
            self._on_stop_tri_test("stopped: P4 controller is no longer controllable")
            return
        elapsed = max(0.0, time.time() - self._tri_test_t0)
        phase = (elapsed * self._tri_test_freq_hz) % 1.0
        tri = 4.0 * abs(phase - 0.5) - 1.0
        center = 0.5 * (self._tri_test_min_deg + self._tri_test_max_deg)
        amp = 0.5 * (self._tri_test_max_deg - self._tri_test_min_deg)
        target = self._clamp_joint_target_deg(self._tri_test_joint_idx, center + amp * tri)
        angles = self.resolved_joint_targets()
        angles[self._tri_test_joint_idx] = target
        self._set_joint_target_widget(self._tri_test_joint_idx, target)
        self._joint_target_user_set[self._tri_test_joint_idx] = True
        try:
            self.context.send_joint_targets(angles, source="Triangle Wave Test", live=True)
        except Exception as exc:
            self._on_stop_tri_test(f"stopped by send error: {exc}")
