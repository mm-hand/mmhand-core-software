"""ControlPanelViewMixin methods for the modular control panel."""

from .ui_control_common import (
    ControlScopeModel,
    ENCODER_COUNT,
    HAND_PROFILE,
    List,
    MOTOR_COUNT,
    Optional,
    QtCore,
    QtGui,
    QtWidgets,
    Tuple,
    VALID_MOTOR_MASK,
    _motor_channel_valid,
)
from .ui_control_device import DeviceControlBar
from .ui_control_tables import JointChannelTable, MotorChannelTable
from .ui_control_test_panel import ManualTestPanel
from .ui_control_modes import ManualControlView, TeleopControlView
from .ui_dialogs import show_scrollable_message


class ControlPanelViewMixin:
    def _build(self) -> None:
        """Handle build behavior for the control panel."""
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)

        device = DeviceControlBar(self)
        self.enable_btn = device.enable_btn
        self.stop_btn = device.stop_btn
        self.pause_btn = device.pause_btn
        self.raw_ik_enable_btn = device.raw_ik_enable_btn
        self.reset_btn = device.reset_btn
        self.calib_btn = device.calib_btn
        self.rope_init_btn = device.rope_init_btn
        self.nvs_control_btn = device.nvs_control_btn
        layout.addWidget(device)
        self._update_connection_lights()
        device.connect_panel(self)
        self.body_container = QtWidgets.QWidget()
        self.body_layout = QtWidgets.QVBoxLayout(self.body_container)
        self.body_layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.body_container, 1)
        self._build_manual_body()

    @staticmethod
    def _scope_range(start: int, end: int, require_can: bool = True) -> Tuple[int, int, bool]:
        """Handle scope range behavior for the control panel."""
        return ControlScopeModel.scope_range(start, end, require_can)

    @staticmethod
    def _motor_mask_for_joint_mask(joint_mask: int) -> int:
        """Compute or update motor mask for joint mask data."""
        return ControlScopeModel.motor_mask_for_joint_mask(joint_mask)

    @staticmethod
    def _centered_cell_widget(widget: QtWidgets.QWidget) -> QtWidgets.QWidget:
        """Handle centered cell widget behavior for the control panel."""
        cell = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(cell)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addStretch(1)
        layout.addWidget(widget)
        layout.addStretch(1)
        return cell

    def _make_use_checkbox_cell(
        self,
        checked: bool,
        tooltip: str,
    ) -> Tuple[QtWidgets.QWidget, QtWidgets.QCheckBox]:
        """Create the use checkbox cell widget or helper."""
        check = QtWidgets.QCheckBox()
        check.setChecked(bool(checked))
        check.setToolTip(tooltip)
        return self._centered_cell_widget(check), check

    def _make_group_use_widget(
        self,
        text: str,
        checked: bool,
        tooltip: str,
    ) -> Tuple[QtWidgets.QWidget, QtWidgets.QCheckBox]:
        """Create the group use widget widget or helper."""
        check = QtWidgets.QCheckBox(text)
        check.setChecked(bool(checked))
        check.setToolTip(tooltip)
        font = check.font()
        font.setBold(True)
        check.setFont(font)
        return self._centered_cell_widget(check), check

    def _paint_group_cell_widget(self, widget: QtWidgets.QWidget, color: QtGui.QColor) -> None:
        """Handle paint group cell widget behavior for the control panel."""
        name = color.name()
        widget.setAutoFillBackground(True)
        palette = widget.palette()
        palette.setColor(QtGui.QPalette.Window, color)
        palette.setColor(QtGui.QPalette.Base, color)
        widget.setPalette(palette)
        widget.setStyleSheet(f"background-color: {name};")

    def _make_scope_combo(self) -> QtWidgets.QComboBox:
        """Create the scope combo widget or helper."""
        combo = QtWidgets.QComboBox()
        combo.setMinimumWidth(230)
        combo.setToolTip("Select the active motors/joints used for control, calibration, and READY checks")
        dual_suffix = " (dual)" if HAND_PROFILE.lower() == "v2" else ""
        combo.addItem("Control Set: Pinky J00-J03", self._scope_range(0, 3, True))
        combo.addItem(f"Control Set: Joint J00{dual_suffix}", self._scope_range(0, 0, True))
        combo.addItem("Control Set: Joint J01", self._scope_range(1, 1, True))
        combo.addItem("Control Set: Joint J02", self._scope_range(2, 2, True))
        combo.addItem("Control Set: Joint J03", self._scope_range(3, 3, True))
        combo.addItem("Control Set: Ring J04-J07", self._scope_range(4, 7, True))
        combo.addItem("Control Set: Middle J08-J11", self._scope_range(8, 11, True))
        combo.addItem("Control Set: Index J12-J15", self._scope_range(12, 15, True))
        combo.addItem("Control Set: Thumb J16-J20", self._scope_range(16, 20, True))
        combo.addItem("Control Set: Full Hand", (VALID_MOTOR_MASK, (1 << ENCODER_COUNT) - 1, True))
        if _motor_channel_valid(14):
            combo.addItem("Control Set: Single M14", (1 << 14, 1 << 14, True))
            combo.addItem("Control Set: M14 Servo Only", (1 << 14, 1 << 14, False))
        if _motor_channel_valid(11):
            combo.addItem("Control Set: Single M11", (1 << 11, 1 << 11, True))
            combo.addItem("Control Set: M11 Servo Only", (1 << 11, 1 << 11, False))
        selected = max(0, min(int(getattr(self, "_scope_selection_index", 0)), combo.count() - 1))
        combo.setCurrentIndex(selected)
        combo.currentIndexChanged.connect(lambda _idx: self._on_active_scope_changed())
        return combo

    def _make_connection_light(self) -> QtWidgets.QLabel:
        """Create the connection light widget or helper."""
        light = QtWidgets.QLabel()
        light.setFixedSize(14, 14)
        light.setToolTip("Lower serial disconnected")
        return light

    def _button_with_connection_light(
        self,
        button: QtWidgets.QPushButton,
        light: QtWidgets.QLabel,
    ) -> QtWidgets.QWidget:
        """Handle button with connection light behavior for the control panel."""
        panel = QtWidgets.QWidget()
        row = QtWidgets.QHBoxLayout(panel)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(6)
        row.addWidget(button, 1)
        row.addWidget(light, 0, QtCore.Qt.AlignVCenter)
        return panel

    def _update_connection_lights(self) -> None:
        """Handle update connection lights behavior for the control panel."""
        connected = bool(self.context.is_comm_connected())
        color = "#2e7d32" if connected else "#c62828"
        border = "#1b5e20" if connected else "#8e0000"
        tip = "Lower serial connected" if connected else "Lower serial disconnected"
        for light in (getattr(self, "run_conn_light", None), getattr(self, "stop_conn_light", None)):
            if light is None:
                continue
            light.setToolTip(tip)
            light.setStyleSheet(
                f"background:{color}; border:1px solid {border}; border-radius:7px;"
            )

    def _clear_body(self) -> None:
        """Clear the body state."""
        self.stop_all_tests("stopped on view switch", update_status=False)
        if getattr(self, "scope_combo", None) is not None:
            try:
                self._scope_selection_index = int(self.scope_combo.currentIndex())
            except RuntimeError:
                pass
        if getattr(self, "tabs", None) is not None:
            try:
                self._last_manual_tab_index = int(self.tabs.currentIndex())
            except RuntimeError:
                pass
        if self._body_widget is not None:
            closer = getattr(self._body_widget, "close", None)
            if callable(closer):
                try:
                    closer()
                except RuntimeError:
                    pass
            self.body_layout.removeWidget(self._body_widget)
            self._body_widget.deleteLater()
            self._body_widget = None
        self.scope_combo = None
        self._replay_view = None
        self._reset_manual_widget_refs()

    def _reset_manual_widget_refs(self) -> None:
        self._control_widgets = []
        self.manual_check = None
        self.new_device_reset_btn = None
        self.calib_load_btn = None
        self.calib_import_btn = None
        self.calib_export_btn = None
        self.open_btn = None
        self.release_half_turn_btn = None
        self.tighten_half_turn_btn = None
        self.fist_btn = None
        self.tabs = None
        self.device_check_view = None
        self.joint_table = None
        self.motor_table = None
        self.test_panel = None
        self.test_start_btn = None
        self.test_stop_btn = None
        self.test_hold_spin = None
        self.ff_start_btn = None
        self.ff_stop_btn = None
        self.ff_joint_combo = None
        self.ff_targets_edit = None
        self.ff_step_spin = None
        self.tri_start_btn = None
        self.tri_stop_btn = None
        self.tri_joint_combo = None
        self.tri_min_spin = None
        self.tri_max_spin = None
        self.tri_freq_spin = None
        self.local_start_btn = None
        self.local_stop_btn = None
        self.local_freq = None
        self.local_amp = None
        self.test_joint_combos = []
        self.test_min_spins = []
        self.test_max_spins = []
        self.test_freq_spins = []
        self.test_phase_spins = []
        self.joint_current_items = []
        self.joint_encoder_items = []
        self.joint_abs_limit_items = []
        self.joint_status_items = []
        self.joint_min_items = []
        self.joint_max_items = []
        self.joint_target_spins = []
        self.joint_sliders = []
        self.joint_send_buttons = []
        self.joint_use_checks = [None] * ENCODER_COUNT
        self.joint_group_use_checks = []
        self.teleop_joint_use_checks = [None] * ENCODER_COUNT
        self.teleop_group_use_checks = []
        self.teleop_hand_use_check = None
        self._joint_target_user_set = []
        self.motor_turn_items = []
        self.motor_raw_items = []
        self.motor_flash_raw_items = []
        self.motor_abs_items = []
        self.motor_load_items = []
        self.motor_joint_items = []
        self.motor_target_spins = []
        self.motor_sliders = []
        self.motor_send_buttons = []
        self.abs_limit_lock_toggle_btn = None
        self.motor_use_checks = [None] * MOTOR_COUNT
        self.motor_group_use_checks = []
        self._motor_target_user_set = []
        self._joint_target_initialized = False
        self._motor_target_initialized = False
        self._joint_run_states = [False] * ENCODER_COUNT
        self._motor_run_states = [False] * MOTOR_COUNT

    def _build_scope_workflow(self, title: str) -> tuple[QtWidgets.QGroupBox, QtWidgets.QHBoxLayout]:
        """Build the scope workflow UI section."""
        return ManualControlView.scope_workflow(self, title)

    def _build_manual_body(self) -> None:
        """Build the manual body UI section."""
        ManualControlView.build(self)

    def _build_teleop_body(self) -> None:
        """Build the teleop body UI section."""
        TeleopControlView.build(self)

    def _build_teleop_scope_workflow(self) -> QtWidgets.QGroupBox:
        """Build the teleop scope workflow UI section."""
        return TeleopControlView.scope_workflow(self)

    def _make_item(self, text: str = "-") -> QtWidgets.QTableWidgetItem:
        """Create the item widget or helper."""
        item = QtWidgets.QTableWidgetItem(text)
        item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
        return item

    @staticmethod
    def _set_item_text_if_changed(item: Optional[QtWidgets.QTableWidgetItem], text: str) -> None:
        """Update the item text if changed value or state."""
        if item is not None and item.text() != str(text):
            item.setText(str(text))

    @staticmethod
    def _set_item_tooltip_if_changed(item: Optional[QtWidgets.QTableWidgetItem], text: str) -> None:
        """Update the item tooltip if changed value or state."""
        if item is not None and item.toolTip() != str(text):
            item.setToolTip(str(text))

    @staticmethod
    def _set_spin_value_if_changed(spin, value, tol: float = 0.0) -> None:
        """Update the spin value if changed value or state."""
        if spin is None:
            return
        current = float(spin.value())
        target = float(value)
        if abs(current - target) <= float(tol):
            return
        was_blocked = spin.blockSignals(True)
        try:
            spin.setValue(value)
        finally:
            spin.blockSignals(was_blocked)

    @staticmethod
    def _set_slider_value_if_changed(slider: Optional[QtWidgets.QSlider], value: int) -> None:
        """Update the slider value if changed value or state."""
        if slider is None:
            return
        target = int(value)
        if int(slider.value()) == target:
            return
        was_blocked = slider.blockSignals(True)
        try:
            slider.setValue(target)
        finally:
            slider.blockSignals(was_blocked)

    @staticmethod
    def _set_spin_range_if_changed(spin, lo, hi) -> None:
        """Update the spin range if changed value or state."""
        if spin is None:
            return
        if abs(float(spin.minimum()) - float(lo)) <= 0.0005 and abs(float(spin.maximum()) - float(hi)) <= 0.0005:
            return
        was_blocked = spin.blockSignals(True)
        try:
            spin.setRange(lo, hi)
        finally:
            spin.blockSignals(was_blocked)

    @staticmethod
    def _set_slider_range_if_changed(slider: Optional[QtWidgets.QSlider], lo: int, hi: int) -> None:
        """Update the slider range if changed value or state."""
        if slider is None:
            return
        lo = int(lo)
        hi = int(hi)
        if int(slider.minimum()) == lo and int(slider.maximum()) == hi:
            return
        was_blocked = slider.blockSignals(True)
        try:
            slider.setRange(lo, hi)
        finally:
            slider.blockSignals(was_blocked)

    def _build_joint_table(self) -> QtWidgets.QTableWidget:
        """Build the joint table UI section."""
        return JointChannelTable.build(self)

    def _build_motor_table(self) -> QtWidgets.QTableWidget:
        """Build the motor table UI section."""
        return MotorChannelTable.build(self)

    def _build_test_panel(self) -> QtWidgets.QWidget:
        """Build the test panel UI section."""
        return ManualTestPanel.build(self)

    def _joint_choices(self) -> List[str]:
        """Compute or update joint choices data."""
        return ManualTestPanel.joint_choices()

    def _build_sine_test_box(self) -> QtWidgets.QGroupBox:
        """Build the sine test box UI section."""
        return ManualTestPanel.sine_test_box(self)

    def _build_feedforward_test_box(self) -> QtWidgets.QGroupBox:
        """Build the feedforward test box UI section."""
        return ManualTestPanel.feedforward_test_box(self)

    def _make_float_spin(low: float, high: float, value: float, decimals: int) -> QtWidgets.QDoubleSpinBox:
        """Create the float spin widget or helper."""
        return ManualTestPanel.make_float_spin(low, high, value, decimals)

    def _set_status(self, text: str) -> None:
        """Update the status value or state."""
        self.context.set_status(text)

    def _warn(self, title: str, text: str) -> None:
        """Handle warn behavior for the control panel."""
        show_scrollable_message(
            self,
            title,
            text,
            severity="warning",
        )

    def _error(self, title: str, text: str) -> None:
        """Handle error behavior for the control panel."""
        show_scrollable_message(
            self,
            title,
            text,
            severity="critical",
        )
