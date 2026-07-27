"""Mode view builders for the modular control panel."""

from .ui_control_common import (
    DeviceCheckView,
    ENCODER_COUNT,
    FINGER_JOINT_RANGES,
    ControlScopeModel,
    QtWidgets,
    TeleopView,
    WORKSPACE_MAPPING_VERSION,
    WORKSPACE_URDF_CRC,
    _ModeBody,
)


class ManualControlView(_ModeBody):
    """Builds the Manual mode body and workflow tabs."""
    @staticmethod
    def scope_workflow(panel, title: str) -> tuple[QtWidgets.QGroupBox, QtWidgets.QHBoxLayout]:
        """Build the scope workflow UI section."""
        panel.scope_combo = panel._make_scope_combo()
        workflow = QtWidgets.QGroupBox(title)
        wf_layout = QtWidgets.QHBoxLayout(workflow)
        wf_layout.addWidget(QtWidgets.QLabel("Control Set:"))
        wf_layout.addWidget(panel.scope_combo)
        return workflow, wf_layout

    @staticmethod
    def build(panel) -> None:
        """Build the manual body UI section."""
        interaction_enabled = bool(getattr(panel, "_interaction_enabled", False))
        panel._clear_body()
        panel._reset_manual_widget_refs()
        panel._app_mode = "manual"

        body = ManualControlView()
        layout = QtWidgets.QVBoxLayout(body)
        layout.setContentsMargins(0, 0, 0, 0)

        workflow = QtWidgets.QGroupBox("Manual Workflow")
        wf_layout = QtWidgets.QHBoxLayout(workflow)
        panel.manual_check = QtWidgets.QCheckBox("Control Enable")
        panel.manual_check.setChecked(False)
        panel.new_device_reset_btn = QtWidgets.QPushButton("New Device Reset")
        panel.new_device_reset_btn.setToolTip(
            "Repair only online Turn Unknown motors, then guide single-joint recalibration"
        )
        panel.calib_load_btn = QtWidgets.QPushButton("Calib Load")
        panel.calib_load_btn.setToolTip("Load the last saved calibration snapshot when mechanics have not changed")
        panel.calib_import_btn = QtWidgets.QPushButton("Import Calib")
        panel.calib_import_btn.setToolTip(
            "Import and activate a portable MMHand calibration bundle"
        )
        panel.calib_export_btn = QtWidgets.QPushButton("Export Calib")
        panel.calib_export_btn.setToolTip(
            "Export the active calibration bundle to another computer"
        )
        panel.open_btn = QtWidgets.QPushButton("Open Pose")
        panel.release_half_turn_btn = QtWidgets.QPushButton("Release Half Turn")
        panel.tighten_half_turn_btn = QtWidgets.QPushButton("Tighten Half Turn")
        panel.fist_btn = QtWidgets.QPushButton("Fist Pose")
        wf_layout.addWidget(panel.manual_check)
        wf_layout.addStretch(1)
        for btn in [
            panel.new_device_reset_btn,
            panel.calib_load_btn,
            panel.calib_import_btn,
            panel.calib_export_btn,
            panel.open_btn,
            panel.release_half_turn_btn,
            panel.tighten_half_turn_btn,
            panel.fist_btn,
        ]:
            wf_layout.addWidget(btn)
        layout.addWidget(workflow)

        panel.manual_check.toggled.connect(panel._apply_interaction_enabled)
        panel.new_device_reset_btn.clicked.connect(lambda _checked=False: panel.new_device_reset())
        panel.calib_load_btn.clicked.connect(lambda _checked=False: panel.load_calibration_snapshot())
        panel.calib_import_btn.clicked.connect(
            lambda _checked=False: panel.import_calibration_snapshot()
        )
        panel.calib_export_btn.clicked.connect(
            lambda _checked=False: panel.export_calibration_snapshot()
        )
        panel.open_btn.clicked.connect(lambda _checked=False: panel.apply_open_pose())
        panel.release_half_turn_btn.clicked.connect(lambda _checked=False: panel.release_all_motors_half_turn())
        panel.tighten_half_turn_btn.clicked.connect(lambda _checked=False: panel.tighten_all_motors_half_turn())
        panel.fist_btn.clicked.connect(lambda _checked=False: panel.apply_fist_pose())

        panel.tabs = QtWidgets.QTabWidget()
        panel.device_check_view = DeviceCheckView(
            panel.context,
            mapping_version=WORKSPACE_MAPPING_VERSION,
            urdf_crc=WORKSPACE_URDF_CRC,
        )
        panel.device_check_view.operation_finished.connect(
            panel._on_device_check_operation_finished
        )
        panel.joint_table = panel._build_joint_table()
        panel.motor_table = panel._build_motor_table()
        joint_page = QtWidgets.QWidget()
        joint_page_layout = QtWidgets.QVBoxLayout(joint_page)
        joint_page_layout.setContentsMargins(0, 0, 0, 0)
        joint_page_layout.setSpacing(4)
        abs_limit_bar = QtWidgets.QWidget()
        abs_limit_layout = QtWidgets.QHBoxLayout(abs_limit_bar)
        abs_limit_layout.setContentsMargins(6, 2, 6, 2)
        abs_limit_layout.addWidget(QtWidgets.QLabel("Feedback ABS Limit (default unlocked)"))
        abs_limit_layout.addStretch(1)
        panel.abs_limit_lock_toggle_btn = QtWidgets.QPushButton("LOCK")
        panel.abs_limit_lock_toggle_btn.setMinimumWidth(86)
        panel.abs_limit_lock_toggle_btn.clicked.connect(
            lambda _checked=False: panel.toggle_abs_limit_lock()
        )
        abs_limit_layout.addWidget(panel.abs_limit_lock_toggle_btn)
        joint_page_layout.addWidget(abs_limit_bar)
        joint_page_layout.addWidget(panel.joint_table, 1)
        panel._sync_scope_group_checks()
        panel.test_panel = panel._build_test_panel()
        panel.tabs.addTab(panel.device_check_view, "Device Check")
        panel.tabs.addTab(joint_page, "Joint Channels")
        panel.tabs.addTab(panel.motor_table, "Motor Channels")
        panel.tabs.addTab(panel.test_panel, "Test")
        panel.tabs.currentChanged.connect(lambda _idx: panel._on_manual_tab_changed())
        tab_index = max(0, min(int(getattr(panel, "_last_manual_tab_index", 0)), panel.tabs.count() - 1))
        panel.tabs.setCurrentIndex(tab_index)
        layout.addWidget(panel.tabs, 1)
        panel.body_layout.addWidget(body)
        panel._body_widget = body
        panel._apply_interaction_enabled(interaction_enabled)
        panel._on_manual_tab_changed()



class TeleopControlView(_ModeBody):
    """Builds the Teleop mode body and active-scope selector."""
    @staticmethod
    def build(panel) -> None:
        """Build the teleop body UI section."""
        panel._clear_body()
        panel._reset_manual_widget_refs()
        panel._app_mode = "teleop"

        body = TeleopControlView()
        layout = QtWidgets.QVBoxLayout(body)
        layout.setContentsMargins(0, 0, 0, 0)

        workflow = TeleopControlView.scope_workflow(panel)
        layout.addWidget(workflow)

        teleop_view = TeleopView(
            panel.context,
            selected_joint_mask_getter=panel._teleop_selected_joint_mask_value,
        )
        body.child = teleop_view
        layout.addWidget(teleop_view, 1)

        panel.body_layout.addWidget(body)
        panel._body_widget = body

    @staticmethod
    def scope_workflow(panel) -> QtWidgets.QGroupBox:
        """Build the teleop scope workflow UI section."""
        workflow = QtWidgets.QGroupBox("Teleop Workflow")
        row_layout = QtWidgets.QHBoxLayout(workflow)
        row_layout.setContentsMargins(8, 4, 8, 4)
        row_layout.setSpacing(10)
        panel._teleop_selected_motor_mask = panel._motor_mask_for_joint_mask(panel._teleop_selected_joint_mask)

        all_joint_mask = (1 << ENCODER_COUNT) - 1
        hand_check = QtWidgets.QCheckBox("Hand")
        hand_check.setToolTip("Select all teleop joints")
        hand_check.setChecked((panel._teleop_selected_joint_mask & all_joint_mask) == all_joint_mask)
        hand_check.toggled.connect(lambda checked: panel._on_teleop_hand_use_changed(checked))
        hand_check.setMinimumWidth(64)
        panel.teleop_hand_use_check = hand_check
        row_layout.addWidget(hand_check, 0)

        separator = QtWidgets.QFrame()
        separator.setFrameShape(QtWidgets.QFrame.VLine)
        separator.setFrameShadow(QtWidgets.QFrame.Sunken)
        row_layout.addWidget(separator)

        for label, start, end in FINGER_JOINT_RANGES:
            group_mask = ControlScopeModel.joint_group_mask(label, start, end)
            column = QtWidgets.QWidget()
            col_layout = QtWidgets.QVBoxLayout(column)
            col_layout.setContentsMargins(0, 0, 0, 0)
            col_layout.setSpacing(2)
            group_check = QtWidgets.QCheckBox(label)
            group_check.setToolTip("Select this finger group for Teleop active scope")
            group_check.setChecked((panel._teleop_selected_joint_mask & group_mask) == group_mask)
            group_check.toggled.connect(
                lambda checked, s=start, e=end: panel._on_teleop_joint_group_use_changed(s, e, checked)
            )
            panel.teleop_group_use_checks.append(group_check)
            col_layout.addWidget(group_check)

            joint_row = QtWidgets.QHBoxLayout()
            joint_row.setContentsMargins(14, 0, 0, 0)
            joint_row.setSpacing(4)
            for joint_idx in ControlScopeModel.joint_indices_for_group(label, start, end):
                use_check = QtWidgets.QCheckBox(f"J{joint_idx:02d}")
                use_check.setToolTip(f"Use J{joint_idx:02d} in Teleop active scope")
                use_check.setChecked((panel._teleop_selected_joint_mask & (1 << joint_idx)) != 0)
                use_check.toggled.connect(
                    lambda checked, idx=joint_idx: panel._on_teleop_joint_use_changed(idx, checked)
                )
                panel.teleop_joint_use_checks[joint_idx] = use_check
                joint_row.addWidget(use_check)
            col_layout.addLayout(joint_row)
            row_layout.addWidget(column, 1)

        row_layout.addStretch(1)
        panel._sync_teleop_group_checks()
        return workflow
