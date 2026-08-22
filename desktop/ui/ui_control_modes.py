"""ui_control_modes.py - MMHand Desktop 模块契约。

职责：构建负反馈、直控和测试模式页面并连接模式事件。
线程：Qt GUI 主线程执行；耗时工作必须委托后台服务，禁止阻塞界面事件循环。
输入：用户操作、HandModel 只读快照以及 HandBackend/服务返回结果。
输出：Qt 控件状态、用户提示和经门面提交的应用命令。
状态所有权：仅拥有本视图控件和瞬时交互状态；设备真值由 core 层拥有。
禁止依赖：禁止直接导入串口通信和底层控制实现，禁止在 UI 中复制协议或安全规则。
"""

from .ui_control_common import (
    DeviceCheckView,
    QtWidgets,
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
        panel._clear_body()
        panel._reset_manual_widget_refs()
        panel._app_mode = "manual"

        body = ManualControlView()
        layout = QtWidgets.QVBoxLayout(body)
        layout.setContentsMargins(0, 0, 0, 0)

        workflow = QtWidgets.QGroupBox("Manual Workflow")
        wf_layout = QtWidgets.QHBoxLayout(workflow)
        panel.calib_file_load_btn = QtWidgets.QPushButton("Load Calibration…")
        panel.calib_file_load_btn.setToolTip(
            "Select a saved or external calibration JSON; validate it and restore it to P4"
        )
        panel.release_half_turn_btn = QtWidgets.QPushButton("Release Half Turn")
        panel.tighten_half_turn_btn = QtWidgets.QPushButton("Tighten Half Turn")
        wf_layout.addStretch(1)
        for btn in [
            panel.calib_file_load_btn,
            panel.release_half_turn_btn,
            panel.tighten_half_turn_btn,
        ]:
            wf_layout.addWidget(btn)
        layout.addWidget(workflow)

        panel.calib_file_load_btn.clicked.connect(
            lambda _checked=False: panel.load_calibration_from_file()
        )
        panel.release_half_turn_btn.clicked.connect(lambda _checked=False: panel.release_all_motors_half_turn())
        panel.tighten_half_turn_btn.clicked.connect(lambda _checked=False: panel.tighten_all_motors_half_turn())

        panel.tabs = QtWidgets.QTabWidget()
        panel.device_check_view = DeviceCheckView(
            panel.context,
            mapping_version=WORKSPACE_MAPPING_VERSION,
            urdf_crc=WORKSPACE_URDF_CRC,
        )
        panel.device_check_view.operation_finished.connect(
            panel._on_device_check_operation_finished
        )
        panel.device_check_view.manual_prepare_finished.connect(
            panel._on_manual_prepare_finished
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
        abs_limit_layout.addWidget(QtWidgets.QLabel("Calibrated ABS Range (diagnostic)"))
        abs_limit_layout.addStretch(1)
        joint_page_layout.addWidget(abs_limit_bar)
        joint_page_layout.addWidget(panel.joint_table, 1)
        panel.motor_page = QtWidgets.QWidget()
        motor_page_layout = QtWidgets.QVBoxLayout(panel.motor_page)
        motor_page_layout.setContentsMargins(0, 0, 0, 0)
        motor_page_layout.setSpacing(4)
        motor_page_layout.addWidget(panel.motor_table, 1)
        panel._sync_scope_group_checks()
        panel.test_panel = panel._build_test_panel()
        panel.tabs.addTab(panel.device_check_view, "Device Check")
        panel.tabs.addTab(joint_page, "Joint Channels")
        panel.tabs.addTab(panel.motor_page, "Motor Channels")
        panel.tabs.addTab(panel.test_panel, "Test")
        panel.tabs.currentChanged.connect(lambda _idx: panel._on_manual_tab_changed())
        tab_index = max(0, min(int(getattr(panel, "_last_manual_tab_index", 0)), panel.tabs.count() - 1))
        panel.tabs.setCurrentIndex(tab_index)
        layout.addWidget(panel.tabs, 1)
        panel.body_layout.addWidget(body)
        panel._body_widget = body
        panel._on_manual_tab_changed()
