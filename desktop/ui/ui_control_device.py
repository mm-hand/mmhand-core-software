"""ui_control_device.py - MMHand Desktop 模块契约。

职责：构建设备控制栏并转发设备级操作。
线程：Qt GUI 主线程执行；耗时工作必须委托后台服务，禁止阻塞界面事件循环。
输入：用户操作、HandModel 只读快照以及 HandBackend/服务返回结果。
输出：Qt 控件状态、用户提示和经门面提交的应用命令。
状态所有权：仅拥有本视图控件和瞬时交互状态；设备真值由 core 层拥有。
禁止依赖：禁止直接导入串口通信和底层控制实现，禁止在 UI 中复制协议或安全规则。
"""

from .ui_control_common import QtWidgets, Signal


class DeviceControlBar(QtWidgets.QGroupBox):
    """Top-level device command buttons for the control panel."""

    start_requested = Signal()
    stop_requested = Signal()
    pause_requested = Signal()
    raw_ik_toggled = Signal(bool)
    clear_fault_requested = Signal()
    calibrate_requested = Signal()
    rope_init_requested = Signal()
    nvs_toggle_requested = Signal()

    def __init__(self, parent=None):
        """Initialize this component."""
        super().__init__("Device", parent)
        self.enable_btn = QtWidgets.QPushButton("ENABLE")
        self.stop_btn = QtWidgets.QPushButton("E-STOP")
        self.pause_btn = QtWidgets.QPushButton("Pause")
        self.raw_ik_enable_btn = QtWidgets.QPushButton("RAW IK ENABLE")
        self.raw_ik_enable_btn.setCheckable(True)
        self.raw_ik_enable_btn.setChecked(False)
        self.raw_ik_enable_btn.setToolTip(
            "Separately authorize ROS2 raw IK/joint target frames; ordinary ENABLE does not arm teleoperation"
        )
        self.reset_btn = QtWidgets.QPushButton("Clear Fault")
        self.reset_btn.setToolTip(
            "Clear latched protection/fault flags, clear command state, and return firmware to UNCONTROLLABLE"
        )
        self.calib_btn = QtWidgets.QPushButton("Calibrate")
        self.rope_init_btn = QtWidgets.QPushButton("Rope Init")
        self.rope_init_btn.setToolTip(
            "Ask the P4 Rope Init task to move selected IDLE joints into their calibrated workspace"
        )
        self.nvs_control_btn = QtWidgets.QPushButton("Flash ON")
        self.nvs_control_btn.setToolTip(
            "Toggle P4 Flash/NVS multi-turn persistence. P4 NVS is the only restore source."
        )
        self._build()

    def _build(self) -> None:
        """Build the widget layout."""
        layout = QtWidgets.QGridLayout(self)
        layout.setHorizontalSpacing(8)
        layout.setVerticalSpacing(6)
        for button in (
            self.enable_btn,
            self.stop_btn,
            self.pause_btn,
            self.raw_ik_enable_btn,
        ):
            button.setMinimumHeight(46)
        for button in (
            self.calib_btn,
            self.reset_btn,
            self.rope_init_btn,
            self.nvs_control_btn,
        ):
            button.setMinimumHeight(34)

        self.enable_btn.setStyleSheet("font-weight:700;background:#2e7d32;color:white;")
        self.stop_btn.setStyleSheet("font-weight:700;background:#c62828;color:white;")
        self.pause_btn.setStyleSheet("font-weight:700;background:#f9a825;color:black;")
        self.raw_ik_enable_btn.setStyleSheet("font-weight:700;background:#455a64;color:white;")
        self.nvs_control_btn.setStyleSheet("font-weight:700;background:#e8f5e9;color:#1b5e20;")

        layout.addWidget(self._centered_action_cell(self.enable_btn), 0, 0, 2, 1)
        layout.addWidget(self._centered_action_cell(self.stop_btn), 0, 1, 2, 1)
        layout.addWidget(self._centered_action_cell(self.pause_btn), 0, 2, 2, 1)
        layout.addWidget(self._centered_action_cell(self.raw_ik_enable_btn), 0, 3, 2, 1)
        layout.addWidget(self.calib_btn, 0, 4)
        layout.addWidget(self.reset_btn, 0, 5)
        layout.addWidget(self.rope_init_btn, 1, 4)
        layout.addWidget(self.nvs_control_btn, 1, 5)
        layout.setRowStretch(0, 1)
        layout.setRowStretch(1, 1)
        for col in range(6):
            layout.setColumnStretch(col, 1)
        self.enable_btn.clicked.connect(lambda _checked=False: self.start_requested.emit())
        self.stop_btn.clicked.connect(lambda _checked=False: self.stop_requested.emit())
        self.pause_btn.clicked.connect(lambda _checked=False: self.pause_requested.emit())
        self.raw_ik_enable_btn.toggled.connect(self.raw_ik_toggled.emit)
        self.reset_btn.clicked.connect(lambda _checked=False: self.clear_fault_requested.emit())
        self.calib_btn.clicked.connect(lambda _checked=False: self.calibrate_requested.emit())
        self.rope_init_btn.clicked.connect(lambda _checked=False: self.rope_init_requested.emit())
        self.nvs_control_btn.clicked.connect(lambda _checked=False: self.nvs_toggle_requested.emit())

    @staticmethod
    def _centered_action_cell(button: QtWidgets.QPushButton) -> QtWidgets.QWidget:
        """Wrap a tall action button in a vertically centered cell."""
        cell = QtWidgets.QWidget()
        cell_layout = QtWidgets.QVBoxLayout(cell)
        cell_layout.setContentsMargins(0, 0, 0, 0)
        cell_layout.addStretch(1)
        cell_layout.addWidget(button)
        cell_layout.addStretch(1)
        return cell

    def connect_panel(self, panel) -> None:
        """Wire button signals to the owning control panel."""
        self.start_requested.connect(panel.start_controller)
        self.stop_requested.connect(panel.stop_controller)
        self.pause_requested.connect(panel.pause_controller)
        self.raw_ik_toggled.connect(panel.toggle_raw_ik_output)
        self.clear_fault_requested.connect(panel.clear_fault_with_confirm)
        self.calibrate_requested.connect(panel.calibrate)
        self.rope_init_requested.connect(panel.rope_init_controller)
        self.nvs_toggle_requested.connect(panel.toggle_multiturn_nvs)
