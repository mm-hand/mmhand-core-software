"""ui_control_test_panel.py - MMHand Desktop 模块契约。

职责：构建手动测试参数、动作和诊断控件。
线程：Qt GUI 主线程执行；耗时工作必须委托后台服务，禁止阻塞界面事件循环。
输入：用户操作、HandModel 只读快照以及 HandBackend/服务返回结果。
输出：Qt 控件状态、用户提示和经门面提交的应用命令。
状态所有权：仅拥有本视图控件和瞬时交互状态；设备真值由 core 层拥有。
禁止依赖：禁止直接导入串口通信和底层控制实现，禁止在 UI 中复制协议或安全规则。
"""

from .ui_control_common import (
    ENCODER_COUNT,
    FF_TEST_DEFAULT_STEP_S,
    FF_TEST_DEFAULT_TARGETS_DEG,
    List,
    QtWidgets,
    TEST_DEFAULT_FREQ_HZ,
    TEST_DEFAULT_HOLD_S,
    TEST_DEFAULT_PHASE_DEG,
    TEST_ROW_COUNT,
    TRI_TEST_DEFAULT_FREQ_HZ,
)


class ManualTestPanel(QtWidgets.QWidget):
    """Builds manual test widgets and registers them on the panel."""
    @staticmethod
    def build(panel) -> QtWidgets.QWidget:
        """Build the test panel UI section."""
        box = ManualTestPanel()
        layout = QtWidgets.QVBoxLayout(box)
        layout.addWidget(ManualTestPanel.pose_test_box(panel))
        layout.addWidget(ManualTestPanel.sine_test_box(panel))
        layout.addWidget(ManualTestPanel.feedforward_test_box(panel))
        layout.addWidget(ManualTestPanel.triangle_test_box(panel))
        layout.addStretch(1)
        return box

    @staticmethod
    def pose_test_box(panel) -> QtWidgets.QGroupBox:
        """构建只在 Test 页面使用的预设姿态测试入口。"""
        group = QtWidgets.QGroupBox("Pose Test")
        layout = QtWidgets.QHBoxLayout(group)
        panel.open_btn = QtWidgets.QPushButton("Open Pose")
        panel.open_btn.setToolTip("Send 0 deg to all 21 joint channels")
        panel.fist_btn = QtWidgets.QPushButton("Fist Pose")
        panel.fist_btn.setToolTip("Send the configured fist pose to the active joints")
        panel.open_btn.clicked.connect(lambda _checked=False: panel.apply_open_pose())
        panel.fist_btn.clicked.connect(lambda _checked=False: panel.apply_fist_pose())
        layout.addWidget(panel.open_btn)
        layout.addWidget(panel.fist_btn)
        layout.addStretch(1)
        return group

    @staticmethod
    def joint_choices() -> List[str]:
        """Compute or update joint choices data."""
        return ["None"] + [f"J{i:02d}" for i in range(ENCODER_COUNT)]

    @staticmethod
    def sine_test_box(panel) -> QtWidgets.QGroupBox:
        """Build the sine test box UI section."""
        group = QtWidgets.QGroupBox("Joint Sine Test")
        layout = QtWidgets.QGridLayout(group)
        for col, text in enumerate(["#", "Joint", "Min(deg)", "Max(deg)", "Freq(Hz)", "Phase(deg)"]):
            layout.addWidget(QtWidgets.QLabel(text), 0, col)
        panel.test_joint_combos: List[QtWidgets.QComboBox] = []
        panel.test_min_spins: List[QtWidgets.QDoubleSpinBox] = []
        panel.test_max_spins: List[QtWidgets.QDoubleSpinBox] = []
        panel.test_freq_spins: List[QtWidgets.QDoubleSpinBox] = []
        panel.test_phase_spins: List[QtWidgets.QDoubleSpinBox] = []
        for row in range(TEST_ROW_COUNT):
            layout.addWidget(QtWidgets.QLabel(str(row + 1)), row + 1, 0)
            combo = QtWidgets.QComboBox()
            combo.addItems(ManualTestPanel.joint_choices())
            combo.currentTextChanged.connect(lambda _text, idx=row: panel._on_test_joint_changed(idx))
            lo = ManualTestPanel.make_float_spin(0.0, 120.0, 0.0, 2)
            hi = ManualTestPanel.make_float_spin(0.0, 120.0, 10.0, 2)
            freq = ManualTestPanel.make_float_spin(0.001, 100.0, TEST_DEFAULT_FREQ_HZ, 4)
            phase = ManualTestPanel.make_float_spin(0.0, 360.0, TEST_DEFAULT_PHASE_DEG, 2)
            for col, widget in enumerate([combo, lo, hi, freq, phase], start=1):
                layout.addWidget(widget, row + 1, col)
            panel.test_joint_combos.append(combo)
            panel.test_min_spins.append(lo)
            panel.test_max_spins.append(hi)
            panel.test_freq_spins.append(freq)
            panel.test_phase_spins.append(phase)
        row = QtWidgets.QHBoxLayout()
        panel.test_start_btn = QtWidgets.QPushButton("Start Test")
        panel.test_stop_btn = QtWidgets.QPushButton("Stop Test")
        panel.test_hold_spin = ManualTestPanel.make_float_spin(0.0, 60.0, TEST_DEFAULT_HOLD_S, 3)
        panel.test_start_btn.clicked.connect(lambda _checked=False: panel._on_start_test())
        panel.test_stop_btn.clicked.connect(lambda _checked=False: panel._on_stop_test())
        for widget in [
            panel.test_start_btn, panel.test_stop_btn, QtWidgets.QLabel("Initial Hold(s)"), panel.test_hold_spin,
        ]:
            row.addWidget(widget)
        row.addStretch(1)
        layout.addLayout(row, TEST_ROW_COUNT + 1, 0, 1, 6)
        return group

    @staticmethod
    def feedforward_test_box(panel) -> QtWidgets.QGroupBox:
        """Build the feedforward test box UI section."""
        group = QtWidgets.QGroupBox("Feedforward Test")
        layout = QtWidgets.QGridLayout(group)
        panel.ff_joint_combo = QtWidgets.QComboBox()
        panel.ff_joint_combo.addItems(ManualTestPanel.joint_choices())
        panel.ff_joint_combo.currentTextChanged.connect(
            lambda _text: panel._refresh_feedforward_test_workspace()
        )
        panel.ff_targets_edit = QtWidgets.QLineEdit(FF_TEST_DEFAULT_TARGETS_DEG)
        panel.ff_step_spin = ManualTestPanel.make_float_spin(0.001, 60.0, FF_TEST_DEFAULT_STEP_S, 3)
        panel.ff_start_btn = QtWidgets.QPushButton("Start Feedforward Test")
        panel.ff_stop_btn = QtWidgets.QPushButton("Stop Feedforward Test")
        panel.ff_start_btn.clicked.connect(lambda _checked=False: panel._on_start_ff_test())
        panel.ff_stop_btn.clicked.connect(lambda _checked=False: panel._on_stop_ff_test())
        for col, text in enumerate(["Joint", "Targets(deg)", "Step Time(s)"]):
            layout.addWidget(QtWidgets.QLabel(text), 0, col)
        layout.addWidget(panel.ff_joint_combo, 1, 0)
        layout.addWidget(panel.ff_targets_edit, 1, 1)
        layout.addWidget(panel.ff_step_spin, 1, 2)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(panel.ff_start_btn)
        row.addWidget(panel.ff_stop_btn)
        row.addStretch(1)
        layout.addLayout(row, 2, 0, 1, 3)
        return group

    @staticmethod
    def triangle_test_box(panel) -> QtWidgets.QGroupBox:
        """构建上位机三角波测试控件，目标仍通过正式关节角链路发送。"""
        group = QtWidgets.QGroupBox("Triangle Wave Test")
        layout = QtWidgets.QGridLayout(group)
        panel.tri_joint_combo = QtWidgets.QComboBox()
        panel.tri_joint_combo.addItems(ManualTestPanel.joint_choices())
        panel.tri_joint_combo.currentTextChanged.connect(
            lambda _text: panel._refresh_single_test_workspace("triangle")
        )
        panel.tri_min_spin = ManualTestPanel.make_float_spin(0.0, 120.0, 0.0, 2)
        panel.tri_max_spin = ManualTestPanel.make_float_spin(0.0, 120.0, 10.0, 2)
        panel.tri_freq_spin = ManualTestPanel.make_float_spin(
            0.001, 100.0, TRI_TEST_DEFAULT_FREQ_HZ, 4
        )
        panel.tri_start_btn = QtWidgets.QPushButton("Start Triangle Test")
        panel.tri_stop_btn = QtWidgets.QPushButton("Stop Triangle Test")
        panel.tri_start_btn.clicked.connect(
            lambda _checked=False: panel._on_start_tri_test()
        )
        panel.tri_stop_btn.clicked.connect(
            lambda _checked=False: panel._on_stop_tri_test()
        )
        for col, text in enumerate(["Joint", "Min(deg)", "Max(deg)", "Freq(Hz)"]):
            layout.addWidget(QtWidgets.QLabel(text), 0, col)
        for col, widget in enumerate(
            [
                panel.tri_joint_combo,
                panel.tri_min_spin,
                panel.tri_max_spin,
                panel.tri_freq_spin,
            ]
        ):
            layout.addWidget(widget, 1, col)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(panel.tri_start_btn)
        row.addWidget(panel.tri_stop_btn)
        row.addStretch(1)
        layout.addLayout(row, 2, 0, 1, 4)
        return group

    @staticmethod
    def make_float_spin(low: float, high: float, value: float, decimals: int) -> QtWidgets.QDoubleSpinBox:
        """Create the float spin widget or helper."""
        spin = QtWidgets.QDoubleSpinBox()
        spin.setRange(float(low), float(high))
        spin.setDecimals(int(decimals))
        spin.setSingleStep(0.1 if decimals <= 2 else 0.01)
        spin.setValue(float(value))
        return spin
