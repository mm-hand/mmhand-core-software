"""ui_control_tables.py - MMHand Desktop 模块契约。

职责：构建关节与舵机通道表格并定义稳定列布局。
线程：Qt GUI 主线程执行；耗时工作必须委托后台服务，禁止阻塞界面事件循环。
输入：用户操作、HandModel 只读快照以及 HandBackend/服务返回结果。
输出：Qt 控件状态、用户提示和经门面提交的应用命令。
状态所有权：仅拥有本视图控件和瞬时交互状态；设备真值由 core 层拥有。
禁止依赖：禁止直接导入串口通信和底层控制实现，禁止在 UI 中复制协议或安全规则。
"""

from .ui_control_common import (
    ENCODER_COUNT,
    FINGER_JOINT_RANGES,
    ControlScopeModel,
    MOTOR_ABS_CMD_MAX,
    MOTOR_ABS_CMD_MIN,
    MOTOR_COUNT,
    MOTOR_TARGET_SCALE_HI,
    MOTOR_TARGET_SCALE_LO,
    MOTOR_TO_JOINT_INDEX,
    QtCore,
    QtGui,
    QtWidgets,
    _motor_channel_valid,
)


class JointChannelTable(QtWidgets.QTableWidget):
    """Builds the Joint Channels table and registers its widgets on the panel."""
    @staticmethod
    def build(panel) -> QtWidgets.QTableWidget:
        """Build the joint table UI section."""
        joint_rows, group_ranges = ControlScopeModel.joint_rows_by_finger_group()
        table = JointChannelTable(len(joint_rows), 13)
        # 双腱 LUT ABS 范围使用显式换行；其他单行单元格不会受此影响。
        table.setWordWrap(True)
        table.setHorizontalHeaderLabels([
            "Group", "Use", "Joint", "Enc", "Current", "ROS2 Target", "Raw", "Control ABS", "Min",
            "Target", "Max", "Action", "Target Slider",
        ])
        table.verticalHeader().setVisible(False)
        table.setAlternatingRowColors(False)
        panel.joint_current_items = [None] * ENCODER_COUNT
        panel.joint_ros2_target_items = [None] * ENCODER_COUNT
        panel.joint_encoder_items = [None] * ENCODER_COUNT
        panel.joint_abs_limit_items = [None] * ENCODER_COUNT
        panel.joint_status_items = [None] * ENCODER_COUNT
        panel.joint_min_items = [None] * ENCODER_COUNT
        panel.joint_max_items = [None] * ENCODER_COUNT
        panel.joint_target_spins = [None] * ENCODER_COUNT
        panel.joint_sliders = [None] * ENCODER_COUNT
        panel.joint_send_buttons = [None] * ENCODER_COUNT
        panel._joint_target_user_set = [False] * ENCODER_COUNT
        group_colors = [QtGui.QColor("#ffffff"), QtGui.QColor("#eeeeee")]
        row_to_group_color = {}
        group_start_rows = set()
        for group_index, (row_start, row_end, label) in enumerate(group_ranges):
            target = next((item for item in FINGER_JOINT_RANGES if item[0] == label), None)
            if target is None:
                continue
            _label, start, end = target
            color = group_colors[group_index % len(group_colors)]
            group_start_rows.add(row_start)
            group_mask = ControlScopeModel.joint_group_mask(label, start, end)
            group_widget, group_check = panel._make_group_use_widget(
                label,
                (panel._selected_joint_mask & group_mask) == group_mask,
                "Select this finger group for Manual Joint Channels active scope",
            )
            panel._paint_group_cell_widget(group_widget, color)
            group_check.toggled.connect(
                lambda checked, s=start, e=end: panel._on_joint_group_use_changed(s, e, checked)
            )
            panel.joint_group_use_checks.append(group_check)
            table.setCellWidget(row_start, 0, group_widget)
            if row_end > row_start:
                table.setSpan(row_start, 0, row_end - row_start + 1, 1)
            for row in range(row_start, row_end + 1):
                row_to_group_color[row] = color

        def apply_group_background(item: QtWidgets.QTableWidgetItem, row: int) -> None:
            """Apply the group background command to the UI or backend."""
            color = row_to_group_color.get(row)
            if color is not None:
                item.setBackground(QtGui.QBrush(color))

        def apply_group_widget_background(widget: QtWidgets.QWidget, row: int) -> None:
            """Apply the group widget background command to the UI or backend."""
            color = row_to_group_color.get(row)
            if color is None:
                return
            name = color.name()
            widget.setAutoFillBackground(True)
            palette = widget.palette()
            palette.setColor(QtGui.QPalette.Window, color)
            palette.setColor(QtGui.QPalette.Base, color)
            widget.setPalette(palette)
            widget.setStyleSheet(f"background-color: {name};")

        panel._joint_table_row_by_index = {}
        for row, i in enumerate(joint_rows):
            panel._joint_table_row_by_index[i] = row
            table.setRowHeight(row, 30)
            if row not in group_start_rows:
                filler = panel._make_item("")
                apply_group_background(filler, row)
                table.setItem(row, 0, filler)
            joint_item = panel._make_item(f"J{i:02d}")
            apply_group_background(joint_item, row)
            table.setItem(row, 2, joint_item)
            use_cell, use_check = panel._make_use_checkbox_cell(
                (panel._selected_joint_mask & (1 << i)) != 0,
                f"Use J{i:02d} in Manual Joint Channels active scope",
            )
            apply_group_widget_background(use_cell, row)
            use_check.toggled.connect(lambda checked, idx=i: panel._on_joint_use_changed(idx, checked))
            panel.joint_use_checks[i] = use_check
            table.setCellWidget(row, 1, use_cell)
            status = panel._make_item("*")
            status.setTextAlignment(QtCore.Qt.AlignCenter)
            status.setForeground(QtGui.QBrush(QtGui.QColor("#9aa0a6")))
            enc = panel._make_item("-")
            cur = panel._make_item("-")
            ros2_target = panel._make_item("-")
            ros2_target.setTextAlignment(QtCore.Qt.AlignCenter)
            ros2_target.setForeground(QtGui.QBrush(QtGui.QColor("#9aa0a6")))
            ros2_target.setToolTip("No ROS2 target received")
            abs_limit = panel._make_item("-")
            min_item = panel._make_item(f"{panel._joint_target_min_deg(i):.2f}")
            max_item = panel._make_item(f"{panel._joint_target_max_deg(i):.2f}")
            for item in (status, enc, cur, ros2_target, abs_limit, min_item, max_item):
                apply_group_background(item, row)
            panel.joint_status_items[i] = status
            panel.joint_current_items[i] = cur
            panel.joint_ros2_target_items[i] = ros2_target
            panel.joint_encoder_items[i] = enc
            panel.joint_abs_limit_items[i] = abs_limit
            panel.joint_min_items[i] = min_item
            panel.joint_max_items[i] = max_item
            table.setItem(row, 3, status)
            table.setItem(row, 4, cur)
            table.setItem(row, 5, ros2_target)
            table.setItem(row, 6, enc)
            table.setItem(row, 7, abs_limit)
            table.setItem(row, 8, min_item)
            spin = QtWidgets.QDoubleSpinBox()
            spin.setRange(panel._joint_target_min_deg(i), panel._joint_target_max_deg(i))
            spin.setDecimals(2)
            spin.setSingleStep(1.0)
            spin.valueChanged.connect(lambda value, idx=i: panel._joint_spin_changed(idx, value))
            spin.editingFinished.connect(lambda idx=i: panel._mark_joint_target_user_set(idx))
            apply_group_widget_background(spin, row)
            panel.joint_target_spins[i] = spin
            table.setCellWidget(row, 9, spin)
            table.setItem(row, 10, max_item)
            send = QtWidgets.QPushButton("Send")
            send.clicked.connect(lambda _=False, idx=i: panel.send_joint_row(idx))
            apply_group_widget_background(send, row)
            panel.joint_send_buttons[i] = send
            table.setCellWidget(row, 11, send)
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(int(panel._joint_target_min_deg(i) * 100), int(panel._joint_target_max_deg(i) * 100))
            slider.valueChanged.connect(lambda value, idx=i: panel._joint_slider_changed(idx, value))
            apply_group_widget_background(slider, row)
            panel.joint_sliders[i] = slider
            table.setCellWidget(row, 12, slider)
        table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(0, 152)
        table.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(1, 44)
        table.horizontalHeader().setSectionResizeMode(2, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(2, 58)
        table.horizontalHeader().setSectionResizeMode(3, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(3, 32)
        table.horizontalHeader().setSectionResizeMode(4, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(4, 70)
        table.horizontalHeader().setSectionResizeMode(5, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(5, 92)
        table.horizontalHeader().setSectionResizeMode(6, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(6, 68)
        table.horizontalHeader().setSectionResizeMode(7, QtWidgets.QHeaderView.Fixed)
        # 双电机关节需要同时显示两台舵机的控制 ABS 范围，固定宽度避免高频刷新时跳动。
        table.setColumnWidth(7, 250)
        table.horizontalHeader().setSectionResizeMode(8, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(8, 64)
        table.horizontalHeader().setSectionResizeMode(9, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(9, 78)
        table.horizontalHeader().setSectionResizeMode(10, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(10, 60)
        table.horizontalHeader().setSectionResizeMode(11, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(11, 82)
        table.horizontalHeader().setSectionResizeMode(12, QtWidgets.QHeaderView.Stretch)
        table.setColumnWidth(12, 420)
        return table



class MotorChannelTable(QtWidgets.QTableWidget):
    """Builds the Motor Channels table and registers its widgets on the panel."""
    @staticmethod
    def build(panel) -> QtWidgets.QTableWidget:
        """Build the motor table UI section."""
        motor_rows, group_ranges = ControlScopeModel.motor_rows_by_finger_group()
        antagonist_groups = ControlScopeModel.antagonist_motor_row_groups(motor_rows)
        antagonist_by_row = {}
        antagonist_start_rows = set()
        for start, end, motors, joint_idx in antagonist_groups:
            antagonist_start_rows.add(start)
            group = (start, end, motors, joint_idx)
            for row in range(start, end + 1):
                antagonist_by_row[row] = group
        table = MotorChannelTable(len(motor_rows), 16)
        table.setHorizontalHeaderLabels([
            "Group", "Use", "Motor", "BUS", "Route", "Turn", "RawNow", "SavedRaw", "AbsNow", "Load",
            "Target(rel)", "Action", "-", "Raw Slider", "+", "Joint Angle",
        ])
        table.verticalHeader().setVisible(False)
        table.setAlternatingRowColors(False)
        panel.motor_turn_items = [None] * MOTOR_COUNT
        panel.motor_bus_items = [None] * MOTOR_COUNT
        panel.motor_route_items = [None] * MOTOR_COUNT
        panel.motor_raw_items = [None] * MOTOR_COUNT
        panel.motor_flash_raw_items = [None] * MOTOR_COUNT
        panel.motor_abs_items = [None] * MOTOR_COUNT
        panel.motor_load_items = [None] * MOTOR_COUNT
        panel.motor_joint_items = [None] * MOTOR_COUNT
        panel.motor_target_spins = [None] * MOTOR_COUNT
        panel.motor_sliders = [None] * MOTOR_COUNT
        panel.motor_send_buttons = [None] * MOTOR_COUNT
        panel.motor_step_buttons = [None] * MOTOR_COUNT
        panel._motor_target_user_set = [False] * MOTOR_COUNT
        group_colors = [QtGui.QColor("#ffffff"), QtGui.QColor("#eeeeee")]
        row_to_group_label = {}
        row_to_group_color = {}
        group_start_rows = set()
        for group_index, (start, end, label) in enumerate(group_ranges):
            group_start_rows.add(start)
            color = group_colors[group_index % len(group_colors)]
            group_mask = 0
            for row in range(start, end + 1):
                if 0 <= row < len(motor_rows):
                    motor_idx = int(motor_rows[row])
                    if _motor_channel_valid(motor_idx):
                        group_mask |= (1 << motor_idx)
            group_widget, group_check = panel._make_group_use_widget(
                label,
                group_mask != 0 and (panel._selected_motor_mask & group_mask) == group_mask,
                "Select this finger group for Manual Motor Channels active scope",
            )
            panel._paint_group_cell_widget(group_widget, color)
            group_check.toggled.connect(
                lambda checked, s=start, e=end, rows=motor_rows: panel._on_motor_group_use_changed(rows[s:e + 1], checked)
            )
            panel.motor_group_use_checks.append(group_check)
            table.setCellWidget(start, 0, group_widget)
            if end > start:
                table.setSpan(start, 0, end - start + 1, 1)
            for row in range(start, end + 1):
                row_to_group_label[row] = label
                row_to_group_color[row] = color

        def apply_group_background(item: QtWidgets.QTableWidgetItem, row: int) -> None:
            """Apply the group background command to the UI or backend."""
            color = row_to_group_color.get(row)
            if color is not None:
                item.setBackground(QtGui.QBrush(color))

        def apply_group_widget_background(widget: QtWidgets.QWidget, row: int) -> None:
            """Apply the group widget background command to the UI or backend."""
            color = row_to_group_color.get(row)
            if color is None:
                return
            name = color.name()
            widget.setAutoFillBackground(True)
            palette = widget.palette()
            palette.setColor(QtGui.QPalette.Window, color)
            palette.setColor(QtGui.QPalette.Base, color)
            widget.setPalette(palette)
            widget.setStyleSheet(f"background-color: {name};")

        for row, i in enumerate(motor_rows):
            group_label = row_to_group_label.get(row, "")
            if row not in group_start_rows and group_label:
                filler = panel._make_item("")
                apply_group_background(filler, row)
                table.setItem(row, 0, filler)
            joint_idx = MOTOR_TO_JOINT_INDEX[i] if i < len(MOTOR_TO_JOINT_INDEX) else -1
            antagonist = antagonist_by_row.get(row)
            if antagonist is None:
                motor_item = panel._make_item(f"M{i:02d}")
                apply_group_background(motor_item, row)
                if group_label:
                    motor_item.setToolTip(
                        f"{group_label}; mapped joint J{int(joint_idx):02d}"
                        if int(joint_idx) >= 0
                        else group_label
                    )
                table.setItem(row, 2, motor_item)
                use_cell, use_check = panel._make_use_checkbox_cell(
                    (panel._selected_motor_mask & (1 << i)) != 0,
                    f"Use M{i:02d} in Manual Motor Channels active scope",
                )
                apply_group_widget_background(use_cell, row)
                use_check.toggled.connect(lambda checked, idx=i: panel._on_motor_use_changed(idx, checked))
                panel.motor_use_checks[i] = use_check
                table.setCellWidget(row, 1, use_cell)
            elif row in antagonist_start_rows:
                pair_start, pair_end, pair_motors, pair_joint = antagonist

                pair_cell = QtWidgets.QWidget()
                pair_layout = QtWidgets.QGridLayout(pair_cell)
                pair_layout.setContentsMargins(0, 0, 0, 0)
                pair_layout.setHorizontalSpacing(0)
                pair_layout.setVerticalSpacing(0)
                pair_layout.setColumnMinimumWidth(0, 44)
                pair_layout.setColumnMinimumWidth(1, 58)
                pair_layout.setColumnStretch(0, 0)
                pair_layout.setColumnStretch(1, 1)
                for pair_row, motor_idx in enumerate(pair_motors):
                    check_row, use_check = panel._make_use_checkbox_cell(
                        (panel._selected_motor_mask & (1 << motor_idx)) != 0,
                        f"Use M{motor_idx:02d} in Manual Motor Channels active scope",
                    )
                    use_check.toggled.connect(
                        lambda checked, idx=motor_idx: panel._on_motor_use_changed(idx, checked)
                    )
                    panel.motor_use_checks[motor_idx] = use_check
                    pair_layout.addWidget(check_row, pair_row, 0)

                    label = QtWidgets.QLabel(f"M{motor_idx:02d}")
                    label.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
                    label.setContentsMargins(4, 0, 0, 0)
                    label.setToolTip(
                        f"J{pair_joint:02d} antagonist motor; feedback and commands remain independent"
                    )
                    font = label.font()
                    font.setBold(True)
                    label.setFont(font)
                    pair_layout.addWidget(label, pair_row, 1)
                    pair_layout.setRowStretch(pair_row, 1)
                apply_group_widget_background(pair_cell, row)
                table.setCellWidget(row, 1, pair_cell)
                table.setSpan(pair_start, 1, pair_end - pair_start + 1, 2)
            turn = panel._make_item("-")
            bus = panel._make_item("-")
            route = panel._make_item("Waiting")
            raw_now = panel._make_item("-")
            flash_raw = panel._make_item("-")
            abs_now = panel._make_item("-")
            load = panel._make_item("-")
            joint = panel._make_item("-")
            for item in (bus, route, turn, raw_now, flash_raw, abs_now, load, joint):
                apply_group_background(item, row)
            panel.motor_bus_items[i] = bus
            panel.motor_route_items[i] = route
            panel.motor_turn_items[i] = turn
            panel.motor_raw_items[i] = raw_now
            panel.motor_flash_raw_items[i] = flash_raw
            panel.motor_abs_items[i] = abs_now
            panel.motor_load_items[i] = load
            panel.motor_joint_items[i] = joint
            table.setItem(row, 3, bus)
            table.setItem(row, 4, route)
            table.setItem(row, 5, turn)
            table.setItem(row, 6, raw_now)
            table.setItem(row, 7, flash_raw)
            table.setItem(row, 8, abs_now)
            table.setItem(row, 9, load)
            spin = QtWidgets.QSpinBox()
            spin.setRange(MOTOR_ABS_CMD_MIN, MOTOR_ABS_CMD_MAX)
            spin.setSingleStep(10)
            spin.valueChanged.connect(lambda value, idx=i: panel._motor_spin_changed(idx, value))
            spin.editingFinished.connect(lambda idx=i: panel._mark_motor_target_user_set(idx))
            apply_group_widget_background(spin, row)
            panel.motor_target_spins[i] = spin
            table.setCellWidget(row, 10, spin)
            send = QtWidgets.QPushButton("Send")
            send.clicked.connect(lambda _=False, idx=i: panel.send_motor_row(idx))
            apply_group_widget_background(send, row)
            panel.motor_send_buttons[i] = send
            table.setCellWidget(row, 11, send)
            minus = QtWidgets.QPushButton("-")
            minus.setMaximumWidth(34)
            minus.clicked.connect(lambda _=False, idx=i: panel._step_motor_target(idx, -1))
            apply_group_widget_background(minus, row)
            table.setCellWidget(row, 12, minus)
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setMinimumWidth(72)
            slider.setRange(MOTOR_TARGET_SCALE_LO, MOTOR_TARGET_SCALE_HI)
            slider.sliderPressed.connect(lambda idx=i: panel._on_motor_slider_press(idx))
            slider.sliderReleased.connect(lambda idx=i: panel._on_motor_slider_release(idx))
            slider.valueChanged.connect(lambda value, idx=i: panel._motor_slider_changed(idx, value))
            apply_group_widget_background(slider, row)
            panel.motor_sliders[i] = slider
            table.setCellWidget(row, 13, slider)
            plus = QtWidgets.QPushButton("+")
            plus.setMaximumWidth(34)
            plus.clicked.connect(lambda _=False, idx=i: panel._step_motor_target(idx, 1))
            apply_group_widget_background(plus, row)
            table.setCellWidget(row, 14, plus)
            panel.motor_step_buttons[i] = (minus, plus)
            table.setItem(row, 15, joint)
            if not _motor_channel_valid(i):
                table.setRowHidden(row, True)
        table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(0, 152)
        table.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(1, 44)
        table.horizontalHeader().setSectionResizeMode(2, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(2, 58)
        table.horizontalHeader().setSectionResizeMode(3, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(3, 54)
        table.horizontalHeader().setSectionResizeMode(4, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(4, 94)
        for col in (6, 7, 8):
            table.horizontalHeader().setSectionResizeMode(col, QtWidgets.QHeaderView.Fixed)
            table.setColumnWidth(col, 76)
        table.horizontalHeader().setSectionResizeMode(10, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(10, 126)
        table.horizontalHeader().setSectionResizeMode(13, QtWidgets.QHeaderView.Stretch)
        table.setColumnWidth(13, 150)
        table.horizontalHeader().setSectionResizeMode(15, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(15, 82)
        return table
