"""ui_control_panels.py - MMHand Desktop 模块契约。

职责：作为模块化 ControlPanel 的稳定门面组合各控制能力。
线程：Qt GUI 主线程执行；耗时工作必须委托后台服务，禁止阻塞界面事件循环。
输入：用户操作、HandModel 只读快照以及 HandBackend/服务返回结果。
输出：Qt 控件状态、用户提示和经门面提交的应用命令。
状态所有权：仅拥有本视图控件和瞬时交互状态；设备真值由 core 层拥有。
禁止依赖：禁止直接导入串口通信和底层控制实现，禁止在 UI 中复制协议或安全规则。
"""

from .ui_control_common import (
    ENCODER_COUNT,
    FF_TEST_DEFAULT_STEP_S,
    HandModel,
    JointTargetModel,
    List,
    MOTOR_COUNT,
    Optional,
    QtCore,
    QtWidgets,
    ReplayView,
    Signal,
    TEST_DEFAULT_HOLD_S,
    TEST_TICK_MS,
    TRI_TEST_DEFAULT_FREQ_HZ,
    Tuple,
    WORKSPACE_MAPPING_VERSION,
    WORKSPACE_URDF_CRC,
    ControlScopeModel,
)
from .ui_control_calibration import CalibrationWorkflowController
from .ui_control_commands import ControlCommandController
from .ui_control_manual import ControlPanelManualMixin
from .ui_control_state import ControlPanelStateMixin
from .ui_control_tests import ManualTestPanelMixin
from .ui_control_view import ControlPanelViewMixin


class ControlPanel(
    ControlPanelViewMixin,
    ControlPanelManualMixin,
    ControlCommandController,
    CalibrationWorkflowController,
    ControlPanelStateMixin,
    ManualTestPanelMixin,
    QtWidgets.QWidget,
):
    """Root control panel facade that composes modular views and controllers."""

    startup_device_check_finished = Signal(int, str, bool, str)

    def __init__(self, context, parent=None):
        """Initialize this control panel component."""
        super().__init__(parent)
        self.context = context
        self.backend = context.backend
        self._updating = False
        self._interaction_enabled = False
        self._control_widgets: List[QtWidgets.QWidget] = []
        self._state: Optional[HandModel] = None
        self._estop_latched = False
        self._estop_in_progress = False

        self._init_joint_panel_state()
        self._init_scope_and_calibration_state()
        self._init_motor_panel_state()
        self._init_test_panel_state()
        self._init_body_state()

        self._build()
        self.startup_device_check_finished.connect(
            self._on_startup_device_check_finished
        )
        self._apply_interaction_enabled(False)
        self._apply_control_mode_policy()

    def _init_joint_panel_state(self) -> None:
        """Initialize joint table state and target caches."""
        self.joint_current_items: List[QtWidgets.QTableWidgetItem] = []
        self.joint_encoder_items: List[QtWidgets.QTableWidgetItem] = []
        self.joint_abs_limit_items: List[QtWidgets.QTableWidgetItem] = []
        self.joint_status_items: List[QtWidgets.QTableWidgetItem] = []
        self.joint_min_items: List[QtWidgets.QTableWidgetItem] = []
        self.joint_max_items: List[QtWidgets.QTableWidgetItem] = []
        self.joint_target_spins: List[QtWidgets.QDoubleSpinBox] = []
        self.joint_sliders: List[QtWidgets.QSlider] = []
        self.joint_send_buttons: List[QtWidgets.QPushButton] = []
        self._joint_target_user_set: List[bool] = []
        self._joint_target_initialized = False
        self._joint_run_states = [False] * ENCODER_COUNT
        self._motors_by_joint = ControlScopeModel.motors_by_joint_index()
        # 21 路磁编共用 S3 的 SPI MISO/MOSI。公共返回线被拉低时，固件可能把
        # 0x0000 当作合法零角度并继续上报全在线；在上位机侧对这一特征做短时去抖。
        self._encoder_common_zero_first_ts = 0.0
        self._encoder_common_zero_last_sample_ts = 0.0
        self._encoder_common_zero_recovery_frames = 0
        self._encoder_common_bus_fault = False

    def _init_scope_and_calibration_state(self) -> None:
        """Initialize calibration state and active scope selections."""
        self._last_calib_status: Optional[str] = None
        self._calibration_warning_popup_mask = 0
        self._last_calib_snapshot_signature: Optional[str] = None
        self._last_joint_calib_valid_mask: Optional[int] = None
        self._startup_device_check_epoch = 0
        self._startup_device_check_running = False
        self._startup_missing_warning_shown = False
        self._pending_calib_joint_mask = 0
        self._pending_calib_motor_mask = 0
        # 记录本轮开始前各 Jxx 的结果签名；运行中只把新产生或变化的
        # 成功结果增量写入快照，避免把上轮旧结果误判成本轮成功。
        self._pending_calib_result_baselines = {}
        default_motor_mask, default_joint_mask, _default_require_can = self._scope_range(0, 3, True)
        self._selected_joint_mask = int(default_joint_mask)
        self._selected_motor_mask = int(default_motor_mask)
        self._teleop_selected_joint_mask = int(default_joint_mask)
        self._teleop_selected_motor_mask = int(default_motor_mask)
        self._scope_update_guard = False
        self._joint_target_model = JointTargetModel(
            WORKSPACE_MAPPING_VERSION,
            WORKSPACE_URDF_CRC,
        )
        self.joint_use_checks: List[Optional[QtWidgets.QCheckBox]] = [None] * ENCODER_COUNT
        self.joint_group_use_checks: List[QtWidgets.QCheckBox] = []
        self.teleop_joint_use_checks: List[Optional[QtWidgets.QCheckBox]] = [None] * ENCODER_COUNT
        self.teleop_group_use_checks: List[QtWidgets.QCheckBox] = []
        self.teleop_hand_use_check: Optional[QtWidgets.QCheckBox] = None

    def _init_motor_panel_state(self) -> None:
        """Initialize motor table state and direct-control caches."""
        self.motor_turn_items: List[QtWidgets.QTableWidgetItem] = []
        self.motor_bus_items: List[QtWidgets.QTableWidgetItem] = []
        self.motor_route_items: List[QtWidgets.QTableWidgetItem] = []
        self.motor_raw_items: List[QtWidgets.QTableWidgetItem] = []
        # FlashRaw 用于比较当前单圈位置和最近一次可信 NVS 锚点。
        self.motor_flash_raw_items: List[QtWidgets.QTableWidgetItem] = []
        self.motor_abs_items: List[QtWidgets.QTableWidgetItem] = []
        self.motor_load_items: List[QtWidgets.QTableWidgetItem] = []
        self.motor_joint_items: List[QtWidgets.QTableWidgetItem] = []
        self.motor_target_spins: List[QtWidgets.QSpinBox] = []
        self.motor_sliders: List[QtWidgets.QSlider] = []
        self.motor_send_buttons: List[QtWidgets.QPushButton] = []
        self._motor_target_user_set: List[bool] = []
        self._motor_target_initialized = False
        self.motor_use_checks: List[Optional[QtWidgets.QCheckBox]] = [None] * MOTOR_COUNT
        self.motor_group_use_checks: List[QtWidgets.QCheckBox] = []
        self.abs_limit_lock_toggle_btn: Optional[QtWidgets.QPushButton] = None
        # UI relative multi-turn targets. Device command = origin feedback + this value.
        self._motor_abs_pos: List[Optional[int]] = [None] * MOTOR_COUNT
        # Servo absolute feedback captured when direct control is synced or held.
        self._motor_session_origin_abs: List[Optional[int]] = [None] * MOTOR_COUNT
        self._motor_dir_hint: List[int] = [0] * MOTOR_COUNT
        self._motor_dir_hint_pending_clear: List[bool] = [False] * MOTOR_COUNT
        self._motor_dir_hint_stable_count: List[int] = [0] * MOTOR_COUNT
        self._motor_last_slider: List[Optional[int]] = [None] * MOTOR_COUNT
        self._motor_slider_dragging: List[bool] = [False] * MOTOR_COUNT
        self._motor_last_sent_abs: List[Optional[int]] = [None] * MOTOR_COUNT
        # 下位机直控准入是异步事务；记录已经展示过的 generation，避免每个
        # SYSTEM_STATUS 周期重复提示同一次执行/拒绝结果。
        self._last_reported_direct_generation: Optional[int] = None
        self._motor_run_states = [False] * MOTOR_COUNT
        self._last_state_transition_count: Optional[int] = None
        self._last_multi_turn_unknown_bitmap: Optional[int] = None
        self._turn_recovery_popup_episode_active = False
        self._turn_recovery_popup_episode_mask = 0
        self._servo_turn_overflow_popup_episode_active = False
        self._servo_turn_overflow_popup_episode_mask = 0
        self._last_servo_online_bitmap: Optional[int] = None
        # 同一轮扫描错误只弹一次；路由恢复正常后清空签名，允许下一轮新故障再次提示。
        self._last_servo_route_warning_signature = None
        self._last_encoder_online_bitmap: Optional[int] = None
        self._last_encoder_warn_bitmap: Optional[int] = None
        self._last_encoder_fault_bitmap: Optional[int] = None
        self._feedback_abs_limit_popup_episode_active = False
        self._abs_limit_warning_unlocked_mask = 0

    def _init_test_panel_state(self) -> None:
        """Initialize manual test timers and cached test parameters."""
        self._test_running = False
        self._ff_test_running = False
        self._tri_test_running = False
        self._test_timer = QtCore.QTimer(self)
        self._test_timer.setInterval(TEST_TICK_MS)
        self._test_timer.timeout.connect(self._test_tick)
        self._ff_test_timer = QtCore.QTimer(self)
        self._ff_test_timer.setInterval(TEST_TICK_MS)
        self._ff_test_timer.timeout.connect(self._ff_test_tick)
        self._tri_test_timer = QtCore.QTimer(self)
        self._tri_test_timer.setInterval(TEST_TICK_MS)
        self._tri_test_timer.timeout.connect(self._tri_test_tick)
        self._p4_rope_init_phase = "idle"
        self._p4_rope_init_joint_mask = 0
        self._p4_rope_init_requested_at = 0.0
        self._p4_rope_init_transition_base = 0
        self._p4_rope_init_sequence_base = 0
        self._test_active_rows: List[Tuple[int, float, float, float, float]] = []
        self._test_t0 = 0.0
        self._test_hold_s = float(TEST_DEFAULT_HOLD_S)
        self._ff_test_joint_idx = 0
        self._ff_test_targets: List[float] = []
        self._ff_test_step_s = float(FF_TEST_DEFAULT_STEP_S)
        self._ff_test_idx = 0
        self._ff_test_t_accum = 0.0
        self._ff_test_last_ts = 0.0
        self._tri_test_joint_idx = 0
        self._tri_test_min_deg = 0.0
        self._tri_test_max_deg = 0.0
        self._tri_test_freq_hz = float(TRI_TEST_DEFAULT_FREQ_HZ)
        self._tri_test_t0 = 0.0

    def _init_body_state(self) -> None:
        """Initialize active body/view bookkeeping."""
        self._body_widget: Optional[QtWidgets.QWidget] = None
        self._replay_view: Optional[ReplayView] = None
        self._app_mode = "manual"
        self.scope_combo: Optional[QtWidgets.QComboBox] = None
        self._scope_selection_index = 0
        self._last_manual_tab_index = 0

    def refresh_active_scope(self) -> None:
        """Refresh active scope selection and publish it to the backend."""
        self._on_active_scope_changed()

    def active_motor_mask(self) -> int:
        """Return the current active motor mask selected in the UI."""
        return self._active_motor_mask_from_ui()

    def active_scope_record_fields(self):
        """Return active scope label and masks for run-data recording."""
        label = self._active_scope_label_from_ui()
        data = self._current_active_scope_tuple()
        if data is None:
            return label, "", "", ""
        motor_mask = f"0x{int(data[0]):08X}"
        joint_mask = f"0x{int(data[1]):08X}"
        require_can = 1 if bool(data[2]) else 0
        return label, motor_mask, joint_mask, require_can

    def joint_encoder_status(self, idx: int, state: Optional[HandModel] = None) -> Optional[str]:
        """Return the display status for one joint encoder."""
        return self._joint_encoder_status(idx, state)

    def joint_angle_feedback_valid(self, idx: int, state: Optional[HandModel] = None) -> bool:
        """Return whether the joint angle feedback is fresh and usable."""
        return self._joint_angle_feedback_valid(idx, state)

    def joint_angle_display_calibrated(self, idx: int, state: Optional[HandModel] = None) -> bool:
        """Return whether the joint angle display should be treated as calibrated."""
        return self._joint_angle_display_calibrated(idx, state)
