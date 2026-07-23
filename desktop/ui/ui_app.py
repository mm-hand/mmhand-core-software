import os
import json
import threading
import time
from typing import Callable, Dict, List, Optional

from desktop.paths import (
    CALIB_DEG_PATH,
    CALIB_LOAD_SNAPSHOT_PATH,
    HAND_GEO_PATH,
    MANUS_CALIBRATION_PATH,
    MOTOR_MID_CALIB_PATH,
    POSE_FIST_PATH,
    PROJECT_ROOT,
    RAW_IK_Q_TARGET_PATH,
)
from desktop.core.auto_calib_logger import AUTO_CALIB_LOG_DIR
from desktop.core.data_recording import DataSessionRecorder
from desktop.core.data_recording.motion_sampler import (
    MOTION_RECORD_PERIOD_SECONDS,
    MotionCommandSampler,
)
from desktop.core.data_recording.topic_schema import (
    DEFAULT_CSV_TOPICS,
    DEFAULT_TOPICS,
    TACTILE_FULL_TOPICS,
)
from desktop.core.data_models import HandModel
from desktop.core.hand_backend import HandBackend, list_ports
from desktop.core.protocol import (
    CALIB_DIAG_PHASE_DUAL_EXCHANGE,
    CALIB_DIAG_PHASE_DUAL_PRELOAD,
    CALIB_DIAG_PHASE_FIND_MAX,
    CALIB_DIAG_PHASE_INIT,
    CALIB_DIAG_PHASE_RELEASE,
    CALIB_DIAG_PHASE_RETURN_ZERO,
    CALIB_DIAG_PHASE_SEQUENCE,
    CALIB_DIAG_REASON_ABS_LIMIT_INVALID,
    CALIB_DIAG_REASON_ACTIVE_SCOPE_EMPTY,
    CALIB_DIAG_REASON_ANGLE_NOT_MOVING,
    CALIB_DIAG_REASON_CAN_FEEDBACK_TIMEOUT,
    CALIB_DIAG_REASON_CONTROL_LOST,
    CALIB_DIAG_REASON_DUPLICATE_JOINT,
    CALIB_DIAG_REASON_ENCODER_ERROR,
    CALIB_DIAG_REASON_FEEDBACK_LOST,
    CALIB_DIAG_REASON_HOLD_TARGET_FAILED,
    CALIB_DIAG_REASON_INTERNAL_ERROR,
    CALIB_DIAG_REASON_JOINT_LIST_INVALID,
    CALIB_DIAG_REASON_JOINT_STEP_FAILED,
    CALIB_DIAG_REASON_LOAD_NOT_REACHED,
    CALIB_DIAG_REASON_OVERLOAD,
    CALIB_DIAG_REASON_SERVO_FEEDBACK_TIMEOUT,
    CALIB_DIAG_REASON_SERVO_OFFLINE,
    CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED,
    CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
    CALIB_DIAG_REASON_TASK_START_FAILED,
    CALIB_DIAG_REASON_TELEMETRY_TIMEOUT,
    CALIB_DIAG_REASON_TURN_UNKNOWN,
    CALIB_DIAG_REASON_WRAP_TOO_LARGE,
    CALIB_DIAG_REASON_WRONG_DIRECTION,
    CALIB_DIAG_REASON_ZERO_CROSS_INVALID,
    CALIB_DIAG_REASON_ZERO_RETURN_TIMEOUT,
    DUAL_TENDON_JOINT_INDICES,
    ENCODER_COUNT,
    FEEDBACK_ADMISSION_REJECTED,
    FEEDBACK_REJECT_ABS_LIMIT,
    FEEDBACK_REJECT_EVENT_QUEUE,
    FEEDBACK_REJECT_MANUAL_REFERENCE,
    FEEDBACK_REJECT_MODE_SWITCH,
    FEEDBACK_REJECT_MULTITURN,
    FEEDBACK_REJECT_STATE,
    FEEDBACK_REJECT_WORKSPACE,
    MOTOR_COUNT,
    VALID_MOTOR_MASK,
    SYSTEM_FAULT_CALIBRATION,
    SYSTEM_FAULT_CAN_OFFLINE,
    SYSTEM_FAULT_JOINT16_DUAL,
    SYSTEM_FAULT_MULTITURN_UNKNOWN,
    SYSTEM_FAULT_OVERLOAD,
    SYSTEM_FAULT_SERVO_OFFLINE,
    SYSTEM_STATE_PAUSED,
    SYSTEM_STATE_RECOVERY_REQUIRED,
    SYSTEM_STATE_ROPE_INIT_RUNNING,
    SYSTEM_STATE_RUN_DIRECT,
    SYSTEM_STATE_RUN_FEEDBACK,
)
from desktop.core.session_log import close_session_log, log_session
from .ui_context import HandGuiContext
from .ui_control_panels import ControlPanel
from .ui_joint_plot_view import JointRealtimePlotView
from .ui_qt_compat import QtCore, QtGui, QtWidgets, Signal, ensure_app
from .ui_services import LatestStateBuffer, clone_hand_model
from .ui_style import apply_main_window_defaults, configure_qt_application
from .ui_task_runtime import TaskRuntimeWindow
from desktop.core.motor_mapping import MOTOR_TO_JOINT_INDEX
from .ui_windows import DisplayWindowManager


PROJECT_ROOT_DIR = str(PROJECT_ROOT)
DISPLAY_POLL_MS = 33
CAMERA_UI_POLL_MS = 50
STATE_RECORD_POLL_MS = 50
SYSTEM_STATUS_STALE_AFTER_S = 1.0
FAULT_STATUS_STALE_AFTER_S = 1.0
FINGER_NAMES = ["thumb", "index", "middle", "ring", "pinky"]


class HandGUI(QtWidgets.QMainWindow):
    test_recording_stop_finished = Signal(str, str, str)
    replay_event_received = Signal(str, str)

    def __init__(
        self,
        backend: HandBackend,
        title_suffix: str = "",
        current_port: Optional[str] = None,
        current_mode: Optional[str] = None,
        on_close_extra: Optional[Callable[[], None]] = None,
        action_callback: Optional[Callable[[str], None]] = None,
        replay_service=None,
        pose_generate: bool = False,
        pose_generate_path: Optional[str] = None,
        tactile_port: Optional[str] = None,
        ros2_enabled: bool = False,
        ros_topic: str = "/mmhand/joint_targets",
    ):
        self._qt_app = ensure_app()
        configure_qt_application(self._qt_app)
        super().__init__()
        self.backend = backend if isinstance(backend, HandBackend) else HandBackend(backend)
        self.current_port = current_port or ""
        self.current_mode = self._normalize_mode(current_mode or "Monitor")
        self._on_close_extra = on_close_extra
        self._action_callback = action_callback
        if replay_service is None:
            from desktop.algorithm.replay_service import ReplayService
            replay_service = ReplayService(self.backend)
        self.replay_service = replay_service
        self._pose_generate = bool(pose_generate)
        self._pose_generate_path = pose_generate_path or str(POSE_FIST_PATH)
        self.tactile_port = tactile_port or ""
        self.ros2_enabled = bool(ros2_enabled)
        self.ros_topic = str(ros_topic)
        self._tactile_state: Optional[HandModel] = None
        self._state: Optional[HandModel] = None
        self._state_buffer = LatestStateBuffer()
        self._state_buffer_version = 0
        self._data_logger = DataSessionRecorder(flush_every=10)
        # P4控制仍为100 Hz；状态与连续动作只以20 Hz落盘，降低CSV/HDF5体积。
        self._motion_record_sampler = MotionCommandSampler(
            MOTION_RECORD_PERIOD_SECONDS
        )
        self._last_state_record_signature = None
        self._test_recording_stop_lock = threading.Lock()
        self._test_recording_stop_thread: Optional[threading.Thread] = None
        self.test_recording_stop_finished.connect(self._on_test_recording_stop_finished)
        self.replay_event_received.connect(self._on_replay_event)
        self._recording_status_note = ""
        self._recording_csv_path = ""
        self._last_recorded_proto_ack_sequence = 0
        self._recording_format_options = {"csv": True, "hdf5": True, "rosbag": False, "tactile_full": False}
        self._record_tactile_full = False
        self._last_draw = 0.0
        self._draw_interval = 0.033
        self._last_status_refresh = 0.0
        self._pending_status_text = ""
        self._last_overload_popup_mask = 0
        self._last_feedback_collision_armed_mask: Optional[int] = None
        self._last_feedback_collision_hold_mask: Optional[int] = None
        self._fault_popup_episode_active = False
        self._fault_dialog_active = False
        self._fault_stop_sent = False
        self._fault_previous_runtime_active = False
        self._task_runtime_window: Optional[TaskRuntimeWindow] = None
        self.context = HandGuiContext(self)

        self.backend.register_update_callback(self._on_state_update)
        self.backend.register_joint_target_callback(self._on_joint_target_command)
        if self.replay_service is not None:
            # Replay工作在线程中运行，必须通过Qt信号回到主线程更新UI和记录器。
            self.replay_service.set_event_sink(self.replay_event_received.emit)
        self.setWindowTitle(f"{ENCODER_COUNT}-DOF Dexterous Hand Control System" + (f" - {title_suffix}" if title_suffix else ""))
        apply_main_window_defaults(self)
        self._build_ui()
        self._display_windows = DisplayWindowManager(self)
        self._attach_mode_menu()
        self._display_windows.attach_menu(self.menuBar())
        self._attach_device_menu()
        self._display_windows.apply_main_geometry()
        shortcut_cls = getattr(QtGui, "QShortcut", getattr(QtWidgets, "QShortcut", None))
        self._esc_shortcut = shortcut_cls(QtGui.QKeySequence("Esc"), self) if shortcut_cls is not None else None
        if self._esc_shortcut is not None:
            self._esc_shortcut.activated.connect(self._on_keyboard_stop)

        self._display_timer = QtCore.QTimer(self)
        self._display_timer.timeout.connect(self._display_poll)
        self._display_timer.start(DISPLAY_POLL_MS)
        self._recording_timer = QtCore.QTimer(self)
        self._recording_timer.timeout.connect(self._recording_poll)
        self._recording_timer.start(STATE_RECORD_POLL_MS)
        self._camera_timer = QtCore.QTimer(self)
        self._camera_timer.timeout.connect(self._display_windows.update_cameras)
        self._camera_timer.start(CAMERA_UI_POLL_MS)
        if self.tactile_port:
            self._start_tactile_port(self.tactile_port)
        QtCore.QTimer.singleShot(250, self._clear_live_displays)

    def _build_ui(self) -> None:
        toolbar = QtWidgets.QToolBar("Main")
        toolbar.setMovable(False)
        self.addToolBar(toolbar)

        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setMinimumWidth(240)
        self.refresh_ports()
        self.port_combo.currentIndexChanged.connect(lambda _idx: self._on_port_selection_changed())
        self.connect_btn = QtWidgets.QPushButton("Refresh")
        self.connect_btn.clicked.connect(lambda _checked=False: self.refresh_ports())
        self.save_data_check = QtWidgets.QCheckBox("Save Data")
        self.save_data_check.setChecked(False)
        self.save_data_check.toggled.connect(self._on_toggle_save_data)
        self.comm_conn_light = self._make_toolbar_connection_light()
        self.start_indicator = QtWidgets.QLabel("STOP")
        self.start_indicator.setAlignment(QtCore.Qt.AlignCenter)
        self.start_indicator.setMinimumWidth(56)
        self._refresh_start_indicator()

        for widget in [
            QtWidgets.QLabel("Port:"), self.port_combo, self.comm_conn_light,
            self.connect_btn, self.save_data_check, self.start_indicator,
        ]:
            toolbar.addWidget(widget)

        toolbar_spacer = QtWidgets.QWidget()
        toolbar_spacer.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        toolbar.addWidget(toolbar_spacer)

        initial_status = self._pending_status_text or ("Ready" if self.is_comm_connected() else "Disconnected")
        self.status_label = QtWidgets.QLabel(initial_status)
        self._build_top_status_strip(toolbar)

        central = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.setCentralWidget(central)
        self.control_panel = ControlPanel(self.context)
        self.control_panel.sync_app_mode(self.current_mode)
        central.addWidget(self.control_panel)
        central.setStretchFactor(0, 1)

        self.statusBar().addWidget(self.status_label, 1)

        self.joint_plot = JointRealtimePlotView(self.context, self)

    def _build_top_status_strip(self, toolbar: QtWidgets.QToolBar) -> None:
        strip = QtWidgets.QWidget()
        strip.setObjectName("TopStatusStrip")
        strip.setSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Preferred)
        layout = QtWidgets.QHBoxLayout(strip)
        layout.setContentsMargins(8, 2, 8, 2)
        layout.setSpacing(5)

        self.fault_label = self._make_top_status_label("SYS -", 132)
        self.calib_label = self._make_top_status_label("CALIB -", 82)
        self.overload_label = self._make_top_status_label("OL -", 96)
        self.feedback_hold_label = self._make_top_status_label("COLLISION -", 180)
        self.turn_label = self._make_top_status_label("TURN -", 98)
        self.delay_label = self._make_top_status_label("--", 62)

        for label in [
            self.fault_label,
            self.calib_label,
            self.overload_label,
            self.feedback_hold_label,
            self.turn_label,
            self.delay_label,
        ]:
            layout.addWidget(label)

        strip.setStyleSheet(
            "QWidget#TopStatusStrip {"
            "background:#f6f8fa;"
            "border:1px solid #d0d7de;"
            "border-radius:4px;"
            "}"
        )
        self.top_status_strip = strip
        toolbar.addWidget(strip)

    def _make_top_status_label(self, text: str, min_width: int) -> QtWidgets.QLabel:
        label = QtWidgets.QLabel(text)
        label.setAlignment(QtCore.Qt.AlignCenter)
        label.setMinimumHeight(24)
        label.setMinimumWidth(min_width)
        self._style_top_status_label(label, "neutral")
        return label

    @staticmethod
    def _style_top_status_label(label: QtWidgets.QLabel, state: str = "neutral") -> None:
        palette = {
            "neutral": ("#24292f", "#eef2f5", "#d0d7de", 600),
            "ok": ("#1b5e20", "#e8f5e9", "#81c784", 700),
            "lag": ("#8a5a00", "#fff8e1", "#ffcc80", 700),
            "warn": ("#c62828", "#ffebee", "#ef9a9a", 700),
        }
        fg, bg, border, weight = palette.get(state, palette["neutral"])
        label.setStyleSheet(
            f"color:{fg};"
            f"background:{bg};"
            f"border:1px solid {border};"
            "border-radius:3px;"
            "padding:2px 5px;"
            f"font-weight:{weight};"
        )

    def _attach_mode_menu(self) -> None:
        action_cls = getattr(QtGui, "QAction", None) or getattr(QtWidgets, "QAction")
        group_cls = getattr(QtGui, "QActionGroup", None) or getattr(QtWidgets, "QActionGroup", None)
        menu = self.menuBar().addMenu("Mode")
        self._mode_action_group = group_cls(self) if group_cls is not None else None
        if self._mode_action_group is not None:
            self._mode_action_group.setExclusive(True)
        self._mode_actions = {}
        for label in ["Manual", "Teleop", "Replay"]:
            action = action_cls(label, self)
            action.setCheckable(True)
            action.triggered.connect(lambda checked=False, name=label: self._on_mode_action_triggered(name, checked))
            if self._mode_action_group is not None:
                self._mode_action_group.addAction(action)
            menu.addAction(action)
            self._mode_actions[label] = action
        self._sync_mode_actions()

    def _on_mode_action_triggered(self, mode_name: str, checked: bool = True) -> None:
        if checked or self._mode_action_group is None:
            self._switch_mode(mode_name)

    def _sync_mode_actions(self) -> None:
        actions = getattr(self, "_mode_actions", {})
        selected = self._mode_display_name(self.current_mode)
        for label, action in actions.items():
            action.blockSignals(True)
            action.setChecked(label == selected)
            action.blockSignals(False)

    def _attach_device_menu(self) -> None:
        action_cls = getattr(QtGui, "QAction", None) or getattr(QtWidgets, "QAction")
        menu = self.menuBar().addMenu("Device")
        refresh_ports = action_cls("Refresh Ports", self)
        refresh_ports.triggered.connect(self.refresh_ports)
        connect = action_cls("Connect / Disconnect", self)
        connect.triggered.connect(self._toggle_connection)
        refresh_cameras = action_cls("Refresh Cameras", self)
        refresh_cameras.triggered.connect(self.refresh_cameras)
        servo_status = action_cls("Servo Status", self)
        servo_status.triggered.connect(self.show_servo_status_dialog)
        protocol_status = action_cls("Protocol / Comm Status", self)
        protocol_status.triggered.connect(self.show_protocol_status_dialog)
        task_runtime = action_cls("Task Runtime", self)
        task_runtime.triggered.connect(self.show_task_runtime_window)
        auto_calib_log = action_cls("Auto Calib Log", self)
        auto_calib_log.triggered.connect(self.show_auto_calib_log_dialog)
        for action in [refresh_ports, connect, refresh_cameras, servo_status, protocol_status, task_runtime, auto_calib_log]:
            menu.addAction(action)

    def show_task_runtime_window(self) -> None:
        if self._task_runtime_window is None:
            self._task_runtime_window = TaskRuntimeWindow(self)
            self._task_runtime_window.destroyed.connect(lambda _obj=None: setattr(self, "_task_runtime_window", None))
        if self._state is not None:
            self._task_runtime_window.update_state(self._state)
        self._task_runtime_window.show()
        self._task_runtime_window.raise_()
        self._task_runtime_window.activateWindow()

    def show_auto_calib_log_dialog(self) -> None:
        latest = self._latest_auto_calib_log_path()
        if not latest:
            QtWidgets.QMessageBox.information(
                self,
                "Auto Calib Log",
                f"No auto calibration log found.\nFolder: {AUTO_CALIB_LOG_DIR}",
            )
            return
        try:
            with open(latest, "r", encoding="utf-8", errors="replace") as handle:
                lines = handle.readlines()
        except OSError as exc:
            QtWidgets.QMessageBox.warning(self, "Auto Calib Log", f"Failed to read:\n{latest}\n\n{exc}")
            return
        log_text = "".join(lines).strip() or "(empty)"

        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle("Auto Calib Log")
        layout = QtWidgets.QVBoxLayout(dialog)

        path_label = QtWidgets.QLabel(f"Latest log: {latest}")
        path_label.setWordWrap(True)
        path_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(path_label)

        log_view = QtWidgets.QPlainTextEdit()
        log_view.setReadOnly(True)
        log_view.setFont(QtGui.QFont("Consolas", 10))
        try:
            log_view.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        except AttributeError:
            log_view.setLineWrapMode(QtWidgets.QPlainTextEdit.LineWrapMode.NoWrap)
        log_view.setPlainText(log_text)
        layout.addWidget(log_view, 1)

        close_button = QtWidgets.QPushButton("Close")
        close_button.clicked.connect(dialog.accept)
        layout.addWidget(close_button, 0, QtCore.Qt.AlignRight)

        screen = dialog.screen() or QtWidgets.QApplication.primaryScreen()
        if screen is not None:
            area = screen.availableGeometry()
            width = min(max(760, int(area.width() * 0.80)), max(480, area.width() - 80))
            height = min(max(520, int(area.height() * 0.78)), max(360, area.height() - 80))
            dialog.resize(width, height)
            dialog.setMinimumSize(min(760, width), min(460, height))
        else:
            dialog.resize(1100, 700)
            dialog.setMinimumSize(760, 460)

        scroll_bar = log_view.verticalScrollBar()
        scroll_bar.setValue(scroll_bar.maximum())
        dialog.exec()

    @staticmethod
    def _latest_auto_calib_log_path() -> str:
        try:
            paths = [
                os.path.join(AUTO_CALIB_LOG_DIR, name)
                for name in os.listdir(AUTO_CALIB_LOG_DIR)
                if name.startswith("auto_calib_") and name.lower().endswith(".log")
            ]
        except OSError:
            return ""
        if not paths:
            return ""
        paths.sort(key=lambda path: os.path.getmtime(path), reverse=True)
        return paths[0]

    def refresh_ports(self) -> None:
        current = self.current_port
        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        self.port_combo.addItem("No Connection", None)
        for device, desc in list_ports():
            self.port_combo.addItem(f"{device} ({desc})", device)
        if current:
            idx = self.port_combo.findData(current)
            if idx >= 0:
                self.port_combo.setCurrentIndex(idx)
        self.port_combo.blockSignals(False)

    def refresh_cameras(self) -> None:
        comp = self._display_windows.components.get("cameras") if hasattr(self, "_display_windows") else None
        if comp is not None and hasattr(comp, "refresh_devices"):
            comp.refresh_devices()
        self._set_status("Camera devices refreshed")

    def _toggle_connection(self) -> None:
        if self.is_comm_connected():
            self._disconnect_current_port("Disconnected")
            return
        self._connect_selected_port()

    def connect_selected_port(self) -> bool:
        return self._connect_selected_port()

    def _ensure_state_callback_registered(self) -> None:
        self.backend.register_update_callback(self._on_state_update)

    def _on_port_selection_changed(self) -> None:
        port = self.port_combo.currentData()
        if port is None:
            if self.is_comm_connected() or self.current_port:
                self._disconnect_current_port("Disconnected")
            else:
                self.current_port = ""
                self._clear_live_displays()
                self._set_status("No serial port selected")
            return
        port = str(port)
        if self.is_comm_connected() and self.current_port == port:
            return
        if self.is_comm_connected():
            self._disconnect_current_port(f"Switching port: {port}", clear_selection=False)
        self._connect_selected_port()

    def _disconnect_current_port(self, status: str = "Disconnected", clear_selection: bool = True, stop_lower: bool = False) -> None:
        self._stop_data_recording("disconnected")
        self.control_panel.stop_all_tests("disconnected", update_status=False)
        self.backend.shutdown(stop_lower=stop_lower)
        self.current_port = ""
        self._state = None
        self._state_buffer_version = 0
        self._fault_popup_episode_active = False
        self._fault_stop_sent = False
        self._fault_previous_runtime_active = False
        self.control_panel.clear_live_state()
        self._display_windows.update_views(HandModel())
        if clear_selection:
            self.port_combo.blockSignals(True)
            idx = self.port_combo.findData(None)
            if idx >= 0:
                self.port_combo.setCurrentIndex(idx)
            self.port_combo.blockSignals(False)
        self._set_status(status)
        self._refresh_start_indicator()

    def _connect_selected_port(self) -> bool:
        port = self.port_combo.currentData()
        if port is None:
            self.current_port = ""
            self._set_status("No serial port selected")
            return False
        if self.is_comm_connected():
            self._set_status(f"Already connected: {self.current_port}")
            return True
        self._ensure_state_callback_registered()
        self._state = None
        self._state_buffer_version = 0
        self._state_buffer = LatestStateBuffer()
        self._clear_live_displays()
        self._set_status("Connecting: waiting for lower status and servo feedback")
        if self.backend.connect(port):
            self.current_port = str(port)
            sync_status = self.backend.last_sync_status
            self._set_status(f"Connected: {self.current_port}" + (f" | {sync_status}" if sync_status else ""))
            self._on_comm_ready()
            return True
        else:
            QtWidgets.QMessageBox.critical(self, "Connection Failed", f"Cannot open port: {port}")
            return False

    def _switch_mode(self, mode_name: str) -> None:
        previous_mode = self.current_mode
        self.current_mode = self._normalize_mode(mode_name)
        if previous_mode == "Replay" and self.current_mode != "Replay":
            self.cancel_replay("mode changed")
        self.backend.set_pid_control(self.current_mode != "Monitor")
        self.control_panel.sync_app_mode(self.current_mode)
        self._sync_mode_actions()
        self._set_status(f"Mode: {self._mode_display_name(self.current_mode)}")

    def _on_comm_ready(self) -> None:
        self._refresh_start_indicator()
        try:
            self.control_panel._on_active_scope_changed()
        except Exception:
            pass
        self.control_panel.begin_startup_device_check()
        if self.save_data_check.isChecked() and not self._data_logger.is_running():
            self._start_data_recording()

    def _on_state_update(self, state: HandModel) -> None:
        if not self.is_comm_connected():
            return
        self._state_buffer.set(clone_hand_model(state))

    def _display_poll(self) -> None:
        latest, version = self._state_buffer.consume_latest(self._state_buffer_version)
        has_new_state = latest is not None
        if latest is not None:
            self._state_buffer_version = version
            self._state = latest
        has_new_tactile = self._consume_tactile_state()
        state = self._state or HandModel()
        if has_new_state or has_new_tactile or not self.is_comm_connected():
            self._update_display(state)
        elif self.is_comm_connected() and (time.time() - self._last_status_refresh) >= 0.2:
            # A serial port can stay open after lower status heartbeats stop.
            self._last_status_refresh = time.time()
            self._update_status_text(state)
    def _consume_tactile_state(self) -> bool:
        has_new, state = self.backend.consume_tactile_state_into(self._state)
        if has_new:
            self._state = state or HandModel()
            self._tactile_state = self._state
        return bool(has_new)

    def _recording_poll(self) -> None:
        """以独立50 ms时钟保存最新状态，避免UI 33 ms刷新周期把20 Hz降成15 Hz。"""
        if not self._data_logger.is_running():
            return
        if self._state is not None:
            self._maybe_write_run_data_row(self._state)
        self._flush_motion_recording_due()

    def _start_tactile_port(self, port: str) -> bool:
        if not port:
            return False
        try:
            ok = self.backend.connect_tactile_serial(port)
        except Exception as exc:
            self._set_status(f"S3 tactile serial failed: {exc}")
            return False
        if not ok:
            self._set_status(f"S3 tactile serial failed: {port}")
            return False
        self.tactile_port = port
        self._tactile_state = None
        self._set_status(f"S3 tactile serial connected: {port}")
        return True

    def _stop_tactile_port(self) -> None:
        self.backend.disconnect_tactile_serial()
        self._tactile_state = None
        if self._state is not None:
            self._state.tactile_data = []
            self._state.tactile_online = False
            self._state.tactile_module_online = [False] * 15
            self._state.tactile_module_last_rx_ts = [0.0] * 15
            self._update_display(self._state)

    def connect_tactile_serial(self, port: str) -> bool:
        return self._start_tactile_port(port)

    def disconnect_tactile_serial(self) -> None:
        self._stop_tactile_port()
        self._set_status("S3 tactile serial disconnected")

    def is_tactile_serial_connected(self) -> bool:
        return self.backend.is_tactile_serial_connected()

    def get_tactile_serial_port(self) -> str:
        return self.backend.get_tactile_serial_port() or self.tactile_port or ""

    def _update_display(self, state: HandModel) -> None:
        now = time.time()
        if now - self._last_draw < self._draw_interval:
            return
        self._last_draw = now
        self.control_panel.update_state(state)
        self._display_windows.update_views(state)
        if self._task_runtime_window is not None:
            self._task_runtime_window.update_state(state)
        self.joint_plot.append_state(state)
        self.joint_plot.update_plot()
        self._update_status_text(state)

    def _update_status_text(self, state: HandModel) -> None:
        self._refresh_start_indicator()
        now = time.time()
        self._last_status_refresh = now
        connected = self.is_comm_connected()
        delay_ms = 0.0
        ts = float(getattr(state, "host_rx_time", 0.0) or 0.0)
        if ts > 0:
            delay_ms = max(0.0, (now - ts) * 1000.0)
        system_status_fresh = self._packet_status_fresh(
            state,
            "has_system_status_data",
            "system_status_last_rx_ts",
            SYSTEM_STATUS_STALE_AFTER_S,
            now,
        )
        system_uncontrollable = system_status_fresh and int(getattr(state, "system_state", 0) or 0) == 0
        calib = str(getattr(state, "calib_status", "IDLE") or "IDLE")
        if system_status_fresh:
            sys_state = self._system_state_name(getattr(state, "system_state", None))
            sys_faults = self._system_fault_names(getattr(state, "system_fault_bitmap", 0))
            sys_text, sys_tip = self._format_system_fault_summary(state, sys_state, sys_faults)
            calib_text = f"CALIB {calib}"
            calib_tip = self._calib_tooltip(state, calib)
            calib_style = self._calib_status_style(calib)
        else:
            has_status = bool(getattr(state, "has_system_status_data", False))
            sys_faults = []
            if connected:
                sys_text = "SYS STATUS LOST" if has_status else "SYS COMM FAULT"
                sys_tip = (
                    "SYSTEM_STATUS heartbeat is stale; lower protocol is not healthy"
                    if has_status
                    else "No SYSTEM_STATUS received; serial open does not mean lower protocol is healthy"
                )
            else:
                sys_text = "SYS -"
                sys_tip = "Lower serial disconnected"
            calib_text = "CALIB -"
            calib_tip = "Calibration status unavailable"
            calib_style = "warn" if connected else "neutral"
        turn_text, turn_ok, turn_tip = self._turn_known_status(state, system_status_fresh)
        if connected:
            overload_text, overload_tip, overload_active = self._format_overload_summary(state, now)
        else:
            overload_text, overload_tip, overload_active = "OL -", "Lower serial disconnected", False
        hold_text, hold_tip, hold_active, hold_known = self._format_feedback_collision_summary(
            state, connected and system_status_fresh
        )
        delay_text, delay_state = self._format_delay_status(delay_ms)
        self.fault_label.setText(sys_text)
        self.fault_label.setToolTip(sys_tip)
        self.calib_label.setText(calib_text)
        self.calib_label.setToolTip(calib_tip)
        self.overload_label.setText(overload_text)
        self.overload_label.setToolTip(overload_tip)
        self.feedback_hold_label.setText(hold_text)
        self.feedback_hold_label.setToolTip(hold_tip)
        self.turn_label.setText(turn_text)
        self.turn_label.setToolTip(turn_tip)
        self.delay_label.setText(delay_text)
        self.delay_label.setToolTip(f"Delay: {delay_ms:.0f} ms" if delay_ms else "Delay: -")
        system_style = "neutral" if not connected else (
            "warn" if (not system_status_fresh or system_uncontrollable or sys_faults) else "ok"
        )
        self._style_top_status_label(self.fault_label, system_style)
        self._style_top_status_label(self.calib_label, calib_style)
        self._style_top_status_label(self.overload_label, "warn" if overload_active else "ok")
        self._style_top_status_label(
            self.feedback_hold_label,
            "warn" if hold_active else ("ok" if hold_known else "neutral"),
        )
        self._style_top_status_label(self.turn_label, "neutral" if not connected else ("ok" if turn_ok else "warn"))
        self._style_top_status_label(self.delay_label, delay_state)
        self._handle_fault_side_effects(state)

    def _format_feedback_collision_summary(
        self,
        state: HandModel,
        system_status_fresh: bool,
    ) -> tuple[str, str, bool, bool]:
        """只使用 P4 上报状态显示碰撞监测，避免 UI 本地猜测控制状态。"""
        if not system_status_fresh or not bool(
            getattr(state, "has_feedback_collision_status", False)
        ):
            self._last_feedback_collision_armed_mask = None
            self._last_feedback_collision_hold_mask = None
            return "COLLISION ?", "Collision status is unavailable or stale", False, False

        valid_mask = (1 << ENCODER_COUNT) - 1
        armed_mask = int(
            getattr(state, "feedback_collision_armed_bitmap", 0) or 0
        ) & valid_mask
        hold_mask = int(
            getattr(state, "feedback_collision_hold_bitmap", 0) or 0
        ) & valid_mask
        previous_armed = self._last_feedback_collision_armed_mask
        previous_hold = self._last_feedback_collision_hold_mask
        self._last_feedback_collision_armed_mask = armed_mask
        self._last_feedback_collision_hold_mask = hold_mask

        if previous_armed is not None and previous_armed != armed_mask:
            entered = armed_mask & ~previous_armed
            exited = previous_armed & ~armed_mask
            if entered:
                joints = ",".join(
                    f"J{i:02d}" for i in range(ENCODER_COUNT)
                    if entered & (1 << i)
                )
                log_session("COLLISION_ARM", f"entered={joints}")
                self._record_event(
                    "collision_monitor_enter",
                    detail=(
                        f"joints={joints}; armed_mask=0x{armed_mask:08X}; "
                        f"hold_mask=0x{hold_mask:08X}"
                    ),
                    source="p4",
                    p4_time_ms=getattr(state, "p4_time_ms", ""),
                )
            if exited:
                joints = ",".join(
                    f"J{i:02d}" for i in range(ENCODER_COUNT)
                    if exited & (1 << i)
                )
                self._record_event(
                    "collision_monitor_exit",
                    detail=(
                        f"joints={joints}; armed_mask=0x{armed_mask:08X}; "
                        f"hold_mask=0x{hold_mask:08X}"
                    ),
                    source="p4",
                    p4_time_ms=getattr(state, "p4_time_ms", ""),
                )
        if previous_hold is not None and previous_hold != hold_mask:
            entered = hold_mask & ~previous_hold
            released = previous_hold & ~hold_mask
            if entered:
                joints = ",".join(
                    f"J{i:02d}" for i in range(ENCODER_COUNT)
                    if entered & (1 << i)
                )
                log_session("COLLISION_HOLD", f"entered={joints}")
                # 碰撞触发和解除均写入事件总表，避免只能从连续状态位图中人工找边沿。
                self._record_event(
                    "collision_hold_enter",
                    detail=(
                        f"joints={joints}; armed_mask=0x{armed_mask:08X}; "
                        f"hold_mask=0x{hold_mask:08X}"
                    ),
                    source="p4",
                    p4_time_ms=getattr(state, "p4_time_ms", ""),
                )
            if released:
                joints = ",".join(
                    f"J{i:02d}" for i in range(ENCODER_COUNT)
                    if released & (1 << i)
                )
                log_session("COLLISION_RELEASE", f"released={joints}")
                self._record_event(
                    "collision_hold_release",
                    detail=(
                        f"joints={joints}; armed_mask=0x{armed_mask:08X}; "
                        f"hold_mask=0x{hold_mask:08X}"
                    ),
                    source="p4",
                    p4_time_ms=getattr(state, "p4_time_ms", ""),
                )

        def joint_detail(joint_index: int) -> str:
            """将碰撞位图中的关节编号转成可读文本，并明确标记双腱关节。"""
            suffix = " (dual tendon)" if joint_index in DUAL_TENDON_JOINT_INDICES else ""
            return f"J{joint_index:02d}{suffix}"

        mask_detail = (
            f"armed_mask=0x{armed_mask:08X}\n"
            f"hold_mask=0x{hold_mask:08X}"
        )

        if hold_mask:
            joints = [i for i in range(ENCODER_COUNT) if hold_mask & (1 << i)]
            text = (
                "COLLISION HOLD " + ",".join(f"J{i:02d}" for i in joints)
                if len(joints) <= 2
                else f"COLLISION HOLD x{len(joints)}"
            )
            detail = "\n".join(joint_detail(i) for i in joints)
            return (
                text,
                "Collision ABS hold is active:\n"
                f"{detail}\n"
                f"{mask_detail}\n"
                "Send a target more than 1.0 deg in the reverse direction to release.",
                True,
                True,
            )

        if armed_mask:
            joints = ",".join(
                joint_detail(i) for i in range(ENCODER_COUNT)
                if armed_mask & (1 << i)
            )
            return (
                "COLLISION MON",
                "Motion confirmed; collision progress windows are monitoring: "
                f"{joints}\n{mask_detail}",
                False,
                True,
            )
        return (
            "COLLISION IDLE",
            f"No joint has met the motion-arm condition.\n{mask_detail}",
            False,
            True,
        )
    @staticmethod
    def _packet_status_fresh(
        state: HandModel,
        valid_attr: str,
        timestamp_attr: str,
        stale_after_s: float,
        now: Optional[float] = None,
    ) -> bool:
        if not bool(getattr(state, valid_attr, False)):
            return False
        last_rx = float(getattr(state, timestamp_attr, 0.0) or 0.0)
        if last_rx <= 0.0:
            return False
        current = time.time() if now is None else float(now)
        return max(0.0, current - last_rx) <= float(stale_after_s)

    @staticmethod
    def _format_delay_status(delay_ms: float) -> tuple[str, str]:
        if delay_ms <= 0:
            return "--", "neutral"
        text = f"{delay_ms:.0f}ms" if delay_ms < 1000.0 else f"{delay_ms / 1000.0:.1f}s"
        if delay_ms < 200.0:
            return text, "ok"
        if delay_ms < 1000.0:
            return text, "lag"
        return text, "warn"

    @staticmethod
    def _calib_status_style(calib: str) -> str:
        text = str(calib or "").upper()
        if text in {"SUCCESS", "DONE"}:
            return "ok"
        if text == "WARN":
            return "warn"
        if text in {"PENDING", "RUNNING", "SYNCING"}:
            return "lag"
        if text in {"FAILED", "NO_CALIB", "DISCONNECTED"}:
            return "warn"
        return "neutral"

    @staticmethod
    def _overload_detail_items(state: HandModel) -> List[tuple[int, str]]:
        overload_flags = getattr(state, "servo_overload_fault", []) or []
        overload_loads = getattr(state, "servo_overload_trigger_load", []) or []
        items: List[tuple[int, str]] = []
        for i, faulted in enumerate(overload_flags):
            if not faulted:
                continue
            load_text = "?"
            if i < len(overload_loads) and overload_loads[i] is not None:
                try:
                    load_text = str(int(overload_loads[i]))
                except Exception:
                    load_text = str(overload_loads[i])
            items.append((i, load_text))
        return items

    @staticmethod
    def _overload_detail_lines(state: HandModel) -> List[str]:
        return [f"M{i:02d} load={load}" for i, load in HandGUI._overload_detail_items(state)]

    @staticmethod
    def _overload_mask_from_state(state: HandModel) -> int:
        mask = 0
        overload_flags = getattr(state, "servo_overload_fault", []) or []
        for i, faulted in enumerate(overload_flags):
            if faulted:
                mask |= (1 << i)
        return mask

    def _format_overload_summary(self, state: HandModel, now: Optional[float] = None) -> tuple[str, str, bool]:
        fault_status_fresh = self._packet_status_fresh(
            state,
            "has_fault_status_data",
            "fault_status_last_rx_ts",
            FAULT_STATUS_STALE_AFTER_S,
            now,
        )
        if not fault_status_fresh:
            if bool(getattr(state, "has_fault_status_data", False)):
                return "OL STALE", "FAULT_STATUS heartbeat is stale", True
            return "OL NO DATA", "No FAULT_STATUS received; overload state is unknown", True
        overload_items = self._overload_detail_items(state)
        if not overload_items:
            return "OL OK", "Overload motors: none", False
        if len(overload_items) <= 2:
            text = "OL " + " ".join(f"M{i:02d}:{load}" for i, load in overload_items)
        else:
            text = f"OL x{len(overload_items)}"
        lines = [f"M{i:02d} load={load}" for i, load in overload_items]
        return text, "Overload motors:\n" + "\n".join(lines), True

    def _maybe_show_overload_popup(self, state: HandModel) -> None:
        mask = self._overload_mask_from_state(state)
        if mask == 0:
            self._last_overload_popup_mask = 0
            return
        new_mask = mask & ~int(getattr(self, "_last_overload_popup_mask", 0) or 0)
        if new_mask == 0:
            return
        self._last_overload_popup_mask = mask
        items = [(i, load) for i, load in self._overload_detail_items(state) if (new_mask & (1 << i)) != 0]
        if not items:
            return
        detail = "\n".join(f"M{i:02d}: 触发负载 {load}" for i, load in items)
        log_session("OVERLOAD", "; ".join(f"M{i:02d} load={load}" for i, load in items))
        QtWidgets.QMessageBox.warning(
            self,
            "电机过载保护",
            "检测到电机过载，控制已进入保护状态。\n\n"
            f"{detail}\n\n"
            "请检查机构受力和绳索状态，确认安全后点击 Clear Fault 恢复。",
        )

    @staticmethod
    def _fault_bitmap_value(state: HandModel) -> int:
        try:
            return int(getattr(state, "system_fault_bitmap", 0) or 0)
        except Exception:
            return 0

    @staticmethod
    def _calib_diag_summary(state: HandModel) -> dict:
        valid = bool(getattr(state, "calib_diag_valid", False))
        reason = int(getattr(state, "calib_diag_reason", 0) or 0)
        phase = int(getattr(state, "calib_diag_phase", 0) or 0)
        joint = int(getattr(state, "calib_diag_joint", 0xFF) or 0)
        motor = int(getattr(state, "calib_diag_motor", 0xFF) or 0)
        value_a = int(getattr(state, "calib_diag_value_a", 0) or 0)
        value_b = int(getattr(state, "calib_diag_value_b", 0) or 0)
        value_c = int(getattr(state, "calib_diag_value_c", 0) or 0)
        bitmap = int(getattr(state, "calib_diag_bitmap", 0) or 0)
        timestamp_ms = int(getattr(state, "calib_diag_timestamp_ms", 0) or 0)

        reason_names = {
            CALIB_DIAG_REASON_ACTIVE_SCOPE_EMPTY: "没有可标定的 active 关节",
            CALIB_DIAG_REASON_CAN_FEEDBACK_TIMEOUT: "CAN/磁编码器反馈超时",
            CALIB_DIAG_REASON_SERVO_FEEDBACK_TIMEOUT: "舵机反馈超时",
            CALIB_DIAG_REASON_TELEMETRY_TIMEOUT: "舵机遥测反馈超时",
            CALIB_DIAG_REASON_ENCODER_ERROR: "磁编码器反馈无效",
            CALIB_DIAG_REASON_SERVO_OFFLINE: "舵机反馈缺失",
            CALIB_DIAG_REASON_TURN_UNKNOWN: "多圈位置未知",
            CALIB_DIAG_REASON_OVERLOAD: "标定中过载",
            CALIB_DIAG_REASON_LOAD_NOT_REACHED: "未达到预期负载",
            CALIB_DIAG_REASON_ANGLE_NOT_MOVING: "角度没有按预期变化",
            CALIB_DIAG_REASON_WRONG_DIRECTION: "方向错误",
            CALIB_DIAG_REASON_ZERO_RETURN_TIMEOUT: "回零超时",
            CALIB_DIAG_REASON_ZERO_CROSS_INVALID: "零点推算无效",
            CALIB_DIAG_REASON_WRAP_TOO_LARGE: "编码器跨度接近整圈",
            CALIB_DIAG_REASON_ABS_LIMIT_INVALID: "舵机目标超过安全绝对范围",
            CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED: "标定目标发送失败",
            CALIB_DIAG_REASON_FEEDBACK_LOST: "标定过程中反馈丢失",
            CALIB_DIAG_REASON_TASK_START_FAILED: "标定任务启动失败",
            CALIB_DIAG_REASON_INTERNAL_ERROR: "标定内部错误",
            CALIB_DIAG_REASON_CONTROL_LOST: "标定控制权丢失",
            CALIB_DIAG_REASON_HOLD_TARGET_FAILED: "保持其他关节失败",
            CALIB_DIAG_REASON_JOINT_LIST_INVALID: "标定关节列表无效",
            CALIB_DIAG_REASON_DUPLICATE_JOINT: "标定关节重复",
            CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED: "标定工作空间准备失败",
            CALIB_DIAG_REASON_JOINT_STEP_FAILED: "部分关节标定失败",
        }
        phase_names = {
            CALIB_DIAG_PHASE_INIT: "初始化反馈检查",
            CALIB_DIAG_PHASE_FIND_MAX: "寻找最大拉紧位置",
            CALIB_DIAG_PHASE_RELEASE: "松绳/方向判断",
            CALIB_DIAG_PHASE_RETURN_ZERO: "回到零位",
            CALIB_DIAG_PHASE_DUAL_PRELOAD: "双电机预紧",
            CALIB_DIAG_PHASE_DUAL_EXCHANGE: "双电机交替测试",
            CALIB_DIAG_PHASE_SEQUENCE: "标定任务调度",
        }

        active = valid and reason != 0
        title = reason_names.get(reason, f"标定诊断 reason={reason}") if active else ""
        phase_text = phase_names.get(phase, f"phase={phase}" if phase else "-")
        joint_text = f"J{joint:02d}" if 0 <= joint < ENCODER_COUNT else "J?"
        motor_text = f"M{motor:02d}" if 0 <= motor < MOTOR_COUNT else "M?"

        lines: List[str] = []
        suggestions: List[str] = []
        if active:
            lines.append(f"标定失败原因：{title}")
            lines.append(f"阶段：{phase_text}")
            lines.append(f"对象：{joint_text}, {motor_text}")

            if reason == CALIB_DIAG_REASON_OVERLOAD:
                lines.append(f"数据：load={value_a}, limit={value_b}, raw_load={value_c}, bitmap=0x{bitmap:08X}")
                suggestions.append("先停止控制并卸载机械外力；检查绳索是否过紧、机构是否卡住，再 Clear Fault 后重新标定。")
            elif reason in {
                CALIB_DIAG_REASON_ENCODER_ERROR,
                CALIB_DIAG_REASON_CAN_FEEDBACK_TIMEOUT,
            }:
                lines.append(f"数据：encoder_ok={value_a}, abs_ok={value_b}, raw_ok={value_c}, bitmap=0x{bitmap:08X}")
                suggestions.append("检查对应磁编码器供电、SPI/CAN 接线、S3/P4 通信和传感器板状态；恢复有效反馈后再标定。")
            elif reason in {
                CALIB_DIAG_REASON_SERVO_OFFLINE,
                CALIB_DIAG_REASON_SERVO_FEEDBACK_TIMEOUT,
                CALIB_DIAG_REASON_TELEMETRY_TIMEOUT,
                CALIB_DIAG_REASON_FEEDBACK_LOST,
            }:
                lines.append(f"数据：abs_ok={value_a}, raw_ok={value_b}, telem_ok={value_c}, bitmap=0x{bitmap:08X}")
                suggestions.append("检查对应舵机电源、串口总线、ID 映射和接线；反馈稳定后 Clear Fault 并重新标定。")
            elif reason == CALIB_DIAG_REASON_TURN_UNKNOWN:
                lines.append(f"数据：unknown_mask=0x{bitmap:08X}")
                suggestions.append("不要继续发送运动命令。若换过机械/舵机/映射或舵机单独断电，先 New Device Reset，再 Clear Fault 和重新标定。")
            elif reason == CALIB_DIAG_REASON_WRONG_DIRECTION:
                if phase == CALIB_DIAG_PHASE_DUAL_EXCHANGE and motor == 0xFF:
                    lines.append(
                        f"数据：signed_span={value_a}, default_dir={value_b}, runtime_dir={value_c}, joint_mask=0x{bitmap:08X}"
                    )
                    suggestions.append("双电机方向一致性检查失败。检查该关节磁编码器方向表、主/副舵机 M/J 映射、tendon 收放方向和绳路是否反绕；方向修正后重新标定。")
                else:
                    lines.append(f"数据：actual_delta={value_a}, expected_delta={value_b}, threshold={value_c}")
                    suggestions.append("检查磁编码器方向、M/J 映射、tendon 收绳方向和绳路是否反绕；方向修正后重新 Rope Init/标定。")
            elif reason == CALIB_DIAG_REASON_ZERO_RETURN_TIMEOUT:
                lines.append(f"数据：abs_error={value_a}, required_travel={value_b}, zero_deg_x100={value_c}")
                suggestions.append("检查回零目标是否被机械限位阻挡、绳索是否太紧、zero pull 参数是否过大；释放机构后重试。")
            elif reason == CALIB_DIAG_REASON_ZERO_CROSS_INVALID:
                lines.append(f"数据：taut={value_a}, zero={value_b}, tight={value_c}")
                suggestions.append("零点不在有效区间内。检查编码器方向、标定起始姿态和绳索预紧；必要时 New Device Reset 后重新标定。")
            elif reason == CALIB_DIAG_REASON_WRAP_TOO_LARGE:
                lines.append(f"数据：encoder_span={value_a}, tight={value_b}, max={value_c}")
                suggestions.append("编码器跨度异常接近整圈，通常是方向/映射错误或多圈状态错误；先 New Device Reset，再检查方向和映射。")
            elif reason == CALIB_DIAG_REASON_ABS_LIMIT_INVALID:
                lines.append(f"数据：command_abs={value_a}, min={value_b}, max={value_c}")
                suggestions.append("舵机绝对目标超出安全范围。先确认多圈读数、舵机 ID 和绳路，再重新 Rope Init/标定。")
            elif reason == CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED:
                lines.append(f"数据：target_abs={value_a}, owner/state={value_b}, bitmap=0x{bitmap:08X}")
                suggestions.append("下位机未接受标定目标。确认没有其他控制流程占用舵机目标，Clear Fault 后重新进入标定。")
            elif reason == CALIB_DIAG_REASON_ACTIVE_SCOPE_EMPTY:
                lines.append(f"数据：active_joint_mask=0x{value_a:08X}, active_motor_mask=0x{value_b:08X}")
                suggestions.append("在 Active Scope 中勾选需要标定的关节/电机后再开始标定。")
            elif reason == CALIB_DIAG_REASON_TASK_START_FAILED:
                suggestions.append("标定任务没有启动。确认上一次标定已经退出；如果仍失败，重启 P4 后再试。")
            elif reason == CALIB_DIAG_REASON_CONTROL_LOST:
                lines.append(f"数据：state={value_a}, owner={value_b}, enabled={value_c}, active_joint_mask=0x{bitmap:08X}")
                suggestions.append("标定过程中控制权被打断。检查是否按过 STOP/E-STOP/Pause/Clear Fault/Enable，或上位机是否发送了其他控制命令；Clear Fault 后重新 ENABLE，再重新标定。")
            elif reason == CALIB_DIAG_REASON_HOLD_TARGET_FAILED:
                lines.append(f"数据：current_index={value_a}, joint_count={value_b}, batch_count={value_c}, selected_joint_mask=0x{bitmap:08X}")
                suggestions.append("标定当前关节时，其他已选关节的保持目标生成失败。检查所选关节对应舵机是否在线、Turn 是否 known、ABS limit 是否有效；必要时减少勾选范围逐个标定。")
            elif reason == CALIB_DIAG_REASON_JOINT_LIST_INVALID:
                lines.append(f"数据：joint_or_count={value_a}, max_joint_count={value_b}, index_or_has_list={value_c}, seen_mask=0x{bitmap:08X}")
                suggestions.append("下位机收到的 active scope/关节列表不合法。取消勾选后重新选择要标定的关节；如果反复出现，检查上位机 active scope 下发和固件关节数量是否一致。")
            elif reason == CALIB_DIAG_REASON_DUPLICATE_JOINT:
                lines.append(f"数据：duplicate_index={value_a}, joint_count={value_b}, seen_mask=0x{bitmap:08X}")
                suggestions.append("同一个关节被重复加入标定序列。重新勾选关节后再标定；如果还出现，检查 active scope 到标定列表的构造逻辑。")
            elif reason == CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED:
                workspace_stage_names = {
                    101: "动作前反馈预检查",
                    102: "辅助关节/目标关节姿态执行",
                    103: "目标标定完成后的让位姿态",
                }
                if value_c in workspace_stage_names:
                    helper_text = f"J{value_a:02d}" if 0 <= value_a < ENCODER_COUNT else "J?"
                    pose_text = {1: "POSE_A（主松、副紧）", 2: "POSE_B（副松、主紧）"}.get(value_b, f"pose={value_b}")
                    lines.append(
                        f"工作空间：标定目标={joint_text}，失败关节={helper_text}，"
                        f"姿态={pose_text}，阶段={workspace_stage_names[value_c]}，"
                        f"相关电机=0x{bitmap:08X}"
                    )
                    suggestions.append("工作空间准备会先处理辅助关节、最后处理标定目标。检查失败关节的舵机在线状态、ABS/load 反馈和 Turn 是否 known；多圈未知时先执行 New Device Reset，再 Rope Init/标定。")
                else:
                    lines.append(f"数据：value_a={value_a}, value_b={value_b}, value_c={value_c}, motor_mask=0x{bitmap:08X}")
                    suggestions.append("自动标定无法安全安排四指侧摆关节的工作空间。检查相关电机是否在线、Turn 是否 known；若出现多圈未知，先执行 New Device Reset 后重新 Rope Init/标定。")
            elif reason == CALIB_DIAG_REASON_JOINT_STEP_FAILED:
                lines.append(
                    f"数据：completed_mask=0x{value_a:08X}, failed_mask=0x{value_b:08X}, "
                    f"selected_count={value_c}, failed_mask_dup=0x{bitmap:08X}"
                )
                suggestions.append("本次已成功的关节结果会保留；下次只勾选 failed_mask 对应的失败关节重新标定。若需要细节，请查看前后几行 [AUTO CALIB] 日志。")
            elif reason in {
                CALIB_DIAG_REASON_LOAD_NOT_REACHED,
                CALIB_DIAG_REASON_ANGLE_NOT_MOVING,
            }:
                lines.append(f"数据：valueA={value_a}, valueB={value_b}, valueC={value_c}")
                suggestions.append("检查绳索是否打滑、过松、未连接或关节被卡住；确认负载和角度都有响应后重新标定。")
            else:
                lines.append(f"数据：valueA={value_a}, valueB={value_b}, valueC={value_c}, bitmap=0x{bitmap:08X}")
                suggestions.append("记录当前日志，检查对应阶段的反馈、映射、方向和机械状态后重新标定。")

        return {
            "active": active,
            "title": title,
            "lines": lines,
            "suggestions": suggestions,
            "key": f"{reason}:{phase}:{joint}:{motor}:{value_a}:{value_b}:{value_c}:{bitmap:08X}:{timestamp_ms}",
        }

    def _calib_tooltip(self, state: HandModel, calib: str) -> str:
        if str(calib or "").upper() == "WARN":
            mask = int(getattr(state, "calibration_warning_bitmap", 0) or 0)
            joints = [f"J{i:02d}" for i in range(21) if mask & (1 << i)]
            suffix = ", ".join(joints) if joints else "unknown joint"
            return (
                "Calibration: WARN\n"
                f"Assembly endpoint offset detected: {suffix}. "
                "Calibration remains usable."
            )
        diag = self._calib_diag_summary(state)
        if diag.get("active"):
            return "Calibration: " + str(calib) + "\n\n" + self._fault_body_text(diag)
        return "Calibration: " + str(calib)

    def _build_fault_summary(self, state: HandModel) -> dict:
        bitmap = self._fault_bitmap_value(state)
        overload_items = self._overload_detail_items(state)
        overload_mask = self._overload_mask_from_state(state)
        try:
            unknown_mask = int(getattr(state, "multi_turn_unknown_bitmap", 0) or 0) & VALID_MOTOR_MASK
        except Exception:
            unknown_mask = 0

        lines: List[str] = []
        suggestions: List[str] = []
        titles: List[str] = []
        calib_diag = self._calib_diag_summary(state)

        if (bitmap & SYSTEM_FAULT_OVERLOAD) or overload_items:
            titles.append("舵机过载")
            if overload_items:
                lines.append("舵机过载：")
                lines.extend(f"  M{i:02d} load={load}" for i, load in overload_items)
            else:
                lines.append("舵机过载：触发电机未知")
            suggestions.append("检查机构是否卡住、绳索是否过紧、目标角度是否过大，以及是否误加载了旧标定。")

        if bitmap & SYSTEM_FAULT_SERVO_OFFLINE:
            titles.append("舵机离线")
            lines.append("舵机离线：下位机未获得完整舵机反馈。")
            suggestions.append("检查舵机电源、总线、ID 映射和接线。")

        if bitmap & SYSTEM_FAULT_CAN_OFFLINE:
            titles.append("CAN/磁编码器离线")
            lines.append("CAN/磁编码器离线：下位机未获得有效磁编码器/CAN 数据。")
            suggestions.append("检查 S3/P4 CAN 通信、磁编码器供电、SPI 接线和传感器板状态。")

        if bitmap & SYSTEM_FAULT_CALIBRATION:
            if calib_diag.get("active"):
                titles.append(str(calib_diag.get("title") or "标定故障"))
                lines.extend(list(calib_diag.get("lines") or []))
                suggestions.extend(list(calib_diag.get("suggestions") or []))
            else:
                titles.append("标定故障")
                lines.append("标定故障：下位机标定流程失败或被保护中断，但没有收到细分诊断包。")
                suggestions.append("先检查下位机固件是否为最新版本；再检查负载阈值、编码器方向、机械限位和绳索预紧后重新标定。")

        if bitmap & SYSTEM_FAULT_JOINT16_DUAL:
            titles.append("J16 双电机约束异常")
            lines.append("J16 双电机约束异常：主/副电机反馈差异超出限制。")
            suggestions.append("检查 J16 双电机映射、副电机反馈、对抗绳路和预紧。")

        if bitmap & SYSTEM_FAULT_MULTITURN_UNKNOWN:
            titles.append("多圈位置未知")
            if unknown_mask:
                lines.append(f"多圈位置未知：Turn UNKNOWN mask=0x{unknown_mask:08X}")
            else:
                lines.append("多圈位置未知：故障已锁存，当前 unknown mask 已清除。")
            suggestions.append(
                "不要继续发运动命令。若刚换机械/舵机/映射，或舵机单独断电、手动转动过机构，"
                "先点击 New Device Reset，把当前 raw 作为 turn0。确认 Turn 恢复正常后，再点击 Clear Fault，"
                "然后重新 ENABLE / Rope Init / 标定。"
            )

        if not lines and bitmap != 0:
            titles.append("未知系统故障")
            lines.append(f"未知系统故障 bitmap=0x{bitmap:08X}")
            suggestions.append("记录日志后检查下位机状态机和通信包解析。")

        active = bool(lines or bitmap != 0)
        offline_bits = SYSTEM_FAULT_CAN_OFFLINE | SYSTEM_FAULT_SERVO_OFFLINE
        offline_only = active and (bitmap & ~offline_bits) == 0 and not overload_items
        only_turn_unknown = bool(
            active and
            (bitmap & SYSTEM_FAULT_MULTITURN_UNKNOWN) and
            (bitmap & ~SYSTEM_FAULT_MULTITURN_UNKNOWN) == 0 and
            not overload_items and
            not calib_diag.get("active")
        )
        title = " / ".join(titles[:2]) if titles else "系统状态正常"
        if len(titles) > 2:
            title += f" 等 {len(titles)} 项"
        key = f"{bitmap:08X}:{overload_mask:08X}:{unknown_mask:08X}:{calib_diag.get('key', '')}"
        return {
            "active": active,
            "title": title,
            "lines": lines,
            "suggestions": suggestions,
            "key": key,
            "offline_only": offline_only,
            "bitmap": bitmap,
            "unknown_mask": unknown_mask,
            "only_turn_unknown": only_turn_unknown,
        }

    @staticmethod
    def _fault_body_text(summary: dict, include_title: bool = False) -> str:
        parts: List[str] = []
        if include_title:
            parts.append(str(summary.get("title") or "系统故障"))
        parts.extend(list(summary.get("lines") or []))
        suggestions = list(summary.get("suggestions") or [])
        if suggestions:
            parts.append("")
            parts.append("建议：")
            for item in suggestions:
                parts.append(f"- {item}")
        return "\n".join(parts).strip()

    def _fault_popup_allowed(self, summary: dict, state: HandModel) -> bool:
        if not summary.get("active"):
            return False
        if not summary.get("offline_only"):
            return True
        system_state = getattr(state, "system_state", None)
        runtime_state = system_state in (SYSTEM_STATE_RUN_DIRECT, SYSTEM_STATE_RUN_FEEDBACK, SYSTEM_STATE_PAUSED)
        return bool(runtime_state or self._fault_previous_runtime_active)

    def _fault_stop_allowed(self, summary: dict, state: HandModel) -> bool:
        if not summary.get("active"):
            return False
        if not summary.get("offline_only"):
            return True
        return self._fault_popup_allowed(summary, state)

    def _handle_fault_side_effects(self, state: HandModel) -> None:
        system_state = getattr(state, "system_state", None)
        runtime_state = system_state in (SYSTEM_STATE_RUN_DIRECT, SYSTEM_STATE_RUN_FEEDBACK, SYSTEM_STATE_PAUSED)
        if runtime_state:
            self._fault_previous_runtime_active = True

        summary = self._build_fault_summary(state)
        if not summary.get("active"):
            self._last_overload_popup_mask = 0
            self._fault_popup_episode_active = False
            self._fault_stop_sent = False
            self._fault_previous_runtime_active = bool(runtime_state)
            return

        if self._fault_stop_allowed(summary, state) and not self._fault_stop_sent:
            self._fault_stop_sent = True
            try:
                self.control_panel.stop_all_tests("fault detected", update_status=False)
            except Exception:
                pass
            try:
                self.backend.stop()
            except Exception:
                pass
            log_session("FAULT", f"stop requested: {summary.get('key')} {summary.get('title')}")
            self._set_status("检测到故障，已发送 STOP")

        if not self._fault_popup_allowed(summary, state):
            return
        if self._fault_popup_episode_active:
            return
        if self._fault_dialog_active:
            return

        unknown_mask = int(summary.get("unknown_mask", 0) or 0)
        turn_popup_seen = bool(
            unknown_mask and self.control_panel.turn_unknown_popup_seen()
        )
        self._fault_popup_episode_active = True
        if summary.get("only_turn_unknown") and turn_popup_seen:
            return
        if unknown_mask:
            self.control_panel.mark_turn_unknown_popup_seen(unknown_mask)
        self._fault_dialog_active = True
        try:
            log_session("FAULT", self._fault_body_text(summary).replace("\n", " | "))
            QtWidgets.QMessageBox.warning(
                self,
                "系统故障保护",
                "检测到系统故障，控制已停止。\n\n"
                f"{self._fault_body_text(summary)}\n\n"
                "确认机械和接线安全后，点击 Clear Fault 清除故障。",
            )
        finally:
            self._fault_dialog_active = False

    def _on_clear_fault_confirmed(self) -> None:
        self._last_overload_popup_mask = 0
        self._fault_stop_sent = False
        self._fault_previous_runtime_active = False

    def _format_system_fault_summary(self, state: HandModel, sys_state: str, sys_faults: List[str]) -> tuple[str, str]:
        if not sys_faults:
            text = f"SYS {sys_state}" if sys_state != "-" else "SYS -"
        elif len(sys_faults) == 1:
            text = f"SYS {sys_faults[0]}"
        else:
            text = f"SYS FAULT x{len(sys_faults)}"

        lines = [f"System state: {sys_state}"]
        if sys_faults:
            lines.append("System faults: " + ", ".join(sys_faults))
        else:
            lines.append("System faults: none")

        overload_lines = self._overload_detail_lines(state)
        if overload_lines:
            lines.append("Overload motors:")
            lines.extend(overload_lines)
        elif "OVERLOAD" in sys_faults:
            lines.append("Overload motors: unknown")

        feedback_reject = self._feedback_admission_reject_text(state)
        if feedback_reject:
            text = f"{text} | FF REJECTED"
            lines.append(f"Feedback admission: rejected ({feedback_reject})")

        summary = self._build_fault_summary(state)
        if summary.get("active"):
            body = self._fault_body_text(summary)
            if body:
                lines.append("")
                lines.append(body)

        return text, "\n".join(lines)

    @staticmethod
    def _feedback_admission_reject_text(state: HandModel) -> str:
        if not bool(getattr(state, "has_feedback_admission_status", False)):
            return ""
        if int(getattr(state, "feedback_admission_status", 0) or 0) != FEEDBACK_ADMISSION_REJECTED:
            return ""
        reason = int(getattr(state, "feedback_reject_reason", 0) or 0)
        names = {
            FEEDBACK_REJECT_MANUAL_REFERENCE: "Manual reference missing",
            FEEDBACK_REJECT_STATE: "state does not allow feedback",
            FEEDBACK_REJECT_WORKSPACE: "active joint is outside READY workspace",
            FEEDBACK_REJECT_MULTITURN: "multi-turn position unknown",
            FEEDBACK_REJECT_ABS_LIMIT: "active joint calibration is invalid",
            FEEDBACK_REJECT_MODE_SWITCH: "command mode switch is busy",
            FEEDBACK_REJECT_EVENT_QUEUE: "state event queue is full",
        }
        return names.get(reason, f"unknown reason {reason}")

    def _turn_known_status(self, state: HandModel, system_status_fresh: Optional[bool] = None) -> tuple[str, bool, str]:
        if system_status_fresh is None:
            system_status_fresh = self._packet_status_fresh(
                state,
                "has_system_status_data",
                "system_status_last_rx_ts",
                SYSTEM_STATUS_STALE_AFTER_S,
            )
        if not system_status_fresh:
            return "TURN -", False, "Turn: unknown"
        try:
            unknown = int(getattr(state, "multi_turn_unknown_bitmap", 0))
        except Exception:
            unknown = 0
        active_mask = 0
        try:
            active_mask = int(self.control_panel._active_motor_mask_from_ui())
        except Exception:
            active_mask = 0
        if active_mask == 0:
            active_mask = VALID_MOTOR_MASK
        active_unknown = unknown & active_mask
        if active_unknown:
            return "TURN UNKNOWN", False, f"Unknown turn mask: 0x{active_unknown:08X}"
        return "TURN KNOWN", True, "Turn: known"

    @staticmethod
    def _system_state_name(value) -> str:
        names = {
            0: "UNCTRL",
            1: "IDLE",
            2: "RUN_FEEDBACK",
            4: "CAL_RUN",
            5: "READY",
            6: "RUN_DIRECT",
            7: "PAUSED",
            SYSTEM_STATE_ROPE_INIT_RUNNING: "ROPE_INIT",
            SYSTEM_STATE_RECOVERY_REQUIRED: "RECOVERY",
        }
        try:
            return names.get(int(value), str(value))
        except Exception:
            return "-"

    @staticmethod
    def _system_fault_names(bitmap) -> List[str]:
        try:
            value = int(bitmap)
        except Exception:
            value = 0
        names = [
            (SYSTEM_FAULT_CAN_OFFLINE, "CAN"),
            (SYSTEM_FAULT_SERVO_OFFLINE, "SERVO"),
            (SYSTEM_FAULT_CALIBRATION, "CALIB"),
            (SYSTEM_FAULT_MULTITURN_UNKNOWN, "MULTITURN"),
            (SYSTEM_FAULT_OVERLOAD, "OVERLOAD"),
            (SYSTEM_FAULT_JOINT16_DUAL, "J16"),
        ]
        return [name for bit, name in names if (value & bit) != 0]

    def _clear_live_displays(self) -> None:
        self.control_panel.clear_live_state()
        self._display_windows.update_views(HandModel())

    def _open_joint_rt_window(self) -> None:
        if not self.joint_plot.isVisible():
            self.joint_plot.show()
        self.joint_plot.raise_()
        self.joint_plot.activateWindow()

    def send_joint_targets(
        self,
        angles: List[float],
        source: str = "",
        live: bool = False,
        input_space: str = "urdf_deg",
        raw_targets=None,
        command_joint_mask=None,
    ):
        guard = getattr(self.control_panel, "feedback_command_allowed", None)
        if callable(guard) and not guard(source=source, show_status=True):
            return False
        previous_state = self.backend.get_current_state()
        previous_targets = list(getattr(previous_state, "target_angles", []) or [])
        result = self.backend.send_joint_targets(
            angles,
            source=source,
            live=live,
            input_space=input_space,
            raw_targets=raw_targets,
            command_joint_mask=command_joint_mask,
        )
        if result.message:
            self._set_status(result.message)
        if source and result.sent:
            self.emit_action(source)
            if not live:
                # 记录整包中真正变化的关节，实机日志可以直接识别单行 Send
                # 是否意外夹带了其他输入框的历史目标。
                changed = []
                for idx, value in enumerate(list(result.targets or [])):
                    if idx >= len(previous_targets) or abs(float(value) - float(previous_targets[idx])) >= 0.001:
                        old = previous_targets[idx] if idx < len(previous_targets) else float("nan")
                        changed.append(f"J{idx:02d}:{old:.3f}->{float(value):.3f}")
                preview = ", ".join(changed[:8]) if changed else "none"
                if len(changed) > 8:
                    preview += f", +{len(changed) - 8} more"
                mask_text = (
                    "active"
                    if command_joint_mask is None
                    else f"0x{int(command_joint_mask):08X}"
                )
                self.emit_action(
                    f"JOINT_CMD source={source} mask={mask_text} changed={preview}"
                )
        return result

    def _on_joint_target_command(self, payload: Dict[str, object]) -> None:
        """记录所有关节动作；连续流20 Hz降采样，离散UI命令逐条保留。"""
        if not self._data_logger.is_running():
            return
        host_time = time.time()
        source = str(payload.get("source", ""))
        p4_time_ms = payload.get("p4_time_ms", "")
        input_targets = payload.get("input_targets", [])
        raw_input_targets = payload.get("raw_input_targets", input_targets)
        output_targets = payload.get("output_targets", [])
        blocked = payload.get("blocked", [])
        blocked_list = list(blocked) if isinstance(blocked, (list, tuple)) else []
        clamped = payload.get("clamped", [])
        clamped_list = list(clamped) if isinstance(clamped, (list, tuple)) else []
        state = self._state
        active_joint_mask = int(
            getattr(state, "active_joint_mask", 0) or 0
        ) if state is not None else 0
        command_joint_mask = payload.get("command_joint_mask")
        if command_joint_mask is None:
            command_joint_mask = active_joint_mask
        row = {
            "host_time": host_time,
            "p4_time_ms": p4_time_ms,
            "source": source,
            "frame_id": "",
            "valid": 1,
            "latency_ms": "",
            "active_joint_mask": f"0x{active_joint_mask:08X}",
            "command_joint_mask": f"0x{int(command_joint_mask):08X}",
            "live_enabled": 1 if payload.get("live", False) else 0,
            "blocked": 1 if blocked_list else 0,
            "block_reason": payload.get("message", ""),
            "sent_to_backend": 1 if payload.get("sent", False) else 0,
            "input_space": payload.get("input_space", "urdf_deg"),
            "output_space": payload.get("output_space", "urdf_deg"),
            "mapping_id": payload.get("mapping_id", ""),
            "mapping_version": payload.get("mapping_version", ""),
            "urdf_config_crc": payload.get("urdf_config_crc", ""),
            "clamped_joints_json": json.dumps(
                clamped_list, ensure_ascii=False, default=str
            ),
            "raw_joint_targets_json": json.dumps(
                raw_input_targets, ensure_ascii=False, default=str
            ),
            "input_joint_targets_json": json.dumps(
                input_targets, ensure_ascii=False, default=str
            ),
            "output_joint_targets_json": json.dumps(
                output_targets, ensure_ascii=False, default=str
            ),
        }
        for sampled_row in self._motion_record_sampler.submit(row):
            self._data_logger.write("teleop_input", sampled_row)

    def _flush_motion_recording_due(self, *, force: bool = False) -> None:
        """补写连续动作末帧；关闭记录时必须强制执行一次。"""
        for row in self._motion_record_sampler.flush_due(force=force):
            self._data_logger.write("teleop_input", row)

    def draw_pose_angles(self, angles: List[float]) -> None:
        self._display_windows.open_pose()
        comp = self._display_windows.components.get("pose")
        if comp is not None:
            comp.draw_angles(angles, None, generate=False)

    def start_calibration(self) -> None:
        if not self.is_comm_connected():
            if not self._connect_selected_port():
                return
        if self.backend.request_firmware_calibration():
            self._set_status("Calibration command sent")
            self.emit_action("Calibrate")
        else:
            self._set_status("Calibration blocked: serial port is not connected")

    def _start_data_recording(self, options: Optional[Dict[str, bool]] = None) -> None:
        if self._test_recording_stop_active():
            raise RuntimeError("previous test recording is still finalizing")
        options = dict(options or self._recording_format_options)
        self._record_tactile_full = bool(options.get("tactile_full", False))
        meta = {
            "format_version": 3,
            "app_mode": self.current_mode,
            "port": self.current_port,
            "ros2_enabled": self.ros2_enabled,
            "ros_topic": self.ros_topic,
            "recording_formats": options,
            "csv_layout_version": 5,
            "state_recording_hz": 20,
            "continuous_motion_recording_hz": 20,
            "replay_interpolation_hz": 100,
            "encoder_count": ENCODER_COUNT,
            "motor_count": MOTOR_COUNT,
            "finger_order": FINGER_NAMES,
            "human_readable_csv": {
                "time_columns": [
                    "sample_index",
                    "host_time",
                    "p4_time_ms",
                ],
                "csv_tables": [
                    "01_基础状态.csv",
                    "02_关节总表.csv",
                    "03_舵机总表.csv",
                    "04_触觉总表.csv",
                    "05_触觉全量.csv（仅勾选时）",
                    "06_操作事件总表.csv",
                    "07_遥操作总表.csv",
                ],
                "time_semantics": {
                    "host_time": "上位机收到并记录该状态的本地时间，精确到毫秒",
                    "p4_time_ms": "下位机自本次上电启动后的毫秒计数；无对应P4包时留空",
                },
            },
            "calibration_snapshot": self._read_json_config(CALIB_LOAD_SNAPSHOT_PATH),
            "config_files": self._recording_config_snapshot(),
        }
        recording_topics = list(DEFAULT_TOPICS)
        csv_topics = list(DEFAULT_CSV_TOPICS)
        if self._record_tactile_full:
            recording_topics.extend(TACTILE_FULL_TOPICS)
            csv_topics.append("tactile_full")
        self._data_logger.start(
            self._run_data_root_dir(),
            meta=meta,
            topics=recording_topics,
            csv_topics=csv_topics,
            enable_csv=bool(options.get("csv", True)),
            enable_hdf5=bool(options.get("hdf5", True)),
            enable_rosbag=bool(options.get("rosbag", False)),
        )
        self._motion_record_sampler.reset()
        self._last_state_record_signature = None
        path = self._data_logger.path
        self._last_recorded_proto_ack_sequence = int(
            getattr(self._state, "proto_ack_sequence", 0) or 0
        )
        self._recording_status_note = path
        self._recording_csv_path = path
        self._start_joint_debug_packet_log()
        notes = []
        if bool(options.get("csv", True)):
            notes.append("CSV: on")
        else:
            notes.append("CSV: off")
        h5_note = f"HDF5: {self._data_logger.hdf5_path}" if self._data_logger.hdf5_enabled else ""
        if self._data_logger.hdf5_error:
            h5_note = f"HDF5 disabled: {self._data_logger.hdf5_error}"
        if h5_note:
            notes.append(h5_note)
        bag_note = f"ROS2 bag: {self._data_logger.rosbag_path}" if self._data_logger.rosbag_enabled else ""
        if self._data_logger.rosbag_error and bool(options.get("rosbag", False)):
            bag_note = f"ROS2 bag disabled: {self._data_logger.rosbag_error}"
        if bag_note:
            notes.append(bag_note)
        self._set_status(f"Recording: {path}" + (f" | {' | '.join(notes)}" if notes else ""))

    def _stop_data_recording(self, reason: str = "stopped") -> None:
        if self._test_recording_stop_active():
            self._set_status("Recording files are still finalizing in the background")
            return
        if self._data_logger.is_running():
            path = self._data_logger.path
            self._flush_motion_recording_due(force=True)
            if self._state is not None:
                self._maybe_write_run_data_row(self._state, force=True)
            self._record_event("save_data_stop", detail=reason, source="recorder")
            self._data_logger.stop()
            self._stop_joint_debug_packet_log()
            self._set_status(f"Recording {reason}: {path}")
        else:
            self._stop_joint_debug_packet_log()

    def start_recording_for_test(self, reason: str = "test") -> None:
        if self._test_recording_stop_active():
            raise RuntimeError("previous test recording is still finalizing; retry shortly")
        if self._data_logger.is_running():
            self._stop_data_recording(f"restart by {reason}")
        if not self.save_data_check.isChecked():
            self.save_data_check.blockSignals(True)
            self.save_data_check.setChecked(True)
            self.save_data_check.blockSignals(False)
        self._start_data_recording(self._recording_format_options)

    def stop_recording_for_test(self, reason: str = "test stopped") -> None:
        self.save_data_check.blockSignals(True)
        self.save_data_check.setChecked(False)
        self.save_data_check.blockSignals(False)
        with self._test_recording_stop_lock:
            thread = self._test_recording_stop_thread
            if thread is not None and thread.is_alive():
                return
            if not self._data_logger.is_running():
                self._stop_joint_debug_packet_log()
                return
            path = self._data_logger.path
            flush_motion = getattr(self, "_flush_motion_recording_due", None)
            if callable(flush_motion):
                flush_motion(force=True)
            state = getattr(self, "_state", None)
            write_state = getattr(self, "_maybe_write_run_data_row", None)
            if state is not None and callable(write_state):
                write_state(state, force=True)
            self._record_event("save_data_stop", detail=reason, source="recorder")
            thread = threading.Thread(
                target=self._finish_test_recording_stop,
                args=(path, reason),
                name="mmhand-test-recording-stop",
                daemon=True,
            )
            self._test_recording_stop_thread = thread
        self._recording_status_note = ""
        self._recording_csv_path = ""
        self._set_status(f"Test stopped; finalizing recording in background: {path}")
        thread.start()

    def _test_recording_stop_active(self) -> bool:
        with self._test_recording_stop_lock:
            thread = self._test_recording_stop_thread
            return bool(thread is not None and thread.is_alive())

    def _finish_test_recording_stop(self, path: str, reason: str) -> None:
        error = ""
        try:
            self._data_logger.stop()
        except Exception as exc:
            error = f"{type(exc).__name__}: {exc}"
        finally:
            try:
                self._stop_joint_debug_packet_log()
            except Exception as exc:
                if not error:
                    error = f"{type(exc).__name__}: {exc}"
        self.test_recording_stop_finished.emit(path, reason, error)

    def _on_test_recording_stop_finished(self, path: str, reason: str, error: str) -> None:
        with self._test_recording_stop_lock:
            self._test_recording_stop_thread = None
        if error:
            log_session("RECORDER", f"test recording finalize failed path={path} error={error}")
            self._set_status(f"Test stopped; recording finalize failed: {error}")
            return
        log_session("RECORDER", f"test recording finalized path={path} reason={reason}")
        self._set_status(f"Recording {reason}: {path}")

    def _wait_for_test_recording_stop(self, timeout_s: float = 4.0) -> None:
        with self._test_recording_stop_lock:
            thread = self._test_recording_stop_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=max(0.0, float(timeout_s)))

    def _on_toggle_save_data(self, checked: bool) -> None:
        if checked:
            if self._test_recording_stop_active():
                self.save_data_check.blockSignals(True)
                self.save_data_check.setChecked(False)
                self.save_data_check.blockSignals(False)
                self._set_status("Recording files are still finalizing; retry shortly")
                return
            options = self._choose_recording_formats()
            if options is None:
                self.save_data_check.blockSignals(True)
                self.save_data_check.setChecked(False)
                self.save_data_check.blockSignals(False)
                return
            self._recording_format_options = options
            if self.is_comm_connected():
                self._start_data_recording(options)
            else:
                self._set_status("Recording waits for an active serial connection")
        else:
            self._stop_data_recording()

    def _choose_recording_formats(self) -> Optional[Dict[str, bool]]:
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle("Save Data Format")
        layout = QtWidgets.QVBoxLayout(dialog)
        label = QtWidgets.QLabel("选择本次保存格式：")
        layout.addWidget(label)

        csv_check = QtWidgets.QCheckBox("CSV 多文件（方便直接查看）")
        csv_check.setChecked(bool(self._recording_format_options.get("csv", True)))
        hdf5_check = QtWidgets.QCheckBox("HDF5 session.h5（训练/分析）")
        hdf5_check.setChecked(bool(self._recording_format_options.get("hdf5", True)))
        rosbag_check = QtWidgets.QCheckBox("ROS2 bag（Linux/ROS2 环境，可选）")
        rosbag_check.setChecked(bool(self._recording_format_options.get("rosbag", False)))
        tactile_full_check = QtWidgets.QCheckBox("触觉全量数据（数据量大，需要时勾选）")
        tactile_full_check.setChecked(bool(self._recording_format_options.get("tactile_full", False)))
        layout.addWidget(csv_check)
        layout.addWidget(hdf5_check)
        layout.addWidget(rosbag_check)
        layout.addWidget(tactile_full_check)

        hint = QtWidgets.QLabel("ROS2 bag 需要系统已安装 ROS2 的 rosbag2_py、rclpy 和 std_msgs。")
        hint.setWordWrap(True)
        layout.addWidget(hint)

        ok_button = getattr(QtWidgets.QDialogButtonBox, "Ok", None)
        cancel_button = getattr(QtWidgets.QDialogButtonBox, "Cancel", None)
        if ok_button is None:
            ok_button = QtWidgets.QDialogButtonBox.StandardButton.Ok
        if cancel_button is None:
            cancel_button = QtWidgets.QDialogButtonBox.StandardButton.Cancel
        buttons = QtWidgets.QDialogButtonBox(ok_button | cancel_button)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)

        exec_dialog = getattr(dialog, "exec", None) or getattr(dialog, "exec_")
        accepted = getattr(QtWidgets.QDialog, "Accepted", None)
        if accepted is None:
            accepted = QtWidgets.QDialog.DialogCode.Accepted
        if exec_dialog() != accepted:
            return None
        options = {
            "csv": csv_check.isChecked(),
            "hdf5": hdf5_check.isChecked(),
            "rosbag": rosbag_check.isChecked(),
            "tactile_full": tactile_full_check.isChecked(),
        }
        if not any(options.get(name, False) for name in ("csv", "hdf5", "rosbag")):
            QtWidgets.QMessageBox.warning(self, "Save Data Format", "至少选择一种保存格式。")
            return None
        return options

    def _maybe_write_run_data_row(
        self,
        state: HandModel,
        *,
        force: bool = False,
    ) -> bool:
        """20 Hz记录最新状态；相同P4/触觉快照不重复落盘。"""
        signature = (
            float(getattr(state, "host_rx_time", 0.0) or 0.0),
            int(getattr(state, "p4_time_ms", 0) or 0),
            getattr(state, "tactile_summary_seq", None),
            getattr(state, "tactile_full_seq", None),
        )
        if not force and signature == self._last_state_record_signature:
            return False
        self._last_state_record_signature = signature
        self._write_run_data_row(state)
        return True

    def _write_run_data_row(self, state: HandModel) -> None:
        scope_label, active_motor_mask, active_joint_mask, active_require_can = self._current_scope_record_fields()
        # 状态行使用通信层实际收包时刻，避免 UI 33 ms 刷新周期引入额外时间偏移。
        host_time = float(getattr(state, "host_rx_time", 0.0) or 0.0) or time.time()
        p4_time_ms = (
            int(getattr(state, "p4_time_ms", 0) or 0)
            if getattr(state, "has_p4_time", False)
            else ""
        )
        core_row = {
            "host_time": host_time,
            "p4_time_ms": p4_time_ms,
            "mode": self.current_mode,
            "controller_started": 1 if self.backend.is_started() else 0,
            "calib_status": getattr(state, "calib_status", ""),
            "system_state": int(getattr(state, "system_state", 0) or 0),
            "system_fault_bitmap": f"0x{int(getattr(state, 'system_fault_bitmap', 0) or 0):08X}",
            "feedback_collision_armed_mask": (
                f"0x{int(getattr(state, 'feedback_collision_armed_bitmap', 0) or 0):08X}"
            ),
            "feedback_collision_hold_mask": (
                f"0x{int(getattr(state, 'feedback_collision_hold_bitmap', 0) or 0):08X}"
            ),
            "feedback_collision_transition_count": int(
                getattr(state, "feedback_collision_transition_count", 0) or 0
            ),
            "active_scope_label": scope_label,
            "active_motor_mask": active_motor_mask,
            "active_joint_mask": active_joint_mask,
            "active_require_can": active_require_can,
            "multi_turn_unknown_mask": f"0x{int(getattr(state, 'multi_turn_unknown_bitmap', 0) or 0):08X}",
            "manual_reference_valid_mask": f"0x{int(getattr(state, 'manual_reference_valid_bitmap', 0) or 0):08X}",
            "manual_reference_crc": f"0x{int(getattr(state, 'manual_reference_crc', 0) or 0):08X}",
            "manual_reference_nvs_sequence": int(getattr(state, "manual_reference_nvs_sequence", 0) or 0),
            "manual_reference_nvs_valid": 1 if getattr(state, "manual_reference_nvs_valid", False) else 0,
            "manual_reference_save_status": int(
                getattr(state, "manual_reference_save_status", 0) or 0
            ),
            "auto_calibration_valid_mask": (
                f"0x{int(getattr(state, 'auto_calibration_valid_bitmap', 0) or 0):08X}"
            ),
            "has_sensor_data": 1 if getattr(state, "has_sensor_data", False) else 0,
            "has_servo_angle_data": 1 if getattr(state, "has_servo_angle_data", False) else 0,
            "has_servo_raw_data": 1 if getattr(state, "has_servo_raw_data", False) else 0,
            "has_system_status_data": 1 if getattr(state, "has_system_status_data", False) else 0,
            "has_fault_status_data": 1 if getattr(state, "has_fault_status_data", False) else 0,
        }
        self._data_logger.write("core_state", core_row)
        ack_sequence = int(getattr(state, "proto_ack_sequence", 0) or 0)
        if ack_sequence > 0 and ack_sequence != self._last_recorded_proto_ack_sequence:
            self._last_recorded_proto_ack_sequence = ack_sequence
            self._data_logger.event(
                "command_ack",
                source="p4",
                detail=(
                    f"cmd=0x{int(getattr(state, 'proto_ack_cmd', 0) or 0):02X} "
                    f"mode={int(getattr(state, 'proto_ack_mode', 0) or 0)} "
                    f"status={int(getattr(state, 'proto_ack_status', 0) or 0)}"
                ),
                p4_time_ms=int(getattr(state, "proto_ack_p4_time_ms", 0) or 0),
                active_scope_label=scope_label,
                active_motor_mask=active_motor_mask,
                active_joint_mask=active_joint_mask,
                host_time=host_time,
            )

        targets = self.control_panel.resolved_joint_targets()
        for i in range(ENCODER_COUNT):
            encoder = state.encoders[i] if i < len(state.encoders) else None
            self._data_logger.write(
                "joint_state",
                {
                    "host_time": host_time,
                    "p4_time_ms": p4_time_ms,
                    "joint_index": i,
                    "is_active_joint": 1 if self._mask_has_bit(active_joint_mask, i) else 0,
                    "joint_deg": state.angles[i] if i < len(state.angles) else "",
                    "joint_target_deg": targets[i] if i < len(targets) else "",
                    "joint_encoder_raw": (
                        getattr(encoder, "raw_sensor", getattr(encoder, "raw", "")) if encoder is not None else ""
                    ),
                    "joint_debug_valid": (
                        1 if i < len(getattr(state, "joint_debug_valid", [])) and state.joint_debug_valid[i] else 0
                    ),
                    "joint_debug_actual_deg": self._seq_value(getattr(state, "joint_debug_actual_deg", []), i),
                    "joint_debug_target_deg": self._seq_value(getattr(state, "joint_debug_target_deg", []), i),
                    "joint_debug_cmd_target_pos": self._seq_value(getattr(state, "joint_debug_cmd_target_pos", []), i),
                },
            )
        self._write_joint_wide_row(state, host_time, p4_time_ms, active_joint_mask, targets)

        motor_targets = self.control_panel.resolved_motor_targets()
        for i in range(MOTOR_COUNT):
            joint_idx = MOTOR_TO_JOINT_INDEX[i] if i < len(MOTOR_TO_JOINT_INDEX) else -1
            load_value = self._seq_value(getattr(state, "servo_load", []), i)
            self._data_logger.write(
                "servo_state",
                {
                    "host_time": host_time,
                    "p4_time_ms": p4_time_ms,
                    "motor_index": i,
                    "joint_index": joint_idx,
                    "is_active_motor": 1 if self._mask_has_bit(active_motor_mask, i) else 0,
                    "is_active_joint": 1 if joint_idx >= 0 and self._mask_has_bit(active_joint_mask, joint_idx) else 0,
                    "motor_target_abs": motor_targets[i] if i < len(motor_targets) else "",
                    "servo_raw": self._seq_value(getattr(state, "servo_raw_positions", []), i),
                    "servo_abs": self._seq_value(getattr(state, "servo_angles", []), i),
                    "servo_hardware_pos": self._seq_value(getattr(state, "servo_hardware_positions", []), i),
                    "online": 1 if i < len(getattr(state, "servo_online", [])) and state.servo_online[i] else 0,
                    "raw_online": (
                        1 if i < len(getattr(state, "servo_raw_online", [])) and state.servo_raw_online[i] else 0
                    ),
                    "telem_online": (
                        1 if i < len(getattr(state, "servo_telem_online", [])) and state.servo_telem_online[i] else 0
                    ),
                    "speed": self._seq_value(getattr(state, "servo_speed", []), i),
                    "load": load_value,
                    "load_mag": self._servo_load_mag(load_value),
                    "load_dir": self._servo_load_dir(load_value),
                    "current_raw": self._seq_value(getattr(state, "servo_current", []), i),
                    "voltage": self._seq_value(getattr(state, "servo_voltage", []), i),
                    "temperature": self._seq_value(getattr(state, "servo_temperature", []), i),
                    "overload_fault": (
                        1 if i < len(getattr(state, "servo_overload_fault", [])) and state.servo_overload_fault[i] else 0
                    ),
                },
            )
        self._write_servo_wide_row(
            state,
            host_time,
            p4_time_ms,
            active_motor_mask,
            active_joint_mask,
            motor_targets,
        )

        self._write_tactile_summary_rows(state, host_time, p4_time_ms)
        if self._record_tactile_full:
            self._write_tactile_full_rows(state, host_time, p4_time_ms)

    def _write_joint_wide_row(
        self,
        state: HandModel,
        host_time: float,
        p4_time_ms,
        active_joint_mask,
        targets: List[float],
    ) -> None:
        row: Dict[str, object] = {
            "host_time": host_time,
            "p4_time_ms": p4_time_ms,
        }
        for i in range(ENCODER_COUNT):
            label = f"J{i:02d}"
            encoder = state.encoders[i] if i < len(state.encoders) else None
            # 关节总表可以直接作为Replay输入，必须沿用Joint Channels的完整有效性门禁。
            current_valid = bool(
                self.control_panel._joint_encoder_status(i, state) in ("ok", "warn")
                and self.control_panel._joint_angle_feedback_valid(i, state)
                and
                self.control_panel._joint_angle_display_calibrated(i, state)
            )
            row[f"{label}_current_deg"] = (
                state.angles[i] if current_valid and i < len(state.angles) else ""
            )
            row[f"{label}_current_valid"] = 1 if current_valid else 0
            row[f"{label}_target_deg"] = targets[i] if i < len(targets) else ""
            row[f"{label}_encoder_raw"] = (
                getattr(encoder, "raw_sensor", getattr(encoder, "raw", "")) if encoder is not None else ""
            )
            row[f"{label}_active"] = 1 if self._mask_has_bit(active_joint_mask, i) else 0
            row[f"{label}_debug_valid"] = (
                1
                if i < len(getattr(state, "joint_debug_valid", []))
                and state.joint_debug_valid[i]
                else 0
            )
            row[f"{label}_debug_actual_deg"] = self._seq_value(getattr(state, "joint_debug_actual_deg", []), i)
            row[f"{label}_debug_target_deg"] = self._seq_value(getattr(state, "joint_debug_target_deg", []), i)
            row[f"{label}_debug_pid_output"] = self._seq_value(
                getattr(state, "joint_debug_loop1_output", []), i
            )
            row[f"{label}_debug_cmd_valid"] = (
                1
                if i < len(getattr(state, "joint_debug_cmd_valid", []))
                and state.joint_debug_cmd_valid[i]
                else 0
            )
            row[f"{label}_debug_cmd_target_pos"] = self._seq_value(
                getattr(state, "joint_debug_cmd_target_pos", []), i
            )
        self._data_logger.write("joint_wide", row)

    def _write_servo_wide_row(
        self,
        state: HandModel,
        host_time: float,
        p4_time_ms,
        active_motor_mask,
        active_joint_mask,
        motor_targets: List[int],
    ) -> None:
        row: Dict[str, object] = {
            "host_time": host_time,
            "p4_time_ms": p4_time_ms,
        }
        for i in range(MOTOR_COUNT):
            label = f"M{i:02d}"
            joint_idx = MOTOR_TO_JOINT_INDEX[i] if i < len(MOTOR_TO_JOINT_INDEX) else -1
            load_value = self._seq_value(getattr(state, "servo_load", []), i)
            row[f"{label}_joint_index"] = joint_idx
            row[f"{label}_target_abs"] = motor_targets[i] if i < len(motor_targets) else ""
            row[f"{label}_raw"] = self._seq_value(getattr(state, "servo_raw_positions", []), i)
            row[f"{label}_abs"] = self._seq_value(getattr(state, "servo_angles", []), i)
            row[f"{label}_hardware_pos"] = self._seq_value(getattr(state, "servo_hardware_positions", []), i)
            row[f"{label}_load"] = load_value
            row[f"{label}_load_mag"] = self._servo_load_mag(load_value)
            row[f"{label}_load_dir"] = self._servo_load_dir(load_value)
            row[f"{label}_speed"] = self._seq_value(getattr(state, "servo_speed", []), i)
            row[f"{label}_current_raw"] = self._seq_value(
                getattr(state, "servo_current", []), i
            )
            row[f"{label}_voltage"] = self._seq_value(getattr(state, "servo_voltage", []), i)
            row[f"{label}_temperature"] = self._seq_value(getattr(state, "servo_temperature", []), i)
            row[f"{label}_online"] = (
                1 if i < len(getattr(state, "servo_online", [])) and state.servo_online[i] else 0
            )
            row[f"{label}_raw_online"] = (
                1 if i < len(getattr(state, "servo_raw_online", [])) and state.servo_raw_online[i] else 0
            )
            row[f"{label}_telem_online"] = (
                1 if i < len(getattr(state, "servo_telem_online", [])) and state.servo_telem_online[i] else 0
            )
            row[f"{label}_active"] = 1 if self._mask_has_bit(active_motor_mask, i) else 0
            if joint_idx >= 0 and self._mask_has_bit(active_joint_mask, joint_idx):
                row[f"{label}_active"] = 1
        self._data_logger.write("servo_wide", row)

    def _current_scope_record_fields(self):
        panel = getattr(self, "control_panel", None)
        if panel is None:
            return "", "", "", ""
        try:
            label = panel._active_scope_label_from_ui()
            data = panel._current_active_scope_tuple()
            if data is None:
                return label, "", "", ""
            motor_mask = f"0x{int(data[0]):08X}"
            joint_mask = f"0x{int(data[1]):08X}"
            require_can = 1 if bool(data[2]) else 0
        except Exception:
            return "", "", "", ""
        return label, motor_mask, joint_mask, require_can

    @staticmethod
    def _servo_load_mag(value) -> object:
        try:
            return abs(int(value))
        except Exception:
            return ""

    @staticmethod
    def _servo_load_dir(value) -> object:
        try:
            raw = int(value)
        except Exception:
            return ""
        if raw > 0:
            return 1
        if raw < 0:
            return -1
        return 0

    def _start_joint_debug_packet_log(self) -> None:
        """Joint Debug 只进入统一多 topic 记录器，避免再生成一份旧格式重复 CSV。"""
        try:
            if not self.backend.start_joint_debug_packet_log(
                "",
                row_callback=self._write_joint_debug_packet_row,
            ):
                return
        except Exception:
            pass

    def _stop_joint_debug_packet_log(self) -> None:
        try:
            self.backend.stop_joint_debug_packet_log()
        finally:
            pass

    def _write_joint_debug_packet_row(self, row: Dict[str, object]) -> None:
        if not self._data_logger.is_running():
            return
        self._data_logger.write(
            "joint_debug_packets",
            {
                "host_time": row.get("host_time", time.time()),
                "p4_time_ms": row.get("p4_time_ms", ""),
                "joint_index": row.get("joint_index", ""),
                "valid": row.get("valid", ""),
                "target_deg": row.get("target_deg", ""),
                "actual_deg": row.get("actual_deg", ""),
                "loop1_output": row.get("loop1_output", ""),
                "cmd_valid": row.get("cmd_valid", ""),
                "cmd_target_pos": row.get("cmd_target_pos", ""),
            },
        )

    def _run_data_root_dir(self) -> str:
        return os.path.join(PROJECT_ROOT_DIR, "run_data")

    @staticmethod
    def _seq_value(values, idx: int):
        try:
            return values[idx]
        except Exception:
            return ""

    @staticmethod
    def _finger_name(idx: int) -> str:
        if 0 <= int(idx) < len(FINGER_NAMES):
            return FINGER_NAMES[int(idx)]
        return f"finger{idx}"

    @staticmethod
    def _tactile_max_force(tactile) -> float:
        vals: List[float] = []
        for finger in tactile:
            for sensor in getattr(finger, "sensors", []) or []:
                try:
                    vals.append(float(getattr(sensor, "magnitude", 0.0)))
                except Exception:
                    pass
        return max(vals) if vals else 0.0

    def _write_tactile_summary_rows(self, state: HandModel, host_time: float, p4_time_ms) -> None:
        tactile = getattr(state, "tactile_data", []) or []
        if not tactile:
            self._data_logger.write(
                "tactile_summary",
                {
                    "host_time": host_time,
                    "p4_time_ms": p4_time_ms,
                    "finger_index": -1,
                    "finger_name": "",
                    "sensor_count": 0,
                    "max_force": 0.0,
                    "mean_force": 0.0,
                    "contact_count": 0,
                    "tactile_online": 1 if getattr(state, "tactile_online", False) else 0,
                    "tactile_summary_seq": self._none_to_empty(getattr(state, "tactile_summary_seq", "")),
                    "tactile_full_seq": self._none_to_empty(getattr(state, "tactile_full_seq", "")),
                    "tactile_full_crc": self._none_to_empty(getattr(state, "tactile_full_crc", "")),
                    "tactile_full_crc_errors": int(getattr(state, "tactile_full_crc_errors", 0) or 0),
                    "tactile_full_seq_mismatches": int(getattr(state, "tactile_full_seq_mismatches", 0) or 0),
                    "tactile_full_dropped": int(getattr(state, "tactile_full_dropped", 0) or 0),
                },
            )
            self._write_tactile_summary_wide_row(state, host_time, p4_time_ms)
            return

        for finger_index, finger in enumerate(tactile):
            mags: List[float] = []
            for sensor in getattr(finger, "sensors", []) or []:
                try:
                    mags.append(float(getattr(sensor, "magnitude", 0.0)))
                except Exception:
                    pass
            self._data_logger.write(
                "tactile_summary",
                {
                    "host_time": host_time,
                    "p4_time_ms": p4_time_ms,
                    "finger_index": finger_index,
                    "finger_name": self._finger_name(finger_index),
                    "sensor_count": len(mags),
                    "max_force": max(mags) if mags else 0.0,
                    "mean_force": (sum(mags) / len(mags)) if mags else 0.0,
                    "contact_count": sum(1 for value in mags if value > 0.0),
                    "tactile_online": 1 if getattr(state, "tactile_online", False) else 0,
                    "tactile_summary_seq": self._none_to_empty(getattr(state, "tactile_summary_seq", "")),
                    "tactile_full_seq": self._none_to_empty(getattr(state, "tactile_full_seq", "")),
                    "tactile_full_crc": self._none_to_empty(getattr(state, "tactile_full_crc", "")),
                    "tactile_full_crc_errors": int(getattr(state, "tactile_full_crc_errors", 0) or 0),
                    "tactile_full_seq_mismatches": int(getattr(state, "tactile_full_seq_mismatches", 0) or 0),
                    "tactile_full_dropped": int(getattr(state, "tactile_full_dropped", 0) or 0),
                },
            )
        self._write_tactile_summary_wide_row(state, host_time, p4_time_ms)

    def _write_tactile_summary_wide_row(self, state: HandModel, host_time: float, p4_time_ms) -> None:
        tactile = getattr(state, "tactile_data", []) or []
        row: Dict[str, object] = {
            "host_time": host_time,
            "p4_time_ms": p4_time_ms,
            "tactile_online": 1 if getattr(state, "tactile_online", False) else 0,
            "tactile_summary_seq": self._none_to_empty(getattr(state, "tactile_summary_seq", "")),
            "tactile_full_seq": self._none_to_empty(getattr(state, "tactile_full_seq", "")),
            "tactile_full_crc": self._none_to_empty(getattr(state, "tactile_full_crc", "")),
            "tactile_full_crc_errors": int(getattr(state, "tactile_full_crc_errors", 0) or 0),
            "tactile_full_seq_mismatches": int(getattr(state, "tactile_full_seq_mismatches", 0) or 0),
            "tactile_full_dropped": int(getattr(state, "tactile_full_dropped", 0) or 0),
        }
        for finger_index, name in enumerate(FINGER_NAMES):
            finger = tactile[finger_index] if finger_index < len(tactile) else None
            mags: List[float] = []
            for sensor in getattr(finger, "sensors", []) or []:
                try:
                    mags.append(float(getattr(sensor, "magnitude", 0.0)))
                except Exception:
                    pass
            row[f"{name}_sensor_count"] = len(mags)
            row[f"{name}_max_force"] = max(mags) if mags else 0.0
            row[f"{name}_mean_force"] = (sum(mags) / len(mags)) if mags else 0.0
            row[f"{name}_contact_count"] = sum(1 for value in mags if value > 0.0)
        self._data_logger.write("tactile_summary_wide", row)

    def _write_tactile_full_rows(self, state: HandModel, host_time: float, p4_time_ms) -> None:
        tactile = getattr(state, "tactile_data", []) or []
        for finger_index, finger in enumerate(tactile):
            finger_wide_row: Dict[str, object] = {
                "host_time": host_time,
                "p4_time_ms": p4_time_ms,
                "finger_index": finger_index,
                "finger_name": self._finger_name(finger_index),
            }
            finger_has_full = False
            for sensor_index, sensor in enumerate(getattr(finger, "sensors", []) or []):
                forces = getattr(sensor, "contact_forces", None)
                if forces is None:
                    continue
                try:
                    rows = int(forces.shape[0])
                    cols = int(forces.shape[1])
                except Exception:
                    continue
                for row_idx in range(rows):
                    for col_idx in range(cols):
                        try:
                            fx = float(forces[row_idx][col_idx][0])
                            fy = float(forces[row_idx][col_idx][1])
                            fz = float(forces[row_idx][col_idx][2])
                            force_mag = (fx * fx + fy * fy + fz * fz) ** 0.5
                        except Exception:
                            continue
                        self._data_logger.write(
                            "tactile_full",
                            {
                                "host_time": host_time,
                                "p4_time_ms": p4_time_ms,
                                "finger_index": finger_index,
                                "finger_name": self._finger_name(finger_index),
                                "sensor_index": sensor_index,
                                "row": row_idx,
                                "col": col_idx,
                                "fx": fx,
                                "fy": fy,
                                "fz": fz,
                                "force_mag": force_mag,
                            },
                        )
                        prefix = f"S{sensor_index}_R{row_idx}_C{col_idx}"
                        finger_wide_row[f"{prefix}_fx"] = fx
                        finger_wide_row[f"{prefix}_fy"] = fy
                        finger_wide_row[f"{prefix}_fz"] = fz
                        finger_wide_row[f"{prefix}_mag"] = force_mag
                        finger_has_full = True
            if finger_has_full:
                topic = f"tactile_full_{self._finger_name(finger_index)}_wide"
                self._data_logger.write(topic, finger_wide_row)

    def _record_event(
        self,
        event_type: str,
        detail: str = "",
        source: str = "ui",
        p4_time_ms="",
    ) -> None:
        """记录操作事件；只有源自P4状态边沿的事件才携带下位机时间。"""
        if not self._data_logger.is_running():
            return
        scope_label, active_motor_mask, active_joint_mask, _active_require_can = self._current_scope_record_fields()
        self._data_logger.event(
            event_type,
            source=source,
            detail=detail,
            # UI点击尚未收到ACK时保持空值；碰撞等P4事件由调用方传入采集时间。
            p4_time_ms=p4_time_ms,
            active_scope_label=scope_label,
            active_motor_mask=active_motor_mask,
            active_joint_mask=active_joint_mask,
        )

    @staticmethod
    def _mask_has_bit(mask_hex, idx: int) -> bool:
        try:
            if isinstance(mask_hex, str):
                mask = int(mask_hex, 16) if mask_hex else 0
            else:
                mask = int(mask_hex or 0)
        except Exception:
            mask = 0
        return bool(mask & (1 << int(idx)))

    @staticmethod
    def _none_to_empty(value):
        return "" if value is None else value

    @staticmethod
    def _read_json_config(path) -> object:
        try:
            with open(path, "r", encoding="utf-8") as handle:
                return json.load(handle)
        except Exception as exc:
            return {"error": str(exc), "path": str(path)}

    def _recording_config_snapshot(self) -> dict:
        paths = {
            "calib_load_snapshot": CALIB_LOAD_SNAPSHOT_PATH,
            "calib_deg": CALIB_DEG_PATH,
            "hand_geo": HAND_GEO_PATH,
            "motor_mid_calib": MOTOR_MID_CALIB_PATH,
            "manus_calibration": MANUS_CALIBRATION_PATH,
            "raw_ik_q_target": RAW_IK_Q_TARGET_PATH,
        }
        return {name: self._read_json_config(path) for name, path in paths.items()}

    def show_servo_status_dialog(self) -> None:
        state = self._state or self.backend.get_current_state()
        rows = []
        for idx in range(MOTOR_COUNT):
            joint_idx = MOTOR_TO_JOINT_INDEX[idx] if idx < len(MOTOR_TO_JOINT_INDEX) else -1
            rows.append(
                f"M{idx:02d} -> J{joint_idx:02d} | "
                f"angle={self._seq_value(getattr(state, 'servo_angles', []), idx)} "
                f"raw={self._seq_value(getattr(state, 'servo_raw_positions', []), idx)} "
                f"online={bool(idx < len(getattr(state, 'servo_online', [])) and state.servo_online[idx])} "
                f"overload={bool(idx < len(getattr(state, 'servo_overload_fault', [])) and state.servo_overload_fault[idx])}"
            )
        QtWidgets.QMessageBox.information(self, "Servo Status", "\n".join(rows))

    def show_protocol_status_dialog(self) -> None:
        lines = [
            f"Port: {self.current_port or '-'}",
            f"Connected: {self.is_comm_connected()}",
            f"Started: {self.backend.is_started()}",
            f"Mode: {self.current_mode}",
            f"Recording: {self._data_logger.is_running()}",
        ]
        for name, value in self.backend.protocol_diag_items():
            lines.append(f"{name}: {value}")
        QtWidgets.QMessageBox.information(self, "Protocol / Comm Status", "\n".join(lines))

    def is_comm_connected(self) -> bool:
        return self.backend.is_comm_connected()

    def request_redraw(self) -> None:
        self._update_display(self._state or HandModel())

    def emit_action(self, msg: str) -> None:
        log_session("UI", msg)
        self._record_event(self._event_type_from_message(msg), detail=msg, source="ui")
        if self._action_callback:
            try:
                self._action_callback(msg)
            except Exception:
                pass
        self._set_status(msg)

    def cancel_replay(self, reason: str = "stop requested") -> None:
        if self.replay_service is None:
            return
        try:
            self.replay_service.stop(reason)
        except Exception:
            pass

    def _on_replay_event(self, event_type: str, detail: str) -> None:
        """将Replay资产与运行事件写入现有CSV/HDF5/rosbag事件链路。"""
        self._record_event(str(event_type), detail=str(detail), source="replay")
        self._set_status(f"{event_type}: {detail}")

    @staticmethod
    def _event_type_from_message(msg: str) -> str:
        text = str(msg or "").strip()
        lower = text.lower()
        if lower.startswith("start"):
            return "start"
        if lower.startswith("stop"):
            return "stop"
        if lower.startswith("pause"):
            return "pause"
        if lower.startswith("resume"):
            return "resume"
        if "clear fault" in lower:
            return "clear_fault"
        if "side adjust" in lower:
            return "side_adjust"
        if "new device reset" in lower:
            return "new_device_reset"
        if "manual" in lower:
            if "clear" in lower:
                return "manual_reference_clear"
            if "refresh" in lower or "read back" in lower:
                return "manual_reference_refresh"
            return "manual_reference_write"
        if "calib" in lower or "calibration" in lower:
            return "calibrate"
        if "rope" in lower:
            return "rope_init"
        if "ros2" in lower:
            return "ros2"
        if "manus" in lower or "teleop" in lower:
            return "teleop"
        if "fault" in lower:
            return "fault"
        return "action"

    def get_state(self) -> Optional[HandModel]:
        return self._state

    def set_state(self, state: Optional[HandModel]) -> None:
        self._state = state

    def _set_status(self, text: str) -> None:
        self._pending_status_text = str(text)
        if hasattr(self, "status_label"):
            self.status_label.setText(str(text))

    def _on_keyboard_stop(self) -> None:
        self.control_panel.stop_controller()

    def _refresh_start_indicator(self) -> None:
        started = bool(self.backend.is_started())
        self.start_indicator.setText("RUN" if started else "STOP")
        self.start_indicator.setStyleSheet(
            "background:#1e8e3e;color:white;padding:4px;" if started else "background:#c62828;color:white;padding:4px;"
        )
        self._refresh_toolbar_connection_lights()

    def _make_toolbar_connection_light(self) -> QtWidgets.QLabel:
        light = QtWidgets.QLabel()
        light.setFixedSize(14, 14)
        light.setToolTip("Lower serial disconnected")
        return light

    def _refresh_toolbar_connection_lights(self) -> None:
        connected = self.is_comm_connected()
        state = self._state or HandModel()
        protocol_healthy = connected and self._packet_status_fresh(
            state,
            "has_system_status_data",
            "system_status_last_rx_ts",
            SYSTEM_STATUS_STALE_AFTER_S,
        )
        if protocol_healthy:
            color, border = "#2e7d32", "#1b5e20"
            tip = "Lower serial connected; SYSTEM_STATUS heartbeat healthy"
        elif connected:
            color, border = "#ed8b00", "#9a6700"
            tip = "Serial open, but lower protocol status is missing or stale"
        else:
            color, border = "#c62828", "#8e0000"
            tip = "Lower serial disconnected"
        light = getattr(self, "comm_conn_light", None)
        if light is None:
            return
        light.setToolTip(tip)
        light.setStyleSheet(
            f"background:{color}; border:1px solid {border}; border-radius:7px;"
        )

    @staticmethod
    def _normalize_mode(mode_name: str) -> str:
        text = str(mode_name or "").strip()
        mapping = {
            "manual": "Monitor",
            "monitor": "Monitor",
            "teleop": "Teleoperation",
            "teleoperation": "Teleoperation",
            "replay": "Replay",
        }
        return mapping.get(text.lower(), text or "Monitor")

    @staticmethod
    def _mode_display_name(mode_name: str) -> str:
        normalized = HandGUI._normalize_mode(mode_name)
        if normalized == "Monitor":
            return "Manual"
        if normalized == "Teleoperation":
            return "Teleop"
        return normalized

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        session_log_path = None
        try:
            log_session("APP", "window closing")
            self.cancel_replay("window closed")
            if self.replay_service is not None:
                self.replay_service.shutdown()
            self.control_panel.stop_all_tests("window closed")
            self._wait_for_test_recording_stop()
            self._display_windows.close_all()
            if self._task_runtime_window is not None:
                self._task_runtime_window.close()
                self._task_runtime_window = None
            self.joint_plot.close()
            self._stop_data_recording("closed")
            self._stop_tactile_port()
            try:
                self.backend.hold_current_motor_positions(delay_s=0.15)
            except Exception:
                pass
            self.backend.shutdown(stop_lower=True)
            self.backend.unregister_update_callback(self._on_state_update)
            self.backend.unregister_joint_target_callback(self._on_joint_target_command)
            if self._on_close_extra:
                self._on_close_extra()
        finally:
            session_log_path = close_session_log()
            if session_log_path:
                print(f"[SESSION LOG] saved {session_log_path}")
            super().closeEvent(event)

    def run(self) -> None:
        self.show()
        exec_fn = getattr(self._qt_app, "exec", None) or getattr(self._qt_app, "exec_")
        exec_fn()
