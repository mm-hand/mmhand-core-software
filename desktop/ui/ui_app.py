import os
import time
from datetime import datetime
from typing import Callable, List, Optional

from comm_layer import list_ports
from core_logic import HandController
from data_models import HandModel
from protocol import ENCODER_COUNT, MOTOR_COUNT
from .ui_context import HandGuiContext
from .ui_control_panels import ControlPanel
from .ui_joint_plot_view import JointRealtimePlotView
from .ui_qt_compat import QtCore, QtGui, QtWidgets, ensure_app
from .ui_services import AsyncRunDataLogger as RunDataLogger, LatestStateBuffer, clone_hand_model
from .ui_style import apply_main_window_defaults, configure_qt_application
from .ui_tendon_guard import TendonGuard, MOTOR_TO_JOINT_INDEX
from .ui_windows import DisplayWindowManager


DESKTOP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT_ROOT_DIR = os.path.dirname(DESKTOP_DIR)
DISPLAY_POLL_MS = 50
CAMERA_UI_POLL_MS = 50


class HandGUI(QtWidgets.QMainWindow):
    def __init__(
        self,
        controller: HandController,
        title_suffix: str = "",
        current_port: Optional[str] = None,
        current_mode: Optional[str] = None,
        on_close_extra: Optional[Callable[[], None]] = None,
        action_callback: Optional[Callable[[str], None]] = None,
        pose_generate: bool = False,
        pose_generate_path: Optional[str] = None,
    ):
        self._qt_app = ensure_app()
        configure_qt_application(self._qt_app)
        super().__init__()
        self.controller = controller
        self.current_port = current_port or ""
        self.current_mode = self._normalize_mode(current_mode or "Monitor")
        self._on_close_extra = on_close_extra
        self._action_callback = action_callback
        self._pose_generate = bool(pose_generate)
        self._pose_generate_path = pose_generate_path or os.path.join(DESKTOP_DIR, "pose_fist_21deg.txt")
        self._state: Optional[HandModel] = None
        self._state_buffer = LatestStateBuffer()
        self._state_buffer_version = 0
        self._data_logger = RunDataLogger(flush_every=10)
        self._recording_status_note = ""
        self._recording_csv_path = ""
        self._joint_debug_csv_path = ""
        self._last_draw = 0.0
        self._draw_interval = 0.1
        self._pending_status_text = ""
        self.context = HandGuiContext(self)
        self.tendon_guard = TendonGuard(os.path.join(DESKTOP_DIR, "config", "tendon_manual_calib.json"))

        self.controller.register_update_callback(self._on_state_update)
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
        self._camera_timer = QtCore.QTimer(self)
        self._camera_timer.timeout.connect(self._display_windows.update_cameras)
        self._camera_timer.start(CAMERA_UI_POLL_MS)
        QtCore.QTimer.singleShot(200, self._display_windows.open_pose)
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
        self.save_data_check.setChecked(True)
        self.save_data_check.toggled.connect(self._on_toggle_save_data)
        self.plot_btn = QtWidgets.QPushButton("Plot")
        self.plot_btn.clicked.connect(lambda _checked=False: self._open_joint_rt_window())
        self.start_indicator = QtWidgets.QLabel("STOP")
        self.start_indicator.setAlignment(QtCore.Qt.AlignCenter)
        self.start_indicator.setMinimumWidth(56)
        self._refresh_start_indicator()

        for widget in [
            QtWidgets.QLabel("Port:"), self.port_combo, self.connect_btn,
            self.save_data_check, self.plot_btn, self.start_indicator,
        ]:
            toolbar.addWidget(widget)

        central = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.setCentralWidget(central)
        self.control_panel = ControlPanel(self.context)
        self.control_panel.sync_app_mode(self.current_mode)
        central.addWidget(self.control_panel)
        central.setStretchFactor(0, 1)

        initial_status = self._pending_status_text or ("Ready" if self.is_comm_connected() else "Disconnected")
        self.status_label = QtWidgets.QLabel(initial_status)
        self.delay_label = QtWidgets.QLabel("")
        self.fault_label = QtWidgets.QLabel("Faults: none")
        self.statusBar().addWidget(self.status_label, 1)
        self.statusBar().addPermanentWidget(self.fault_label)
        self.statusBar().addPermanentWidget(self.delay_label)

        self.joint_plot = JointRealtimePlotView(self.context, self)

    def _attach_mode_menu(self) -> None:
        action_cls = getattr(QtGui, "QAction", None) or getattr(QtWidgets, "QAction")
        group_cls = getattr(QtGui, "QActionGroup", None) or getattr(QtWidgets, "QActionGroup", None)
        menu = self.menuBar().addMenu("Mode")
        self._mode_action_group = group_cls(self) if group_cls is not None else None
        if self._mode_action_group is not None:
            self._mode_action_group.setExclusive(True)
        self._mode_actions = {}
        for label in ["Manual", "Teleop", "Algorithm"]:
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
        for action in [refresh_ports, connect, refresh_cameras, servo_status, protocol_status]:
            menu.addAction(action)

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
        try:
            if self._on_state_update not in self.controller.update_callbacks:
                self.controller.register_update_callback(self._on_state_update)
        except Exception:
            pass

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

    def _disconnect_current_port(self, status: str = "Disconnected", clear_selection: bool = True) -> None:
        self._stop_data_recording("disconnected")
        self.control_panel.stop_all_tests("disconnected", update_status=False)
        self.controller.shutdown()
        self.current_port = ""
        self._state = None
        self._state_buffer_version = 0
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
        if self.controller.initialize(port):
            self.current_port = str(port)
            self._set_status(f"Connected: {self.current_port}")
            self._on_comm_ready()
            return True
        else:
            QtWidgets.QMessageBox.critical(self, "Connection Failed", f"Cannot open port: {port}")
            return False

    def _switch_mode(self, mode_name: str) -> None:
        self.current_mode = self._normalize_mode(mode_name)
        self.controller.set_pid_control(self.current_mode != "Monitor")
        self.control_panel.sync_app_mode(self.current_mode)
        self._sync_mode_actions()
        self._set_status(f"Mode: {self._mode_display_name(self.current_mode)}")

    def _on_comm_ready(self) -> None:
        self._refresh_start_indicator()
        self.tendon_guard.sync_to_controller(self.controller, force=True)
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
        state = self._state or HandModel()
        if has_new_state or not self.is_comm_connected():
            self._update_display(state)
        if has_new_state and self._data_logger.is_running():
            self._write_run_data_row(state)

    def _update_display(self, state: HandModel) -> None:
        now = time.time()
        if now - self._last_draw < self._draw_interval:
            return
        self._last_draw = now
        self.control_panel.update_state(state)
        self._display_windows.update_views(state)
        self.joint_plot.append_state(state)
        self.joint_plot.update_plot()
        self._update_status_text(state)
        if self.current_mode == "Algorithm":
            line = " ".join([f"J{i}:{state.angles[i]:.1f}" for i in range(min(ENCODER_COUNT, len(state.angles)))])
            self.control_panel.append_algorithm_output(line)

    def _update_status_text(self, state: HandModel) -> None:
        self._refresh_start_indicator()
        delay_ms = 0.0
        ts = float(getattr(state, "timestamp", 0.0) or 0.0)
        if ts > 0:
            delay_ms = max(0.0, (time.time() - ts) * 1000.0)
        self.delay_label.setText(f"{delay_ms:.0f} ms" if delay_ms else "")
        overload = [str(i) for i, f in enumerate(getattr(state, "servo_overload_fault", []) or []) if f]
        release = [f"J{i}" for i, f in enumerate(getattr(state, "joint_reverse_release_fault", []) or []) if f]
        mode = self.current_mode
        calib = str(getattr(state, "calib_status", "IDLE"))
        self.fault_label.setText(
            f"Mode: {mode} | Calib: {calib} | "
            f"Overload: {','.join(overload) if overload else 'none'} | "
            f"Release: {','.join(release) if release else 'none'}"
        )

    def _clear_live_displays(self) -> None:
        self.control_panel.clear_live_state()
        self._display_windows.update_views(HandModel())

    def _open_joint_rt_window(self) -> None:
        if not self.joint_plot.isVisible():
            self.joint_plot.show()
        self.joint_plot.raise_()
        self.joint_plot.activateWindow()

    def send_joint_targets(self, angles: List[float], source: str = "", live: bool = False) -> None:
        try:
            self.control_panel._refresh_tendon_guard_from_ui()
        except Exception:
            pass
        self.tendon_guard.sync_to_controller(self.controller, force=False)
        guarded, blocked, msg = self.tendon_guard.apply(self._state or self.controller.current_state, angles, source=source)
        if msg:
            self._set_status(msg)
        if live:
            self.controller.set_target_angles_live(guarded)
        else:
            self.controller.set_target_angles(guarded)
        if source and not blocked:
            self.emit_action(source)

    def draw_pose_angles(self, angles: List[float]) -> None:
        self._display_windows.open_pose()
        comp = self._display_windows.components.get("pose")
        if comp is not None:
            comp.draw_angles(angles, None, generate=False)

    def start_calibration(self) -> None:
        def progress(current: int, total: int, msg: str):
            QtCore.QTimer.singleShot(0, lambda: self._set_status(f"Calibration: {current + 1}/{total} {msg}"))

        def done(success: bool, _zero_raw):
            QtCore.QTimer.singleShot(0, lambda: QtWidgets.QMessageBox.information(
                self, "Calibration", "Calibration completed" if success else "Calibration failed"
            ))

        self.controller.calibrate(progress_cb=progress, done_cb=done)
        self._set_status("Calibration in progress...")

    def _start_data_recording(self) -> None:
        path = self._build_run_data_csv_path()
        self._data_logger.start(path, fieldnames=self._build_run_data_fieldnames())
        self._recording_status_note = path
        self._recording_csv_path = path
        self._start_joint_debug_packet_log(path)
        self._set_status(f"Recording: {path}")

    def _stop_data_recording(self, reason: str = "stopped") -> None:
        if self._data_logger.is_running():
            path = self._data_logger.path
            self._data_logger.stop()
            debug_path = self._joint_debug_csv_path
            self._stop_joint_debug_packet_log()
            try:
                self.joint_plot.save_snapshot(path, reason=reason)
                if debug_path:
                    self.joint_plot.save_debug_packet_snapshot(debug_path, reason=reason)
            except Exception as exc:
                self._set_status(f"Plot snapshot failed: {exc}")
            self._set_status(f"Recording {reason}: {path}")
        else:
            self._stop_joint_debug_packet_log()

    def start_recording_for_test(self, reason: str = "test") -> None:
        if self._data_logger.is_running():
            self._stop_data_recording(f"restart by {reason}")
        if not self.save_data_check.isChecked():
            self.save_data_check.blockSignals(True)
            self.save_data_check.setChecked(True)
            self.save_data_check.blockSignals(False)
        self._start_data_recording()

    def stop_recording_for_test(self, reason: str = "test stopped") -> None:
        self._stop_data_recording(reason)
        self.save_data_check.blockSignals(True)
        self.save_data_check.setChecked(False)
        self.save_data_check.blockSignals(False)

    def _on_toggle_save_data(self, checked: bool) -> None:
        if checked:
            if self.is_comm_connected():
                self._start_data_recording()
            else:
                self._set_status("Recording waits for an active serial connection")
        else:
            self._stop_data_recording()

    def _write_run_data_row(self, state: HandModel) -> None:
        row = {
            "host_time": time.time(),
            "device_timestamp": getattr(state, "timestamp", 0.0),
            "mode": self.current_mode,
            "controller_started": 1 if self.controller.is_started() else 0,
            "calib_status": getattr(state, "calib_status", ""),
            "has_sensor_data": 1 if getattr(state, "has_sensor_data", False) else 0,
            "has_servo_angle_data": 1 if getattr(state, "has_servo_angle_data", False) else 0,
            "has_servo_raw_data": 1 if getattr(state, "has_servo_raw_data", False) else 0,
        }
        for i in range(ENCODER_COUNT):
            row[f"joint_deg_j{i:02d}"] = state.angles[i] if i < len(state.angles) else ""
            row[f"joint_encoder_raw_j{i:02d}"] = state.encoders[i].raw if i < len(state.encoders) else ""
            row[f"joint_reverse_release_fault_j{i:02d}"] = (
                1 if i < len(getattr(state, "joint_reverse_release_fault", [])) and state.joint_reverse_release_fault[i] else 0
            )
            row[f"joint_debug_valid_j{i:02d}"] = (
                1 if i < len(getattr(state, "joint_debug_valid", [])) and state.joint_debug_valid[i] else 0
            )
            row[f"joint_debug_target_deg_j{i:02d}"] = self._seq_value(getattr(state, "joint_debug_target_deg", []), i)
            row[f"joint_debug_actual_deg_j{i:02d}"] = self._seq_value(getattr(state, "joint_debug_actual_deg", []), i)
            row[f"joint_debug_cmd_target_pos_j{i:02d}"] = self._seq_value(getattr(state, "joint_debug_cmd_target_pos", []), i)
            row[f"joint_debug_timestamp_ms_j{i:02d}"] = self._seq_value(getattr(state, "joint_debug_timestamp_ms", []), i)
        targets = self.control_panel.resolved_joint_targets()
        for i in range(ENCODER_COUNT):
            row[f"joint_target_deg_j{i:02d}"] = targets[i] if i < len(targets) else ""
        motor_targets = self.control_panel.resolved_motor_targets()
        for i in range(MOTOR_COUNT):
            row[f"motor_target_abs_m{i:02d}"] = motor_targets[i] if i < len(motor_targets) else ""
            row[f"servo_angle_m{i:02d}"] = self._seq_value(getattr(state, "servo_angles", []), i)
            row[f"servo_online_m{i:02d}"] = 1 if i < len(getattr(state, "servo_online", [])) and state.servo_online[i] else 0
            row[f"servo_raw_m{i:02d}"] = self._seq_value(getattr(state, "servo_raw_positions", []), i)
            row[f"servo_raw_online_m{i:02d}"] = (
                1 if i < len(getattr(state, "servo_raw_online", [])) and state.servo_raw_online[i] else 0
            )
            row[f"servo_speed_m{i:02d}"] = self._seq_value(getattr(state, "servo_speed", []), i)
            row[f"servo_load_m{i:02d}"] = self._seq_value(getattr(state, "servo_load", []), i)
            row[f"servo_voltage_m{i:02d}"] = self._seq_value(getattr(state, "servo_voltage", []), i)
            row[f"servo_temperature_m{i:02d}"] = self._seq_value(getattr(state, "servo_temperature", []), i)
            row[f"servo_overload_fault_m{i:02d}"] = (
                1 if i < len(getattr(state, "servo_overload_fault", [])) and state.servo_overload_fault[i] else 0
            )
        tactile = getattr(state, "tactile_data", []) or []
        row["tactile_fingers"] = len(tactile)
        row["tactile_max_force"] = self._tactile_max_force(tactile)
        self._data_logger.write_row(row)

    def _build_run_data_fieldnames(self) -> List[str]:
        fields = [
            "host_time", "device_timestamp", "mode", "controller_started", "calib_status",
            "has_sensor_data", "has_servo_angle_data", "has_servo_raw_data",
        ]
        fields += [f"joint_deg_j{i:02d}" for i in range(ENCODER_COUNT)]
        fields += [f"joint_encoder_raw_j{i:02d}" for i in range(ENCODER_COUNT)]
        fields += [f"joint_reverse_release_fault_j{i:02d}" for i in range(ENCODER_COUNT)]
        fields += [f"joint_debug_valid_j{i:02d}" for i in range(ENCODER_COUNT)]
        fields += [f"joint_debug_target_deg_j{i:02d}" for i in range(ENCODER_COUNT)]
        fields += [f"joint_debug_actual_deg_j{i:02d}" for i in range(ENCODER_COUNT)]
        fields += [f"joint_debug_cmd_target_pos_j{i:02d}" for i in range(ENCODER_COUNT)]
        fields += [f"joint_debug_timestamp_ms_j{i:02d}" for i in range(ENCODER_COUNT)]
        fields += [f"joint_target_deg_j{i:02d}" for i in range(ENCODER_COUNT)]
        fields += [f"motor_target_abs_m{i:02d}" for i in range(MOTOR_COUNT)]
        fields += [f"servo_angle_m{i:02d}" for i in range(MOTOR_COUNT)]
        fields += [f"servo_online_m{i:02d}" for i in range(MOTOR_COUNT)]
        fields += [f"servo_raw_m{i:02d}" for i in range(MOTOR_COUNT)]
        fields += [f"servo_raw_online_m{i:02d}" for i in range(MOTOR_COUNT)]
        fields += [f"servo_speed_m{i:02d}" for i in range(MOTOR_COUNT)]
        fields += [f"servo_load_m{i:02d}" for i in range(MOTOR_COUNT)]
        fields += [f"servo_voltage_m{i:02d}" for i in range(MOTOR_COUNT)]
        fields += [f"servo_temperature_m{i:02d}" for i in range(MOTOR_COUNT)]
        fields += [f"servo_overload_fault_m{i:02d}" for i in range(MOTOR_COUNT)]
        fields += ["tactile_fingers", "tactile_max_force"]
        return fields

    def _start_joint_debug_packet_log(self, csv_path: str) -> None:
        comm = getattr(self.controller, "comm", None)
        starter = getattr(comm, "start_joint_debug_packet_log", None)
        if starter is None:
            self._joint_debug_csv_path = ""
            return
        path = self._build_joint_debug_packet_csv_path(csv_path)
        try:
            starter(path)
            self._joint_debug_csv_path = path
        except Exception:
            self._joint_debug_csv_path = ""

    def _stop_joint_debug_packet_log(self) -> None:
        comm = getattr(self.controller, "comm", None)
        stopper = getattr(comm, "stop_joint_debug_packet_log", None)
        if stopper is None:
            self._joint_debug_csv_path = ""
            return
        try:
            stopper()
        finally:
            pass

    @staticmethod
    def _build_joint_debug_packet_csv_path(csv_path: str) -> str:
        base, _ext = os.path.splitext(os.path.abspath(csv_path))
        return f"{base}_joint_debug_packets.csv"

    def _run_data_root_dir(self) -> str:
        return os.path.join(PROJECT_ROOT_DIR, "run_data")

    def _build_run_data_csv_path(self) -> str:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        return os.path.join(self._run_data_root_dir(), f"hand_run_{ts}.csv")

    @staticmethod
    def _seq_value(values, idx: int):
        try:
            return values[idx]
        except Exception:
            return ""

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

    def show_servo_status_dialog(self) -> None:
        state = self._state or self.controller.current_state
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
        comm = getattr(self.controller, "comm", None)
        lines = [
            f"Port: {self.current_port or '-'}",
            f"Connected: {self.is_comm_connected()}",
            f"Started: {self.controller.is_started()}",
            f"Mode: {self.current_mode}",
            f"Recording: {self._data_logger.is_running()}",
        ]
        if comm is not None:
            for name in [
                "_rx_diag_total_packets", "_rx_diag_sensor_packets", "_rx_diag_joint_debug_packets",
                "_rx_diag_bytes", "_tx_diag_total_cmds", "_tx_diag_angle_live_cmds", "_tx_diag_bytes",
            ]:
                lines.append(f"{name}: {getattr(comm, name, '-')}")
        QtWidgets.QMessageBox.information(self, "Protocol / Comm Status", "\n".join(lines))

    def is_comm_connected(self) -> bool:
        comm = getattr(self.controller, "comm", None)
        return bool(comm and getattr(comm, "serial", None) and getattr(comm.serial, "is_open", False))

    def request_redraw(self) -> None:
        self._update_display(self._state or HandModel())

    def emit_action(self, msg: str) -> None:
        if self._action_callback:
            try:
                self._action_callback(msg)
            except Exception:
                pass
        self._set_status(msg)

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
        started = bool(self.controller.is_started())
        self.start_indicator.setText("RUN" if started else "STOP")
        self.start_indicator.setStyleSheet(
            "background:#1e8e3e;color:white;padding:4px;" if started else "background:#c62828;color:white;padding:4px;"
        )

    @staticmethod
    def _normalize_mode(mode_name: str) -> str:
        text = str(mode_name or "").strip()
        mapping = {
            "manual": "Monitor",
            "monitor": "Monitor",
            "teleop": "Teleoperation",
            "teleoperation": "Teleoperation",
            "algorithm": "Algorithm",
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
        try:
            self.control_panel.stop_all_tests("window closed")
            self._display_windows.close_all()
            self.joint_plot.close()
            self._stop_data_recording("closed")
            self.controller.shutdown()
            if self._on_close_extra:
                self._on_close_extra()
        finally:
            super().closeEvent(event)

    def run(self) -> None:
        self.show()
        exec_fn = getattr(self._qt_app, "exec", None) or getattr(self._qt_app, "exec_")
        exec_fn()
