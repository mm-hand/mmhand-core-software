import os
import time
from datetime import datetime
from typing import Callable, List, Optional

from data_models import HandModel
from hand_backend import HandBackend, list_ports
from protocol import (
    ENCODER_COUNT,
    MOTOR_COUNT,
    VALID_MOTOR_MASK,
    SYSTEM_FAULT_CALIBRATION,
    SYSTEM_FAULT_CAN_OFFLINE,
    SYSTEM_FAULT_JOINT16_DUAL,
    SYSTEM_FAULT_OVERLOAD,
    SYSTEM_FAULT_RELEASE_GUARD,
    SYSTEM_FAULT_SERVO_OFFLINE,
    SYSTEM_STATE_PAUSED,
    SYSTEM_STATE_RUN_DIRECT,
    SYSTEM_STATE_RUN_FEEDBACK,
)
from session_log import close_session_log, log_session
from .ui_context import HandGuiContext
from .ui_control_panels import ControlPanel
from .ui_joint_plot_view import JointRealtimePlotView
from .ui_qt_compat import QtCore, QtGui, QtWidgets, ensure_app
from .ui_services import AsyncRunDataLogger as RunDataLogger, LatestStateBuffer, clone_hand_model
from .ui_style import apply_main_window_defaults, configure_qt_application
from .ui_tendon_guard import MOTOR_TO_JOINT_INDEX
from .ui_windows import DisplayWindowManager


DESKTOP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT_ROOT_DIR = os.path.dirname(DESKTOP_DIR)
DISPLAY_POLL_MS = 33
CAMERA_UI_POLL_MS = 50


class HandGUI(QtWidgets.QMainWindow):
    def __init__(
        self,
        backend: HandBackend,
        title_suffix: str = "",
        current_port: Optional[str] = None,
        current_mode: Optional[str] = None,
        on_close_extra: Optional[Callable[[], None]] = None,
        action_callback: Optional[Callable[[str], None]] = None,
        pose_generate: bool = False,
        pose_generate_path: Optional[str] = None,
        tactile_port: Optional[str] = None,
    ):
        self._qt_app = ensure_app()
        configure_qt_application(self._qt_app)
        super().__init__()
        self.backend = backend if isinstance(backend, HandBackend) else HandBackend(backend)
        self.current_port = current_port or ""
        self.current_mode = self._normalize_mode(current_mode or "Monitor")
        self._on_close_extra = on_close_extra
        self._action_callback = action_callback
        self._pose_generate = bool(pose_generate)
        self._pose_generate_path = pose_generate_path or os.path.join(DESKTOP_DIR, "pose_fist_21deg.txt")
        self.tactile_port = tactile_port or ""
        self._tactile_state: Optional[HandModel] = None
        self._state: Optional[HandModel] = None
        self._state_buffer = LatestStateBuffer()
        self._state_buffer_version = 0
        self._data_logger = RunDataLogger(flush_every=10)
        self._recording_status_note = ""
        self._recording_csv_path = ""
        self._joint_debug_csv_path = ""
        self._last_draw = 0.0
        self._draw_interval = 0.033
        self._pending_status_text = ""
        self._last_overload_popup_mask = 0
        self._last_fault_popup_key = ""
        self._last_fault_popup_time = 0.0
        self._fault_dialog_active = False
        self._fault_stop_sent = False
        self._fault_previous_runtime_active = False
        self.context = HandGuiContext(self)
        self.tendon_guard = self.backend.tendon_guard

        self.backend.register_update_callback(self._on_state_update)
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
        if self.tactile_port:
            self._start_tactile_port(self.tactile_port)
        QtCore.QTimer.singleShot(200, self._display_windows.open_pose)
        QtCore.QTimer.singleShot(320, self._display_windows.open_tactile)
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
        self.plot_btn = QtWidgets.QPushButton("Plot")
        self.plot_btn.clicked.connect(lambda _checked=False: self._open_joint_rt_window())
        self.tactile_btn = QtWidgets.QPushButton("Tactile")
        self.tactile_btn.clicked.connect(lambda _checked=False: self._display_windows.open_tactile())
        self.comm_conn_light = self._make_toolbar_connection_light()
        self.start_indicator = QtWidgets.QLabel("STOP")
        self.start_indicator.setAlignment(QtCore.Qt.AlignCenter)
        self.start_indicator.setMinimumWidth(56)
        self._refresh_start_indicator()

        for widget in [
            QtWidgets.QLabel("Port:"), self.port_combo, self.connect_btn,
            self.save_data_check, self.plot_btn, self.tactile_btn, self.comm_conn_light, self.start_indicator,
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

        self.fault_label = self._make_top_status_label("SYS -", 150)
        self.calib_label = self._make_top_status_label("CALIB -", 82)
        self.overload_label = self._make_top_status_label("OL OK", 120)
        self.turn_label = self._make_top_status_label("TURN -", 102)
        self.delay_label = self._make_top_status_label("--", 62)

        for label in [
            self.fault_label,
            self.calib_label,
            self.overload_label,
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
        self.current_mode = self._normalize_mode(mode_name)
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
        self.backend.sync_tendon_guard(force=True)
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
        if (has_new_state or has_new_tactile) and self._data_logger.is_running():
            self._write_run_data_row(state)

    def _consume_tactile_state(self) -> bool:
        has_new, state = self.backend.consume_tactile_state_into(self._state)
        if has_new:
            self._state = state or HandModel()
            self._tactile_state = self._state
        return bool(has_new)

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
        calib = str(getattr(state, "calib_status", "IDLE") or "IDLE")
        sys_state = self._system_state_name(getattr(state, "system_state", None)) if getattr(state, "has_system_status_data", False) else "-"
        sys_faults = self._system_fault_names(getattr(state, "system_fault_bitmap", 0))
        turn_text, turn_ok, turn_tip = self._turn_known_status(state)
        sys_text, sys_tip = self._format_system_fault_summary(state, sys_state, sys_faults)
        calib_text = f"CALIB {calib}"
        overload_text, overload_tip, overload_active = self._format_overload_summary(state)
        delay_text, delay_state = self._format_delay_status(delay_ms)
        self.fault_label.setText(sys_text)
        self.fault_label.setToolTip(sys_tip)
        self.calib_label.setText(calib_text)
        self.calib_label.setToolTip(f"Calibration: {calib}")
        self.overload_label.setText(overload_text)
        self.overload_label.setToolTip(overload_tip)
        self.turn_label.setText(turn_text)
        self.turn_label.setToolTip(turn_tip)
        self.delay_label.setText(delay_text)
        self.delay_label.setToolTip(f"Delay: {delay_ms:.0f} ms" if delay_ms else "Delay: -")
        self._style_top_status_label(self.fault_label, "warn" if sys_faults else "ok")
        self._style_top_status_label(self.calib_label, self._calib_status_style(calib))
        self._style_top_status_label(self.overload_label, "warn" if overload_active else "ok")
        self._style_top_status_label(self.turn_label, "ok" if turn_ok else "warn")
        self._style_top_status_label(self.delay_label, delay_state)
        self._handle_fault_side_effects(state)

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

    @staticmethod
    def _release_guard_detail_lines(state: HandModel) -> List[str]:
        release_flags = getattr(state, "joint_reverse_release_fault", []) or []
        return [f"J{i:02d}" for i, faulted in enumerate(release_flags) if faulted]

    def _format_overload_summary(self, state: HandModel) -> tuple[str, str, bool]:
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

    def _build_fault_summary(self, state: HandModel) -> dict:
        bitmap = self._fault_bitmap_value(state)
        overload_items = self._overload_detail_items(state)
        overload_mask = self._overload_mask_from_state(state)
        release_joints = self._release_guard_detail_lines(state)
        release_mask = 0
        release_flags = getattr(state, "joint_reverse_release_fault", []) or []
        for i, faulted in enumerate(release_flags):
            if faulted:
                release_mask |= (1 << i)

        lines: List[str] = []
        suggestions: List[str] = []
        titles: List[str] = []

        if (bitmap & SYSTEM_FAULT_OVERLOAD) or overload_items:
            titles.append("舵机过载")
            if overload_items:
                lines.append("舵机过载：")
                lines.extend(f"  M{i:02d} load={load}" for i, load in overload_items)
            else:
                lines.append("舵机过载：触发电机未知")
            suggestions.append("检查机构是否卡住、绳索是否过紧、目标角度是否过大，以及是否误加载了旧标定。")

        if (bitmap & SYSTEM_FAULT_RELEASE_GUARD) or release_joints:
            titles.append("反向松绳保护")
            if release_joints:
                lines.append("反向松绳保护：" + ", ".join(release_joints))
            else:
                lines.append("反向松绳保护：触发关节未知")
            suggestions.append("检查 tendon 方向、标定方向、磁编码器方向和绳路，确认松绳时关节角度没有反向异常。")

        if bitmap & SYSTEM_FAULT_SERVO_OFFLINE:
            titles.append("舵机离线")
            lines.append("舵机离线：下位机未获得完整舵机反馈。")
            suggestions.append("检查舵机电源、总线、ID 映射和接线。")

        if bitmap & SYSTEM_FAULT_CAN_OFFLINE:
            titles.append("CAN/磁编码器离线")
            lines.append("CAN/磁编码器离线：下位机未获得有效磁编码器/CAN 数据。")
            suggestions.append("检查 S3/P4 CAN 通信、磁编码器供电、SPI 接线和传感器板状态。")

        if bitmap & SYSTEM_FAULT_CALIBRATION:
            titles.append("标定故障")
            lines.append("标定故障：下位机标定流程失败或被保护中断。")
            suggestions.append("检查负载阈值、编码器方向、机械限位和绳索预紧后重新标定。")

        if bitmap & SYSTEM_FAULT_JOINT16_DUAL:
            titles.append("J16 双电机约束异常")
            lines.append("J16 双电机约束异常：主/副电机反馈差异超出限制。")
            suggestions.append("检查 J16 双电机映射、副电机反馈、对抗绳路和预紧。")

        if not lines and bitmap != 0:
            titles.append("未知系统故障")
            lines.append(f"未知系统故障 bitmap=0x{bitmap:08X}")
            suggestions.append("记录日志后检查下位机状态机和通信包解析。")

        active = bool(lines or bitmap != 0)
        offline_bits = SYSTEM_FAULT_CAN_OFFLINE | SYSTEM_FAULT_SERVO_OFFLINE
        offline_only = active and (bitmap & ~offline_bits) == 0 and not overload_items and not release_joints
        title = " / ".join(titles[:2]) if titles else "系统状态正常"
        if len(titles) > 2:
            title += f" 等 {len(titles)} 项"
        key = f"{bitmap:08X}:{overload_mask:08X}:{release_mask:08X}"
        return {
            "active": active,
            "title": title,
            "lines": lines,
            "suggestions": suggestions,
            "key": key,
            "offline_only": offline_only,
            "bitmap": bitmap,
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
            self._last_fault_popup_key = ""
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

        key = str(summary.get("key") or "")
        now = time.time()
        if key == self._last_fault_popup_key:
            return
        if self._fault_dialog_active or (now - self._last_fault_popup_time) < 1.0:
            return

        self._last_fault_popup_key = key
        self._last_fault_popup_time = now
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
        self._last_fault_popup_key = ""
        self._last_fault_popup_time = 0.0
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

        release_lines = self._release_guard_detail_lines(state)
        if release_lines:
            lines.append("Release guard joints: " + ", ".join(release_lines))
        elif "RELEASE" in sys_faults:
            lines.append("Release guard joints: unknown")

        summary = self._build_fault_summary(state)
        if summary.get("active"):
            body = self._fault_body_text(summary)
            if body:
                lines.append("")
                lines.append(body)

        return text, "\n".join(lines)

    def _turn_known_status(self, state: HandModel) -> tuple[str, bool, str]:
        if not getattr(state, "has_system_status_data", False):
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
            (SYSTEM_FAULT_OVERLOAD, "OVERLOAD"),
            (SYSTEM_FAULT_RELEASE_GUARD, "RELEASE"),
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

    def send_joint_targets(self, angles: List[float], source: str = "", live: bool = False) -> None:
        guard = getattr(self.control_panel, "feedback_command_allowed", None)
        if callable(guard) and not guard(source=source, show_status=True):
            return
        try:
            self.control_panel._refresh_tendon_guard_from_ui()
        except Exception:
            pass
        result = self.backend.send_joint_targets(angles, source=source, live=live)
        if result.message:
            self._set_status(result.message)
        if source and not result.blocked:
            self.emit_action(source)

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
        scope_label, active_motor_mask, active_joint_mask, active_require_can = self._current_scope_record_fields()
        row = {
            "host_time": time.time(),
            "device_timestamp": getattr(state, "timestamp", 0.0),
            "mode": self.current_mode,
            "controller_started": 1 if self.backend.is_started() else 0,
            "calib_status": getattr(state, "calib_status", ""),
            "active_scope_label": scope_label,
            "active_motor_mask_hex": active_motor_mask,
            "active_joint_mask_hex": active_joint_mask,
            "active_require_can": active_require_can,
            "multi_turn_unknown_mask_hex": f"0x{int(getattr(state, 'multi_turn_unknown_bitmap', 0)):08X}",
            "has_sensor_data": 1 if getattr(state, "has_sensor_data", False) else 0,
            "has_servo_angle_data": 1 if getattr(state, "has_servo_angle_data", False) else 0,
            "has_servo_raw_data": 1 if getattr(state, "has_servo_raw_data", False) else 0,
        }
        for i in range(ENCODER_COUNT):
            row[f"joint_deg_j{i:02d}"] = state.angles[i] if i < len(state.angles) else ""
            row[f"joint_encoder_raw_j{i:02d}"] = getattr(state.encoders[i], "raw_sensor", state.encoders[i].raw) if i < len(state.encoders) else ""
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
            load_value = self._seq_value(getattr(state, "servo_load", []), i)
            row[f"servo_load_m{i:02d}"] = load_value
            row[f"servo_load_mag_m{i:02d}"] = self._servo_load_mag(load_value)
            row[f"servo_load_dir_m{i:02d}"] = self._servo_load_dir(load_value)
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
            "active_scope_label", "active_motor_mask_hex", "active_joint_mask_hex",
            "active_require_can", "multi_turn_unknown_mask_hex",
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
        fields += [f"servo_load_mag_m{i:02d}" for i in range(MOTOR_COUNT)]
        fields += [f"servo_load_dir_m{i:02d}" for i in range(MOTOR_COUNT)]
        fields += [f"servo_voltage_m{i:02d}" for i in range(MOTOR_COUNT)]
        fields += [f"servo_temperature_m{i:02d}" for i in range(MOTOR_COUNT)]
        fields += [f"servo_overload_fault_m{i:02d}" for i in range(MOTOR_COUNT)]
        fields += ["tactile_fingers", "tactile_max_force"]
        return fields

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

    def _start_joint_debug_packet_log(self, csv_path: str) -> None:
        path = self._build_joint_debug_packet_csv_path(csv_path)
        try:
            if not self.backend.start_joint_debug_packet_log(path):
                self._joint_debug_csv_path = ""
                return
            self._joint_debug_csv_path = path
        except Exception:
            self._joint_debug_csv_path = ""

    def _stop_joint_debug_packet_log(self) -> None:
        try:
            self.backend.stop_joint_debug_packet_log()
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
        color = "#2e7d32" if connected else "#c62828"
        border = "#1b5e20" if connected else "#8e0000"
        tip = "Lower serial connected" if connected else "Lower serial disconnected"
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
        session_log_path = None
        try:
            log_session("APP", "window closing")
            self.control_panel.stop_all_tests("window closed")
            self._display_windows.close_all()
            self.joint_plot.close()
            self._stop_data_recording("closed")
            self._stop_tactile_port()
            try:
                self.backend.hold_current_motor_positions(delay_s=0.15)
            except Exception:
                pass
            self.backend.shutdown(stop_lower=True)
            self.backend.unregister_update_callback(self._on_state_update)
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
