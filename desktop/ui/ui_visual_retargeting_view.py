"""ui_visual_retargeting_view.py - MMHand Desktop 模块契约。

职责：管理视觉遥操线程、目标适配和画面状态。
线程：Qt GUI 主线程执行；耗时工作必须委托后台服务，禁止阻塞界面事件循环。
输入：用户操作、HandModel 只读快照以及 HandBackend/服务返回结果。
输出：Qt 控件状态、用户提示和经门面提交的应用命令。
状态所有权：仅拥有本视图控件和瞬时交互状态；设备真值由 core 层拥有。
禁止依赖：禁止直接导入串口通信和底层控制实现，禁止在 UI 中复制协议或安全规则。
"""
import threading
import traceback
from typing import Callable, Optional

from desktop.core.protocol import ENCODER_COUNT
from desktop.teleop.joint_target_adapter import URDF_DEG, adapt_joint_targets
from .ui_qt_compat import QtCore, QtGui, QtWidgets, Signal


class _LazyCameraCombo(QtWidgets.QComboBox):
    popup_requested = Signal()

    def showPopup(self):  # noqa: N802 - Qt override
        self.popup_requested.emit()
        super().showPopup()


class _VisualRetargetingBridge(QtCore.QObject):
    frame_ready = Signal(QtGui.QImage, float)
    joints_ready = Signal(list, int)
    cameras_ready = Signal(list)
    status_changed = Signal(str)
    stopped = Signal()


class _CameraScanRunner:
    def __init__(self, bridge: _VisualRetargetingBridge, max_index: int = 8) -> None:
        self.bridge = bridge
        self.max_index = int(max_index)
        self.thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self.thread is not None and self.thread.is_alive():
            return
        self.thread = threading.Thread(target=self._run, name="VisualCameraScan", daemon=True)
        self.thread.start()

    def is_running(self) -> bool:
        return self.thread is not None and self.thread.is_alive()

    def _run(self) -> None:
        cameras = []
        try:
            import cv2
            from desktop.teleop.Posture.camera_mocap import open_camera

            # Probing absent numeric indices is expected on Windows. Silence only
            # this scan so an unavailable Camera 2..8 does not look like the
            # selected camera has failed to start.
            logging = getattr(getattr(cv2, "utils", None), "logging", None)
            old_log_level = None
            if logging is not None:
                try:
                    old_log_level = logging.getLogLevel()
                    logging.setLogLevel(logging.LOG_LEVEL_SILENT)
                except Exception:
                    old_log_level = None
            try:
                for idx in range(self.max_index + 1):
                    cap = None
                    try:
                        cap = open_camera(idx, cv2)
                        if cap is not None and cap.isOpened():
                            ok, _frame = cap.read()
                            if ok:
                                cameras.append(idx)
                    except Exception:
                        pass
                    finally:
                        if cap is not None:
                            try:
                                cap.release()
                            except Exception:
                                pass
            finally:
                if logging is not None and old_log_level is not None:
                    try:
                        logging.setLogLevel(old_log_level)
                    except Exception:
                        pass
        except Exception as exc:
            self.bridge.status_changed.emit(f"Camera scan failed: {exc}")
        self.bridge.cameras_ready.emit(cameras)


class _PostureMocapRunner:
    def __init__(self, bridge: _VisualRetargetingBridge, camera_id: int, num_hands: int) -> None:
        self.bridge = bridge
        self.camera_id = int(camera_id)
        self.num_hands = int(num_hands)
        self.stop_event = threading.Event()
        self.thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self.thread = threading.Thread(target=self._run, name="PostureMocap", daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()

    def is_running(self) -> bool:
        return self.thread is not None and self.thread.is_alive()

    def _run(self) -> None:
        try:
            import cv2
            from desktop.teleop.Posture.camera_mocap import run_mocap_loop

            def on_frame(frame_bgr, fps: float) -> None:
                frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                height, width, channels = frame_rgb.shape
                bytes_per_line = channels * width
                image = QtGui.QImage(
                    frame_rgb.data,
                    width,
                    height,
                    bytes_per_line,
                    QtGui.QImage.Format_RGB888,
                ).copy()
                self.bridge.frame_ready.emit(image, fps)

            def on_joints(timestamp_ms: int, joint_names, joint_values, encoder_degrees) -> None:
                del joint_names, joint_values
                targets = [float(v) for v in list(encoder_degrees)]
                self.bridge.joints_ready.emit(targets, int(timestamp_ms))

            run_mocap_loop(
                camera_id=self.camera_id,
                num_hands=self.num_hands,
                stop_event=self.stop_event,
                status_callback=self.bridge.status_changed.emit,
                frame_callback=on_frame,
                joint_callback=on_joints,
            )
        except Exception as exc:
            self.bridge.status_changed.emit(f"Posture mocap error: {exc}")
            traceback.print_exc()
        finally:
            self.bridge.stopped.emit()


class VisualRetargetingView(QtWidgets.QWidget):
    def __init__(self, context, selected_joint_mask_getter: Optional[Callable[[], int]] = None, parent=None):
        super().__init__(parent)
        self.context = context
        self.selected_joint_mask_getter = selected_joint_mask_getter
        self.bridge = _VisualRetargetingBridge()
        self._runner: Optional[_PostureMocapRunner] = None
        self._scanner: Optional[_CameraScanRunner] = None
        self._camera_scan_started = False
        self._last_frame: Optional[QtGui.QImage] = None
        self._last_targets: list[Optional[float]] = [None] * ENCODER_COUNT
        self._last_sent_targets: list[Optional[float]] = [None] * ENCODER_COUNT
        self._feedback_angles: list[Optional[float]] = [None] * ENCODER_COUNT
        self._last_timestamp_ms = 0
        self._send_targets_enabled = False
        self._build()
        self._connect_bridge()
        self.destroyed.connect(lambda _obj=None: self._stop_runner_only())

    def _build(self) -> None:
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)

        toolbar = QtWidgets.QHBoxLayout()
        self.camera_combo = _LazyCameraCombo()
        self.camera_combo.addItem("No camera selected", None)
        self.camera_combo.popup_requested.connect(self.refresh_cameras_once)
        self.refresh_btn = QtWidgets.QPushButton("Refresh Cameras")
        self.hand_count_spin = QtWidgets.QSpinBox()
        self.hand_count_spin.setRange(1, 2)
        self.hand_count_spin.setValue(1)
        self.send_targets_btn = QtWidgets.QPushButton("Send")
        self.stop_send_btn = QtWidgets.QPushButton("Stop Send")
        self.stop_send_btn.setEnabled(False)
        self.send_targets_btn.setToolTip("Start sending visual retargeting joint targets to the hand")
        self.stop_send_btn.setToolTip("Stop sending visual retargeting targets; camera tracking keeps running")
        self.send_targets_btn.setStyleSheet("QPushButton { font-weight: 600; color: #116329; }")
        self.stop_send_btn.setStyleSheet("QPushButton { font-weight: 600; color: #b3261e; }")
        self.start_btn = QtWidgets.QPushButton("Start")
        self.stop_btn = QtWidgets.QPushButton("Stop")
        self.stop_btn.setEnabled(False)

        toolbar.addWidget(QtWidgets.QLabel("Camera:"))
        toolbar.addWidget(self.camera_combo, 1)
        toolbar.addWidget(self.refresh_btn)
        toolbar.addWidget(QtWidgets.QLabel("Hands:"))
        toolbar.addWidget(self.hand_count_spin)
        toolbar.addWidget(self.send_targets_btn)
        toolbar.addWidget(self.stop_send_btn)
        toolbar.addStretch(1)
        toolbar.addWidget(self.start_btn)
        toolbar.addWidget(self.stop_btn)
        root.addLayout(toolbar)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.preview = QtWidgets.QLabel("Camera preview")
        self.preview.setAlignment(QtCore.Qt.AlignCenter)
        self.preview.setMinimumSize(480, 360)
        self.preview.setStyleSheet("background:#111;color:#bbb;border:1px solid #444;")
        self.preview.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        splitter.addWidget(self.preview)

        self.angle_table = QtWidgets.QTableWidget(ENCODER_COUNT, 2)
        self.angle_table.setHorizontalHeaderLabels(["Feedback Angle", "Visual Target"])
        self.angle_table.verticalHeader().setVisible(True)
        self.angle_table.setVerticalHeaderLabels([f"J{i:02d}" for i in range(ENCODER_COUNT)])
        self.angle_table.setAlternatingRowColors(True)
        self.angle_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.angle_table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.angle_table.horizontalHeader().setStretchLastSection(True)
        for idx in range(ENCODER_COUNT):
            self.angle_table.setItem(idx, 0, self._make_item("-"))
            self.angle_table.setItem(idx, 1, self._make_item("-"))
        splitter.addWidget(self.angle_table)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 1)
        root.addWidget(splitter, 1)

        status_row = QtWidgets.QHBoxLayout()
        self.status_label = QtWidgets.QLabel("Stopped")
        self.fps_label = QtWidgets.QLabel("FPS: -")
        self.frame_label = QtWidgets.QLabel("Frame: -")
        status_row.addWidget(self.status_label, 1)
        status_row.addWidget(self.fps_label)
        status_row.addWidget(self.frame_label)
        root.addLayout(status_row)

        self.refresh_btn.clicked.connect(lambda _checked=False: self.refresh_cameras(force=True))
        self.start_btn.clicked.connect(lambda _checked=False: self.start_posture_mocap())
        self.stop_btn.clicked.connect(lambda _checked=False: self.stop_posture_mocap())
        self.send_targets_btn.clicked.connect(lambda _checked=False: self.start_sending_targets())
        self.stop_send_btn.clicked.connect(lambda _checked=False: self.stop_sending_targets())

    def _connect_bridge(self) -> None:
        self.bridge.frame_ready.connect(self._on_frame_ready)
        self.bridge.joints_ready.connect(self._on_joints_ready)
        self.bridge.cameras_ready.connect(self._on_cameras_ready)
        self.bridge.status_changed.connect(self._on_status_changed)
        self.bridge.stopped.connect(self._on_runner_stopped)

    def _make_item(self, text: str) -> QtWidgets.QTableWidgetItem:
        item = QtWidgets.QTableWidgetItem(text)
        item.setTextAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        return item

    def refresh_cameras_once(self) -> None:
        if not self._camera_scan_started:
            self.refresh_cameras(force=True)

    def refresh_cameras(self, force: bool = False) -> None:
        if self._runner is not None and self._runner.is_running():
            return
        if self._scanner is not None and self._scanner.is_running():
            return
        if self._camera_scan_started and not force:
            return
        self._camera_scan_started = True
        self.refresh_btn.setEnabled(False)
        self.camera_combo.setEnabled(False)
        self._on_status_changed("Scanning cameras...")
        self._scanner = _CameraScanRunner(self.bridge, max_index=8)
        self._scanner.start()

    def start_posture_mocap(self) -> None:
        if self._runner is not None and self._runner.is_running():
            return
        camera_id = self.camera_combo.currentData()
        if camera_id is None:
            self._on_status_changed("Select a camera before starting")
            return
        self._runner = _PostureMocapRunner(
            self.bridge,
            camera_id=int(camera_id),
            num_hands=self.hand_count_spin.value(),
        )
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.refresh_btn.setEnabled(False)
        self.camera_combo.setEnabled(False)
        self.hand_count_spin.setEnabled(False)
        self._on_status_changed("Starting Posture mocap...")
        self._runner.start()

    def stop_posture_mocap(self) -> None:
        if self._runner is not None:
            self._runner.stop()
            self.stop_btn.setEnabled(False)
            self._on_status_changed("Stopping Posture mocap...")

    def _set_send_targets_enabled(self, enabled: bool) -> None:
        self._send_targets_enabled = bool(enabled)
        self.send_targets_btn.setEnabled(not self._send_targets_enabled)
        self.stop_send_btn.setEnabled(self._send_targets_enabled)

    def start_sending_targets(self) -> None:
        if self._runner is None or not self._runner.is_running():
            self._on_status_changed("Teleop send blocked: start camera tracking first")
            return
        if not self.context.is_comm_connected():
            self._on_status_changed("Teleop send blocked: connect the P4 serial port first")
            return
        self._set_send_targets_enabled(True)
        self.context.emit_action("Visual Retargeting send ON")
        self._on_status_changed("Visual Retargeting send enabled; waiting for a tracked hand")

    def stop_sending_targets(self) -> None:
        self._set_send_targets_enabled(False)
        self.context.emit_action("Visual Retargeting send OFF")
        self._on_status_changed("Visual Retargeting send stopped")

    def _stop_runner_only(self) -> None:
        if self._runner is not None:
            self._runner.stop()

    def update_state(self, state) -> None:
        angles = list(getattr(state, "angles", []) or []) if state is not None else []
        for idx in range(ENCODER_COUNT):
            if idx < len(angles):
                try:
                    self._feedback_angles[idx] = float(angles[idx])
                except (TypeError, ValueError):
                    self._feedback_angles[idx] = None
            else:
                self._feedback_angles[idx] = None
            text = f"{self._feedback_angles[idx]:.2f}" if self._feedback_angles[idx] is not None else "-"
            self.angle_table.item(idx, 0).setText(text)

    def close(self) -> None:
        self.stop_posture_mocap()
        super().close()

    def _on_cameras_ready(self, cameras: list) -> None:
        current = self.camera_combo.currentData()
        self.camera_combo.blockSignals(True)
        self.camera_combo.clear()
        self.camera_combo.addItem("No camera selected", None)
        for camera_id in cameras:
            self.camera_combo.addItem(f"Camera {camera_id}", int(camera_id))
        if current is not None:
            idx = self.camera_combo.findData(current)
            if idx >= 0:
                self.camera_combo.setCurrentIndex(idx)
        self.camera_combo.blockSignals(False)
        self.camera_combo.setEnabled(True)
        self.refresh_btn.setEnabled(True)
        if cameras:
            labels = ", ".join(f"Camera {int(camera_id)}" for camera_id in cameras)
            self._on_status_changed(f"Found {len(cameras)} camera(s): {labels}")
        else:
            self._on_status_changed("No camera found: check USB connection, privacy permission, or camera occupancy")

    def _on_runner_stopped(self) -> None:
        if self._send_targets_enabled:
            self._set_send_targets_enabled(False)
            self.context.emit_action("Visual Retargeting send OFF: camera stopped")
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.refresh_btn.setEnabled(True)
        self.camera_combo.setEnabled(True)
        self.hand_count_spin.setEnabled(True)
        self._runner = None
        self._on_status_changed("Stopped")

    def _on_status_changed(self, text: str) -> None:
        self.status_label.setText(str(text))
        self.context.set_status(str(text))

    def _on_frame_ready(self, image: QtGui.QImage, fps: float) -> None:
        self._last_frame = image
        self.fps_label.setText(f"FPS: {fps:.1f}")
        self._refresh_preview()

    def _selected_joint_mask(self) -> int:
        if callable(self.selected_joint_mask_getter):
            try:
                return int(self.selected_joint_mask_getter()) & ((1 << ENCODER_COUNT) - 1)
            except Exception:
                return 0
        return (1 << ENCODER_COUNT) - 1

    def _targets_for_selected_joints(self, visual_targets: list[float]) -> tuple[Optional[list[float]], list[int]]:
        mask = self._selected_joint_mask()
        if mask == 0:
            return None, []

        targets: list[float] = []
        missing_feedback: list[int] = []
        for idx in range(ENCODER_COUNT):
            if (mask & (1 << idx)) != 0:
                targets.append(float(visual_targets[idx]))
                continue

            hold = self._feedback_angles[idx]
            if hold is None:
                hold = self._last_sent_targets[idx]
            if hold is None:
                missing_feedback.append(idx)
                hold = 0.0
            targets.append(float(hold))

        if missing_feedback:
            return None, missing_feedback
        return targets, []

    def _on_joints_ready(self, targets: list, timestamp_ms: int) -> None:
        # 视觉节点输出必须先经过统一21关节适配器；禁止错误长度静默补零，
        # 也禁止NaN/Inf进入负反馈链路。
        try:
            raw_visual_targets = [float(v) for v in list(targets)]
            adaptation = adapt_joint_targets(
                raw_visual_targets,
                source_space=URDF_DEG,
            )
        except (TypeError, ValueError) as exc:
            self._on_status_changed(f"Visual Retargeting target dropped: {exc}")
            return

        visual_targets = list(adaptation.urdf_targets_deg)
        self._last_targets = visual_targets
        self._last_timestamp_ms = int(timestamp_ms)
        self.frame_label.setText(f"Frame: {timestamp_ms}")
        for idx in range(ENCODER_COUNT):
            text = f"{self._last_targets[idx]:.2f}" if idx < len(self._last_targets) else "-"
            self.angle_table.item(idx, 1).setText(text)
        if self._send_targets_enabled:
            if not self.context.is_comm_connected():
                self._set_send_targets_enabled(False)
                self.context.emit_action("Visual Retargeting send OFF: P4 serial disconnected")
                self._on_status_changed("Teleop send stopped: P4 serial port disconnected")
                return
            selected_targets, missing_feedback = self._targets_for_selected_joints(self._last_targets)
            if selected_targets is None:
                if missing_feedback:
                    missing = ",".join(f"J{idx:02d}" for idx in missing_feedback[:4])
                    self._on_status_changed(f"Teleop send blocked: waiting feedback {missing}")
                else:
                    self._on_status_changed("Teleop send blocked: no joint selected")
                return
            result = self.context.send_joint_targets(
                selected_targets,
                source="Visual Retargeting",
                live=True,
                input_space=adaptation.source_space,
                raw_targets=adaptation.source_targets,
            )
            if result is False or not bool(getattr(result, "sent", True)):
                message = str(getattr(result, "message", "") or "")
                self._on_status_changed(message or "Teleop send blocked by workspace guard")
                return
            self._last_sent_targets = list(selected_targets)

    def _refresh_preview(self) -> None:
        if self._last_frame is None:
            return
        pixmap = QtGui.QPixmap.fromImage(self._last_frame)
        pixmap = pixmap.scaled(
            self.preview.size(),
            QtCore.Qt.KeepAspectRatio,
            QtCore.Qt.SmoothTransformation,
        )
        self.preview.setPixmap(pixmap)

    def resizeEvent(self, event):  # noqa: N802 - Qt override
        super().resizeEvent(event)
        self._refresh_preview()

    def closeEvent(self, event):  # noqa: N802 - Qt override
        self.stop_posture_mocap()
        super().closeEvent(event)


__all__ = ["VisualRetargetingView"]
