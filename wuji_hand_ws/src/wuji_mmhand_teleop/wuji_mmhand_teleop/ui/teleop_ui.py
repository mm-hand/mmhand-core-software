#!/usr/bin/env python3

import os
import signal
import subprocess
import sys
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from wuji_mmhand_teleop.ui.qt_compat import QtCore, QtWidgets
from wuji_mmhand_teleop.ui.urdf_viewer import FastHandCanvas, FastHandGlView
from wuji_mmhand_teleop.urdf_contract import (
    JOINT_COUNT as ENCODER_COUNT,
    MESSAGE_LAYOUT_LABEL,
    MODEL_ID,
    TOPIC_NAME,
    validate_target,
)


CALIBRATION_COMMAND_TOPIC = "/wuji_calibration_command"
CALIBRATION_STATUS_TOPIC = "/wuji_calibration_status"


class RawIkTargetWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.sender_process: Optional[subprocess.Popen] = None
        self.calibration_command_publisher = None
        self.setWindowTitle("Wuji Teleop")
        self.resize(1320, 820)

        root = QtWidgets.QVBoxLayout(self)
        panel = QtWidgets.QGroupBox("Wuji Input / Preview")
        panel_layout = QtWidgets.QVBoxLayout(panel)

        source_row = QtWidgets.QHBoxLayout()
        source_combo = QtWidgets.QComboBox()
        source_combo.addItem(f"ROS2 {TOPIC_NAME}")
        source_row.addWidget(QtWidgets.QLabel("Source:"))
        source_row.addWidget(source_combo, 1)
        panel_layout.addLayout(source_row)

        retarget_row = QtWidgets.QHBoxLayout()
        retarget_combo = QtWidgets.QComboBox()
        retarget_combo.addItem("Full raw skeleton IK")
        retarget_row.addWidget(QtWidgets.QLabel("Retarget:"))
        retarget_row.addWidget(retarget_combo, 1)
        panel_layout.addLayout(retarget_row)

        status_row = QtWidgets.QHBoxLayout()
        self.status_label = QtWidgets.QLabel("Sender stopped")
        self.live_label = QtWidgets.QLabel("STALE")
        self.seq_label = QtWidgets.QLabel("seq=-")
        self.rate_label = QtWidgets.QLabel("recv_hz=-")
        self.age_label = QtWidgets.QLabel("age=-")
        status_row.addWidget(self.status_label, 1)
        for label in (self.live_label, self.seq_label, self.rate_label, self.age_label):
            status_row.addWidget(label)
        panel_layout.addLayout(status_row)

        info = QtWidgets.QFormLayout()
        info.addRow("Listen:", QtWidgets.QLabel(TOPIC_NAME))
        info.addRow("Thread:", QtWidgets.QLabel("ROS2 subscription"))
        self.connection_label = QtWidgets.QLabel("no")
        info.addRow("Publisher Connected:", self.connection_label)
        self.update_label = QtWidgets.QLabel("-")
        info.addRow("Last Update:", self.update_label)
        self.frame_label = QtWidgets.QLabel("0")
        info.addRow("Frames:", self.frame_label)
        info.addRow("Safety:", QtWidgets.QLabel("Raw URDF angles; no lower-controller calibration mapping"))
        info.addRow("Model Contract:", QtWidgets.QLabel(MODEL_ID))
        panel_layout.addLayout(info)

        control_box = QtWidgets.QGroupBox("Wuji Retarget Calibration / Joint Send")
        control_layout = QtWidgets.QVBoxLayout(control_box)
        calibration_row = QtWidgets.QHBoxLayout()
        calibration_actions = (
            ("Record Align", "record:align"),
            ("Record Open AA", "record:open_aa"),
            ("Record Fist Flex", "record:fist_flex"),
            ("Record Thumb->PinkyRoot", "record:thumb_pinky_root"),
            ("Load Calibration", "load"),
        )
        for text, command in calibration_actions:
            button = QtWidgets.QPushButton(text)
            button.clicked.connect(lambda _checked=False, value=command: self.request_calibration(value))
            calibration_row.addWidget(button)
        control_layout.addLayout(calibration_row)
        controls = QtWidgets.QHBoxLayout()
        self.side_combo = QtWidgets.QComboBox()
        self.side_combo.addItems(["left", "right"])
        self.device_sn_edit = QtWidgets.QLineEdit()
        self.device_sn_edit.setPlaceholderText("auto scan")
        self.device_sn_edit.setMaximumWidth(180)
        self.confidence_spin = QtWidgets.QDoubleSpinBox()
        self.confidence_spin.setRange(0.0, 1.0)
        self.confidence_spin.setSingleStep(0.05)
        self.confidence_spin.setValue(0.4)
        controls.addWidget(QtWidgets.QLabel("Side:"))
        controls.addWidget(self.side_combo)
        controls.addWidget(QtWidgets.QLabel("Device SN:"))
        controls.addWidget(self.device_sn_edit)
        controls.addWidget(QtWidgets.QLabel("Confidence:"))
        controls.addWidget(self.confidence_spin)
        self.send_button = QtWidgets.QPushButton("Send")
        self.stop_button = QtWidgets.QPushButton("Stop")
        self.send_button.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_MediaPlay))
        self.stop_button.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_MediaStop))
        self.send_button.clicked.connect(self.start_sender)
        self.stop_button.clicked.connect(self.stop_sender)
        controls.addWidget(self.send_button)
        controls.addWidget(self.stop_button)
        self.received_label = QtWidgets.QLabel("received=0")
        controls.addWidget(self.received_label)
        controls.addStretch(1)
        control_layout.addLayout(controls)
        control_layout.addWidget(QtWidgets.QLabel("Send starts sender.py; Stop terminates it"))
        self.calibration_status_label = QtWidgets.QLabel("Wuji retarget calibration is executed by sender.py")
        control_layout.addWidget(self.calibration_status_label)
        panel_layout.addWidget(control_box)

        tabs = QtWidgets.QTabWidget()
        for name, note in (
            ("Typed Ergonomics", "Not carried by /raw_ik_target"),
            ("Ergonomics Angle Mapping", "Lower-controller mapping removed"),
            ("Raw Skeleton", "Consumed inside sender.py"),
        ):
            page = QtWidgets.QTableWidget(1, 1)
            page.setHorizontalHeaderLabels(["Status"])
            page.setItem(0, 0, QtWidgets.QTableWidgetItem(note))
            page.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
            tabs.addTab(page, name)

        self.table = QtWidgets.QTableWidget(ENCODER_COUNT, 9)
        self.table.setHorizontalHeaderLabels(["Robot Joint", "Target Debug", "Robot Target XYZ", "IK Error mm", "Joint Limit", "q_urdf", "q_target", "Valid", "Send"])
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
        for idx in range(ENCODER_COUNT):
            self._set_item(idx, 0, f"J{idx:02d}")
            self._set_item(idx, 1, TOPIC_NAME)
            for column in (2, 3, 4, 5, 6):
                self._set_item(idx, column, "-")
            self._set_item(idx, 7, "waiting")
            self._set_item(idx, 8, "yes")
        tabs.addTab(self.table, "Raw IK")

        viewer_panel = QtWidgets.QWidget()
        viewer_layout = QtWidgets.QVBoxLayout(viewer_panel)
        viewer_layout.addWidget(QtWidgets.QLabel(f"q_urdf source: {TOPIC_NAME}"))
        self.viewer_status = QtWidgets.QLabel("waiting for raw_ik_target")
        viewer_layout.addWidget(self.viewer_status)
        try:
            self.viewer = FastHandGlView()
            self.viewer_mode = "3D OpenGL"
        except Exception as exc:
            self.viewer = FastHandCanvas()
            self.viewer_mode = f"software fallback: {exc}"
        viewer_layout.addWidget(self.viewer, 1)
        tabs.addTab(viewer_panel, "URDF Viewer")
        panel_layout.addWidget(tabs, 1)
        root.addWidget(panel, 1)

        self.process_timer = QtCore.QTimer(self)
        self.process_timer.timeout.connect(self._poll_sender)
        self.process_timer.start(250)
        self._update_buttons()

    def start_sender(self):
        if self.sender_process is not None and self.sender_process.poll() is None:
            self.status_label.setText("sender already running")
            return
        self.sender_process = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "wuji_mmhand_teleop.nodes.sender",
                "--ros-args",
                "-p",
                f"side:={self.side_combo.currentText()}",
                "-p",
                f"device_sn:={self.device_sn_edit.text().strip()}",
                "-p",
                f"confidence_threshold:={self.confidence_spin.value():.2f}",
                "-p",
                f"output_topic:={TOPIC_NAME}",
            ],
            env=os.environ.copy(),
        )
        self.status_label.setText(f"sender running pid={self.sender_process.pid}")
        self._update_buttons()

    def request_calibration(self, command):
        if self.sender_process is None or self.sender_process.poll() is not None:
            self.calibration_status_label.setText("Start sender before Wuji calibration")
            return
        if self.calibration_command_publisher is None:
            self.calibration_status_label.setText("ROS calibration command publisher is unavailable")
            return
        self.calibration_command_publisher(command)
        self.calibration_status_label.setText(f"Wuji calibration requested: {command}")

    def stop_sender(self):
        process = self.sender_process
        if process is None or process.poll() is not None:
            self.sender_process = None
            self.status_label.setText("sender stopped")
            self._update_buttons()
            return
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3.0)
        self.sender_process = None
        self.status_label.setText("sender stopped")
        self._update_buttons()

    def set_raw_ik_target(self, values: list[float]):
        for idx, value in enumerate(values):
            self._set_item(idx, 5, f"{value:+.3f} deg")
            self._set_item(idx, 7, "yes")
        self.viewer.set_angles(values)
        count = int(self.frame_label.text()) + 1
        self.frame_label.setText(str(count))
        self.received_label.setText(f"received={count}")
        self.live_label.setText("LIVE")
        self.connection_label.setText("yes")
        self.seq_label.setText(f"seq={count}")
        self.update_label.setText(time.strftime("%H:%M:%S"))
        self.viewer_status.setText(f"{TOPIC_NAME}: {len(values)}/{ENCODER_COUNT} valid | {self.viewer_mode}")

    def set_input_error(self, message: str):
        self.viewer_status.setText(message)

    def _poll_sender(self):
        if self.sender_process is not None and self.sender_process.poll() is not None:
            code = self.sender_process.returncode
            self.sender_process = None
            self.status_label.setText(f"sender exited code={code}")
            self._update_buttons()

    def _update_buttons(self):
        running = self.sender_process is not None and self.sender_process.poll() is None
        self.send_button.setEnabled(not running)
        self.stop_button.setEnabled(running)
        self.side_combo.setEnabled(not running)
        self.device_sn_edit.setEnabled(not running)
        self.confidence_spin.setEnabled(not running)

    def _set_item(self, row: int, column: int, text: str):
        item = self.table.item(row, column)
        if item is None:
            item = QtWidgets.QTableWidgetItem()
            item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
            self.table.setItem(row, column, item)
        item.setText(str(text))

    def closeEvent(self, event):
        self.stop_sender()
        super().closeEvent(event)


class RawIkTargetSubscriber(Node):
    def __init__(self, window: RawIkTargetWindow):
        super().__init__("raw_ik_target_ui")
        self.window = window
        self.subscription = self.create_subscription(
            Float32MultiArray,
            TOPIC_NAME,
            self._on_target,
            10,
        )
        self.calibration_command_publisher = self.create_publisher(
            String, CALIBRATION_COMMAND_TOPIC, 10
        )
        self.calibration_status_subscription = self.create_subscription(
            String, CALIBRATION_STATUS_TOPIC, self._on_calibration_status, 10
        )
        self.window.calibration_command_publisher = self.publish_calibration_command

    def publish_calibration_command(self, command):
        message = String()
        message.data = str(command)
        self.calibration_command_publisher.publish(message)

    def _on_calibration_status(self, message):
        self.window.calibration_status_label.setText(str(message.data))

    def _on_target(self, message: Float32MultiArray):
        values = [float(value) for value in message.data]
        label = message.layout.dim[0].label if message.layout.dim else ""
        error = validate_target(values, label)
        if error is not None:
            self.window.set_input_error(f"rejected {TOPIC_NAME}: {error}")
            return
        self.window.set_raw_ik_target(values)


def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    window = RawIkTargetWindow()
    node = RawIkTargetSubscriber(window)
    ros_timer = QtCore.QTimer()
    def spin_ros():
        if rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)

    ros_timer.timeout.connect(spin_ros)
    ros_timer.start(10)
    signal.signal(signal.SIGINT, lambda _sig, _frame: app.quit())
    window.show()
    try:
        return int(app.exec())
    finally:
        ros_timer.stop()
        window.stop_sender()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
