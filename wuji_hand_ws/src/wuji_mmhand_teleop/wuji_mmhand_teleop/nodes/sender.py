#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout, String

from wuji_mmhand_teleop.retarget.manus_raw_skeleton import FullRawSkeletonRetargeter
from wuji_mmhand_teleop.retarget.wuji_calibration import WujiCalibrationManager
from wuji_mmhand_teleop.sources.wuji_sdk_source import WujiReceiver
from wuji_mmhand_teleop.urdf_contract import (
    JOINT_COUNT,
    MESSAGE_LAYOUT_LABEL,
    MODEL_ID,
    TOPIC_NAME,
    validate_bundled_urdf,
)


class WujiRawIkTargetPublisher(Node):
    def __init__(self) -> None:
        super().__init__("wuji_raw_ik_target_sender")
        validate_bundled_urdf()
        self.declare_parameter("device_sn", "")
        self.declare_parameter("side", "left")
        self.declare_parameter("confidence_threshold", 0.4)
        self.declare_parameter("output_topic", TOPIC_NAME)
        self.declare_parameter("timer_period_s", 0.01)

        device_sn = self.get_parameter("device_sn").value
        self.side = str(self.get_parameter("side").value).lower()
        threshold = float(self.get_parameter("confidence_threshold").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        timer_period = float(self.get_parameter("timer_period_s").value)
        if self.side not in {"left", "right"}:
            raise ValueError("side must be 'left' or 'right'")

        self.publisher_ = self.create_publisher(Float32MultiArray, self.output_topic, 10)
        self.status_publisher = self.create_publisher(String, "wuji_calibration_status", 10)
        self.command_subscription = self.create_subscription(
            String, "wuji_calibration_command", self._on_calibration_command, 10
        )
        self.receiver = WujiReceiver(device_sn=device_sn, side=self.side, confidence_threshold=threshold)
        self.calibration = WujiCalibrationManager()
        self.retargeter = FullRawSkeletonRetargeter(calibration_manager=self.calibration)
        self.last_processed_seq = None
        self.last_status = ""

        self.receiver.start()
        self.calibration.set_hand_model_path(self.receiver.hand_model_path)
        offsets = ", ".join(
            f"{name}={transform.translation.tolist()}"
            for name, transform in self.receiver._fingertip_contact_transforms.items()
        )
        self.get_logger().info(
            f"Wuji tip semantics={self.receiver.tip_semantics}; "
            f"profile={self.receiver.hand_profile}; model={self.receiver.hand_model_path}; "
            f"offsets_m: {offsets}"
        )
        if self.calibration.requires_align_recalibration:
            self.get_logger().warning(
                "Wuji calibration uses legacy tip semantics; record align again"
            )
        if self.calibration.requires_model_recalibration:
            self.get_logger().warning(
                f"Wuji calibration belongs to another robot model; record all poses for {MODEL_ID}"
            )
        self.timer = self.create_timer(timer_period, self._on_timer)
        self.get_logger().info(
            f"Wuji {self.side} glove -> {self.output_topic}; confidence >= {threshold:.2f}; "
            f"contract={MODEL_ID}; space=urdf_deg"
        )

    def _publish_status(self, status: str, error: bool = False) -> None:
        if status != self.last_status:
            self.last_status = status
            if error:
                self.get_logger().error(status)
            else:
                self.get_logger().info(status)
        message = String()
        message.data = status
        self.status_publisher.publish(message)

    def _on_calibration_command(self, message: String) -> None:
        command = str(message.data).strip()
        try:
            if command == "load":
                loaded = self.calibration.load()
                status = "Wuji retarget calibration loaded" if loaded else "Wuji calibration file not found"
            elif command.startswith("record:"):
                pose = command.split(":", 1)[1]
                count = self.calibration.record_pose(pose, self.receiver, duration_s=1.0)
                status = f"Wuji calibration recorded {pose}: {count} frames"
            else:
                raise ValueError("supported commands: load, record:align/open_aa/fist_flex/thumb_pinky_root")
        except Exception as exc:
            self._publish_status(f"Wuji calibration failed: {exc}", error=True)
        else:
            self._publish_status(status)

    def _on_timer(self) -> None:
        error = self.receiver.last_error()
        if error:
            self._publish_status(f"Wuji input error: {error}", error=True)
            return
        frame = self.receiver.get_latest_frame()
        if frame is None or not self.receiver.is_alive():
            self._publish_status(f"Waiting for live {self.side} Wuji skeleton")
            return
        if frame.side != self.side or frame.seq == self.last_processed_seq:
            return
        try:
            preview = self.retargeter.preview(frame)
        except Exception as exc:
            self._publish_status(f"Wuji raw IK failed: {exc}", error=True)
            return

        target = [0.0] * JOINT_COUNT
        valid_count = 0
        for item in preview:
            if item.valid and 0 <= item.joint_index < JOINT_COUNT:
                target[item.joint_index] = float(item.robot_q_urdf_preview)
                valid_count += 1
        if valid_count != JOINT_COUNT:
            self._publish_status(f"Wuji IK rejected incomplete solution: {valid_count}/{JOINT_COUNT}", error=True)
            return
        message = Float32MultiArray()
        message.data = target
        message.layout = MultiArrayLayout(
            dim=[MultiArrayDimension(label=MESSAGE_LAYOUT_LABEL, size=JOINT_COUNT, stride=JOINT_COUNT)],
            data_offset=0,
        )
        self.publisher_.publish(message)
        self.last_processed_seq = frame.seq
        self._publish_status(f"Wuji raw_ik_target valid {valid_count}/{JOINT_COUNT}")


    def destroy_node(self):
        self.receiver.stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = WujiRawIkTargetPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        rclpy.logging.get_logger("wuji_raw_ik_target_sender").error(str(exc))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
