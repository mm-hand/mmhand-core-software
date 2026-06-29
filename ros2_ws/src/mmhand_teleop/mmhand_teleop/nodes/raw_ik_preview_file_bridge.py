from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64MultiArray

from mmhand_teleop.stream_writer import write_preview_stream


TOPIC_NAME = "/mmhand/teleop/raw_ik_preview_deg"


class RawIkPreviewFileBridge(Node):
    def __init__(self) -> None:
        super().__init__("raw_ik_preview_file_bridge")
        self.declare_parameter("output_path", "")
        output_path = str(self.get_parameter("output_path").value).strip()
        if not output_path:
            raise ValueError(
                "parameter 'output_path' is required and must point to "
                "desktop/config/raw_ik_q_target.json"
            )
        self._output_path = Path(output_path).expanduser()
        self._subscription = self.create_subscription(
            Float64MultiArray,
            TOPIC_NAME,
            self._on_preview,
            QoSProfile(depth=1),
        )
        self.get_logger().info(
            f"bridging {TOPIC_NAME} to {self._output_path}"
        )

    def _on_preview(self, message: Float64MultiArray) -> None:
        try:
            write_preview_stream(self._output_path, message.data)
        except (OSError, TypeError, ValueError) as exc:
            self.get_logger().error(f"preview message rejected: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = RawIkPreviewFileBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ValueError) as exc:
        if node is None:
            rclpy.logging.get_logger("raw_ik_preview_file_bridge").error(str(exc))
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
