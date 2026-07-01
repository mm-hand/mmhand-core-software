import math

import pytest

from desktop.core.protocol import ENCODER_COUNT
from desktop.ros2.joint_target_publisher import Ros2JointTargetPublisher


class FakeFloat32MultiArray:
    def __init__(self) -> None:
        self.data = []
        self.layout = None


class FakeMultiArrayDimension:
    def __init__(self) -> None:
        self.label = ""
        self.size = 0
        self.stride = 0


class FakeMultiArrayLayout:
    def __init__(self) -> None:
        self.dim = []
        self.data_offset = 0


class FakePublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, msg) -> None:
        self.messages.append(msg)

    def get_subscription_count(self) -> int:
        return 1


class FakeNode:
    def __init__(self) -> None:
        self.publisher = FakePublisher()
        self.destroyed = False

    def create_publisher(self, message_type, topic, depth):
        assert message_type is FakeFloat32MultiArray
        assert topic == "/mmhand/joint_targets"
        assert depth == 10
        return self.publisher

    def destroy_node(self) -> None:
        self.destroyed = True


class FakeRclpy:
    def __init__(self) -> None:
        self.initialized = False
        self.shutdown_called = False
        self.node = FakeNode()

    def ok(self) -> bool:
        return self.initialized and not self.shutdown_called

    def init(self, args=None) -> None:
        assert args is None
        self.initialized = True

    def create_node(self, name: str):
        assert name == "mmhand_manus_joint_target_publisher"
        return self.node

    def shutdown(self) -> None:
        self.shutdown_called = True


def make_publisher():
    rclpy = FakeRclpy()
    publisher = Ros2JointTargetPublisher(
        rclpy_module=rclpy,
        message_types=(FakeFloat32MultiArray, FakeMultiArrayDimension, FakeMultiArrayLayout),
    )
    return publisher, rclpy


def test_publisher_emits_21_degree_targets_with_layout() -> None:
    publisher, rclpy = make_publisher()
    targets = [float(index) for index in range(ENCODER_COUNT)]

    publisher.publish_values(targets)

    assert publisher.started is True
    assert publisher.subscription_count() == 1
    assert len(rclpy.node.publisher.messages) == 1
    msg = rclpy.node.publisher.messages[0]
    assert msg.data == targets
    assert msg.layout.data_offset == 0
    assert len(msg.layout.dim) == 1
    assert msg.layout.dim[0].label == "mmhand_joints_deg"
    assert msg.layout.dim[0].size == ENCODER_COUNT
    assert msg.layout.dim[0].stride == ENCODER_COUNT


def test_publisher_rejects_invalid_targets_before_starting_ros() -> None:
    publisher, rclpy = make_publisher()

    with pytest.raises(ValueError, match="expected 21"):
        publisher.publish_values([0.0] * (ENCODER_COUNT - 1))
    with pytest.raises(ValueError, match="finite"):
        publisher.publish_values([math.inf] * ENCODER_COUNT)

    assert publisher.started is False
    assert rclpy.initialized is False


def test_publisher_cleans_up_owned_rclpy_context() -> None:
    publisher, rclpy = make_publisher()
    publisher.start()

    publisher.stop()

    assert rclpy.node.destroyed is True
    assert rclpy.shutdown_called is True
    assert publisher.started is False
