import threading

import pytest

from desktop.core.angle_mapping import get_spec, urdf_manual_scaled_mapping_metadata
from desktop.core.data_models import HandModel, resolve_joint_workspace_range
from desktop.core.hand_backend import HandBackend
from desktop.core.protocol import (
    ENCODER_COUNT,
    manual_encoder_reference_entry_crc,
)


def _ready_state(joint_idx=0, current_min=None):
    version, urdf_crc = urdf_manual_scaled_mapping_metadata()
    spec = get_spec(joint_idx)
    state = HandModel()
    state.manual_reference_committed = True
    state.manual_reference_valid_bitmap = 1 << joint_idx
    state.manual_reference_crc = 0x12345678
    state.manual_reference_min_raw[joint_idx] = 1000
    state.manual_reference_max_raw[joint_idx] = 5000
    state.auto_calibration_valid_bitmap = 1 << joint_idx
    state.active_joint_mask = 1 << joint_idx
    # current_min 参数仅保留给既有测试调用；V4 下它不再改变工作区。
    _ = current_min
    state.joint_calib_results[joint_idx] = {
        "success": True,
        "mappingVersion": version,
        "urdfConfigCrc": urdf_crc,
        "manualReferenceCrc": manual_encoder_reference_entry_crc(
            joint_idx,
            state.manual_reference_min_raw[joint_idx],
            state.manual_reference_max_raw[joint_idx],
        ),
        "currentMinDeg": spec.urdf_range_deg[0],
        "currentMaxDeg": spec.urdf_range_deg[1],
        "angleMin": spec.urdf_range_deg[0],
        "angleMax": spec.urdf_range_deg[1],
        "hasManualScaledV4Fields": True,
    }
    return state


def test_workspace_uses_static_urdf_min_and_max():
    state = _ready_state()
    version, urdf_crc = urdf_manual_scaled_mapping_metadata()
    spec = get_spec(0)

    workspace = resolve_joint_workspace_range(
        state,
        0,
        spec.urdf_range_deg[0],
        spec.urdf_range_deg[1],
        version,
        urdf_crc,
    )

    assert workspace.ready
    assert workspace.current_min_deg == pytest.approx(spec.urdf_range_deg[0])
    assert workspace.current_max_deg == pytest.approx(spec.urdf_range_deg[1])
    assert workspace.clamp(-1000.0) == pytest.approx(spec.urdf_range_deg[0])
    assert workspace.clamp(1000.0) == pytest.approx(spec.urdf_range_deg[1])


def test_workspace_rejects_dynamic_min_from_removed_mapping():
    state = _ready_state()
    state.joint_calib_results[0]["currentMinDeg"] = 52.0
    version, urdf_crc = urdf_manual_scaled_mapping_metadata()
    spec = get_spec(0)

    workspace = resolve_joint_workspace_range(
        state,
        0,
        spec.urdf_range_deg[0],
        spec.urdf_range_deg[1],
        version,
        urdf_crc,
    )

    assert not workspace.ready
    assert workspace.reason == "V4 range must equal static URDF Min/Max"


def test_workspace_rejects_manual_crc_mismatch():
    state = _ready_state()
    state.joint_calib_results[0]["manualReferenceCrc"] ^= 1
    version, urdf_crc = urdf_manual_scaled_mapping_metadata()
    spec = get_spec(0)

    workspace = resolve_joint_workspace_range(
        state,
        0,
        spec.urdf_range_deg[0],
        spec.urdf_range_deg[1],
        version,
        urdf_crc,
    )

    assert not workspace.ready
    assert workspace.reason == "Manual CRC mismatch"


class _Controller:
    def __init__(self, state):
        self.current_state = state
        self.state_lock = threading.Lock()
        self.update_callbacks = []
        self.sent = []
        self.run_requests = 0
        self.stop_requests = 0
        self.pause_requests = 0
        self.queue_result = True

    def register_update_callback(self, callback):
        self.update_callbacks.append(callback)

    def set_target_angles(self, targets):
        self.sent.append(("normal", list(targets)))
        return True

    def set_target_angles_live(self, targets, *, joint_mask=None):
        self.sent.append(("live", list(targets), joint_mask))
        return True

    def send_target_angles_and_run_feedback(self, targets, *, live=False, joint_mask=None):
        self.sent.append(("live" if live else "normal", list(targets), joint_mask))
        self.run_requests += 1
        return self.queue_result

    def request_run_feedback(self):
        self.run_requests += 1
        return True

    def stop(self):
        self.stop_requests += 1

    def pause(self):
        self.pause_requests += 1

    def set_control_mode(self, _mode):
        pass

    def set_active_scope(self, motor_mask, joint_mask, require_can):
        self.current_state.active_motor_mask = int(motor_mask)
        self.current_state.active_joint_mask = int(joint_mask)
        self.current_state.require_can_feedback = bool(require_can)
        return True


def test_backend_clamps_active_joint_to_static_urdf_workspace():
    state = _ready_state(current_min=12.5)
    controller = _Controller(state)
    backend = HandBackend(controller=controller)
    targets = [-1000.0] * ENCODER_COUNT

    result = backend.send_joint_targets(targets, source="test", live=False)

    assert result.sent
    assert result.clamped == [0]
    assert result.targets[0] == pytest.approx(get_spec(0).urdf_range_deg[0])
    assert controller.sent[0][1][0] == pytest.approx(get_spec(0).urdf_range_deg[0])
    assert controller.run_requests == 1


def test_backend_blocks_active_joint_without_valid_auto():
    state = _ready_state()
    state.auto_calibration_valid_bitmap = 0
    controller = _Controller(state)
    backend = HandBackend(controller=controller)

    result = backend.send_joint_targets([0.0] * ENCODER_COUNT, source="test")

    assert not result.sent
    assert result.blocked == [0]
    assert controller.sent == []
    assert controller.run_requests == 0


def test_backend_reports_not_sent_when_atomic_queue_rejects_command():
    state = _ready_state(current_min=12.5)
    controller = _Controller(state)
    controller.queue_result = False
    backend = HandBackend(controller=controller)

    result = backend.send_joint_targets([20.0] * ENCODER_COUNT, source="test")

    assert not result.sent
    assert "not queued" in result.message


def test_backend_forwards_single_joint_command_mask_to_controller():
    state = _ready_state(joint_idx=3, current_min=5.0)
    controller = _Controller(state)
    backend = HandBackend(controller=controller)

    result = backend.send_joint_targets(
        [10.0] * ENCODER_COUNT,
        source="Run J3",
        command_joint_mask=1 << 3,
    )

    assert result.sent
    assert controller.sent[-1][2] == (1 << 3)


def test_backend_masked_command_ignores_uncalibrated_active_joint():
    state = _ready_state(joint_idx=0, current_min=5.0)
    state.active_joint_mask = 0b11
    controller = _Controller(state)
    backend = HandBackend(controller=controller)

    result = backend.send_joint_targets(
        [10.0] * ENCODER_COUNT,
        source="masked sparse command",
        command_joint_mask=1,
    )

    assert result.sent
    assert result.blocked == []
    assert controller.sent[-1][2] == 1


def test_backend_rejects_command_mask_outside_current_active_scope():
    state = _ready_state(joint_idx=0, current_min=5.0)
    controller = _Controller(state)
    backend = HandBackend(controller=controller)

    result = backend.send_joint_targets(
        [10.0] * ENCODER_COUNT,
        source="bad mask",
        command_joint_mask=0b10,
    )

    assert not result.sent
    assert result.blocked == [1]
    assert controller.sent == []


def _arm_ros2_backend(backend):
    backend.is_comm_connected = lambda: True
    backend.is_started = lambda: True
    ok, message = backend.set_ros2_output_armed(True, reason="test")
    assert ok, message


def test_ros2_output_gate_reuses_ros2_authorization():
    state = _ready_state(current_min=12.5)
    backend = HandBackend(controller=_Controller(state))
    backend.is_comm_connected = lambda: True
    backend.is_started = lambda: True

    ok, message = backend.set_ros2_output_enabled(True)
    assert ok, message
    assert backend.ros2_output_enabled()
    assert backend.ros2_output_armed()

    ok, message = backend.set_ros2_output_enabled(False)
    assert ok, message
    assert not backend.ros2_output_enabled()


def test_ros2_output_gate_rejects_prearming_before_ordinary_enable():
    state = _ready_state(current_min=12.5)
    backend = HandBackend(controller=_Controller(state))
    backend.is_comm_connected = lambda: True
    backend.is_started = lambda: False

    ok, message = backend.set_ros2_output_enabled(True)

    assert not ok
    assert "press ENABLE first" in message
    assert not backend.ros2_output_enabled()


def test_ros2_frame_requires_existing_enable_authorization():
    state = _ready_state(current_min=12.5)
    controller = _Controller(state)
    backend = HandBackend(controller=controller)
    backend.is_comm_connected = lambda: True
    backend.is_started = lambda: True
    recorded_commands = []
    backend.register_joint_target_callback(recorded_commands.append)

    blocked = backend.send_ros2_joint_targets(
        [20.0] * ENCODER_COUNT,
        source="ROS2:/raw_ik_target",
    )
    assert not blocked.sent
    assert blocked.message.endswith("not_armed")
    assert controller.sent == []
    _arm_ros2_backend(backend)
    sent = backend.send_ros2_joint_targets(
        [20.0] * ENCODER_COUNT,
        source="ROS2:/raw_ik_target",
    )
    assert sent.sent
    assert controller.run_requests == 1
    assert len(recorded_commands) == 1
    assert recorded_commands[0]["source"] == "ROS2:/raw_ik_target"
    assert recorded_commands[0]["sent"] is True
    assert recorded_commands[0]["command_joint_mask"] == state.active_joint_mask
    assert len(recorded_commands[0]["output_targets"]) == ENCODER_COUNT
    # ROS2 每帧绑定 ENABLE 时的 active scope，不继承单行 UI 掩码，
    # 也不在串口稍后发送时重新解释可能已变化的 scope。
    assert controller.sent[-1][2] == state.active_joint_mask


def test_collision_hold_does_not_block_ros2_retreat_target():
    state = _ready_state(current_min=12.5)
    state.feedback_collision_hold_bitmap = state.active_joint_mask
    controller = _Controller(state)
    backend = HandBackend(controller=controller)
    _arm_ros2_backend(backend)

    forward = backend.send_ros2_joint_targets(
        [20.0] * ENCODER_COUNT,
        source="ROS2:/raw_ik_target",
    )
    retreat = backend.send_ros2_joint_targets(
        [15.0] * ENCODER_COUNT,
        source="ROS2:/raw_ik_target",
    )

    assert forward.sent
    assert retreat.sent
    assert backend.ros2_output_armed()
    assert controller.run_requests == 2
    assert controller.sent[-1][1][0] == pytest.approx(15.0)


def test_estop_revokes_ros2_and_following_frames_cannot_restart_feedback():
    state = _ready_state(current_min=12.5)
    controller = _Controller(state)
    backend = HandBackend(controller=controller)
    _arm_ros2_backend(backend)

    before_stop = backend.send_ros2_joint_targets(
        [20.0] * ENCODER_COUNT,
        source="ROS2:/mmhand/joint_targets",
    )
    assert before_stop.sent

    backend.stop()
    after_stop = backend.send_ros2_joint_targets(
        [20.0] * ENCODER_COUNT,
        source="ROS2:/mmhand/joint_targets",
    )

    assert controller.stop_requests == 1
    assert not backend.ros2_output_armed()
    assert not after_stop.sent
    # E-STOP 前一帧允许发送，E-STOP 后持续到来的帧不能产生第二次 RUN。
    assert controller.run_requests == 1


def test_expired_replay_token_cannot_enqueue_an_old_target():
    state = _ready_state(current_min=12.5)
    controller = _Controller(state)
    backend = HandBackend(controller=controller)
    backend.is_comm_connected = lambda: True
    backend.is_started = lambda: True

    ok, token, message = backend.begin_replay_motion()
    assert ok, message
    assert backend.cancel_replay_motion("test stop")

    result = backend.send_joint_targets(
        [20.0] * ENCODER_COUNT,
        source="Replay:expired-token",
        live=True,
        command_joint_mask=state.active_joint_mask,
        motion_token=token,
    )

    assert not result.sent
    assert "ownership expired" in result.message
    assert controller.sent == []
    assert controller.run_requests == 0


def test_pause_and_scope_change_both_revoke_ros2_authorization():
    state = _ready_state(current_min=12.5)
    controller = _Controller(state)
    backend = HandBackend(controller=controller)
    _arm_ros2_backend(backend)

    backend.pause()
    assert controller.pause_requests == 1
    assert not backend.ros2_output_armed()

    _arm_ros2_backend(backend)
    assert backend.set_active_scope(0x02, 0x02, True)
    assert not backend.ros2_output_armed()
