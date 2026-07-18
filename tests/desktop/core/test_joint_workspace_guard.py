import threading

import pytest

from desktop.core.angle_mapping import get_spec, urdf_manual_scaled_mapping_metadata
from desktop.core.data_models import HandModel, resolve_joint_workspace_range
from desktop.core.hand_backend import HandBackend
from desktop.core.protocol import (
    ENCODER_COUNT,
    manual_encoder_reference_entry_crc,
)


def _ready_state(joint_idx=0, current_min=12.5):
    version, urdf_crc = urdf_manual_scaled_mapping_metadata()
    spec = get_spec(joint_idx)
    state = HandModel()
    state.manual_reference_nvs_valid = True
    state.manual_reference_valid_bitmap = 1 << joint_idx
    state.manual_reference_crc = 0x12345678
    state.manual_reference_min_raw[joint_idx] = 1000
    state.manual_reference_max_raw[joint_idx] = 5000
    state.auto_calibration_valid_bitmap = 1 << joint_idx
    state.active_joint_mask = 1 << joint_idx
    state.joint_calib_results[joint_idx] = {
        "success": True,
        "mappingVersion": version,
        "urdfConfigCrc": urdf_crc,
        "manualReferenceCrc": manual_encoder_reference_entry_crc(
            joint_idx,
            state.manual_reference_min_raw[joint_idx],
            state.manual_reference_max_raw[joint_idx],
        ),
        "currentMinDeg": current_min,
        "currentMaxDeg": spec.urdf_range_deg[1],
        "angleMin": current_min,
        "angleMax": spec.urdf_range_deg[1],
        "hasManualScaledV3Fields": True,
    }
    return state


def test_workspace_uses_dynamic_min_and_static_urdf_max():
    state = _ready_state(current_min=12.5)
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
    assert workspace.current_min_deg == pytest.approx(12.5)
    assert workspace.current_max_deg == pytest.approx(spec.urdf_range_deg[1])
    assert workspace.clamp(-1000.0) == pytest.approx(12.5)
    assert workspace.clamp(1000.0) == pytest.approx(spec.urdf_range_deg[1])
    assert workspace.open_pose_target() == pytest.approx(17.5)


def test_workspace_open_pose_target_clamps_at_urdf_max():
    state = _ready_state(current_min=52.0)
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
    assert workspace.open_pose_target() == pytest.approx(spec.urdf_range_deg[1])


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

    def register_update_callback(self, callback):
        self.update_callbacks.append(callback)

    def set_target_angles(self, targets):
        self.sent.append(("normal", list(targets)))

    def set_target_angles_live(self, targets):
        self.sent.append(("live", list(targets)))

    def request_run_feedback(self):
        self.run_requests += 1
        return True


class _TendonGuard:
    def sync_to_controller(self, backend, force=False):
        return None

    def apply(self, state, targets, source=""):
        return list(targets), [], ""


def test_backend_clamps_active_joint_to_dynamic_workspace():
    state = _ready_state(current_min=12.5)
    controller = _Controller(state)
    backend = HandBackend(controller=controller, tendon_guard=_TendonGuard())
    targets = [0.0] * ENCODER_COUNT

    result = backend.send_joint_targets(targets, source="test", live=False)

    assert result.sent
    assert result.clamped == [0]
    assert result.targets[0] == pytest.approx(12.5)
    assert controller.sent[0][1][0] == pytest.approx(12.5)
    assert controller.run_requests == 1


def test_backend_blocks_active_joint_without_valid_auto():
    state = _ready_state()
    state.auto_calibration_valid_bitmap = 0
    controller = _Controller(state)
    backend = HandBackend(controller=controller, tendon_guard=_TendonGuard())

    result = backend.send_joint_targets([0.0] * ENCODER_COUNT, source="test")

    assert not result.sent
    assert result.blocked == [0]
    assert controller.sent == []
    assert controller.run_requests == 0
