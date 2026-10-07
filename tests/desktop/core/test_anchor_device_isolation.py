"""Unknown identity, device switching and upgrade backup cannot reuse anchors."""

from desktop.core import core_logic
from desktop.core.data_models import HandModel
from desktop.core.protocol import HOST_PROTOCOL_VERSION


def test_anchor_paths_follow_device_and_model_and_preserve_legacy(tmp_path, monkeypatch):
    anchor = tmp_path / "multiturn_runtime.json"
    bundle = tmp_path / "calibration_bundle.mmcal.json"
    anchor.write_text('{"legacy":true}', encoding="utf-8")
    bundle.write_text('{"legacy":true}', encoding="utf-8")
    monkeypatch.setattr(core_logic, "MULTITURN_RUNTIME_PATH", anchor)
    monkeypatch.setattr(core_logic, "CALIB_LOAD_SNAPSHOT_PATH", bundle)
    monkeypatch.setattr(core_logic, "_active_urdf_repo_path", lambda: "model/hand.urdf")
    controller = core_logic.HandController()
    state = HandModel()
    state.has_product_fingerprint = True
    state.product_fingerprint = {"protocol_version": HOST_PROTOCOL_VERSION}
    assert controller._multiturn_identity(state) is None
    state.product_fingerprint["device_id"] = "0000123456789ABC"
    identity = controller._multiturn_identity(state)
    path_a = controller._multiturn_store.path
    assert identity["device_id"] in path_a.parts
    state.product_fingerprint["device_id"] = "0000123456789ABD"
    controller._calibration_snapshot_anchor_valid_mask = 1
    controller._multiturn_identity(state)
    assert controller._multiturn_store.path != path_a
    assert controller._calibration_snapshot_anchor_valid_mask == 0
    for path in (anchor, bundle):
        assert path.read_bytes() == path.with_name(path.name + ".before-device-isolation.bak").read_bytes()


def test_bad_device_id_or_protocol_cannot_choose_storage(monkeypatch):
    controller = core_logic.HandController()
    state = HandModel()
    state.has_product_fingerprint = True
    for device_id in ("", "0000000000000000", "../other/device!", "FFFFFFFFFFFFFFFZ"):
        state.product_fingerprint = {"protocol_version": HOST_PROTOCOL_VERSION, "device_id": device_id}
        assert controller._multiturn_identity(state) is None
    state.product_fingerprint = {"protocol_version": HOST_PROTOCOL_VERSION + 1, "device_id": "0000123456789ABC"}
    assert controller._multiturn_identity(state) is None
