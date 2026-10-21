"""Unknown identity, device switching and upgrade backup cannot reuse anchors."""

from desktop.core import core_logic
from desktop.core.data_models import HandModel
from desktop.core.protocol import HOST_PROTOCOL_VERSION


def test_anchor_paths_use_one_active_bundle_without_legacy_files(tmp_path, monkeypatch):
    bundle = tmp_path / "calibration_bundle.mmcal.json"
    bundle.write_text('{"legacy":true}', encoding="utf-8")
    monkeypatch.setattr(core_logic, "CALIB_ACTIVE_SNAPSHOT_PATH", bundle)
    monkeypatch.setattr(core_logic, "_active_urdf_repo_path", lambda: "model/hand.urdf")
    controller = core_logic.HandController()
    state = HandModel()
    state.has_product_fingerprint = True
    state.product_fingerprint = {"protocol_version": HOST_PROTOCOL_VERSION}
    assert controller._multiturn_identity(state) is None
    state.product_fingerprint["device_id"] = "0000123456789ABC"
    identity = controller._multiturn_identity(state)
    assert identity["device_id"] == "0000123456789ABC"
    assert controller._multiturn_store.path == bundle
    state.product_fingerprint["device_id"] = "0000123456789ABD"
    controller._multiturn_identity(state)
    assert controller._multiturn_store.path == bundle
    assert bundle.read_text(encoding="utf-8") == '{"legacy":true}'
    assert not bundle.with_name(bundle.name + ".before-device-isolation.bak").exists()


def test_bad_device_id_or_protocol_cannot_choose_storage(monkeypatch):
    controller = core_logic.HandController()
    state = HandModel()
    state.has_product_fingerprint = True
    for device_id in ("", "0000000000000000", "../other/device!", "FFFFFFFFFFFFFFFZ"):
        state.product_fingerprint = {"protocol_version": HOST_PROTOCOL_VERSION, "device_id": device_id}
        assert controller._multiturn_identity(state) is None
    state.product_fingerprint = {"protocol_version": HOST_PROTOCOL_VERSION + 1, "device_id": "0000123456789ABC"}
    assert controller._multiturn_identity(state) is None
