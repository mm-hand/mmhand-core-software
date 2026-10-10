"""Current calibration imports retain source-branch device and model isolation."""
from types import SimpleNamespace
import pytest
from desktop.core.calibration_workflow import CalibrationWorkflowService
from desktop.core.calibration_snapshot import CalibrationSnapshotError

@pytest.fixture
def service(tmp_path):
    identity = dict(device_id='0000123456789ABC', urdf_repo_path='model/hand.urdf',
                    product_fingerprint=dict(schema_version=2, manual_scaled_mapping_version=4,
                                             control_model_crc=123, hardware_map_crc=456))
    controller = SimpleNamespace(calibration_data_identity=identity,
                                 calibration_snapshot_path=tmp_path / 'device.mmcal.json')
    workflow = CalibrationWorkflowService(controller)
    yield workflow, controller
    workflow._executor.shutdown(wait=True)

@pytest.mark.parametrize('key,value', [('device_id', '0000123456789ABD'), ('urdf_repo_path', 'other.urdf')])
def test_foreign_device_or_model_is_rejected_before_restore(service, key, value):
    workflow, controller = service
    payload = dict(controller.calibration_data_identity, **{key: value})
    with pytest.raises(CalibrationSnapshotError, match='identity mismatch'):
        workflow._validate_device_identity(payload)

@pytest.mark.parametrize('field', ['schema_version', 'manual_scaled_mapping_version', 'control_model_crc', 'hardware_map_crc'])
def test_missing_or_foreign_fingerprint_rejected(service, field):
    workflow, controller = service
    payload = dict(controller.calibration_data_identity)
    payload['product_fingerprint'] = dict(payload['product_fingerprint'])
    payload['product_fingerprint'].pop(field)
    with pytest.raises(CalibrationSnapshotError, match='contract mismatch'):
        workflow._validate_device_identity(payload)

def test_store_follows_current_device_and_valid_identity_passes(service, tmp_path):
    workflow, controller = service
    workflow._validate_device_identity(dict(controller.calibration_data_identity))
    assert workflow.snapshot_path == controller.calibration_snapshot_path
    controller.calibration_snapshot_path = tmp_path / 'second-device.mmcal.json'
    assert workflow.snapshot_path == controller.calibration_snapshot_path
