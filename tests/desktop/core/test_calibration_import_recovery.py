"""导入成功必须包含多圈恢复确认，不能只以标定命令 ACK 为准。"""

import threading
from unittest.mock import Mock

import pytest

from desktop.core.calibration_workflow import CalibrationWorkflowService
from desktop.core.core_logic import HandController


@pytest.mark.parametrize('recovered', [True, False])
def test_import_waits_after_activation_and_reports_recovery_result(tmp_path, monkeypatch, recovered):
    controller = HandController()
    service = CalibrationWorkflowService(controller, tmp_path / 'import.mmcal.json')
    monkeypatch.setattr('desktop.core.calibration_workflow.validate_calibration_snapshot_for_load', lambda _: None)
    calls, events = [], []
    payload = {'active_motor_mask': '0x80', 'active_joint_mask': 1}
    controller.load_calibration_snapshot = lambda _: (True, 'loaded 1 joints')
    controller._multiturn_store_identity = {'old': True}
    controller._multiturn_store_payload = {'old': True}
    service._store.activate = lambda _: calls.append('activate')

    def recover(mask, *, cancelled):
        assert mask == 0x80 and not cancelled()
        assert controller._multiturn_store_identity is None
        assert controller._multiturn_store_payload is None
        calls.append('recover')
        assert not events
        return recovered, 'multi-turn recovery confirmed' if recovered else 'failed after 3 requests: M07'

    controller.wait_for_multiturn_recovery = recover
    service.register_callback(events.append)
    assert service.begin_import(payload)
    service._executor.shutdown(wait=True)
    assert calls == ['activate', 'recover']
    assert len(events) == 1 and events[0].ok == recovered
    assert events[0].details['activated']
    assert ('recovery confirmed' if recovered else 'M07') in events[0].message
    if not recovered:
        assert 'Calibration installed, but multi-turn recovery failed' in events[0].message


def test_stop_during_import_recovery_cancels_without_success(tmp_path, monkeypatch):
    controller = HandController()
    service = CalibrationWorkflowService(controller, tmp_path / 'import.mmcal.json')
    monkeypatch.setattr('desktop.core.calibration_workflow.validate_calibration_snapshot_for_load', lambda _: None)
    controller.load_calibration_snapshot = lambda _: (True, 'loaded 1 joints')
    service._store.activate = Mock()
    entered, release = threading.Event(), threading.Event()
    events = []

    def recover(mask, *, cancelled):
        entered.set()
        assert release.wait(2)
        assert cancelled()
        return False, 'multi-turn recovery cancelled'

    controller.wait_for_multiturn_recovery = recover
    service.register_callback(events.append)
    try:
        assert service.begin_import({'active_motor_mask': 0x80})
        assert entered.wait(1)
        service.cancel()
    finally:
        release.set()
        service._executor.shutdown(wait=True)
    assert len(events) == 1 and not events[0].ok
    assert 'after local activation' in events[0].message
