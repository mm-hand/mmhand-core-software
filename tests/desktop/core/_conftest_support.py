"""跨测试模块共享的 HandBackend 夹具辅助。

职责：构造不接硬件的 HandBackend，并提供等待后台操作收敛的助手。
线程：测试主线程。
输入：pytest 的 monkeypatch / tmp_path、操作结果。
输出：已就绪的后端实例和操作终态。
状态所有权：只拥有每用例的临时后端。
禁止依赖：禁止连接真实串口或写真实标定文件。
"""
import time
from unittest.mock import Mock

import pytest

from _clock_helpers import stamp
from desktop.core.hand_backend import HandBackend
from desktop.core.protocol import SYSTEM_STATE_READY


def make_hand(monkeypatch, tmp_path) -> HandBackend:
    """构造一个系统状态已就绪、但未接硬件的后端。"""
    hand = HandBackend()
    from desktop.core.generated_control_parameters import DEFAULT_OUTPUT_MARGINS
    hand._controller._applied_output_margins = DEFAULT_OUTPUT_MARGINS
    from desktop.core.calibration_workflow import CalibrationWorkflowService
    hand._calibration_workflow._executor.shutdown(wait=True)
    hand._calibration_workflow = CalibrationWorkflowService(hand._controller, tmp_path / "test.mmcal.json")
    hand._calibration_workflow.register_callback(hand._application_calibration_event)
    monkeypatch.setattr(hand, 'is_comm_connected', lambda: True)
    monkeypatch.setattr(hand, 'stop', Mock())
    state = hand._controller.current_state
    state.has_system_status_data = True
    state.system_status_last_rx_ts = time.time()
    stamp(state, 'system_status_last_rx_ts')
    state.system_state = SYSTEM_STATE_READY
    state.protocol_compatible = True
    state.has_encoder_raw_data = True
    state.encoder_raw_last_rx_ts = time.time()
    stamp(state, 'encoder_raw_last_rx_ts')
    state.p4_time_ms = state.encoder_raw_p4_time_ms = 1000
    state.encoder_online_bitmap = (1 << 21) - 1
    state.encoders[0].raw_error = state.encoders[0].error = False
    state.encoders[0].raw_sensor = 12000
    return hand


def wait_operation(hand, result, timeout_s=10):
    """Wait for an operation to settle.

    The budget must exceed every internal wait the operation performs: ENABLE
    alone allows up to 5 s for the P4 confirmation, so a 2 s budget expired
    whenever the suite ran under load.
    """
    assert result.accepted, result.message
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        item = next(row for row in hand._application_operations if row['operation_id'] == result.operation_id)
        if item['status'] in ('SUCCESS', 'FAILED'):
            return item
        time.sleep(0.005)
    pytest.fail(f'operation did not finish within {timeout_s}s')
