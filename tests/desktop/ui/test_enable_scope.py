from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from desktop.core.protocol import SYSTEM_STATE_IDLE, SYSTEM_STATE_UNCONTROLLABLE
from desktop.ui.ui_control_commands import ControlCommandController


@pytest.mark.parametrize("scan,latched,expected", [
    (1, False, "blocked"), (2, False, "hold"), (0, True, "hold"),
])
def test_scan_and_estop_reenable_require_fresh_hold(scan, latched, expected):
    backend = MagicMock()
    backend.enable_direct_hold.return_value = (True, "hold", [3000] * 27)
    panel = SimpleNamespace(
        _app_mode="manual", _manual_submode=lambda: "motor",
        _state=SimpleNamespace(servo_route_scan_state=scan,
                               system_state=SYSTEM_STATE_UNCONTROLLABLE if latched else SYSTEM_STATE_IDLE),
        _estop_latched=latched,
        _active_motor_mask_from_ui=lambda: 8,
        _active_scope_offline_motor_indices=lambda _: [],
        _active_scope_unroutable_motor_indices=lambda _: [],
        _active_scope_multiturn_unknown_mask=lambda: 0,
        _apply_active_scope_for_current_mode=lambda: True,
        _clear_motor_target_after_single_send=MagicMock(),
        _set_status=MagicMock(), context=MagicMock(), backend=backend,
    )
    ControlCommandController.start_controller(panel)
    if expected == "hold":
        backend.enable_direct_hold.assert_called_once_with(8)
        panel._clear_motor_target_after_single_send.assert_called_once_with(3, 3000)
        assert backend.reset.called == latched
    else:
        backend.enable_direct_hold.assert_not_called()
        backend.start.assert_not_called()
