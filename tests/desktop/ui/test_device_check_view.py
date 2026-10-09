"""Device Check 磁编码器方向显示测试。"""

import pytest

from desktop.core.data_models import HandModel
from desktop.ui.ui_device_check_view import DeviceCheckView
from desktop.ui.ui_qt_compat import ensure_app


def test_encoder_direction_display_uses_clear_arrows() -> None:
    assert DeviceCheckView._encoder_direction_display(1) == (
        "↑",
        "Raw increases from Manual Min toward Manual Max",
    )
    assert DeviceCheckView._encoder_direction_display(-1) == (
        "↓",
        "Raw decreases from Manual Min toward Manual Max",
    )
    assert DeviceCheckView._encoder_direction_display(0) == (
        "?",
        "Waiting for P4 encoderDirection",
    )


def test_device_check_replaces_visible_refresh_with_manual_prepare() -> None:
    """Device Check 只保留自动同步，工具栏显示高风险 Manual Prepare。"""
    _app = ensure_app()

    class _Backend:
        @staticmethod
        def get_state_snapshot():
            return HandModel()

    class _Context:
        backend = _Backend()

        @staticmethod
        def is_comm_connected():
            return False

        @staticmethod
        def set_status(_text):
            return None

        @staticmethod
        def emit_action(_text):
            return None

    view = DeviceCheckView(_Context(), mapping_version=4, urdf_crc=0x12345678)
    assert view.manual_prepare_btn.text() == "Release Rope"
    assert not hasattr(view, "refresh_btn")


def _calibration_status_view():
    app = ensure_app()

    class _Backend:
        @staticmethod
        def is_started():
            # 宿主校准期间普通控制入口关闭，不应改变 Manual/Auto 数据状态。
            return False

    class _Context:
        backend = _Backend()

        @staticmethod
        def is_comm_connected():
            return True

    view = DeviceCheckView(_Context(), mapping_version=4, urdf_crc=0x12345678)
    view._test_app = app
    return view


@pytest.mark.parametrize("calib_status", ["IDLE", "PENDING", "RUNNING", "FAILED"])
def test_joint_status_preserves_manual_before_auto_during_host_calibration(calib_status):
    """Manual 尚未录入的关节与只缺 Auto 的关节必须继续区分。"""
    view = _calibration_status_view()
    state = HandModel()
    state.calib_status = calib_status
    state.manual_reference_committed = True
    state.manual_reference_valid_bitmap = 1
    state.manual_reference_min_raw[0] = 5000
    state.manual_reference_max_raw[0] = 1000
    state.manual_reference_encoder_directions[0] = -1
    state.has_manual_reference_encoder_directions = True

    view.update_state(state)

    assert view._status_items[0].text() == "Needs auto calib"
    assert view._status_items[1].text() == "Manual missing"
    assert view._status_items[0].toolTip() == "Auto calibration missing"
    assert view._status_items[1].toolTip() == "Manual reference missing"
    view.deleteLater()


def test_full_manual_table_without_auto_then_manual_clear_updates_each_joint_status():
    """还原或写入全部 Manual 后全为 Need Auto；清空后回到 Need Manual。"""
    view = _calibration_status_view()
    state = HandModel()
    count = len(state.encoders)
    state.manual_reference_committed = True
    state.manual_reference_valid_bitmap = (1 << count) - 1
    state.manual_reference_min_raw = [5000] * count
    state.manual_reference_max_raw = [1000] * count
    state.manual_reference_encoder_directions = [-1] * count
    state.has_manual_reference_encoder_directions = True

    view.update_state(state)
    assert {item.text() for item in view._status_items} == {"Needs auto calib"}

    state.manual_reference_valid_bitmap = 0
    view.update_state(state)
    assert {item.text() for item in view._status_items} == {"Manual missing"}
    view.deleteLater()


def test_manual_missing_remains_visible_with_stale_auto_mask():
    """Manual 被清空后的旧 Auto 位不能把关节误标成 Need Auto 或 READY。"""
    view = _calibration_status_view()
    state = HandModel()
    state.manual_reference_committed = True
    state.auto_calibration_valid_bitmap = (1 << len(state.encoders)) - 1

    view.update_state(state)

    assert {item.text() for item in view._status_items} == {"Manual missing"}
    view.deleteLater()
