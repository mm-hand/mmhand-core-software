"""Device Check 磁编码器方向显示测试。"""

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
