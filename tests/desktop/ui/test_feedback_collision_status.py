from types import SimpleNamespace

from desktop.ui.ui_app import HandGUI


def _format_status(*, armed_mask: int, hold_mask: int):
    """不创建 Qt 窗口，直接验证 P4 碰撞位图到顶部状态栏的格式化结果。"""
    app = SimpleNamespace(
        _last_feedback_collision_armed_mask=None,
        _last_feedback_collision_hold_mask=None,
    )
    state = SimpleNamespace(
        has_feedback_collision_status=True,
        feedback_collision_armed_bitmap=armed_mask,
        feedback_collision_hold_bitmap=hold_mask,
    )
    return HandGUI._format_feedback_collision_summary(app, state, True)


def test_dual_tendon_j00_hold_is_visible_in_top_status():
    text, tooltip, active, known = _format_status(
        armed_mask=1 << 0,
        hold_mask=1 << 0,
    )

    assert text == "COLLISION HOLD J00"
    assert "J00 (dual tendon)" in tooltip
    assert "hold_mask=0x00000001" in tooltip
    assert active is True
    assert known is True


def test_dual_tendon_j20_armed_is_visible_in_top_status():
    text, tooltip, active, known = _format_status(
        armed_mask=1 << 20,
        hold_mask=0,
    )

    assert text == "COLLISION MON"
    assert "J20 (dual tendon)" in tooltip
    assert "armed_mask=0x00100000" in tooltip
    assert active is False
    assert known is True
