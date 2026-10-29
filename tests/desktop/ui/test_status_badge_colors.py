"""状态色卡：红=必须人工处理，黄=需要恢复，蓝=正在动作，绿=可操作，灰=中性。"""
import pytest

from desktop.ui.browser_view import status_background


@pytest.mark.parametrize('label,color', [
    ('Stopped (0)', '#fecaca'),
    ('Stopped', '#fecaca'),
    ('Fault', '#fecaca'),
    ('Joint Beyond Workspace (9)', '#fde68a'),
    ('Joint Beyond Workspace', '#fde68a'),
    ('BEYOND', '#fde68a'),
    ('Pull Joint into Workspace (8)', '#bfdbfe'),
    ('Running (2)', '#bfdbfe'),
    ('Ready (5)', '#bbf7d0'),
    ('Idle (1)', '#d1fae5'),
    ('Disabled', '#e2e8f0'),
    ('UNKNOWN', '#e2e8f0')])
def test_state_colors_tell_the_operator_what_to_do(label, color):
    assert status_background(label) == color
