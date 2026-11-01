"""core 测试共享 ``hand`` 夹具。

职责：构造不接硬件的 HandBackend，并在用例结束后关闭。
线程：测试主线程。
输入：pytest 的 monkeypatch / tmp_path。
输出：已就绪的 HandBackend。
状态所有权：只拥有每用例的临时后端。
禁止依赖：禁止连接真实串口或写真实标定文件。

具体装配与 ``wait_operation`` 放在 ``_conftest_support``，供需要显式导入的用例复用。
"""

import pytest

from _conftest_support import make_hand

@pytest.fixture
def hand(monkeypatch, tmp_path):
    backend = make_hand(monkeypatch, tmp_path)
    yield backend
    backend.close_application()
