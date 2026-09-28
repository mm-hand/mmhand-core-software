"""Desktop 文件头和静态依赖边界回归测试。"""

from tools import check_desktop_architecture


def test_desktop_architecture_contract() -> None:
    """正式模块必须保留六项中文契约头，且不得反向穿透分层。"""
    failures = check_desktop_architecture.collect_failures()
    assert not failures, "\n" + "\n".join(failures)
