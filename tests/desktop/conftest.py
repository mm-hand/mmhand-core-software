"""Desktop 测试不得写真实的运行期标定文件与日志目录。

2026-10-25 事故：``test_core_logic.py`` 的归零用例调用了真实的失效处理，
把 ``desktop/resources/config/calibration_bundle.mmcal.json`` 里的 J01 结果、
M02 限位和多圈锚点清掉了。这里的 autouse fixture 把标定包路径（及其
``backups/`` 兄弟目录）和标定下发日志目录都指向每用例的 tmp_path；
用例自己再用 monkeypatch 指定路径时，以用例的设置为准。

共享的 HandBackend 夹具位于 ``tests/desktop/core/_conftest_support.py``，
由需要它的用例显式 import；这里不重复定义。
"""

import importlib
from pathlib import Path

import pytest


PATH_CONSTANT_MODULES = (
    "desktop.core.core_logic",
    "desktop.core.hand_backend",
    "desktop.core.application_actions",
    "desktop.core.host_calibration.runtime",
)


@pytest.fixture(autouse=True)
def _isolate_runtime_persistence(tmp_path, monkeypatch):
    bundle = Path(tmp_path) / "calibration_bundle.mmcal.json"
    for name in PATH_CONSTANT_MODULES:
        module = importlib.import_module(name)
        if hasattr(module, "CALIB_ACTIVE_SNAPSHOT_PATH"):
            monkeypatch.setattr(module, "CALIB_ACTIVE_SNAPSHOT_PATH", bundle)
    from desktop.core.host_calibration import runtime
    monkeypatch.setattr(runtime, "CALIB_PUBLISH_LOG_DIR", Path(tmp_path) / "diag")
    yield
