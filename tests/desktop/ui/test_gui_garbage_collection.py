"""后台分配不能析构 Qt 循环对象；周期回收必须在 GUI 线程执行。"""

import gc
import threading
import weakref

import pytest

from desktop.ui.ui_gc import (
    GC_INTERVAL_MS,
    GuiGarbageCollector,
    install_gui_garbage_collector,
)
from desktop.ui.ui_qt_compat import QtCore, ensure_app


@pytest.fixture
def collector():
    app = ensure_app()
    guard = install_gui_garbage_collector(app)
    guard._timer.stop()
    yield app, guard
    # 继续保留应用级保护，避免后续 UI 测试又在后台自动 GC。
    guard._timer.setInterval(GC_INTERVAL_MS)
    guard._timer.start()


def _unreachable_qobject(destroyed_threads):
    obj = QtCore.QObject()
    obj._self_cycle = obj
    obj.destroyed.connect(
        lambda *_args: destroyed_threads.append(threading.get_ident())
    )
    return weakref.ref(obj)


def _run_worker(action):
    errors = []

    def run():
        try:
            action()
        except BaseException as exc:
            errors.append(exc)

    worker = threading.Thread(target=run)
    worker.start()
    worker.join(timeout=5.0)
    assert not worker.is_alive(), "worker failed to finish"
    return errors


def test_installation_is_application_owned_and_idempotent(collector):
    app, guard = collector
    assert install_gui_garbage_collector(app) is guard
    assert guard.parent() is app
    assert guard._timer.parent() is app
    assert guard.thread() == app.thread()
    assert guard._timer.thread() == app.thread()
    assert guard._timer.interval() == 1000
    assert not gc.isenabled()


def test_worker_allocations_leave_qobject_cycle_for_gui_collection(collector):
    _app, guard = collector
    destroyed_threads = []
    reference = _unreachable_qobject(destroyed_threads)

    def allocate_feedback_snapshots():
        for _ in range(5):
            snapshots = [[index, {}] for index in range(5000)]
            del snapshots

    assert _run_worker(allocate_feedback_snapshots) == []
    assert reference() is not None
    assert destroyed_threads == []
    guard.collect()
    assert reference() is None
    assert destroyed_threads == [threading.get_ident()]


@pytest.mark.parametrize("operation", ["collect", "install", "construct"])
def test_worker_cannot_collect_or_install(collector, operation):
    app, guard = collector
    operations = {
        "collect": guard.collect,
        "install": lambda: install_gui_garbage_collector(app),
        "construct": lambda: GuiGarbageCollector(app),
    }
    errors = _run_worker(operations[operation])
    assert len(errors) == 1
    assert isinstance(errors[0], RuntimeError)
    assert "GUI thread" in str(errors[0])
    assert not gc.isenabled()


def test_timer_collects_real_qobject_cycle_on_gui_thread(collector):
    app, guard = collector
    destroyed_threads = []
    reference = _unreachable_qobject(destroyed_threads)
    loop = QtCore.QEventLoop()
    deadline = QtCore.QTimer()
    deadline.setSingleShot(True)
    deadline.timeout.connect(loop.quit)
    obj = reference()
    obj.destroyed.connect(loop.quit)
    del obj
    guard._timer.start(1)
    deadline.start(2000)
    try:
        exec_loop = getattr(loop, "exec", None) or loop.exec_
        exec_loop()
    finally:
        deadline.stop()
        guard._timer.stop()
    assert reference() is None
    assert destroyed_threads == [threading.get_ident()]
    assert QtCore.QThread.currentThread() == app.thread()


def test_collection_prevents_reentry_and_recovers_after_exception(
    collector, monkeypatch
):
    _app, guard = collector
    calls = []

    def collect_with_nested_request():
        calls.append(threading.get_ident())
        assert guard.collect() == 0
        raise ValueError("simulated collection failure")

    monkeypatch.setattr(gc, "collect", collect_with_nested_request)
    with pytest.raises(ValueError, match="simulated collection failure"):
        guard.collect()
    assert calls == [threading.get_ident()]
    monkeypatch.setattr(gc, "collect", lambda: 7)
    assert guard.collect() == 7


def test_about_to_quit_collects_without_reenabling_automatic_gc(collector):
    app, _guard = collector
    destroyed_threads = []
    reference = _unreachable_qobject(destroyed_threads)
    app.aboutToQuit.emit()
    assert reference() is None
    assert destroyed_threads == [threading.get_ident()]
    assert not gc.isenabled()


def test_restore_requires_explicit_completed_shutdown(collector):
    _app, guard = collector
    with pytest.raises(RuntimeError, match="background workers are cleaned up"):
        guard.restore_after_shutdown()
    assert install_gui_garbage_collector(_app) is guard
    assert not gc.isenabled()


def test_restore_preserves_previously_disabled_gc(collector, monkeypatch):
    app, _guard = collector
    enabled_calls = []
    monkeypatch.setattr(gc, "enable", lambda: enabled_calls.append(True))
    standalone = GuiGarbageCollector(app)
    assert standalone.automatic_gc_was_enabled is False
    standalone.restore_after_shutdown(qt_objects_and_workers_stopped=True)
    assert enabled_calls == []
    assert not gc.isenabled()
    assert not standalone._timer.isActive()


def test_explicit_completed_shutdown_restores_saved_enabled_policy(
    collector, monkeypatch
):
    app, _guard = collector
    enabled_calls = []
    # 仅模拟进入前的策略，不能在测试中的活跃 QApplication 上真正开启自动 GC。
    monkeypatch.setattr(gc, "isenabled", lambda: True)
    monkeypatch.setattr(gc, "enable", lambda: enabled_calls.append(True))
    standalone = GuiGarbageCollector(app)
    assert standalone.automatic_gc_was_enabled is True
    standalone.restore_after_shutdown(qt_objects_and_workers_stopped=True)
    standalone.restore_after_shutdown(qt_objects_and_workers_stopped=True)
    assert enabled_calls == [True]
    assert not standalone._timer.isActive()
