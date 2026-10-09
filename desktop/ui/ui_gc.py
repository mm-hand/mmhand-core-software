"""ui_gc.py - MMHand Desktop 模块契约。

职责：把 Python 循环垃圾回收固定到 Qt GUI 线程，保护 Qt 对象析构。
线程：创建、显式回收与最终恢复均只能在 QApplication 所属线程执行。
输入：QApplication 生命周期以及 GUI 定时器事件。
输出：GUI 线程执行的循环垃圾回收与显式关闭后的原 GC 策略恢复。
状态所有权：拥有本应用的 GC 策略和定时器；不拥有设备或校准状态。
禁止依赖：禁止导入设备通信、控制算法或从后台线程析构 Qt 对象。
"""

from __future__ import annotations

import gc

from .ui_qt_compat import QtCore, Slot


GC_INTERVAL_MS = 1000
_APP_COLLECTOR_ATTRIBUTE = "_mmhand_gui_garbage_collector"


def _require_gui_thread(app) -> None:
    if QtCore.QThread.currentThread() != app.thread():
        raise RuntimeError("Qt garbage collection must run on the GUI thread")


class GuiGarbageCollector(QtCore.QObject):
    """QApplication 持有的循环回收器；自动 GC 在其有效期内保持关闭。

    自动 GC 会在任何触发分配的 Python 线程执行，可能把不可达的 Qt
    包装对象连同其 C++ 对象在后台线程销毁。引用计数仍正常工作；仅把
    循环回收改为 GUI 定时执行。退出信号不恢复自动 GC，因为此时后台
    worker 和 Qt 对象不一定都已清理完成。
    """

    def __init__(self, app) -> None:
        _require_gui_thread(app)
        super().__init__(app)
        self._app = app
        self.automatic_gc_was_enabled = gc.isenabled()
        self._collecting = False
        self._restored = False
        self._timer = QtCore.QTimer(app)
        self._timer.setInterval(GC_INTERVAL_MS)
        self._timer.timeout.connect(self.collect)
        app.aboutToQuit.connect(self.collect)
        gc.disable()
        self._timer.start()

    @Slot()
    def collect(self) -> int:
        """仅 GUI 线程可收集；析构回调再次请求回收时不重入。"""
        _require_gui_thread(self._app)
        if self._restored or self._collecting:
            return 0
        self._collecting = True
        try:
            return gc.collect()
        finally:
            self._collecting = False

    def restore_after_shutdown(
        self, *, qt_objects_and_workers_stopped: bool = False
    ) -> None:
        """仅在调用者已清理 Qt 对象并停止所有 worker 后恢复原 GC 策略。

        正常 GUI 生命周期无需调用，回收器由 QApplication 持有至销毁。
        此入口供完整关闭流程或隔离测试使用，不能用来暂停运行中的回收器。
        """
        _require_gui_thread(self._app)
        if qt_objects_and_workers_stopped is not True:
            raise RuntimeError(
                "Restore GC only after Qt objects and background workers are cleaned up"
            )
        if self._restored:
            return
        if self._collecting:
            raise RuntimeError("Cannot restore GC during an active collection")
        self._timer.stop()
        self._timer.timeout.disconnect(self.collect)
        self._app.aboutToQuit.disconnect(self.collect)
        self._restored = True
        if getattr(self._app, _APP_COLLECTOR_ATTRIBUTE, None) is self:
            delattr(self._app, _APP_COLLECTOR_ATTRIBUTE)
        if self.automatic_gc_was_enabled:
            gc.enable()
        else:
            gc.disable()


def install_gui_garbage_collector(app) -> GuiGarbageCollector:
    """为应用安装一次回收器，并以应用上的强引用保持其 Python 生命周期。"""
    _require_gui_thread(app)
    collector = getattr(app, _APP_COLLECTOR_ATTRIBUTE, None)
    if collector is None:
        collector = GuiGarbageCollector(app)
        setattr(app, _APP_COLLECTOR_ATTRIBUTE, collector)
    return collector


__all__ = ["GuiGarbageCollector", "install_gui_garbage_collector"]
