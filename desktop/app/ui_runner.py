"""本地网页应用装配契约。

职责：创建唯一后台、网页和可选 ROS2 桥；默认 HTTP 端口被占用时自动顺延。
线程：网页事件循环及后台服务线程。
输入：启动选项。
输出：本地控制台和关闭结果。
状态所有权：进程生命周期，设备状态属于 HandBackend。
禁止依赖：不得实现协议、标定和机械控制算法。
"""
import os
import socket

from desktop.core.hand_backend import HandBackend
from desktop.replay.replay_service import ReplayService
from desktop.ros2.joint_targets import Ros2JointTargetBridge

UI_PORT_SCAN_SPAN = 50


def resolve_ui_port(host='127.0.0.1', preferred=8080, span=UI_PORT_SCAN_SPAN):
    """从首选端口起依次探测可用 HTTP 端口，全部被占用时报错。"""
    for candidate in range(preferred, preferred + span):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
            # POSIX 下与 uvicorn 绑定行为一致，忽略 TIME_WAIT 残留；
            # Windows 的 SO_REUSEADDR 语义不同，不设置。
            if os.name == 'posix':
                probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                probe.bind((host, candidate))
            except OSError:
                continue
        return candidate
    raise RuntimeError(
        f'No free HTTP port on {host} within {preferred}-{preferred + span - 1}'
    )


def run_ui(*, ros2_enabled=True, backend=None, show=True, port=8080):
    from nicegui import app, ui, run
    from desktop.ui.browser_view import build_page
    hand = backend or HandBackend()
    # 唯一外部目标入口：J00-J20 的角度（度）。
    bridges = [Ros2JointTargetBridge(hand)] if ros2_enabled else []
    hand.open_application(replay_service=ReplayService(hand))
    try:
        if bridges:
            failures = []
            for bridge in bridges:
                try:
                    bridge.start()
                except Exception as exc:
                    failures.append(f'{type(bridge).__name__}: {exc}')
            if failures:
                hand.set_ros_availability(False, 'ROS2 unavailable: ' + '; '.join(failures))
            else:
                hand.set_ros_availability(True)
        else:
            hand.set_ros_availability(False, 'Disabled by --no-ros2')
        @ui.page('/')
        def index():
            build_page(hand)

        async def shutdown():
            try:
                for bridge in reversed(bridges):
                    await run.io_bound(bridge.stop)
            finally:
                await run.io_bound(hand.close_application)

        app.on_shutdown(shutdown)
        bound_port = resolve_ui_port(preferred=port)
        if bound_port != port:
            print(f'[desktop] HTTP port {port} is occupied; serving on port {bound_port} instead')
        ui.run(host='127.0.0.1', port=bound_port, show=show, reload=False, title='MM-Hand')
    finally:
        stop_error = None
        for bridge in reversed(bridges):
            try:
                bridge.stop()
            except Exception as exc:
                stop_error = stop_error or exc
        try:
            hand.close_application()
        finally:
            if stop_error:
                raise stop_error
    return 0
