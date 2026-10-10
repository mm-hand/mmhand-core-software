"""本地网页应用装配契约。

职责：创建唯一后台、网页和可选 ROS2 桥。
线程：网页事件循环及后台服务线程。
输入：启动选项。
输出：本地控制台和关闭结果。
状态所有权：进程生命周期，设备状态属于 HandBackend。
禁止依赖：不得实现协议、标定和机械控制算法。
"""
from desktop.core.hand_backend import HandBackend
from desktop.replay.replay_service import ReplayService
from desktop.ros2.joint_targets import Ros2JointTargetBridge


def run_ui(*, ros2_enabled=True, backend=None, show=True, port=8080):
    from nicegui import app, ui, run
    from desktop.ui.browser_view import build_page
    hand = backend or HandBackend()
    bridge = Ros2JointTargetBridge(hand) if ros2_enabled else None
    hand.open_application(replay_service=ReplayService(hand))
    try:
        if bridge:
            try:
                bridge.start()
                hand.set_ros_availability(True)
            except Exception as exc:
                hand.set_ros_availability(False, f'ROS2 unavailable: {exc}')
        else:
            hand.set_ros_availability(False, 'Disabled by --no-ros2')
        @ui.page('/')
        def index():
            build_page(hand)

        async def shutdown():
            if bridge:
                await run.io_bound(bridge.stop)
            await run.io_bound(hand.close_application)

        app.on_shutdown(shutdown)
        ui.run(host='127.0.0.1', port=port, show=show, reload=False, title='MM-Hand')
    finally:
        if bridge:
            bridge.stop()
        hand.close_application()
    return 0
