"""为 Desktop 正式模块生成统一的中文文件契约头。

职责：集中维护各模块职责，并写入线程、输入输出、状态所有权和禁止依赖说明。
线程：维护工具仅在开发者主动执行时单线程运行。
输入：desktop 正式 Python 文件及本文件中的模块职责表。
输出：只替换目标模块首部文档字符串，不修改任何可执行语句。
状态所有权：只拥有本次文件改写过程，不接触运行期设备状态。
禁止依赖：禁止导入 desktop 业务模块，避免执行上位机启动副作用。
"""

from __future__ import annotations

import ast
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DESKTOP_ROOT = PROJECT_ROOT / "desktop"
TARGET_TOP_LEVEL_FILES = (
    "__init__.py",
    "__main__.py",
    "main.py",
    "paths.py",
    "serial_log_filter.py",
)
TARGET_PACKAGES = ("app", "core", "algorithm", "recording", "ros2", "teleop", "ui")
GENERATED_FILES = {"desktop/core/generated_upper_protocol.py"}

# 职责文字按模块名维护；重名 __init__.py 由下方 PACKAGE_PURPOSES 单独描述。
MODULE_PURPOSES = {
    "__main__": "实现 `python -m desktop` 的标准包启动入口。",
    "main": "保留旧启动方式到正式应用入口的兼容转发。",
    "paths": "集中解析仓库、资源、配置和运行数据目录。",
    "serial_log_filter": "过滤串口文本日志并保留诊断所需记录。",
    "cli_monitor": "提供无图形界面的设备连接、状态查看和诊断命令。",
    "entry": "解析上位机命令行参数并选择 CLI 或 Qt 运行模式。",
    "ui_runner": "装配 HandBackend、Qt 主界面和可选 ROS2 输入桥。",
    "angle_mapping": "统一 URDF、标定坐标和下位机角度空间之间的换算。",
    "auto_calib_logger": "解析并持久化自动标定过程的结构化诊断日志。",
    "calibration_snapshot": "校验、合并并原子保存当前版本的标定快照。",
    "comm_layer": "实现串口收发、帧解析、ACK 协调和设备状态快照更新。",
    "core_logic": "封装面向业务的下位机命令序列和基础控制流程。",
    "data_models": "定义上位机共享的设备状态、标定状态和遥测数据模型。",
    "hand_backend": "向 Qt、ROS2、遥操和 Replay 提供统一的设备能力门面。",
    "hand_geometry": "提供手部关节、手指和执行器几何关系查询。",
    "motor_mapping": "维护 J00-J20 与 M00-M26 的稳定映射和位图换算。",
    "protocol": "编码下行命令帧并集中暴露生成的协议常量。",
    "session_log": "管理单次连接会话的文本日志文件和日志上下文。",
    "csv_exporter": "将类型化 Topic 样本输出为便于人工查看的分表 CSV。",
    "event_recorder": "把 UI、ROS2、Replay 和标定操作写入统一事件流。",
    "motion_sampler": "分类控制输入并将连续运动轨迹降采样到记录频率。",
    "recorder": "编排 CSV 和事件记录会话生命周期。",
    "topic_schema": "定义各运行数据 Topic 的字段、类型和 CSV 文件契约。",
    "replay_controller": "按 CSV 原始时间轴调度完整动作轨迹并处理停止和重定位。",
    "replay_csv_loader": "识别遥操或关节总表 CSV 并恢复带时间戳的稀疏轨迹。",
    "replay_models": "定义 Replay 轨迹、帧、模型契约和导入诊断数据结构。",
    "replay_service": "向 Qt 页面提供轨迹导入、预览、播放和停止用例。",
    "joint_targets": "订阅最终关节目标 Topic 并经 HandBackend 下发 21 关节角。",
    "raw_ik_target": "订阅原始 IK Topic，完成目标适配后交给统一关节命令处理器。",
    "example_teleop": "提供遥操链路的人工启动和联调示例。",
    "joint_target_adapter": "把不同遥操来源统一转换到 J00-J20 URDF 角度空间。",
    "manus_calibration": "维护 MANUS 手套姿态标定和目标角适配流程。",
    "manus_raw_skeleton_retarget": "把 MANUS 原始骨架实时重定向为机器人关节目标。",
    "manus_receiver": "接收外部 MANUS 数据流并发布线程安全的最新帧。",
    "manus_sdk_receiver": "封装 MANUS SDK 生命周期和回调数据接收。",
    "camera_mocap": "采集摄像头手部关键点并生成视觉遥操目标。",
    "constants": "集中保存视觉手部重定向的稳定常量和默认资源路径。",
    "interfaces": "定义视觉重定向输出端口和集成接口。",
    "math_utils": "提供重定向所需的向量、旋转和坐标变换函数。",
    "retargeting": "把人体手部关键点通过约束求解转换为机器人关节角。",
    "urdf_model": "解析活动 URDF 并提供轻量前向运动学模型。",
    "vision": "封装 MediaPipe 检测和视觉诊断绘制。",
    "official_tactile_layout": "定义官方触觉模组到五指 UI 的固定布局。",
    "ui_app": "创建主窗口并编排状态刷新、故障展示和各功能页面。",
    "ui_camera_view": "展示摄像头画面并管理视觉预览控件。",
    "ui_context": "保存 UI 组件共享的只读服务引用和短生命周期上下文。",
    "ui_control_calibration": "实现控制面板中的标定、Rope Init 和恢复交互。",
    "ui_control_commands": "实现控制面板的 Enable、Stop、E-STOP 和目标发送命令。",
    "ui_control_common": "提供控制面板模块共享的格式化、校验和小型工具函数。",
    "ui_control_device": "构建设备控制栏并转发设备级操作。",
    "ui_control_manual": "实现直控模式下的舵机目标编辑和发送交互。",
    "ui_control_modes": "构建负反馈、直控和测试模式页面并连接模式事件。",
    "ui_control_panels": "作为模块化 ControlPanel 的稳定门面组合各控制能力。",
    "ui_control_scope": "管理关节/舵机选择范围和活动位图的 UI 规则。",
    "ui_control_state": "根据 HandModel 快照刷新控制状态、门禁和提示。",
    "ui_control_tables": "构建关节与舵机通道表格并定义稳定列布局。",
    "ui_control_target_model": "计算 UI 目标角、动态工作空间和边界钳位结果。",
    "ui_control_test_panel": "构建手动测试参数、动作和诊断控件。",
    "ui_control_tests": "实现测试模式的阶跃、扫频、前馈和停止操作。",
    "ui_control_view": "构建控制页面公共视图并协调控件可见性。",
    "ui_device_check_view": "展示 21 关节 Manual/Auto 数据并完成写入核验。",
    "ui_dialogs": "提供 Linux/Windows 可滚动且可关闭的统一模态对话框。",
    "ui_joint_plot_view": "实时绘制关节反馈、目标和控制诊断曲线。",
    "ui_manus_view": "展示 MANUS 状态、标定流程和遥操目标。",
    "ui_pose_view": "显示活动 URDF 的 3D 手部姿态并支持模型重载。",
    "ui_qt_compat": "隔离 PySide6 与 PyQt5 差异并提供统一 Qt 符号。",
    "ui_replay_view": "导入运行 CSV、预览实时关节角并回放完整动作。",
    "ui_services": "管理 UI 使用的后台服务、线程和停止生命周期。",
    "ui_style": "集中定义上位机颜色、字体和控件样式。",
    "ui_tactile_view": "展示五指触觉汇总和可选全量触觉阵列。",
    "ui_task_runtime": "展示 P4 任务周期、执行时间、栈和 CPU 诊断。",
    "ui_teleop_view": "构建遥操入口、RAW IK 门禁和连接状态视图。",
    "ui_visual_retargeting_view": "管理视觉遥操线程、目标适配和画面状态。",
    "ui_windows": "创建独立工具窗口并统一窗口生命周期。",
}

PACKAGE_PURPOSES = {
    "desktop/__init__.py": "声明 MMHand 上位机 Python 包及稳定公共边界。",
    "desktop/app/__init__.py": "声明 CLI 与 Qt 应用入口包。",
    "desktop/core/__init__.py": "声明协议、设备模型和后端核心包。",
    "desktop/core/data_recording/__init__.py": "声明多模态运行数据记录子包。",
    "desktop/replay/__init__.py": "声明 CSV 动作回放用例包。",
    "desktop/recording/__init__.py": "声明可选外部记录后端包。",
    "desktop/ros2/__init__.py": "声明可选 ROS2 输入集成包。",
    "desktop/teleop/__init__.py": "声明遥操数据接入与目标适配包。",
    "desktop/teleop/Posture/__init__.py": "声明摄像头姿态重定向包。",
    "desktop/teleop/Posture/hand_retargeting/__init__.py": "声明轻量手部重定向算法包。",
    "desktop/ui/__init__.py": "声明 Qt 图形界面包。",
}


def iter_target_files() -> list[Path]:
    """返回所有手写正式模块；生成文件由对应 schema 生成器维护。"""
    files = [DESKTOP_ROOT / name for name in TARGET_TOP_LEVEL_FILES]
    for package in TARGET_PACKAGES:
        files.extend((DESKTOP_ROOT / package).rglob("*.py"))
    return sorted(
        path
        for path in set(files)
        if path.is_file() and path.relative_to(PROJECT_ROOT).as_posix() not in GENERATED_FILES
    )


def _profile(path: Path) -> tuple[str, str, str, str, str, str]:
    """按架构层生成线程、数据流和依赖边界说明。"""
    relative = path.relative_to(PROJECT_ROOT).as_posix()
    purpose = PACKAGE_PURPOSES.get(relative, MODULE_PURPOSES.get(path.stem))
    if not purpose:
        raise KeyError(f"缺少模块职责说明: {relative}")

    if "/ui/" in relative:
        return (
            purpose,
            "Qt GUI 主线程执行；耗时工作必须委托后台服务，禁止阻塞界面事件循环。",
            "用户操作、HandModel 只读快照以及 HandBackend/服务返回结果。",
            "Qt 控件状态、用户提示和经门面提交的应用命令。",
            "仅拥有本视图控件和瞬时交互状态；设备真值由 core 层拥有。",
            "禁止直接导入串口通信和底层控制实现，禁止在 UI 中复制协议或安全规则。",
        )
    if "/ros2/" in relative:
        return (
            purpose,
            "ROS2 executor 线程执行；与 Qt/串口线程通过 HandBackend 边界协作。",
            "ROS2 Topic 消息、节点参数和活动模型契约。",
            "校验后的 21 关节目标或明确拒绝诊断。",
            "仅拥有 ROS2 节点、订阅器和最近消息元数据，不拥有设备状态。",
            "禁止依赖 Qt/UI，禁止直接写串口或绕过 HandBackend 的控制门禁。",
        )
    if "/algorithm/" in relative:
        return (
            purpose,
            "解析在调用线程执行，播放调度在 Replay 工作线程执行。",
            "运行 CSV、模型契约、活动关节范围和播放控制事件。",
            "带时间轴的稀疏关节帧、进度、诊断和停止结果。",
            "轨迹对象由 ReplayService 拥有；设备运行状态只读且归 HandBackend。",
            "禁止依赖 Qt、串口和 ROS2，禁止直接修改 HandModel 或下位机状态。",
        )
    if "/recording/" in relative and "/core/data_recording/" not in relative:
        return (
            purpose,
            "由记录会话工作线程调用；外部运行时初始化不得阻塞 Qt 主线程。",
            "统一 Topic 样本、会话元数据和记录配置。",
            "外部记录文件、写入状态和可诊断错误。",
            "仅拥有记录后端句柄和会话资源，不拥有控制状态。",
            "禁止依赖 UI 或参与控制决策，禁止反向修改采集快照。",
        )
    if "/teleop/" in relative:
        forbidden = "禁止依赖 Qt/UI 或直接写串口，目标必须经过统一角度适配和后端门禁。"
        if path.name == "example_teleop.py":
            forbidden = "禁止承载正式控制规则；该文件只装配示例 UI、ROS2 和遥操组件。"
        return (
            purpose,
            "接收器可在 SDK/摄像头工作线程执行；纯映射和数学函数不创建线程。",
            "手套、骨架或视觉关键点，以及活动 URDF/标定配置。",
            "统一 J00-J20 目标、接收状态和可诊断质量信息。",
            "接收器拥有最近输入帧；角度映射和模型配置按只读快照使用。",
            forbidden,
        )
    if "/core/data_recording/" in relative:
        return (
            purpose,
            "采样入口可被通信线程调用，磁盘写入由记录器串行化；不得阻塞设备控制。",
            "带双时间戳的类型化 Topic 样本、事件和会话配置。",
            "CSV 文件以及结构化写入状态。",
            "记录会话拥有文件句柄、采样序号和降采样缓存，不拥有设备真值。",
            "禁止依赖 UI 或改变控制状态，禁止在 writer 内重新解释协议业务语义。",
        )
    if "/core/" in relative:
        thread = "除 comm_layer 的串口收发线程外不主动创建 GUI 线程；共享状态必须按既有锁读取。"
        if path.name == "comm_layer.py":
            thread = "串口接收线程更新设备快照；命令可由 UI、ROS2 或 Replay 调用线程提交。"
        return (
            purpose,
            thread,
            "P4 字节流、上层类型化命令、配置和持久化数据。",
            "线程安全的 HandModel 快照、协议帧、ACK 和后端操作结果。",
            "通信对象拥有串口与解析缓存；HandModel 的写入只允许在核心同步边界内发生。",
            "禁止依赖 Qt/UI、app、ROS2 或 Replay 页面，禁止把控件和展示规则带入核心层。",
        )
    if "/app/" in relative or relative in {"desktop/__main__.py", "desktop/main.py"}:
        return (
            purpose,
            "在进程主线程执行；仅负责创建和关闭 CLI、Qt 及可选 ROS2 生命周期。",
            "命令行参数、环境变量和启动配置。",
            "已装配的应用服务、进程退出码和启动诊断。",
            "只拥有启动期配置与生命周期句柄，不拥有设备运行状态。",
            "禁止实现协议解析、控制算法和设备安全判断，统一委托 core/服务层。",
        )
    return (
        purpose,
        "纯模块初始化或工具调用线程执行，不创建后台线程。",
        "包配置、路径参数或文本流。",
        "稳定导入边界、路径或过滤后的数据。",
        "只拥有模块级不可变配置，不拥有设备运行状态。",
        "禁止导入 Qt 控件或直接操作下位机通信。",
    )


def _replace_module_docstring(path: Path) -> bool:
    """只替换模块 docstring，并保留源文件原有换行风格。"""
    original = path.read_bytes().decode("utf-8")
    newline = "\r\n" if "\r\n" in original else "\n"
    tree = ast.parse(original, filename=str(path))
    purpose, thread, inputs, outputs, ownership, forbidden = _profile(path)
    header = newline.join(
        (
            f'"""{path.name} - MMHand Desktop 模块契约。',
            "",
            f"职责：{purpose}",
            f"线程：{thread}",
            f"输入：{inputs}",
            f"输出：{outputs}",
            f"状态所有权：{ownership}",
            f"禁止依赖：{forbidden}",
            '"""',
            "",
        )
    )
    lines = original.splitlines(keepends=True)
    first = tree.body[0] if tree.body else None
    if (
        isinstance(first, ast.Expr)
        and isinstance(first.value, ast.Constant)
        and isinstance(first.value.value, str)
    ):
        updated = "".join(lines[: first.lineno - 1]) + header + "".join(lines[first.end_lineno :])
    else:
        insert_at = 1 if lines and lines[0].startswith("#!") else 0
        updated = "".join(lines[:insert_at]) + header + "".join(lines[insert_at:])
    if updated == original:
        return False
    path.write_bytes(updated.encode("utf-8"))
    return True


def main() -> int:
    """更新全部手写正式模块，并输出变更数量供维护者核对。"""
    changed = [path for path in iter_target_files() if _replace_module_docstring(path)]
    print(f"Desktop headers updated: {len(changed)} files.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
