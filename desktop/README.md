# Desktop Application

`desktop` 是 mmhand 上位机 Python package。当前统一入口是：

```bash
python -m desktop
```

旧的 `desktop/development_mode.py` 和 `desktop/ui_main.py` 兼容文件已经删除；`desktop/main.py` 只保留为薄兼容入口。新代码应优先使用 `python -m desktop ui|cli`，或直接导入 `desktop.app`、`desktop.core`、`desktop.ui` 下的模块。

## Environment

常用验证环境：

```bash
conda activate torch
```

基础检查：

```bash
python -m desktop --help
python -m desktop ui --help
python -m desktop cli --help
python -m compileall -q desktop tests
pytest -q tests
```

ROS2 功能需要当前 Python 环境能导入 `rclpy` 和 `std_msgs`。如果使用 ROS2 Jazzy，请注意它通常绑定 Python 3.12；不要把 ROS2 的 Python 3.12 site-packages 混到 Python 3.10 环境中运行。

## Entrypoints

主入口：

```bash
python -m desktop
```

对应文件：

- `desktop/__main__.py`：`python -m desktop` 入口。
- `desktop/app/entry.py`：argparse 参数解析和 `ui`/`cli` 子命令分发。
- `desktop/app/ui_runner.py`：UI 模式启动、ROS2 bridge 生命周期管理。
- `desktop/app/cli_monitor.py`：CLI 监视和控制入口。

## UI Mode

启动 monitor UI：

```bash
python -m desktop ui --mode monitor
```

连接 P4 主串口：

```bash
python -m desktop ui --mode monitor --port COMx
```

同时指定 S3 tactile 串口：

```bash
python -m desktop ui --mode monitor --port COMx --tactile-port COMy
```

可选 UI mode：

- `monitor`：常规监视和控制界面。
- `teleop`：遥操作入口，调用 `desktop.teleop.example_teleop.run()`。
- `algorithm`：算法入口，调用 `desktop.algorithm.example_algorithm.run()` 并同时打开 UI。

示例：

```bash
python -m desktop ui --mode teleop --port COMx
python -m desktop ui --mode algorithm --port COMx
```

## CLI Mode

启动 CLI monitor：

```bash
python -m desktop cli --port COMx
```

禁用调试 plot：

```bash
python -m desktop cli --port COMx --no-plot
```

CLI 控制键：

- `s`：start
- `x`：stop
- `r`：reset
- `c`：calibrate
- `q`：quit

CLI 只通过 `HandBackend` 操作硬件，不直接持有或调用 `HandController`。

## ROS2 Joint Target Input

ROS2 默认关闭，需要显式加 `--ros2`。

UI + ROS2：

```bash
python -m desktop ui --mode monitor --port COMx --ros2
```

CLI + ROS2：

```bash
python -m desktop cli --port COMx --ros2
```

自定义 topic：

```bash
python -m desktop cli --port COMx --ros2 --ros-topic /mmhand/joint_targets
```

自动 start：

```bash
python -m desktop cli --port COMx --ros2 --auto-start
```

`--auto-start` 只启动普通控制器，不会自动放行遥操。GUI 中还需点击
`RAW IK ENABLE`；CLI 中按 `i` 开启、按 `o` 关闭。ROS2 节点在未授权时
仍持续接收最新角度，但不会向 P4 下发运动命令。

ROS2 消息约定：

- topic 默认值：`/mmhand/joint_targets`
- type：`std_msgs.msg.Float32MultiArray`
- data length：必须正好为 21
- unit：degree
- order：desktop J00-J20
- invalid length、NaN、Inf 会被拒收并节流 warning

测试发布脚本：

```bash
python -m desktop.examples.ros2_joint_target_sender
```

## Directory Structure

当前 `desktop` 目录按职责拆分如下：

```text
desktop/
  __init__.py
  __main__.py                 # package 主入口：python -m desktop
  main.py                     # 薄兼容入口，转发到 desktop.app.entry
  paths.py                    # 资源路径 helper
  requirements.txt            # desktop 依赖说明
  serial_log_filter.py        # 串口日志过滤工具

  app/
    entry.py                  # argparse 子命令：ui、cli
    ui_runner.py              # UI 启动和 mode 分发
    cli_monitor.py            # CLI monitor/control

  core/
    protocol.py               # P4 串口协议打包/解析
    data_models.py            # 核心状态数据模型
    comm_layer.py             # 串口通信层
    core_logic.py             # HandController 等核心控制逻辑
    hand_backend.py           # UI/CLI/ROS2 共用 backend facade
    hand_geometry.py          # 手部几何与姿态计算
    motor_mapping.py          # 电机/关节映射与直控辅助函数
    angle_mapping.py          # 关节/电机映射
    auto_calib_logger.py      # 自动标定日志
    run_data_logger.py        # 运行数据日志
    session_log.py            # session 日志

  ui/
    ui_app.py                 # HandGUI 主窗口
    ui_context.py             # UI component context
    ui_control_panels.py      # 控制面板
    ui_pose_view.py           # 手姿态视图
    ui_tactile_view.py        # 触觉视图
    ui_camera_view.py         # 摄像头视图
    ui_joint_plot_view.py     # 实时曲线视图
    ui_windows.py             # 弹窗/子窗口管理
    ui_services.py            # UI 后台服务与 state buffer
    readme.md                 # UI 内部架构说明

  ros2/
    joint_targets.py          # /mmhand/joint_targets 订阅和校验

  resources/
    config/                   # 标定和运行配置
    hku_hand_v2_urdf/         # URDF、mesh 和相关资源
    joint_sequence_example.txt
    pose_fist_21deg.txt

  examples/
    ros2_joint_target_sender.py

  teleop/
    example_teleop.py         # teleop mode 示例入口
    manus_*.py                # Manus/retargeting 相关模块
    Posture/                  # 姿态/重定向依赖和模型

  algorithm/
    example_algorithm.py      # algorithm mode 示例入口

  tools/
    raw_ik_urdf_viewer.py

  visualize/
    plot_*.py                 # 离线绘图和调试可视化脚本

  logs/                       # 本地运行日志
```

## Tests

测试目录不放在 `desktop` 源码根目录，统一放在仓库根目录的 `tests/` 下：

```text
tests/
  desktop/
    app/
    core/
    ros2/
```

推荐每次改动后至少运行：

```bash
conda activate torch
python -m compileall -q desktop tests
pytest -q tests
```
