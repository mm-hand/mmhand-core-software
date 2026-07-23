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
- `replay`：读取 Save Data 遥操作 CSV，并按原时间轴整段回放。

示例：

```bash
python -m desktop ui --mode teleop --port COMx
python -m desktop ui --mode replay --port COMx
```

## Replay 遥操作轨迹

Replay 不再保存单帧手势 JSON，而是直接读取 `Save Data` 生成的
`07_遥操作总表.csv`，按 `host_time` 还原整段21关节动作。

操作顺序：

1. 连接P4，选择需要控制的关节并按`ENABLE`。
2. 完成标定和`Rope Init`，确认系统状态为`READY`。
3. 遥操时勾选`Save Data`，记录完整目标轨迹。
4. 打开`Replay`页面，点击`导入 CSV`选择轨迹文件。
5. 核对轨迹时长、帧数、active scope和URDF CRC后点击`Send`。

Replay只有在启动瞬间系统状态严格等于`READY`时才会进入。运行期间如果发生
E-STOP、Pause、断线、FAULT、active scope变化或碰撞保持，Replay会立即终止。
播放期间`Send`会切换为`Stop`；URDF CRC不一致或轨迹越过当前工作空间时会拒绝启动。

完整操作说明见 [algorithm/README.md](algorithm/README.md)。

## Save Data 文件

默认保存目录只生成以下 6 张便于人工查看的 CSV：

1. `01_基础状态.csv`：系统状态、故障、active scope、多圈状态和碰撞检测位图。
2. `02_关节总表.csv`：21 路标定后实时角度、目标角、磁编码 raw 和 Joint Debug。
3. `03_舵机总表.csv`：27 路目标、raw/ABS、负载、速度、原始电流、电压、温度和在线状态。
4. `04_触觉总表.csv`：五指触觉汇总。
5. `06_操作事件总表.csv`：按钮操作、状态切换和碰撞检测进入/解除事件。
6. `07_遥操作总表.csv`：遥操作输入、实际下发目标和对应时间轴。

勾选“触觉全量”后额外生成 `05_触觉全量.csv`。原来的关节长表、舵机长表、
Joint Debug 原始包和五指触觉宽表不再重复生成 CSV，但仍可写入 HDF5/ROS2 bag
供算法训练和底层诊断。

运行中发生碰撞时可以从三处确认：

- P4 文本日志：`[COLLISION_ARM]`、`[COLLISION_HOLD]`、`[COLLISION_RELEASE]`。
- `01_基础状态.csv`：连续记录 `feedback_collision_armed_mask` 和
  `feedback_collision_hold_mask`。
- `06_操作事件总表.csv`：记录 `collision_monitor_enter/exit` 和
  `collision_hold_enter/release`，并注明具体关节和位图。

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
    replay_models.py          # 手势与序列数据模型
    replay_store.py           # JSON原子存储、备份和导入导出
    replay_capture.py         # 遥操目标/P4反馈捕获
    replay_controller.py      # READY门禁与平滑回放线程
    replay_service.py         # UI无关Replay服务

  tools/
    raw_ik_urdf_viewer.py

  visualize/
    plot_*.py                 # 离线绘图和调试可视化脚本

run_data/
  运行日志/                   # 会话、自动标定和任务运行诊断日志
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
