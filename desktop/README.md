# Desktop Application

`desktop` 是 mmhand 上位机 Python package。当前统一入口是：

```bash
python -m desktop
```

旧的 `desktop/development_mode.py`、`desktop/ui_main.py` 和 `desktop/main.py` 兼容入口已经删除。请使用 `python -m desktop ui|cli`，或直接导入 `desktop.app`、`desktop.core`、`desktop.ui` 下的模块。

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
python tools/check_desktop_architecture.py
python -m compileall -q desktop tests
pytest -q tests
```

正式运行模块统一使用“职责、线程、输入、输出、状态所有权、禁止依赖”六项中文文件头。
分层边界、允许依赖和新增文件维护方式见 [ARCHITECTURE_P0.md](ARCHITECTURE_P0.md)。

ROS2 功能需要当前 Python 环境能导入 `rclpy` 和 `std_msgs`。如果使用 ROS2 Jazzy，请注意它通常绑定 Python 3.12；不要把 ROS2 的 Python 3.12 site-packages 混到 Python 3.10 环境中运行。
本机独立 `mh` checkout 已使用 Python 3.12 创建 `.venv`；Linux 上先执行
`source /opt/ros/jazzy/setup.bash`，再用 `.venv/bin/python -m desktop ...`。

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
- `replay`：读取 Save Data 中已实际下发的关节命令 CSV，并按原时间轴整段回放。

示例：

```bash
python -m desktop ui --mode replay --port COMx
```

## Replay 动作轨迹

Replay 不再保存单帧手势 JSON，而是直接读取 `Save Data` 生成的
`07_遥操作总表.csv` 或 `02_关节总表.csv`，按 `host_time` 还原整段21关节
目标或实际运动。

操作顺序：

1. 连接P4，选择需要控制的关节并按`ENABLE`。
2. 完成标定；P4 会自动执行 Rope Init，确认系统状态为`READY`。
3. `Save Data` 默认开启，连接 P4 后会自动开始保存基础 CSV；如不需要记录可手动取消勾选。
4. 打开`Replay`页面，点击`导入 CSV`选择轨迹文件。
5. 核对轨迹时长、帧数、active scope及21关节实时角后点击`Send`。

Replay只有在启动瞬间系统状态严格等于`READY`时才会进入。运行期间如果发生
E-STOP、Pause、断线、FAULT或active scope变化，Replay会立即终止。碰撞保持
不会暂停轨迹；后续反向目标会继续发往P4并用于解除保持。
播放期间`Send`会切换为`Stop`；URDF CRC不一致或轨迹越过当前工作空间时会拒绝启动。

完整操作说明见 [algorithm/README.md](algorithm/README.md)。

## Save Data 文件

`Save Data` 默认记录基础 CSV，触觉全量表默认不保存；需要触觉原始全量数据时，
可先取消勾选再重新勾选，并在弹出的选项中启用。

默认保存目录只生成以下 6 张便于人工查看的 CSV：

1. `01_基础状态.csv`：系统状态、故障、active scope、多圈状态和碰撞检测位图。
2. `02_关节总表.csv`：21 路标定后实时角度、目标角、磁编码 raw 和 Joint Debug。
3. `03_舵机总表.csv`：27 路目标、raw/ABS、负载、速度、原始电流、电压、温度和在线状态。
4. `04_触觉总表.csv`：五指触觉汇总。
5. `06_操作事件总表.csv`：按钮操作、状态切换和碰撞检测进入/解除事件。
6. `07_遥操作总表.csv`：普通 UI、Test 与外部 ROS2 实际下发目标及时间轴，可作为 Replay 输入。

勾选“触觉全量”后额外生成 `05_触觉全量.csv`。原来的关节长表、舵机长表、
Joint Debug 原始包和五指触觉宽表不再保存；需要触觉阵列明细时，勾选“触觉全量”
生成 `05_触觉全量.csv`。

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
它显示当前 P4 映射关节角与 27 路舵机反馈；不依赖已删除的 `JOINT1_DEBUG` 包。

## ROS2 HORA closed loop

ROS2 输入桥默认启用；没有 ROS2 环境或只使用普通 UI 时，可显式加
`--no-ros2` 关闭。

UI + ROS2：

```bash
python -m desktop ui --mode monitor --port COMx
```

CLI + ROS2：

```bash
python -m desktop cli --port COMx
```

自定义 topic：

```bash
python -m desktop cli --port COMx --ros2 --ros-topic /mmhand/joint_targets
```

默认同时把 P4 实测映射角发布到 `/mmhand/joint_states`。完整 HORA 配置：

```bash
python -m desktop ui --mode monitor --port /dev/ttyUSB0 \
  --ros2-input joint_target \
  --ros2-feedback-mode armed \
  --ros2-joint-state-topic /mmhand/joint_states \
  --ros2-joint-target-topic /mmhand/joint_targets
```

`--ros2-input joint_target|raw_ik` 只创建所选目标 topic 的一份订阅。
`--ros2-feedback-mode` 支持：

- `armed`（默认）：现有 `ROS2 INPUT` 同时控制目标下发与反馈发布。
- `always`：持续发布反馈用于诊断；关闭按钮不会使 HORA 超时重置。
- `off`：不发布关节反馈，仅保留目标输入。

自动 start：

```bash
python -m desktop cli --port COMx --ros2 --auto-start
```

`--auto-start` 只启动普通控制器，不会自动放行遥操。GUI 中还需点击
`ROS2 INPUT`；CLI 中按 `i` 开启、按 `o` 关闭。在默认 `armed` 模式下，
关闭后至少停发反馈 300 ms；重新开启时界面显示“等待策略重置”，并丢弃
冷却期间的反馈和目标。冷却完成后只接受新到达的有效 P4 映射角，第一帧
反馈发布成功后才允许目标下发。

推荐先启动上位机与 HORA，保持 `ROS2 INPUT` 关闭，完成人工稳定抓握后再
开启。启动程序本身不要求已经抓握，但放行策略前必须由操作员确认抓稳。
暂停、急停、断连和作用域变化会撤销授权且不会自动恢复。

ROS2 消息约定：

- 目标：`/mmhand/joint_targets`，`std_msgs.msg.Float32MultiArray`，21 个
  J00-J20 URDF degree（raw-IK 可由 CLI 显式选 radian）。
- 反馈：`/mmhand/joint_states`，`sensor_msgs.msg.JointState`，名称 J00-J20，
  `position` 为 21 个 P4 实测 URDF radian。
- 反馈 `header.stamp` 是该映射角帧到达上位机的系统时间；其他遥测不会刷新。
- 反馈 `header.frame_id` 使用 `mmhand:p4_time_ms=<uint32>`，携带同一映射角包
  自带的 P4 `millis()`。它是 P4 开机后的回绕毫秒计数，只用于跨日志关联，
  不是 TF 坐标系名称，也不能直接与 `header.stamp` 相减。当前不做时钟映射。
- 不完整、无效、重复或超过 200 ms 的反馈不发布；错误长度、NaN、Inf 目标拒收。

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
  paths.py                    # 资源路径 helper
  requirements.txt            # desktop 依赖说明

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
    motor_mapping.py          # 电机/关节映射与直控辅助函数
    angle_mapping.py          # 关节/电机映射
    auto_calib_logger.py      # 自动标定日志
    session_log.py            # session 日志

  ui/
    ui_app.py                 # HandGUI 主窗口
    ui_context.py             # UI component context
    ui_control_panels.py      # 控制面板
    ui_pose_view.py           # 手姿态视图
    ui_tactile_view.py        # 触觉视图
    ui_joint_plot_view.py     # 实时曲线视图
    ui_windows.py             # 弹窗/子窗口管理
    ui_services.py            # UI 后台服务与 state buffer
    readme.md                 # UI 内部架构说明

  ros2/
    joint_targets.py          # /mmhand/joint_targets 订阅和校验

  resources/
    config/                   # 标定和运行配置
    hku_hand_v2_urdf/         # URDF、mesh 和相关资源

  examples/
    ros2_joint_target_sender.py

  replay/
    replay_models.py          # 轨迹数据模型
    replay_controller.py      # READY门禁与平滑回放线程
    replay_service.py         # UI无关Replay服务

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
