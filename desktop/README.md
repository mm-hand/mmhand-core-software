# Desktop Application

```powershell
python -m desktop
python -m desktop --no-ros2
```

无参数打开 Qt GUI。端口及 Monitor/Replay 在界面中选择，连接和重连不自动授权运动。
业务参数只保留 `--no-ros2`；旧 `ui`、`cli`、端口、模式和自动启动参数明确报错。
未安装 ROS2 时 GUI 正常启动，ROS 输入显示不可用原因。

检查入口：

```powershell
python tools/check.py
python tools/check.py --fast
```

完整检查需要原生 C++ 编译器，可用 `MMHAND_CXX` 指定其可执行文件。
快速检查用于现有提交 hook；固件构建及实机验收需另外执行。

模型生成：

```powershell
python tools/generate_robot_model.py "<结构仓库中的原始 URDF 路径>"
```

结构仓库必须干净，URDF 与资源必须已跟踪。检查全部通过后更新固定生成位置。
绑定表维护原名关节与电机关系；限位来自原始 URDF。

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
E-STOP、断线、FAULT或active scope变化，Replay会立即终止。碰撞保持
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

## ROS2 Joint Target Input

固定 topic `/mmhand/joint_targets`，类型 `std_msgs/msg/Float32MultiArray`。
`data` 为 21 个有限数值，单位 degree，顺序 J00–J20，含义为当前控制模型的原始 URDF 关节角。
非法消息整帧拒绝，原因限频记录。通过 GUI 授权后才能发送；STOP、急停和断联撤销授权。
高频订阅只保留最新帧，不支持自定义 topic、单位切换、raw_ik 或自动使能。

## Directory Structure

当前 `desktop` 目录按职责拆分如下：

```text
desktop/
  __init__.py
  __main__.py                 # package 主入口：python -m desktop
  paths.py                    # 资源路径 helper
  requirements.txt            # desktop 依赖说明

  app/
    entry.py                  # Qt 启动参数解析
    ui_runner.py              # UI 启动和 mode 分发

  core/
    protocol.py               # P4 串口协议打包/解析
    data_models.py            # 核心状态数据模型
    comm_layer.py             # 串口通信层
    core_logic.py             # HandController 等核心控制逻辑
    hand_backend.py           # UI/ROS2 共用 backend facade
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
