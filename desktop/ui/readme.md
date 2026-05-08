# Desktop UI Architecture

本文档说明 `desktop/ui/` 目录下 Tkinter 上位机 UI 的当前拆分结构，便于快速定位代码和理解数据流。

## 总览

UI 仍然使用 `Tkinter + ttk + Matplotlib TkAgg`。外部入口保持不变：

```python
from ui_main import HandGUI, LauncherWindow
```

`HandGUI` 是主窗口协调器，位于 `app.py`。它不再承载所有绘图和控制面板实现，而是通过多个 mixin/组件组合出完整 GUI：

- `PoseViewMixin` / `Pose3DView`：手部 3D 位姿显示。
- `TactileViewMixin` / `TactileView`：触觉热力图显示。
- `CameraViewMixin` / `CameraView`：双相机显示和采集。
- `JointRealtimePlotMixin` / `JointRealtimePlotView`：实时关节曲线窗口。
- `ControlPanelsMixin`：Manual/Test/Teleop/Algorithm 控制面板。
- `DisplayWindowManager`：弹窗创建、菜单勾选、窗口布局和刷新节流。
- `HandGuiContext`：给弹窗组件使用的窄接口，避免组件直接依赖 `HandGUI` 私有实现。

## 文件职责

`app.py`

- 定义 `HandGUI` 主类。
- 创建 root、菜单、主布局、状态栏。
- 管理串口状态轮询、latest state 消费、日志调度、关闭清理。
- 通过继承各功能 mixin 获得控制面板、显示、Plot 等能力。
- 保留少量兼容方法，例如 `_draw_hand_model()`，用于旧控制流程触发姿态窗口刷新。

`launcher.py`

- 定义 `LauncherWindow`。
- 负责启动前选择串口和运行模式。
- 启动后仍调用 `modes.run(..., gui=True)`，因此外部启动流程不变。

`context.py`

- 定义 `HandGuiContext`。
- 对组件暴露稳定入口：`root`、`controller`、`current_mode`、`get_state()`、`set_state()`、`is_comm_connected()`、`request_redraw()`、`emit_action()`。
- 相机组件通过 `list_available_cameras()`、`select_camera()`、`stop_camera_capture()` 访问主应用能力。

`windows.py`

- 定义 `DisplayWindowManager`。
- 管理 `View` 菜单中的 `Hand Pose`、`Tactile`、`Cameras` 三类弹窗。
- 每类窗口是单例：重复打开会 `deiconify/lift/focus`，关闭会同步菜单勾选状态。
- 负责根据 Windows work area 自动摆放主窗口右侧的显示弹窗。
- 刷新策略：
  - pose：连接时约 10 FPS，离线预览约 4 FPS。
  - tactile：交给 `TactileView.update_if_due()` 自行节流。
  - camera：只在相机弹窗打开时更新画布。

`pose_view.py`

- 定义 `PoseViewMixin` 和 `Pose3DView`。
- 持有 URDF kinematics、Matplotlib figure/canvas/axes、静态/动态 artists、鼠标缩放拖拽状态。
- 主要接口：
  - `build(parent)`
  - `update(state)`
  - `draw_angles(angles, state=None, generate=None)`
  - `clear()`
  - `close()`
- 性能策略：静态场景尽量复用，只更新 link line 和 node scatter。

`tactile_view.py`

- 定义 `TactileViewMixin`、`TactileView` 和 latest-only 触觉矩阵 worker。
- 持有 axis/fake 控件、fake tactile generator、触觉矩阵构造、anatomy overlay、colormap/norm、Matplotlib canvas。
- 真实数据刷新约 10 FPS，fake/offline 预览约 4 FPS。
- 常规刷新路径中，触觉矩阵在后台 worker 计算；Tk/Matplotlib 绘制仍在主线程执行。
- overlay 几何只在 axis 或 sensor mask 变化时重建，避免每帧 remove/recreate artists。

`camera_view.py`

- 定义 `CameraViewMixin` 和 `CameraView`。
- 持有双相机 UI、OpenCV capture、采集线程、frame buffer、PIL/PhotoImage 显示。
- 相机采集在后台线程执行，Tk canvas 和 PhotoImage 更新在主线程执行。
- `CameraView.close()` 会释放 capture 并停止采集。

`joint_plot_view.py`

- 定义 `JointRealtimePlotMixin` 和 `JointRealtimePlotView`。
- 管理实时曲线窗口、关节选择、series buffer、line cache、清空和截图导出。
- 修复过重复打开 Plot 时的嵌套/画中画问题：重建前销毁旧 container/canvas/figure。
- 曲线更新复用 line objects，不再每帧全量重建。

`control_panels.py`

- 定义 `ControlPanelsMixin`。
- 包含主控制面板和模式相关控制：
  - Manual Control
  - Joint/Motor channel table
  - Joint Sine Test
  - Feedforward Test
  - Triangle Wave Test
  - Local Test Mode
  - Teleoperation Control
  - Algorithm Control
- 保留硬件命令逻辑入口，例如 Start/Stop/Reset/Home/Calibrate、Send All、单行 Send、测试 tick 调度。

`services.py`

- `LatestStateBuffer`：控制器回调写入 latest-only 状态，UI 轮询只消费最新状态，避免 backlog。
- `AsyncRunDataLogger`：后台写 CSV，主线程只入队。
- `clone_hand_model()`：将控制器状态复制为 UI 使用快照。

`style.py`

- Windows DPI、root、ttk 主题和字体设置。
- UI 入口应通过这里配置 Windows 风格，避免在各组件里分散设置。

## 数据流

1. `HandController` 收到下位机状态后触发 `HandGUI._on_state_update(state)`。
2. `_on_state_update()` 复制状态并写入 `LatestStateBuffer`，不直接绘图。
3. `_start_display_poll()` 在 Tk 主线程周期消费最新状态。
4. `_update_display()` 更新主控制面板反馈、Plot 采样、状态栏，并调用 `DisplayWindowManager.update_views(state)`。
5. `DisplayWindowManager` 只刷新已经打开的显示弹窗：
   - pose 弹窗：`Pose3DView.update(state)`
   - tactile 弹窗：`TactileView.update_if_due(state)`
   - camera 弹窗：由 camera poll 单独触发 `update_cameras()`

## 线程与性能约束

- Tk 控件、`PhotoImage`、Matplotlib canvas 必须只在主线程更新。
- 串口回调、触觉矩阵计算、相机采集、CSV 写入可以在后台线程做。
- 所有后台线程都应采用 latest-only 或 bounded queue 策略，避免状态积压。
- 关闭的弹窗不应继续执行对应的重绘逻辑。
- 新增显示组件时，应优先提供 `build(parent)`、`update(state)`、`clear()`、`close()` 这组接口。

## 维护建议

- 不要把新的绘图实现继续写回 `app.py`；显示相关代码放入对应 view 文件。
- 不要在弹窗组件中调用 `owner._setup_xxx()`、`owner._update_xxx()` 这类壳式私有方法。
- 组件需要主应用能力时，优先通过 `HandGuiContext` 增加明确方法。
- 控制协议、串口协议、校准文件读写不属于 UI 组件拆分范围，修改时应单独评估。
- UI 性能问题优先检查：
  - 是否在 Tk 主线程里做了大矩阵/图像计算；
  - 是否每帧 `ax.clear()` 或重建 artists；
  - 是否存在无限队列或过高频率 `after()` 刷新。
