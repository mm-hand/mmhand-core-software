# MM-Hand Web Console

在源目录 `C:\Users\73547\mmhand` 执行：

```powershell
python -m pip install -r desktop/requirements.txt
python -m desktop --no-ros2
```

浏览器地址 `http://127.0.0.1:8080`（端口被占用时自动顺延到下一个可用端口并在
终端打印实际端口），只监听本机。启用 ROS2 时先加载可导入
rclpy/std_msgs 的环境，再运行 `python -m desktop`。只有 Web GUI 可以管理连接、
ENABLE、授权与停止。没有 CLI、独立 ROS2 控制入口或自动运动启动参数。

八页：Joint Calibration → Motor Calibration → Feedback Control → Direct Control →
Replay → Plots & Recordings → Tactile → Diagnostics。界面全英文，操作按钮集中在
左上角的一行，ROS2 INPUT 在该行末尾。Hand Pose、3D、STL 和离线绘图工具已删除；
URDF 仍用于关节限位和产品指纹，不要求网格存在。

先选 P4/S3 设备再连接。Save Data 默认 ON，有效 P4 数据就绪后开始新记录。
STOP 停止运动，保留记录。联合标定导入先预览，确认后恢复 P4、读回、激活本地文件，
不会自动 ENABLE。Joint 两端首次 Set 后自动保存，之后任一 Set 更新都会重新保存。
Motor 只运行当前 Host Calibration，按关节处理完整关联电机，逐关节提交，Rope Init 收尾。

数据默认写到 `C:\Users\73547\mmhand_data`，用唯一环境变量 `MMHAND_DATA_DIR`
可覆盖。recordings、logs、calibration、device_state 分目录保存；标定备份在活动文件
旁的 backups。archive 是历史资料，不会自动加载或激活。日志和记录不自动删除。

代码职责：

- app/entry.py、ui_runner.py：唯一入口、后台服务装配和关闭。
- ui/browser_view.py：固定连接/状态栏、导航、导入对话框和日志。
- ui/web/：每页一个普通 build/refresh 模块；UI 只持有目标草稿与显示缓存。
- core/hand_backend.py：统一控制接口、授权和 STOP。
- core/application_actions.py：后台操作、端点确认、记录生命周期。
- core/console_snapshot.py：Joint/Motor 质量状态解释，不改变保护规则。
- core/calibration_workflow.py：一个普通事务服务，拥有执行器、epoch、取消、事件和快照导入。
- core/host_calibration/：当前单/双电机算法、反馈时间对齐、LUT、逐关节提交和机械收尾。
- core/communication/：包路由、ACK 和按包类型分开的状态归并函数。
- replay/、ros2/：复用原有播放和 GUI 授权的 ROS2 输入。

开发前安装门禁依赖和钩子：

```powershell
python -m pip install -r requirements-test.txt
powershell -ExecutionPolicy Bypass -File tools/install_git_hooks.ps1
python tools/precommit_checks.py
```

Windows 需要 MSVC Build Tools 的 C++ 工具组件；脚本会读取其官方开发环境。
也可以使用 PATH 内的 g++ 或 clang++。提交钩子不修改文件、不生成、不自动暂存。
所有源码、测试与配置改动必须完整暂存；缺依赖、生成物过期、架构违规、测试失败或
任一测试跳过都会拒绝 commit。全量测试统一由 `tests/` 自动发现。
固件源码/宏变更后另行完整编译对应固件，提交时不重复完整编译。

维护和实机验收见 [交付说明](../docs/simplification.md)。
