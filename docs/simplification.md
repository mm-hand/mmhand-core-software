# MM-Hand 八页简化交付与维护

正式工作目录最终统一为 `C:\Users\73547\mmhand`，分支
`codex/mmhand-simplified-refactor`。唯一控制入口是 `python -m desktop [--no-ros2]`。

## 保留与清理

八页为 Joint Calibration、Motor Calibration、Feedback Control、Direct Control、Replay、
Plots & Recordings、Tactile、Diagnostics。页面操作集中在左上角紧凑的一行，ROS2 INPUT
在末尾。公共栏和 STOP、默认 ON 记录、纯输出日志沿用已实现布局。

删除旧 CLI、Raw IK 独立输入桥、Hand Pose/3D/静态模型服务、STL、全部离线绘图、
FTServo 示例、旧 Qt 页面及对应专属测试。保留固件库源码和许可证；保留 URDF、映射、
限位、方向、指纹字段及协议定义，模型校验不依赖网格。完整文件列表见
[删除清单](deleted-files.txt)；当前代码结构和逐文件物理行数见
[目录统计](code-inventory/README.md)和[逐文件表](code-inventory/files.csv)。

自动标定只有 Web GUI → HandBackend → CalibrationWorkflowService → Host Calibration →
单/双电机流程 → 逐关节提交 → Rope Init。服务不再继承旧工作流，也没有旧固件标定或
启动恢复入口。辅助指让位、顺序、阈值、时间对齐、LUT、取消和部分成功处理保留。
端点确认属于后端，首次两个 Set 后保存，成功后单端 Set 使用另一端重新保存；
导入仍先校验预览，恢复读回成功后才激活与备份，不自动 ENABLE。

HandModel 从 418 行逐项复制改为 dataclass 字段复制，明确保留本地目标/零点。
列表、字典、触觉 NumPy 数组和嵌套标定结果不共享；标量列表使用快速 list.copy。
旧代码浅复制部分嵌套数据的速度不能作为相同隔离语义的基准；测量结果见
[复制耗时](state-copy-benchmark.json)。当前满触觉与示例 LUT 的复制约 0.45 ms，
高于旧浅复制的 0.045 ms，软件测试和页面刷新正常，持续实机负载仍需验收。

StateReducer 的长函数按 23 种包类型拆为同模块普通函数，保留批次顺序、时间戳、
新鲜度和发布边界。没有增加事件总线或新控制层。相同周期收尾统计在现有
TaskRuntimeMetrics 中复用，周期及超期跳过策略不变。

## 数据和依赖

默认数据目录 `C:\Users\73547\mmhand_data`；唯一覆盖变量是 `MMHAND_DATA_DIR`。
recordings/logs/calibration/device_state 在仓库外。标定备份存活动文件旁的 backups。
历史资料分 source/worktree 存在 archive，不会自动激活。此次迁移源目录 39 个文件、
工作树 200 个文件，合计 1,567,737,647 bytes；复制前后与删除前都核对 SHA-256。
190 个已跟踪标定备份已移出 Git。证据见 [迁移摘要](runtime-data-migration.json)，
逐文件哈希在 archive 的两个 manifest.json 中。日志与记录不自动清理。

直接运行依赖只有 NumPy、pyserial、NiceGUI 3.16.0；测试另需 pytest 8.4.2 和主机
C++ 编译器。不直接依赖 matplotlib、plotly、requests、Pillow；NiceGUI 的间接依赖
由其安装声明管理。已用没有系统 site-packages 的 Python 3.13 环境安装并跑全量门禁。

## 提交门禁

安装命令见 [启动说明](../desktop/README.md)。`.githooks/pre-commit` 调用唯一
`tools/precommit_checks.py --staged`，不修改文件、不生成、不自动 git add。
要求测试依赖与主机编译器、源码/测试/配置完整暂存、cached diff --check、协议/模型/
安全规则生成物一致、Desktop/P4 架构、安全闭环、全量 pytest 全部通过。
缺依赖或必测项跳过直接拒绝提交。Windows 可自动读取已安装 MSVC 官方开发环境。
不在每次 commit 完整编译固件；修改固件源码/宏后必须在交付和烧录前完整编译。

测试统一在 tests/，默认 pytest 不再遗漏旧 desktop/tests。修正旧协议载荷/索引、
活动与退役安全规则分别审计、缺失文件头与过期指纹；没有失败白名单或跳过。
当前清理工作树全量结果为 **698 passed / 0 failed / 0 skipped**，包含两项生产 C++
主机编译测试。干净 Python 环境同样通过。门禁负例覆盖失败测试、过期生成物、
缺失依赖/编译器、跳过/空测试报告和部分暂存；失败不会改动暂存区。

## 合并与实机验收

通过正常 Git merge 合入源目录，核对两边已提交的设备隔离和恢复证明安全改进。
用户授权丢弃的源目录未提交 Rope Init 文件、测试和招聘文档不进入新版本。
未生成或改写 docs/ai 的双语自动指南。旧 GUI 截图仅作为历史参考，未打开页面。

实机验收按以下顺序执行：

1. 从源目录启动八页 Web GUI；无标定首次启动、P4/S3 选择与连接/断开、默认记录等待。
2. Joint Min→Max 与 Max→Min；重复单端不首次保存；成功后任一端再次 Set 自动保存。
3. 导入当前联合文件，检查预览范围、设备身份、读回、备份与状态；不自动 ENABLE。
4. Motor 按关节选完整关联电机；辅助指让位、逐关节结果、Rope Init、部分失败与 STOP 取消。
5. OFFSET >360、WEAK RETURN ≥350、HIGH FRICTION ≥300；两类 Motor 告警可同时显示，原始测量值可查。
6. Feedback/Direct/ROS2/Replay 授权及互斥；编辑不发送；Pause/STOP 响应与取消后无后续运动。
7. Save Data ON/OFF、断开/重连、STOP 保留记录、退出收尾、写入故障转 OFF；检查 CSV/包记录完整性。
8. FULL/SUMMARY 触觉、新鲜度断点、持续记录及自动标定的 CPU/延迟；1920×1080 和笔记本下 STOP 不被遮挡。

上述实机项目需要连接真实设备，本次没有连接设备或执行运动。
