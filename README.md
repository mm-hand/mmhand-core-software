# MM-Hand

MM-Hand 上位机使用 Python NiceGUI，硬件状态机、协议与保护策略保留在现有控制后端和固件。

## 启动

在仓库根目录运行：

```powershell
python -m pip install -r requirements-test.txt
python -m desktop --no-ros2
```

正式控制入口只有 `python -m desktop [--no-ros2]`。省略 `--no-ros2` 时接收
`/mmhand/joint_targets`，运动仍需 Web GUI 授权。启动不会自动连接或 ENABLE。

八页依次为 Joint Calibration、Motor Calibration、Feedback Control、Direct Control、
Replay、Plots & Recordings、Tactile、Diagnostics。所有硬件操作经过 HandBackend。

数据默认写入用户目录下的 `mmhand_data`，可用 `MMHAND_DATA_DIR` 覆盖。
历史资料保存在 archive，不自动加载为活动标定。

## 维护

先安装 Git 钩子，再提交：

```powershell
powershell -ExecutionPolicy Bypass -File tools/install_git_hooks.ps1
python tools/precommit_checks.py
```

每次 commit 必须通过生成物、架构、安全闭环及全量 Python/生产 C++ 主机测试。
Windows 需安装 MSVC Build Tools 或 GCC/Clang；必测项缺依赖或跳过会阻止提交。
固件源码或宏修改后，还须完整编译对应固件。

[启动与目录职责](desktop/README.md) · [简化交付与实机清单](docs/simplification.md) ·
[逐文件行数](docs/code-inventory/README.md) · [删除清单](docs/deleted-files.txt)
