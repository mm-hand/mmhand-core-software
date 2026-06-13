本版主要改动 README
本文简述当前 cl 分支这一版的主要修改点，方便后续审查、测试和继续开发。
注：本版pid参数适用于我的灵巧手，若参数不符需要自行调整，触觉有问题尚未调通
1. 电机直控与姿态按钮
Joint Channels 中的 Open Pose / Fist Pose / Zero Pose 仍然走关节负反馈目标。
Motor Channels / 直控模式中的全局按钮和关节模式区分语义，直控侧用于电机原始控制。
直控模式的 Release Half Turn 会让当前 active/选中的电机松半圈绳，未选中的电机不动作。
直控模式每个 Send 只控制对应舵机，发送后清空 UI 输入，避免影响下一次指令。
已清理或调整重复/无效按钮逻辑，例如 Home、Save PID ABS 等。
2. Calibration Save / Load
标定完成后，标定结果会自动保存到上位机本地 JSON。
新增 Calib Load，用于机械状态未改变时手动恢复上次标定，避免重复标定。
适用场景包括上位机重启、下位机未断电、或排障后确认机械状态未变化。
故障清除流程会显示故障类型，并提示用户是否需要恢复之前的 calibration。
3. 日志与故障显示
默认降低/取消高频 [RX diag] / [TX diag] 传输速率刷屏。
上位机关闭时保存最近终端环形日志到： desktop/logs/session_YYYYMMDD_HHMMSS.log
日志目录不加入 Git。
overload 故障会在上位机显示具体超载电机和当时 load。
出现运行故障时会进入 STOP/故障处理流程，Clear Fault 提供中文确认提示。
4. 遥操窗口 UI
遥操关节选择区改为更紧凑的按手指分列布局。
支持单指全选、全手选择，也支持选择特定几个关节。
未选中的关节在 MANUS 遥操发送时保持当前反馈角，不被手套数据覆盖。
5. MANUS 手套遥操整合
保留旧版 TCP Bridge 链路： MANUS 手套 -> MANUS Core -> manus_bridge_test -> TCP -> mmhand desktop
新增 Direct SDK 链路： MANUS 手套 -> MANUS Core -> MANUS C++ SDK -> mmhand_manus_sdk.dll -> mmhand desktop
两种输入源共用 Open/Close 校准、归一化、J00-J20 关节映射和 Enable Joint Send 门控。
Direct SDK 包装层源码已接入；真实运行需要 MANUS Core 启动并连接手套。
更详细说明见 docs/manus_teleop_chain.md。
