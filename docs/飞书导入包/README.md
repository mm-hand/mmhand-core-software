# MMHand 飞书导入包

整理日期：2026-09-10

本目录是根据当前工作区源码重新编排的文档包，供逐篇导入飞书。它不是对旧
`docs` 目录的简单复制，也不替代源码。主文档只记录当前正式入口、当前数据流
和当前安全边界；历史推理、旧实验结论和重复的 Word 文档放在最后的历史附录。

## 推荐导入顺序

1. `00_项目总览与接手指南.md`
2. `01_上位机架构与使用流程.md`
3. `02_P4固件架构与状态机.md`
4. `03_控制算法与标定恢复.md`
5. `04_通信硬件映射与触觉数据.md`
6. `05_数据记录测试与故障排查.md`
7. `06_文件职责与维护规则.md`
8. `07_历史资料附录.md`

导入飞书时建议在项目 Wiki 根页面下建立“MMHand 项目交接”目录，再将上述
文件分别作为子文档导入；本 README 可作为目录页正文。

## 事实源优先级

发生冲突时按以下顺序判断，不以旧文档中的行号、旧参数或历史日志覆盖当前实现：

1. 当前源码、生成器、JSON schema、manifest 和 URDF contract；
2. 本导入包中的现行说明；
3. 已标明版本和设备身份的日志、CSV 与实测报告；
4. `docs` 中带日期的专项说明；
5. 历史事故复盘、旧脚本和旧 Word 文档。

关键事实源：

| 主题 | 当前源码/资源 |
| --- | --- |
| 上位机入口 | `desktop/__main__.py`、`desktop/app/entry.py` |
| 上位机分层 | `desktop/app`、`desktop/core`、`desktop/core/communication`、`desktop/ui` |
| P4 任务与参数 | `ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp`、`src/config/*.h` |
| P4 映射 | `ESP32-P4/ServoBoardMain/src/config/topology/HardwareMap.cpp` |
| P4 状态/安全 | `src/tasks/system_state`、`src/safety`、`src/generated/SafetyRuleCatalog.generated.h` |
| 上位机协议 | `desktop/resources/config/upper_protocol_schema.json`及其生成物 |
| S3 采集 | `ESP32-S3/System/src/config/Config.h`、`src/hal`、`src/tasks` |
| 触觉/磁编模型 | `desktop/core/data_models.py`、`desktop/core/communication/state_reducer.py` |
| 记录格式 | `desktop/core/data_recording/topic_schema.py`、`desktop/ui/ui_app.py` |

## 本包不收录

- `.gitignore` 命中的运行日志、缓存、构建产物和本机路径文件；
- `calibration_bundle.mmcal.json`、`multiturn_runtime.json` 等真实设备运行数据；
- 旧 `.docx` 的重复正文；
- 已删除的旧启动入口、旧串口日志工具和旧实验绘图链路。

真实配置、固件、日志和图片应按设备序列号、手型、日期和 commit 另行归档，
不要直接把个人电脑路径或密码写入飞书。

