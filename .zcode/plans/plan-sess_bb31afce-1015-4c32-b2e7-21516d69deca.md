## 交付物总览（3 件）

### 1. `AGENTS.md` 新增英文小节 "ESP32 JTAG debugging (P4/S3)"

保持现有 6 行不动，追加一节（精简速查 + 铁律 + 指向详细文档）：

- **一句话架构**：Arduino 编译产物自带 DWARF 符号；调试栈 = arduino-cli（编译+烧录）→ openocd-esp32 → GDB，全部随 esp32 Arduino core 3.3.10 自带（`~/AppData/Local/Arduino15/packages/esp32/tools/`），**不需要 ESP-IDF/idf.py**。
- **四步速查命令**（P4 为主，标注 S3 差异：`xtensa-esp32s3-elf-gdb` + `board/esp32s3-builtin.cfg` + fqbn `esp32:esp32:esp32s3`）：build（`--optimize-for-debug --build-path .debug/p4`）→ upload → openocd（必须带 `-c "gdb_memory_map disable"`）→ gdb（`target remote :3333`）。
- **铁律（Iron rules，违反会锁死芯片）**：
  1. 永远先 `delete` 断点再 `detach`/复位——P4 硬件断点触发器跨软复位存活，残留触发器 → 停机 → HP WDT 复位死循环 → gdb 连接被拒；恢复法：`esptool -p COMx chip-id`（下载模式全系统复位）。
  2. OpenOCD 占用 USB-JTAG 期间禁止 esptool/串口监视。
  3. 烧录的 bin 与 GDB 读的 ELF 必须同一构建目录同源。
  4. `-Og` 下 `<optimized out>` 变量属正常，换断点行或读寄存器。
- 指向 `tools/debug/gdb_batch.py`（agent 一键入口）与详细文档。

### 2. `docs/debug/esp32-jtag-debugging.md`（英文详细文档）

章节：架构与原理（为什么不需要 ESP-IDF）→ 本机工具绝对路径 → 端到端流程完整命令 → **人工调试分步教程**（见下）→ agent 脚本用法 → GDB 命令速查（break/continue/info threads/info locals/print/bt/step/next/watch/set var/detach）→ 已知坑与恢复（memory_map、残留断点、COM 口与 JTAG 口互斥）→ 故障排查表（含"板子没枚举"物理检查清单：数据线/电源开关/USB-C 口位）。

### 3. `tools/debug/`（`git mv` 自 `tools/p4_debug/` 后原地改造）

- `p4_debug_env.ps1`：删除 g8765 硬编码与 junction 逻辑 → 自动探测 arduino-cli（IDE 内置）与 openocd/gdb（glob Arduino15 包内版本目录）；`-Target p4|s3` 参数（选 cfg/gdb/fqbn/构建目录）；本机用户名纯 ASCII 无需 junction。
- `build_debug.ps1`：保留 `--optimize-for-debug` 与 ELF SHA256 sidecar，参数化 Target。
- `upload_debug.ps1`：参数化 Target。
- `start_openocd.ps1`：**补上 `gdb_memory_map disable`**，按 Target 选 cfg。
- `verify_debug_env.ps1`：动态路径 + 设备可见性检查。
- **新增 `gdb_batch.py`（AI agent 一键入口，Python 因 tools/ 以 Python 为主且对 agent 最友好）**：
  - 用法：`python tools/debug/gdb_batch.py --target p4 --break MotionControlTask.cpp:37 --print nowMs --print periodTicks --locals --bt`
  - 行为：自动起 OpenOCD（若 3333 未监听）→ 生成 gdb `--batch` 命令链 → **try/finally 强制收尾纪律（delete → 确认 → detach）** → 原样输出 gdb 日志给 agent 阅读；不提供"带断点 reset"选项，让 agent 犯不了错。

## 人工打断点/查中间变量步骤（写入文档"Human workflow"节，也是本回答）

1. 编译+烧录（同速查命令，`--optimize-for-debug` 版）。
2. 终端 1 启动 OpenOCD 并保持运行（`board/esp32p4-builtin.cfg -c "gdb_memory_map disable"`）。
3. 终端 2 交互式 GDB：`riscv32-esp-elf-gdb.exe -q .debug/p4/ESP32-P4.ino.elf` → `target remote :3333` → `break 文件:行号或函数名` → `continue` → 命中后 `info locals` / `print 变量` / `bt` / `info threads`，`next`/`step` 单步，`continue` 恢复，`set var x=5` 可改值。
4. 结束：`delete` → `detach` → `quit`，终端 1 Ctrl+C。

## 实施与验证顺序

1. `git mv tools/p4_debug tools/debug` → 改 4 个 ps1 → 写 `gdb_batch.py` → 写 docs 文档 → 改 AGENTS.md。
2. 实测回归（用今天已验证的链路）：`gdb_batch.py --target p4 --break MotionControlTask.cpp:37 --locals --print nowMs` 应复现断点命中+变量输出，且收尾后板子继续正常运行、可再次连接。
3. 范围边界：不做 VS Code 配置；不 commit（未被要求）；docs/ai 与 AI_README 索引再生不在本轮。