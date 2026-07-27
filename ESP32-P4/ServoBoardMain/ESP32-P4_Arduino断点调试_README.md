# ESP32-P4 Arduino 固件断点调试

本文档适用于本机的 Waveshare ESP32-P4-Pico 和
`ESP32-P4/ServoBoardMain` 固件。

## 1. 调试架构

本项目继续使用 Arduino 编译和烧录，不创建第二套 ESP-IDF 固件：

```text
ServoBoardMain Arduino 源码
          |
          +-- Arduino IDE Release 编译/烧录：日常使用
          |
          +-- Arduino CLI Debug 编译/烧录：-Og -g3
                                             |
                                             +-- ESP-IDF OpenOCD + GDB 断点调试
```

ESP-IDF 5.5.2 只提供 OpenOCD 和 RISC-V GDB。不要在本工程执行
`idf.py build`、`idf.py flash` 或 `idf.py erase-flash`。

GDB 加载的 ELF 必须和板上固件来自同一次 Arduino Debug 编译。
Arduino IDE 重新编译 Release 固件后，旧 Debug ELF 不能继续用于断点调试。

## 2. 本机固定路径

| 项目 | 路径 |
|---|---|
| 英文工作区 | `C:\Users\g8765\mmhand-debug` |
| 原工作区 | `C:\浏览器下周\mmhand-cl\mmhand-cl` |
| Arduino CLI | `D:\g8765\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe` |
| OpenOCD | `D:\esp32\Espressif\tools\openocd-esp32\v0.12.0-esp32-20250707\openocd-esp32\bin\openocd.exe` |
| GDB | `D:\esp32\Espressif\tools\riscv32-esp-elf-gdb\16.3_20250913\riscv32-esp-elf-gdb\bin\riscv32-esp-elf-gdb.exe` |
| Debug ELF | `C:\Users\g8765\mmhand-debug\.debug\p4\ServoBoardMain.ino.elf` |

`C:\Users\g8765\mmhand-debug` 是 Windows Junction，不复制代码，也不占用第二份仓库空间。
调试时必须从该英文路径打开 VS Code：

```powershell
code C:\Users\g8765\mmhand-debug
```

不要从原中文路径启动调试，否则 GDB 可能无法将 ELF 中的源文件路径映射到编辑器。

## 3. 调试前安全要求

首次验证必须断开 27 台舵机的动力电源，只给 P4 控制板供电。

CPU 停在断点时，控制任务、上位机通信、心跳、CAN 接收和舵机目标刷新都会暂停。
已经发送给舵机的目标不会因为 CPU 断点自动撤销。带动力调试前必须确认：

1. 机械结构处于安全位置。
2. E-STOP 可立即切断舵机动力。
3. 不在舵机持续受力时长时间停住 Control 或 Servo 任务。
4. 优先使用条件断点、硬件观察点和 Logpoint。

## 4. 一次完整的断点调试

### 4.1 打开英文工作区

```powershell
code C:\Users\g8765\mmhand-debug
```

### 4.2 检查环境

在 VS Code 中按 `Ctrl+Shift+P`，执行：

```text
Tasks: Run Task
P4: Verify Debug Environment
```

也可以在 PowerShell 执行：

```powershell
powershell -ExecutionPolicy Bypass -File `
  C:\Users\g8765\mmhand-debug\tools\p4_debug\verify_debug_env.ps1
```

### 4.3 编译 Arduino Debug 固件

执行 VS Code 任务：

```text
P4: Arduino Debug Build
```

或者：

```powershell
powershell -ExecutionPolicy Bypass -File `
  C:\Users\g8765\mmhand-debug\tools\p4_debug\build_debug.ps1
```

该命令使用 Arduino Core 3.3.8 和 `--optimize-for-debug`，实际优化参数为：

```text
-Og -g3
```

构建成功后必须存在：

```text
.debug\p4\ServoBoardMain.ino.elf
.debug\p4\ServoBoardMain.ino.bin
.debug\p4\ServoBoardMain.ino.map
```

### 4.4 烧录同一份 Debug 固件

先查找串口：

```powershell
& "D:\g8765\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" board list
```

然后执行任务：

```text
P4: Arduino Debug Upload
```

输入实际端口，例如 `COM7`。也可以执行：

```powershell
powershell -ExecutionPolicy Bypass -File `
  C:\Users\g8765\mmhand-debug\tools\p4_debug\upload_debug.ps1 `
  -Port COM7
```

脚本保持以下配置不变：

```text
Flash Size      4MB
Partition       default
Erase Flash     Disabled
PSRAM           Disabled
Chip Variant    Before v3.00
```

因此正常情况下不会清除 Manual、Auto、LUT 和多圈 NVS。禁止执行整片擦除。

### 4.5 启动 OpenOCD

使用 ESP32-P4-Pico 板载 Type-C 调试口连接电脑，执行任务：

```text
P4: Start OpenOCD
```

成功时应看到：

```text
esp_usb_jtag: Device found
JTAG tap: esp32p4...
Listening on port 3333 for gdb connections
```

保持该终端运行。停止 OpenOCD 时在该终端按 `Ctrl+C`。

### 4.6 设置断点并连接

建议第一次在以下低风险位置设置断点：

```text
System_Init()
StateMachineTask 的状态切换分支
上位机命令解析入口
```

不建议第一次就在 10 ms Control 循环或 Servo 总线发送中长时间停住。

按 `F5`，选择：

```text
P4: Attach Arduino Debug Firmware
```

VS Code 会连接 `localhost:3333`、复位并暂停 P4，然后继续运行。
断点命中后可以查看：

- 局部变量和函数参数；
- `sharedData` 状态；
- FreeRTOS 任务相关控制字段；
- 调用栈；
- 内存和硬件观察点。

常用按键：

| 操作 | 快捷键 |
|---|---|
| 继续 | `F5` |
| 单步跳过 | `F10` |
| 单步进入 | `F11` |
| 跳出函数 | `Shift+F11` |
| 停止调试 | `Shift+F5` |

## 5. 恢复日常 Release 固件

调试结束后，在 Arduino IDE 中打开：

```text
C:\浏览器下周\mmhand-cl\mmhand-cl\ESP32-P4\ServoBoardMain\ServoBoardMain.ino
```

保持原有板卡、4MB 分区和 `Erase All Flash = Disabled`，正常点击 Upload。
Arduino IDE 会使用 `-Os` 重新生成 Release 固件。

## 6. 常见问题

### OpenOCD 报 `LIBUSB_ERROR_NOT_FOUND`

Windows 没有为 Espressif USB-JTAG 接口安装 WinUSB 驱动。通过 ESP-IDF
Installation Manager 安装 Espressif JTAG 驱动，然后重新插拔 Type-C。

### OpenOCD 找不到 `board/esp32p4-builtin.cfg`

必须通过 `start_openocd.ps1` 启动，脚本已经传入正确的 `-s` scripts 目录。

### 断点为空心或永远不命中

依次检查：

1. 板上是否烧录了当前 `.debug/p4` 目录对应的 Debug BIN。
2. GDB 是否加载同目录中的 `ServoBoardMain.ino.elf`。
3. 是否从英文工作区打开 VS Code。
4. Debug 构建是否使用 `--optimize-for-debug`。
5. OpenOCD 是否仍监听 3333 端口。

### 断点后上位机掉线或系统进入故障

这是 CPU 暂停导致心跳和串口协议停止的预期现象。恢复运行后，根据实际状态重新连接、
清除故障或重启 P4。不要用此现象判断正常运行时通信链路有问题。

### 暂停后板卡重启

先确认是不是任务 watchdog 或外部供电复位。当前第一阶段不修改业务固件的 watchdog
行为；只有实测确认断点必然触发 watchdog 后，才增加仅 Debug 构建启用的条件配置。

### VS Code 提示找不到 ELF

先执行：

```text
P4: Arduino Debug Build
```

`.debug/` 是本地构建目录，已经加入 `.gitignore`，拉取 Git 仓库后不会自动存在。
