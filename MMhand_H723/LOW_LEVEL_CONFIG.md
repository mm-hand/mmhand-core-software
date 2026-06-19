# MMhand_H723 底层配置说明

本文档记录当前 `MMhand_H723` 工程的 STM32CubeMX 底层配置依据。目标硬件为达妙科技 `DM-MC-Board02` 电机开发板，主控为 `STM32H723VGT6`，封装为 `LQFP100`。

当前配置已经按迁移目标重新生成：USB 时钟改为 `HSI48 48MHz`，四路舵机 UART 均为 `1000000 baud`，FDCAN1 为 Classic CAN 1Mbps。

## 工程定位

这个工程是从 `ESP32-P4/ServoBoardMain` 向 STM32H723 迁移的底层框架。CubeMX 负责生成这些底座：

- 系统时钟和外设时钟
- GPIO 复用引脚
- FDCAN1
- 四路 UART 舵机总线
- USB CDC 虚拟串口
- FreeRTOS CMSIS-RTOS V2
- SWD 调试

业务层迁移时，建议把 ESP32 侧的 `HardwareSerial`、`Serial`、`TWAI` 等接口集中替换到 STM32 平台适配层，而不是在业务代码里零散替换。

## 当前配置总览

| 模块 | 当前配置 | 用途 |
| --- | --- | --- |
| MCU | `STM32H723VGT6` | 达妙 DM-MC-Board02 主控 |
| Toolchain | `STM32CubeIDE` / GCC | 适合后续在 Ubuntu 开发 |
| HSE | 24MHz 外部晶振 | 系统主 PLL 输入 |
| CPU | 480MHz | 主控运行频率 |
| HCLK/AXI/AHB | 240MHz | 总线时钟 |
| APB1/2/3/4 | 120MHz | 外设总线时钟 |
| FDCAN1 | Classic CAN, Normal, 1Mbps | 编码器/触觉/错误帧 CAN 通信 |
| UART7 | PE8 TX, PE7 RX, 1Mbps | 舵机总线 |
| USART10 | PE3 TX, PE2 RX, 1Mbps | 舵机总线 |
| USART1 | PA9 TX, PA10 RX, 1Mbps | 舵机总线 |
| UART9 | PD15 TX, PD14 RX, 1Mbps | 24Pin 扩展口舵机总线 |
| USB_OTG_HS | Internal FS PHY, Device Only | 板载 USB CDC |
| USB clock | HSI48, 48MHz | USB FS 所需 48MHz |
| FreeRTOS | CMSIS_V2, 1kHz tick, 128KB heap | 多任务框架 |
| HAL timebase | TIM6 | 避免 SysTick 与 FreeRTOS 混用 |
| Debug | SWD, PA13/PA14 | 下载和调试 |

## 与原 ESP32 工程的对应关系

| 原 ESP32-P4 能力 | STM32H723 当前承接方式 |
| --- | --- |
| 4 路飞特 STS 舵机串口总线 | `UART7`、`USART10`、`USART1`、`UART9` |
| ESP32 TWAI CAN 1Mbps | `FDCAN1` Classic CAN 1Mbps |
| 上位机串口/USB 串口通信 | `USB_DEVICE CDC` 虚拟串口 |
| FreeRTOS 任务 | CubeMX FreeRTOS CMSIS_V2 |
| `millis()`/延时/周期任务 | FreeRTOS tick + HAL TIM6 timebase |

板载直接可用的 UART 是 3 路：`USART1`、`UART7`、`USART10`。第 4 路 `UART9` 位于 24Pin 扩展口，当前先在工程中预留，扩展板到位后再做实测。

## 引脚分配

| 功能 | 外设 | 引脚 | 板上位置/来源 | 配置理由 |
| --- | --- | --- | --- | --- |
| 外部高速晶振 | RCC HSE | PH0/PH1 | 板载 24MHz 晶振 | 提供稳定系统时钟源 |
| SWD 调试 | DEBUG | PA13/PA14 | 调试下载接口 | 保留 ST-LINK 下载和断点调试 |
| CAN1 | FDCAN1 | PD0 RX, PD1 TX | 板载 CAN1 | 与达妙板接口图一致 |
| 舵机总线 0 | UART7 | PE7 RX, PE8 TX | 板载 UART7 | 板载直接引出 |
| 舵机总线 1 | USART10 | PE2 RX, PE3 TX | 板载 UART10 | 板载直接引出 |
| 舵机总线 2 | USART1 | PA10 RX, PA9 TX | 板载 USART1 | 板载直接引出 |
| 舵机总线 3 | UART9 | PD14 RX, PD15 TX | 24Pin 扩展口 | 第 4 路舵机总线预留 |
| USB CDC | USB_OTG_HS FS PHY | PA11 DM, PA12 DP | 板载 USB | 上位机虚拟串口通信 |

这里按板子实际接口图配置，优先级高于 CubeMX 自动推荐引脚。特别是 FDCAN1 必须使用 `PD0/PD1`，不要误选到其它未引出的复用脚。

## 时钟树

### 输入时钟

| 时钟 | 当前值 | 用途 |
| --- | ---: | --- |
| HSE | 24MHz | PLL1 输入，系统主时钟基准 |
| HSI48 | 48MHz | USB FS kernel clock |
| LSI | 32kHz | 保留内部低速时钟 |
| LSE | 未启用 | 当前不需要 RTC 外部晶振 |

选择 HSE 24MHz 的原因：

- DM-MC-Board02 板载 24MHz 晶振。
- FDCAN、UART 和控制任务对时钟稳定性有要求。
- 与达妙 H723 例程风格一致，方便后续对照排查。

选择 HSI48 给 USB 的原因：

- USB Full Speed 需要 48MHz 时钟。
- PLL1Q 当前用于 FDCAN，频率为 120MHz，不适合再作为 USB 时钟。
- 将 USB 独立挂到 HSI48，可以避免为了 USB 改动 CAN 依赖的 PLL1Q。

### PLL1 配置

| 参数 | 当前值 | 推导 |
| --- | ---: | --- |
| PLL source | HSE | 24MHz |
| `DIVM1` | 2 | VCO input = 24 / 2 = 12MHz |
| `DIVN1` | 40 | VCO = 12 * 40 = 480MHz |
| `DIVP1` | 1 | PLL1P = 480MHz |
| `DIVQ1` | 4 | PLL1Q = 120MHz |
| `DIVR1` | 2 | PLL1R = 240MHz |
| `PLLFRACN` | 0 | 不使用小数 PLL |

### 主频和总线

| 时钟域 | 当前值 | 说明 |
| --- | ---: | --- |
| SYSCLK | 480MHz | 来源 PLL1P |
| CPU Clock | 480MHz | `D1CPRE = /1` |
| AXI Peripheral Clock | 240MHz | `HPRE = /2` |
| HCLK | 240MHz | AHB 总线 |
| APB1 | 120MHz | `D2PPRE1 = /2` |
| APB2 | 120MHz | `D2PPRE2 = /2` |
| APB3 | 120MHz | `D1PPRE = /2` |
| APB4 | 120MHz | `D3PPRE = /2` |
| APB timer clock | 240MHz | APB 分频后定时器时钟翻倍 |

当前先选择 480MHz，而不是把 H723 拉到更高频，是为了首版迁移阶段优先稳定。480MHz 已足够支撑当前通信、控制和 FreeRTOS 任务，等所有链路跑通后再评估是否需要更高主频。

### 外设时钟选择

| 外设 | 时钟源 | 当前频率 | 说明 |
| --- | --- | ---: | --- |
| FDCAN1 | PLL1Q | 120MHz | 用于推导 1Mbps CAN timing |
| USB_OTG_HS | HSI48 | 48MHz | USB Full Speed 必需 |
| USART1/UART9/USART10 | PCLK2 | 120MHz | 1Mbps UART |
| UART7 | PCLK1 | 120MHz | 1Mbps UART |
| TIM6 | APB1 timer clock | 240MHz | HAL timebase |

当前关键结果：

```text
RCC.FDCANFreq_Value = 120000000
RCC.USBCLockSelection = RCC_USBCLKSOURCE_HSI48
RCC.USBFreq_Value = 48000000
```

生成代码中 USB 时钟确认：

```c
PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
```

## 电源和 Flash

生成代码中：

```c
HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);
```

配置理由：

- `VOS0` 支撑 H7 高性能频率运行。
- `FLASH_LATENCY_3` 由 CubeMX 按 480MHz 主频生成。
- 当前使用 LDO 供电模式，符合常规板级配置。

## SYS 和调试

| 项目 | 当前值 | 理由 |
| --- | --- | --- |
| Debug | Serial Wire | 使用 ST-LINK/SWD 下载和调试 |
| Timebase Source | TIM6 | FreeRTOS 下避免 HAL 和 RTOS 共用 SysTick |
| `TIM6_DAC_IRQn` | Priority 15 | 低优先级 HAL tick |

FreeRTOS 工程里不建议让 HAL timebase 继续使用 SysTick。CubeMX 这里选 TIM6 是正确的，后续 `HAL_Delay()` 和 HAL timeout 由 TIM6 维护，RTOS 调度仍按 FreeRTOS tick 运行。

## FDCAN1

### 引脚

| 信号 | 引脚 |
| --- | --- |
| FDCAN1_RX | PD0 |
| FDCAN1_TX | PD1 |

### 基本参数

| 参数 | 当前值 |
| --- | --- |
| Frame Format | Classic mode |
| Mode | Normal mode |
| Auto Retransmission | Enable |
| Transmit Pause | Disable |
| Protocol Exception | Disable |
| Std Filters Nbr | 1 |
| Ext Filters Nbr | 0 |
| Rx FIFO0 Elements | 32 |
| Rx FIFO0 Element Size | 8 bytes |
| Tx FIFO Queue Elements | 8 |
| Tx FIFO Queue Mode | FIFO mode |
| Tx Element Size | 8 bytes |

配置理由：

- 原 ESP32 工程使用 TWAI Classic CAN，不使用 CAN FD 数据段。
- 当前协议帧 DLC 都按 8 字节以内设计，Classic CAN 足够。
- `Rx FIFO0 = 32` 能容纳编码器、触觉、错误详情帧的短时突发。
- `Tx FIFO Queue = 8` 足够首版命令下发和调试发送。
- `Std Filters Nbr = 1` 给后续 accept-all 或 ID 范围过滤留入口。

### 1Mbps 位时序

FDCAN kernel clock 为 120MHz，当前参数：

| 参数 | 当前值 |
| --- | ---: |
| Nominal Prescaler | 4 |
| Nominal Sync Jump Width | 4 |
| Nominal Time Seg1 | 23 |
| Nominal Time Seg2 | 6 |

推导：

```text
Time quantum clock = 120MHz / 4 = 30MHz
1 TQ = 33.333ns
1 bit = (1 + 23 + 6) * 33.333ns = 1000ns
baud = 1 / 1000ns = 1Mbps
sample point = (1 + 23) / 30 = 80%
```

CubeMX 显示 `999999 bit/s` 属于显示取整误差，可按 1Mbps 使用。

## 四路 UART 舵机总线

当前四路 UART 已经全部配置为 `1000000 baud`。

| Servo Bus | 外设 | TX | RX | Baud | 来源 |
| --- | --- | --- | --- | ---: | --- |
| 0 | UART7 | PE8 | PE7 | 1000000 | 板载 UART7 |
| 1 | USART10 | PE3 | PE2 | 1000000 | 板载 UART10 |
| 2 | USART1 | PA9 | PA10 | 1000000 | 板载 USART1 |
| 3 | UART9 | PD15 | PD14 | 1000000 | 24Pin 扩展口 |

通用 UART 参数：

| 参数 | 当前值 |
| --- | --- |
| Mode | Asynchronous |
| Data Direction | Receive and Transmit |
| Word Length | 8 bits |
| Parity | None |
| Stop Bits | 1 |
| Hardware Flow Control | None |
| Over Sampling | 16 samples |
| FIFO Mode | Disabled |
| GPIO Pull | No pull-up/no pull-down |

配置理由：

- 原 ESP32 工程四路 `HardwareSerial` 都以 1Mbps 初始化。
- 飞特 STS 舵机常用 `8N1` 串口格式。
- 不启用 RTS/CTS，避免占用板上其它复用功能和 USB/CAN 相关脚。
- FIFO 暂时关闭，首版更容易用阻塞或中断方式跑通。后续如果要 DMA + IDLE，可以再评估开启 FIFO 或 DMA。

重要硬件提醒：

飞特 STS 总线是 TTL 半双工总线。CubeMX 这里只完成 MCU UART 配置，不等于物理层已经完整。实际接舵机时还要确认：

- 电平是否匹配
- 是否需要半双工方向控制或外部收发器
- 舵机电源和 MCU 地是否可靠共地
- 总线长度和边沿质量

## USB_OTG_HS 和 USB_DEVICE

### USB_OTG_HS

| 参数 | 当前值 |
| --- | --- |
| External Phy | Disable |
| Internal FS Phy | Device Only |
| Speed | Device Full Speed 12MBit/s |
| Physical interface | Internal Phy |
| Enable internal IP DMA | Disabled |
| Low power | Disabled |
| Link Power Management | Disabled |
| Activate SOF | Disabled |
| VBUS sensing | Disabled |
| USB clock | HSI48 48MHz |

配置理由：

- 虽然外设名叫 `USB_OTG_HS`，但当前使用的是内部 FS PHY，因此显示 Full Speed 12MBit/s 是正确的。
- 板载 USB 使用 `PA11/PA12`，对应 FS DM/DP。
- 不启用 External ULPI PHY，避免错误占用一整组 ULPI 引脚。
- USB DMA 暂时关闭，避免 DCache/MPU/内存一致性问题。
- USB 时钟使用 `HSI48`，独立于 FDCAN 的 PLL1Q。

### USB_DEVICE

| 参数 | 当前值 |
| --- | --- |
| Class For HS IP | Communication Device Class (Virtual Port Com) |
| Max interfaces | 1 |
| Max configurations | 1 |
| CDC Rx Buffer | 2048 bytes |
| CDC Tx Buffer | 2048 bytes |
| Debug level | 0 |

配置理由：

- 上位机协议适合用 USB CDC 虚拟串口承接。
- 2048 字节缓冲能更稳地处理状态包、调试包和突发命令。
- 首版只需要 CDC，不启用 HID/MSC/DFU 等其它类。

生成代码中 `MX_USB_DEVICE_Init()` 位于 `StartDefaultTask()` 内，这符合 CubeMX 的 FreeRTOS 生成方式：RTOS scheduler 启动后再初始化 USB device 栈。

## FreeRTOS

| 参数 | 当前值 |
| --- | --- |
| Interface | CMSIS_V2 |
| FreeRTOS version | 10.3.1 |
| CMSIS-RTOS version | 2.00 |
| `configTICK_RATE_HZ` | 1000 |
| `configTOTAL_HEAP_SIZE` | 131072 bytes |
| Memory scheme | heap_4 |
| Static allocation | Enabled |
| Dynamic allocation | Enabled |
| Mutexes | Enabled |
| Recursive mutexes | Enabled |
| Counting semaphores | Enabled |
| Timers | Enabled |
| Task notifications | Enabled |
| `configUSE_NEWLIB_REENTRANT` | 1 |
| defaultTask stack | 128 words |

配置理由：

- CMSIS_V2 与 CubeMX/ST 中间件集成更顺。
- 1kHz tick 与原工程中毫秒级控制、超时和状态机逻辑更容易对应。
- `heap_4` 支持动态分配和释放，适合中等复杂度任务系统。
- 128KB RTOS heap 用于任务栈、队列、互斥锁、消息缓冲等，比默认 15KB 更适合本项目。
- `configUSE_NEWLIB_REENTRANT=1` 让 `printf/snprintf/strtof` 等 newlib 调用在多任务下更安全。

当前 CubeMX 只生成了 `defaultTask`。后续建议在 `defaultTask` 中进入自己的系统启动函数，再创建真实业务任务。

## NVIC

| 中断 | 当前状态 | 优先级 | 用途 |
| --- | --- | ---: | --- |
| OTG_HS_IRQn | Enabled | 5 | USB device 中断 |
| TIM6_DAC_IRQn | Enabled | 15 | HAL timebase |
| PendSV_IRQn | Enabled | 15 | FreeRTOS 上下文切换 |
| SysTick_IRQn | Enabled | 15 | RTOS tick 相关 |

FreeRTOS 配置：

```c
configLIBRARY_LOWEST_INTERRUPT_PRIORITY = 15
configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5
```

如果某个中断里要调用 `FromISR` 类 FreeRTOS API，中断优先级数值不能高于 5。当前 USB 中断优先级为 5，处在可调用 RTOS API 的边界上。

当前没有启用 UART/FDCAN 的 NVIC 接收中断。首轮空工程编译和基础外设验证可以先这样；正式移植时建议按实际驱动方案增加：

- FDCAN RX FIFO0 new message interrupt
- UART RX interrupt、IDLE interrupt 或 DMA interrupt
- 必要的错误中断

## 未启用外设

| 外设 | 当前策略 |
| --- | --- |
| ETH | 不启用，当前无以太网需求 |
| FDCAN2/FDCAN3 | 不启用，首版只迁移一路 CAN |
| UART8 | 不启用，第 4 路选 UART9 扩展口 |
| I2C1/I2C2/I2C3 | 不启用，首版没有 I2C 传感器需求 |
| SPI/OCTOSPI/SDMMC/FMC | 不启用，降低引脚冲突和初始化复杂度 |
| DMA/MDMA/BDMA | 暂不启用，先完成阻塞/中断 bring-up |

## 生成代码入口顺序

当前 `main()` 初始化顺序：

1. `MPU_Config()`
2. `HAL_Init()`
3. `SystemClock_Config()`
4. `MX_GPIO_Init()`
5. `MX_FDCAN1_Init()`
6. `MX_UART7_Init()`
7. `MX_UART9_Init()`
8. `MX_USART1_UART_Init()`
9. `MX_USART10_UART_Init()`
10. `osKernelInitialize()`
11. 创建 `defaultTask`
12. `osKernelStart()`
13. `StartDefaultTask()` 内执行 `MX_USB_DEVICE_Init()`

这个顺序适合作为移植起点。业务层可以从 `StartDefaultTask()` 的 user code 区域开始接入，也可以在 `USER CODE BEGIN RTOS_THREADS` 位置创建自己的任务。

## 首次硬件验证顺序

建议按下面顺序验证，不要一上来就移植全部业务代码：

1. 空工程编译通过。
2. ST-LINK 能下载，SWD 能连接。
3. 上电后确认 `SystemCoreClock = 480MHz`。
4. USB CDC 能被 Windows/Ubuntu 枚举成虚拟串口。
5. FDCAN1 接 CAN 分析仪或目标节点，确认 1Mbps 标准帧收发。
6. UART7 1Mbps 回环或接舵机测试。
7. USART10 1Mbps 回环或接舵机测试。
8. USART1 1Mbps 回环或接舵机测试。
9. UART9 等 24Pin 扩展板到位后测试。

## 后续移植建议

建议新增 STM32 平台适配层：

| 适配层 | 绑定对象 | 作用 |
| --- | --- | --- |
| `PlatformServoBus` | `UART_HandleTypeDef*` | 统一四路舵机 UART 收发 |
| `PlatformCan` | `FDCAN_HandleTypeDef*` | 标准 CAN 帧收发和过滤 |
| `PlatformUpperLink` | USB CDC | 上位机协议收发 |
| `PlatformTime` | FreeRTOS/HAL tick | 毫秒时间、周期延时 |
| `PlatformLog` | USB CDC 或 SWO | 启动日志和调试输出 |

这样可以把硬件差异集中在平台层，保留原 ESP32 工程中状态机、控制逻辑、协议解析和标定逻辑的大部分结构。
