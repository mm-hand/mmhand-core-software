# SPI 模式与磁编/触觉连线说明

本文按当前仓库代码整理 ESP32-S3 侧两条 SPI 链路：

- AS5047P 磁编码器链路
- Paxini PX-6AX GEN3 触觉链路

依据代码主要是：

- `ESP32-S3/System/src/config/Config.h`
- `ESP32-S3/System/src/hal/HalEncoders.cpp`
- `ESP32-S3/System/src/hal/HalTactile.cpp`

## 1. SPI mode 是什么意思

SPI 有两根和时序直接相关的线：

- `SCLK`：时钟线，由主机 ESP32 输出。
- `MOSI/MISO`：数据线。数据到底在时钟的哪个边沿被采样，由 SPI mode 决定。

SPI mode 由两个参数组成：

| 参数 | 含义 |
| --- | --- |
| `CPOL` | Clock Polarity，时钟空闲时是低电平还是高电平 |
| `CPHA` | Clock Phase，数据在第一个边沿采样还是第二个边沿采样 |

常见四种模式如下：

| SPI mode | CPOL | CPHA | SCLK 空闲电平 | 常见理解 |
| --- | ---: | ---: | --- | --- |
| Mode 0 | 0 | 0 | 低 | 第一个边沿采样 |
| Mode 1 | 0 | 1 | 低 | 第二个边沿采样 |
| Mode 2 | 1 | 0 | 高 | 第一个边沿采样 |
| Mode 3 | 1 | 1 | 高 | 第二个边沿采样 |

Mode 1 和 Mode 3 的共同点是 `CPHA=1`，也就是都在第二个边沿采样。区别主要是 `CPOL`：

| 项目 | Mode 1 | Mode 3 |
| --- | --- | --- |
| CPOL | 0 | 1 |
| CPHA | 1 | 1 |
| SCLK 空闲状态 | 低电平 | 高电平 |
| 采样边沿 | 通常在下降沿采样 | 通常在上升沿采样 |

所以 Mode 1 和 Mode 3 不能混用。即使 MOSI/MISO/SCLK 三根线接对了，如果 mode 错了，设备可能会读到错位数据、CRC 错误，或者完全无响应。

当前代码中：

- 磁编码器使用 `mode = 1`。
- 触觉使用 `SPI_MODE3`。

Paxini 手册里的 SPI 配置写的是 `CPOL = High`、`CPHA = 2Edge`，对应 Mode 3，因此触觉代码使用 `SPI_MODE3` 是合理的。

## 2. 当前两条 SPI 总线

磁编和触觉是两套不同 SPI 引脚，不共用 SCLK/MOSI/MISO。

| 链路 | SCLK | MOSI | MISO | 代码模式 | 当前实际时钟 |
| --- | ---: | ---: | ---: | --- | ---: |
| AS5047P 磁编码器 | GPIO9 | GPIO38 | GPIO47 | Mode 1 | 1 MHz |
| Paxini 触觉 | GPIO8 | GPIO10 | GPIO21 | Mode 3 | 1 MHz |

注意：`Config.h` 中有 `SPI_FREQ_ENC 10000000`，但当前 `HalEncoders.cpp` 的 `setupSPI()` 实际写死为 `1000 * 1000`，也就是 1 MHz。触觉使用 `SPI_FREQ_TAC 1000000`，也是 1 MHz。

## 3. 磁编码器连线与五组选择

### 3.1 磁编 SPI 基本线

| 功能 | ESP32-S3 引脚 | 代码名 |
| --- | ---: | --- |
| MISO | GPIO47 | `PIN_ENC_MISO` |
| MOSI | GPIO38 | `PIN_ENC_MOSI` |
| SCLK | GPIO9 | `PIN_ENC_SCLK` |
| CS | GPIO7 | `PIN_ENC_CS` |

当前默认模式是：

```cpp
#define ENC_HW_MODE ENC_HW_MODE_138_CS_DEMUX
```

在这个默认模式下，代码没有直接拉低/拉高 `PIN_ENC_CS`，而是通过三根 MUX 选择线选择五组编码器通道。`PIN_ENC_CS = GPIO7` 主要在备用的 `ENC_HW_MODE_151_MISO_MUX` 模式中才被代码作为统一 CS 使用。

### 3.2 磁编 MUX 选择线

| MUX 位 | ESP32-S3 引脚 | 代码名 |
| --- | ---: | --- |
| bit0 | GPIO2 | `PIN_MUX_A` |
| bit1 | GPIO4 | `PIN_MUX_B` |
| bit2 | GPIO5 | `PIN_MUX_C` |

代码设置 MUX 的方式是：

```cpp
digitalWrite(PIN_MUX_A, ch & 0x01);
digitalWrite(PIN_MUX_B, (ch >> 1) & 0x01);
digitalWrite(PIN_MUX_C, (ch >> 2) & 0x01);
```

### 3.3 默认 138 CS demux 模式下的五组磁编

默认模式下，五组磁编使用 MUX 通道 `0..4`。

| 磁编组 | 逻辑关节编号 | MUX 通道 | GPIO5 bit2 | GPIO4 bit1 | GPIO2 bit0 |
| --- | --- | ---: | ---: | ---: | ---: |
| Group 0 | J00-J03 | 0 | 0 | 0 | 0 |
| Group 1 | J04-J07 | 1 | 0 | 0 | 1 |
| Group 2 | J08-J11 | 2 | 0 | 1 | 0 |
| Group 3 | J12-J15 | 3 | 0 | 1 | 1 |
| Group 4 | J16-J20 | 4 | 1 | 0 | 0 |

每组数量来自代码：

```cpp
group_sizes_ = {4, 4, 4, 4, 5}
```

也就是说一共是 `4 + 4 + 4 + 4 + 5 = 21` 路磁编码器。

### 3.4 备用 151 MISO mux 模式

代码还保留了备用模式：

```cpp
#define ENC_HW_MODE_151_MISO_MUX 1
static constexpr uint8_t ENC_151_GROUP_CHANNEL_MAP[5] = {4, 3, 5, 6, 7};
```

如果编译时切换到 `ENC_HW_MODE_151_MISO_MUX`，五组磁编的 MUX 通道会变成：

| 磁编组 | MUX 通道 |
| --- | ---: |
| Group 0 | 4 |
| Group 1 | 3 |
| Group 2 | 5 |
| Group 3 | 6 |
| Group 4 | 7 |

同时该模式会使用 `PIN_ENC_CS = GPIO7` 作为磁编统一 CS。这个模式下 GPIO7 会和触觉的 `PIN_TAC_CS_A` 形成同一引脚复用，必须确保磁编和触觉任务被互斥保护，且每次访问前后都把选择线恢复到安全状态。

## 4. 触觉连线与五组选择

### 4.1 触觉 SPI 基本线

| 功能 | ESP32-S3 引脚 | 代码名 |
| --- | ---: | --- |
| MISO | GPIO21 | `PIN_TAC_MISO` |
| MOSI | GPIO10 | `PIN_TAC_MOSI` |
| SCLK | GPIO8 | `PIN_TAC_SCLK` |

触觉没有使用一个单独的传统 CS，而是用 5 根选择线组合出 pattern。每次读取时，代码先把所有选择线拉高，再写入目标 pattern，然后执行 SPI 读。

### 4.2 触觉 5-bit pattern 对应的 ESP32 引脚

代码里的 pattern 是一个 5 位数。bit 到引脚的对应关系如下：

| pattern 位 | ESP32-S3 引脚 | 代码名 |
| --- | ---: | --- |
| bit0 | GPIO7 | `PIN_TAC_CS_A` |
| bit1 | GPIO6 | `PIN_TAC_CS_B` |
| bit2 | GPIO13 | `PIN_TAC_GRP_A` |
| bit3 | GPIO14 | `PIN_TAC_GRP_B` |
| bit4 | GPIO17 | `PIN_TAC_GRP_C` |

代码设置 pattern 的方式是：

```cpp
digitalWrite(PIN_TAC_CS_A,  (pattern & 0x01) ? HIGH : LOW);
digitalWrite(PIN_TAC_CS_B,  (pattern & 0x02) ? HIGH : LOW);
digitalWrite(PIN_TAC_GRP_A, (pattern & 0x04) ? HIGH : LOW);
digitalWrite(PIN_TAC_GRP_B, (pattern & 0x08) ? HIGH : LOW);
digitalWrite(PIN_TAC_GRP_C, (pattern & 0x10) ? HIGH : LOW);
```

因此代码注释中的 `00011` 是按 `GPIO17 GPIO14 GPIO13 GPIO6 GPIO7` 的顺序书写。比如：

| pattern | GPIO17 | GPIO14 | GPIO13 | GPIO6 | GPIO7 |
| --- | ---: | ---: | ---: | ---: | ---: |
| `00011` | 0 | 0 | 0 | 1 | 1 |
| `01010` | 0 | 1 | 0 | 1 | 0 |
| `10010` | 1 | 0 | 0 | 1 | 0 |

### 4.3 当前五指触觉映射

当前代码启用的是 5 个手指，每个手指 2 个触觉点。第三个触觉点保留在协议格式里，但代码中禁用并填 0。

| 手指 | sensor_A pattern | sensor_B pattern | sensor_C |
| --- | --- | --- | --- |
| Thumb | `00011` | `00010` | disabled |
| Index | `00111` | `00110` | disabled |
| Middle | `01011` | `01010` | disabled |
| Ring | `01111` | `01110` | disabled |
| Pinky | `10011` | `10010` | disabled |

展开到 ESP32 引脚电平如下：

| 手指/点位 | GPIO17 | GPIO14 | GPIO13 | GPIO6 | GPIO7 |
| --- | ---: | ---: | ---: | ---: | ---: |
| Thumb A | 0 | 0 | 0 | 1 | 1 |
| Thumb B | 0 | 0 | 0 | 1 | 0 |
| Index A | 0 | 0 | 1 | 1 | 1 |
| Index B | 0 | 0 | 1 | 1 | 0 |
| Middle A | 0 | 1 | 0 | 1 | 1 |
| Middle B | 0 | 1 | 0 | 1 | 0 |
| Ring A | 0 | 1 | 1 | 1 | 1 |
| Ring B | 0 | 1 | 1 | 1 | 0 |
| Pinky A | 1 | 0 | 0 | 1 | 1 |
| Pinky B | 1 | 0 | 0 | 1 | 0 |

### 4.4 触觉上电时的选择线处理

Paxini 触觉模块在上电时会根据 CS/地址脚电平进入对应通信协议和地址状态。为了避免 ESP32 复位期间 GPIO 浮空导致触觉模块锁存到错误状态，当前代码在启动早期调用：

```cpp
tactile.prepareBootSelectPins();
```

它会把相关选择脚先配置为 `INPUT_PULLUP`，再切为 `OUTPUT HIGH`，并关闭下拉、开启上拉。涉及的脚包括：

| 引脚 | 用途 |
| ---: | --- |
| GPIO2 | 磁编 MUX_A，也被触觉启动保护拉高 |
| GPIO4 | 磁编 MUX_B，也被触觉启动保护拉高 |
| GPIO5 | 磁编 MUX_C，也被触觉启动保护拉高 |
| GPIO18 | `PIN_MUX_G`，触觉启动保护拉高 |
| GPIO7 | 触觉 bit0 / 备用磁编 CS |
| GPIO6 | 触觉 bit1 |
| GPIO13 | 触觉 bit2 / 代码中也命名为 `PIN_MUX_D` |
| GPIO14 | 触觉 bit3 / 代码中也命名为 `PIN_MUX_E` |
| GPIO17 | 触觉 bit4 / 代码中也命名为 `PIN_MUX_F` |

`System.ino` 中启动后等待 20 秒，也是为了给触觉模块电源和上电锁存留窗口。

## 5. 复用关系和注意事项

### 5.1 实际 SPI 数据线不复用

磁编和触觉的 SCLK/MOSI/MISO 是两套不同引脚：

- 磁编：GPIO9/GPIO38/GPIO47
- 触觉：GPIO8/GPIO10/GPIO21

所以它们不会在 SPI 数据线上互相抢总线。

### 5.2 选择线存在命名和潜在功能复用

需要特别注意这些引脚：

| ESP32-S3 引脚 | 当前主要用途 | 复用/备注 |
| ---: | --- | --- |
| GPIO7 | 触觉 pattern bit0 | 备用 `ENC_HW_MODE_151_MISO_MUX` 中也是磁编 CS |
| GPIO13 | 触觉 pattern bit2 | `Config.h` 中也命名为 `PIN_MUX_D` |
| GPIO14 | 触觉 pattern bit3 | `Config.h` 中也命名为 `PIN_MUX_E` |
| GPIO17 | 触觉 pattern bit4 | `Config.h` 中也命名为 `PIN_MUX_F` |
| GPIO2/4/5 | 磁编默认 MUX A/B/C | 触觉启动保护阶段也会拉高 |

当前默认 `ENC_HW_MODE_138_CS_DEMUX` 下，磁编运行期只使用 GPIO2/4/5 做 MUX，触觉运行期使用 GPIO7/6/13/14/17 做 pattern。两者通过 `g_sensorMuxMutex` 互斥，避免同时改选择线。

### 5.3 为什么触觉第三点 disabled

协议和上位机保留的是 `5 fingers x 3 sensors x 3 axes = 45 bytes`。当前硬件实测/接线按 5 个手指、每指 2 个触觉点使用。为了不破坏上位机协议格式，代码保留第三个 slot，但禁用读取并填 0。

这样做的好处是：

- 上位机数据结构不用变。
- CAN/串口 payload 长度不用变。
- 诊断只统计真实启用的 10 个触觉点，不会把不存在的第三点当成故障。

## 6. 快速总表

| 类别 | 信号 | ESP32-S3 引脚 |
| --- | --- | ---: |
| 磁编 SPI | SCLK | GPIO9 |
| 磁编 SPI | MOSI | GPIO38 |
| 磁编 SPI | MISO | GPIO47 |
| 磁编选择 | MUX_A | GPIO2 |
| 磁编选择 | MUX_B | GPIO4 |
| 磁编选择 | MUX_C | GPIO5 |
| 磁编备用 | CS | GPIO7 |
| 触觉 SPI | SCLK | GPIO8 |
| 触觉 SPI | MOSI | GPIO10 |
| 触觉 SPI | MISO | GPIO21 |
| 触觉选择 | bit0 / CS_A | GPIO7 |
| 触觉选择 | bit1 / CS_B | GPIO6 |
| 触觉选择 | bit2 / GRP_A | GPIO13 |
| 触觉选择 | bit3 / GRP_B | GPIO14 |
| 触觉选择 | bit4 / GRP_C | GPIO17 |
