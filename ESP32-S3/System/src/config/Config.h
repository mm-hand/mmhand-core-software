#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <freertos/semphr.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include "driver/twai.h"

// ===========================================
// [系统] 全局规模与功能开关
// ===========================================
#define ENCODER_TOTAL_NUM 21
#define TACTILE_GROUP_NUM 5

#ifndef ENABLE_TACTILE_TASK
#define ENABLE_TACTILE_TASK 1
#endif

#ifndef ENC_ACTIVE_GROUP_MASK
#define ENC_ACTIVE_GROUP_MASK 0x1F
#endif

// ===========================================
// [AS5047P] 错误寄存器位及锁存错误码范围
// ===========================================
#define AS5047P_ERR_FRERR    0x0001
#define AS5047P_ERR_INVCOMM  0x0002
#define AS5047P_ERR_PARERR   0x0004

#define ERR_CODE_NONE        0x0000
#define ERR_CODE_LINK_LOST   0xFFFF
#define ERR_CODE_ERRFL_BASE  0x1000
#define ERR_CODE_DIAG_BASE   0x2000

// ===========================================
// [系统] 单路磁编诊断状态机；AS5047P响应具有一拍流水线。
// ===========================================
enum EncoderFSMState {
    FSM_READ_ANGLE = 0,
    FSM_READ_ERRFL = 1,
    FSM_READ_DIAAGC = 2
};

// ===========================================
// [系统] 任务与通信层共享的数据结构
// ===========================================
struct EncoderData {
    uint16_t rawAngles[ENCODER_TOTAL_NUM];     ///< AS5047P 14位原始角度，范围0~16383。
    uint16_t errorFlags[ENCODER_TOTAL_NUM];    ///< 0=本帧有效，1=本帧错误。
    uint16_t latchedErrors[ENCODER_TOTAL_NUM]; ///< 跨帧锁存的详细错误码。
};

struct TacIndent1610 {
    uint8_t forces[25][3]; ///< 25个触元的X/Y/Z三轴原始值。
    uint8_t global[3];     ///< 当前模块的三轴摘要值。
    uint8_t rawSummary[15];///< 预留摘要缓存；当前仅读写前3字节。
};

struct TacIndent2015 {
    uint8_t forces[52][3]; ///< 52个触元的X/Y/Z三轴原始值。
    uint8_t global[3];     ///< 当前模块的三轴摘要值。
    uint8_t rawSummary[15];///< 预留摘要缓存；当前仅读写前3字节。
};

struct TacGroup {
    TacIndent2015 sensor_A; ///< 指腹/较大面积模块，52触元。
    TacIndent1610 sensor_B; ///< 中段模块，25触元。
    TacIndent1610 sensor_C; ///< 近端模块，25触元。
};

struct TactileData {
    TacGroup groups[TACTILE_GROUP_NUM];
};

// ===========================================
// [硬件] ESP32-S3引脚映射
// ===========================================
// 磁编SPI：21路共用MISO/MOSI/SCLK，通过138板CS译码器选择手指组。
#define PIN_ENC_MISO    47
#define PIN_ENC_MOSI    38
#define PIN_ENC_SCLK    9

// 触觉SPI及板级路由选择线。
#define PIN_TAC_MISO    21
#define PIN_TAC_MOSI    10
#define PIN_TAC_SCLK    8
#define PIN_TAC_CS_A    7
#define PIN_TAC_CS_B    6
#define PIN_TAC_GRP_A   13
#define PIN_TAC_GRP_B   14
#define PIN_TAC_GRP_C   17
#define PIN_TAC_GRP_EN  -1
#define PIN_TAC_GRP_EN_ACTIVE_LEVEL LOW

// TWAI/CAN：与ESP32-P4通信，速率1 Mbit/s。
#define PIN_TWAI_TX     48
#define PIN_TWAI_RX     1
#define CAN_BAUD_RATE   TWAI_TIMING_CONFIG_1MBITS()

// 多路选择器控制脚：A/B/C用于磁编，G用于触觉路由使能。
#define PIN_MUX_A       2
#define PIN_MUX_B       4
#define PIN_MUX_C       5
#define PIN_MUX_G       18

// ===========================================
// [磁编硬件] 138板CS译码拓扑
// ===========================================
// 通道7没有连接有效磁编组，事务结束后切到该通道释放全部磁编CS。
#define ENC_DEMUX_IDLE_CHANNEL 7

// 磁编逻辑组顺序：group0小拇指、group1无名指、group2中指、
// group3食指、group4拇指；分别映射J00~J20和PCB上的SPI5~SPI1。
static constexpr uint8_t ENC_GROUP_CHANNEL_MAP[5] = {4, 3, 2, 1, 0};

// ===========================================
// [参数] SPI、触觉数据规模及任务配置
// ===========================================
#define SPI_FREQ_ENC    10000000
#define SPI_FREQ_TAC    1000000
#define TACTILE_SENSOR_PER_GROUP 3
#define TACTILE_AXIS_COUNT 3
#define TACTILE_SUMMARY_BYTES (TACTILE_GROUP_NUM * TACTILE_SENSOR_PER_GROUP * TACTILE_AXIS_COUNT)
#define TACTILE_SUMMARY_NOISE_FLOOR 0
#define TACTILE_SENSOR0_TAXELS 52
#define TACTILE_SENSOR1_TAXELS 25
#define TACTILE_SENSOR2_TAXELS 25
#define TACTILE_PAD_TAXELS TACTILE_SENSOR0_TAXELS
#define TACTILE_TIP_TAXELS TACTILE_SENSOR1_TAXELS
#define TACTILE_FULL_BYTES \
    (TACTILE_GROUP_NUM * (TACTILE_SENSOR0_TAXELS + TACTILE_SENSOR1_TAXELS + TACTILE_SENSOR2_TAXELS) * TACTILE_AXIS_COUNT)

#ifndef ENABLE_TACTILE_CAN_RELAY
#define ENABLE_TACTILE_CAN_RELAY 1
#endif

#ifndef ENABLE_TACTILE_CAN_FULL_DUMP
#define ENABLE_TACTILE_CAN_FULL_DUMP 0
#endif

// 触觉逻辑组顺序与磁编保持一致：G0小拇指到G4拇指。
static constexpr uint8_t TACTILE_GROUP_CHANNEL_MAP[TACTILE_GROUP_NUM] = {0, 1, 2, 3, 4};

// FreeRTOS任务调度配置；SystemTasks创建任务时统一引用，避免配置与执行不一致。
#define TASK_ENC_PRIO   10
#define TASK_ENC_CORE   1
#define TASK_TAC_PRIO   5
#define TASK_TAC_CORE   0
#define TASK_CAN_PRIO   5
#define TASK_CAN_CORE   0

#endif // CONFIG_H
