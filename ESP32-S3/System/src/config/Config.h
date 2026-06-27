#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <freertos/semphr.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include "driver/twai.h"

// ===========================================
// [System] global parameters
// ===========================================
#define ENCODER_TOTAL_NUM 21
#define TACTILE_GROUP_NUM 5

#ifndef ENABLE_TACTILE_TASK
#define ENABLE_TACTILE_TASK 1
#endif

#ifndef ENC_ACTIVE_GROUP_MASK
#define ENC_ACTIVE_GROUP_MASK 0x1F
#endif

extern uint16_t g_enc_diag[ENCODER_TOTAL_NUM];
extern SemaphoreHandle_t g_sensorSelectMutex;

// ===========================================
// [AS5047P] error bits and latched code ranges
// ===========================================
#define AS5047P_ERR_FRERR    0x0001
#define AS5047P_ERR_INVCOMM  0x0002
#define AS5047P_ERR_PARERR   0x0004

#define ERR_CODE_NONE        0x0000
#define ERR_CODE_LINK_LOST   0xFFFF
#define ERR_CODE_ERRFL_BASE  0x1000
#define ERR_CODE_DIAG_BASE   0x2000

// ===========================================
// [System] encoder state machine
// ===========================================
enum EncoderFSMState {
    FSM_READ_ANGLE = 0,
    FSM_READ_ERRFL = 1,
    FSM_READ_DIAAGC = 2
};

// ===========================================
// [System] data structures
// ===========================================
struct EncoderData {
    uint16_t rawAngles[ENCODER_TOTAL_NUM];
    uint16_t finalAngles[ENCODER_TOTAL_NUM];
    uint16_t errorFlags[ENCODER_TOTAL_NUM];    // 0=OK, 1=error
    uint16_t latchedErrors[ENCODER_TOTAL_NUM]; // detail code
};

struct CheckData {
    uint16_t rawData[ENCODER_TOTAL_NUM];
    uint16_t rawAngles[ENCODER_TOTAL_NUM];
    uint8_t errorFlags[ENCODER_TOTAL_NUM];
    uint8_t parityCheckFlags[ENCODER_TOTAL_NUM];
    uint16_t connectionStatus[ENCODER_TOTAL_NUM]; // 0: OK, 0xFFFF: Lost
};

struct TacIndent1610 {
    uint8_t forces[25][3];
    uint8_t global[3];
    uint8_t rawSummary[15];
};

struct TacIndent2015 {
    uint8_t forces[52][3];
    uint8_t global[3];
    uint8_t rawSummary[15];
};

struct TacGroup {
    TacIndent2015 sensor_A;
    TacIndent1610 sensor_B;
    TacIndent1610 sensor_C;
};

struct TactileData {
    TacGroup groups[TACTILE_GROUP_NUM];
};

struct RemoteCommand {
    uint8_t cmd_type;
    float value;
    bool is_new;
};

// ===========================================
// [Hardware] ESP32-S3 pin map
// ===========================================
// HSPI: encoders (AS5047P)
#define PIN_ENC_MISO    47
#define PIN_ENC_MOSI    38
#define PIN_ENC_SCLK    9
#define PIN_ENC_CS      7

// VSPI: tactile sensors
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

// TWAI (CAN)
#define PIN_TWAI_TX     48
#define PIN_TWAI_RX     1
#define CAN_BAUD_RATE   TWAI_TIMING_CONFIG_1MBITS()

// MUX control pins
#define PIN_MUX_A       2
#define PIN_MUX_B       4
#define PIN_MUX_C       5
#define PIN_MUX_D       13
#define PIN_MUX_E       14
#define PIN_MUX_F       17
#define PIN_MUX_G       18

// ===========================================
// [Encoder HW Compat] board compatibility
// ===========================================
// 138: CS demux topology (default)
// 151: MISO mux topology (temporary board)
#define ENC_HW_MODE_138_CS_DEMUX 0
#define ENC_HW_MODE_151_MISO_MUX 1

#ifndef ENC_HW_MODE
#define ENC_HW_MODE ENC_HW_MODE_138_CS_DEMUX
#endif

#define ENC_MUX_BASE_CHANNEL 0
#define ENC_151_IDLE_CHANNEL 7

// Encoder logical group order follows the assembled finger order:
// group0/J00-J03=pinky(SPI5), group1/J04-J07=ring(SPI4),
// group2/J08-J11=middle(SPI3), group3/J12-J15=index(SPI2),
// group4/J16-J20=thumb(SPI1).
static constexpr uint8_t ENC_138_GROUP_CHANNEL_MAP[5] = {4, 3, 2, 1, 0};

// 151 path keeps the board-specific channel numbers but applies the same
// logical finger order as above.
static constexpr uint8_t ENC_151_GROUP_CHANNEL_MAP[5] = {7, 6, 5, 3, 4};

// 151 diagnostics
// SAFE: on-demand ERRFL only (default)
// FULL: keep ANGLE->ERRFL->DIAAGC flow
#define ENC_151_DIAG_MODE_SAFE 0
#define ENC_151_DIAG_MODE_FULL 1

#ifndef ENC_151_DIAG_MODE
#define ENC_151_DIAG_MODE ENC_151_DIAG_MODE_FULL
#endif

// ===========================================
// [Parameter] tasks and communication
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

// Tactile logical order follows the assembled finger order:
// G0=pinky(SPI5), G1=ring(SPI4), G2=middle(SPI3), G3=index(SPI2), G4=thumb(SPI1).
static constexpr uint8_t TACTILE_GROUP_CHANNEL_MAP[TACTILE_GROUP_NUM] = {0, 1, 2, 3, 4};

#define TASK_ENC_PRIO   10
#define TASK_ENC_CORE   0
#define TASK_TAC_PRIO   5
#define TASK_TAC_CORE   1

#endif // CONFIG_H
