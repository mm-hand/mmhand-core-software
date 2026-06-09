/*
 * Paxini GEN3 SPI1/P1 single-route test for ESP32-S3.
 *
 * Use this when only one tactile SPI route is connected. It keeps all known
 * selector pins high at boot for protocol-mode latch, then reads only P1
 * with the four A/B line states.
 */

#include <Arduino.h>
#include <SPI.h>
#include <string.h>

static const int PIN_SPI_SCK = 8;
static const int PIN_SPI_MISO = 21;
static const int PIN_SPI_MOSI = 10;
static const int PIN_SPI_CS_A = 7;
static const int PIN_SPI_CS_B = 6;

static const int PIN_GRP_D = 13;
static const int PIN_GRP_E = 14;
static const int PIN_GRP_F = 17;
static const int PIN_GRP_G = -1;

static const uint8_t P1_GROUP_CODE = 0;
static const uint8_t CMD_READ = 0xFB;
static const uint16_t LEN_CMD_HEAD = 5;
static const uint16_t LEN_STATUS = 1;
static const uint16_t LEN_CRC = 1;
static const uint16_t TARGET_ADDR = 1008;
static const uint16_t TARGET_LEN = 15;

static const uint32_t SPI_CLOCK_HZ = 1000000;
static const uint16_t DELAY_CS_HOLD_US = 10;
static const uint16_t DELAY_RESPONSE_US = 30;
static const uint16_t DELAY_GROUP_SETTLE_US = 80;
static const uint16_t DELAY_SENSOR_GAP_MS = 5;
static const uint16_t REFRESH_PERIOD_MS = 500;

static const uint8_t CRC8_TABLE[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
    0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
    0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
    0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
    0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
    0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
    0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
    0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
    0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

SPIClass tactileSpi(FSPI);

uint8_t crc8(const uint8_t *data, uint32_t len) {
    uint8_t crc = 0xFF;
    while (len--) {
        crc = CRC8_TABLE[crc ^ *data++];
    }
    return crc;
}

void idleSelectorsHigh() {
    const int pins[] = {PIN_SPI_CS_A, PIN_SPI_CS_B, PIN_GRP_D, PIN_GRP_E, PIN_GRP_F};
    for (int pin : pins) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, HIGH);
    }
    if (PIN_GRP_G >= 0) {
        pinMode(PIN_GRP_G, OUTPUT);
        digitalWrite(PIN_GRP_G, HIGH);
    }
}

void setP1Group() {
    digitalWrite(PIN_GRP_D, (P1_GROUP_CODE & 0x01) ? HIGH : LOW);
    digitalWrite(PIN_GRP_E, (P1_GROUP_CODE & 0x02) ? HIGH : LOW);
    digitalWrite(PIN_GRP_F, (P1_GROUP_CODE & 0x04) ? HIGH : LOW);
    delayMicroseconds(DELAY_GROUP_SETTLE_US);
}

bool readOne(uint8_t aLine, uint8_t bLine, uint8_t *payload, uint8_t *status, uint8_t *recvCrc, uint8_t *calcCrc) {
    uint8_t raw[LEN_CMD_HEAD + LEN_STATUS + TARGET_LEN + LEN_CRC] = {0};
    const uint16_t totalLen = sizeof(raw);

    raw[0] = CMD_READ;
    raw[1] = (uint8_t)(TARGET_ADDR & 0xFF);
    raw[2] = (uint8_t)(TARGET_ADDR >> 8);
    raw[3] = (uint8_t)(TARGET_LEN & 0xFF);
    raw[4] = (uint8_t)(TARGET_LEN >> 8);

    tactileSpi.beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE3));
    digitalWrite(PIN_SPI_CS_A, aLine ? HIGH : LOW);
    digitalWrite(PIN_SPI_CS_B, bLine ? HIGH : LOW);
    delayMicroseconds(DELAY_CS_HOLD_US);

    for (uint16_t i = 0; i < LEN_CMD_HEAD; i++) {
        tactileSpi.transfer(raw[i]);
    }
    delayMicroseconds(DELAY_RESPONSE_US);
    for (uint16_t i = LEN_CMD_HEAD; i < totalLen; i++) {
        raw[i] = tactileSpi.transfer(0x00);
    }

    delayMicroseconds(DELAY_CS_HOLD_US);
    digitalWrite(PIN_SPI_CS_A, HIGH);
    digitalWrite(PIN_SPI_CS_B, HIGH);
    tactileSpi.endTransaction();

    *status = raw[LEN_CMD_HEAD];
    *recvCrc = raw[totalLen - 1];
    *calcCrc = crc8(raw, totalLen - 1);

    if (*recvCrc != *calcCrc) {
        memset(payload, 0, TARGET_LEN);
        return false;
    }
    memcpy(payload, raw + LEN_CMD_HEAD + LEN_STATUS, TARGET_LEN);
    return true;
}

void printPayload(const uint8_t *payload) {
    Serial.printf("Fx=%4d Fy=%4d Fz=%3u tail=", (int8_t)payload[0], (int8_t)payload[1], payload[2]);
    for (uint8_t i = 3; i < TARGET_LEN; i++) {
        Serial.printf("%02X", payload[i]);
        if (i + 1 < TARGET_LEN) {
            Serial.print(' ');
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println();
    Serial.println("[SPI1-P1] Paxini GEN3 single-route test");
    Serial.printf("[SPI1-P1] SCK=%d MISO=%d MOSI=%d CS_A=%d CS_B=%d D/E/F=%d/%d/%d addr=%u len=%u\n",
                  PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS_A, PIN_SPI_CS_B,
                  PIN_GRP_D, PIN_GRP_E, PIN_GRP_F, TARGET_ADDR, TARGET_LEN);

    idleSelectorsHigh();
    Serial.println("[SPI1-P1] Holding CS/group selector pins HIGH for 10s. Power-cycle tactile sensor power now if needed.");
    for (int remain = 10; remain > 0; remain--) {
        Serial.printf("[SPI1-P1] latch window: %ds\n", remain);
        delay(1000);
    }

    tactileSpi.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);
    setP1Group();
    Serial.println("[SPI1-P1] Start fixed P1 reads. A/B line states are physical line levels; LOW means selected on active-low CS.");
}

void loop() {
    static uint32_t lastRun = 0;
    static uint32_t frame = 0;
    if (millis() - lastRun < REFRESH_PERIOD_MS) {
        return;
    }
    lastRun = millis();
    frame++;

    setP1Group();
    Serial.printf("\n[SPI1-P1] frame=%lu groupCode=%u\n", (unsigned long)frame, P1_GROUP_CODE);

    const uint8_t states[4][2] = {
        {0, 0},
        {0, 1},
        {1, 0},
        {1, 1},
    };

    for (uint8_t i = 0; i < 4; i++) {
        uint8_t payload[TARGET_LEN] = {0};
        uint8_t status = 0;
        uint8_t recv = 0;
        uint8_t calc = 0;
        const uint8_t a = states[i][0];
        const uint8_t b = states[i][1];
        const bool ok = readOne(a, b, payload, &status, &recv, &calc);
        Serial.printf("  A%uB%u %s st=%02X crc=%02X/%02X ",
                      a, b, ok ? "OK  " : "FAIL", status, recv, calc);
        if (ok) {
            printPayload(payload);
        }
        Serial.println();
        delay(DELAY_SENSOR_GAP_MS);
    }
}
