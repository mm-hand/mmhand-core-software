/*
 * Final minimal version:
 * - Fixed to SPI-P1 group code.
 * - Output 4 AB states: A0B0 / A0B1 / A1B0 / A1B1.
 * - Keep only read path and diagnostics required for stable use.
 */

#include <SPI.h>
#include <vector>
#include <cstring>

// ==================== Pin mapping ====================
static const int PIN_SPI_SCK  = 8;
static const int PIN_SPI_MISO = 21;
static const int PIN_SPI_MOSI = 10;
static const int PIN_SPI_CS_A = 7;
static const int PIN_SPI_CS_B = 6;

static const int PIN_GRP_D = 13;
static const int PIN_GRP_E = 14;
static const int PIN_GRP_F = 17;
static const int PIN_GRP_G = -1;  // disabled in software, use board default wiring

static const uint8_t GROUP_ENABLE_ACTIVE_LEVEL = LOW;
static const uint8_t P1_GROUP_CODE = 0;  // D/E/F=000
static const bool CS_ACTIVE_LOW = true;

// ==================== Protocol ====================
static const uint8_t CMD_READ = 0xFB;
static const uint16_t LEN_CMD_HEAD = 5;
static const uint16_t LEN_STATUS = 1;
static const uint16_t LEN_CRC = 1;

static const uint16_t TARGET_ADDR = 1008;
static const uint16_t TARGET_LEN = 15;

static const uint32_t SPI_CLOCK_SPEED = 1000000;
static const uint16_t DELAY_CS_HOLD_US = 10;
static const uint16_t DELAY_RESPONSE_US = 30;
static const uint16_t DELAY_GROUP_SETTLE_US = 50;
static const uint16_t DELAY_SENSOR_GAP_MS = 5;
static const uint16_t REFRESH_PERIOD_MS = 1000;

// CRC8 table (Init=0xFF)
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

SPIClass *vspi = nullptr;

uint8_t calculateCRC8(const uint8_t *puchMsg, uint32_t usDataLen) {
    uint8_t uchCRCLo = 0xFF;
    while (usDataLen--) {
        const uint8_t uIndex = uchCRCLo ^ *puchMsg++;
        uchCRCLo = CRC8_TABLE[uIndex];
    }
    return uchCRCLo;
}

bool useSelectFromLineLevel(const bool csActiveLow, const uint8_t lineLevel) {
    return csActiveLow ? (lineLevel == 0) : (lineLevel == 1);
}

void setGroupEnable(const bool enabled) {
    if (PIN_GRP_G < 0) {
        return;
    }
    const uint8_t inactive = (GROUP_ENABLE_ACTIVE_LEVEL == HIGH) ? LOW : HIGH;
    digitalWrite(PIN_GRP_G, enabled ? GROUP_ENABLE_ACTIVE_LEVEL : inactive);
    delayMicroseconds(DELAY_GROUP_SETTLE_US);
}

void setGroupCode(const uint8_t code) {
    digitalWrite(PIN_GRP_D, (code & 0x01) ? HIGH : LOW);
    digitalWrite(PIN_GRP_E, (code & 0x02) ? HIGH : LOW);
    digitalWrite(PIN_GRP_F, (code & 0x04) ? HIGH : LOW);
    delayMicroseconds(DELAY_GROUP_SETTLE_US);
}

void selectP1Group() {
    setGroupCode(P1_GROUP_CODE);
    setGroupEnable(true);
}

bool spiReadRegisters(
    const uint16_t addr,
    const uint16_t readLen,
    uint8_t *userBuffer,
    const bool useCSA,
    const bool useCSB,
    uint8_t *outStatus,
    uint8_t *outRecvCRC,
    uint8_t *outCalcCRC,
    uint8_t *outRaw0,
    uint8_t *outRaw1
) {
    const uint16_t totalLen = LEN_CMD_HEAD + LEN_STATUS + readLen + LEN_CRC;
    std::vector<uint8_t> rawBuffer(totalLen, 0);
    rawBuffer[0] = CMD_READ;
    rawBuffer[1] = (uint8_t)(addr & 0xFF);
    rawBuffer[2] = (uint8_t)(addr >> 8);
    rawBuffer[3] = (uint8_t)(readLen & 0xFF);
    rawBuffer[4] = (uint8_t)(readLen >> 8);

    vspi->beginTransaction(SPISettings(SPI_CLOCK_SPEED, MSBFIRST, SPI_MODE3));
    const uint8_t csSelect = CS_ACTIVE_LOW ? LOW : HIGH;
    const uint8_t csIdle = CS_ACTIVE_LOW ? HIGH : LOW;
    digitalWrite(PIN_SPI_CS_A, useCSA ? csSelect : csIdle);
    digitalWrite(PIN_SPI_CS_B, useCSB ? csSelect : csIdle);
    delayMicroseconds(DELAY_CS_HOLD_US);

    for (int i = 0; i < LEN_CMD_HEAD; i++) {
        vspi->transfer(rawBuffer[i]);
    }

    delayMicroseconds(DELAY_RESPONSE_US);
    for (int i = LEN_CMD_HEAD; i < totalLen; i++) {
        rawBuffer[i] = vspi->transfer(0x00);
    }

    delayMicroseconds(DELAY_CS_HOLD_US);
    digitalWrite(PIN_SPI_CS_A, csIdle);
    digitalWrite(PIN_SPI_CS_B, csIdle);
    vspi->endTransaction();

    const uint8_t calcCRC = calculateCRC8(rawBuffer.data(), totalLen - 1);
    const uint8_t recvCRC = rawBuffer[totalLen - 1];
    const uint8_t status = rawBuffer[LEN_CMD_HEAD];
    const uint8_t raw0 = rawBuffer[LEN_CMD_HEAD];
    const uint8_t raw1 = (LEN_CMD_HEAD + 1 < totalLen) ? rawBuffer[LEN_CMD_HEAD + 1] : 0;

    if (outStatus) *outStatus = status;
    if (outRecvCRC) *outRecvCRC = recvCRC;
    if (outCalcCRC) *outCalcCRC = calcCRC;
    if (outRaw0) *outRaw0 = raw0;
    if (outRaw1) *outRaw1 = raw1;

    if (calcCRC != recvCRC) {
        return false;
    }

    memcpy(userBuffer, &rawBuffer[LEN_CMD_HEAD + LEN_STATUS], readLen);
    return true;
}

void printForceData(const uint8_t *data, const uint16_t len) {
    if (len >= 3) {
        const int8_t fx = (int8_t)data[0];
        const int8_t fy = (int8_t)data[1];
        const uint8_t fz = data[2];
        Serial.printf("Fx:%+4d Fy:%+4d Fz:%3u | ", fx, fy, fz);
        for (int i = 3; i < len; i++) {
            Serial.printf("%02X ", data[i]);
        }
        return;
    }
    for (int i = 0; i < len; i++) {
        Serial.printf("%02X ", data[i]);
    }
}

void printConfig() {
    Serial.println("[Config] Group mapping (name => D/E/F code):");
    Serial.printf("  P1 => code %u (D/E/F=%u%u%u)\n",
                  P1_GROUP_CODE,
                  (P1_GROUP_CODE >> 2) & 1, (P1_GROUP_CODE >> 1) & 1, P1_GROUP_CODE & 1);
    if (PIN_GRP_G < 0) {
        Serial.println("  G pin is disabled in code (PIN_GRP_G = -1).");
    } else {
        Serial.printf("  G pin active level = %s\n",
                      (GROUP_ENABLE_ACTIVE_LEVEL == LOW) ? "LOW" : "HIGH");
    }
    Serial.printf("[Config] GPIO map: SCK=%d MISO=%d MOSI=%d | CS_A=%d CS_B=%d | D=%d E=%d F=%d G=%d | CSactive=%s\n",
                  PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI,
                  PIN_SPI_CS_A, PIN_SPI_CS_B,
                  PIN_GRP_D, PIN_GRP_E, PIN_GRP_F, PIN_GRP_G,
                  CS_ACTIVE_LOW ? "LOW" : "HIGH");
    Serial.println("[Mode] P1 AB tune: A0B0 / A0B1 / A1B0 / A1B1");
}

void runP1ABTuneFrame(const uint32_t frameCount) {
    static const uint8_t AB_LINES[4][2] = {
        {0, 0}, {0, 1}, {1, 0}, {1, 1}
    };
    uint8_t recvBuf[TARGET_LEN];

    selectP1Group();

    Serial.println();
    Serial.println("================================================================"
                  "========================================");
    Serial.printf("  P1 AB Tune Monitor  |  Frame #%lu  |  %lu ms\n", frameCount, millis());
    Serial.println("================================================================"
                  "========================================");
    Serial.printf("  [Group P1] code=%u (D/E/F=%d%d%d)\n",
                  P1_GROUP_CODE,
                  (P1_GROUP_CODE >> 2) & 1, (P1_GROUP_CODE >> 1) & 1, P1_GROUP_CODE & 1);
    Serial.println("  ----------------------------------------------------------------"
                  "------------------------------");
    Serial.printf("  %-10s | %-6s | %-27s | %s\n",
                  "STATE", "STATUS", "FORCE (Fx/Fy/Fz)", "RAW TAIL");
    Serial.println("  ----------------------------------------------------------------"
                  "------------------------------");

    for (int i = 0; i < 4; i++) {
        const uint8_t aLine = AB_LINES[i][0];
        const uint8_t bLine = AB_LINES[i][1];
        const bool useCSA = useSelectFromLineLevel(CS_ACTIVE_LOW, aLine);
        const bool useCSB = useSelectFromLineLevel(CS_ACTIVE_LOW, bLine);

        memset(recvBuf, 0, sizeof(recvBuf));
        uint8_t st = 0, recvCRC = 0, calcCRC = 0, raw0 = 0, raw1 = 0;
        const bool ok = spiReadRegisters(
            TARGET_ADDR,
            TARGET_LEN,
            recvBuf,
            useCSA,
            useCSB,
            &st, &recvCRC, &calcCRC, &raw0, &raw1
        );

        Serial.printf("  A%uB%u      | ", aLine, bLine);
        if (ok) {
            Serial.print("\033[32m OK \033[0m  | ");
            printForceData(recvBuf, TARGET_LEN);
        } else {
            Serial.print("\033[31mFAIL\033[0m  | ");
            Serial.print("---  ---  ---             | -- -- --");
            Serial.printf("  [st=%02X crc=%02X/%02X raw=%02X,%02X]",
                          st, recvCRC, calcCRC, raw0, raw1);
        }
        Serial.println();
        delay(DELAY_SENSOR_GAP_MS);
    }

    setGroupEnable(false);
    Serial.println("================================================================"
                  "========================================");
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n[NewBoard] GEN3 tactile reader init...");

    pinMode(PIN_GRP_D, OUTPUT);
    pinMode(PIN_GRP_E, OUTPUT);
    pinMode(PIN_GRP_F, OUTPUT);
    if (PIN_GRP_G >= 0) {
        pinMode(PIN_GRP_G, OUTPUT);
        setGroupEnable(false);
    }

    pinMode(PIN_SPI_CS_A, OUTPUT);
    pinMode(PIN_SPI_CS_B, OUTPUT);
    digitalWrite(PIN_SPI_CS_A, CS_ACTIVE_LOW ? HIGH : LOW);
    digitalWrite(PIN_SPI_CS_B, CS_ACTIVE_LOW ? HIGH : LOW);

    vspi = new SPIClass(FSPI);
    vspi->begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);

    setGroupCode(P1_GROUP_CODE);
    printConfig();
    Serial.println("[NewBoard] Init done.");
}

void loop() {
    static unsigned long lastRun = 0;
    static uint32_t frameCount = 0;
    if (millis() - lastRun <= REFRESH_PERIOD_MS) {
        return;
    }
    lastRun = millis();
    frameCount++;
    runP1ABTuneFrame(frameCount);
}

