/*
 * Paxini GEN3 tactile SPI scanner for ESP32-S3.
 *
 * It scans the pin maps that exist in this repository and all group/CS
 * address combinations. Use this before hard-coding a board mapping into
 * ESP32-S3/System.
 */

#include <Arduino.h>
#include <SPI.h>
#include <string.h>

static const uint8_t CMD_READ = 0xFB;
static const uint16_t LEN_CMD_HEAD = 5;
static const uint16_t LEN_STATUS = 1;
static const uint16_t LEN_CRC = 1;
static const uint16_t TARGET_ADDR = 1008;
static const uint16_t TARGET_LEN = 15;
static const uint32_t SPI_CLOCK_HZ = 1000000;

static const int PIN_CS1 = 7;
static const int PIN_CS2 = 6;

static const uint16_t DELAY_CS_HOLD_US = 10;
static const uint16_t DELAY_RESPONSE_US = 100;
static const uint16_t DELAY_GROUP_SETTLE_US = 1000;
static const uint16_t DELAY_BETWEEN_READS_MS = 10;
static const uint16_t SCAN_PERIOD_MS = 2000;

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

struct PinProfile {
    const char *name;
    int sck;
    int miso;
    int mosi;
    int groupA;
    int groupB;
    int groupC;
    int groupEnable;
    uint8_t enableActiveLevel;
};

static const PinProfile PIN_PROFILES[] = {
    {"S3_KNOWN_SPI1_5ROUTE_D13_E14_F17", 8, 21, 10, 13, 14, 17, -1, LOW},
};

struct CsLevel {
    const char *name;
    uint8_t cs1Level;
    uint8_t cs2Level;
};

static const CsLevel CS_LEVELS[] = {
    {"A0B0 active-low", LOW, LOW},
    {"A0B1 active-low", LOW, HIGH},
    {"A1B0 active-low", HIGH, LOW},
    {"A1B1 idle/extra", HIGH, HIGH},
};

SPIClass tactileSpi(FSPI);

uint8_t crc8(const uint8_t *data, uint32_t len) {
    uint8_t crc = 0xFF;
    while (len--) {
        crc = CRC8_TABLE[crc ^ *data++];
    }
    return crc;
}

void setEnable(const PinProfile &p, bool enabled) {
    if (p.groupEnable < 0) {
        return;
    }
    const uint8_t inactive = (p.enableActiveLevel == HIGH) ? LOW : HIGH;
    digitalWrite(p.groupEnable, enabled ? p.enableActiveLevel : inactive);
    delayMicroseconds(DELAY_GROUP_SETTLE_US);
}

void setGroupCode(const PinProfile &p, uint8_t code) {
    digitalWrite(p.groupA, (code & 0x01) ? HIGH : LOW);
    digitalWrite(p.groupB, (code & 0x02) ? HIGH : LOW);
    digitalWrite(p.groupC, (code & 0x04) ? HIGH : LOW);
    delayMicroseconds(DELAY_GROUP_SETTLE_US);
}

bool readOne(
    const CsLevel &cs,
    uint8_t *payload,
    uint8_t *outStatus,
    uint8_t *outRecvCrc,
    uint8_t *outCalcCrc,
    uint8_t *outRaw0,
    uint8_t *outRaw1
) {
    uint8_t raw[LEN_CMD_HEAD + LEN_STATUS + TARGET_LEN + LEN_CRC] = {0};
    const uint16_t totalLen = sizeof(raw);
    raw[0] = CMD_READ;
    raw[1] = (uint8_t)(TARGET_ADDR & 0xFF);
    raw[2] = (uint8_t)(TARGET_ADDR >> 8);
    raw[3] = (uint8_t)(TARGET_LEN & 0xFF);
    raw[4] = (uint8_t)(TARGET_LEN >> 8);

    tactileSpi.beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE3));
    digitalWrite(PIN_CS1, cs.cs1Level);
    digitalWrite(PIN_CS2, cs.cs2Level);
    delayMicroseconds(DELAY_CS_HOLD_US);

    for (uint16_t i = 0; i < LEN_CMD_HEAD; i++) {
        tactileSpi.transfer(raw[i]);
    }
    delayMicroseconds(DELAY_RESPONSE_US);
    for (uint16_t i = LEN_CMD_HEAD; i < totalLen; i++) {
        raw[i] = tactileSpi.transfer(0x00);
    }

    delayMicroseconds(DELAY_CS_HOLD_US);
    digitalWrite(PIN_CS1, HIGH);
    digitalWrite(PIN_CS2, HIGH);
    tactileSpi.endTransaction();

    const uint8_t calc = crc8(raw, totalLen - 1);
    const uint8_t recv = raw[totalLen - 1];
    if (outStatus) {
        *outStatus = raw[LEN_CMD_HEAD];
    }
    if (outRecvCrc) {
        *outRecvCrc = recv;
    }
    if (outCalcCrc) {
        *outCalcCrc = calc;
    }
    if (outRaw0) {
        *outRaw0 = raw[LEN_CMD_HEAD];
    }
    if (outRaw1) {
        *outRaw1 = raw[LEN_CMD_HEAD + 1];
    }

    const bool nonZero = raw[LEN_CMD_HEAD] != 0 || raw[LEN_CMD_HEAD + 1] != 0 || raw[LEN_CMD_HEAD + 2] != 0 || recv != 0;
    if (calc == recv && nonZero) {
        memcpy(payload, raw + LEN_CMD_HEAD + LEN_STATUS, TARGET_LEN);
        return true;
    }
    memset(payload, 0, TARGET_LEN);
    return false;
}

void setupProfile(const PinProfile &p) {
    tactileSpi.end();

    pinMode(p.groupA, OUTPUT);
    pinMode(p.groupB, OUTPUT);
    pinMode(p.groupC, OUTPUT);
    if (p.groupEnable >= 0) {
        pinMode(p.groupEnable, OUTPUT);
        setEnable(p, false);
    }

    pinMode(PIN_CS1, OUTPUT);
    pinMode(PIN_CS2, OUTPUT);
    digitalWrite(PIN_CS1, HIGH);
    digitalWrite(PIN_CS2, HIGH);

    tactileSpi.begin(p.sck, p.miso, p.mosi, -1);
    setEnable(p, true);
}

void scanOnce(uint32_t scanNo) {
    Serial.println();
    Serial.println("============================================================");
    Serial.printf("[SCAN] #%lu millis=%lu addr=%u len=%u spi=%luHz mode=3\n",
                  (unsigned long)scanNo,
                  (unsigned long)millis(),
                  TARGET_ADDR,
                  TARGET_LEN,
                  (unsigned long)SPI_CLOCK_HZ);
    Serial.println("OK rows show: profile | groupCode | CS mode | Fx Fy Fz | crc status");

    uint16_t totalOk = 0;
    uint16_t totalFail = 0;

    for (const PinProfile &profile : PIN_PROFILES) {
        setupProfile(profile);
        Serial.printf("[PROFILE] %s\n", profile.name);

        uint16_t profileOk = 0;
        bool printedFirstFail = false;
        for (uint8_t group = 0; group < 8; group++) {
            setGroupCode(profile, group);
            for (const CsLevel &cs : CS_LEVELS) {
                uint8_t payload[TARGET_LEN] = {0};
                uint8_t status = 0, recvCrc = 0, calcCrc = 0, raw0 = 0, raw1 = 0;
                const bool ok = readOne(cs, payload, &status, &recvCrc, &calcCrc, &raw0, &raw1);
                if (ok) {
                    profileOk++;
                    totalOk++;
                    Serial.printf("  OK  g=%u(%u%u%u) %-22s Fx=%4d Fy=%4d Fz=%4u crc=%02X st=%02X tail=",
                                  group,
                                  (group >> 2) & 1,
                                  (group >> 1) & 1,
                                  group & 1,
                                  cs.name,
                                  (int8_t)payload[0],
                                  (int8_t)payload[1],
                                  payload[2],
                                  recvCrc,
                                  status);
                    for (uint8_t i = 3; i < TARGET_LEN; i++) {
                        Serial.printf("%02X", payload[i]);
                        if (i + 1 < TARGET_LEN) {
                            Serial.print(' ');
                        }
                    }
                    Serial.println();
                } else {
                    totalFail++;
                    if (!printedFirstFail) {
                        printedFirstFail = true;
                        Serial.printf("  first_fail g=%u %-22s st=%02X crc=%02X/%02X raw=%02X,%02X\n",
                                      group,
                                      cs.name,
                                      status,
                                      recvCrc,
                                      calcCrc,
                                      raw0,
                                      raw1);
                    }
                }
                delay(DELAY_BETWEEN_READS_MS);
            }
        }

        Serial.printf("[PROFILE END] %s ok=%u\n", profile.name, profileOk);
        setEnable(profile, false);
    }

    Serial.printf("[SCAN DONE] ok=%u fail=%u\n", totalOk, totalFail);
}

void setup() {
    Serial.begin(921600);
    delay(1000);
    Serial.println("\n[PaxiniScan] GEN3 tactile SPI scanner");
    Serial.println("[PaxiniScan] Manual default selectors: DP=CS2CS1 11, IP=10, CP=01; group code scanned 0..7.");
    Serial.println("[PaxiniScan] Holding all known CS/address pins HIGH for 10s. Power-cycle tactile sensors now if possible.");

    const int selectorPins[] = {2, 4, 5, 6, 7, 13, 14, 17, 18};
    for (int pin : selectorPins) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, HIGH);
    }
    for (int remain = 10; remain > 0; remain--) {
        Serial.printf("[PaxiniScan] SPI mode latch window: %ds\n", remain);
        delay(1000);
    }
    Serial.println("[PaxiniScan] Starting scan.");
}

void loop() {
    static uint32_t lastScan = 0;
    static uint32_t scanNo = 0;
    if (millis() - lastScan < SCAN_PERIOD_MS) {
        return;
    }
    lastScan = millis();
    scanNo++;
    scanOnce(scanNo);
}
