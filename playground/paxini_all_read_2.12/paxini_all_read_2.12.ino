/*
 * GEN3 传感器 (1610) ESP32-S3 VSPI 多传感器轮询例程
 * 核心逻辑：XSimple
 * 
 * 硬件连接:
 * SPI_SCK  | GPIO 1
 * SPI_MISO | GPIO 21
 * SPI_MOSI | GPIO 10
 * SPI_CS_A | GPIO 7
 * SPI_CS_B | GPIO 6
 * MUX_A    | GPIO 2
 * MUX_B    | GPIO 4
 * MUX_C    | GPIO 5
 * 
 * MUX 通道映射 (D3~D7):
 *   D3: C=0 B=1 A=1  |  D4: C=1 B=0 A=0
 *   D5: C=1 B=0 A=1  |  D6: C=1 B=1 A=0
 *   D7: C=1 B=1 A=1
 * 
 * 每通道 3 个传感器: CS(A+B), CS(A), CS(B)
 * 总计: 5 组 x 3 = 15 个传感器
 */

#include <SPI.h>
#include <vector>
#include <cstring>

// ==================== 引脚定义 ====================
#define PIN_SPI_SCK   1  
#define PIN_SPI_MISO  21    
#define PIN_SPI_MOSI  10    
#define PIN_SPI_CS_A  7
#define PIN_SPI_CS_B  6

#define PIN_MUX_A     2     
#define PIN_MUX_B     4     
#define PIN_MUX_C     5     

// ==================== 协议常量 ====================
#define CMD_READ      0xFB
#define LEN_CMD_HEAD  5 
#define LEN_STATUS    1 
#define LEN_CRC       1 

#define DELAY_CS_HOLD      10  
#define DELAY_RESPONSE     30  
#define DELAY_MUX_SETTLE   50  // MUX 切换后稳定延时 (us)

static const uint32_t SPI_CLOCK_SPEED = 1000000; 
SPIClass *vspi = NULL;

// CRC8 表 (Init=0xFF)
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

// ==================== MUX 通道定义 ====================
struct MuxChannel {
    const char* name;   // 通道名称
    uint8_t muxVal;     // 3-bit 译码值 (bit2=C, bit1=B, bit0=A)
};

// D3~D7 共 5 个 MUX 通道
MuxChannel muxChannels[] = {
    {"D3", 0b011},  // C=0, B=1, A=1
    {"D4", 0b100},  // C=1, B=0, A=0
    {"D5", 0b101},  // C=1, B=0, A=1
    {"D6", 0b110},  // C=1, B=1, A=0
    {"D7", 0b111},  // C=1, B=1, A=1
};
const int MUX_COUNT = sizeof(muxChannels) / sizeof(MuxChannel);

// 每个 MUX 通道下的 3 个传感器片选组合
struct CSConfig {
    const char* label;
    bool useCSA;
    bool useCSB;
};

CSConfig csConfigs[] = {
    {"DP(A+B)", true,  true },  // 指尖: 双片选
    {"IP(A)",   true,  false},  // 中节: 单A
    {"CP(B)",   false, true },  // 近节: 单B
};
const int CS_COUNT = sizeof(csConfigs) / sizeof(CSConfig);

// ==================== MUX 切换函数 ====================
void setMux(uint8_t val) {
    digitalWrite(PIN_MUX_A, (val & 0x01) ? HIGH : LOW);
    digitalWrite(PIN_MUX_B, (val & 0x02) ? HIGH : LOW);
    digitalWrite(PIN_MUX_C, (val & 0x04) ? HIGH : LOW);
    delayMicroseconds(DELAY_MUX_SETTLE);
}

// ==================== CRC 计算 ====================
uint8_t calculateCRC8(const uint8_t *puchMsg, uint32_t usDataLen) {
    unsigned char uchCRCLo = 0xFF; 
    unsigned uIndex;
    while (usDataLen--) {
        uIndex   = uchCRCLo ^ *puchMsg++;
        uchCRCLo = CRC8_TABLE[uIndex];
    }
    return uchCRCLo;
}

// ==================== SPI 核心读取函数 ====================
bool spiReadRegisters(uint16_t addr, uint16_t readLen, uint8_t *userBuffer, bool useCSA, bool useCSB) {
    uint16_t totalLen = LEN_CMD_HEAD + LEN_STATUS + readLen + LEN_CRC;
    std::vector<uint8_t> rawBuffer(totalLen, 0);

    rawBuffer[0] = CMD_READ;
    rawBuffer[1] = (uint8_t)(addr & 0xFF);
    rawBuffer[2] = (uint8_t)(addr >> 8);
    rawBuffer[3] = (uint8_t)(readLen & 0xFF);
    rawBuffer[4] = (uint8_t)(readLen >> 8);

    vspi->beginTransaction(SPISettings(SPI_CLOCK_SPEED, MSBFIRST, SPI_MODE3));
    
    digitalWrite(PIN_SPI_CS_A, useCSA ? LOW : HIGH);
    digitalWrite(PIN_SPI_CS_B, useCSB ? LOW : HIGH);
    delayMicroseconds(DELAY_CS_HOLD);

    for (int i = 0; i < LEN_CMD_HEAD; i++) {
        vspi->transfer(rawBuffer[i]); 
    }
    delayMicroseconds(DELAY_RESPONSE);

    for (int i = LEN_CMD_HEAD; i < totalLen; i++) {
        rawBuffer[i] = vspi->transfer(0x00);
    }

    delayMicroseconds(DELAY_CS_HOLD);
    digitalWrite(PIN_SPI_CS_A, HIGH);
    digitalWrite(PIN_SPI_CS_B, HIGH);
    vspi->endTransaction();

    uint8_t calculatedCRC = calculateCRC8(rawBuffer.data(), totalLen - 1);
    if (calculatedCRC != rawBuffer[totalLen - 1]) {
        return false;
    }

    if (userBuffer != NULL) {
        memcpy(userBuffer, &rawBuffer[LEN_CMD_HEAD + LEN_STATUS], readLen);
    }
    return true;
}

// ==================== 数据解析: 提取合力 Fx Fy Fz ====================
// 地址 1008 开始读 15 字节: [0]=Fx, [1]=Fy, [2]=Fz (合力), 后续为分布力
void printForceData(uint8_t *data, uint16_t len) {
    if (len >= 3) {
        int8_t fx = (int8_t)data[0];
        int8_t fy = (int8_t)data[1];
        uint8_t fz = data[2];
        Serial.printf("Fx:%+4d Fy:%+4d Fz:%3u | ", fx, fy, fz);
        // 打印剩余原始字节
        for (int i = 3; i < len; i++) {
            Serial.printf("%02X ", data[i]);
        }
    } else {
        for (int i = 0; i < len; i++) {
            Serial.printf("%02X ", data[i]);
        }
    }
}

// ==================== Setup ====================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n[XSimple] GEN3 Multi-Group Sensor Init...");

    // MUX init (默认 D3)
    pinMode(PIN_MUX_A, OUTPUT);
    pinMode(PIN_MUX_B, OUTPUT);
    pinMode(PIN_MUX_C, OUTPUT);
    setMux(0b011);
    
    // CS init
    pinMode(PIN_SPI_CS_A, OUTPUT); digitalWrite(PIN_SPI_CS_A, HIGH);
    pinMode(PIN_SPI_CS_B, OUTPUT); digitalWrite(PIN_SPI_CS_B, HIGH);
    
    // SPI init
    vspi = new SPIClass(FSPI); 
    vspi->begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);

    Serial.printf("[XSimple] Init Done. Polling %d groups x %d sensors = %d total\n", 
                  MUX_COUNT, CS_COUNT, MUX_COUNT * CS_COUNT);
}

// ==================== Loop ====================
void loop() {
    static unsigned long lastRun = 0;
    static uint32_t frameCount = 0;
    
    if (millis() - lastRun > 500) {
        lastRun = millis();
        frameCount++;
        
        uint16_t targetAddr = 1008;
        uint16_t dataLen    = 15;
        uint8_t  recvBuf[15];

        // ===== 打印总表头 =====
        Serial.println();
        Serial.println("================================================================"
                        "========================================");
        Serial.printf("  XSimple GEN3 Sensor Monitor  |  Frame #%lu  |  %lu ms\n", 
                       frameCount, millis());
        Serial.println("================================================================"
                        "========================================");

        // ===== 逐组遍历 =====
        for (int g = 0; g < MUX_COUNT; g++) {
            
            // 切换 MUX 通道
            setMux(muxChannels[g].muxVal);
            
            // 组标题
            uint8_t val = muxChannels[g].muxVal;
            Serial.printf("  [Group %s] MUX=(%d%d%d)  ", 
                          muxChannels[g].name,
                          (val >> 2) & 1, (val >> 1) & 1, val & 1);
            Serial.println();
            Serial.println("  ---------------------------------------------------------------"
                            "-------------------------------");
            Serial.printf("  %-10s | %-6s | %-27s | %s\n", 
                          "SENSOR", "STATUS", "FORCE (Fx/Fy/Fz)", "RAW TAIL");
            Serial.println("  ---------------------------------------------------------------"
                            "-------------------------------");

            // 遍历该组下的 3 个传感器
            for (int s = 0; s < CS_COUNT; s++) {
                memset(recvBuf, 0, dataLen);
                
                bool ok = spiReadRegisters(targetAddr, dataLen, recvBuf,
                                           csConfigs[s].useCSA,
                                           csConfigs[s].useCSB);
                
                // 传感器全局编号: g*3 + s
                Serial.printf("  [%02d]%-5s | ", g * CS_COUNT + s, csConfigs[s].label);
                
                if (ok) {
                    Serial.print("\033[32m OK \033[0m  | ");
                    printForceData(recvBuf, dataLen);
                } else {
                    Serial.print("\033[31mFAIL\033[0m  | ");
                    Serial.print("---  ---  ---             | -- -- --");
                }
                Serial.println();
                
                delay(5); // 传感器间短延时
            }
            Serial.println();
        }

        // ===== 统计尾部 =====
        Serial.println("================================================================"
                        "========================================");
        Serial.printf("  Total: %d groups, %d sensors  |  Next refresh in 500ms\n",
                       MUX_COUNT, MUX_COUNT * CS_COUNT);
        Serial.println("================================================================"
                        "========================================");
    }
}

// /*
//  * GEN3 传感器 (1610) ESP32-S3 VSPI 多传感器轮询例程
//  * 核心逻辑：XSimple
//  * 
//  * 硬件连接 (保持不变):
//  * SPI_SCK  | GPIO 1
//  * SPI_MISO | GPIO 21
//  * SPI_MOSI | GPIO 10
//  * SPI_CS1  | GPIO 7 (CS A)
//  * SPI_CS2  | GPIO 6 (CS B)
//  * MUX 控制 | GPIO 2, 4, 5
//  */

// #include <SPI.h>
// #include <vector>
// #include <cstring>

// // ==================== 引脚定义 ====================
// #define PIN_SPI_SCK   1  
// #define PIN_SPI_MISO  21    
// #define PIN_SPI_MOSI  10    
// #define PIN_SPI_CS_A  7
// #define PIN_SPI_CS_B  6

// // MUX & 其他
// #define PIN_MUX_A     2     
// #define PIN_MUX_B     4     
// #define PIN_MUX_C     5     
// #define PONIXI_UNUSE  7      

// // ==================== 协议常量 ====================
// #define CMD_READ      0xFB
// #define LEN_CMD_HEAD  5 
// #define LEN_STATUS    1 
// #define LEN_CRC       1 

// // 延时参数 (us)
// #define DELAY_CS_HOLD      10  
// #define DELAY_RESPONSE     30  

// // SPI 配置
// static const uint32_t SPI_CLOCK_SPEED = 1000000; 
// SPIClass *vspi = NULL;

// // CRC8 表 (Start=0xFF)
// static const uint8_t CRC8_TABLE[] = {
//     0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
//     0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
//     0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
//     0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
//     0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
//     0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
//     0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
//     0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
//     0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
//     0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
//     0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
//     0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
//     0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
//     0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
//     0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
//     0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
// };

// // ==================== 定义传感器列表 ====================
// struct SensorTarget {
//     const char* name; // 传感器名称/描述
//     bool useCSA;      // 是否拉低 CS_A
//     bool useCSB;      // 是否拉低 CS_B
// };

// // 此处定义3个要读取的传感器组合
// SensorTarget sensorList[] = {
//     {"Dual CS(A+B)", true,  true},  // 组合片选
//     {"Single CS(A)", true,  false}, // 单A
//     {"Single CS(B)", false, true}   // 单B
// };
// const int SENSOR_COUNT = sizeof(sensorList) / sizeof(SensorTarget);

// // ==================== CRC 计算 ====================
// uint8_t calculateCRC8(const uint8_t *puchMsg, uint32_t usDataLen) {
//     unsigned char uchCRCLo = 0xFF; 
//     unsigned uIndex;
//     while (usDataLen--) {
//         uIndex   = uchCRCLo ^ *puchMsg++;
//         uchCRCLo = CRC8_TABLE[uIndex];
//     }
//     return uchCRCLo;
// }

// // ==================== Setup ====================
// void setup() {
//     Serial.begin(115200);
//     delay(1000);
//     Serial.println("\n[XSimple] System Init...");

//     // MUX init
//     pinMode(PIN_MUX_A, OUTPUT); pinMode(PIN_MUX_B, OUTPUT); pinMode(PIN_MUX_C, OUTPUT);
//     digitalWrite(PIN_MUX_A, HIGH); digitalWrite(PIN_MUX_B, HIGH); digitalWrite(PIN_MUX_C, LOW);
    
//     // CS init (默认拉高=未选中)
//     pinMode(PIN_SPI_CS_A, OUTPUT); digitalWrite(PIN_SPI_CS_A, HIGH);
//     pinMode(PIN_SPI_CS_B, OUTPUT); digitalWrite(PIN_SPI_CS_B, HIGH);
    
//     // SPI init
//     vspi = new SPIClass(FSPI); 
//     vspi->begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);
    
//     // Unused
//     pinMode(PONIXI_UNUSE, OUTPUT);
//     digitalWrite(PONIXI_UNUSE, HIGH); 

//     Serial.println("[XSimple] Init Done. Starting Polling...");
// }

// // ==================== SPI 核心读取函数 (修改后) ====================
// // 增加了 useCSA 和 useCSB 参数来动态控制片选
// bool spiReadRegisters(uint16_t addr, uint16_t readLen, uint8_t *userBuffer, bool useCSA, bool useCSB) {
//     uint16_t totalLen = LEN_CMD_HEAD + LEN_STATUS + readLen + LEN_CRC;
//     std::vector<uint8_t> rawBuffer(totalLen, 0);

//     // 填充指令头 (0xFB + Addr + Len)
//     rawBuffer[0] = CMD_READ;
//     rawBuffer[1] = (uint8_t)(addr & 0xFF);
//     rawBuffer[2] = (uint8_t)(addr >> 8);
//     rawBuffer[3] = (uint8_t)(readLen & 0xFF);
//     rawBuffer[4] = (uint8_t)(readLen >> 8);

//     vspi->beginTransaction(SPISettings(SPI_CLOCK_SPEED, MSBFIRST, SPI_MODE3));
    
//     // === 动态片选逻辑 ===
//     // 如果需要选中A，则拉低A；否则拉高
//     digitalWrite(PIN_SPI_CS_A, useCSA ? LOW : HIGH);
//     // 如果需要选中B，则拉低B；否则拉高
//     digitalWrite(PIN_SPI_CS_B, useCSB ? LOW : HIGH);
    
//     delayMicroseconds(DELAY_CS_HOLD);

//     // 发送指令
//     for(int i = 0; i < LEN_CMD_HEAD; i++) {
//         vspi->transfer(rawBuffer[i]); 
//     }

//     delayMicroseconds(DELAY_RESPONSE);

//     // 接收数据
//     for(int i = LEN_CMD_HEAD; i < totalLen; i++) {
//         rawBuffer[i] = vspi->transfer(0x00);
//     }

//     // 结束片选 (全部拉高)
//     delayMicroseconds(DELAY_CS_HOLD);
//     digitalWrite(PIN_SPI_CS_A, HIGH);
//     digitalWrite(PIN_SPI_CS_B, HIGH);
    
//     vspi->endTransaction();

//     // 校验
//     uint8_t calculatedCRC = calculateCRC8(rawBuffer.data(), totalLen - 1);
//     if (calculatedCRC != rawBuffer[totalLen - 1]) {
//         return false;
//     }

//     // 拷贝有效载荷
//     if (userBuffer != NULL) {
//         memcpy(userBuffer, &rawBuffer[LEN_CMD_HEAD + LEN_STATUS], readLen);
//     }
//     return true;
// }

// // ==================== Loop 轮询 ====================
// void loop() {
//     static unsigned long lastRun = 0;
    
//     // 设置循环频率: 500ms
//     if (millis() - lastRun > 500) {
//         lastRun = millis();
        
//         uint16_t targetAddr = 1008; // 目标地址
//         uint16_t dataLen = 15;      // 数据长度
//         uint8_t recvBuf[dataLen];   // 接收缓存

//         // 打印表头 (清屏效果或分割线)
//         Serial.println("\n----------------- Sensor Data Snapshot -----------------");
//         Serial.printf("%-15s | %-8s | %s\n", "TARGET", "STATUS", "DATA PAYLOAD (Hex)");
//         Serial.println("--------------------------------------------------------");

//         // 循环遍历 3 个传感器配置
//         for (int i = 0; i < SENSOR_COUNT; i++) {
//             // 清空缓存
//             memset(recvBuf, 0, dataLen);
            
//             // 调用读取函数，传入对应传感器的 CS 配置
//             bool success = spiReadRegisters(targetAddr, dataLen, recvBuf, 
//                                           sensorList[i].useCSA, 
//                                           sensorList[i].useCSB);
            
//             // 格式化输出
//             Serial.printf("%-15s | ", sensorList[i].name);
            
//             if (success) {
//                 Serial.printf("\033[32mOK      \033[0m | "); // 绿色 OK
//                 for (int j = 0; j < dataLen; j++) {
//                     Serial.printf("%02X ", recvBuf[j]);
//                 }
//             } else {
//                 Serial.printf("\033[31mCRC ERR \033[0m | -- -- --"); // 红色 Error
//             }
//             Serial.println(); // 换行
            
//             // 传感器之间稍作延时，防止干扰
//             delay(10); 
//         }
//         Serial.println("--------------------------------------------------------");
//     }
// }