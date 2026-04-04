#ifndef SERVO_BUS_MANAGER_H
#define SERVO_BUS_MANAGER_H

#include <Arduino.h>
#include "SMS_STS.h"

// ServoBusManager 头文件职责：
// 1) 声明单总线舵机读写接口；
// 2) 定义舵机反馈结构与多圈位置跟踪字段；
// 3) 提供总线级缓存写入与同步读取能力。

/* ==================== 配置参数 ==================== */

#define NUM_BUSES              4      // 4 路串口总线
#define MAX_SERVOS_PER_BUS     8      // 单条总线最大舵机数
#define MAX_SERVO_ID           32     // 支持的最大舵机ID
#define SERVO_SYNC_READ_TIMEOUT_MS_DEFAULT 3

/* ==================== 舵机反馈数据（支持多圈） ==================== */

struct ServoFeedback {
    // 原始反馈数据（单圈）
    int16_t rawPosition;       // 单圈位置 (0~4096)
    int16_t speed;             // 速度（带符号）
    int16_t load;              // 负载（带符号）
    uint8_t voltage;           // 电压 (0.1V)
    uint8_t temperature;       // 温度 (°C)

    // 多圈跟踪数据
    int32_t absolutePosition;  // 绝对位置（多圈累计，范围 -30719 ~ 30719）
    int16_t turnCount;         // 圈数计数
    int16_t lastRawPosition;   // 上一次原始位置（用于跨圈检测）
    bool    initialized;       // 是否已初始化

    // 状态标志
    bool    online;            // 是否在线
    uint32_t lastUpdate;       // 最后更新时间戳
};

/* ==================== 舵机总线管理器 ==================== */

class ServoBusManager {
public:
    ServoBusManager();

    /* 初始化串口总线 */
    void begin(uint8_t busIndex, int rxPin, int txPin, uint32_t baud = 1000000);

    /* ========== 同步写入（控制） ========== */

    /**
     * @brief 设置舵机目标位置（加入缓存）
     * @param id 舵机ID
     * @param position 目标位置 (-30719 ~ 30719，支持多圈)
     * @param speed 速度
     * @param acc 加速度
     */
    void setTarget(uint8_t id, int16_t position, uint16_t speed = 1000, uint8_t acc = 50);

    /**
     * @brief 同步写入所有缓存的目标位置
     * 调用后会清空缓存
     */
    void syncWriteAll();

    /* ========== 同步读取（反馈） ========== */

    /**
     * @brief 同步读取指定舵机列表的位置（自动进行跨圈检测）
     * @param ids 舵机ID数组
     * @param count 舵机数量
     * @return 成功读取的舵机数量
     */
    int syncReadPositions(const uint8_t* ids, uint8_t count);
    void setSyncReadTimeoutMs(uint16_t timeoutMs);
    uint16_t getSyncReadTimeoutMs() const;

    /**
     * @brief 获取舵机原始位置（单圈）
     * @param id 舵机ID
     * @return 原始位置，失败返回 -1
     */
    int16_t getRawPosition(uint8_t id) const;

    /**
     * @brief 获取舵机绝对位置（多圈累计，-30719 ~ 30719）
     * @param id 舵机ID
     * @return 绝对位置，失败返回 0
     */
    int32_t getAbsolutePosition(uint8_t id) const;

    /**
     * @brief 重置舵机的多圈计数器（将当前位置设为0圈）
     * @param id 舵机ID
     */
    void resetTurnCounter(uint8_t id);

    /**
     * @brief 获取完整的舵机反馈数据
     */
    const ServoFeedback& getFeedback(uint8_t id) const;

    /**
     * @brief 检查舵机是否在线
     */
    bool isOnline(uint8_t id) const;

    /**
     * @brief 直接读取舵机负载（用于校准流程中的阈值判断）
     * @param id 舵机ID
     * @return 负载值，读取失败返回 0
     */
    int16_t readLoad(uint8_t id);

private:
    SMS_STS _sms;                    // 飞特舵机协议对象
    HardwareSerial* _serial;         // 串口指针

    /* 同步写入缓存 */
    uint8_t  _writeIDs[MAX_SERVOS_PER_BUS];
    int16_t  _writePos[MAX_SERVOS_PER_BUS];
    uint16_t _writeSpd[MAX_SERVOS_PER_BUS];
    uint8_t  _writeAcc[MAX_SERVOS_PER_BUS];
    uint8_t  _writeCount;

    /* 反馈数据缓存 */
    ServoFeedback _feedback[MAX_SERVO_ID + 1];
    uint16_t _syncReadTimeoutMs;

    /* 内部辅助函数 */
    void _updateMultiTurnPosition(uint8_t id, int16_t newRawPos);
};

#endif
