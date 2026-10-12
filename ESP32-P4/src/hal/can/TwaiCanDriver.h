/**
 * @file TwaiCanDriver.h
 * @brief 封装 ESP32-P4 TWAI 控制器及其引脚供电，向业务层提供稳定 CAN 帧接口。
 * @details
 * - 线程：仅由 S3CanRxTask 调用，不在内部创建任务。
 * - 输入：BoardConfig 中的 TX/RX 引脚和 1 Mbit/s 固定总线参数。
 * - 输出：标准 CAN 接收帧、控制器健康状态和恢复结果。
 * - 所有权：本类独占 TWAI 驱动安装、启动、恢复和 VO4 LDO 句柄。
 * - 禁止依赖：modules、tasks、system 业务状态和任何关节/舵机映射。
 */
#ifndef P4_HAL_CAN_TWAI_CAN_DRIVER_H
#define P4_HAL_CAN_TWAI_CAN_DRIVER_H

#include <stdint.h>

struct CanFrame {
    uint32_t identifier;       ///< 11 位标准 CAN 标识符。
    uint8_t dataLength;        ///< 有效数据长度，范围 0..8。
    uint8_t data[8];           ///< CAN 数据区。
};

struct CanControllerStatus {
    uint32_t pendingRxCount;  ///< 驱动接收队列中尚未消费的帧数。
    uint32_t rxMissedCount;    ///< 驱动接收队列溢出累计数。
    uint32_t rxOverrunCount;   ///< 控制器硬件接收溢出累计数。
    uint32_t busErrorCount;    ///< 总线错误累计数。
    uint32_t rxErrorCounter;   ///< CAN 接收错误计数器。
    uint32_t txErrorCounter;   ///< CAN 发送错误计数器。
    uint8_t state;             ///< ESP-IDF TWAI 状态的无依赖数值表示。
    uint8_t valid;             ///< 1 表示本次状态读取成功。
    uint8_t busOff;            ///< 1 表示控制器处于 Bus-Off。
    uint8_t stopped;           ///< 1 表示恢复完成、等待 restart。
};

class TwaiCanDriver {
public:
    TwaiCanDriver();

    /** 幂等初始化板级供电并启动 TWAI。 */
    bool begin();

    /** 非阻塞读取一帧；无数据或驱动不可用时返回 false。 */
    bool tryReceive(CanFrame* outFrame);

    /** 读取驱动健康状态。 */
    CanControllerStatus status() const;

    /** Bus-Off 后请求恢复；恢复完成后可调用 restart()。 */
    bool requestRecovery();

    /** 重新启动已安装的 TWAI 驱动。 */
    bool restart();

private:
    bool ensureIoPower();

    bool installed_;          ///< 驱动已安装。
    bool started_;            ///< 驱动已启动。
    void* ioPowerHandle_;     ///< VO4 LDO 句柄，避免在头文件暴露 ESP-IDF 类型。
};

#endif // P4_HAL_CAN_TWAI_CAN_DRIVER_H
