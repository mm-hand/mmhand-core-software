/**
 * @file DeviceIdentity.h
 * @brief 从芯片 eFuse 读取稳定设备身份；失败返回 0，不猜测设备身份。
 * @details 只读平台接口，不拥有运动许可或持久化状态。
 */
#pragma once
#include <stdint.h>
#include <esp_mac.h>

namespace p4::platform {
inline uint64_t deviceIdentity()
{
    uint8_t mac[6] = {};
    if (esp_efuse_mac_get_default(mac) != ESP_OK) return 0;
    uint64_t value = 0;
    for (uint8_t byte : mac) value = (value << 8U) | byte;
    return value;
}
}
