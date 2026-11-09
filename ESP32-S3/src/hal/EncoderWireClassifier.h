#pragma once
#include <stdint.h>

/**
 * @brief 逐指判定 MISO 通断：O=有驱动，N=浮空/没接，S=短地，H=被拉死。
 * @note 一次判定 = 同一位置字在内部上拉/下拉下各读一次；浮空线跟随弱拉。
 */
inline bool wireDriven(uint16_t up, uint16_t down) {
    if (up == 0xFFFFu && down == 0x0000u) {
        return false;  // 跟随弱拉：线上没人驱动
    }
    if (up == 0x0000u && down == 0x0000u) {
        return false;  // 被按在 0
    }
    if (up == 0xFFFFu && down == 0xFFFFu) {
        return false;  // 被按在 1
    }
    return true;
}

/** @brief 一根手指的结论：任一位置有驱动=O；否则 0/0=S，F/F=H，其余=N。 */
inline char wireFingerChar(const uint16_t* up, const uint16_t* down, uint8_t count) {
    bool low = false;
    bool high = false;
    for (uint8_t i = 0; i < count; i++) {
        if (wireDriven(up[i], down[i])) {
            return 'O';
        }
        if (up[i] == 0x0000u && down[i] == 0x0000u) {
            low = true;
        } else if (up[i] == 0xFFFFu && down[i] == 0xFFFFu) {
            high = true;
        }
    }
    return low ? 'S' : high ? 'H' : 'N';
}
