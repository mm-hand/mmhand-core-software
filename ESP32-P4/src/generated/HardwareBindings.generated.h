/**
 * @file HardwareBindings.generated.h
 * @brief 原始关节绑定表生成的 P4 拓扑数据，禁止手工编辑。
 */
#ifndef MMHAND_HARDWARE_BINDINGS_GENERATED_H
#define MMHAND_HARDWARE_BINDINGS_GENERATED_H
#include "../data/HardwareTypes.h"
JointMapItem jointMap[ENCODER_TOTAL_NUM] = {{0}, {2}, {3}, {4}, {5}, {7}, {8}, {9}, {10}, {12}, {13}, {14}, {15}, {17}, {18}, {19}, {20}, {22}, {23}, {24}, {25}};
MotorMapItem motorMap[SERVO_TOTAL_NUM] = {{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}, {12}, {13}, {14}, {15}, {16}, {17}, {18}, {19}, {20}, {21}, {22}, {23}, {24}, {25}, {26}};
static constexpr int8_t kMotorTendonShortenAbsSign[SERVO_TOTAL_NUM] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
JointActuatorGroup jointActuatorGroups[ENCODER_TOTAL_NUM] = {
    {2, {{0, 1}, {1, -1}}},
    {1, {{2, 1}, {0xFF, 1}}},
    {1, {{3, 1}, {0xFF, 1}}},
    {1, {{4, 1}, {0xFF, 1}}},
    {2, {{5, 1}, {6, -1}}},
    {1, {{7, 1}, {0xFF, 1}}},
    {1, {{8, 1}, {0xFF, 1}}},
    {1, {{9, 1}, {0xFF, 1}}},
    {2, {{10, 1}, {11, -1}}},
    {1, {{12, 1}, {0xFF, 1}}},
    {1, {{13, 1}, {0xFF, 1}}},
    {1, {{14, 1}, {0xFF, 1}}},
    {2, {{15, 1}, {16, -1}}},
    {1, {{17, 1}, {0xFF, 1}}},
    {1, {{18, 1}, {0xFF, 1}}},
    {1, {{19, 1}, {0xFF, 1}}},
    {2, {{20, 1}, {21, -1}}},
    {1, {{22, 1}, {0xFF, 1}}},
    {1, {{23, 1}, {0xFF, 1}}},
    {1, {{24, 1}, {0xFF, 1}}},
    {2, {{25, 1}, {26, -1}}}
};
#endif // MMHAND_HARDWARE_BINDINGS_GENERATED_H
