#include "JointCalibrationProfile.h"

// JointCalibrationProfile 模块职责：
// - 维护“关节索引 -> 标定参数”的静态映射表；
// - 为 CalibrationTask 提供统一的默认/专用标定配置来源；
// - 约定角度相关字段在表内统一以“编码器计数”存储。

// 14 位磁编码器：每圈 16384 计数。
static constexpr float kEncoderCountsPerRev = 16384.0f;

// 角度(度) -> 编码器计数：
// 采用“四舍五入到最近整数”（正数 +0.5，负数 -0.5）以减少系统性偏差。
static constexpr int32_t degToEnc(float deg) {
    return (int32_t)(deg * kEncoderCountsPerRev / 360.0f + (deg >= 0.0f ? 0.5f : -0.5f));
}

// 默认标定配置：
// 供未单独调参的关节使用，作为保守可用的通用参数。
// 字段顺序：
// angleScope, bottomReserved, topReserved, angleReserved,
// loadThreshold, tightenStep, tightenSpeed, tightenAcc, settleMs, maxSearchMs
static const JointCalibrationConfig kDefaultJointCalib = {
    degToEnc(90.0f), // angleScope：可用角度范围（度 -> 计数）
    degToEnc(3.0f),  // bottomReserved：下边界预留（度 -> 计数）
    degToEnc(3.0f),  // topReserved：上边界预留（度 -> 计数）
    degToEnc(6.0f),  // angleReserved：标定完成后回退预留角（度 -> 计数）
    180,   // loadThreshold：负载阈值
    10,    // tightenStep：每步收紧步长
    150,   // tightenSpeed：收紧速度
    10,    // tightenAcc：收紧加速度
    20,    // settleMs：每步下发后等待稳定时间
    5000   // maxSearchMs：单关节搜索超时
};

// Joint 0~3：实机专用调参项（可按实测更新）。
static const JointCalibrationConfig kJoint0Calib = {
    degToEnc(70.0f), degToEnc(3.0f), degToEnc(3.0f), degToEnc(45.0f),
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint1Calib = {
    degToEnc(80.0f), degToEnc(3.0f), degToEnc(3.0f), degToEnc(40.0f),
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint2Calib = {
    degToEnc(95.0f), degToEnc(5.0f), degToEnc(2.0f), degToEnc(45.0f),
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint3Calib = {
    degToEnc(100.0f), degToEnc(5.0f), degToEnc(2.0f), degToEnc(50.0f),
    180, 10, 150, 10, 20, 5000
};

// Joint 4~7：实机专用调参项（可按实测更新）。
static const JointCalibrationConfig kJoint4Calib = {
    degToEnc(70.0f), degToEnc(3.0f), degToEnc(2.0f), degToEnc(45.0f),
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint5Calib = {
    degToEnc(80.0f), degToEnc(3.0f), degToEnc(2.0f), degToEnc(40.0f),
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint6Calib = {
    degToEnc(95.0f), degToEnc(5.0f), degToEnc(2.0f), degToEnc(45.0f),
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint7Calib = {
    degToEnc(100.0f), degToEnc(5.0f), degToEnc(2.0f), degToEnc(50.0f),
    180, 10, 150, 10, 20, 5000
};

// Joint 8~11：实机专用调参项（可按实测更新）。
static const JointCalibrationConfig kJoint8Calib = {
    degToEnc(70.0f), degToEnc(3.0f), degToEnc(2.0f), degToEnc(45.0f),
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint9Calib = {
    degToEnc(80.0f), degToEnc(3.0f), degToEnc(2.0f), degToEnc(40.0f),
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint10Calib = {
    degToEnc(95.0f), degToEnc(5.0f), degToEnc(2.0f), degToEnc(45.0f),
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint11Calib = {
    degToEnc(100.0f), degToEnc(5.0f), degToEnc(2.0f), degToEnc(50.0f),
    180, 10, 150, 10, 20, 5000
};

// Joint 12~15：实机专用调参项（可按实测更新）。
static const JointCalibrationConfig kJoint12Calib = {
    degToEnc(70.0f), degToEnc(3.0f), degToEnc(2.0f), degToEnc(45.0f),
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint13Calib = {
    degToEnc(80.0f), degToEnc(3.0f), degToEnc(2.0f), degToEnc(40.0f),
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint14Calib = {
    degToEnc(95.0f), degToEnc(5.0f), degToEnc(2.0f), degToEnc(45.0f),
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint15Calib = {
    degToEnc(100.0f), degToEnc(5.0f), degToEnc(2.0f), degToEnc(50.0f),
    180, 10, 150, 10, 20, 5000
};

// Joint 16~20：实机专用调参项（可按实测更新）。
static const JointCalibrationConfig kJoint16Calib = {
    degToEnc(70.0f), degToEnc(3.0f), degToEnc(2.0f), degToEnc(45.0f),
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint17Calib = {
    degToEnc(70.0f), degToEnc(3.0f), degToEnc(2.0f), degToEnc(45.0f),
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint18Calib = {
    degToEnc(80.0f), degToEnc(3.0f), degToEnc(2.0f), degToEnc(40.0f),
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint19Calib = {
    degToEnc(95.0f), degToEnc(5.0f), degToEnc(2.0f), degToEnc(45.0f),
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint20Calib = {
    degToEnc(100.0f), degToEnc(5.0f), degToEnc(2.0f), degToEnc(50.0f),
    180, 10, 150, 10, 20, 5000
};


// 标定配置总表：
// - 下标严格对应 jointIndex；
// - 0~7 使用专用参数；
// - 8~20 使用默认参数；
// - 索引与关节号一一对应，不可随意换序。
const JointCalibrationConfig kJointCalibrationProfile[ENCODER_TOTAL_NUM] = {
    // joint 0 ~ 6（专用）
    kJoint0Calib,       // 0
    kJoint1Calib,       // 1
    kJoint2Calib,       // 2
    kJoint3Calib,       // 3
    kJoint4Calib,       // 4
    kJoint5Calib,       // 5
    kJoint6Calib,       // 6

    // joint 7 ~ 13（7 专用，其余默认）
    kJoint7Calib,       // 7
    kDefaultJointCalib, // 8
    kDefaultJointCalib, // 9
    kDefaultJointCalib, // 10
    kDefaultJointCalib, // 11
    kDefaultJointCalib, // 12
    kDefaultJointCalib, // 13

    // joint 14 ~ 20（默认）
    kDefaultJointCalib, // 14
    kDefaultJointCalib, // 15
    kDefaultJointCalib, // 16
    kDefaultJointCalib, // 17
    kDefaultJointCalib, // 18
    kDefaultJointCalib, // 19
    kDefaultJointCalib  // 20
};

// 对外导出 profile 拷贝接口，避免调用方直接依赖内部静态对象布局。
// 将 profile 拷贝到调用方缓存。
// 安全约束：
// - dst 为空时直接返回；
// - count 超过 ENCODER_TOTAL_NUM 时自动截断。
void loadJointCalibrationProfile(JointCalibrationConfig* dst, uint8_t count)
{
    if (!dst) return;

    uint8_t n = count;
    if (n > ENCODER_TOTAL_NUM) n = ENCODER_TOTAL_NUM;

    for (uint8_t i = 0; i < n; i++) {
        dst[i] = kJointCalibrationProfile[i];
    }
}
