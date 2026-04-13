#ifndef SERVO_MANAGER_H
#define SERVO_MANAGER_H

#include <SMS_STS.h>
#include "ServoState.h"

class ServoManager {
public:
    ServoState* servoStates;       // 舵机状态数组
    bool emergencyStop = false;    // 紧急停止标志
    uint8_t numServos;
    uint8_t busID;                 // 总线标识符

    // 构造函数
    ServoManager(uint8_t servoNum = 1);

    // 析构函数
    ~ServoManager();
    
    // 初始化串口通信 + 舵机极限位置
    void begin(uint8_t busID_, uint8_t rxPin, uint8_t txPin, const uint8_t* servoIDs = nullptr);

    // 初始化单个舵机
    bool InitializeSingleServo(uint8_t servoID, float percent = 0.0);
    
    // 移动单个舵机到指定位置
    bool MoveSingleServo(uint8_t servoID, int16_t position, uint16_t speed = 0, uint16_t acc = 0);

    // 读取舵机当前位置负载
    bool ReadSinglePosLoad(uint8_t servoID, bool update = true);
    
    // 同步移动所有舵机
    bool SyncMoveServos(const int16_t positions[], const uint16_t speeds[] = nullptr, const uint8_t accs[] = nullptr);

    // 同步读取所有舵机位置负载
    bool SyncReadPosLoad(uint8_t maxRetries = 2, bool update = true);
    
    // 停止所有舵机
    void StopAll();
    
    // 紧急停止
    void EmergencyStop();
    
    // 从紧急停止恢复
    void Resume();
    
    // 生成随机目标位置
    void GenerateRandomTargets(int16_t targets[]);

    // 生成往复运动目标位置
    void GenerateReciprocatingTargets(int16_t targets[]);
    
    // 清除所有错误状态
    void ClearErrors();

private:
    SMS_STS driver_; // 舵机驱动对象
    uint8_t* virtual2ServoID; // 从servoStates编号到物理ID
    uint8_t* servo2VirtualID; // 从物理ID到servoStates编号
    
    // 设置多圈模式
    bool SetMultiTurnMode(uint8_t servoID);
    
    // 校准中心位置
    bool CalibrateCenter(uint8_t servoID);

    // 寻找舵机极限位置
    bool FindLimitPosition(uint8_t servoID);

    // 启用/禁用扭矩
    bool EnableTorque(uint8_t servoID, bool enable);
    
    // 更新内部位置状态
    void UpdateServoPosLoad(ServoState& servo, int32_t newRawPos, int32_t newLoad);

    // 重新初始化舵机记录状态
    bool ResetServoState(ServoState& servo);
};

#endif // SERVO_MANAGER_H