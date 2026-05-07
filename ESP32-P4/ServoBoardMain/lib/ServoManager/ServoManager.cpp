#include "ServoManager.h"
#define MAX_POSSIBLE_SERVO_NUM 254

ServoManager::ServoManager(uint8_t servoNum) {
    numServos = servoNum;
    servoStates = new ServoState[numServos];
    virtual2ServoID = new uint8_t[numServos];
    servo2VirtualID = new uint8_t[MAX_POSSIBLE_SERVO_NUM];
}

ServoManager::~ServoManager() {
    delete[] servoStates;
    delete[] virtual2ServoID;
    servoStates = nullptr;
    virtual2ServoID = nullptr;
}

void ServoManager::begin(uint8_t busID_, uint8_t rxPin, uint8_t txPin, const uint8_t* servoIDs) {
    // 初始化串口通信
    busID = busID_;
    HardwareSerial* targetSerial = nullptr;
    switch(busID) {
        case 0: targetSerial = &Serial1; break;
        case 1: targetSerial = &Serial2; break;
        case 2: targetSerial = &Serial3; break;
        case 3: targetSerial = &Serial4; break;
        default:
            Serial.println("错误: 无效的总线ID");
            while(1) { ; }
    }
    
    pinMode(rxPin, INPUT_PULLUP); // 防止悬空噪声
    targetSerial->begin(1000000, SERIAL_8N1, rxPin, txPin);
    driver_.pSerial = targetSerial;
    while (!(*targetSerial)) {
        delay(10);
    }

    // 扫描所有可能的ID (0-253)
    Serial.printf("正在扫描总线 %d ...\n", busID);
    uint8_t detectedIDs[MAX_POSSIBLE_SERVO_NUM];
    uint8_t detectedCount = 0;
    for (int i = 0; i < 254; i++) {
        if (driver_.Ping(i) != -1) {
            delay(2); // 连续ping两次，防止虚假应答
            if (driver_.Ping(i) != -1) {
                detectedIDs[detectedCount++] = i;
                Serial.printf("总线 %d 扫描到舵机 ID: %d\n", busID, i);
            }
        }
    }

    // 验证ID匹配
    bool match = (detectedCount >= numServos);
    if (match) {
        // 检查集合是否一致（无视顺序）
        for (uint8_t virtualID = 0; virtualID < numServos; virtualID++) {
            uint8_t expectedID = (servoIDs != nullptr) ? servoIDs[virtualID] : virtualID;
            bool found = false;
            for (uint8_t j = 0; j < detectedCount; j++) {
                if (detectedIDs[j] == expectedID) {
                    found = true;
                    break;
                }
            }
            if (!found) { match = false; break; }
        }
    }

    if (!match) {
        Serial.printf("\n总线 %d 舵机ID不匹配!\n", busID);
        Serial.printf("期望数量: %d, 实际扫描数量: %d\n", numServos, detectedCount);
        
        Serial.print("期望ID列表: ");
        for(uint8_t virtualID = 0; virtualID < numServos; virtualID++) {
            Serial.printf("%d ", (servoIDs ? servoIDs[virtualID] : virtualID));
        }
        
        Serial.print("\n实际ID列表: ");
        for(uint8_t i = 0; i < detectedCount; i++) Serial.printf("%d ", detectedIDs[i]);
        Serial.println("\n\n系统进入死循环...");
        while (1) { delay(100); }
    }
    
    for (uint8_t virtualID = 0; virtualID < numServos; virtualID++) {
        if (servoIDs != nullptr) {
            servoStates[virtualID].id = servoIDs[virtualID];
            virtual2ServoID[virtualID] = servoIDs[virtualID];
            servo2VirtualID[servoIDs[virtualID]] = virtualID;
        } else {
            servoStates[virtualID].id = virtualID;
            virtual2ServoID[virtualID] = virtualID;
            servo2VirtualID[virtualID] = virtualID;
        }
    }

    Serial.println("总线扫描完成，等待指令初始化舵机...");
}

bool ServoManager::InitializeSingleServo(uint8_t servoID, float percent) {
    uint8_t virtualID = servo2VirtualID[servoID];
    ResetServoState(servoStates[virtualID]);
    
    // 1. 启用扭矩
    if (!EnableTorque(servoID, true)) {
        delay(50);
        if (!EnableTorque(servoID, true)) {
            return false;
        }
    }

    // 2. 设置多圈模式
    if (!SetMultiTurnMode(servoID)) {
        delay(50);
        if (!SetMultiTurnMode(servoID)) {
            return false;
        }
    }
    
    // TODO: 暂时假定已经标定好了中心位置
    if (!MoveSingleServo(servoID, 2048, 0, 0)) {
        delay(50);
        if (!MoveSingleServo(servoID, 2048, 0, 0)) {
            return false;
        }
    }
    delay(1000);

    // 3. 校准中心位置
    if (!CalibrateCenter(servoID)) {
        delay(50);
        if (!CalibrateCenter(servoID)) {
            return false;
        }
    }
    
    // 4. 寻找极限位置
    if (!FindLimitPosition(servoID)) {
        return false;
    }

    // 5. 根据百分比移动到指定位置 (2048 ~ limitPos)
    int16_t targetPos = 2048 + (int16_t)((servoStates[virtualID].limitPos - 2048) * percent);
    Serial.printf("初始化完成，移动到设定位置: %d (%.1f%%)\n", targetPos, percent * 100);
    MoveSingleServo(servoID, targetPos, 0, 0); 

    return true;
}

bool ServoManager::SetMultiTurnMode(uint8_t servoID) {
    uint8_t virtualID = servo2VirtualID[servoID];

    // 解锁EPROM
    driver_.unLockEprom(servoID);
    servoStates[virtualID].lastCommTime = millis();
    if (int result = driver_.getLastError()) {
        Serial.printf("ServoManager::SetMultiTurnMode 解锁EPROM失败, 通信失败类型 %d \n", result);
        servoStates[virtualID].lastError = ServoError::Communication;
        return false;
    }
    delay(100);
    
    // 设置角度限制为0（多圈模式）
    uint8_t zero_data[4] = {0, 0, 0, 0};
    driver_.genWrite(servoID, SMS_STS_MIN_ANGLE_LIMIT_L, zero_data, 4);
    servoStates[virtualID].lastCommTime = millis();
    if (int result = driver_.getLastError()) {
        Serial.printf("ServoManager::SetMultiTurnMode 校准中心位置失败, 通信失败类型 %d \n", result);
        servoStates[virtualID].lastError = ServoError::Communication;
        return false;
    }
    delay(100);
    
    // 锁定EPROM
    driver_.LockEprom(servoID);
    servoStates[virtualID].lastCommTime = millis();
    if (int result = driver_.getLastError()) {
        Serial.printf("ServoManager::SetMultiTurnMode 锁定EPROM失败, 通信失败类型 %d \n", result);
        servoStates[virtualID].lastError = ServoError::Communication;
        return false;
    }
    delay(100);
    
    return true;
}

bool ServoManager::CalibrateCenter(uint8_t servoID) {
    uint8_t virtualID = servo2VirtualID[servoID];

    Serial.print("校准舵机 ");
    Serial.print(servoID);
    Serial.println(" 当前位置为2048...");
    
    // 1. 解锁EPROM
    driver_.unLockEprom(servoID);
    servoStates[virtualID].lastCommTime = millis();
    if (int result = driver_.getLastError()) {
        Serial.printf("ServoManager::CalibrateCenter 解锁EPROM失败, 通信失败类型 %d \n", result);
        servoStates[virtualID].lastError = ServoError::Communication;
        return false;
    }
    delay(100);
    
    // 2. 发送校准指令
    // driver_.Recal(servoID); // TODO
    servoStates[virtualID].lastCommTime = millis();
    if (int result = driver_.getLastError()) {
        Serial.printf("ServoManager::CalibrateCenter 发送校准指令失败, 通信失败类型 %d \n", result);
        servoStates[virtualID].lastError = ServoError::Communication;
        return false;
    }
    delay(100); // 等待校准完成
    
    // 3. 重新锁定EPROM
    driver_.LockEprom(servoID);
    servoStates[virtualID].lastCommTime = millis();
    if (int result = driver_.getLastError()) {
        Serial.printf("ServoManager::CalibrateCenter 锁定EPROM失败 %d \n", result);
        servoStates[virtualID].lastError = ServoError::Communication;
        return false;
    }
    delay(100);
    
    // 4. 验证校准结果
    int current_pos = driver_.ReadPos(servoID);
    servoStates[virtualID].lastCommTime = millis();
    if (int result = driver_.getLastError()) {
        Serial.printf("ServoManager::CalibrateCenter 读取校准后位置失败 %d \n", result);
        servoStates[virtualID].lastError = ServoError::Communication;
        return false;
    }
    delay(100);
    
    // 5. 更新内部位置状态
    UpdateServoPosLoad(servoStates[virtualID], current_pos, 0);
    
    Serial.print("舵机 ");
    Serial.print(servoID);
    Serial.println(" 位置校准完成（中位2048）");
    
    return true;
}

bool ServoManager::EnableTorque(uint8_t servoID, bool enable) {
    uint8_t virtualID = servo2VirtualID[servoID];
    driver_.EnableTorque(servoID, enable ? 1 : 0);
    servoStates[virtualID].lastCommTime = millis();
    if (int result = driver_.getLastError()) {
        Serial.printf("ServoManager::EnableTorque 对总线%d上的舵机%d配置扭矩为%d失败, 通信失败类型 %d \n", busID, servoID, enable, result);
        servoStates[virtualID].lastError = ServoError::Communication;
        return false;
    }
    delay(100); 
    return true;
}

bool ServoManager::MoveSingleServo(uint8_t servoID, int16_t position, uint16_t speed, uint16_t acc) {
    uint8_t virtualID = servo2VirtualID[servoID];

    if (emergencyStop) {
        Serial.println("ServoManager::MoveSingleServo 已经紧急停机");
        return false;
    }

    driver_.WritePosEx(servoID, position, speed, acc);
    servoStates[virtualID].lastCommTime = millis();
    
    int result = driver_.getLastError();
    if (!result) {
        return true;
    } else {
        servoStates[virtualID].lastError = ServoError::Communication;
        Serial.printf("ServoManager::MoveSingleServo 发送位置控制指令失败, 通信错误类型 %d \n", result);
        return false;
    }
}

bool ServoManager::ReadSinglePosLoad(uint8_t servoID, bool update) {
    uint8_t virtualID = servo2VirtualID[servoID];

    int32_t pos = driver_.ReadPos(servoID);
    int32_t load = driver_.ReadLoad(servoID);
    servoStates[virtualID].lastCommTime = millis();
    if (int result = driver_.getLastError()) {
        Serial.printf("ServoManager::readSinglePoseLoad 舵机 %d 读取指令失败, 通信错误类型 %d \n", servoID, result);
        servoStates[virtualID].lastError = ServoError::Communication;
        delay(500); // 等待舵机自己恢复
        return false;
    }
    else {
        if (update) {
            UpdateServoPosLoad(servoStates[virtualID], pos, load);
        }
        return true;
    }
}

bool ServoManager::FindLimitPosition(uint8_t servoID) {
    uint8_t virtualID = servo2VirtualID[servoID];
    Serial.printf("开始寻找舵机 %d 的极限位置...\n", servoID);
    
    const int STEP_INCREMENT = 50; // 每次增加的位置步长
    unsigned long start_time = millis();
    bool limit_reached = false;
    
    while (millis() - start_time < 50000 && !limit_reached) {
        // 读取和打印当前状态
        if (ReadSinglePosLoad(servoID)) {
            Serial.print("舵机 ");
            Serial.print(servoID);
            Serial.print(" - 当前位置: ");
            Serial.print(servoStates[virtualID].absolutePos);
            Serial.print(" | 负载: ");
            Serial.println(servoStates[virtualID].currentLoad);
            
            // 计算并发送下一步移动指令
            int16_t nextTarget = servoStates[virtualID].absolutePos + STEP_INCREMENT;
            MoveSingleServo(servoID, nextTarget, 120, 0);
            
            // 检查负载是否过大
            if (abs(servoStates[virtualID].currentLoad) > 150) {
                Serial.println("检测到负载过大，记录极限位置");
                servoStates[virtualID].limitPos = servoStates[virtualID].absolutePos;
                limit_reached = true;
            }
        } else {
            // 通信失败，等待后继续尝试
            Serial.printf("读取总线%d上舵机%d失败，正进行下一次尝试\n", busID, servoID);
            delay(100);
        }
        
        delay(100);
    }
    
    if (!limit_reached) {
        Serial.println("寻找超时，使用保守值");
        servoStates[virtualID].limitPos = 2048;
    }
    
    return true;
}

bool ServoManager::SyncMoveServos(const int16_t positions[], const uint16_t speeds[], const uint8_t accs[]) {
    if (emergencyStop) {
        Serial.println("ServoManager::SyncMoveServos 已经急停");
        return false;
    }
    
    // 检查所有舵机状态和位置有效性
    for (uint8_t virtualID = 0; virtualID < numServos; virtualID++) {
        if (positions[virtualID] < 2048 || positions[virtualID] > servoStates[virtualID].limitPos) {
            Serial.printf("ServoManager::SyncMoveServos 舵机%d 目标位置%d 超出限制%d\n", virtual2ServoID[virtualID], positions[virtualID], servoStates[virtualID].limitPos);
            servoStates[virtualID].lastError = ServoError::InvalidPosition;
            return false;
        }
    }
    
    int16_t pos_array[numServos];
    uint16_t speed_array[numServos];
    uint8_t acc_array[numServos];
    
    for (uint8_t virtualID = 0; virtualID < numServos; virtualID++) {
        pos_array[virtualID] = positions[virtualID];
        speed_array[virtualID] = (speeds != nullptr) ? speeds[virtualID] : 0;
        acc_array[virtualID] = (accs != nullptr) ? accs[virtualID] : 0;
    }

    // 同步写入
    driver_.SyncWritePosEx(virtual2ServoID, numServos, pos_array, speed_array, acc_array);
    
    uint32_t now = millis();
    for (uint8_t virtualID = 0; virtualID < numServos; virtualID++) {
        servoStates[virtualID].lastCommTime = now;
    }
    return true;
}

bool ServoManager::SyncReadPosLoad(uint8_t maxRetries, bool update) {
    uint8_t rx_packet[6]; // 位置2+速度2+负载2
    uint8_t retry_count = 0;
    
    while (retry_count <= maxRetries) {
        bool success = true;
        
        // 开始同步读，读取位置和负载（共4字节）
        driver_.syncReadBegin(numServos, sizeof(rx_packet), 500); // 500ms超时
        driver_.syncReadPacketTx(virtual2ServoID, numServos, SMS_STS_PRESENT_POSITION_L, sizeof(rx_packet));
        uint32_t now = millis();
        for (uint8_t virtualID = 0; virtualID < numServos; virtualID++) {
            servoStates[virtualID].lastCommTime = now;
        }
          
        for (uint8_t virtualID = 0; virtualID < numServos; virtualID++) {
            uint8_t servoID = virtual2ServoID[virtualID];
            driver_.syncReadPacketRx(servoID, rx_packet);
            if (int result = driver_.getLastError()) {
                Serial.printf("ServoManager::SyncReadPosLoad 舵机ID %d 读取错误, 总线ID %d, 通信错误为 %d, 重试次数 %d\n", servoID, busID, result, retry_count);
                servoStates[virtualID].lastError = ServoError::Communication;
                success = false;
                break;
            }
            
            // 解析位置数据（2字节）
            int32_t raw_position = (rx_packet[0] | (rx_packet[1] << 8));
            if(raw_position & (1<<15)){
                raw_position = -(raw_position & ~(1<<15));
            }
            
            // 解析负载数据（2字节）
            int32_t raw_load = (rx_packet[4] | (rx_packet[5] << 8));
            if(raw_load & (1<<15)){
                raw_load = -(raw_load & ~(1<<15));
            }
            
            // 更新内部位置和负载状态
            if (update) {
                UpdateServoPosLoad(servoStates[virtualID], raw_position, raw_load);
            }

            // 查看负载是否超限
            if (abs(raw_load) > EMERGENCY_LOAD) {
                Serial.printf("ServoManager::SyncReadPosLoad 总线%d上的舵机%d的负载%d超限, 系统急停\n", busID, servoID, raw_load);
                EmergencyStop();
            }
            
            servoStates[virtualID].lastCommTime = millis();
        }
        driver_.syncReadEnd();
        if (success) {
            return true;
        }
        retry_count++;
        delay(10); // 重试前短暂延迟
    }
    
    // 所有重试都失败，记录错误
    for (uint8_t virtualID = 0; virtualID < numServos; virtualID++) {
        servoStates[virtualID].lastError = ServoError::Communication;
    }
    
    return false;
}

void ServoManager::EmergencyStop() {
    emergencyStop = true;
    for (uint8_t virtualID = 0; virtualID < numServos; virtualID++) {
        EnableTorque(virtual2ServoID[virtualID], false);
    }
}

void ServoManager::Resume() {
    emergencyStop = false;
    for (uint8_t virtualID = 0; virtualID < numServos; virtualID++) {
        EnableTorque(virtual2ServoID[virtualID], true);
    }
}

void ServoManager::GenerateRandomTargets(int16_t targets[]) {
    for (uint8_t virtualID = 0; virtualID < numServos; virtualID++) {
        targets[virtualID] = random(2048, servoStates[virtualID].limitPos);
    }
}

void ServoManager::GenerateReciprocatingTargets(int16_t targets[]) {
    // 需要往复运动的舵机ID
    const uint8_t reciprocatingIDs[] = {1, 2, 3, 5, 6, 7, 9, 10, 11, 13, 14, 15, 17, 18, 19};
    const uint8_t numReciprocating = sizeof(reciprocatingIDs) / sizeof(reciprocatingIDs[0]);
    
    // 根据时间判断方向（10秒一个周期）
    bool isForwardDirection = (millis() / 1000) % 10 == 0;
    
    for (uint8_t virtualID = 0; virtualID < numServos; virtualID++) {
        uint8_t servoID = virtual2ServoID[virtualID];
        
        // 检查是否为往复运动舵机
        bool isReciprocating = false;
        for (uint8_t i = 0; i < numReciprocating; i++) {
            if (servoID == reciprocatingIDs[i]) {
                isReciprocating = true;
                break;
            }
        }
        
        if (isReciprocating) {
            // 往复运动舵机：在2048和中间位置之间切换
            if (isForwardDirection) {
                // 正向：移动到极限位置
                targets[virtualID] = (int16_t)servoStates[virtualID].limitPos;
            } else {
                // 反向：回到中心位置2048
                targets[virtualID] = 2048;
            }
        } else {
            // 非往复运动舵机：始终在中间位置
            targets[virtualID] = 2048 + (int16_t)((servoStates[virtualID].limitPos - 2048) * 0.5f);
        }
    }
}

void ServoManager::ClearErrors() {
    for (uint8_t virtualID = 0; virtualID < numServos; virtualID++) {
        servoStates[virtualID].lastError = ServoError::Success;
    }
}

void ServoManager::UpdateServoPosLoad(ServoState& servo, int32_t newRawPos, int32_t newLoad) {
    if (!servo.initialized) {
        servo.absolutePos = newRawPos;
        servo.initialized = true;
    }
    else {
        int32_t diff = newRawPos - servo.currentRawPos;
        // 检测跨圈
        if (diff < -2048) {
            servo.turns++;  // 顺时针跨圈
        } else if (diff > 2048) {
            servo.turns--;  // 逆时针跨圈
        }
        servo.absolutePos = servo.turns * 4096 + newRawPos;
    }
    servo.currentRawPos = newRawPos;
    servo.currentLoad = newLoad;
}

bool ServoManager::ResetServoState(ServoState& servo) {
    servo.currentRawPos = 2048;
    servo.turns = 0;
    servo.absolutePos = 0;
    servo.initialized = false;
    servo.limitPos = 2048;
    servo.lastError = ServoError::Success;
    servo.lastCommTime = 0;
    return true;
}