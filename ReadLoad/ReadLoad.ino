#include <SCServo.h>

// 定义常量
#define numServos 4
#define servoIDS {0, 1, 2, 3}
#define UART1_rxPin 20
#define UART1_txPin 21

// 创建舵机对象
SMS_STS sms_sts;

// 舵机ID数组
byte servoIDs[] = servoIDS;

void setup() {
  // 初始化串口通信
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, UART1_rxPin, UART1_txPin);
  
  // 设置舵机通信接口
  sms_sts.pSerial = &Serial1;
  
  // 等待串口初始化
  delay(1000);
  Serial.println("开始初始化舵机...");
  
  // 使能所有舵机并保持当前位置
  for(int i = 0; i < numServos; i++) {
    // 先使能舵机
    sms_sts.EnableTorque(servoIDs[i], 1);
    delay(10);
    
    // 读取当前位置
    int currentPos = sms_sts.ReadPos(servoIDs[i]);
    
    if(sms_sts.getLastError()) {
      Serial.print("读取舵机 ");
      Serial.print(servoIDs[i]);
      Serial.println(" 当前位置失败");
    } else {
      Serial.print("舵机 ");
      Serial.print(servoIDs[i]);
      Serial.print(" 当前位置: ");
      Serial.println(currentPos);
      
      // 设置舵机转动到当前位置（维持不变）
      // 使用适当的速度参数，这里设为50（根据舵机型号调整）
      // sms_sts.WritePosEx(servoIDs[i], currentPos, 50, 0);
      
      if(sms_sts.getLastError()) {
        Serial.print("设置舵机 ");
        Serial.print(servoIDs[i]);
        Serial.println(" 位置失败");
      } else {
        Serial.print("舵机 ");
        Serial.print(servoIDs[i]);
        Serial.println(" 已设置为保持当前位置");
      }
    }
    delay(10); // 短暂延迟
  }
  
  Serial.println("初始化完成，开始读取舵机负载...");
}

void loop() {
  // 读取并打印每个舵机的负载
  for(int i = 0; i < numServos; i++) {
    int load = sms_sts.ReadLoad(servoIDs[i]);
    
    if(sms_sts.getLastError()) {
      Serial.print("读取舵机 ");
      Serial.print(servoIDs[i]);
      Serial.println(" 负载失败");
    } else {
      Serial.print("舵机 ");
      Serial.print(servoIDs[i]);
      Serial.print(" 负载: ");
      Serial.println(load);
    }
  }
  
  Serial.println("-------------------");
  
  // 短暂延迟
  delay(100);
}