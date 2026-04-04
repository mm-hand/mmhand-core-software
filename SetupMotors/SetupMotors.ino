#include <SCServo.h>

SMS_STS sms_sts;  // Create SMS_STS object
bool isRunning = true;  // Servo running status flag
int currentServoID = -1; // Variable to store the found servo ID

// Pin definitions
// int Uart1RxPin = 20; // Test
// int Uart1TxPin = 21;
int Uart1RxPin = 21;
int Uart1TxPin = 20;

void setup() {
  Serial.begin(115200);  // Initialize serial communication for status reporting
  Serial1.begin(1000000, SERIAL_8N1, Uart1RxPin, Uart1TxPin); // Initialize servo communication with specified pins
  sms_sts.pSerial = &Serial1; // Set servo communication port
  
  // Wait until serial communication is set up
  while (!Serial) {
    ;
  }
  delay(1000); // Wait for system stabilization
  
  // Scan for servo IDs from 0 to 253
  Serial.println("Scanning for servos...");
  for (int id = 0; id <= 253; id++) {
    int response = sms_sts.Ping(id);
    if (!sms_sts.getLastError()) {
      currentServoID = id;
      Serial.print("Found servo with ID: ");
      Serial.println(currentServoID);
      break;
    }
    delay(50); // Short delay between pings
  }
  
  // If no servo found, wait for user input
  if (currentServoID == -1) {
    Serial.println("No servo found. Please check connections and restart.");
    while(1); // Stop execution
  }
  
  // Prompt user for new ID
  while (Serial.available() > 0) {
    Serial.read(); // 清空缓冲区
  }
  Serial.println("Please set the baud rate to 115200 in the Arduino IDE Serial Monitor");
  Serial.println("Then enter a new ID for the servo (0-253):");
  
  // Wait for user input
  while (Serial.available() == 0) {
    delay(100);; // Wait for input
  }
  
  // Read user input
  String input = Serial.readString();
  input.trim(); // Remove whitespace
  int newID = input.toInt();
  
  // Validate input
  if (newID < 0 || newID > 253) {
    Serial.println("Error: ID must be between 0 and 253. Using found ID instead.");
  } else {
    // Change servo ID
    if (changeServoID(currentServoID, newID)) {
      currentServoID = newID;
    }
  }
  
  // Verify the new ID by scanning again
  Serial.println("Verifying new servo ID...");
  bool found = false;
  for (int id = 0; id <= 253; id++) {
    int response = sms_sts.Ping(id);
    if (!sms_sts.getLastError()) {
      found = true;
      break;
    }
    delay(10); // Short delay between pings
  }
  
  if (!found) {
    Serial.println("No servo found after ID change. Please check connections.");
    while(1); // Stop execution
  }
  
  // Set servo to continuous rotation mode (wheel mode)
  if (sms_sts.WheelMode(currentServoID)) {
    Serial.println("Servo set to continuous rotation mode");
  } else {
    Serial.println("Failed to set servo mode");
  }
  
  // Start servo clockwise rotation
  startRotation();
  
  // Print instructions for user
  Serial.println("Servo is now tightening the cable.");
  Serial.println("Send 's' to stop tightening and switch to position mode.");
}

void loop() {
  // Check for user input
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    if (command == 's' || command == 'S') {
      Serial.println("Stop command received.");
      stopRotation();
    }
  }
  
  // Short delay to reduce processing load
  delay(100);
}

// Function to change servo ID
bool changeServoID(int oldID, int newID) {
  // Unlock EPROM
  if (!sms_sts.unLockEprom(oldID)) {
    Serial.println("Failed to unlock EPROM");
    return false;
  }
  
  // Write new ID
  if (!sms_sts.writeByte(oldID, SMS_STS_ID, newID)) {
    Serial.println("Failed to write new ID");
    return false;
  }
  
  // Lock EPROM
  if (!sms_sts.LockEprom(newID)) {
    Serial.println("Failed to lock EPROM");
    return false;
  }
  
  return true;
}

// Start servo clockwise rotation
void startRotation() {
  // Set servo to rotate clockwise at speed 60 (acceleration 50)
  if (sms_sts.WriteSpe(currentServoID, 200, 50)) {
    isRunning = true;
    Serial.println("Servo started clockwise rotation");
  } else {
    Serial.println("Failed to start servo");
  }
}

// Stop servo rotation and switch back to position servo mode
void stopRotation() {
  // Set servo speed to 0 to stop rotation
  if (sms_sts.WriteSpe(currentServoID, 0, 50)) {
    isRunning = false;
    Serial.println("Servo stopped");
    
    // Switch back to position servo mode
    if (sms_sts.ServoMode(currentServoID)) {
      Serial.println("Servo mode changed back to position control");
    } else {
      Serial.println("Failed to change servo mode");
    }
  } else {
    Serial.println("Failed to stop servo");
  }
}