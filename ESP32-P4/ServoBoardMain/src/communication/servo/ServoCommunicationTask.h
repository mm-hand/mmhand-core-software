#ifndef SERVO_COMMUNICATION_TASK_H
#define SERVO_COMMUNICATION_TASK_H
#include <Arduino.h>
#include "../../shared/TaskSharedData.h"

// Only this task may access the Feetech UART bus directly.
void servoCommunicationTask(void* parameter);

#endif // SERVO_COMMUNICATION_TASK_H
