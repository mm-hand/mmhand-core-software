#ifndef STATE_MACHINE_TASK_H
#define STATE_MACHINE_TASK_H

#include <Arduino.h>
#include "TaskSharedData.h"

bool postSystemEvent(TaskSharedData_t* sharedData, uint8_t event);
void stateMachineTask(void* parameter);

#endif // STATE_MACHINE_TASK_H
