#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include <Arduino.h>
#include "TaskSharedData.h"

void controlTask(void* parameter);
void taskSolver(void* parameter);

#endif // CONTROL_TASK_H
