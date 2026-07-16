#ifndef ROPE_INIT_TASK_H
#define ROPE_INIT_TASK_H

#include <Arduino.h>

#include "../shared/TaskSharedData.h"

bool startRopeInitTask(TaskSharedData_t* sharedData);
bool ropeInitActiveScopeReady(TaskSharedData_t* sharedData);
uint8_t ropeInitActiveScopeRejectReason(TaskSharedData_t* sharedData);

#endif // ROPE_INIT_TASK_H
