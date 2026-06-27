#ifndef STATE_MACHINE_TASK_H
#define STATE_MACHINE_TASK_H

#include <Arduino.h>
#include "../shared/TaskSharedData.h"

// StateMachineTask is the single authority for system state transitions.
// Host commands should enter this task as events instead of directly changing
// control_enabled, servo_target_owner, or system_state.
// Runtime faults are handled like an E-STOP: clear output ownership, request
// emergency hold, and return to UNCONTROLLABLE.
bool postSystemEvent(TaskSharedData_t* sharedData, uint8_t event);
bool postSystemEventForToken(TaskSharedData_t* sharedData, uint8_t event, uint32_t token);
void stateMachineTask(void* parameter);

#endif // STATE_MACHINE_TASK_H
