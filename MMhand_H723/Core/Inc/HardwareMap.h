#ifndef HARDWARE_MAP_H
#define HARDWARE_MAP_H

#include "TaskSharedData.h"

#define HAND_PROFILE_V1 1
#define HAND_PROFILE_V2 2

#ifndef HAND_PROFILE
#define HAND_PROFILE HAND_PROFILE_V2
#endif

#ifdef __cplusplus
extern "C" {
#endif

bool isMotorMapItemValid(const MotorMapItem *item);
bool isMotorChannelValid(uint8_t motorChannel);
uint32_t validMotorChannelMask(void);
uint8_t getJointActuatorCount(uint8_t jointIndex);
bool getJointActuator(uint8_t jointIndex, uint8_t actuatorIndex, JointActuatorItem *out);
bool getJointActuatorGroup(uint8_t jointIndex, JointActuatorGroup *out);
int findPrimaryMotorChannelForJoint(uint8_t jointIndex);
int findMotorChannelByBusId(uint8_t busIndex, uint8_t servoID);

#ifdef __cplusplus
}
#endif

#endif /* HARDWARE_MAP_H */
