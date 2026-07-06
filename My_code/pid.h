#ifndef PID_H
#define PID_H

#include "gom_protocol.h"
typedef struct
{
	float kp;
	float kd;
} PosPID_t;
void PosPID_Set(MotorCmd_t *cmd,float kp, float kd);

#endif