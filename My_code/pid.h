#ifndef PID_H
#define PID_H

#include "gom_protocol.h"

typedef struct
{
	float kp;
	float ki;
	float kd;
	float integ;
	float out_limit;
	float integ_limit;
} PosPID_t;

void PosPID_Init(PosPID_t *pid, float kp, float ki, float kd, float out_limit, float integ_limit);
void PosPID_UpdateCmd(MotorCmd_t *cmd, float target_pos, PosPID_t *pid);

#endif