#include "pid.h"


void PosPID_Init(PosPID_t *pid, float kp, float ki, float kd, float out_limit, float integ_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integ = 0.0f;
    pid->out_limit = out_limit;
    pid->integ_limit = integ_limit;
}

void PosPID_UpdateCmd(MotorCmd_t *cmd, float target_pos, PosPID_t *pid)
{
    float err;
    err = target_pos - cmd->Pos;
    pid->integ += err;
    if((pid->integ > pid->integ_limit)) pid->integ = pid->integ_limit;
    else if((pid->integ < -pid->integ_limit)) pid->integ = -pid->integ_limit;
    cmd->T += pid->ki * pid->integ;
    cmd->K_P = pid->kp;
    cmd->K_W = pid->kd;
}

