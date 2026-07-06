#include "pid.h"


void PosPID_Set(MotorCmd_t *cmd,float kp, float kd)
{
    cmd->K_P = kp;
    cmd->K_W = kd;
}
// void PosPID_UpdateCmd(MotorCmd_t *cmd, float target_pos, PosPID_t *pid)
// {
//     cmd->K_P = pid->kp;
//     cmd->K_W = pid->kd;
// }

