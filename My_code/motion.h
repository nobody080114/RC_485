#ifndef MOTION_H
#define MOTION_H

#include <stdbool.h>
#include "stm32h7xx_hal.h"
typedef struct {
    float x;
    float y;
} Point2D;

typedef struct
{
    float step_length;   // 步长 L (mm)
    float step_height;   // 抬腿高度 H (mm)
    float stand_height;  // 站立高度 y0 (mm)
    float period;        // 周期 T (s)
} FootTrajParam;

typedef struct
{
    float rotor_zero;   // 零位转子角
    float output_zero;  // 零位输出角
    float ratio;        // 减速比 (6.33)
    int8_t dir;         // 方向 (+1 / -1)
} JointParam;

bool fivebar_forward(float theta1, float theta2, Point2D *P, bool elbow_up);
bool fivebar_inverse(float x, float y,float *theta1,float *theta2,bool elbow_up);
void foot_ellipse_trajectory(float time,FootTrajParam *param,float *x,float *y);
float output_to_rotor(float theta_out, JointParam *param);
#endif // MOTION_H