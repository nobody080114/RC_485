#ifndef MOTION_H
#define MOTION_H

#include <stdbool.h>

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

bool fivebar_forward(float theta1, float theta2, Point2D *P, bool elbow_up);
bool fivebar_inverse(float x, float y,float *theta1,float *theta2,bool elbow_up);
void foot_ellipse_trajectory(float time,FootTrajParam *param,float *x,float *y);
#endif // MOTION_H