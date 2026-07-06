#ifndef MOTION_H
#define MOTION_H

#include "main.h"
#include <math.h>
#include <stdbool.h>
#define TWO_PI (2.0f * PI)
#define L1 0.100f // m
#define L2 0.200f // m
// #define TURN_INNER_STEP_RATIO 0.55f
// #define TURN_OUTER_STEP_RATIO 1.00f
typedef struct {
    float x;
    float y;
} Point2D;

typedef struct {
    float J00; float J01;
    float J10; float J11;
} JacobianMatrix;

typedef struct
{
    float step_length;   // 步长 L (m)
    float step_height;   // 抬腿高度 H (m)
    float stand_height;  // 站立高度 y0 (m)
    float period;        // 周期 T (s)
} FootTrajParam;

typedef struct
{
    float rotor_zero;   // 零位转子角
    float output_zero;  // 零位输出角
    float ratio;        // 减速比 (6.33)
    int8_t dir;         // 方向 (+1 / -1)
} JointParam;

// 在全局或静态区定义历史状态变量
typedef struct {
    float x1, x2; // 输入历史
    float y1, y2; // 输出历史
} Filter2ndState;

typedef struct{
    float Fx,Fy;
    float Fx_real,Fy_real;
    float G_0,G_1;
    uint16_t Kp_x,Kd_xt,Kd_xr;
    uint16_t Kp_y,Kd_yt,Kd_yr;
    uint16_t Kp_x_j,Kd_xt_j,Kd_xr_j;
    uint16_t Kp_y_j,Kd_yt_j,Kd_yr_j;
    float output_now_0,output_now_1;
    float raw_vx, raw_vy;
    float filtered_vx, filtered_vy;
    float target_vx, target_vy;
    float now_tau1, now_tau2;
    float target_tau1, target_tau2;
    float omega1, omega2;
    Point2D P,current_P,last_P;
    JacobianMatrix J;
    FootTrajParam track;
    uint8_t motion_state;//0支撑，1抬腿
} Foot_motion;

bool fivebar_forward(float theta1, float theta2, Point2D *P, bool elbow_up);
bool fivebar_inverse(float x, float y,float *theta1,float *theta2,bool elbow_up);
void foot_ellipse_trajectory(float time,Foot_motion *param,FootTrajParam *traj);
void foot_ellipse_trajectory_dir(float time, Foot_motion *param, FootTrajParam *traj, float x_dir);
float output_to_rotor(float theta_out, JointParam *param);
float rotor_to_output(float rotor_now, JointParam *param);
void wrap_pi_fast(float *angle);
float output_to_rotor_stand(float theta_out, JointParam *param);
bool fwd_kinematics_and_jacobian(float theta1, float theta2, bool elbow_up, 
                                 Point2D *P, JacobianMatrix *J);
float apply_2nd_order_lpf(float input, Filter2ndState *s);
void estimate_foot_force(float tau1, float tau2, JacobianMatrix *J, float *Fx_real, float *Fy_real);
void set_left_right_step_length(float left_step, float right_step);
void nav_update_step_length(int16_t go_speed_cm_s, int16_t switch_speed_cm_s, float period, float *inner_ratio);
void apply_curve_step_length(uint8_t dir, float base_step,float inner_ratio);
void jump_reset(void);
void jump_update(float dt);

extern int8_t jump_start_req,jump_armed;
extern float jump_ff_x,jump_ff_y;

#endif // MOTION_H
