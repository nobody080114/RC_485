/*
    正运动结算
*/
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include "motion.h"
#define L1 100.0f
#define L2 200.0f
#define EPS 1e-6f

double PI_VAL = 3.14159265358979323846;

bool fivebar_forward(float theta1, float theta2, Point2D *P, bool elbow_up);
bool fivebar_inverse(float x, float y,float *theta1,float *theta2,bool elbow_up);
/*
 * @brief  椭圆轨迹足端位置计算
 *
 * @param  time    当前累计时间 (秒)
 * @param  param   轨迹参数
 * @param  x       输出足端 x 坐标 (mm)
 * @param  y       输出足端 y 坐标 (mm)
 *
 *  轨迹说明：
 *  - φ ∈ [0 , π]       → 支撑相（贴地）
 *  - φ ∈ [π , 2π]      → 摆动相（抬腿）
 *
 *  特点：
 *  - x 方向完整余弦
 *  - y 方向只在摆动相抬腿
 *  - 连续、无突变
 */
void foot_ellipse_trajectory(float time,FootTrajParam *param,float *x,float *y)
{
    float omega = 2.0f * PI_VAL / param->period;

    // 相位范围限制在 0~2π
    float phi = fmodf(omega * time, 2.0f * PI_VAL);

    // X方向：完整余弦椭圆
    *x = 0.5f * param->step_length * cosf(phi);

    // 支撑相（贴地）
    if (phi < PI_VAL)
    {
        *y = param->stand_height;
    }
    // 摆动相（抬腿）
    else
    {
        *y = param->stand_height
             - param->step_height * sinf(phi);
    }
}

/**
 * @brief  五杆正运动学
 * @param  theta1  左电机角度 (弧度)
 * @param  theta2  右电机角度 (弧度)
 * @param  P       输出足端坐标
 * @param  elbow_up true=上解  false=下解
 * @return 是否成功（是否在工作空间内）
 */
bool fivebar_forward(float theta1, float theta2, Point2D *P, bool elbow_up)
{
    // 主动杆端点 A
    float Ax = L1 * cosf(theta1);
    float Ay = L1 * sinf(theta1);

    // 主动杆端点 B
    float Bx = L1 * cosf(theta2);
    float By = L1 * sinf(theta2);

    // AB 向量
    float dx = Bx - Ax;
    float dy = By - Ay;

    float d = sqrtf(dx*dx + dy*dy);

    // 奇异情况处理
    if (d < EPS)
        return false;

    // 圆是否相交
    if (d > 2.0f * L2)
        return false;

    // 中点
    float mx = (Ax + Bx) * 0.5f;
    float my = (Ay + By) * 0.5f;

    // 高度
    float h_sq = L2*L2 - (d*d)*0.25f;
    if (h_sq < 0.0f)
        return false;

    float h = sqrtf(h_sq);

    // 单位法向量
    float nx = -dy / d;
    float ny =  dx / d;

    if (elbow_up)
    {
        P->x = mx + h * nx;
        P->y = my + h * ny;
    }
    else
    {
        P->x = mx - h * nx;
        P->y = my - h * ny;
    }

    return true;
}

/*
 * @brief  五杆逆运动学
 * @param  x       足端目标坐标 x (mm)
 * @param  y       足端目标坐标 y (mm)
 * @param  theta1  输出左电机角度 (弧度)
 * @param  theta2  输出右电机角度 (弧度)
 * @param  elbow_up true=上解  false=下解
 *  2-DOF Symmetric Five-Bar Parallel Mechanism
 *  Closed-Form Inverse Kinematics (MCU Version)
 *  机构结构说明：
 *  - 两个电机关于运动平面对称
 *  - 两电机轴垂直于运动平面
 *  - 每个电机轴距运动平面 22mm
 *  - 主动杆、从动杆均为碳板，厚度 8mm
 *  - 总厚度 44mm（2主动 + 2从动 + 轴承/螺丝）
 *
 *  坐标定义：
 *  - 运动平面为 XY 平面
 *  - 左右电机坐标 (0 , 0)
 *  - 足端 P(x , y)
 *
 *  返回值：
 *      true  —— 有解
 *      false —— 不可达（超出工作空间）
 */

bool fivebar_inverse(float x, float y,
                     float *theta1,
                     float *theta2,
                     bool elbow_up)
{
    float r = sqrtf(x*x + y*y);
    if (r < EPS)
        return false;

    float R2 = x*x + y*y;

    float C = (R2 + L1*L1 - L2*L2) / (2.0f * L1);

    float cos_val = C / r;

    // 工作空间判断
    if (cos_val > 1.0f || cos_val < -1.0f)
        return false;

    float alpha = atan2f(y, x);
    float delta = acosf(cos_val);

    if (elbow_up)
    {
        *theta1 = alpha + delta;
        *theta2 = alpha - delta;
    }
    else
    {
        *theta1 = alpha - delta;
        *theta2 = alpha + delta;
    }

    return true;
}

/*
 * @brief  输出轴角度 → 转子角度
 *
 * @param  theta_out   输出轴目标角 (rad)
 * @param  param       关节参数
 *
 * @return 转子目标角 (rad)
 */
float output_to_rotor(float theta_out, JointParam *param)
{
    return ((theta_out-param->output_zero) * param->ratio) / param->dir
           + param->rotor_zero;
}

/*
 * @brief  角度归一化到 (-π, π]
 */
void wrap_pi_fast(float *angle)
{
    *angle = fmodf(*angle + PI, TWO_PI);
    if (*angle < 0)
        *angle += TWO_PI;
    *angle -= PI;
}

