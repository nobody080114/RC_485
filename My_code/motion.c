/*
    正运动结算
*/
#include "motion.h"

#define EPS 1e-6f

double PI_VAL = 3.14159265358979323846;

bool fivebar_forward(float theta1, float theta2, Point2D *P, bool elbow_up);
bool fivebar_inverse(float x, float y,float *theta1,float *theta2,bool elbow_up);
extern Foot_motion foot_motion_0,foot_motion_1,foot_motion_2,foot_motion_3;
/*
 * @brief  椭圆轨迹足端位置计算
 *
 * @param  time    当前累计时间 (秒)
 * @param  param   轨迹参数
 * @param  x       输出足端 x 坐标 (m)
 * @param  y       输出足端 y 坐标 (m)
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
void foot_ellipse_trajectory(float time,Foot_motion *param,FootTrajParam *traj)
{
    float omega = 2.0f * PI_VAL / traj->period; // 角频率 (rad/s)

    // 相位范围限制在 0~2π
    float phi = fmodf(omega * time, 2.0f * PI_VAL);

    // X方向：完整余弦椭圆
    param->P.x = 0.5f * param->track.step_length * arm_cos_f32(phi);

    // 支撑相（贴地）
    if (phi < PI_VAL)
    {
        param->P.y = param->track.stand_height;
    }
    // 摆动相（抬腿）
    else
    {
        param->P.y = param->track.stand_height
                     - param->track.step_height * sinf(phi);
    }
}

void foot_ellipse_trajectory_dir(float time, Foot_motion *param, FootTrajParam *traj, float x_dir)
{
    foot_ellipse_trajectory(time, param, traj);
    param->P.x *= x_dir;
}

/**
 * @brief  高度优化的联合结算函数 (正解 + 雅可比)
 * 一次调用完成位置感知与力控映射准备
 */
bool fwd_kinematics_and_jacobian(float theta1, float theta2, bool elbow_up, 
                                 Point2D *P, JacobianMatrix *J)
{
    // 1. 仅调用一次三角函数计算主动杆坐标
    float Ax = L1 * arm_cos_f32(theta1);
    float Ay = L1 * arm_sin_f32(theta1);
    float Bx = L1 * arm_cos_f32(theta2);
    float By = L1 * arm_sin_f32(theta2);

    float dx = Bx - Ax;
    float dy = By - Ay;
    float d2 = dx*dx + dy*dy; // 优化：直接用平方判断，延迟开方

    if (d2 < EPS || d2 > 4.0f * L2 * L2) return false;

    float d = sqrtf(d2);
    float mx = (Ax + Bx) * 0.5f;
    float my = (Ay + By) * 0.5f;
    
    float h_sq = L2*L2 - d2*0.25f;
    if (h_sq < 0.0f) return false;
    float h = sqrtf(h_sq);

    float nx = -dy / d;
    float ny =  dx / d;

    // 2. 结算当前足端坐标
    if (elbow_up) {
        P->x = mx + h * nx;
        P->y = my + h * ny;
    } else {
        P->x = mx - h * nx;
        P->y = my - h * ny;
    }

    // 3. 复用 Ax, Ay, Bx, By，无缝衔接雅可比矩阵计算 (0次三角函数开销)
    float dx1 = P->x - Ax;
    float dy1 = P->y - Ay;
    float dx2 = P->x - Bx;
    float dy2 = P->y - By;

    float B00 = P->y * Ax - P->x * Ay; // 完全取代了 L1*(Y*cos - X*sin)
    float B11 = P->y * Bx - P->x * By;

    float detA = (dx1 * dy2) - (dx2 * dy1);
    
    // 奇异点保护
    if (fabsf(detA) < 1e-4f) return false;

    float inv_detA = 1.0f / detA;
    J->J00 =  dy2 * B00 * inv_detA;
    J->J01 = -dy1 * B11 * inv_detA;
    J->J10 = -dx2 * B00 * inv_detA;
    J->J11 =  dx1 * B11 * inv_detA;

    return true;
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
    float Ax = L1 * arm_cos_f32(theta1);
    float Ay = L1 * arm_sin_f32(theta1);

    // 主动杆端点 B
    float Bx = L1 * arm_cos_f32(theta2);
    float By = L1 * arm_sin_f32(theta2);

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
    float angle_1,angle_2;
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
        angle_1 = alpha + delta;
        angle_2 = alpha - delta;
    }
    else
    {
        angle_1 = alpha - delta;
        angle_2 = alpha + delta;
    }

    *theta1 = angle_1;
    *theta2 = angle_2;
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
float output_to_rotor_stand(float theta_out, JointParam *param)
{
    // if(theta_out-param->output_zero > PI)
    // return ((theta_out-param->output_zero-TWO_PI) * param->ratio) / param->dir
    //     + param->rotor_zero;
    // else if(theta_out-param->output_zero < -PI)
    // return ((theta_out-param->output_zero+TWO_PI) * param->ratio) / param->dir
    //     + param->rotor_zero;
    // else
    return ((theta_out-param->output_zero) * param->ratio) / param->dir
        + param->rotor_zero;
}
float output_to_rotor(float theta_out, JointParam *param)
{
    // if(theta_out-param->output_zero > PI)
    // return ((theta_out-param->output_zero-TWO_PI) * param->ratio) / param->dir
    //     + param->rotor_zero;
    // else if(theta_out-param->output_zero < -PI)
    // return ((theta_out-param->output_zero+TWO_PI) * param->ratio) / param->dir
    //     + param->rotor_zero;
    // else
    return ((theta_out-param->output_zero) * param->ratio) / param->dir
        + param->rotor_zero;
}

/*
 * @brief  转子轴角度 → 输出轴角度
 *
 * @param  theta_in    转子轴当前角 (rad)
 * @param  param       关节参数
 *
 * @return 输出轴角 (rad)
 */
float rotor_to_output(float rotor_now, JointParam *param)
{
    
    // return ((rotor_now-param->rotor_zero) * param->dir) / param->ratio
    //        + param->output_zero;
    if((rotor_now-param->rotor_zero)!=0)
    {
        return ((rotor_now-param->rotor_zero) * param->dir) / param->ratio
               + param->output_zero;
        // wrap_pi_fast(&angle);
        // return angle;
    }
    else
    {
        return param->output_zero;
    }
}

/*
 * @brief  角度归一化到 (-π, π]
 */
void wrap_pi_fast(float *angle)
{
    float a;
    a = fmodf(*angle + PI, TWO_PI);
    if (a < 0)
        a += TWO_PI;
    *angle = a - PI;
}



Filter2ndState vx_filter = {0, 0, 0, 0};
Filter2ndState vy_filter = {0, 0, 0, 0};

// 二阶低通滤波函数 (基于 1kHz 采样, 30Hz 截止频率预计算的参数)
// 如果需要更改频率，可以用 MATLAB/Python 的 scipy.signal.butter 重新生成系数
float apply_2nd_order_lpf(float input, Filter2ndState *s) 
{
    // 预计算的滤波系数 (b为前馈，a为反馈)
    const float b0 = 0.008074f, b1 = 0.016148f, b2 = 0.008074f;
    const float a1 = -1.74534f, a2 = 0.77764f;

    // 差分方程计算
    float output = b0 * input + b1 * s->x1 + b2 * s->x2 
                 - a1 * s->y1 - a2 * s->y2;

    // 更新历史状态
    s->x2 = s->x1;  s->x1 = input;
    s->y2 = s->y1;  s->y1 = output;

    return output;
}

void estimate_foot_force(float tau1, float tau2, JacobianMatrix *J, float *Fx_real, float *Fy_real) 
{
    // 计算 J^T 的行列式
    float detJT = (J->J00 * J->J11) - (J->J10 * J->J01);
    
    if (fabsf(detJT) > 1e-4f) {
        float inv_det = 1.0f / detJT;
        // 计算逆矩阵并乘上实际力矩
        *Fx_real =  (J->J11 * tau1 - J->J10 * tau2) * inv_det;
        *Fy_real = (-J->J01 * tau1 + J->J00 * tau2) * inv_det;
    } else {
        *Fx_real = 0.0f;
        *Fy_real = 0.0f;
    }
}

void set_left_right_step_length(float left_step, float right_step)
{
  foot_motion_0.track.step_length = left_step;   // left front
  foot_motion_3.track.step_length = left_step;   // left rear
  foot_motion_1.track.step_length = right_step;  // right front
  foot_motion_2.track.step_length = right_step;  // right rear
}

void apply_curve_step_length(uint8_t dir, float base_step,float inner_ratio)
{
  float inner_step = base_step * inner_ratio;
  float outer_step = base_step;

  if(dir == 5 || dir == 7) {
    set_left_right_step_length(outer_step, inner_step); // forward right / backward left
  } else if(dir == 6 || dir == 8) {
    set_left_right_step_length(inner_step, outer_step); // forward left / backward right
  }
}
