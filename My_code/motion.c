/*
    正运动结算
*/
#include "motion.h"
#include <stdint.h>

#define EPS 1e-6f

double PI_VAL = 3.14159265358979323846;

bool fivebar_forward(float theta1, float theta2, Point2D *P, bool elbow_up);
bool fivebar_inverse(float x, float y,float *theta1,float *theta2,bool elbow_up);
extern Foot_motion foot_motion_0,foot_motion_1,foot_motion_2,foot_motion_3;

// 跳跃足端目标参数，单位：x/y 为 m，time 为 s，前馈力为 N。
// 坐标约定：y 越负腿越伸长；x 越负表示足端向后蹬，机体获得向前趋势。
float JUMP_STAND_X        = 0.000f;   // 站立/恢复时足端 x 位置。
float JUMP_STAND_Y        = -0.173f;  // 站立高度，作为跳跃前后默认腿长。
float JUMP_CROUCH_X       = 0.000f;   // 下蹲阶段足端 x 位置，通常保持中位。
float JUMP_CROUCH_Y       = -0.100f;  // 下蹲高度，越接近 0 腿收得越短，蓄力空间越大。
float JUMP_THRUST_X       = -0.100f;  // 蹬伸阶段足端向后目标，越负向前跳得越远。
float JUMP_THRUST_Y       = -0.250f;  // 蹬伸阶段腿伸长目标，越负向上蹬地越强。
float JUMP_FLIGHT_X       = 0.040f;   // 腾空阶段收腿向前摆的位置，用于准备落地。
float JUMP_FLIGHT_Y       = -0.13f;  // 腾空阶段收腿高度，越接近 0 越不容易拖地。
float JUMP_LAND_X         = 0.000f;   // 落地缓冲阶段足端 x 回中。
float JUMP_LAND_Y         = -0.180f;  // 落地缓冲腿长，略低于站立高度用于吸收冲击。
float JUMP_CROUCH_TIME    = 0.120f;   // 下蹲持续时间，过短会蓄力不足，过长动作拖沓。
float JUMP_THRUST_TIME    = 0.110f;   // 蹬伸持续时间，增大可提高跳高/跳远，但过大会冲击大。
float JUMP_FLIGHT_TIME    = 0.40f;   // 腾空最长等待时间，超时会强制进入落地缓冲。
float JUMP_LAND_TIME      = 0.5f;   // 落地缓冲持续时间，增大可更软但恢复更慢。
float JUMP_RECOVER_TIME   = 0.150f;   // 从落地缓冲恢复到站立的时间。
float JUMP_THRUST_FF_X    = -20.0f;   // 蹬伸 x 方向前馈，越负向前冲量越大。
float JUMP_THRUST_FF_Y    = -150.0f;   // 蹬伸 y 方向前馈，越负向上蹬地越强。
float JUMP_LAND_FF_Y      = -8.0f;    // 落地阶段 y 方向轻微支撑前馈，用于辅助缓冲。
float JUMP_LAND_Y_DETECT  = -0.160f;  // 落地检测阈值，至少两腿 current_P.y 大于该值认为触地。

#define NAV_GO_SPEED_LIMIT_CM_S     40.0f 
#define NAV_SWITCH_SPEED_LIMIT_CM_S 40.0f
#define NAV_MAX_STEP_LENGTH          0.120f

#define STAIR_JUMP_HEIGHT            0.150f
#define STAIR_JUMP_FRONT_Y_LIMIT    -0.070f
#define STAIR_JUMP_REAR_LAND_Y      (STAIR_JUMP_FRONT_Y_LIMIT - STAIR_JUMP_HEIGHT)



int8_t jump_start_req = 0, jump_armed = 1;
float jump_ff_x = 0.0f, jump_ff_y = 0.0f;

JumpState jump_state = JUMP_IDLE;
static float jump_state_time = 0.0f;
static float jump_start_x = 0.000f;
static float jump_start_y = -0.173f;
static float jump_end_x = 0.000f;
static float jump_end_y = -0.173f;
static uint8_t stair_jump_enable = 0U;

static float clamp01(float value)
{
    if(value < 0.0f) return 0.0f;
    if(value > 1.0f) return 1.0f;
    return value;
}

static float clampf_local(float value, float min_value, float max_value)
{
    if(value < min_value) return min_value;
    if(value > max_value) return max_value;
    return value;
}

static void set_all_feet_target(float x, float y, float dt)
{
    foot_motion_0.target_vx = (x - foot_motion_0.last_P.x) / dt;
    foot_motion_0.target_vy = (y - foot_motion_0.last_P.y) / dt;
    foot_motion_0.P.x = x; foot_motion_0.P.y = y;
    foot_motion_0.last_P.x = x; foot_motion_0.last_P.y = y;

    foot_motion_1.target_vx = (x - foot_motion_1.last_P.x) / dt;
    foot_motion_1.target_vy = (y - foot_motion_1.last_P.y) / dt;
    foot_motion_1.P.x = x; foot_motion_1.P.y = y;
    foot_motion_1.last_P.x = x; foot_motion_1.last_P.y = y;

    foot_motion_2.target_vx = (x - foot_motion_2.last_P.x) / dt;
    foot_motion_2.target_vy = (y - foot_motion_2.last_P.y) / dt;
    foot_motion_2.P.x = x; foot_motion_2.P.y = y;
    foot_motion_2.last_P.x = x; foot_motion_2.last_P.y = y;

    foot_motion_3.target_vx = (x - foot_motion_3.last_P.x) / dt;
    foot_motion_3.target_vy = (y - foot_motion_3.last_P.y) / dt;
    foot_motion_3.P.x = x; foot_motion_3.P.y = y;
    foot_motion_3.last_P.x = x; foot_motion_3.last_P.y = y;
}

static void set_front_rear_feet_target(float front_x, float front_y,
                                       float rear_x, float rear_y,
                                       float dt)
{
    if(front_y > STAIR_JUMP_FRONT_Y_LIMIT) front_y = STAIR_JUMP_FRONT_Y_LIMIT;
    if(rear_y > STAIR_JUMP_FRONT_Y_LIMIT) rear_y = STAIR_JUMP_FRONT_Y_LIMIT;

    foot_motion_0.target_vx = (front_x - foot_motion_0.last_P.x) / dt;
    foot_motion_0.target_vy = (front_y - foot_motion_0.last_P.y) / dt;
    foot_motion_0.P.x = front_x; foot_motion_0.P.y = front_y;
    foot_motion_0.last_P.x = front_x; foot_motion_0.last_P.y = front_y;

    foot_motion_1.target_vx = (front_x - foot_motion_1.last_P.x) / dt;
    foot_motion_1.target_vy = (front_y - foot_motion_1.last_P.y) / dt;
    foot_motion_1.P.x = front_x; foot_motion_1.P.y = front_y;
    foot_motion_1.last_P.x = front_x; foot_motion_1.last_P.y = front_y;

    foot_motion_2.target_vx = (rear_x - foot_motion_2.last_P.x) / dt;
    foot_motion_2.target_vy = (rear_y - foot_motion_2.last_P.y) / dt;
    foot_motion_2.P.x = rear_x; foot_motion_2.P.y = rear_y;
    foot_motion_2.last_P.x = rear_x; foot_motion_2.last_P.y = rear_y;

    foot_motion_3.target_vx = (rear_x - foot_motion_3.last_P.x) / dt;
    foot_motion_3.target_vy = (rear_y - foot_motion_3.last_P.y) / dt;
    foot_motion_3.P.x = rear_x; foot_motion_3.P.y = rear_y;
    foot_motion_3.last_P.x = rear_x; foot_motion_3.last_P.y = rear_y;
}

static void jump_enter_state(JumpState next_state, float x, float y)
{
    jump_state = next_state;
    jump_state_time = 0.0f;
    if(stair_jump_enable && (next_state == JUMP_LAND || next_state == JUMP_RECOVER)) {
        jump_start_x = foot_motion_2.P.x;
        jump_start_y = foot_motion_2.P.y;
    } else {
        jump_start_x = foot_motion_0.P.x;
        jump_start_y = foot_motion_0.P.y;
    }
    jump_end_x = x;
    jump_end_y = y;
}

static uint8_t jump_land_detected(void)
{
    uint8_t contact_count = 0;
    if(foot_motion_0.current_P.y > JUMP_LAND_Y_DETECT) contact_count++;
    if(foot_motion_1.current_P.y > JUMP_LAND_Y_DETECT) contact_count++;
    if(foot_motion_2.current_P.y > JUMP_LAND_Y_DETECT) contact_count++;
    if(foot_motion_3.current_P.y > JUMP_LAND_Y_DETECT) contact_count++;
    return (contact_count >= 2);
}

void jump_reset(void)
{
    jump_state = JUMP_IDLE;
    jump_state_time = 0.0f;
    jump_start_x = JUMP_STAND_X;
    jump_start_y = JUMP_STAND_Y;
    jump_end_x = JUMP_STAND_X;
    jump_end_y = JUMP_STAND_Y;
    jump_start_req = 0;
    jump_ff_x = 0.0f;
    jump_ff_y = 0.0f;
}

void jump_F_set(uint16_t speed_state)
{
    if(speed_state == 3)
    {
        stair_jump_enable = 0U;
        JUMP_CROUCH_X       = 0.000f;   // 下蹲阶段足端 x 位置，通常保持中位。
        JUMP_CROUCH_Y       = -0.100f;  // 下蹲高度，越接近 0 腿收得越短，蓄力空间越大。
        JUMP_THRUST_X       = -0.100f;  // 蹬伸阶段足端向后目标，越负向前跳得越远。
        JUMP_THRUST_Y       = -0.250f;  // 蹬伸阶段腿伸长目标，越负向上蹬地越强。
        JUMP_FLIGHT_X       = 0.040f;   // 腾空阶段收腿向前摆的位置，用于准备落地。
        JUMP_FLIGHT_Y       = -0.07f;  // 腾空阶段收腿高度，越接近 0 越不容易拖地。
        JUMP_LAND_X         = 0.000f;   // 落地缓冲阶段足端 x 回中。
        JUMP_LAND_Y         = -0.180f;  // 落地缓冲腿长，略低于站立高度用于吸收冲击。
        JUMP_CROUCH_TIME    = 0.120f;   // 下蹲持续时间，过短会蓄力不足，过长动作拖沓。
        JUMP_THRUST_TIME    = 0.110f;   // 蹬伸持续时间，增大可提高跳高/跳远，但过大会冲击大。
        JUMP_FLIGHT_TIME    = 1.00f;   // 腾空最长等待时间，超时会强制进入落地缓冲。
        JUMP_LAND_TIME      = 1.0f;   // 落地缓冲持续时间，增大可更软但恢复更慢。
        JUMP_RECOVER_TIME   = 0.150f;   // 从落地缓冲恢复到站立的时间。
        JUMP_THRUST_FF_X    = -50.0f;   // 蹬伸 x 方向前馈，越负向前冲量越大。
        JUMP_THRUST_FF_Y    = -300.0f;   // 蹬伸 y 方向前馈，越负向上蹬地越强。
    }
    else if(speed_state == 2)
    {
        stair_jump_enable = 1U;
        // 跳跃足端目标参数，单位：x/y 为 m，time 为 s，前馈力为 N。
        // 坐标约定：y 越负腿越伸长；x 越负表示足端向后蹬，机体获得向前趋势。
        JUMP_CROUCH_X       = 0.000f;   // 下蹲阶段足端 x 位置，通常保持中位。
        // JUMP_CROUCH_Y       = -0.100f;  // 下蹲高度，越接近 0 腿收得越短，蓄力空间越大。
        // JUMP_THRUST_X       = -0.100f;  // 蹬伸阶段足端向后目标，越负向前跳得越远。
        // JUMP_THRUST_Y       = -0.250f;  // 蹬伸阶段腿伸长目标，越负向上蹬地越强。
        // JUMP_FLIGHT_X       = 0.040f;   // 腾空阶段收腿向前摆的位置，用于准备落地。
        // JUMP_FLIGHT_Y       = -0.13f;  // 腾空阶段收腿高度，越接近 0 越不容易拖地。
        // JUMP_LAND_X         = 0.000f;   // 落地缓冲阶段足端 x 回中。
        // JUMP_LAND_Y         = -0.180f;  // 落地缓冲腿长，略低于站立高度用于吸收冲击。
        // JUMP_CROUCH_TIME    = 0.120f;   // 下蹲持续时间，过短会蓄力不足，过长动作拖沓。
        // JUMP_THRUST_TIME    = 0.110f;   // 蹬伸持续时间，增大可提高跳高/跳远，但过大会冲击大。
        // JUMP_FLIGHT_TIME    = 0.40f;   // 腾空最长等待时间，超时会强制进入落地缓冲。
        // JUMP_LAND_TIME      = 0.5f;   // 落地缓冲持续时间，增大可更软但恢复更慢。
        // JUMP_RECOVER_TIME   = 0.150f;   // 从落地缓冲恢复到站立的时间。
        // JUMP_THRUST_FF_X    = -20.0f;   // 蹬伸 x 方向前馈，越负向前冲量越大。
        // JUMP_THRUST_FF_Y    = -150.0f;   // 蹬伸 y 方向前馈，越负向上蹬地越强。

        JUMP_CROUCH_Y       = -0.095f;
        JUMP_THRUST_X       = -0.135f;
        JUMP_THRUST_Y       = -0.265f;
        JUMP_FLIGHT_X       = 0.075f;
        JUMP_FLIGHT_Y       = STAIR_JUMP_FRONT_Y_LIMIT;
        JUMP_LAND_X         = 0.020f;
        JUMP_LAND_Y         = STAIR_JUMP_REAR_LAND_Y;
        JUMP_CROUCH_TIME    = 0.160f;
        JUMP_THRUST_TIME    = 0.140f;
        JUMP_FLIGHT_TIME    = 0.52f;
        JUMP_LAND_TIME      = 0.45f;
        JUMP_RECOVER_TIME   = 0.250f;
        JUMP_THRUST_FF_X    = -35.0f;
        JUMP_THRUST_FF_Y    = -240.0f;
        JUMP_LAND_FF_Y      = -10.0f;
    }
    else if (speed_state == 1)
    {
        stair_jump_enable = 0U;
        // 跳跃足端目标参数，单位：x/y 为 m，time 为 s，前馈力为 N。
        // 坐标约定：y 越负腿越伸长；x 越负表示足端向后蹬，机体获得向前趋势。
        JUMP_CROUCH_X       = 0.000f;   // 下蹲阶段足端 x 位置，通常保持中位。
        JUMP_CROUCH_Y       = -0.100f;  // 下蹲高度，越接近 0 腿收得越短，蓄力空间越大。
        JUMP_THRUST_X       = -0.100f;  // 蹬伸阶段足端向后目标，越负向前跳得越远。
        JUMP_THRUST_Y       = -0.250f;  // 蹬伸阶段腿伸长目标，越负向上蹬地越强。
        JUMP_FLIGHT_X       = 0.040f;   // 腾空阶段收腿向前摆的位置，用于准备落地。
        JUMP_FLIGHT_Y       = -0.13f;  // 腾空阶段收腿高度，越接近 0 越不容易拖地。
        JUMP_LAND_X         = 0.000f;   // 落地缓冲阶段足端 x 回中。
        JUMP_LAND_Y         = -0.180f;  // 落地缓冲腿长，略低于站立高度用于吸收冲击。
        JUMP_CROUCH_TIME    = 0.120f;   // 下蹲持续时间，过短会蓄力不足，过长动作拖沓。
        JUMP_THRUST_TIME    = 0.110f;   // 蹬伸持续时间，增大可提高跳高/跳远，但过大会冲击大。
        JUMP_FLIGHT_TIME    = 0.40f;   // 腾空最长等待时间，超时会强制进入落地缓冲。
        JUMP_LAND_TIME      = 0.5f;   // 落地缓冲持续时间，增大可更软但恢复更慢。
        JUMP_RECOVER_TIME   = 0.150f;   // 从落地缓冲恢复到站立的时间。
        JUMP_THRUST_FF_X    = -20.0f;   // 蹬伸 x 方向前馈，越负向前冲量越大。
        JUMP_THRUST_FF_Y    = -150.0f;   // 蹬伸 y 方向前馈，越负向上蹬地越强。
    }
}
void jump_update(float dt)
{
    float duration = JUMP_RECOVER_TIME;
    float phase = 0.0f;
    float target_x = JUMP_STAND_X;
    float target_y = JUMP_STAND_Y;

    jump_ff_x = 0.0f;
    jump_ff_y = 0.0f;

    if(jump_state != JUMP_IDLE) {
        jump_start_req = 0;
    }

    if(jump_state == JUMP_IDLE) {
        set_all_feet_target(JUMP_STAND_X, JUMP_STAND_Y, dt);
        if(jump_start_req) {
            jump_start_req = 0;
            jump_enter_state(JUMP_CROUCH, JUMP_CROUCH_X, JUMP_CROUCH_Y);
        }
    }

    switch(jump_state) {
        case JUMP_CROUCH:
            duration = JUMP_CROUCH_TIME;
            break;
        case JUMP_THRUST:
            duration = JUMP_THRUST_TIME;
            jump_ff_x = JUMP_THRUST_FF_X;
            jump_ff_y = JUMP_THRUST_FF_Y;
            break;
        case JUMP_FLIGHT:
            duration = JUMP_FLIGHT_TIME;
            break;
        case JUMP_LAND:
            duration = JUMP_LAND_TIME;
            jump_ff_y = JUMP_LAND_FF_Y;
            break;
        case JUMP_RECOVER:
            duration = JUMP_RECOVER_TIME;
            break;
        case JUMP_IDLE:
        default:
            return;
    }

    jump_state_time += dt;
    phase = clamp01(jump_state_time / duration);
    target_x = jump_start_x + (jump_end_x - jump_start_x) * phase;
    target_y = jump_start_y + (jump_end_y - jump_start_y) * phase;
    if(stair_jump_enable && (jump_state == JUMP_LAND || jump_state == JUMP_RECOVER)) {
        float front_y = target_y + STAIR_JUMP_HEIGHT;
        if(front_y > STAIR_JUMP_FRONT_Y_LIMIT) front_y = STAIR_JUMP_FRONT_Y_LIMIT;
        set_front_rear_feet_target(target_x, front_y, target_x, target_y, dt);
    } else {
        set_all_feet_target(target_x, target_y, dt);
    }

    if(jump_state == JUMP_CROUCH && jump_state_time >= JUMP_CROUCH_TIME) {
        jump_enter_state(JUMP_THRUST, JUMP_THRUST_X, JUMP_THRUST_Y);
    } else if(jump_state == JUMP_THRUST && jump_state_time >= JUMP_THRUST_TIME) {
        jump_enter_state(JUMP_FLIGHT, JUMP_FLIGHT_X, JUMP_FLIGHT_Y);
    } else if(jump_state == JUMP_FLIGHT &&
              ((jump_state_time > 0.040f && jump_land_detected()) || jump_state_time >= JUMP_FLIGHT_TIME)) {
        jump_enter_state(JUMP_LAND, JUMP_LAND_X, JUMP_LAND_Y);
    } else if(jump_state == JUMP_LAND && jump_state_time >= JUMP_LAND_TIME) {
        jump_enter_state(JUMP_RECOVER, JUMP_STAND_X, JUMP_STAND_Y);
    } else if(jump_state == JUMP_RECOVER && jump_state_time >= JUMP_RECOVER_TIME) {
        jump_reset();
        set_all_feet_target(JUMP_STAND_X, JUMP_STAND_Y, dt);
    }
}
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
                     - param->track.step_height * arm_sin_f32(phi);
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

void nav_update_step_length(int16_t go_speed_cm_s, int16_t switch_speed_cm_s, float period, float *inner_ratio)
{
  float go_speed = clampf_local((float)go_speed_cm_s, -NAV_GO_SPEED_LIMIT_CM_S, NAV_GO_SPEED_LIMIT_CM_S);
  float switch_speed = clampf_local((float)switch_speed_cm_s, -NAV_SWITCH_SPEED_LIMIT_CM_S, NAV_SWITCH_SPEED_LIMIT_CM_S);
  float go_step = fabsf(go_speed) * 0.01f * period;
  float switch_step = fabsf(switch_speed) * 0.01f * period;
  float left_step = go_step;
  float right_step = go_step;
  float ratio = 1.0f;

  go_step = clampf_local(go_step, 0.0f, NAV_MAX_STEP_LENGTH);
  switch_step = clampf_local(switch_step, 0.0f, NAV_MAX_STEP_LENGTH);

  if(go_step <= EPS && switch_step <= EPS) {
    set_left_right_step_length(0.0f, 0.0f);
    if(inner_ratio != NULL) *inner_ratio = 1.0f;
    return;
  }

  if(go_step <= EPS) {
    set_left_right_step_length(switch_step, switch_step);
    if(inner_ratio != NULL) *inner_ratio = 1.0f;
    return;
  }

  if(switch_step > EPS) {
    float outer_step = clampf_local(go_step + switch_step, 0.0f, NAV_MAX_STEP_LENGTH);
    float inner_step = clampf_local(go_step - switch_step, 0.0f, NAV_MAX_STEP_LENGTH);

    ratio = (outer_step > EPS) ? (inner_step / outer_step) : 1.0f;
    if(switch_speed > 0.0f) {
      left_step = outer_step;
      right_step = inner_step;
    } else {
      left_step = inner_step;
      right_step = outer_step;
    }
  }

  set_left_right_step_length(left_step, right_step);
  if(inner_ratio != NULL) *inner_ratio = clampf_local(ratio, 0.0f, 1.0f);
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
