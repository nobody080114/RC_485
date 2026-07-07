#ifndef DM_CONTROL_H
#define DM_CONTROL_H
#include "dm_motor_drv.h"
#include <stdint.h>

// 控制状态枚举
typedef enum {
    STATE_READ_PARAMS = 0,  // 读取参数阶段
    STATE_RUNNING,          // 正常控制阶段
} ctrl_state_e;

// 1号电机POS调大则从上往下看逆时针运动，限位范围:-2.0~2.0弧度(快撞上两边的机械限位)
// 2号电机POS调大则朝上运动，限位范围:-3.14~0.0弧度(向上抬头约为90度，向下低头略大于90度)
// 3号电机POS调大则夹爪收紧，限位范围:0.0~1.2弧度(夹爪向内收最小为0弧度(7.2cm)即将碰上限位，向外张为-0.8弧度(30cm)即将碰上限位)

#define P_KP	10.0f		// MIT模式P值
#define P_KD	2.7f		// MIT模式D值
#define P_KP_1	20.0f		// 2号电机MIT模式P值
#define V_KP	0.0f		// 速度模式P值
#define V_KD	2.9f		// 速度模式D值
#define TORC_2	0.75f		// 0弧度时(放平)前馈力矩

void dm_motor_ctrl_update(motor_t *motor, uint8_t mode, float pos_set, float vel_set, float tor_set, float cur_set, float kp_set, float kd_set);
void dm_motor_ctrl_save_pos_zero(motor_t *motor);
void DM4310_Init(void);
void DM_Ctrl(void);
void DM_upset(uint16_t count);
#endif //CTRBOARD_CONTROL_H
