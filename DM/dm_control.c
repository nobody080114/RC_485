#include "dm_control.h"
#include "dm_motor_ctrl.h"
#include "fdcan.h"
#include <stdint.h>
#include "gpio.h"
uint16_t angle_raw;

// extern uint8_t flag, set_flag;
float Pos1_set = 0.0f,angle_1 = 0.0f,angle_2 = 0.0f,angle_3 = 0.0f;
extern float angle;
extern uint8_t turn, set_flag, control_motor;
extern uint8_t DM_Enable, DM_Enable_Flag;
uint8_t DM_Enable_Flag_1 = 0;
extern int16_t need_angle;
// extern uint8_t turn,send_index;
// static uint8_t turn = 0;
// static uint8_t send_index = 0;

void DM4310_Init(void)
{

    bsp_fdcan_set_baud(&hfdcan1, CAN_CLASS, CAN_BR_1M);
    bsp_can_init();
    dm_motor_init();
    read_all_motor_data(&motor[Motor1]);
    read_all_motor_data(&motor[Motor2]);
    read_all_motor_data(&motor[Motor3]);

    dm_motor_enable(&hfdcan1, &motor[Motor1]);
    dm_motor_enable(&hfdcan1, &motor[Motor2]);
    dm_motor_enable(&hfdcan1, &motor[Motor3]);
    HAL_Delay(500);

    dm_motor_ctrl_update(&motor[Motor1], mit_mode, 0.0f, 0.0f, 0.0f, 0.0f, P_KP_1, P_KD);
    dm_motor_ctrl_update(&motor[Motor2], mit_mode, 0.0f, 0.0f, 0.0f, 0.0f, P_KP, P_KD);
    dm_motor_ctrl_update(&motor[Motor3], mit_mode, 0.0f, 0.0f, 0.0f, 0.0f, P_KP, P_KD);
    dm_motor_ctrl_update(&motor[Motor2], mit_mode, -3.14f, 0.0f, 0.0f, 0.0f, P_KP, P_KD);
	HAL_Delay(1000);
	dm_motor_ctrl_update(&motor[Motor3], mit_mode, 1.2f, 0.0f, 0.0f, 0.0f, P_KP, P_KD);
	HAL_Delay(1000);
	dm_motor_ctrl_update(&motor[Motor2], mit_mode, 0.0f, 0.0f, 0.0f, 0.0f, P_KP, P_KD);
	HAL_Delay(2000);
	dm_motor_ctrl_update(&motor[Motor2], mit_mode, -3.14f, 0.0f, 0.0f, 0.0f, P_KP, P_KD);
	HAL_Delay(1000);
	dm_motor_ctrl_update(&motor[Motor3], mit_mode, 0.0f, 0.0f, 0.0f, 0.0f, P_KP, P_KD);
	HAL_Delay(1000);
	dm_motor_ctrl_update(&motor[Motor2], mit_mode, 0.0f, 0.0f, 0.0f, 0.0f, P_KP, P_KD);
}

void DM_Ctrl(void)
{
    
    if(DM_Enable_Flag == 1 && DM_Enable_Flag_1 == 0)
    {
        bsp_fdcan_set_baud(&hfdcan1, CAN_CLASS, CAN_BR_1M);
        bsp_can_init();
        dm_motor_init();
        read_all_motor_data(&motor[Motor1]);
        read_all_motor_data(&motor[Motor2]);
        read_all_motor_data(&motor[Motor3]);

        dm_motor_enable(&hfdcan1, &motor[Motor1]);
        dm_motor_enable(&hfdcan1, &motor[Motor2]);
        dm_motor_enable(&hfdcan1, &motor[Motor3]);
        DM_Enable_Flag = 2;
        DM_Enable_Flag_1 = 1;
        // dm_motor_ctrl_update(&motor[Motor1], mit_mode, 0.0f, 0.0f, 0.0f, 0.0f, P_KP_1, P_KD);
        // dm_motor_ctrl_update(&motor[Motor2], mit_mode, 0.0f, 0.0f, 0.0f, 0.0f, P_KP, P_KD);
        dm_motor_ctrl_update(&motor[Motor1], mit_mode,  0.0f, 0.0f, 0.0f, 0.0f, P_KP, P_KD);
        dm_motor_ctrl_update(&motor[Motor2], mit_mode, -0.5f, 0.0f, 0.0f, 0.0f, P_KP_1, P_KD_1);
        dm_motor_ctrl_update(&motor[Motor3], mit_mode, 0.0f, 0.0f, 0.0f, 0.0f, P_KP, P_KD);
    }


    // if(turn == 0x01)
    // {
    //     Pos1_set += angle;
    //     angle = 0;
    //     turn = 0;
    // }
    // else if(turn == 0x02)
    // {
    //     Pos1_set -= angle;
    //     angle = 0;
    //     turn = 0;
    // }

    // dm_motor_ctrl_update(&motor[Motor1], mit_mode, Pos1_set, 0.0f, 0.0f, 0.0f, P_KP_1, P_KD);
    
}

void DM_upset(uint16_t *cnt_dm)
{
	static uint8_t send_index = 0;
    static float angle_cnt = 0.0001f;
    switch(send_index)
    {
        case 0:
            dm_motor_ctrl_send(&hfdcan1, &motor[Motor1]);
            break;
        case 1:
            dm_motor_ctrl_send(&hfdcan1, &motor[Motor2]);
            break;
        case 2:
            dm_motor_ctrl_send(&hfdcan1, &motor[Motor3]);
            break;
        default:
            send_index = 0;
            break;
    }
    if(DM_Enable == 1) 
    {
      if(need_angle>300) 
      { 
        if(control_motor == 1)
        {      
            angle_1 +=angle_cnt;
            dm_motor_ctrl_update(&motor[Motor1], mit_mode, angle_1, 0.0f, 0.0f, 0.0f, P_KP, P_KD);
        }       
        else if (control_motor == 2) 
        {
            angle_2 +=angle_cnt;
            dm_motor_ctrl_update(&motor[Motor2], mit_mode, angle_2, 0.0f, 0.0f, 0.0f, P_KP_1, P_KD_1);
        }
        else if (control_motor == 3) 
        {
            angle_3 +=angle_cnt;
            dm_motor_ctrl_update(&motor[Motor3], mit_mode, angle_3, 0.0f, 0.0f, 0.0f, P_KP, P_KD);
        }
      }
      else if(need_angle< -300) 
      {
        if(control_motor == 1)
        {
            angle_1 -=angle_cnt;
            dm_motor_ctrl_update(&motor[Motor1], mit_mode, angle_1, 0, 0.0f, 0.0f, P_KP, P_KD);
        } 
        else if (control_motor == 2) 
        {
            angle_2 -=angle_cnt;
            dm_motor_ctrl_update(&motor[Motor2], mit_mode, angle_2, 0, 0.0f, 0.0f, P_KP_1, P_KD_1);
        }
        else if (control_motor == 3) 
        {
            angle_3 -=angle_cnt;
            dm_motor_ctrl_update(&motor[Motor3], mit_mode, angle_3, 0, 0.0f, 0.0f, P_KP, P_KD);
        }
      }       
    }
    else if (DM_Enable == 0)
    {
        dm_motor_ctrl_update(&motor[Motor1], mit_mode, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0);
        dm_motor_ctrl_update(&motor[Motor2], mit_mode, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0);
        dm_motor_ctrl_update(&motor[Motor3], mit_mode, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0);
    }
    (*cnt_dm)++;
    if(*cnt_dm >= 3)
    {
        send_index++;
        send_index %= 3;
		*cnt_dm = 0;
    }

}

void dm_motor_ctrl_update(motor_t *motor, uint8_t mode, float pos_set, float vel_set, float tor_set, float cur_set, float kp_set, float kd_set)
{
    motor->ctrl.mode = mode;
    motor->ctrl.pos_set = pos_set;
    motor->ctrl.vel_set = vel_set;
    motor->ctrl.tor_set = tor_set;
    motor->ctrl.cur_set = cur_set;
    motor->ctrl.kp_set = kp_set;
    motor->ctrl.kd_set = kd_set;
}

void dm_motor_ctrl_save_pos_zero(motor_t *motor)
{
    disable_motor_mode(&hfdcan1, motor->id, motor->ctrl.mode);
    save_pos_zero(&hfdcan1, motor->id, motor->ctrl.mode);
    enable_motor_mode(&hfdcan1, motor->id, motor->ctrl.mode);
}
