#include "dm_control.h"
#include "dm_motor_ctrl.h"
#include "fdcan.h"
#include <stdint.h>
#include "gpio.h"
uint16_t angle_raw;

// extern uint8_t flag, set_flag;
float Pos1_set = 0.0f;
extern float angle;
extern uint8_t turn, set_flag;
// extern uint8_t turn,send_index;
// static uint8_t turn = 0;
// static uint8_t send_index = 0;

void DM4310_Init(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
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
}

void DM_Ctrl(void)
{
    if(turn == 0x01)
    {
        Pos1_set += angle;
        angle = 0;
        turn = 0;
    }
    else if(turn == 0x02)
    {
        Pos1_set -= angle;
        angle = 0;
        turn = 0;
    }

    dm_motor_ctrl_update(&motor[Motor1], mit_mode, Pos1_set, 0.0f, 0.0f, 0.0f, P_KP_1, P_KD);
}

void DM_upset(uint16_t count)
{
	static uint8_t send_index = 0;
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

    count++;
    if(count >= 33)
    {
        send_index %= 3;
		count = 0;
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
