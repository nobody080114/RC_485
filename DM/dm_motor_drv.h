#ifndef __DM_MOTOR_DRV_H__
#define __DM_MOTOR_DRV_H__

#include <stdint.h>
#include "bsp_fdcan.h"

#define MIT_MODE 0x000
#define POS_MODE 0x100
#define SPD_MODE 0x200
#define PSI_MODE 0x300

#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

typedef enum
{
    Motor1,
    Motor2,
    Motor3,
    Motor4,
    Motor5,
    Motor6,
    Motor7,
    Motor8,
    Motor9,
    Motor10,
    num
} motor_num;

typedef enum
{
    mit_mode = 1,
    pos_mode = 2,
    spd_mode = 3,
    psi_mode = 4
} mode_e;

typedef enum
{
    RID_UV_VALUE = 0,
    RID_KT_VALUE = 1,
    RID_OT_VALUE = 2,
    RID_OC_VALUE = 3,
    RID_ACC = 4,
    RID_DEC = 5,
    RID_MAX_SPD = 6,
    RID_MST_ID = 7,
    RID_ESC_ID = 8,
    RID_TIMEOUT = 9,
    RID_CMODE = 10,
    RID_DAMP = 11,
    RID_INERTIA = 12,
    RID_HW_VER = 13,
    RID_SW_VER = 14,
    RID_SN = 15,
    RID_NPP = 16,
    RID_RS = 17,
    RID_LS = 18,
    RID_FLUX = 19,
    RID_GR = 20,
    RID_PMAX = 21,
    RID_VMAX = 22,
    RID_TMAX = 23,
    RID_I_BW = 24,
    RID_KP_ASR = 25,
    RID_KI_ASR = 26,
    RID_KP_APR = 27,
    RID_KI_APR = 28,
    RID_OV_VALUE = 29,
    RID_GREF = 30,
    RID_DETA = 31,
    RID_V_BW = 32,
    RID_IQ_CL = 33,
    RID_VL_CL = 34,
    RID_CAN_BR = 35,
    RID_SUB_VER = 36,
    RID_U_OFF = 50,
    RID_V_OFF = 51,
    RID_K1 = 52,
    RID_K2 = 53,
    RID_M_OFF = 54,
    RID_DIR = 55,
    RID_P_M = 80,
    RID_X_OUT = 81
} rid_e;

typedef struct
{
    uint8_t read_flag;
    uint8_t write_flag;
    uint8_t save_flag;

    float UV_Value;
    float KT_Value;
    float OT_Value;
    float OC_Value;
    float ACC;
    float DEC;
    float MAX_SPD;
    uint32_t MST_ID;
    uint32_t ESC_ID;
    uint32_t TIMEOUT;
    uint32_t cmode;
    float Damp;
    float Inertia;
    uint32_t hw_ver;
    uint32_t sw_ver;
    uint32_t SN;
    uint32_t NPP;
    float Rs;
    float Ls;
    float Flux;
    float Gr;
    float PMAX;
    float VMAX;
    float TMAX;
    float I_BW;
    float KP_ASR;
    float KI_ASR;
    float KP_APR;
    float KI_APR;
    float OV_Value;
    float GREF;
    float Deta;
    float V_BW;
    float IQ_cl;
    float VL_cl;
    uint32_t can_br;
    uint32_t sub_ver;
    float u_off;
    float v_off;
    float k1;
    float k2;
    float m_off;
    float dir;
    float p_m;
    float x_out;
} esc_inf_t;

typedef struct
{
    int id;
    int state;
    int p_int;
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;
    float pos;
    float vel;
    float tor;
    float Kp;
    float Kd;
    float Tmos;
    float Tcoil;
} motor_fbpara_t;

typedef struct
{
    uint8_t mode;
    float pos_set;
    float vel_set;
    float tor_set;
    float cur_set;
    float kp_set;
    float kd_set;
} motor_ctrl_t;

typedef struct
{
    uint16_t id;
    uint16_t mst_id;
    motor_fbpara_t para;
    motor_ctrl_t ctrl;
    esc_inf_t tmp;
} motor_t;

float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x_float, float x_min, float x_max, int bits);
void dm_motor_ctrl_send(hcan_t *hcan, motor_t *motor);
void dm_motor_enable(hcan_t *hcan, motor_t *motor);
void dm_motor_disable(hcan_t *hcan, motor_t *motor);
void dm_motor_clear_para(motor_t *motor);
void dm_motor_clear_err(hcan_t *hcan, motor_t *motor);
void dm_motor_fbdata(motor_t *motor, uint8_t *rx_data);

void enable_motor_mode(hcan_t *hcan, uint16_t motor_id, uint16_t mode_id);
void disable_motor_mode(hcan_t *hcan, uint16_t motor_id, uint16_t mode_id);

void mit_ctrl(hcan_t *hcan, motor_t *motor, uint16_t motor_id, float pos, float vel, float kp, float kd, float tor);
void pos_ctrl(hcan_t *hcan, uint16_t motor_id, float pos, float vel);
void spd_ctrl(hcan_t *hcan, uint16_t motor_id, float vel);
void psi_ctrl(hcan_t *hcan, uint16_t motor_id, float pos, float vel, float cur);

void save_pos_zero(hcan_t *hcan, uint16_t motor_id, uint16_t mode_id);
void clear_err(hcan_t *hcan, uint16_t motor_id, uint16_t mode_id);

void read_motor_data(uint16_t id, uint8_t rid);
void read_motor_ctrl_fbdata(uint16_t id);
void write_motor_data(uint16_t id, uint8_t rid, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);
void save_motor_data(uint16_t id, uint8_t rid);

#endif /* __DM_MOTOR_DRV_H__ */
