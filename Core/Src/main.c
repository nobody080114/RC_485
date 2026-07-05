/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 速度低通滤波器参数 (截止频率约 30~50Hz)
// alpha 越小滤波越强，但延迟越大；alpha 越大越灵敏，但噪声越大。
#define VEL_ALPHA 0.2f 
RS485_Scheduler_t rs485;
FootTrajParam traj_param;
Point2D P,current_P;
JointParam joint_param_0, joint_param_1, joint_param_2, joint_param_3, joint_param_4, joint_param_5, joint_param_6, joint_param_7;
PosPID_t Pospid[8];
float G_0 = -72.0F,G_1 = -72.0f,G_2 = -68.0f,G_3 = -68.0f;
float time = 0.0f,dt = 0.001f;
float theta0_out = 0, theta1_out = 0;
float go_0_pos=0,go_1_pos=0,go_2_pos=0,go_3_pos=0,go_4_pos=0,go_5_pos=0,go_6_pos=0,go_7_pos=0;
int8_t start = 0,start_1 = 0,run  = 0,flag_1 = 1,stand_flag = 0,mode = 0,last_mode = 0,control_mode = 0,Enable = 0,go_dir = 0,stand_state = 0;
// go_dir: 1-forward, 2-backward, 3-turn right, 4-turn left,
//         5-forward right, 6-forward left, 7-backward right, 8-backward left
//go_dir: 1-前进，2-后退，3-左移，4-右移
uint16_t speed = 0,speed_state = 0,last_speed_state = 0;//speed_state: 0-停止，1-低速，2-中速，3-高速
uint32_t zhen = 0,cnt = 0,cnt_tx = 0;
uint16_t Key[10];
float kp_x = 400.0f, kd_xt = 25.0f, kd_xr = 20.0f;
float kp_y = 1000.0f, kd_yt = 115.0f, kd_yr = 87.0f;
float inner_ratio = 0.55f;
Filter2ndState filter_w1 = {0};
Filter2ndState filter_w2 = {0};
Foot_motion foot_motion_0 = {
 .Kp_x = 400, /* X方向的比例增益 (N/m) */ .Kd_xt = 25,  /* X轴前馈阻尼系数 (N·s/m) */ .Kd_xr = 20,  /* X轴反馈阻尼系数 (N·s/m)*/
 .Kp_y = 2000,/* Y方向的比例增益 (N/m) */ .Kd_yt = 115,  /* Y轴前馈阻尼系数 (N·s/m)  */  .Kd_yr = 87,  /* Y轴反馈阻尼系数 (N·s/m) */
 .G_0 = 3.2f, /* 单腿重力补偿增益 */ .G_1 = 0.0f /* 单腿重力补偿增益 */ 
};

Foot_motion foot_motion_1 = {
 .Kp_x = 400, /* X方向的比例增益 (N/m) */ .Kd_xt = 25,  /* X轴前馈阻尼系数 (N·s/m) */ .Kd_xr = 20,  /* X轴反馈阻尼系数 (N·s/m)*/
 .Kp_y = 2000,/* Y方向的比例增益 (N/m) */ .Kd_yt = 115,  /* Y轴前馈阻尼系数 (N·s/m)  */  .Kd_yr = 87,  /* Y轴反馈阻尼系数 (N·s/m) */
 .G_0 = 3.2f, /* 单腿重力补偿增益 */ .G_1 = 0.0f /* 单腿重力补偿增益 */
};

Foot_motion foot_motion_2 = {
 .Kp_x = 400, /* X方向的比例增益 (N/m) */ .Kd_xt = 25,  /* X轴前馈阻尼系数 (N·s/m) */ .Kd_xr = 20,  /* X轴反馈阻尼系数 (N·s/m)*/
 .Kp_y = 2000,/* Y方向的比例增益 (N/m) */ .Kd_yt = 115,  /* Y轴前馈阻尼系数 (N·s/m)  */  .Kd_yr = 87,  /* Y轴反馈阻尼系数 (N·s/m) */
 .G_0 = 3.2f, /* 单腿重力补偿增益 */ .G_1 = 0.0f /* 单腿重力补偿增益 */
};
Foot_motion foot_motion_3 = {
 .Kp_x = 400, /* X方向的比例增益 (N/m) */ .Kd_xt = 25,  /* X轴前馈阻尼系数 (N·s/m) */ .Kd_xr = 20,  /* X轴反馈阻尼系数 (N·s/m)*/
 .Kp_y = 2000,/* Y方向的比例增益 (N/m) */ .Kd_yt = 115,  /* Y轴前馈阻尼系数 (N·s/m)  */  .Kd_yr = 87,  /* Y轴反馈阻尼系数 (N·s/m) */
 .G_0 = 3.2f, /* 单腿重力补偿增益 */ .G_1 = 0.0f /* 单腿重力补偿增益 */
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  DWT_Init();
  HAL_Delay(500);
  P.x = 0.0f;
  P.y = 0.0f;

  joint_param_0.rotor_zero = 0;// 根据实际安装调整零位
  joint_param_0.output_zero = -2.622;// 根据实际安装调整零位
  joint_param_0.ratio = 6.33f;
  joint_param_0.dir = 1;

  joint_param_1.rotor_zero = 0.0f;// 根据实际安装调整零位
  joint_param_1.output_zero = -0.54070;// 根据实际安装调整零位
  joint_param_1.ratio = 6.33f;
  joint_param_1.dir = -1;

  joint_param_2.rotor_zero = 0.0f;// 根据实际转子调整零位
  joint_param_2.output_zero = -2.622;// 根据实际安装调整零位
  joint_param_2.ratio = 6.33f;
  joint_param_2.dir = -1;

  joint_param_3.rotor_zero = 0.0f;// 根据实际转子调整零位
  joint_param_3.output_zero = -0.54070;// 根据实际安装调整零位
  joint_param_3.ratio = 6.33f;
  joint_param_3.dir = 1;

  joint_param_4.rotor_zero = 0.0f;// 根据实际转子调整零位
  joint_param_4.output_zero = -2.622;// 根据实际安装调整零位
  joint_param_4.ratio = 6.33f;
  joint_param_4.dir = -1;

  joint_param_5.rotor_zero = 0.0f;// 根据实际转子调整零位
  joint_param_5.output_zero = -0.54070;// 根据实际安装调整零位
  joint_param_5.ratio = 6.33f;
  joint_param_5.dir = 1;

  joint_param_6.rotor_zero = 0.0f;// 根据实际转子调整零位
  joint_param_6.output_zero = -0.54070;// 根据实际安装调整零位
  joint_param_6.ratio = 6.33f;
  joint_param_6.dir = -1;

  joint_param_7.rotor_zero = 0.0f;// 根据实际转子调整零位
  joint_param_7.output_zero = -2.622;// 根据实际安装调整零位
  joint_param_7.ratio = 6.33f;
  joint_param_7.dir = 1;
  
  foot_motion_0.track.step_length  = 0.060f;
  foot_motion_0.track.step_height  = 0.025f;
  foot_motion_0.track.stand_height = -0.150f;

  foot_motion_1.track.step_length  = 0.060f;
  foot_motion_1.track.step_height  = 0.025f;
  foot_motion_1.track.stand_height = -0.150f;

  foot_motion_2.track.step_length  = 0.060f;
  foot_motion_2.track.step_height  = 0.025f;
  foot_motion_2.track.stand_height = -0.150f;

  foot_motion_3.track.step_length  = 0.060f;
  foot_motion_3.track.step_height  = 0.025f;
  foot_motion_3.track.stand_height = -0.150f;

  traj_param.period = 1.0f;
  foot_motion_0.P.x = 0; foot_motion_0.P.y = -0.173f;
  foot_motion_1.P.x = 0; foot_motion_1.P.y = -0.173f;
  foot_motion_2.P.x = 0; foot_motion_2.P.y = -0.173f;
  foot_motion_3.P.x = 0; foot_motion_3.P.y = -0.173f;
  rs485.current_motor = 0;
  rs485.tx_busy = 0;
  rs485.rx_ready = 1;
  rs485.waiting_rx = 0;
  rs485.rx_start_tick = 0;
  rs485.huart_ch1 = &huart2;
  rs485.huart_ch2 = &huart3;

  RS485_SetRxTimeout(&rs485, 10U);
  RS485_SetMotorMask(&rs485, RS485_ALL_MOTOR_MASK); // 改这里可选择本次参与读写的电机
  // RS485_SetMotorMask(&rs485, (1U << 0) | (1U << 1)); // 只读写电机0和1
  
  //RS485_1
  cmd_0.id = 0;     cmd_1.id = 1;     cmd_2.id = 2;    cmd_3.id = 3;
  cmd_0.mode = 1;   cmd_1.mode = 1;   cmd_2.mode = 1;  cmd_3.mode = 1;
  cmd_0.K_P = 0;    cmd_1.K_P = 0;    cmd_2.K_P = 0;   cmd_3.K_P = 0;
  cmd_0.K_W = 0;    cmd_1.K_W = 0;    cmd_2.K_W = 0;   cmd_3.K_W = 0;
  cmd_0.Pos = 0;    cmd_1.Pos = 0;    cmd_2.Pos = 0;   cmd_3.Pos = 0;
  cmd_0.W = 0;      cmd_1.W = 0;      cmd_2.W = 0;     cmd_3.W = 0;
  cmd_0.T = 0;      cmd_1.T = 0;      cmd_2.T = 0;     cmd_3.T = 0;
  //RS485_2
  cmd_4.id = 4;     cmd_5.id = 5;     cmd_6.id = 6;    cmd_7.id = 7;
  cmd_4.mode = 1;   cmd_5.mode = 1;   cmd_6.mode = 1;  cmd_7.mode = 1;
  cmd_4.K_P = 0;    cmd_5.K_P = 0;    cmd_6.K_P = 0;   cmd_7.K_P = 0;
  cmd_4.K_W = 0;    cmd_5.K_W = 0;    cmd_6.K_W = 0;   cmd_7.K_W = 0;
  cmd_4.Pos = 0;    cmd_5.Pos = 0;    cmd_6.Pos = 0;   cmd_7.Pos = 0;
  cmd_4.W = 0;      cmd_5.W = 0;      cmd_6.W = 0;     cmd_7.W = 0;
  cmd_4.T = 0;      cmd_5.T = 0;      cmd_6.T = 0;     cmd_7.T = 0;
  HAL_TIM_Base_Start_IT(&htim1); // 启动定时器1的中断，定时器1的周期由cubemx设置，这里是1ms
  CRSF_Init();
  COM_UART_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    foot_motion_0.Kp_x = kp_x; foot_motion_0.Kd_xt = kd_xt; foot_motion_0.Kd_xr = kd_xr;
    foot_motion_1.Kp_x = kp_x; foot_motion_1.Kd_xt = kd_xt; foot_motion_1.Kd_xr = kd_xr;
    foot_motion_2.Kp_x = kp_x; foot_motion_2.Kd_xt = kd_xt; foot_motion_2.Kd_xr = kd_xr;
    foot_motion_3.Kp_x = kp_x; foot_motion_3.Kd_xt = kd_xt; foot_motion_3.Kd_xr = kd_xr;
    foot_motion_0.Kp_y = kp_y; foot_motion_0.Kd_yt = kd_yt; foot_motion_0.Kd_yr = kd_yr;
    foot_motion_1.Kp_y = kp_y; foot_motion_1.Kd_yt = kd_yt; foot_motion_1.Kd_yr = kd_yr;
    foot_motion_2.Kp_y = kp_y; foot_motion_2.Kd_yt = kd_yt; foot_motion_2.Kd_yr = kd_yr;
    foot_motion_3.Kp_y = kp_y; foot_motion_3.Kd_yt = kd_yt; foot_motion_3.Kd_yr = kd_yr;
    RS485_Schedule(&rs485);
    CRSF_Schedule();
    if(Enable)
    {
        if(flag_1 == 1)
        {
          // if(mode == 3)
          // {
          //   PosPID_Init(&Pospid[0], 0.25f, 0.0f, 0.01f, 0, 0.003f);  
          //   PosPID_Init(&Pospid[1], 0.25f, 0.0f, 0.01f, 0, 0.003f);
          //   PosPID_Init(&Pospid[2], 0.25f, 0.0f, 0.01f, 0, 0.003f);
          //   PosPID_Init(&Pospid[3], 0.25f, 0.0f, 0.01f, 0, 0.003f);
          //   PosPID_Init(&Pospid[4], 0.25f, 0.0f, 0.01f, 0, 0.003f);
          //   PosPID_Init(&Pospid[5], 0.25f, 0.0f, 0.01f, 0, 0.003f);
          //   PosPID_Init(&Pospid[6], 0.25f, 0.0f, 0.01f, 0, 0.003f);
          //   PosPID_Init(&Pospid[7], 0.25f, 0.0f, 0.01f, 0, 0.003f);
          // }
        }
        else if(flag_1 == 2) 
        {
            if(control_mode == 0 || control_mode == 2)
            {
              cmd_0.T = 0;cmd_1.T = 0;cmd_2.T = 0;cmd_3.T = 0;
              cmd_4.T = 0;cmd_5.T = 0;cmd_6.T = 0;cmd_7.T = 0;  
              if(speed_state == 3)
              {
                  PosPID_Init(&Pospid[0], 1.0f, 0.0f, 0.01f, 0, 0.001f);  
                  PosPID_Init(&Pospid[1], 1.0f, 0.0f, 0.01f, 0, 0.001f);
                  PosPID_Init(&Pospid[2], 1.0f, 0.0f, 0.01f, 0, 0.001f);
                  PosPID_Init(&Pospid[3], 1.0f, 0.0f, 0.01f, 0, 0.001f);
                  PosPID_Init(&Pospid[4], 1.0f, 0.0f, 0.01f, 0, 0.001f);
                  PosPID_Init(&Pospid[5], 1.0f, 0.0f, 0.01f, 0, 0.001f);
                  PosPID_Init(&Pospid[6], 1.0f, 0.0f, 0.01f, 0, 0.001f);
                  PosPID_Init(&Pospid[7], 1.0f, 0.0f, 0.01f, 0, 0.001f);
              }
              else if(speed_state == 2)
              {
                  PosPID_Init(&Pospid[0], 1.0f, 0.0f, 0.01f, 0, 0.002f);  
                  PosPID_Init(&Pospid[1], 1.0f, 0.0f, 0.01f, 0, 0.002f);
                  PosPID_Init(&Pospid[2], 1.0f, 0.0f, 0.01f, 0, 0.002f);
                  PosPID_Init(&Pospid[3], 1.0f, 0.0f, 0.01f, 0, 0.002f);
                  PosPID_Init(&Pospid[4], 1.0f, 0.0f, 0.01f, 0, 0.002f);
                  PosPID_Init(&Pospid[5], 1.0f, 0.0f, 0.01f, 0, 0.002f);
                  PosPID_Init(&Pospid[6], 1.0f, 0.0f, 0.01f, 0, 0.002f);
                  PosPID_Init(&Pospid[7], 1.0f, 0.0f, 0.01f, 0, 0.002f);
              }
              else if(speed_state == 1)
              {
                  PosPID_Init(&Pospid[0], 0.6f, 0.0f, 0.01f, 0, 0.003f);  
                  PosPID_Init(&Pospid[1], 0.6f, 0.0f, 0.01f, 0, 0.003f);
                  PosPID_Init(&Pospid[2], 0.6f, 0.0f, 0.01f, 0, 0.003f);
                  PosPID_Init(&Pospid[3], 0.6f, 0.0f, 0.01f, 0, 0.003f);
                  PosPID_Init(&Pospid[4], 0.6f, 0.0f, 0.01f, 0, 0.003f);
                  PosPID_Init(&Pospid[5], 0.6f, 0.0f, 0.01f, 0, 0.003f);
                  PosPID_Init(&Pospid[6], 0.6f, 0.0f, 0.01f, 0, 0.003f);
                  PosPID_Init(&Pospid[7], 0.6f, 0.0f, 0.01f, 0, 0.003f);
              }
            }
            else if(control_mode == 1)
            {
                PosPID_Init(&Pospid[0], 0.0f, 0.0f, 0.00f, 0, 0.003f);  
                PosPID_Init(&Pospid[1], 0.0f, 0.0f, 0.00f, 0, 0.003f);
                PosPID_Init(&Pospid[2], 0.0f, 0.0f, 0.00f, 0, 0.003f);
                PosPID_Init(&Pospid[3], 0.0f, 0.0f, 0.00f, 0, 0.003f);
                PosPID_Init(&Pospid[4], 0.0f, 0.0f, 0.00f, 0, 0.003f);
                PosPID_Init(&Pospid[5], 0.0f, 0.0f, 0.00f, 0, 0.003f);
                PosPID_Init(&Pospid[6], 0.0f, 0.0f, 0.00f, 0, 0.003f);
                PosPID_Init(&Pospid[7], 0.0f, 0.0f, 0.00f, 0, 0.003f);
            }
        }
    }
    else
    {
      cmd_0.T = 0;cmd_1.T = 0;cmd_2.T = 0;cmd_3.T = 0;
      cmd_4.T = 0;cmd_5.T = 0;cmd_6.T = 0;cmd_7.T = 0;   
      PosPID_Init(&Pospid[0], 0, 0, 0, 0, 0);  
      PosPID_Init(&Pospid[1], 0, 0, 0, 0, 0);
      PosPID_Init(&Pospid[2], 0, 0, 0, 0, 0);
      PosPID_Init(&Pospid[3], 0, 0, 0, 0, 0);
      PosPID_Init(&Pospid[4], 0, 0, 0, 0, 0);
      PosPID_Init(&Pospid[5], 0, 0, 0, 0, 0);
      PosPID_Init(&Pospid[6], 0, 0, 0, 0, 0);
      PosPID_Init(&Pospid[7], 0, 0, 0, 0, 0);
    } 
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 68;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 6144;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM1) //check if the interrupt comes from TIM1
    {
        static uint16_t count = 0;
        static uint16_t count_1 = 0;
        count_1++;
        if(count_1>=1000)
        {
          zhen = cnt_tx;
          cnt_tx = 0;
          count_1 = 0;
        }
        // if((start_1 == 1)&&(flag_1==0))
        // {
        //     if(count < 1000)count++;
        //     if(count>= 1000)
        //     {
        //         joint_param_0.rotor_zero = data_0.Pos;
        //         joint_param_1.rotor_zero = data_1.Pos;
        //         joint_param_2.rotor_zero = data_2.Pos;
        //         joint_param_3.rotor_zero = data_3.Pos;
        //         joint_param_4.rotor_zero = data_4.Pos;
        //         joint_param_5.rotor_zero = data_5.Pos;
        //         joint_param_6.rotor_zero = data_6.Pos;
        //         joint_param_7.rotor_zero = data_7.Pos;

        //         time = 0.0f;
        //         flag_1 = 2;
        //         count = 0;
        //     }
        // }
        // if(flag_1 == 1)
        // {
        //     if(mode == 3)
        //     {
        //         cmd_0.Pos = output_to_rotor(0, &joint_param_0); PosPID_UpdateCmd(&cmd_0, 0, &Pospid[0]);
        //         cmd_1.Pos = output_to_rotor_stand(-2*PI, &joint_param_1); PosPID_UpdateCmd(&cmd_1, -2*PI, &Pospid[1]);
        //         cmd_2.Pos = output_to_rotor(PI, &joint_param_2); PosPID_UpdateCmd(&cmd_2, PI, &Pospid[2]);
        //         cmd_3.Pos = output_to_rotor_stand(PI, &joint_param_3); PosPID_UpdateCmd(&cmd_3, PI, &Pospid[3]);
        //         cmd_4.Pos = output_to_rotor_stand(PI, &joint_param_4); PosPID_UpdateCmd(&cmd_4, PI, &Pospid[4]);
        //         cmd_5.Pos = output_to_rotor(PI, &joint_param_5); PosPID_UpdateCmd(&cmd_5, PI, &Pospid[5]);
        //         cmd_6.Pos = output_to_rotor(0, &joint_param_6); PosPID_UpdateCmd(&cmd_6, -2*PI, &Pospid[6]);
        //         cmd_7.Pos = output_to_rotor_stand(-2*PI, &joint_param_7); PosPID_UpdateCmd(&cmd_7, 0, &Pospid[7]);                                        
        //     }
        // }
        if((start_1 == 1)&&(flag_1==1))
        {
            if(count < 1000)count++;
            if(count>= 1000)
            {
                joint_param_0.rotor_zero = data_0.Pos;
                joint_param_1.rotor_zero = data_1.Pos;
                joint_param_2.rotor_zero = data_2.Pos;
                joint_param_3.rotor_zero = data_3.Pos;
                joint_param_4.rotor_zero = data_4.Pos;
                joint_param_5.rotor_zero = data_5.Pos;
                joint_param_6.rotor_zero = data_6.Pos;
                joint_param_7.rotor_zero = data_7.Pos;
                joint_param_0.output_zero = -2.622;// 根据实际安装调整零位
                joint_param_1.output_zero = -0.54070;// 根据实际安装调整零位
                joint_param_2.output_zero = -2.63684;// 根据实际安装调整零位
                joint_param_3.output_zero = -0.54070;// 根据实际安装调整零位
                joint_param_4.output_zero = -2.622;// 根据实际安装调整零位
                joint_param_5.output_zero = -0.54070;// 根据实际安装调整零位
                joint_param_6.output_zero = -0.54070;// 根据实际安装调整零位
                joint_param_7.output_zero = -2.622;// 根据实际安装调整零位
                time = 0.0f;     
                flag_1 = 2;
            }
        }
        if(flag_1 == 2)
        {
            if((run == 1))
            {   
                last_speed_state = speed_state;
                if(speed>=1100) 
                {
                  foot_motion_0.track.step_length  = 0.100f;
                  foot_motion_1.track.step_length  = 0.100f;
                  foot_motion_2.track.step_length  = 0.100f;
                  foot_motion_3.track.step_length  = 0.100f;
                  traj_param.period = 0.25f;
                  speed_state = 3;
                }
                else if(speed<1100 && speed >= 400) 
                {
                  foot_motion_0.track.step_length  = 0.100f;
                  foot_motion_1.track.step_length  = 0.100f;
                  foot_motion_2.track.step_length  = 0.100f;
                  foot_motion_3.track.step_length  = 0.100f;
                  traj_param.period = 0.3f;
                  speed_state = 2;
                }
                else if(speed<400 && speed >=0) 
                {
                  foot_motion_0.track.step_length  = 0.100f; 
                  foot_motion_1.track.step_length  = 0.100f; 
                  foot_motion_2.track.step_length  = 0.100f; 
                  foot_motion_3.track.step_length  = 0.100f; 
                  traj_param.period = 0.4f;
                  speed_state = 1;
                }
                apply_curve_step_length(go_dir, foot_motion_0.track.step_length, inner_ratio);
                if(last_speed_state!= speed_state) {time = 0.0f;}
                // if(last_mode == 1 && mode == 2) {stand_flag = 2;}
                if(control_mode == 0 || control_mode == 2)
                {                                  
                  if(mode == 1)
                  {
                    cmd_0.Pos = output_to_rotor(-PI, &joint_param_0); PosPID_UpdateCmd(&cmd_0, 0, &Pospid[0]);
                    cmd_1.Pos = output_to_rotor(0, &joint_param_1); PosPID_UpdateCmd(&cmd_1, -PI, &Pospid[1]);
                    cmd_2.Pos = output_to_rotor(-PI, &joint_param_2); PosPID_UpdateCmd(&cmd_2, PI, &Pospid[2]);
                    cmd_3.Pos = output_to_rotor(0, &joint_param_3); PosPID_UpdateCmd(&cmd_3, PI, &Pospid[3]);
                    cmd_4.Pos = output_to_rotor(-PI, &joint_param_4); PosPID_UpdateCmd(&cmd_4, PI, &Pospid[4]);
                    cmd_5.Pos = output_to_rotor(0, &joint_param_5); PosPID_UpdateCmd(&cmd_5, PI, &Pospid[5]);
                    cmd_6.Pos = output_to_rotor(0, &joint_param_6); PosPID_UpdateCmd(&cmd_6, -2*PI, &Pospid[6]);
                    cmd_7.Pos = output_to_rotor(-PI, &joint_param_7); PosPID_UpdateCmd(&cmd_7, 0, &Pospid[7]);
                    time = 0.0f;
                    // stand_flag = 1;
                  }
                  if(mode == 2)
                  {
                      if(go_dir == 1 || go_dir == 5 || go_dir == 6||go_dir == 3 || go_dir == 4) 
                      {
                        time += dt; 
                        if(time > traj_param.period) time -= traj_param.period;
                        else if(time < 0) time += traj_param.period;
                      }//前进             
                      else if(go_dir == 2 || go_dir == 7 || go_dir == 8) 
                      {
                        time -= dt;
                        if(time > traj_param.period) time -= traj_param.period;
                        else if(time < 0) time += traj_param.period;
                      }//后退
                      if(go_dir == 1 || go_dir == 2 || (go_dir >= 5 && go_dir <= 8))
                      {
                        foot_ellipse_trajectory(time, &foot_motion_0, &traj_param);  //左前
                        foot_ellipse_trajectory((time+traj_param.period/2.0f), &foot_motion_1, &traj_param); //右前
                        foot_ellipse_trajectory(time, &foot_motion_2, &traj_param);  //2右后
                        foot_ellipse_trajectory((time+traj_param.period/2.0f), &foot_motion_3, &traj_param); //3左后
                      }
                      if(go_dir == 3)
                      {
                        foot_ellipse_trajectory_dir(time, &foot_motion_0, &traj_param, 1.0f);
                        foot_ellipse_trajectory_dir((time+traj_param.period/2.0f), &foot_motion_1, &traj_param, -1.0f); 
                        foot_ellipse_trajectory_dir(time, &foot_motion_2, &traj_param, -1.0f);
                        foot_ellipse_trajectory_dir((time+traj_param.period/2.0f), &foot_motion_3, &traj_param, 1.0f);                                                                       
                      }
                      else if(go_dir == 4)
                      {
                        foot_ellipse_trajectory_dir(time, &foot_motion_0, &traj_param, -1.0f);
                        foot_ellipse_trajectory_dir((time+traj_param.period/2.0f), &foot_motion_1, &traj_param, 1.0f); 
                        foot_ellipse_trajectory_dir(time, &foot_motion_2, &traj_param, 1.0f);
                        foot_ellipse_trajectory_dir((time+traj_param.period/2.0f), &foot_motion_3, &traj_param, -1.0f);                                                                       
                      }  
                      if(fivebar_inverse(foot_motion_0.P.x, foot_motion_0.P.y, &theta0_out, &theta1_out, true))//左前腿 0，1
                      {
                        go_0_pos = theta1_out;
                        go_1_pos = theta0_out;
                        cmd_0.Pos = output_to_rotor(go_0_pos, &joint_param_0); PosPID_UpdateCmd(&cmd_0, (PI-theta1_out), &Pospid[0]);//正向
                        cmd_1.Pos = output_to_rotor(go_1_pos, &joint_param_1); PosPID_UpdateCmd(&cmd_1, theta0_out, &Pospid[1]);//正向
                      }
                      if(fivebar_inverse(foot_motion_1.P.x, foot_motion_1.P.y, &theta0_out, &theta1_out, true))//右前腿 2，3，  
                      {
                        go_2_pos = theta1_out; 
                        go_3_pos = theta0_out; 
                        cmd_2.Pos = output_to_rotor(go_2_pos, &joint_param_2); PosPID_UpdateCmd(&cmd_2, (PI-theta0_out), &Pospid[2]); //反向                  
                        cmd_3.Pos = output_to_rotor(go_3_pos, &joint_param_3); PosPID_UpdateCmd(&cmd_3, theta1_out, &Pospid[3]);//反向
                      }   
                      if(fivebar_inverse(foot_motion_2.P.x, foot_motion_2.P.y, &theta0_out, &theta1_out, true))//右后腿 4，5
                      {
                        go_4_pos = theta1_out;                
                        go_5_pos = theta0_out;                  
                        cmd_4.Pos = output_to_rotor(go_4_pos, &joint_param_4); PosPID_UpdateCmd(&cmd_4, (PI-theta0_out), &Pospid[4]);//反向                 
                        cmd_5.Pos = output_to_rotor(go_5_pos, &joint_param_5); PosPID_UpdateCmd(&cmd_5, theta1_out, &Pospid[5]);//反向                    
                      }                 
                      if(fivebar_inverse(foot_motion_3.P.x, foot_motion_3.P.y, &theta0_out, &theta1_out, true))//左后腿 6，7  
                      {                       
                        go_7_pos=  theta1_out;
                        go_6_pos = theta0_out;           
                        cmd_6.Pos = output_to_rotor(go_6_pos, &joint_param_6); PosPID_UpdateCmd(&cmd_6, theta0_out, &Pospid[6]);//正向                  
                        cmd_7.Pos = output_to_rotor(go_7_pos, &joint_param_7); PosPID_UpdateCmd(&cmd_7, (PI-theta1_out), &Pospid[7]);//正向                            
                      }     
                               
                  }
                }
                else if (control_mode == 1) 
                {
                  foot_motion_0.output_now_0 = rotor_to_output(data_0.Pos, &joint_param_0);
                  foot_motion_0.output_now_1 = rotor_to_output(data_1.Pos, &joint_param_1);
                  foot_motion_1.output_now_0 = rotor_to_output(data_2.Pos, &joint_param_2);
                  foot_motion_1.output_now_1 = rotor_to_output(data_3.Pos, &joint_param_3);
                  foot_motion_2.output_now_0 = rotor_to_output(data_4.Pos, &joint_param_4);
                  foot_motion_2.output_now_1 = rotor_to_output(data_5.Pos, &joint_param_5);
                  foot_motion_3.output_now_1 = rotor_to_output(data_6.Pos, &joint_param_6);
                  foot_motion_3.output_now_0 = rotor_to_output(data_7.Pos, &joint_param_7);

                  foot_motion_0.omega1 = data_1.W * joint_param_1.dir / joint_param_1.ratio;
                  foot_motion_0.omega2 = data_0.W * joint_param_0.dir / joint_param_0.ratio;
                  foot_motion_0.now_tau1 = data_1.T * joint_param_1.dir * joint_param_1.ratio;
                  foot_motion_0.now_tau2 = data_0.T * joint_param_0.dir * joint_param_0.ratio;
                  foot_motion_1.omega1 = data_3.W * joint_param_3.dir / joint_param_3.ratio;
                  foot_motion_1.omega2 = data_2.W * joint_param_2.dir / joint_param_2.ratio;
                  foot_motion_1.now_tau1 = data_3.T * joint_param_3.dir * joint_param_3.ratio;
                  foot_motion_1.now_tau2 = data_2.T * joint_param_2.dir * joint_param_2.ratio;
                  foot_motion_2.omega1 = data_5.W * joint_param_5.dir / joint_param_5.ratio;
                  foot_motion_2.omega2 = data_4.W * joint_param_4.dir / joint_param_4.ratio;
                  foot_motion_2.now_tau1 = data_5.T * joint_param_5.dir * joint_param_5.ratio;
                  foot_motion_2.now_tau2 = data_4.T * joint_param_4.dir * joint_param_4.ratio;
                  foot_motion_3.omega1 = data_6.W * joint_param_6.dir / joint_param_6.ratio;
                  foot_motion_3.omega2 = data_7.W * joint_param_7.dir / joint_param_7.ratio;
                  foot_motion_3.now_tau1 = data_6.T * joint_param_6.dir * joint_param_6.ratio;
                  foot_motion_3.now_tau2 = data_7.T * joint_param_7.dir * joint_param_7.ratio;
                  if(mode != 3)
                  {
                    jump_reset();
                  }
                  if(mode == 1)
                  {
                    foot_motion_0.G_1 = -25.0f;
                    foot_motion_1.G_1 = -25.0f;
                    foot_motion_2.G_1 = -20.0f;
                    foot_motion_3.G_1 = -20.0f;
                    foot_motion_0.P.x = 0; foot_motion_0.P.y = -0.173f;
                    foot_motion_1.P.x = 0; foot_motion_1.P.y = -0.173f;
                    foot_motion_2.P.x = 0; foot_motion_2.P.y = -0.173f;
                    foot_motion_3.P.x = 0; foot_motion_3.P.y = -0.173f;
                  }
                  if(mode == 2)
                  {
                    if(go_dir == 1 || go_dir == 5 || go_dir == 6||go_dir == 3 || go_dir == 4) 
                    {
                      time += dt; 
                      if(time > traj_param.period) time -= traj_param.period;
                      else if(time < 0) time += traj_param.period;
                    }//前进             
                    else if(go_dir == 2 || go_dir == 7 || go_dir == 8) 
                    {
                      time -= dt;
                      if(time > traj_param.period) time -= traj_param.period;
                      else if(time < 0) time += traj_param.period;
                    }//后退
                    if(time> traj_param.period/2.0f && time < traj_param.period) 
                    {
                      foot_motion_0.motion_state = 1; 
                      foot_motion_1.motion_state = 0;
                      foot_motion_2.motion_state = 1; 
                      foot_motion_3.motion_state = 0;// 0支撑相，1摆动相
                    }
                    else
                    {
                      foot_motion_0.motion_state = 0; 
                      foot_motion_1.motion_state = 1;
                      foot_motion_2.motion_state = 0; 
                      foot_motion_3.motion_state = 1;// 0支撑相，1摆动相
                    }
                    if(foot_motion_0.motion_state == 0) foot_motion_0.G_1 = G_0; else foot_motion_0.G_1 = 0.0f;
                    if(foot_motion_1.motion_state == 0) foot_motion_1.G_1 = G_1; else foot_motion_1.G_1 = 0.0f;
                    if(foot_motion_2.motion_state == 0) foot_motion_2.G_1 = G_2; else foot_motion_2.G_1 = 0.0f;
                    if(foot_motion_3.motion_state == 0) foot_motion_3.G_1 = G_3; else foot_motion_3.G_1 = 0.0f;
                    if(go_dir == 1 || go_dir == 2 || (go_dir >= 5 && go_dir <= 8))
                    {
                      foot_ellipse_trajectory(time, &foot_motion_0, &traj_param);
                      foot_ellipse_trajectory(time+traj_param.period/2.0f, &foot_motion_1, &traj_param);
                      foot_ellipse_trajectory(time, &foot_motion_2, &traj_param);
                      foot_ellipse_trajectory(time+traj_param.period/2.0f, &foot_motion_3, &traj_param);
                    }
                    else if(go_dir == 3)
                    {
                      foot_ellipse_trajectory_dir(time, &foot_motion_0, &traj_param, 1.0f);
                      foot_ellipse_trajectory_dir(time+traj_param.period/2.0f, &foot_motion_1, &traj_param, -1.0f);
                      foot_ellipse_trajectory_dir(time, &foot_motion_2, &traj_param, -1.0f);
                      foot_ellipse_trajectory_dir(time+traj_param.period/2.0f, &foot_motion_3, &traj_param, 1.0f);
                    }
                    else if(go_dir == 4)
                    {
                      foot_ellipse_trajectory_dir(time, &foot_motion_0, &traj_param, -1.0f);
                      foot_ellipse_trajectory_dir(time+traj_param.period/2.0f, &foot_motion_1, &traj_param, 1.0f);
                      foot_ellipse_trajectory_dir(time, &foot_motion_2, &traj_param, 1.0f);
                      foot_ellipse_trajectory_dir(time+traj_param.period/2.0f, &foot_motion_3, &traj_param, -1.0f);
                    }
                    // 计算期望足端速度 (对目标轨迹求导)
                    foot_motion_0.target_vx =  (foot_motion_0.P.x - foot_motion_0.last_P.x) / dt;
                    foot_motion_0.target_vy =  (foot_motion_0.P.y - foot_motion_0.last_P.y) / dt;
                    foot_motion_0.last_P.x = foot_motion_0.P.x;
                    foot_motion_0.last_P.y = foot_motion_0.P.y;
                    foot_motion_1.target_vx =  (foot_motion_1.P.x - foot_motion_1.last_P.x) / dt;
                    foot_motion_1.target_vy =  (foot_motion_1.P.y - foot_motion_1.last_P.y) / dt;
                    foot_motion_1.last_P.x = foot_motion_1.P.x;
                    foot_motion_1.last_P.y = foot_motion_1.P.y;
                    foot_motion_2.target_vx =  (foot_motion_2.P.x - foot_motion_2.last_P.x) / dt;
                    foot_motion_2.target_vy =  (foot_motion_2.P.y - foot_motion_2.last_P.y) / dt;
                    foot_motion_2.last_P.x = foot_motion_2.P.x;
                    foot_motion_2.last_P.y = foot_motion_2.P.y;
                    foot_motion_3.target_vx =  (foot_motion_3.P.x - foot_motion_3.last_P.x) / dt;
                    foot_motion_3.target_vy =  (foot_motion_3.P.y - foot_motion_3.last_P.y) / dt;
                    foot_motion_3.last_P.x = foot_motion_3.P.x;
                    foot_motion_3.last_P.y = foot_motion_3.P.y;             
                  }
                  if(mode == 3)
                  {
                    foot_motion_0.G_1 = 0.0f;
                    foot_motion_1.G_1 = 0.0f;
                    foot_motion_2.G_1 = 0.0f;
                    foot_motion_3.G_1 = 0.0f;
                    jump_update(dt);           
                  }
                  if(fwd_kinematics_and_jacobian(foot_motion_0.output_now_1, foot_motion_0.output_now_0, true, &foot_motion_0.current_P, &foot_motion_0.J)) 
                  {
                    foot_motion_0.raw_vx = foot_motion_0.J.J00 * foot_motion_0.omega1 + foot_motion_0.J.J01 * foot_motion_0.omega2;
                    foot_motion_0.raw_vy = foot_motion_0.J.J10 * foot_motion_0.omega1 + foot_motion_0.J.J11 * foot_motion_0.omega2;
                    foot_motion_0.filtered_vx = VEL_ALPHA * foot_motion_0.raw_vx + (1.0f - VEL_ALPHA) * foot_motion_0.filtered_vx;
                    foot_motion_0.filtered_vy = VEL_ALPHA * foot_motion_0.raw_vy + (1.0f - VEL_ALPHA) * foot_motion_0.filtered_vy;
                    foot_motion_0.Fx = jump_ff_x + foot_motion_0.Kp_x * (foot_motion_0.P.x - foot_motion_0.current_P.x)+ foot_motion_0.Kd_xt * foot_motion_0.target_vx - foot_motion_0.Kd_xr*foot_motion_0.filtered_vx;
                    foot_motion_0.Fy = jump_ff_y + foot_motion_0.G_1 + foot_motion_0.G_0 + foot_motion_0.Kp_y * (foot_motion_0.P.y - foot_motion_0.current_P.y)+ foot_motion_0.Kd_yt * foot_motion_0.target_vy - foot_motion_0.Kd_yr*foot_motion_0.filtered_vy;
                    foot_motion_0.target_tau1 = (foot_motion_0.J.J00 * foot_motion_0.Fx + foot_motion_0.J.J10 * foot_motion_0.Fy)* joint_param_1.dir / joint_param_1.ratio;
                    foot_motion_0.target_tau2 = (foot_motion_0.J.J01 * foot_motion_0.Fx + foot_motion_0.J.J11 * foot_motion_0.Fy)* joint_param_0.dir / joint_param_0.ratio;
                    estimate_foot_force(foot_motion_0.now_tau1, foot_motion_0.now_tau2, &foot_motion_0.J, &foot_motion_0.Fx_real, &foot_motion_0.Fy_real);
                    if(Enable)
                    {
                      cmd_1.T = foot_motion_0.target_tau1;
                      cmd_0.T = foot_motion_0.target_tau2;
                    }
                  } 
                  if(fwd_kinematics_and_jacobian(foot_motion_1.output_now_1, foot_motion_1.output_now_0, true, &foot_motion_1.current_P, &foot_motion_1.J)) 
                  {
                    foot_motion_1.raw_vx = foot_motion_1.J.J00 * foot_motion_1.omega1 + foot_motion_1.J.J01 * foot_motion_1.omega2;
                    foot_motion_1.raw_vy = foot_motion_1.J.J10 * foot_motion_1.omega1 + foot_motion_1.J.J11 * foot_motion_1.omega2;
                    foot_motion_1.filtered_vx = VEL_ALPHA * foot_motion_1.raw_vx + (1.0f - VEL_ALPHA) * foot_motion_1.filtered_vx;
                    foot_motion_1.filtered_vy = VEL_ALPHA * foot_motion_1.raw_vy + (1.0f - VEL_ALPHA) * foot_motion_1.filtered_vy;
                    foot_motion_1.Fx = jump_ff_x + foot_motion_1.Kp_x * (foot_motion_1.P.x - foot_motion_1.current_P.x)+ foot_motion_1.Kd_xt * foot_motion_1.target_vx - foot_motion_1.Kd_xr*foot_motion_1.filtered_vx;
                    foot_motion_1.Fy = jump_ff_y + foot_motion_1.G_1 + foot_motion_1.G_0 + foot_motion_1.Kp_y * (foot_motion_1.P.y - foot_motion_1.current_P.y)+ foot_motion_1.Kd_yt * foot_motion_1.target_vy - foot_motion_1.Kd_yr*foot_motion_1.filtered_vy;
                    foot_motion_1.target_tau1 = (foot_motion_1.J.J00 * foot_motion_1.Fx + foot_motion_1.J.J10 * foot_motion_1.Fy)* joint_param_3.dir / joint_param_3.ratio;
                    foot_motion_1.target_tau2 = (foot_motion_1.J.J01 * foot_motion_1.Fx + foot_motion_1.J.J11 * foot_motion_1.Fy)* joint_param_2.dir / joint_param_2.ratio;
                    estimate_foot_force(foot_motion_1.now_tau1, foot_motion_1.now_tau2, &foot_motion_1.J, &foot_motion_1.Fx_real, &foot_motion_1.Fy_real);
                    if(Enable)
                    {
                      cmd_3.T = foot_motion_1.target_tau1;
                      cmd_2.T = foot_motion_1.target_tau2;
                    }
                  }
                  if(fwd_kinematics_and_jacobian(foot_motion_2.output_now_1, foot_motion_2.output_now_0, true, &foot_motion_2.current_P, &foot_motion_2.J)) 
                  {
                    foot_motion_2.raw_vx = foot_motion_2.J.J00 * foot_motion_2.omega1 + foot_motion_2.J.J01 * foot_motion_2.omega2;
                    foot_motion_2.raw_vy = foot_motion_2.J.J10 * foot_motion_2.omega1 + foot_motion_2.J.J11 * foot_motion_2.omega2;
                    foot_motion_2.filtered_vx = VEL_ALPHA * foot_motion_2.raw_vx + (1.0f - VEL_ALPHA) * foot_motion_2.filtered_vx;
                    foot_motion_2.filtered_vy = VEL_ALPHA * foot_motion_2.raw_vy + (1.0f - VEL_ALPHA) * foot_motion_2.filtered_vy;
                    foot_motion_2.Fx = jump_ff_x + foot_motion_2.Kp_x * (foot_motion_2.P.x - foot_motion_2.current_P.x)+ foot_motion_2.Kd_xt * foot_motion_2.target_vx - foot_motion_2.Kd_xr*foot_motion_2.filtered_vx;
                    foot_motion_2.Fy = jump_ff_y + foot_motion_2.G_1 + foot_motion_2.G_0 + foot_motion_2.Kp_y * (foot_motion_2.P.y - foot_motion_2.current_P.y)+ foot_motion_2.Kd_yt * foot_motion_2.target_vy - foot_motion_2.Kd_yr*foot_motion_2.filtered_vy;
                    foot_motion_2.target_tau1 = (foot_motion_2.J.J00 * foot_motion_2.Fx + foot_motion_2.J.J10 * foot_motion_2.Fy)* joint_param_5.dir / joint_param_5.ratio;
                    foot_motion_2.target_tau2 = (foot_motion_2.J.J01 * foot_motion_2.Fx + foot_motion_2.J.J11 * foot_motion_2.Fy)* joint_param_4.dir / joint_param_4.ratio;
                    estimate_foot_force(foot_motion_2.now_tau1, foot_motion_2.now_tau2, &foot_motion_2.J, &foot_motion_2.Fx_real, &foot_motion_2.Fy_real);
                    if(Enable)
                    {
                      cmd_5.T = foot_motion_2.target_tau1;
                      cmd_4.T = foot_motion_2.target_tau2;
                    }     
                  }
                  if(fwd_kinematics_and_jacobian(foot_motion_3.output_now_1, foot_motion_3.output_now_0, true, &foot_motion_3.current_P, &foot_motion_3.J)) 
                  {
                    foot_motion_3.raw_vx = foot_motion_3.J.J00 * foot_motion_3.omega1 + foot_motion_3.J.J01 * foot_motion_3.omega2;
                    foot_motion_3.raw_vy = foot_motion_3.J.J10 * foot_motion_3.omega1 + foot_motion_3.J.J11 * foot_motion_3.omega2;
                    foot_motion_3.filtered_vx = VEL_ALPHA * foot_motion_3.raw_vx + (1.0f - VEL_ALPHA) * foot_motion_3.filtered_vx;
                    foot_motion_3.filtered_vy = VEL_ALPHA * foot_motion_3.raw_vy + (1.0f - VEL_ALPHA) * foot_motion_3.filtered_vy;
                    foot_motion_3.Fx = jump_ff_x + foot_motion_3.Kp_x * (foot_motion_3.P.x - foot_motion_3.current_P.x)+ foot_motion_3.Kd_xt * foot_motion_3.target_vx - foot_motion_3.Kd_xr*foot_motion_3.filtered_vx;
                    foot_motion_3.Fy = jump_ff_y + foot_motion_3.G_1 + foot_motion_3.G_0 + foot_motion_3.Kp_y * (foot_motion_3.P.y - foot_motion_3.current_P.y)+ foot_motion_3.Kd_yt * foot_motion_3.target_vy - foot_motion_3.Kd_yr*foot_motion_3.filtered_vy;
                    foot_motion_3.target_tau1 = (foot_motion_3.J.J00 * foot_motion_3.Fx + foot_motion_3.J.J10 * foot_motion_3.Fy)* joint_param_6.dir / joint_param_6.ratio;
                    foot_motion_3.target_tau2 = (foot_motion_3.J.J01 * foot_motion_3.Fx + foot_motion_3.J.J11 * foot_motion_3.Fy)* joint_param_7.dir / joint_param_7.ratio;
                    estimate_foot_force(foot_motion_3.now_tau1, foot_motion_3.now_tau2, &foot_motion_3.J, &foot_motion_3.Fx_real, &foot_motion_3.Fy_real);
                    if(Enable)
                    {
                      cmd_7.T = foot_motion_3.target_tau2;
                      cmd_6.T = foot_motion_3.target_tau1;
                    }
                  }
                }             
            }
            else if (run == 0) 
            {
                jump_start_req = 0;
                jump_reset();
                cmd_0.Pos = joint_param_0.rotor_zero; PosPID_UpdateCmd(&cmd_0, cmd_0.Pos, &Pospid[0]);
                cmd_1.Pos = joint_param_1.rotor_zero; PosPID_UpdateCmd(&cmd_1, cmd_1.Pos, &Pospid[1]);
                cmd_2.Pos = joint_param_2.rotor_zero; PosPID_UpdateCmd(&cmd_2, cmd_2.Pos, &Pospid[2]);
                cmd_3.Pos = joint_param_3.rotor_zero; PosPID_UpdateCmd(&cmd_3, cmd_3.Pos, &Pospid[3]);
                cmd_4.Pos = joint_param_4.rotor_zero; PosPID_UpdateCmd(&cmd_4, cmd_4.Pos, &Pospid[4]);
                cmd_5.Pos = joint_param_5.rotor_zero; PosPID_UpdateCmd(&cmd_5, cmd_5.Pos, &Pospid[5]);
                cmd_6.Pos = joint_param_6.rotor_zero; PosPID_UpdateCmd(&cmd_6, cmd_6.Pos, &Pospid[6]);
                cmd_7.Pos = joint_param_7.rotor_zero; PosPID_UpdateCmd(&cmd_7, cmd_7.Pos, &Pospid[7]);
            }
            if(Enable == 0)
            {
                PosPID_UpdateCmd(&cmd_0, cmd_0.Pos, &Pospid[0]);
                PosPID_UpdateCmd(&cmd_1, cmd_1.Pos, &Pospid[1]);
                PosPID_UpdateCmd(&cmd_2, cmd_2.Pos, &Pospid[2]);
                PosPID_UpdateCmd(&cmd_3, cmd_3.Pos, &Pospid[3]);
                PosPID_UpdateCmd(&cmd_4, cmd_4.Pos, &Pospid[4]);
                PosPID_UpdateCmd(&cmd_5, cmd_5.Pos, &Pospid[5]);
                PosPID_UpdateCmd(&cmd_6, cmd_6.Pos, &Pospid[6]);
                PosPID_UpdateCmd(&cmd_7, cmd_7.Pos, &Pospid[7]);
            }
        }
        last_mode = mode;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    RS485_TxCpltHandler(&rs485, huart);
    cnt_tx++;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == USART2 || huart->Instance == USART3)
    {
      RS485_RxCpltHandler(&rs485, huart, Size);
    }
    else if(huart->Instance == UART5)
    {
      CRSF_Init();
      CRSF_AcceptData();
    }
    else if(huart->Instance == USART1)
    {
      COM_UART_Init();
      COM_UART_Handle();
    }
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
