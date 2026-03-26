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
#include "pid.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gom_protocol.h"
#include "control.h"
#include "motion.h"
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "my_system.h"
#include "pid.h"
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

RS485_Scheduler_t rs485;
FootTrajParam traj_param;
Point2D P;
JointParam joint_param_0, joint_param_1;
PosPID_t Pospid[8];
float time = 0.0f;
float dt = 0.1f;
float theta0_out = 0, theta1_out = 0;
uint8_t start = 0,start_1 = 0;
float rotor_now_0, rotor_now_1;
float output_now_0, output_now_1;

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
  /* USER CODE BEGIN 2 */
  DWT_Init();
  HAL_Delay(500);
  P.x = 0.0f;
  P.y = 0.0f;

  joint_param_0.rotor_zero = 0;// 根据实际安装调整零位
  joint_param_0.output_zero = 0;// 根据实际安装调整零位
  joint_param_0.ratio = 6.33f;
  joint_param_0.dir = -1;

  joint_param_1.rotor_zero = 0.0f;// 根据实际安装调整零位
  joint_param_1.output_zero = 0;// 根据实际安装调整零位
  joint_param_1.ratio = 6.33f;
  joint_param_1.dir = -1;

  traj_param.step_length  = 40.0f;
  traj_param.step_height  = 25.0f;
  traj_param.stand_height = -150.0f;
  // traj_param.period       = 0.6f;
  traj_param.period       = 60.0f;

  rs485.current_motor = 0;
  rs485.tx_busy = 0;
  rs485.rx_ready = 1;
  rs485.waiting_rx = 0;
  rs485.rx_start_tick = 0;
  rs485.huart_ch1 = &huart2;
  rs485.huart_ch2 = &huart3;
  RS485_SetRxTimeout(&rs485, 10U);
  // RS485_SetMotorMask(&rs485, RS485_ALL_MOTOR_MASK); // 改这里可选择本次参与读写的电机
  RS485_SetMotorMask(&rs485, (1U << 0) | (1U << 1)); // 只读写电机0和1

  //RS485_1
  cmd_0.id = 0;     cmd_1.id = 1;     cmd_2.id = 2;    cmd_3.id = 3;
  cmd_0.mode = 1;   cmd_1.mode = 1;   cmd_2.mode = 1;  cmd_3.mode = 1;
  cmd_0.K_P = 0;    cmd_1.K_P = 0;    cmd_2.K_P = 0.7;   cmd_3.K_P = 0.7;
  cmd_0.K_W = 0;  cmd_1.K_W = 0;  cmd_2.K_W = 0.1; cmd_3.K_W = 0.1;
  cmd_0.Pos = 0;    cmd_1.Pos = 0;    cmd_2.Pos = 0;   cmd_3.Pos = 0;
  cmd_0.W = 0;      cmd_1.W = 0;      cmd_2.W = 0;     cmd_3.W = 0;
  cmd_0.T = 0;      cmd_1.T = 0;      cmd_2.T = 0;     cmd_3.T = 0;
  //RS485_2
  cmd_4.id = 4;     cmd_5.id = 5;     cmd_6.id = 6;    cmd_7.id = 7;
  cmd_4.mode = 1;   cmd_5.mode = 1;   cmd_6.mode = 1;  cmd_7.mode = 1;
  cmd_4.K_P = 0.7;    cmd_5.K_P = 0.7;    cmd_6.K_P = 0.7;   cmd_7.K_P = 0.7;
  cmd_4.K_W = 0.1;  cmd_5.K_W = 0.1;  cmd_6.K_W = 0.1; cmd_7.K_W = 0.1;
  cmd_4.Pos = 0;    cmd_5.Pos = 0;    cmd_6.Pos = 0;   cmd_7.Pos = 0;
  cmd_4.W = 0;      cmd_5.W = 0;      cmd_6.W = 0;     cmd_7.W = 0;
  cmd_4.T = 0;      cmd_5.T = 0;      cmd_6.T = 0;     cmd_7.T = 0;
  HAL_TIM_Base_Start_IT(&htim1); // 启动定时器1的中断，定时器1的周期由cubemx设置，这里是1ms
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    RS485_Schedule(&rs485);
    rotor_now_0 = data_0.Pos;
    rotor_now_1 = data_1.Pos;
    if(start)
    {
        // output_now_0 = rotor_to_output(rotor_now_0, &joint_param_0);
        // wrap_pi_fast(&output_now_0);
        // output_now_1 = rotor_to_output(rotor_now_1, &joint_param_1);
        // wrap_pi_fast(&output_now_1);
        PosPID_Init(&Pospid[0], 0.12f, 0.0f, 0.004f, 0, 0.001f);
        
        PosPID_Init(&Pospid[1], 0.12f, 0.0f, 0.004f, 0, 0.001f);
        // cmd_0.K_P = 0.1f;    cmd_1.K_P = 0.1f;
        // cmd_0.K_W = 0.003f;  cmd_1.K_W = 0.003f;
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
        if(start_1)
        {
            if(count < 1000)count++;
            // if(count >= 10.f) 
            if(count>= 1000&&start==0)
            {
                start = 1;
                joint_param_0.rotor_zero = data_0.Pos;
                joint_param_1.rotor_zero = data_1.Pos;
                time = 0.0f;
                start_1 = 0;
            }
        }
        if(start)
        {
            time += dt;
            if(time > traj_param.period) time -= traj_param.period;
            foot_ellipse_trajectory(time, &traj_param, &P.x, &P.y);
            fivebar_inverse(P.x, P.y, &theta0_out, &theta1_out, true);
            cmd_0.Pos = output_to_rotor(theta0_out, &joint_param_0);
            PosPID_UpdateCmd(&cmd_0, theta0_out, &Pospid[0]);
            cmd_1.Pos = output_to_rotor((PI-theta1_out), &joint_param_1);
            PosPID_UpdateCmd(&cmd_1, (PI-theta1_out), &Pospid[1]);
        }
        
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    RS485_TxCpltHandler(&rs485, huart);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    RS485_RxCpltHandler(&rs485, huart);
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
