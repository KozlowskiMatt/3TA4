/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l476g_discovery.h"
#include "stm32l476g_discovery_glass_lcd.h"
#include "stm32l4xx_it.h"
#include "stm32l4xx_hal_conf.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// *** The state variables for the FSM *** //
typedef enum{
	Full_Step=1,
	Half_Step = 2
	
}Steps;

typedef enum{
	Increase_Speed = 1,
	Decrease_Speed = 2
}Speeds;

typedef enum{
	CW = 1,
	CCW = 2
}Orientations;


Steps Step =1; // Set default Step to Full-step-drive
Speeds Speed = 1; // Set default speed
Orientations Orientation = 1; // Set default orientation to CW

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

// *** Variables to dictate which joystick button is pressed ***//
uint8_t Selection_Pressed = 0;
uint8_t Right_Pressed = 0;
uint8_t Left_Pressed = 0;
uint8_t Up_Pressed = 0;
uint8_t Down_Pressed = 0;

int full_step = 0; // full step variable to represent which step of the full-step-drive the motor is in
int half_step = 0; //half step variable to represent which step of the half-step-drive the motor is in

char lcd_buffer[6]; // To change the LCD Screen string
uint16_t pins[4] = {OUT_A_Pin, OUT_B_Pin, OUT_C_Pin, OUT_D_Pin}; // To represent the pins of the motor
	



/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
void Change_Orientation(void); // To change the Orientation CW or CCW of the motor
void Change_Speed(void); // To change the speed of the motor (increase or decrease)
void HalfStep(uint8_t); // Function to drive Half Step motor rotations
void FullStep(uint8_t); // Finction to drive the Full Step motor roatations

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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	BSP_LCD_GLASS_Init();
	BSP_LED_Init(LED4);
	BSP_JOY_Init(JOY_MODE_EXTI);
	
	BSP_LCD_GLASS_Clear();
	BSP_LCD_GLASS_DisplayString((uint8_t*)"Lab 5");
	HAL_Delay(500);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { 	//***								For testing the pins							***//
			//HAL_GPIO_WritePin(GPIOE, pins[0], GPIO_PIN_RESET);   // IN1
			 /// HAL_GPIO_WritePin(GPIOE, pins[1], GPIO_PIN_RESET);   // IN2
			  //HAL_GPIO_WritePin(GPIOE, pins[2], GPIO_PIN_RESET);   // IN3
			  //HAL_GPIO_WritePin(GPIOE, pins[3], GPIO_PIN_SET);   // IN4
		
		// ** Statements to determine what occurs when a button is pressed **//
		
		if (Selection_Pressed ==1) // To swtich between Full and Half Stepping
		{
			Selection_Pressed = 0;
			
			if (Step == Full_Step)
			{
				Step = Half_Step;
			}
			
			else if (Step == Half_Step)
			{
				Step = Full_Step;
			}
		}
		
		
		if (Right_Pressed == 1) // To have stepper motor change to CW
		{
			Right_Pressed = 0;

			if (Orientation == CCW)
			{
				Orientation = CW;
				Change_Orientation();
			}
		}
		
		
		if (Left_Pressed == 1) // To have stepper motor change to CCW
		{
			Left_Pressed = 0;
			if (Orientation == CW)
			{
				Orientation = CCW;
				Change_Orientation();
			}
		}
		
		
		if (Up_Pressed ==1) // To increase the speed
		{
			// Increase the speed of the motor
			Up_Pressed = 0;
			if (Speed == Decrease_Speed)
			{
				Speed = Increase_Speed;
				Change_Speed();
			}
		}
		
		if (Down_Pressed ==1) //To decrease the speed
		{
			// Decrease the speed of the motor
			Down_Pressed = 0;
			if (Speed == Increase_Speed)
			{
				Speed = Decrease_Speed;
				Change_Speed();
			}
		}
		
    // ********* Switch Statements that correspond to the FSM // ********* 
		
		switch (Step)
		{
			case (Full_Step): //To have the motor in Full Step mode
			{
				FullStep(full_step);
				full_step = (full_step+1)%4;
				if (Orientation == CW)
				{
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t*)"CW F");
				}
				
				else if (Orientation == CCW)
				{
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t*)"CCW F");
				}
					
				break;
			}
			
			case (Half_Step): // To have the motor in Half Step mode
			{
				HalfStep(half_step);
				half_step = (half_step+1)%8;
				if (Orientation == CW)
				{
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t*)"CW H");
				}
				
				else if (Orientation == CCW)
				{
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t*)"CCW H");
				}
					
				break;
			}
		}
	}
	

		
			/* USER CODE END WHILE */
			
}
    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
	
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 499;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 5999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OUT_A_Pin|OUT_B_Pin|OUT_C_Pin|OUT_D_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : JOY_CENTER_Pin */
  GPIO_InitStruct.Pin = JOY_CENTER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(JOY_CENTER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : JOY_LEFT_Pin JOY_RIGHT_Pin JOY_UP_Pin JOY_DOWN_Pin */
  GPIO_InitStruct.Pin = JOY_LEFT_Pin|JOY_RIGHT_Pin|JOY_UP_Pin|JOY_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT_A_Pin OUT_B_Pin OUT_C_Pin OUT_D_Pin */
  GPIO_InitStruct.Pin = OUT_A_Pin|OUT_B_Pin|OUT_C_Pin|OUT_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
//*** To change the Direction of the motor ***//
void Change_Orientation(void)
{
	if (Orientation == CW) // Arranging CW Pin Arrangement
	{
		pins[0] = OUT_A_Pin;
		pins[1] = OUT_B_Pin;
		pins[2] = OUT_C_Pin;
		pins[3] = OUT_D_Pin;
	}
	
	else if (Orientation == CCW) // Arranging CCW pin Arrangement
	{
		pins[0] = OUT_D_Pin;
		pins[1] = OUT_C_Pin;
		pins[2] = OUT_B_Pin;
		pins[3] = OUT_A_Pin;
	}
}

void Change_Speed(void)
{
	if (Speed == Increase_Speed)
		htim3.Instance->ARR =2999; //Reducing ARR values leads to higher frequecny --> Shorter Period
	
	else if (Speed == Decrease_Speed) //Increasing ARR value leads to lower frequency --> Longer Period
		htim3.Instance->ARR =5999;
}

//*** For Full Step Mode ***//
void FullStep(uint8_t full_step)
{
	switch (full_step) 
	{
		case 0: //1,0,1,0
			HAL_GPIO_WritePin(GPIOE, pins[0], GPIO_PIN_SET);   // IN1
			HAL_GPIO_WritePin(GPIOE, pins[1], GPIO_PIN_RESET);   // IN2
			HAL_GPIO_WritePin(GPIOE, pins[2], GPIO_PIN_SET);   // IN3
			HAL_GPIO_WritePin(GPIOE, pins[3], GPIO_PIN_RESET);   // IN4
			break;
		
		case 1: //1,0,0,1
			  HAL_GPIO_WritePin(GPIOE, pins[0], GPIO_PIN_SET);   // IN1
			  HAL_GPIO_WritePin(GPIOE, pins[1], GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(GPIOE, pins[2], GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(GPIOE, pins[3], GPIO_PIN_SET);   // IN4
			  break;
		
		case 2: //0,1,0,1
			  HAL_GPIO_WritePin(GPIOE, pins[0], GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(GPIOE, pins[1], GPIO_PIN_SET);   // IN2
			  HAL_GPIO_WritePin(GPIOE, pins[2], GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(GPIOE, pins[3], GPIO_PIN_SET);   // IN4
			  break;
		
		case 3: //0,1,1,0
			  HAL_GPIO_WritePin(GPIOE, pins[0], GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(GPIOE, pins[1], GPIO_PIN_SET);   // IN2
			  HAL_GPIO_WritePin(GPIOE, pins[2], GPIO_PIN_SET);   // IN3
			  HAL_GPIO_WritePin(GPIOE, pins[3], GPIO_PIN_RESET);   // IN4
			  break;
	}
}

// *** For Half Step Mode ***//
void HalfStep(uint8_t half_step)
{
	switch (half_step)
	{
		case 0: // 1,0,1,0
			HAL_GPIO_WritePin(GPIOE, pins[0], GPIO_PIN_SET);   // IN1
			HAL_GPIO_WritePin(GPIOE, pins[1], GPIO_PIN_RESET);   // IN2
			HAL_GPIO_WritePin(GPIOE, pins[2], GPIO_PIN_SET);   // IN3
			HAL_GPIO_WritePin(GPIOE, pins[3], GPIO_PIN_RESET);   // IN4
			break;
		
		case 1: //1,0,0,0
			  HAL_GPIO_WritePin(GPIOE, pins[0], GPIO_PIN_SET);   // IN1
			  HAL_GPIO_WritePin(GPIOE, pins[1], GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(GPIOE, pins[2], GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(GPIOE, pins[3], GPIO_PIN_RESET);   // IN4
			  break;
		
		case 2: //1,0,0,1
			  HAL_GPIO_WritePin(GPIOE, pins[0], GPIO_PIN_SET);   // IN1
			  HAL_GPIO_WritePin(GPIOE, pins[1], GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(GPIOE, pins[2], GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(GPIOE, pins[3], GPIO_PIN_SET);   // IN4
			  break;
		
		case 3: //0,0,0,1
			  HAL_GPIO_WritePin(GPIOE, pins[0], GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(GPIOE, pins[1], GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(GPIOE, pins[2], GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(GPIOE, pins[3], GPIO_PIN_SET);   // IN4
			  break;
		
		case 4: //0,1,0,1
			HAL_GPIO_WritePin(GPIOE, pins[0], GPIO_PIN_RESET);   // IN1
			HAL_GPIO_WritePin(GPIOE, pins[1], GPIO_PIN_SET);   // IN2
			HAL_GPIO_WritePin(GPIOE, pins[2], GPIO_PIN_RESET);   // IN3
			HAL_GPIO_WritePin(GPIOE, pins[3], GPIO_PIN_SET);   // IN4
			break;
		
		case 5: //0,1,0,0
			  HAL_GPIO_WritePin(GPIOE, pins[0], GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(GPIOE, pins[1], GPIO_PIN_SET);   // IN2
			  HAL_GPIO_WritePin(GPIOE, pins[2], GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(GPIOE, pins[3], GPIO_PIN_RESET);   // IN4
			  break;
		
		case 6: //0,1,1,0
			  HAL_GPIO_WritePin(GPIOE, pins[0], GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(GPIOE, pins[1], GPIO_PIN_SET);   // IN2
			  HAL_GPIO_WritePin(GPIOE, pins[2], GPIO_PIN_SET);   // IN3
			  HAL_GPIO_WritePin(GPIOE, pins[3], GPIO_PIN_RESET);   // IN4
			  break;
		
		case 7: //0,0,1,0
			  HAL_GPIO_WritePin(GPIOE, pins[0], GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(GPIOE, pins[1], GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(GPIOE, pins[2], GPIO_PIN_SET);   // IN3
			  HAL_GPIO_WritePin(GPIOE, pins[3], GPIO_PIN_RESET);   // IN4
			  break;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin)
	{
		case (GPIO_PIN_0): //Selection
			Selection_Pressed = 1;
			break;
		
		case (GPIO_PIN_1): //Selection
			Left_Pressed = 1;
			break;
		
		case (GPIO_PIN_2): //Right
			Right_Pressed = 1;
			break;
		
		case (GPIO_PIN_3): //Up
			Up_Pressed = 1;
			break;
		
		case (GPIO_PIN_5): //Down
			Down_Pressed = 1;
			break;
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3) // When timer interrupt occurs the motor will transitiom to its next state.
	{
		if (Step == Full_Step)
		{
			FullStep(Full_Step);
			full_step = (full_step+1)%4; // Full step only has 4 steps 0->3
		}
		
		else if (Step == Half_Step)
		{
			HalfStep(half_step);
			half_step = (half_step+1)%8; // Half-drive has 8 steps 0->7
		}
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
