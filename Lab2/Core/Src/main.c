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
#include "eeprom.h"


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
RNG_HandleTypeDef hrng;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5555,0x6666,0x7777};
uint16_t best_time = 0;// To store the best time to store in eeprom
char lcd_buffer[6]; // Used to display content to the LCD screen
uint16_t current_score = 0; //To store the current reaction time of the user
uint16_t LED_STATUS = 1; // to toggle between on and off states





/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_RNG_Init(void);
void Check_Best_Time(void); // Check if current time is better than value stored in EEPROM
void Show_Time(uint16_t);   // Display Current Reaction time of user
uint16_t GetRandomNumber(void); //Get random number for the wait time (ms)
void Display_Best_Time(void); //Display current best time stored in EEPROM



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
	
	//EEPROM INIT
	EE_Init();
	EE_WriteVariable(VirtAddVarTab[2],9999);
	EE_ReadVariable(VirtAddVarTab[2],&best_time); //Practice writting to EEPROM

  /* USER CODE BEGIN Init */
	BSP_LCD_GLASS_Init();
	BSP_LCD_GLASS_DisplayString((uint8_t*)"MT3TA4");
	BSP_JOY_Init(JOY_MODE_EXTI);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_RNG_Init();
	

	
	
  /* USER CODE BEGIN 2 */
HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1); //USED FOR GETTING BEST TIME SCORE
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RNG;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
	__HAL_RCC_PWR_CLK_ENABLE();
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
	__HAL_RCC_PWR_CLK_DISABLE();
}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3999; //Counts at every milisecond
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
	if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) //Enables timer for tim2
	{
		Error_Handler();
	}
  /* USER CODE END TIM2_Init 2 */

}
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
	// Frequency =  CLK_FREQUENCY /[(PSC+1)(Period+1)] = 1Hz
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 399; // 
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999; // 
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
	if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */


static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
	// Have Tim4 counter clock to 1kHz --> PSC to 3999
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 3999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999999; // Have tim4 count from 0 -> 99999ms
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 9999;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim4, TIM_CHANNEL_1);
  /* USER CODE BEGIN TIM4_Init 2 */
	if (HAL_TIM_OC_Start_IT(&htim4,TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
  /* USER CODE END TIM4_Init 2 */

}


void Check_Best_Time()// Compare best time in EEPROM to the score by the current user
{
	EE_ReadVariable(VirtAddVarTab[2],&best_time);
	if (current_score < best_time)
		EE_WriteVariable(VirtAddVarTab[2],current_score);
}

void Show_Time(uint16_t time)
{
	sprintf(lcd_buffer,"%d",time);
	BSP_LCD_GLASS_Clear();
	BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
}
void Display_Best_Time(void)
{
	EE_ReadVariable(VirtAddVarTab[2],&best_time);
	sprintf(lcd_buffer,"%d",best_time);
	BSP_LCD_GLASS_Clear();
	BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	BSP_LED_Init(LED5);
	BSP_LED_Init(LED4);
}

/* USER CODE BEGIN 4 */
uint16_t capture_time_val = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
	if (htim->Instance == TIM4)
	{
		capture_time_val = __HAL_TIM_GetCounter(htim); // This will get the time once the selection button is pressed
		__HAL_TIM_SetCounter(htim,0); //Set the timer back to zero
	}
}



uint16_t GetRandomNumber() //Get random number from 0->4 as the time period
{
	uint16_t random_num = (HAL_RNG_GetRandomNumber(&hrng));
	random_num = random_num%4000; // limit the random number to values between 0->4000 as described by lab manual
	sprintf(lcd_buffer,"%d",random_num);	
	return random_num;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/*Control the state of the LEDs during the game */	
	if (LED_STATUS == 1) // Toggle at 1HZ
	{
		BSP_LED_Toggle(LED_RED);
		BSP_LED_Toggle(LED_GREEN);
	}
	else if (LED_STATUS == 2) //Keep LED ON 
	{
		BSP_LED_On(LED_RED);
		BSP_LED_On(LED_GREEN);
	}
	else //Turn off the LEDs during the wait time
	{
		BSP_LED_Off(LED_RED);
		BSP_LED_Off(LED_GREEN);
	}
	
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{	
	uint32_t wait_time = 0;
	wait_time = GetRandomNumber();
	wait_time+= 2000; // make sure that the wait time will be at least 2000ms (2 seconds)

	
	switch(GPIO_Pin)
	{ 
		case GPIO_PIN_0: // Select Button This will stop the game and display the reaction time
			HAL_TIM_IC_CaptureCallback(&htim4); // When button selection button is pressed the time value will be stored
			
			Show_Time(capture_time_val);
			current_score = capture_time_val; //Current_score is global variable used to store current score for Check_Best_Time() function
		
			Check_Best_Time(); //Check current score with what is stored in EEPROM and update if neccesary
		
			LED_STATUS = 1; 
			break;
		
		case GPIO_PIN_1: // Left Pin will reset

			LED_STATUS = 1;
			BSP_LCD_GLASS_Clear();
			BSP_LCD_GLASS_DisplayString((uint8_t*)"Reset");
		
			break;
			
		case GPIO_PIN_2: // Right	Pin Will start the game.
			/*
			Have Starting display on screen for random amount of time.
			Will start counting the reaction time after.
			*/
			LED_STATUS = 0;
			BSP_LCD_GLASS_Clear();
			__HAL_TIM_SetCounter(&htim4,0x0000);
			while (1)
			{
				BSP_LCD_GLASS_DisplayString((uint8_t*)"Wait");
				
				/*
				Implementing the Cheat state of the system
				
				if (__HAL_TIM_GetCounter(&htim4) < wait_time && GPIO_PIN_0)
				{
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t*)"Cheat");
				}
				*/
				
				if (__HAL_TIM_GetCounter(&htim4) > wait_time)
					break;
			}
			__HAL_TIM_SetCounter(&htim4,0x0000);
			LED_STATUS = 2; // Keep the LEDs on while the timer counts
			/* 
			This while loop will not break once the Selection Button is pressed.
			Planned Outcome: Once Selection button is pressed, the case will switch to the GPIO_PIN_0
											 case and display the user's reaction time
			Actual Outcome: Timer 4 counter resets, and counts up from zero once the slection button is 
											pressed.
			Reason why this might happen: __HAL_TIM_SetCounter(&htim4,0x0000) resets the tim4 counter to 0.
																		which is probably the reason why the counter resets.
			
			Nonetheless in case GPIO_PIN_0 if the LED_STATUS is changed (0 or 1) then the LED's state changes
			as well when selection button is pressed during counting process. Therefore the EXTI_IRQHandler() must
			trigger the GPIO_PIN_0. 
			For example: change the LED_STATUS in cas GPIO_PIN_0 to 0,1 or 2 and notice the changes when the 
			interrupt is called in the while loop.
			*/
			while (1)
			{
				sprintf(lcd_buffer,"%d",__HAL_TIM_GetCounter(&htim4));
				BSP_LCD_GLASS_Clear();
				BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
				HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
			}
			break;
		
		case GPIO_PIN_3: // Up	Pin will bring you back to main screen of the game
			BSP_LCD_GLASS_Clear();
			BSP_LCD_GLASS_DisplayString((uint8_t*)"Lab 2");
			LED_STATUS = 1;
			break;
		
		case GPIO_PIN_5: // Down Pin Display current record of fastest time
			LED_STATUS = 1;
			Display_Best_Time();
			break;
		
		default:
			break;
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
    