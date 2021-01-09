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
#include "i2c_at24c64.h"
#include "stm32l4xx_it.h"
#include "stm32l4xx_hal_conf.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EEPROM_ADDRESS 0xA0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

RTC_DateTypeDef RTC_DateStruct;
RTC_TimeTypeDef RTC_TimeStruct;

/* USER CODE BEGIN PV */
__IO uint16_t memLocation = 0x000A;
uint16_t EE_status;

char lcd_buffer[6]; // LCD Display Buffer
char timestr[9]; // To store the time Hours:Minutes:Seconds as a string 
char datestr[6]; //To store the date Day/Month/Year as a string
char get_time[6]; //Used to get the past times stored in the EEPROM
char *DATE[]={"Wk","Da","Mo","Yr"}; //Used for nice LCD Display
char *TIME[]={"Hr","Min","Sec"}; //Used for nice LCD Display
int date_increment = 0; //Used to increment UserDate and DATE while getting input from the user
int time_increment = 0; //Used to increment UserTime and TIME while getting input from the user

uint8_t param = 0;




uint8_t Day, Weekday, Month, Year; 
uint8_t Hour,Minutes,Seconds;

uint8_t UserDate[] ={0,0,0,0}; // To store the user defined Date - Weekday, Day, Month, Year
uint8_t UserTime[] = {0,0,0}; //To store the User defined time --> Hour, Minutes, Seconds
uint16_t UserDate_MAX[] = {7,31,12,40}; // Set the maximum date values (weekday, day, month, year(2040)
uint8_t UserTime_MAX[] = {24,60,60}; // Set the maximum time values (hours, minutes, seconds)

__IO uint32_t Selection_Pressed_Start_time = 0; //Used to help determine if the selection button is being held down


//// Read variables used to help with reading Hour,Minute,Seconds values from the EEPROM ////
uint8_t readHourLast, readMinuteLast, readSecondsLast;
uint8_t readHourSecond_Last, readMinuteSecond_Last, readSecondsSecond_Last;

uint8_t MemoryIndex = 0; //To store the amount of times data is written to the EEPROM


//// Varaibles used if button is pressed   ////
uint8_t Selection_Pressed = 0;
uint8_t Right_Pressed = 0;
uint8_t Left_Pressed = 0;


////   State Variables   ////
typedef enum 
{
	Display_Time = 0,
	Display_Date = 1,
	Store_Time = 2,
	Display_Last_Time = 3,
	Time_Setting = 4,
	Date_Setting = 5
	
}state_variables;


state_variables state = Display_Time; // Have display time as initial state (have running RTC on LCD screen)



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);

/* USER CODE BEGIN PFP */
void RTC_TimeDisplay(void); // Used to display time onto the LCD Screen
void Get_Date_Time(void); // Used to retrieve the current date and time from the RTC 

/// Functions used to help during the user input settings  ////
void Update_Date_Time(void); //Update the date/time from users input
void Current_Date_Time(void); // Get the date/time that the user is inputting



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
	uint8_t temp_index = 0; // Temporary variable to help with going through EEPROM addresses
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	BSP_LCD_GLASS_Init();
	BSP_JOY_Init(JOY_MODE_EXTI);
	BSP_LED_Init(LED4);
	
	BSP_LCD_GLASS_Clear();
	BSP_LCD_GLASS_DisplayString((uint8_t*)"Lab 3");
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
//*********************Testing I2C EEPROM------------------
	//the following variables are for testging I2C_EEPROM
		/*uint8_t data1 = 0x80,  data2=0x81;
	uint8_t readData=0x00;
	
	EE_status=I2C_ByteWrite(&hi2c1,EEPROM_ADDRESS, memLocation, data1);

  if(EE_status != HAL_OK)
  {
    I2C_Error(&hi2c1);
  }
	
	BSP_LCD_GLASS_Clear();
	if (EE_status==HAL_OK) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 1 ok");
	}else
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 1 X");

	HAL_Delay(1000);
	
	EE_status=I2C_ByteWrite(&hi2c1,EEPROM_ADDRESS, memLocation+1 , data2);////////////////////////////////
	
  if(EE_status != HAL_OK)
  {
    I2C_Error(&hi2c1);
  }
	
	BSP_LCD_GLASS_Clear();
	if (EE_status==HAL_OK) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 2 ok");
	}else
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 2 X");

	HAL_Delay(1000);
	
	readData=I2C_ByteRead(&hi2c1,EEPROM_ADDRESS, memLocation); 

	BSP_LCD_GLASS_Clear();
	if (data1 == readData) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"r 1 ok");;
	}else{
			BSP_LCD_GLASS_DisplayString((uint8_t*)"r 1 X");
	}	
	
	HAL_Delay(1000);
	
	readData=I2C_ByteRead(&hi2c1,EEPROM_ADDRESS, memLocation+1); 

	BSP_LCD_GLASS_Clear();
	if (data2 == readData) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"r 2 ok");;
	}else{
			BSP_LCD_GLASS_DisplayString((uint8_t *)"r 2 X");
	}	

	HAL_Delay(1000);
	
	BSP_LCD_GLASS_Clear();
	sprintf(lcd_buffer,"%d",readData);
	BSP_LCD_GLASS_DisplayString((uint8_t*) lcd_buffer);*/
	

  //******************************Testing I2C EEPROM*****************************
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //  To determine whether the selction button is being held down  //
		if (BSP_JOY_GetState() == JOY_SEL)
		{
			Selection_Pressed_Start_time = HAL_GetTick();
			while (BSP_JOY_GetState() == JOY_SEL)
			{
				if ((HAL_GetTick() - Selection_Pressed_Start_time) > 1000) // If the Selection button is held down for more than 1 second
				{
					state = Display_Date;
				}
				
			}
		}
						// Instances where the slection button is pressed *NOT HELD DOWN* //
		if (Selection_Pressed == 1)
		{
			Selection_Pressed = 0;
			
						///   Store the current time if the timer is on the LCD Screen   ///
			if (state == Display_Time)
				state = Store_Time;
			
						//Increment the time each instance the seleciton button is pressed
			else if (state == Time_Setting)
				UserTime[time_increment] = (UserTime[time_increment] % UserTime_MAX[time_increment])+1;// %UsetTime_MAX so that time value is not greater then it should (ex. having hours =26 is not reasonable
					
			
						// Increment the date variable each time the selection button is pressed
			else if (state == Date_Setting)
				UserDate[date_increment] = (UserDate[date_increment] %UserDate_MAX[date_increment])+1; // %UsetDate_MAX so that time value is not greater then it should (ex. having seconds =100 is not reasonable
			
			else
				break;
		}
			//////////////////////////////////////////////////////////////////////////
		
								// Instance where the left button is pressed //
		if (Left_Pressed == 1)
		{
			Left_Pressed=0;
							// While LCD Shows the time, left button will display the recent stored times
			if (state == Display_Time)
				state = Display_Last_Time;
			
							// While User is changing the time, left button will change between Hour -> Min -> Second parameters //
			else if (state == Time_Setting)
				time_increment = (time_increment+1)%3; // %3 so that indexing UserTime will not cause issues (OverIndexing)
			
						// While User is changing the date, left button will change between Weekday -> Day -> Month --> Year parameters //
			else if (state == Date_Setting)
				date_increment = (date_increment +1) %4; // %4 so that indexing UserDate will not cause issues (OverIndexing)

			else break;
		}
		///////////////////////////////////////////////////////////////////////////////////
		
		// Instance where the right button is pressed //
		if (Right_Pressed == 1)
		{
			Right_Pressed=0;
			//    First time going into the Time setting mode   //
			if (state == Display_Time)
			{
				time_increment =0; //Ensure that user will set time from Hour ->Min -> Second each time they jump into this state
				
				Get_Date_Time(); //Get current values
				Current_Date_Time(); //Set userDate and userTime values to what is currently stored in date/time variables for chanigng values during runtime
				Update_Date_Time(); //Update the time in RTC with what the user is inputting
				
				state = Time_Setting;
			}
			
			//   First time going into the Date Setting mode   //
			else if (state == Time_Setting)
			{
				date_increment =0; //Ensure that user will set date from Weekday ->Day -> Month -->Year each time they jump into this state
				
				Update_Date_Time(); //Update the time in RTC with what the user is inputting
				Get_Date_Time(); //Get current values
				Current_Date_Time(); //Set userDate and userTime values to what is currently stored in date/time variables for chanigng values during runtime
				
				state = Date_Setting;
			}
			// While in date setting mode if right button pressed --> return to RTC timer display with updated time/date settings
			else if (state == Date_Setting)
			{
				Update_Date_Time(); // Ensure all values are updated with what the user specified
				state = Display_Time; // Return to the main LCD screen with running time
			}
		}
					
							/// Create  a switch case statement for all the potential casses ///
		
		switch (state)
		{
			case (Display_Time):
					// Show the current Time which is default
				break;
			
			case (Display_Date):
				// Show the current date
				Get_Date_Time();
			
																// Display the Weekday //
				BSP_LCD_GLASS_Clear();
				sprintf(datestr, "%s  %02d",DATE[0], Weekday); // Store the WeekDay into date string
				BSP_LCD_GLASS_DisplayString((uint8_t*)datestr); //Display Weekday on the lcd screen
				HAL_Delay(1000);
				
																// Display the Month //
				BSP_LCD_GLASS_Clear();
				sprintf(datestr, "%s  %02d",DATE[2], Month); // Store the Month into date string
				BSP_LCD_GLASS_DisplayString((uint8_t*)datestr); // Diplay the Month on the lcd screen
				HAL_Delay(1000);
				
			
																// Display the Day //
				BSP_LCD_GLASS_Clear();
				sprintf(datestr, "%s  %02d",DATE[1], Day); // Store the day into date string
				BSP_LCD_GLASS_DisplayString((uint8_t*)datestr); // Display the day on the lcd screen
				HAL_Delay(1000);
				
																// Display the Year //
				BSP_LCD_GLASS_Clear();
				sprintf(datestr, "%s20%02d",DATE[3], Year); // Store the year in the date string (2020 format)
				BSP_LCD_GLASS_DisplayString((uint8_t*)datestr); // Display the year on the lcd screen
				HAL_Delay(1000);
				
				state = Display_Time; // Return back to state where time is being displayed on the LCD Screen
				break;
			
			case (Store_Time):
				temp_index = 0;
				Get_Date_Time();
				
								// Writting Hour, Minutes, and Seconds to the EEPROM // 
			
				EE_status=I2C_ByteWrite(&hi2c1,EEPROM_ADDRESS, memLocation+temp_index, Hour); //Store the Hours in the EEPROM @ base location
				if(EE_status != HAL_OK)
					I2C_Error(&hi2c1);
				
				EE_status=I2C_ByteWrite(&hi2c1,EEPROM_ADDRESS, memLocation+temp_index+1, Minutes); //Store the Minutes in the EEPROM @base location +1
				if(EE_status != HAL_OK)
					I2C_Error(&hi2c1);
				
				EE_status=I2C_ByteWrite(&hi2c1,EEPROM_ADDRESS, memLocation+temp_index+2, Seconds); //Store the Seconds in the EEPROM @base location +2
				if(EE_status != HAL_OK)
					I2C_Error(&hi2c1);
				
				
				BSP_LCD_GLASS_Clear();
				BSP_LCD_GLASS_DisplayString((uint8_t*)"Stored");
				HAL_Delay(100);  
				MemoryIndex+=1; //Increment the memory index to indicate how many times are stored
				state = Display_Time;
				break;
				
			
			case (Display_Last_Time):
				if (MemoryIndex>1) //If more than one time is stored in the EEPROM
				{
					temp_index = 3*MemoryIndex-6; //Temp index stores at the base adress of the second last entry in EEPROM
					
					readHourSecond_Last=I2C_ByteRead(&hi2c1,EEPROM_ADDRESS, memLocation+temp_index); 
					readMinuteSecond_Last=I2C_ByteRead(&hi2c1,EEPROM_ADDRESS, memLocation+temp_index+1); 
					readSecondsSecond_Last=I2C_ByteRead(&hi2c1,EEPROM_ADDRESS, memLocation+temp_index+2);
					
					readHourLast=I2C_ByteRead(&hi2c1,EEPROM_ADDRESS, memLocation+temp_index+3); 
					readMinuteLast=I2C_ByteRead(&hi2c1,EEPROM_ADDRESS, memLocation+temp_index+4); 
					readSecondsLast=I2C_ByteRead(&hi2c1,EEPROM_ADDRESS, memLocation+temp_index+5);
				}
				
				else if (MemoryIndex == 1)// If only one time is stored in the EEPROM
				{
					temp_index=0; 
					
					readHourLast=I2C_ByteRead(&hi2c1,EEPROM_ADDRESS, memLocation+temp_index); 
					readMinuteLast=I2C_ByteRead(&hi2c1,EEPROM_ADDRESS, memLocation+temp_index+1); 
					readSecondsLast=I2C_ByteRead(&hi2c1,EEPROM_ADDRESS, memLocation+temp_index+2);
				}
				else //If nothing has been written to the EEPROM, nothing will be displayed and keep timer running on LCD Screen
				{
					state = Display_Time;
					break;
				}
				
																//// Display the second last entry in EEPROM ////
				BSP_LCD_GLASS_Clear();
				sprintf(timestr,"%02d%02d%02d",readHourSecond_Last,readMinuteSecond_Last,readSecondsSecond_Last);
				BSP_LCD_GLASS_DisplayString((uint8_t*)timestr);
				
				BSP_LCD_GLASS_DisplayChar((uint8_t*) &timestr[1], POINT_OFF, DOUBLEPOINT_ON, LCD_DIGIT_POSITION_2);
				BSP_LCD_GLASS_DisplayChar((uint8_t*) &timestr[3], POINT_OFF, DOUBLEPOINT_ON, LCD_DIGIT_POSITION_4);
				HAL_Delay(1000);
				
																//// Display the last entry in EEPROM////
				BSP_LCD_GLASS_Clear();
				sprintf(timestr,"%02d%02d%02d",readHourLast,readMinuteLast,readSecondsLast);
				BSP_LCD_GLASS_DisplayString((uint8_t*)timestr);
				
				BSP_LCD_GLASS_DisplayChar((uint8_t*) &timestr[1], POINT_OFF, DOUBLEPOINT_ON, LCD_DIGIT_POSITION_2);
				BSP_LCD_GLASS_DisplayChar((uint8_t*) &timestr[3], POINT_OFF, DOUBLEPOINT_ON, LCD_DIGIT_POSITION_4);
				HAL_Delay(1000);
				
				state = Display_Time;
				break;
			
			
			case (Time_Setting):
				// Display the Time on LCD Screen for user to change given to their specification //
				sprintf(timestr,"%s  %d",TIME[time_increment],UserTime[time_increment]);
				BSP_LCD_GLASS_Clear();
				BSP_LCD_GLASS_DisplayString((uint8_t*)timestr);
				break;
			
			case (Date_Setting):
				// Display the Date on LCD Screen for user to change given to their specification //
				sprintf(datestr,"%s  %d",DATE[date_increment],UserDate[date_increment]);
				BSP_LCD_GLASS_Clear();
				BSP_LCD_GLASS_DisplayString((uint8_t*)datestr);
				
				break;
			
			
			default:
				break;
		}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000E14;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 12;
  sTime.Minutes = 30;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_SATURDAY;
  sDate.Month = RTC_MONTH_NOVEMBER;
  sDate.Date = 7;
  sDate.Year = 20;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0;
  sAlarm.AlarmTime.Minutes = 0;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_ALL;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}


/* USER CODE BEGIN 4 */

/// RTC_TimeDisplay used to display time on LCD Screen ///
void RTC_TimeDisplay(void)
{
	Get_Date_Time();
	sprintf(timestr,"%02d%02d%02d",Hour,Minutes,Seconds);
	BSP_LCD_GLASS_Clear();
	BSP_LCD_GLASS_DisplayString((uint8_t*)timestr);
	
	// Used to seperate the hours, minutes, seconds on LCD screen for easy legibiility
	BSP_LCD_GLASS_DisplayChar((uint8_t*) &timestr[1], POINT_OFF, DOUBLEPOINT_ON, LCD_DIGIT_POSITION_2);
	BSP_LCD_GLASS_DisplayChar((uint8_t*) &timestr[3], POINT_OFF, DOUBLEPOINT_ON, LCD_DIGIT_POSITION_4);

}

// RTC Interrupt Function //
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	BSP_LED_Toggle(LED4);
	if (state == Display_Time)
		RTC_TimeDisplay();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin)
	{
		case GPIO_PIN_0: // SELECT Button
			Selection_Pressed = 1;	
			break;
		
		case GPIO_PIN_1: // LEFT Button
			Left_Pressed = 1;
			break;
		
		case GPIO_PIN_2: // RIGHT Button
			Right_Pressed = 1;
			break;
		
		case GPIO_PIN_3: // UP Button
			BSP_LCD_GLASS_Clear();
			BSP_LCD_GLASS_DisplayString((uint8_t*)"UP");
			break;
		
		case GPIO_PIN_5: // DOWN Button
			BSP_LCD_GLASS_Clear();
			BSP_LCD_GLASS_DisplayString((uint8_t*)"DOWN");
			break;
		
		default:
			break;	
	}
}

////   Get the current date and time and store them into global variables  ////
void Get_Date_Time(void)
{
	HAL_RTC_GetDate(&hrtc,&RTC_DateStruct, RTC_FORMAT_BIN);
	HAL_RTC_GetTime(&hrtc,&RTC_TimeStruct, RTC_FORMAT_BIN);
	
	Day = RTC_DateStruct.Date;
	Weekday = RTC_DateStruct.WeekDay;
	Month = RTC_DateStruct.Month;
	Year = RTC_DateStruct.Year;
	
	Hour = RTC_TimeStruct.Hours;
	Minutes = RTC_TimeStruct.Minutes;
	Seconds = RTC_TimeStruct.Seconds;
}

//// Update the Date and Time of in the RTC to user specifications////
void Update_Date_Time(void)
{
	RTC_TimeStruct.Hours = UserTime[0];
	RTC_TimeStruct.Minutes = UserTime[1];
	RTC_TimeStruct.Seconds = UserTime[2];
	HAL_RTC_SetTime(&hrtc,&RTC_TimeStruct, RTC_FORMAT_BIN);
	
	RTC_DateStruct.WeekDay = UserDate[0];
	RTC_DateStruct.Date = UserDate[1];
	RTC_DateStruct.Month = UserDate[2];
	RTC_DateStruct.Year = UserDate[3];
	HAL_RTC_SetDate(&hrtc,&RTC_DateStruct, RTC_FORMAT_BIN);
}


// Retrieve current date and time for User ability to change the parameters //
void Current_Date_Time(void)
{
	UserTime[0] = Hour;
	UserTime[1] = Minutes;
	UserTime[2] = Seconds;
	
	UserDate[0] = Weekday;
	UserDate[1] = Day;
	UserDate[2] = Month;
	UserDate[3] = Year;
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
