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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
volatile int line_valid = 0;
#define LINEMAX 10 // Maximal allowed/expected line length
static char line_buffer[LINEMAX]; // Holding buffer with space for terminating NUL
static char rx_buffer[LINEMAX]; // Local holding buffer to build line
void substring(char [], char[], int, int);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
#define ADS1115_ADDRESS 0x48
int16_t reading;


char byte, rx;
char chardata[15];
uint8_t dimmer=0;
uint16_t command_value = 0;
bool reportStatus = false;
bool CW_MODE = false;
bool CR_MODE = false;
txDone = true;
autoFanSpeedMode = true;
uint32_t previousMillis = 0;
uint32_t previousMillis_INTEGRATION = 0;
uint32_t zeroTimeValue = 0;
uint32_t statusInterval = 500;           // interval at which to blink (milliseconds)
uint32_t integrationTime = 0;
uint32_t milliAmpHours = 0;
uint32_t milliWattHours = 0;
uint16_t setCurrent = 0;           // interval at which to blink (milliseconds)
float voltageCompensationConstant = 3.5614330954;
float currentCompensationConstant = 1.34733435793;
float temperatureCompensationConstant = 1; //VOLT: 1.98 (16 deg) - 0.065 (152 deg)
unsigned char ADSwrite[6];

uint16_t maxTemp = 100;
uint16_t maxAmpDiff = 0;
uint16_t maxWatt = 200;
uint16_t maxCurrent = 20;

uint16_t minVolt = 0;

int16_t ADS11115_VOLT = 0;
int16_t ADS11115_CURRENT = 0;
int16_t ADS11115_TEMPERATURE = 0;


struct statusValues {
	uint32_t   timestamp;
	uint16_t   temperature;
	uint16_t   setCurrent;
	uint16_t   setPower;
	uint16_t   measuredCurrent;
	uint16_t   measuredVoltage;
	uint32_t   measuredPower;
	float   amperehours;
	float   watthours;
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
USART2_IRQHandler();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)

{

 /* Set transmission flag: transfer complete */

txDone = true;

}

// This callback is called by the HAL_UART_IRQHandler when the given number of bytes are received
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(&huart2, &byte, 1);//Restart the interrupt reception mode
	static int rx_index = 0;
	//if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) // Received character?
	if (huart->Instance == USART2)
	{
		//char rx = HAL_UART_Receive_IT(&huart2, &byte, 1);
		//HAL_UART_Transmit(&huart2, &byte, 1, 10);
		rx = byte;
		if ((rx == '\r') || (rx == '\n')) // Is this an end-of-line condition, either will suffice?
		{
			if (rx_index != 0) // Line has some content?
			{
				memcpy((void *)line_buffer, rx_buffer, rx_index); // Copy to static line buffer from dynamic receive buffer
				line_buffer[rx_index] = 0; // Add terminating NUL
				line_valid = 1; // flag new line valid for processing
				rx_index = 0; // Reset content pointer
				memset(&rx_buffer, '\0', sizeof(rx_buffer));
			}
		}
		else
		{
			if (rx_index == LINEMAX) // If overflows pull back to start
				rx_index = 0;
			rx_buffer[rx_index++] = rx; // Copy to buffer and increment
		}
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//Connect ADDR pin to GND and I2C slave adress will be 0X48 .
	unsigned char ADSwrite[6];
	int16_t *ADS11115_Readings[4];
	int16_t voltage[3];
	struct statusValues my_statusValues;        /* Declare Book1 of type Book */
	my_statusValues.amperehours=0;
	my_statusValues.watthours=0;
	my_statusValues.setCurrent=0;
  /* USER CODE END 1 */

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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2); //Start Pwm signal on PB-6 Pin
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1); //Start Pwm signal on PB-6 Pin
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2); //Start Pwm signal on PB-6 Pin
	HAL_UART_Receive_IT(&huart2, &byte, 1);//Start the interrupt reception mode
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	//*ADS11115_Readings = ADS1115_read(&hi2c1,&huart2);








	    for(int i=0; i< 3; i++){

	    	ADSwrite[0] = 0x01;

	    				switch(i){
	    					case(0):
	    						ADSwrite[1] = 0xC1; //11000001
	    					break;
	    					case(1):
	    						ADSwrite[1] = 0xD1; //11010001
	    					break;
	    					case(2):
								ADSwrite[1] = 0xF1;//ADSwrite[1] = 0xE1;
	    					break;
	    					/*
	    					case(3):
	    						ADSwrite[1] = 0xF1;
	    					break;*/
	    				}

	    				ADSwrite[2] = 0x83; //10000011 LSB

	    				HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, ADSwrite, 3, 100);
	    				ADSwrite[0] = 0x00;
	    				HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1 , ADSwrite, 1 ,100);
	    				HAL_Delay(10);//20

	    				HAL_I2C_Master_Receive(&hi2c1, ADS1115_ADDRESS <<1, ADSwrite, 2, 100);
	    				reading = (ADSwrite[0] << 8 | ADSwrite[1] );
	    				if(reading < 0) {
	    					reading = 0;
	    				}

	    				voltage[i] = reading;
	    		}















	my_statusValues.measuredVoltage = voltage[0] * voltageCompensationConstant;
	my_statusValues.measuredCurrent = voltage[1] * currentCompensationConstant;
	my_statusValues.temperature = adc2Temperature(voltage[2]);
	my_statusValues.measuredPower = (my_statusValues.measuredVoltage*my_statusValues.measuredCurrent)/1000;
	my_statusValues.timestamp = (HAL_GetTick()-zeroTimeValue);

	integrationTime = HAL_GetTick()-previousMillis_INTEGRATION;
	my_statusValues.amperehours = my_statusValues.amperehours + integrationTime * my_statusValues.measuredCurrent/3600000.0;
	my_statusValues.watthours = my_statusValues.watthours + integrationTime * my_statusValues.measuredCurrent * my_statusValues.measuredVoltage /3600000000.0;
	previousMillis_INTEGRATION = HAL_GetTick();



	//Set current by setting voltage drop over shunt
	MCP4725_write(&hi2c1, my_statusValues.setCurrent);

if (line_valid==1){ // A new line has arrived
	BEEP(&htim2);
	line_valid = 0; // clear pending flag
	debugPrint(&huart2, "Sent command: ");
	debugPrintln(&huart2, line_buffer);
	int i = 0;
	char *p = strtok (line_buffer, " ");
	char *array[4];

	while (p != NULL){
		array[i++] = p;
		p = strtok (NULL, "/");}

	if((strncmp(array[0], "??" ,10) == 0) ||  (strncmp(array[0], "help" ,10) == 0) || (strncmp(array[0], "h" ,10) == 0)){
		printHELP(&huart2,my_statusValues);}

	else if(strncmp(array[0], "fs" ,10) == 0){
		if (strncmp(array[1], "A" ,10) == 0){
			autoFanSpeedMode=true;
			debugPrintln(&huart2, "Setting fan speed to Auto");}
		else{command_value = stringToInt(array[1]);
			setFanSpeed(&huart2, &htim1, command_value);
			autoFanSpeedMode=false;
		}
	}

	else if(strncmp(array[0], "mv" ,10) == 0){
		command_value = stringToInt(array[1]);
		minVolt = command_value; }

	else if(strncmp(array[0], "cc" ,10) == 0){
		if (stringToInt(array[1]) <= 20000){
			my_statusValues.setCurrent = stringToInt(array[1]);
			reportStatus = true;
		}

		else{
			debugPrintln(&huart2, "Requested current is too high...");}
		}

	else if(strncmp(array[0], "cp" ,10) == 0){
		my_statusValues.setPower = stringToInt(array[1]);
		if (my_statusValues.setPower <= 200){
			my_statusValues.setCurrent = 1000000.0*my_statusValues.setPower/my_statusValues.measuredVoltage;
			reportStatus = true;
			CW_MODE = true;
		}

		else{
			debugPrintln(&huart2, "Requested power is too high...");}
		}



	else if(strncmp(array[0], "status" ,10) == 0){
		printStatus(my_statusValues, &huart2);}

	else if((strncmp(array[0], "stop" ,10) == 0) || strncmp(array[0], "s" ,10) == 0){
		reportStatus = false;
		my_statusValues.setCurrent = 0;
		CW_MODE = false;
	}

	else if(strncmp(array[0], "reset" ,10) == 0){
		debugPrintln(&huart2, "Resetting mAh, mWh and time");
		my_statusValues.amperehours=0;
		my_statusValues.watthours=0;
		zeroTimeValue = HAL_GetTick();
	}

	else if(strncmp(array[0], "log" ,10) == 0){
		statusInterval=stringToInt(array[1]);}

	else{
		debugPrintln(&huart2, "Unknown command..., showing HELP");
		printHELP(&huart2);}
	memset(&line_buffer, '\0', sizeof(line_buffer));
}



	  HAL_GPIO_TogglePin(GPIOB, LED_Pin); //Toggle LED
	  HAL_GPIO_TogglePin(GPIOA, BILED_1_Pin); //Toggle LED
	  HAL_GPIO_TogglePin(GPIOB, BILED_2_Pin); //Toggle LED
	  HAL_GPIO_TogglePin(GPIOA,  DISCHARGE_LED_Pin); //Toggle LED

	  //HAL_GPIO_WritePin(GPIOA, STATLED_Pin,GPIO_PIN_SET);               //If pressed Led Switch On
	  //HAL_GPIO_WritePin(GPIOA, STATLED_Pin,GPIO_PIN_RESET);          //Else Led Switch Off







	  if(reportStatus){
		  if(HAL_GetTick() - previousMillis >= statusInterval){
			  printStatus(my_statusValues, &huart2);
				previousMillis = HAL_GetTick();
		  }
	  }

	  if (autoFanSpeedMode){
		  autoFanSpeed(&htim1, my_statusValues.temperature);
	  }

	  if(my_statusValues.measuredVoltage<minVolt){
			my_statusValues.setCurrent = 0;
			reportStatus = false;
			BEEP(&htim2);
		  	debugPrintln(&huart2, "Min voltage reached. Stopped discharge");
	  }


	  if(my_statusValues.measuredPower/1000>maxWatt){
			my_statusValues.setCurrent = 0;
			reportStatus = false;
			BEEP(&htim2);
			debugPrintln(&huart2, "OVERPOWER, Stopped discharge");
	  }

	  if(my_statusValues.measuredCurrent/1000>maxCurrent){
			my_statusValues.setCurrent = 0;
			reportStatus = false;
			BEEP(&htim2);
			debugPrintln(&huart2, "OVERCURRENT, Stopped discharge");
	  }


	  if(my_statusValues.temperature>maxTemp*10){
			my_statusValues.setCurrent = 0;
			reportStatus = false;
			BEEP(&htim2);
			HAL_Delay(50);

			debugPrintln(&huart2, "OVERTEMP, Stopped discharge");
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks 
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00506682;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 610;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 24;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  //__HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DISCHARGE_LED_Pin|OVERTEMP_Pin|BILED_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BILED_2_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : FANIN_Pin */
  GPIO_InitStruct.Pin = FANIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FANIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DISCHARGE_LED_Pin OVERTEMP_Pin BILED_1_Pin */
  GPIO_InitStruct.Pin = DISCHARGE_LED_Pin|OVERTEMP_Pin|BILED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BILED_2_Pin LED_Pin */
  GPIO_InitStruct.Pin = BILED_2_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//HAL_GPIO_WritePin(GPIOA, BILED_1_Pin,GPIO_PIN_SET);               //If pressed Led Switch On
//HAL_GPIO_WritePin(GPIOA, BILED_1_Pin,GPIO_PIN_RESET);          //Else Led Switch Off

//HAL_GPIO_WritePin(GPIOA, STATLED_Pin,GPIO_PIN_SET);               //If pressed Led Switch On
//HAL_GPIO_WritePin(GPIOA, STATLED_Pin,GPIO_PIN_RESET);          //Else Led Switch Off
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
