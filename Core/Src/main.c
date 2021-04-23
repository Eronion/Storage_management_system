/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l475e_iot01.h"
#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_hsensor.h"
#include "LiquidCrystal.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

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
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

I2C_HandleTypeDef hi2c2;

QSPI_HandleTypeDef hqspi;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

osThreadId Temp_monitorHandle;
osThreadId Hum_monitorHandle;
osThreadId LCD_tempHandle;
osThreadId LCD_humHandle;
osThreadId Servo_managerHandle;
osThreadId T_ProblemDetectHandle;
osThreadId H_ProblemDetectHandle;
osThreadId Buzzer_managerHandle;
osThreadId LCD_TempProblemHandle;
osThreadId LCD_HumProblemHandle;
osThreadId LED_managerHandle;
osThreadId Button_managerHandle;
osTimerId myTimerLed1Handle;
osTimerId myTimerLed2Handle;
osMutexId myMutex_tempHandle;
osMutexId myMutex_humHandle;
osMutexId myMutex_LCDHandle;
osMutexId myMutexEmergencyHandle;
/* USER CODE BEGIN PV */
float temp_value = 0; // Variable to store temperature value
float hum_value = 0; // Variable to store humidity value
int emergency_temp = 0; //Codifica l'emergenza in 0: nessuna emergenza, 1:emergenza_temp

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM2_Init(void);
void StartTemp_monitor(void const * argument);
void StartHum_monitor(void const * argument);
void StartLCD_temp(void const * argument);
void StartLCD_hum(void const * argument);
void StartServo_manager(void const * argument);
void StartT_ProblemDetect(void const * argument);
void StartH_ProblemDetect(void const * argument);
void StartBuzzer_manager(void const * argument);
void StartLCD_TempProblem(void const * argument);
void StartLCD_HumProblem(void const * argument);
void StartLED_manager(void const * argument);
void StartButton_manager(void const * argument);
void CallbackLed1(void const * argument);
void CallbackLed2(void const * argument);

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

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DFSDM1_Init();
	MX_I2C2_Init();
	MX_QUADSPI_Init();
	MX_SPI3_Init();
	MX_USART1_UART_Init();
	MX_USART3_UART_Init();
	MX_USB_OTG_FS_PCD_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	BSP_TSENSOR_Init();
	BSP_HSENSOR_Init();
	LiquidCrystal(GPIOC, LCD_RS_Pin, LCD_E_Pin, LCD_E_Pin, LCD_D4_Pin, LCD_D5_Pin, LCD_D6_Pin, LCD_D7_Pin); //Initialize the LCD
	print("Caricamento..");

	/* USER CODE END 2 */

	/* Create the mutex(es) */
	/* definition and creation of myMutex_temp */
	osMutexDef(myMutex_temp);
	myMutex_tempHandle = osMutexCreate(osMutex(myMutex_temp));

	/* definition and creation of myMutex_hum */
	osMutexDef(myMutex_hum);
	myMutex_humHandle = osMutexCreate(osMutex(myMutex_hum));

	/* definition and creation of myMutex_LCD */
	osMutexDef(myMutex_LCD);
	myMutex_LCDHandle = osMutexCreate(osMutex(myMutex_LCD));

	/* definition and creation of myMutexEmergency */
	osMutexDef(myMutexEmergency);
	myMutexEmergencyHandle = osMutexCreate(osMutex(myMutexEmergency));

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* Create the timer(s) */
	/* definition and creation of myTimerLed1 */
	osTimerDef(myTimerLed1, CallbackLed1);
	myTimerLed1Handle = osTimerCreate(osTimer(myTimerLed1), osTimerPeriodic, NULL);

	/* definition and creation of myTimerLed2 */
	osTimerDef(myTimerLed2, CallbackLed2);
	myTimerLed2Handle = osTimerCreate(osTimer(myTimerLed2), osTimerPeriodic, NULL);

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of Temp_monitor */
	osThreadDef(Temp_monitor, StartTemp_monitor, osPriorityAboveNormal, 0, 128);
	Temp_monitorHandle = osThreadCreate(osThread(Temp_monitor), NULL);

	/* definition and creation of Hum_monitor */
	osThreadDef(Hum_monitor, StartHum_monitor, osPriorityAboveNormal, 0, 128);
	Hum_monitorHandle = osThreadCreate(osThread(Hum_monitor), NULL);

	/* definition and creation of LCD_temp */
	osThreadDef(LCD_temp, StartLCD_temp, osPriorityBelowNormal, 0, 128);
	LCD_tempHandle = osThreadCreate(osThread(LCD_temp), NULL);

	/* definition and creation of LCD_hum */
	osThreadDef(LCD_hum, StartLCD_hum, osPriorityBelowNormal, 0, 128);
	LCD_humHandle = osThreadCreate(osThread(LCD_hum), NULL);

	/* definition and creation of Servo_manager */
	osThreadDef(Servo_manager, StartServo_manager, osPriorityHigh, 0, 128);
	Servo_managerHandle = osThreadCreate(osThread(Servo_manager), NULL);

	/* definition and creation of T_ProblemDetect */
	osThreadDef(T_ProblemDetect, StartT_ProblemDetect, osPriorityIdle, 0, 128);
	T_ProblemDetectHandle = osThreadCreate(osThread(T_ProblemDetect), NULL);

	/* definition and creation of H_ProblemDetect */
	osThreadDef(H_ProblemDetect, StartH_ProblemDetect, osPriorityIdle, 0, 128);
	H_ProblemDetectHandle = osThreadCreate(osThread(H_ProblemDetect), NULL);

	/* definition and creation of Buzzer_manager */
	osThreadDef(Buzzer_manager, StartBuzzer_manager, osPriorityNormal, 0, 128);
	Buzzer_managerHandle = osThreadCreate(osThread(Buzzer_manager), NULL);

	/* definition and creation of LCD_TempProblem */
	osThreadDef(LCD_TempProblem, StartLCD_TempProblem, osPriorityNormal, 0, 128);
	LCD_TempProblemHandle = osThreadCreate(osThread(LCD_TempProblem), NULL);

	/* definition and creation of LCD_HumProblem */
	osThreadDef(LCD_HumProblem, StartLCD_HumProblem, osPriorityNormal, 0, 128);
	LCD_HumProblemHandle = osThreadCreate(osThread(LCD_HumProblem), NULL);

	/* definition and creation of LED_manager */
	osThreadDef(LED_manager, StartLED_manager, osPriorityNormal, 0, 128);
	LED_managerHandle = osThreadCreate(osThread(LED_manager), NULL);

	/* definition and creation of Button_manager */
	osThreadDef(Button_manager, StartButton_manager, osPriorityHigh, 0, 128);
	Button_managerHandle = osThreadCreate(osThread(Button_manager), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
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
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 40;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART3
			|RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_DFSDM1
			|RCC_PERIPHCLK_USB;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
	PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
	PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
	PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
	PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
	PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
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
	 if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	 {
		 Error_Handler();
	 }
}

/**
 * @brief DFSDM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DFSDM1_Init(void)
{

	/* USER CODE BEGIN DFSDM1_Init 0 */

	/* USER CODE END DFSDM1_Init 0 */

	/* USER CODE BEGIN DFSDM1_Init 1 */

	/* USER CODE END DFSDM1_Init 1 */
	hdfsdm1_channel1.Instance = DFSDM1_Channel1;
	hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
	hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
	hdfsdm1_channel1.Init.OutputClock.Divider = 2;
	hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
	hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
	hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
	hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
	hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
	hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
	hdfsdm1_channel1.Init.Awd.Oversampling = 1;
	hdfsdm1_channel1.Init.Offset = 0;
	hdfsdm1_channel1.Init.RightBitShift = 0x00;
	if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN DFSDM1_Init 2 */

	/* USER CODE END DFSDM1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x00000E14;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief QUADSPI Initialization Function
 * @param None
 * @retval None
 */
static void MX_QUADSPI_Init(void)
{

	/* USER CODE BEGIN QUADSPI_Init 0 */

	/* USER CODE END QUADSPI_Init 0 */

	/* USER CODE BEGIN QUADSPI_Init 1 */

	/* USER CODE END QUADSPI_Init 1 */
	/* QUADSPI parameter configuration*/
	hqspi.Instance = QUADSPI;
	hqspi.Init.ClockPrescaler = 2;
	hqspi.Init.FifoThreshold = 4;
	hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
	hqspi.Init.FlashSize = 23;
	hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
	hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
	if (HAL_QSPI_Init(&hqspi) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN QUADSPI_Init 2 */

	/* USER CODE END QUADSPI_Init 2 */

}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void)
{

	/* USER CODE BEGIN SPI3_Init 0 */

	/* USER CODE END SPI3_Init 0 */

	/* USER CODE BEGIN SPI3_Init 1 */

	/* USER CODE END SPI3_Init 1 */
	/* SPI3 parameter configuration*/
	hspi3.Instance = SPI3;
	hspi3.Init.Mode = SPI_MODE_MASTER;
	hspi3.Init.Direction = SPI_DIRECTION_2LINES;
	hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi3.Init.NSS = SPI_NSS_SOFT;
	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi3.Init.CRCPolynomial = 7;
	hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI3_Init 2 */

	/* USER CODE END SPI3_Init 2 */

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

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 4294967295;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief USB_OTG_FS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_FS_PCD_Init(void)
{

	/* USER CODE BEGIN USB_OTG_FS_Init 0 */

	/* USER CODE END USB_OTG_FS_Init 0 */

	/* USER CODE BEGIN USB_OTG_FS_Init 1 */

	/* USER CODE END USB_OTG_FS_Init 1 */
	hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
	hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
	hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
	hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
	hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
	hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
	if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USB_OTG_FS_Init 2 */

	/* USER CODE END USB_OTG_FS_Init 2 */

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
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, LCD_D7_Pin|LCD_D6_Pin|LCD_D5_Pin|LCD_D4_Pin
			|LCD_E_Pin|LCD_RS_Pin|VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|Buzzer_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
			|SPSGRF_915_SDN_Pin|ARD_D5_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

	/*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
	GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
	GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin|ISM43362_DRDY_EXTI1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : BUTTON_EXTI13_Pin */
	GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LCD_D7_Pin LCD_D6_Pin LCD_D5_Pin LCD_D4_Pin
                           LCD_E_Pin LCD_RS_Pin VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
	GPIO_InitStruct.Pin = LCD_D7_Pin|LCD_D6_Pin|LCD_D5_Pin|LCD_D4_Pin
			|LCD_E_Pin|LCD_RS_Pin|VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : ARD_D1_Pin ARD_D0_Pin */
	GPIO_InitStruct.Pin = ARD_D1_Pin|ARD_D0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : ARD_D10_Pin Buzzer_Pin SPBTLE_RF_RST_Pin ARD_D9_Pin */
	GPIO_InitStruct.Pin = ARD_D10_Pin|Buzzer_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin */
	GPIO_InitStruct.Pin = ARD_D13_Pin|ARD_D12_Pin|ARD_D11_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : ARD_D3_Pin */
	GPIO_InitStruct.Pin = ARD_D3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : ARD_D6_Pin */
	GPIO_InitStruct.Pin = ARD_D6_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin LED2_Pin
                           SPSGRF_915_SDN_Pin ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
	GPIO_InitStruct.Pin = ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
			|SPSGRF_915_SDN_Pin|ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin ARD_D2_Pin HTS221_DRDY_EXTI15_Pin
                           PMOD_IRQ_EXTI12_Pin */
	GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|ARD_D2_Pin|HTS221_DRDY_EXTI15_Pin
			|PMOD_IRQ_EXTI12_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
	GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
	GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
	GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
	GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin|PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : ARD_D15_Pin ARD_D14_Pin */
	GPIO_InitStruct.Pin = ARD_D15_Pin|ARD_D14_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTemp_monitor */
/**
 * @brief  Function implementing the Temp_monitor thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTemp_monitor */
void StartTemp_monitor(void const * argument)
{
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for(;;)
	{
		osMutexWait(myMutex_tempHandle, 1000);
		temp_value = BSP_TSENSOR_ReadTemp();
		osMutexRelease(myMutex_tempHandle);
		osDelay(500);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartHum_monitor */
/**
 * @brief Function implementing the Hum_monitor thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartHum_monitor */
void StartHum_monitor(void const * argument)
{
	/* USER CODE BEGIN StartHum_monitor */
	/* Infinite loop */
	for(;;)
	{
		osMutexWait(myMutex_humHandle, 1000);
		hum_value = BSP_HSENSOR_ReadHumidity();
		osMutexRelease(myMutex_humHandle);
		osDelay(500);
	}
	/* USER CODE END StartHum_monitor */
}

/* USER CODE BEGIN Header_StartLCD_temp */
/**
 * @brief Function implementing the LCD_temp thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLCD_temp */
void StartLCD_temp(void const * argument)
{
	/* USER CODE BEGIN StartLCD_temp */
	char str_tmpT[100] = ""; // Formatted message to display the temperature value
	double temp_local = 0; //Variabile locale su cui copiamo il valore di temperatura
	/* Infinite loop */
	for(;;)
	{
		osMutexWait(myMutex_tempHandle, 1000);
		temp_local = temp_value;
		osMutexRelease(myMutex_tempHandle);

		osMutexWait(myMutex_LCDHandle, 10000);
		clear();
		int tmpIntT1 = temp_local;
		float tmpFracT = temp_local - tmpIntT1;
		int tmpIntT2 = trunc(tmpFracT * 100);
		snprintf(str_tmpT,100,"%d.%02d Celsius", tmpIntT1, tmpIntT2);
		setCursor(0, 0);
		print("TEMPERATURE"); //Print this
		setCursor(0, 1); //At secound row first column
		print(str_tmpT); //Print this
		osDelay(2500);
		osMutexRelease(myMutex_LCDHandle);
		osDelay(1000);
	}
	/* USER CODE END StartLCD_temp */
}

/* USER CODE BEGIN Header_StartLCD_hum */
/**
 * @brief Function implementing the LCD_hum thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLCD_hum */
void StartLCD_hum(void const * argument)
{
	/* USER CODE BEGIN StartLCD_hum */
	char str_tmpH[100] = ""; // Formatted message to display the humidity value
	double hum_local = 0; //Variabile locale su cui copiamo il valore di umidità
	/* Infinite loop */
	for(;;)
	{
		osMutexWait(myMutex_humHandle, 1000);
		hum_local = hum_value;
		osMutexRelease(myMutex_humHandle);

		osMutexWait(myMutex_LCDHandle, 10000);
		clear();
		int tmpIntH1 = hum_local;
		float tmpFracH = hum_local - tmpIntH1;
		int tmpIntH2 = trunc(tmpFracH * 100);
		snprintf(str_tmpH,100,"%d.%02d RH", tmpIntH1, tmpIntH2);
		setCursor(0, 0);
		print("UMIDITY"); //Print this
		setCursor(0, 1); //At second row first column
		print(str_tmpH); //Print this
		osDelay(2500);
		osMutexRelease(myMutex_LCDHandle);
		osDelay(1000);
	}
	/* USER CODE END StartLCD_hum */
}

/* USER CODE BEGIN Header_StartServo_manager */
/**
 * @brief Function implementing the Servo_manager thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartServo_manager */
void StartServo_manager(void const * argument)
{
	/* USER CODE BEGIN StartServo_manager */
	TIM2->PSC = 71;
	TIM2->ARR = 20000;
	TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;
	TIM2->BDTR |= TIM_BDTR_MOE;
	TIM2->CCER |= TIM_CCER_CC1E;
	TIM2->CR1 |= TIM_CR1_ARPE;
	TIM2->EGR |= TIM_EGR_UG;
	TIM2->CR1 |= TIM_CR1_CEN;

	bool servo_actuated = false;
	TIM2->CCR4 = 5.556*0 + 1000;
	HAL_Delay(200);

	int emergency_local = 0;
	/* Infinite loop */
	for(;;)
	{
		//Qui devo capire se siamo in emergenza dal flag o sem, e quindi muovere il
		// servo di conseguenza
		osMutexWait(myMutexEmergencyHandle, 10000);
		emergency_local = emergency_temp;
		osMutexRelease(myMutexEmergencyHandle);

		if((emergency_local == 1 /*Siamo in emergenza*/) && (servo_actuated == false)){
			servo_actuated = true;
			TIM2->CCR4 = 5.556*300 + 1000;
			HAL_Delay(500);
		} else if ((emergency_local == 0 /*Non siamo in emergenza*/) && (servo_actuated)){
			servo_actuated = false;
			TIM2->CCR4 = 5.556*0 + 1000;
			HAL_Delay(500);
		}
		osDelay(2000);
	}
	/* USER CODE END StartServo_manager */
}

/* USER CODE BEGIN Header_StartT_ProblemDetect */
/**
 * @brief Function implementing the T_ProblemDetect thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartT_ProblemDetect */
void StartT_ProblemDetect(void const * argument)
{
	/* USER CODE BEGIN StartT_ProblemDetect */
	/* Infinite loop */
	for(;;)
	{
		//Qui leggiamo temp e in caso di problemi lo segnaliamo
		osDelay(10000);
		osMutexWait(myMutexEmergencyHandle, 10000);
		emergency_temp = 1;
		osMutexRelease(myMutexEmergencyHandle);
		osDelay(10000);
		osMutexWait(myMutexEmergencyHandle, 10000);
		emergency_temp = 0;
		osMutexRelease(myMutexEmergencyHandle);
	}
	/* USER CODE END StartT_ProblemDetect */
}

/* USER CODE BEGIN Header_StartH_ProblemDetect */
/**
 * @brief Function implementing the H_ProblemDetect thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartH_ProblemDetect */
void StartH_ProblemDetect(void const * argument)
{
	/* USER CODE BEGIN StartH_ProblemDetect */
	/* Infinite loop */
	for(;;)
	{
		//Qui leggiamo hum e in caso di problemi lo segnaliamo
		osDelay(1000);
	}
	/* USER CODE END StartH_ProblemDetect */
}

/* USER CODE BEGIN Header_StartBuzzer_manager */
/**
 * @brief Function implementing the Buzzer_manager thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBuzzer_manager */
void StartBuzzer_manager(void const * argument)
{
	/* USER CODE BEGIN StartBuzzer_manager */
	int emergency_local = 0;
	/* Infinite loop */
	for(;;)
	{
		osMutexWait(myMutexEmergencyHandle, 10000);
		emergency_local = emergency_temp;
		osMutexRelease(myMutexEmergencyHandle);
		//emergency_local = leggicodiceemergenza();

		if(emergency_local == 1){
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
			osDelay(1000);
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
			osMutexRelease(myMutexEmergencyHandle);
		}

		osDelay(700);

	}
	/* USER CODE END StartBuzzer_manager */
}

/* USER CODE BEGIN Header_StartLCD_TempProblem */
/**
 * @brief Function implementing the LCD_TempProblem thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLCD_TempProblem */
void StartLCD_TempProblem(void const * argument)
{
	/* USER CODE BEGIN StartBuzzer_manager */
	int emergency_local = 0;
	/* Infinite loop */
	for(;;)
	{
		osMutexWait(myMutexEmergencyHandle, 10000);
		emergency_local = emergency_temp;
		osMutexRelease(myMutexEmergencyHandle);
		if(emergency_local == 1){
			osMutexWait(myMutex_LCDHandle, 10000);
			clear();
			setCursor(0, 0);
			print("EMERGENCY");
			osDelay(2000);
			clear();
			setCursor(0, 0);
			print("TEMP VALUE"); //Print this
			setCursor(0, 1); //At secound row first column
			print("OUT OF RANGE"); //Print this
			osDelay(2000);
			osMutexRelease(myMutex_LCDHandle);
		}
		osDelay(500);
	}
	/* USER CODE END StartLCD_TempProblem */
}

/* USER CODE BEGIN Header_StartLCD_HumProblem */
/**
 * @brief Function implementing the LCD_HumProblem thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLCD_HumProblem */
void StartLCD_HumProblem(void const * argument)
{
	/* USER CODE BEGIN StartLCD_HumProblem */

	/* Infinite loop */
	for(;;)
	{
		if(0/*Se siamo in emergenza per temp */){
			osMutexWait(myMutex_LCDHandle, 10000);
			clear();
			setCursor(0, 0);
			print("EMERGENCY");
			osDelay(2000);
			clear();
			setCursor(0, 0);
			print("HUMIDITY VALUE"); //Print this
			setCursor(0, 1); //At secound row first column
			print("OUT OF RANGE"); //Print this
			osDelay(2000);
			osMutexRelease(myMutex_LCDHandle);
		}
		osDelay(12000);
	}
	/* USER CODE END StartLCD_HumProblem */
}

/* USER CODE BEGIN Header_StartLED_manager */
/**
 * @brief Function implementing the LED_manager thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLED_manager */
void StartLED_manager(void const * argument)
{
	/* USER CODE BEGIN StartLED_manager */
	bool timers_started = false;
	//Questa flag serve a vedere se i timer sono attivi così
	// da agggiungere una consizione all'else if in modo da non entrare a stoppare i timer se
	// non sono stati effettivamente avviati ed allo stessso tempo di avviarli in continuazione
	int emergency_local = 0;
	/* Infinite loop */
	for(;;)
	{
		//Leggo la flag di emergenza
		osMutexWait(myMutexEmergencyHandle, 10000);
		emergency_local = emergency_temp;
		osMutexRelease(myMutexEmergencyHandle);



		if((timers_started == false)&&(emergency_local == 1 /*Inseriremo qui il check della variabile o sem che segnala l'emergenza*/)){
			timers_started = true;
			osTimerStart(myTimerLed1Handle, 500);
			osTimerStart(myTimerLed2Handle, 300);
		} else if ((timers_started) && (emergency_local == 0 /*Qui inseriamo check sul sem o variabile emergenza*/)){
			timers_started = false;
			osTimerStop(myTimerLed1Handle);
			osTimerStop(myTimerLed2Handle);
		}
		osDelay(500);

	}
	/* USER CODE END StartLED_manager */
}

/* USER CODE BEGIN Header_StartButton_manager */
/**
 * @brief Function implementing the Button_manager thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartButton_manager */
void StartButton_manager(void const * argument)
{
	/* USER CODE BEGIN StartButton_manager */
	/* Infinite loop */
	for(;;)
	{
		if(HAL_GPIO_ReadPin(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN) == GPIO_PIN_RESET){
			//Here when the button is pressed
			//Qui dobbiamo semplicemente riportare il flag o il semaforo che segnala l'emergenza
			//in modalità non-emergenza.
			osMutexWait(myMutexEmergencyHandle, 10000);
			emergency_temp = 0;
			osMutexRelease(myMutexEmergencyHandle);
		}

		osDelay(100);
	}
	/* USER CODE END StartButton_manager */
}

/* CallbackLed1 function */
void CallbackLed1(void const * argument)
{
	/* USER CODE BEGIN CallbackLed1 */
	HAL_GPIO_TogglePin (LED2_GPIO_PORT, LED2_PIN);

	/* USER CODE END CallbackLed1 */
}

/* CallbackLed2 function */
void CallbackLed2(void const * argument)
{
	/* USER CODE BEGIN CallbackLed2 */
	HAL_GPIO_TogglePin (LED3_WIFI__LED4_BLE_GPIO_Port, LED3_WIFI__LED4_BLE_Pin);

	/* USER CODE END CallbackLed2 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM17 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM17) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l475e_iot01.h"
#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_hsensor.h"
#include "LiquidCrystal.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

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
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

I2C_HandleTypeDef hi2c2;

QSPI_HandleTypeDef hqspi;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

osThreadId Temp_monitorHandle;
osThreadId Hum_monitorHandle;
osThreadId LCD_tempHandle;
osThreadId LCD_humHandle;
osThreadId Servo_managerHandle;
osThreadId T_ProblemDetectHandle;
osThreadId H_ProblemDetectHandle;
osThreadId Buzzer_managerHandle;
osThreadId LCD_TempProblemHandle;
osThreadId LCD_HumProblemHandle;
osThreadId LED_managerHandle;
osThreadId Button_managerHandle;
osTimerId myTimerLed1Handle;
osTimerId myTimerLed2Handle;
osMutexId myMutex_tempHandle;
osMutexId myMutex_humHandle;
osMutexId myMutex_LCDHandle;
osMutexId myMutexEmergencyHandle;
/* USER CODE BEGIN PV */
float temp_value = 0; // Variable to store temperature value
float hum_value = 0; // Variable to store humidity value
int emergency_temp = 0; //Codifica l'emergenza in 0: nessuna emergenza, 1:emergenza_temp

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM2_Init(void);
void StartTemp_monitor(void const * argument);
void StartHum_monitor(void const * argument);
void StartLCD_temp(void const * argument);
void StartLCD_hum(void const * argument);
void StartServo_manager(void const * argument);
void StartT_ProblemDetect(void const * argument);
void StartH_ProblemDetect(void const * argument);
void StartBuzzer_manager(void const * argument);
void StartLCD_TempProblem(void const * argument);
void StartLCD_HumProblem(void const * argument);
void StartLED_manager(void const * argument);
void StartButton_manager(void const * argument);
void CallbackLed1(void const * argument);
void CallbackLed2(void const * argument);

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

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DFSDM1_Init();
	MX_I2C2_Init();
	MX_QUADSPI_Init();
	MX_SPI3_Init();
	MX_USART1_UART_Init();
	MX_USART3_UART_Init();
	MX_USB_OTG_FS_PCD_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	BSP_TSENSOR_Init();
	BSP_HSENSOR_Init();
	LiquidCrystal(GPIOC, LCD_RS_Pin, LCD_E_Pin, LCD_E_Pin, LCD_D4_Pin, LCD_D5_Pin, LCD_D6_Pin, LCD_D7_Pin); //Initialize the LCD
	print("Caricamento..");

	/* USER CODE END 2 */

	/* Create the mutex(es) */
	/* definition and creation of myMutex_temp */
	osMutexDef(myMutex_temp);
	myMutex_tempHandle = osMutexCreate(osMutex(myMutex_temp));

	/* definition and creation of myMutex_hum */
	osMutexDef(myMutex_hum);
	myMutex_humHandle = osMutexCreate(osMutex(myMutex_hum));

	/* definition and creation of myMutex_LCD */
	osMutexDef(myMutex_LCD);
	myMutex_LCDHandle = osMutexCreate(osMutex(myMutex_LCD));

	/* definition and creation of myMutexEmergency */
	osMutexDef(myMutexEmergency);
	myMutexEmergencyHandle = osMutexCreate(osMutex(myMutexEmergency));

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* Create the timer(s) */
	/* definition and creation of myTimerLed1 */
	osTimerDef(myTimerLed1, CallbackLed1);
	myTimerLed1Handle = osTimerCreate(osTimer(myTimerLed1), osTimerPeriodic, NULL);

	/* definition and creation of myTimerLed2 */
	osTimerDef(myTimerLed2, CallbackLed2);
	myTimerLed2Handle = osTimerCreate(osTimer(myTimerLed2), osTimerPeriodic, NULL);

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of Temp_monitor */
	osThreadDef(Temp_monitor, StartTemp_monitor, osPriorityAboveNormal, 0, 128);
	Temp_monitorHandle = osThreadCreate(osThread(Temp_monitor), NULL);

	/* definition and creation of Hum_monitor */
	osThreadDef(Hum_monitor, StartHum_monitor, osPriorityAboveNormal, 0, 128);
	Hum_monitorHandle = osThreadCreate(osThread(Hum_monitor), NULL);

	/* definition and creation of LCD_temp */
	osThreadDef(LCD_temp, StartLCD_temp, osPriorityBelowNormal, 0, 128);
	LCD_tempHandle = osThreadCreate(osThread(LCD_temp), NULL);

	/* definition and creation of LCD_hum */
	osThreadDef(LCD_hum, StartLCD_hum, osPriorityBelowNormal, 0, 128);
	LCD_humHandle = osThreadCreate(osThread(LCD_hum), NULL);

	/* definition and creation of Servo_manager */
	osThreadDef(Servo_manager, StartServo_manager, osPriorityHigh, 0, 128);
	Servo_managerHandle = osThreadCreate(osThread(Servo_manager), NULL);

	/* definition and creation of T_ProblemDetect */
	osThreadDef(T_ProblemDetect, StartT_ProblemDetect, osPriorityIdle, 0, 128);
	T_ProblemDetectHandle = osThreadCreate(osThread(T_ProblemDetect), NULL);

	/* definition and creation of H_ProblemDetect */
	osThreadDef(H_ProblemDetect, StartH_ProblemDetect, osPriorityIdle, 0, 128);
	H_ProblemDetectHandle = osThreadCreate(osThread(H_ProblemDetect), NULL);

	/* definition and creation of Buzzer_manager */
	osThreadDef(Buzzer_manager, StartBuzzer_manager, osPriorityNormal, 0, 128);
	Buzzer_managerHandle = osThreadCreate(osThread(Buzzer_manager), NULL);

	/* definition and creation of LCD_TempProblem */
	osThreadDef(LCD_TempProblem, StartLCD_TempProblem, osPriorityNormal, 0, 128);
	LCD_TempProblemHandle = osThreadCreate(osThread(LCD_TempProblem), NULL);

	/* definition and creation of LCD_HumProblem */
	osThreadDef(LCD_HumProblem, StartLCD_HumProblem, osPriorityNormal, 0, 128);
	LCD_HumProblemHandle = osThreadCreate(osThread(LCD_HumProblem), NULL);

	/* definition and creation of LED_manager */
	osThreadDef(LED_manager, StartLED_manager, osPriorityNormal, 0, 128);
	LED_managerHandle = osThreadCreate(osThread(LED_manager), NULL);

	/* definition and creation of Button_manager */
	osThreadDef(Button_manager, StartButton_manager, osPriorityHigh, 0, 128);
	Button_managerHandle = osThreadCreate(osThread(Button_manager), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
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
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 40;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART3
			|RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_DFSDM1
			|RCC_PERIPHCLK_USB;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
	PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
	PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
	PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
	PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
	PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
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
	 if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	 {
		 Error_Handler();
	 }
}

/**
 * @brief DFSDM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DFSDM1_Init(void)
{

	/* USER CODE BEGIN DFSDM1_Init 0 */

	/* USER CODE END DFSDM1_Init 0 */

	/* USER CODE BEGIN DFSDM1_Init 1 */

	/* USER CODE END DFSDM1_Init 1 */
	hdfsdm1_channel1.Instance = DFSDM1_Channel1;
	hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
	hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
	hdfsdm1_channel1.Init.OutputClock.Divider = 2;
	hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
	hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
	hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
	hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
	hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
	hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
	hdfsdm1_channel1.Init.Awd.Oversampling = 1;
	hdfsdm1_channel1.Init.Offset = 0;
	hdfsdm1_channel1.Init.RightBitShift = 0x00;
	if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN DFSDM1_Init 2 */

	/* USER CODE END DFSDM1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x00000E14;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief QUADSPI Initialization Function
 * @param None
 * @retval None
 */
static void MX_QUADSPI_Init(void)
{

	/* USER CODE BEGIN QUADSPI_Init 0 */

	/* USER CODE END QUADSPI_Init 0 */

	/* USER CODE BEGIN QUADSPI_Init 1 */

	/* USER CODE END QUADSPI_Init 1 */
	/* QUADSPI parameter configuration*/
	hqspi.Instance = QUADSPI;
	hqspi.Init.ClockPrescaler = 2;
	hqspi.Init.FifoThreshold = 4;
	hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
	hqspi.Init.FlashSize = 23;
	hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
	hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
	if (HAL_QSPI_Init(&hqspi) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN QUADSPI_Init 2 */

	/* USER CODE END QUADSPI_Init 2 */

}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void)
{

	/* USER CODE BEGIN SPI3_Init 0 */

	/* USER CODE END SPI3_Init 0 */

	/* USER CODE BEGIN SPI3_Init 1 */

	/* USER CODE END SPI3_Init 1 */
	/* SPI3 parameter configuration*/
	hspi3.Instance = SPI3;
	hspi3.Init.Mode = SPI_MODE_MASTER;
	hspi3.Init.Direction = SPI_DIRECTION_2LINES;
	hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi3.Init.NSS = SPI_NSS_SOFT;
	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi3.Init.CRCPolynomial = 7;
	hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI3_Init 2 */

	/* USER CODE END SPI3_Init 2 */

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

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 4294967295;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief USB_OTG_FS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_FS_PCD_Init(void)
{

	/* USER CODE BEGIN USB_OTG_FS_Init 0 */

	/* USER CODE END USB_OTG_FS_Init 0 */

	/* USER CODE BEGIN USB_OTG_FS_Init 1 */

	/* USER CODE END USB_OTG_FS_Init 1 */
	hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
	hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
	hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
	hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
	hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
	hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
	if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USB_OTG_FS_Init 2 */

	/* USER CODE END USB_OTG_FS_Init 2 */

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
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, LCD_D7_Pin|LCD_D6_Pin|LCD_D5_Pin|LCD_D4_Pin
			|LCD_E_Pin|LCD_RS_Pin|VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|Buzzer_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
			|SPSGRF_915_SDN_Pin|ARD_D5_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

	/*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
	GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
	GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin|ISM43362_DRDY_EXTI1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : BUTTON_EXTI13_Pin */
	GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LCD_D7_Pin LCD_D6_Pin LCD_D5_Pin LCD_D4_Pin
                           LCD_E_Pin LCD_RS_Pin VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
	GPIO_InitStruct.Pin = LCD_D7_Pin|LCD_D6_Pin|LCD_D5_Pin|LCD_D4_Pin
			|LCD_E_Pin|LCD_RS_Pin|VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : ARD_D1_Pin ARD_D0_Pin */
	GPIO_InitStruct.Pin = ARD_D1_Pin|ARD_D0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : ARD_D10_Pin Buzzer_Pin SPBTLE_RF_RST_Pin ARD_D9_Pin */
	GPIO_InitStruct.Pin = ARD_D10_Pin|Buzzer_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin */
	GPIO_InitStruct.Pin = ARD_D13_Pin|ARD_D12_Pin|ARD_D11_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : ARD_D3_Pin */
	GPIO_InitStruct.Pin = ARD_D3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : ARD_D6_Pin */
	GPIO_InitStruct.Pin = ARD_D6_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin LED2_Pin
                           SPSGRF_915_SDN_Pin ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
	GPIO_InitStruct.Pin = ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
			|SPSGRF_915_SDN_Pin|ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin ARD_D2_Pin HTS221_DRDY_EXTI15_Pin
                           PMOD_IRQ_EXTI12_Pin */
	GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|ARD_D2_Pin|HTS221_DRDY_EXTI15_Pin
			|PMOD_IRQ_EXTI12_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
	GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
	GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
	GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
	GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin|PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : ARD_D15_Pin ARD_D14_Pin */
	GPIO_InitStruct.Pin = ARD_D15_Pin|ARD_D14_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTemp_monitor */
/**
 * @brief  Function implementing the Temp_monitor thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTemp_monitor */
void StartTemp_monitor(void const * argument)
{
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for(;;)
	{
		osMutexWait(myMutex_tempHandle, 1000);
		temp_value = BSP_TSENSOR_ReadTemp();
		osMutexRelease(myMutex_tempHandle);
		osDelay(500);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartHum_monitor */
/**
 * @brief Function implementing the Hum_monitor thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartHum_monitor */
void StartHum_monitor(void const * argument)
{
	/* USER CODE BEGIN StartHum_monitor */
	/* Infinite loop */
	for(;;)
	{
		osMutexWait(myMutex_humHandle, 1000);
		hum_value = BSP_HSENSOR_ReadHumidity();
		osMutexRelease(myMutex_humHandle);
		osDelay(500);
	}
	/* USER CODE END StartHum_monitor */
}

/* USER CODE BEGIN Header_StartLCD_temp */
/**
 * @brief Function implementing the LCD_temp thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLCD_temp */
void StartLCD_temp(void const * argument)
{
	/* USER CODE BEGIN StartLCD_temp */
	char str_tmpT[100] = ""; // Formatted message to display the temperature value
	double temp_local = 0; //Variabile locale su cui copiamo il valore di temperatura
	/* Infinite loop */
	for(;;)
	{
		osMutexWait(myMutex_tempHandle, 1000);
		temp_local = temp_value;
		osMutexRelease(myMutex_tempHandle);

		osMutexWait(myMutex_LCDHandle, 10000);
		clear();
		int tmpIntT1 = temp_local;
		float tmpFracT = temp_local - tmpIntT1;
		int tmpIntT2 = trunc(tmpFracT * 100);
		snprintf(str_tmpT,100,"%d.%02d Celsius", tmpIntT1, tmpIntT2);
		setCursor(0, 0);
		print("TEMPERATURE"); //Print this
		setCursor(0, 1); //At secound row first column
		print(str_tmpT); //Print this
		osDelay(2500);
		osMutexRelease(myMutex_LCDHandle);
		osDelay(1000);
	}
	/* USER CODE END StartLCD_temp */
}

/* USER CODE BEGIN Header_StartLCD_hum */
/**
 * @brief Function implementing the LCD_hum thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLCD_hum */
void StartLCD_hum(void const * argument)
{
	/* USER CODE BEGIN StartLCD_hum */
	char str_tmpH[100] = ""; // Formatted message to display the humidity value
	double hum_local = 0; //Variabile locale su cui copiamo il valore di umidità
	/* Infinite loop */
	for(;;)
	{
		osMutexWait(myMutex_humHandle, 1000);
		hum_local = hum_value;
		osMutexRelease(myMutex_humHandle);

		osMutexWait(myMutex_LCDHandle, 10000);
		clear();
		int tmpIntH1 = hum_local;
		float tmpFracH = hum_local - tmpIntH1;
		int tmpIntH2 = trunc(tmpFracH * 100);
		snprintf(str_tmpH,100,"%d.%02d RH", tmpIntH1, tmpIntH2);
		setCursor(0, 0);
		print("UMIDITY"); //Print this
		setCursor(0, 1); //At second row first column
		print(str_tmpH); //Print this
		osDelay(2500);
		osMutexRelease(myMutex_LCDHandle);
		osDelay(1000);
	}
	/* USER CODE END StartLCD_hum */
}

/* USER CODE BEGIN Header_StartServo_manager */
/**
 * @brief Function implementing the Servo_manager thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartServo_manager */
void StartServo_manager(void const * argument)
{
	/* USER CODE BEGIN StartServo_manager */
	TIM2->PSC = 71;
	TIM2->ARR = 20000;
	TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;
	TIM2->BDTR |= TIM_BDTR_MOE;
	TIM2->CCER |= TIM_CCER_CC1E;
	TIM2->CR1 |= TIM_CR1_ARPE;
	TIM2->EGR |= TIM_EGR_UG;
	TIM2->CR1 |= TIM_CR1_CEN;

	bool servo_actuated = false;
	TIM2->CCR4 = 5.556*0 + 1000;
	HAL_Delay(200);

	int emergency_local = 0;
	/* Infinite loop */
	for(;;)
	{
		//Qui devo capire se siamo in emergenza dal flag o sem, e quindi muovere il
		// servo di conseguenza
		osMutexWait(myMutexEmergencyHandle, 10000);
		emergency_local = emergency_temp;
		osMutexRelease(myMutexEmergencyHandle);

		if((emergency_local == 1 /*Siamo in emergenza*/) && (servo_actuated == false)){
			servo_actuated = true;
			TIM2->CCR4 = 5.556*300 + 1000;
			HAL_Delay(500);
		} else if ((emergency_local == 0 /*Non siamo in emergenza*/) && (servo_actuated)){
			servo_actuated = false;
			TIM2->CCR4 = 5.556*0 + 1000;
			HAL_Delay(500);
		}
		osDelay(2000);
	}
	/* USER CODE END StartServo_manager */
}

/* USER CODE BEGIN Header_StartT_ProblemDetect */
/**
 * @brief Function implementing the T_ProblemDetect thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartT_ProblemDetect */
void StartT_ProblemDetect(void const * argument)
{
	/* USER CODE BEGIN StartT_ProblemDetect */
	/* Infinite loop */
	for(;;)
	{
		//Qui leggiamo temp e in caso di problemi lo segnaliamo
		osDelay(10000);
		osMutexWait(myMutexEmergencyHandle, 10000);
		emergency_temp = 1;
		osMutexRelease(myMutexEmergencyHandle);
		osDelay(10000);
		osMutexWait(myMutexEmergencyHandle, 10000);
		emergency_temp = 0;
		osMutexRelease(myMutexEmergencyHandle);
	}
	/* USER CODE END StartT_ProblemDetect */
}

/* USER CODE BEGIN Header_StartH_ProblemDetect */
/**
 * @brief Function implementing the H_ProblemDetect thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartH_ProblemDetect */
void StartH_ProblemDetect(void const * argument)
{
	/* USER CODE BEGIN StartH_ProblemDetect */
	/* Infinite loop */
	for(;;)
	{
		//Qui leggiamo hum e in caso di problemi lo segnaliamo
		osDelay(1000);
	}
	/* USER CODE END StartH_ProblemDetect */
}

/* USER CODE BEGIN Header_StartBuzzer_manager */
/**
 * @brief Function implementing the Buzzer_manager thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBuzzer_manager */
void StartBuzzer_manager(void const * argument)
{
	/* USER CODE BEGIN StartBuzzer_manager */
	int emergency_local = 0;
	/* Infinite loop */
	for(;;)
	{
		osMutexWait(myMutexEmergencyHandle, 10000);
		emergency_local = emergency_temp;
		osMutexRelease(myMutexEmergencyHandle);
		//emergency_local = leggicodiceemergenza();

		if(emergency_local == 1){
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
			osDelay(1000);
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
			osMutexRelease(myMutexEmergencyHandle);
		}

		osDelay(700);

	}
	/* USER CODE END StartBuzzer_manager */
}

/* USER CODE BEGIN Header_StartLCD_TempProblem */
/**
 * @brief Function implementing the LCD_TempProblem thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLCD_TempProblem */
void StartLCD_TempProblem(void const * argument)
{
	/* USER CODE BEGIN StartBuzzer_manager */
	int emergency_local = 0;
	/* Infinite loop */
	for(;;)
	{
		osMutexWait(myMutexEmergencyHandle, 10000);
		emergency_local = emergency_temp;
		osMutexRelease(myMutexEmergencyHandle);
		if(emergency_local == 1){
			osMutexWait(myMutex_LCDHandle, 10000);
			clear();
			setCursor(0, 0);
			print("EMERGENCY");
			osDelay(2000);
			clear();
			setCursor(0, 0);
			print("TEMP VALUE"); //Print this
			setCursor(0, 1); //At secound row first column
			print("OUT OF RANGE"); //Print this
			osDelay(2000);
			osMutexRelease(myMutex_LCDHandle);
		}
		osDelay(500);
	}
	/* USER CODE END StartLCD_TempProblem */
}

/* USER CODE BEGIN Header_StartLCD_HumProblem */
/**
 * @brief Function implementing the LCD_HumProblem thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLCD_HumProblem */
void StartLCD_HumProblem(void const * argument)
{
	/* USER CODE BEGIN StartLCD_HumProblem */

	/* Infinite loop */
	for(;;)
	{
		if(0/*Se siamo in emergenza per temp */){
			osMutexWait(myMutex_LCDHandle, 10000);
			clear();
			setCursor(0, 0);
			print("EMERGENCY");
			osDelay(2000);
			clear();
			setCursor(0, 0);
			print("HUMIDITY VALUE"); //Print this
			setCursor(0, 1); //At secound row first column
			print("OUT OF RANGE"); //Print this
			osDelay(2000);
			osMutexRelease(myMutex_LCDHandle);
		}
		osDelay(12000);
	}
	/* USER CODE END StartLCD_HumProblem */
}

/* USER CODE BEGIN Header_StartLED_manager */
/**
 * @brief Function implementing the LED_manager thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLED_manager */
void StartLED_manager(void const * argument)
{
	/* USER CODE BEGIN StartLED_manager */
	bool timers_started = false;
	//Questa flag serve a vedere se i timer sono attivi così
	// da agggiungere una consizione all'else if in modo da non entrare a stoppare i timer se
	// non sono stati effettivamente avviati ed allo stessso tempo di avviarli in continuazione
	int emergency_local = 0;
	/* Infinite loop */
	for(;;)
	{
		//Leggo la flag di emergenza
		osMutexWait(myMutexEmergencyHandle, 10000);
		emergency_local = emergency_temp;
		osMutexRelease(myMutexEmergencyHandle);



		if((timers_started == false)&&(emergency_local == 1 /*Inseriremo qui il check della variabile o sem che segnala l'emergenza*/)){
			timers_started = true;
			osTimerStart(myTimerLed1Handle, 500);
			osTimerStart(myTimerLed2Handle, 300);
		} else if ((timers_started) && (emergency_local == 0 /*Qui inseriamo check sul sem o variabile emergenza*/)){
			timers_started = false;
			osTimerStop(myTimerLed1Handle);
			osTimerStop(myTimerLed2Handle);
		}
		osDelay(500);

	}
	/* USER CODE END StartLED_manager */
}

/* USER CODE BEGIN Header_StartButton_manager */
/**
 * @brief Function implementing the Button_manager thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartButton_manager */
void StartButton_manager(void const * argument)
{
	/* USER CODE BEGIN StartButton_manager */
	/* Infinite loop */
	for(;;)
	{
		if(HAL_GPIO_ReadPin(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN) == GPIO_PIN_RESET){
			//Here when the button is pressed
			//Qui dobbiamo semplicemente riportare il flag o il semaforo che segnala l'emergenza
			//in modalità non-emergenza.
			osMutexWait(myMutexEmergencyHandle, 10000);
			emergency_temp = 0;
			osMutexRelease(myMutexEmergencyHandle);
		}

		osDelay(100);
	}
	/* USER CODE END StartButton_manager */
}

/* CallbackLed1 function */
void CallbackLed1(void const * argument)
{
	/* USER CODE BEGIN CallbackLed1 */
	HAL_GPIO_TogglePin (LED2_GPIO_PORT, LED2_PIN);

	/* USER CODE END CallbackLed1 */
}

/* CallbackLed2 function */
void CallbackLed2(void const * argument)
{
	/* USER CODE BEGIN CallbackLed2 */
	HAL_GPIO_TogglePin (LED3_WIFI__LED4_BLE_GPIO_Port, LED3_WIFI__LED4_BLE_Pin);

	/* USER CODE END CallbackLed2 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM17 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM17) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
