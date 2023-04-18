/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @code copyrights: Dhinesh Kumar S B.E EIE, M.Tech EDT (Embedded Domain)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_customhid.h"
#include "pid.h"
#include "stdlib.h"
#include "math.h"
#include "stdio.h"
#include "i2c-lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Define pins for rotary encoder
#define ENCODER_A_Pin GPIO_PIN_6
#define ENCODER_A_GPIO_Port GPIOA
#define ENCODER_B_Pin GPIO_PIN_7
#define ENCODER_B_GPIO_Port GPIOA
#define ANGLE_PER_PULSE 0.9 // degrees per encoder pulse
#define PULSES_PER_REV 400 // pulses per revolution
/* Define the pins for the relays */
#define RELAY1_Pin GPIO_PIN_8
#define RELAY1_GPIO_Port GPIOA
#define RELAY2_Pin GPIO_PIN_9
#define RELAY2_GPIO_Port GPIOA


#define START_BUTTON GPIO_PIN_6
#define START_BUTTON_GPIO_Port GPIOB

#define STOP_BUTTON GPIO_PIN_7
#define STOP_BUTTON_GPIO_Port GPIOB


#define R1_PORT GPIOB
#define R1_PIN GPIO_PIN_12

#define R2_PORT GPIOB
#define R2_PIN GPIO_PIN_13

#define R3_PORT GPIOB
#define R3_PIN GPIO_PIN_14

#define R4_PORT GPIOB
#define R4_PIN GPIO_PIN_15

#define C1_PORT GPIOC
#define C1_PIN GPIO_PIN_6

#define C2_PORT GPIOC
#define C2_PIN GPIO_PIN_7

#define C3_PORT GPIOC
#define C3_PIN GPIO_PIN_8

#define C4_PORT GPIOC
#define C4_PIN GPIO_PIN_9
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart3;

TIM_HandleTypeDef htim1;

osThreadId InputControlHandle;
osThreadId EncoderContolHandle;
osThreadId PIDContolHandle;
osSemaphoreId myBinarySem01Handle;
osSemaphoreId myBinarySem02Handle;
osSemaphoreId myBinarySem03Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
void InputControlTask(void const * argument);
void EncoderTask(void const * argument);
void PIDControllerTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern USBD_HandleTypeDef hUsbDeviceFS;

typedef struct
{
	uint8_t MODIFIER;
	uint8_t RESERVED;
	uint8_t KEYCODE1;
	uint8_t KEYCODE2;
	uint8_t KEYCODE3;
	uint8_t KEYCODE4;
	uint8_t KEYCODE5;
	uint8_t KEYCODE6;
} keyboardHID;

keyboardHID keyboardhid = {0,0,0,0,0,0,0,0};


/*************************************** KEYPAD RELATED CODE STARTS HERE *********************************/


float key;

char read_keypad (void)
{
	/* Make ROW 1 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (R1_PORT, R1_PIN, GPIO_PIN_RESET);  //Pull the R1 low
	HAL_GPIO_WritePin (R2_PORT, R2_PIN, GPIO_PIN_SET);  // Pull the R2 High
	HAL_GPIO_WritePin (R3_PORT, R3_PIN, GPIO_PIN_SET);  // Pull the R3 High
	HAL_GPIO_WritePin (R4_PORT, R4_PIN, GPIO_PIN_SET);  // Pull the R4 High

	if (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)))   // if the Col 1 is low
	{
		while (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)));   // wait till the button is pressed
		return '1';
	}

	if (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)))   // if the Col 2 is low
	{
		while (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)));   // wait till the button is pressed
		return '2';
	}

	if (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)))   // if the Col 3 is low
	{
		while (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)));   // wait till the button is pressed
		return '3';
	}

	if (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)))   // if the Col 4 is low
	{
		while (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)));   // wait till the button is pressed
		return 'A';
	}

	/* Make ROW 2 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (R1_PORT, R1_PIN, GPIO_PIN_SET);  //Pull the R1 low
	HAL_GPIO_WritePin (R2_PORT, R2_PIN, GPIO_PIN_RESET);  // Pull the R2 High
	HAL_GPIO_WritePin (R3_PORT, R3_PIN, GPIO_PIN_SET);  // Pull the R3 High
	HAL_GPIO_WritePin (R4_PORT, R4_PIN, GPIO_PIN_SET);  // Pull the R4 High

	if (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)))   // if the Col 1 is low
	{
		while (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)));   // wait till the button is pressed
		return '4';
	}

	if (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)))   // if the Col 2 is low
	{
		while (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)));   // wait till the button is pressed
		return '5';
	}

	if (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)))   // if the Col 3 is low
	{
		while (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)));   // wait till the button is pressed
		return '6';
	}

	if (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)))   // if the Col 4 is low
	{
		while (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)));   // wait till the button is pressed
		return 'B';
	}


	/* Make ROW 3 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (R1_PORT, R1_PIN, GPIO_PIN_SET);  //Pull the R1 low
	HAL_GPIO_WritePin (R2_PORT, R2_PIN, GPIO_PIN_SET);  // Pull the R2 High
	HAL_GPIO_WritePin (R3_PORT, R3_PIN, GPIO_PIN_RESET);  // Pull the R3 High
	HAL_GPIO_WritePin (R4_PORT, R4_PIN, GPIO_PIN_SET);  // Pull the R4 High

	if (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)))   // if the Col 1 is low
	{
		while (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)));   // wait till the button is pressed
		return '7';
	}

	if (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)))   // if the Col 2 is low
	{
		while (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)));   // wait till the button is pressed
		return '8';
	}

	if (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)))   // if the Col 3 is low
	{
		while (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)));   // wait till the button is pressed
		return '9';
	}

	if (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)))   // if the Col 4 is low
	{
		while (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)));   // wait till the button is pressed
		return 'C';
	}


	/* Make ROW 4 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (R1_PORT, R1_PIN, GPIO_PIN_SET);  //Pull the R1 low
	HAL_GPIO_WritePin (R2_PORT, R2_PIN, GPIO_PIN_SET);  // Pull the R2 High
	HAL_GPIO_WritePin (R3_PORT, R3_PIN, GPIO_PIN_SET);  // Pull the R3 High
	HAL_GPIO_WritePin (R4_PORT, R4_PIN, GPIO_PIN_RESET);  // Pull the R4 High

	if (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)))   // if the Col 1 is low
	{
		while (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)));   // wait till the button is pressed
		return '*';
	}

	if (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)))   // if the Col 2 is low
	{
		while (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)));   // wait till the button is pressed
		return '0';
	}

	if (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)))   // if the Col 3 is low
	{
		while (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)));   // wait till the button is pressed
		return '#';
	}

	if (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)))   // if the Col 4 is low
	{
		while (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)));   // wait till the button is pressed
		return 'D';
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
  MX_LPUART1_UART_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_USB_Device_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of myBinarySem01 */
  osSemaphoreDef(myBinarySem01);
  myBinarySem01Handle = osSemaphoreCreate(osSemaphore(myBinarySem01), 1);

  /* definition and creation of myBinarySem02 */
  osSemaphoreDef(myBinarySem02);
  myBinarySem02Handle = osSemaphoreCreate(osSemaphore(myBinarySem02), 1);

  /* definition and creation of myBinarySem03 */
  osSemaphoreDef(myBinarySem03);
  myBinarySem03Handle = osSemaphoreCreate(osSemaphore(myBinarySem03), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of InputControl */
  osThreadDef(InputControl, InputControlTask, osPriorityNormal, 0, 128);
  InputControlHandle = osThreadCreate(osThread(InputControl), NULL);

  /* definition and creation of EncoderContol */
  osThreadDef(EncoderContol, EncoderTask, osPriorityNormal, 0, 128);
  EncoderContolHandle = osThreadCreate(osThread(EncoderContol), NULL);

  /* definition and creation of PIDContol */
  osThreadDef(PIDContol, PIDControllerTask, osPriorityAboveNormal, 0, 128);
  PIDContolHandle = osThreadCreate(osThread(PIDContol), NULL);

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hi2c1.Init.Timing = 0x20303E5D;
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
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
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
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA8 PA9 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
volatile int32_t counter = 0;
volatile int32_t temp = 0;
volatile int32_t buf;
volatile int32_t buffer, buff, bool;
volatile float angle = 0.0;
volatile float freq = 0.01;

/* PID parameters */
float Kp = 0.1;
float Ki = 0.0;
float Kd = 0.0;
float Ku = 1.0;  // Ultimate gain
float Tu = 1.0;  // Ultimate period
float Pu = 0.0;  // Ultimate frequency

/* Feedback parameters */
//volatile long counter = 0;
float setpoint; // Setpoint in degrees
float feedback = 0;
float error = 0;
float last_error = 0;
float integral = 0;
float derivative = 0;
float output = 0;
volatile float last_feedback;
float process_variable = 0.0;
float dt = 10;//(10 milliseconds)
float new_setpoint;
float prev_process_variable;
/* Anti-windup parameters */
float windup_guard = 100;
float proportional_on_measurement = 0;



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{//semtake
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	//xSemaphoreTakeFromISR(myBinarySem01Handle, portMAX_DELAY);
	if(GPIO_Pin == ENCODER_A_Pin) // Encoder pin A
		  {
		    if(HAL_GPIO_ReadPin(ENCODER_B_GPIO_Port, ENCODER_B_Pin) == GPIO_PIN_RESET) // Check pin B to determine direction
		    {
		      counter++; // Increment counter
		    }
		    else
		    {
		      counter--; // Decrement counter
		    }
		  }

		  else if(GPIO_Pin == ENCODER_B_Pin) // Encoder pin B
		  {
		    if(HAL_GPIO_ReadPin(ENCODER_A_GPIO_Port, ENCODER_A_Pin) == GPIO_PIN_RESET) // Check pin A to determine direction
		    {
		      counter--; // Decrement counter
		    }
		    else
		    {
		      counter++; // Increment counter
		    }
		  }

	xTaskNotifyFromISR( EncoderContolHandle, 0, eNoAction, &xHigherPriorityTaskWoken);
	  xSemaphoreGiveFromISR(myBinarySem01Handle, &xHigherPriorityTaskWoken);
	  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	//semgive
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_InputControlTask */
/**
  * @brief  Function implementing the InputControl thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_InputControlTask */
void InputControlTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  key = read_keypad ();

	  if (key != 0x01)
	  {
	      if (key == '.') {
	          keyboardhid.MODIFIER = 0x02;  // left shift pressed
	          keyboardhid.KEYCODE1 = 0x37;  // press '.'
	      }
	      else if(key == '\r\n')
	      {
	    	  keyboardhid.MODIFIER = 0x02;  // left shift pressed
	    	  keyboardhid.KEYCODE1 = 0x28;  // press 'Enter or Return'
	      }
	         else if (key >= '0' && key <= '9') {
	          if (key == '1') {
	              keyboardhid.KEYCODE1 = 0x1E;
	          } else if (key == '2') {
	              keyboardhid.KEYCODE1 = 0x1F;
	          } else if (key == '3') {
	              keyboardhid.KEYCODE1 = 0x20;
	          } else if (key == '4') {
	              keyboardhid.KEYCODE1 = 0x21;
	          } else if (key == '5') {
	              keyboardhid.KEYCODE1 = 0x22;
	          } else if (key == '6') {
	              keyboardhid.KEYCODE1 = 0x23;
	          } else if (key == '7') {
	              keyboardhid.KEYCODE1 = 0x24;
	          } else if (key == '8') {
	              keyboardhid.KEYCODE1 = 0x25;
	          } else if (key == '9') {
	              keyboardhid.KEYCODE1 = 0x26;
	          } else if (key == '0') {
	              keyboardhid.KEYCODE1 = 0x27;
	          }
	      } else {
	          // unsupported key, do nothing
	    	  lcd_send_cmd(0x80);
	    	  lcd_send_string("Invalid Key");
	    	  osDelay(1000);
	          return;
	      }

	      USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof (keyboardhid));
	      osDelay(50);
	      keyboardhid.MODIFIER = 0x00;  // shift release
	      keyboardhid.KEYCODE1 = 0x00;  // release key
	      USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof (keyboardhid));

	      //display the key on the LCD
	      lcd_send_cmd(0x85);
	      lcd_send_string("Enter the value in degrees: ");
	      lcd_send_data(key);
	      HAL_UART_Transmit(&hlpuart1, &key, 1, HAL_MAX_DELAY);

	  }
	  // update setpoint value
	  setpoint = key;

	  xSemaphoreGive(myBinarySem02Handle);



	  //osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_EncoderTask */
/**
* @brief Function implementing the EncoderContol thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_EncoderTask */
void EncoderTask(void const * argument)
{
  /* USER CODE BEGIN EncoderTask */
  /* Infinite loop */
  for(;;)
  {
	  lcd_init();
	  lcd_clear();
	 ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	  if(counter != temp)
	  {
		  angle = (float)counter * ANGLE_PER_PULSE / PULSES_PER_REV;
		  sprintf(buf, "Counter: %ld", counter);
		  lcd_SetCursor(0, 0); // Set cursor to first row, first column
		  lcd_WriteString(0, 0, buf); // Write counter value to LCD display
		  sprintf(buffer, "Angle: %f deg", angle);
		  lcd_SetCursor(1, 0); // Set cursor to second row, first column
		  lcd_WriteString(1, 0, buffer); // Write angle value to LCD display
		  temp = counter;
	  }
    osDelay(1);
  }
  /* USER CODE END EncoderTask */
}

/* USER CODE BEGIN Header_PIDControllerTask */
/**
* @brief Function implementing the PIDContol thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PIDControllerTask */
void PIDControllerTask(void const * argument)
{
  /* USER CODE BEGIN PIDControllerTask */
  /* Infinite loop */
  for(;;)
  {
	  xSemaphoreTake(myBinarySem02Handle, portMAX_DELAY);
	  		  if(setpoint && HAL_GPIO_ReadPin(START_BUTTON_GPIO_Port, START_BUTTON) == GPIO_PIN_SET)
	  		  	  {
	  		  		  HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_SET);
	  		  		  HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, GPIO_PIN_RESET);

	  		  		lcd_send_cmd(0x80);
	  		  		lcd_send_string("The valve is opening");
	  		  	  }

	  		  return process_variable;

	  double get_current_time()
	  {
		  const TickType_t ticks	 = xTaskGetTickCount();
		  const double seconds = (double) ticks / configTICK_RATE_HZ;
		  return seconds;
	  }

	  if (xSemaphoreTakeFromISR(myBinarySem01Handle, portMAX_DELAY) == pdTRUE)
	  {
	  // Read the feedback value from the encoder and convert it to angle */
	  //semtake
	  feedback = ((float)counter / 1600) * 360;


	  /* Calculate the error between the setpoint and feedback */
	 double last_time = get_current_time();
	 double start_time = get_current_time();
	 double end_time = start_time + dt;
	 while(get_current_time() < end_time)
	 {
		 if(setpoint > feedback)
			  {
				  error = setpoint - feedback;
			  }
			  else if(setpoint < feedback)
			  {
				  error = feedback - setpoint;
			  }
			  else
			  {
				  error = 0.0;
			  }

	 }

	 double duration = get_current_time() - last_time;
	  /* Calculate the integral error over time */
	  integral += error*duration;

	  /* Limit the integral error to avoid windup */
	/*  if (integral > windup_guard)
	  {
	    integral = windup_guard;
	  }
	  else if (integral < -windup_guard)
	  {
	    integral = -windup_guard;
	  }  */

	  /* Calculate the derivative error over time */
	  if (proportional_on_measurement)
	  {
	    derivative = (feedback - last_feedback)/duration;
	  }
	  else
	  {
	    derivative = (error - last_error)/duration;
	  }

	  /* Calculate the output value to the actuator using PID algorithm */
	  output = Kp * error + Ki * integral + Kd * derivative;
	  last_error = error;
	  last_time = get_current_time();
	  float input = output;
	  // Autotuning function for PID controller using Ziegler-Nichols method
	  // Perform step change in setpoint
	      new_setpoint = setpoint * 1.1;
	      prev_process_variable = process_variable;
	     /* while (abs(process_variable - new_setpoint) > 0.01)
	          {
	              process_variable = read_process_variable(new_setpoint);
	              osDelay(dt);
	          }
	      // Calculate ultimate gain and period
	         Ku = abs(new_setpoint - prev_process_variable) / (4 * process_variable);
	         Tu = dt/(output-setpoint); /*(float)counter / (float)freq;*/
	        // Pu = Tu / 1.2;


	         // Calculate PID controller parameters
	         //Kp = 0.6 * Ku;
	         //Ki = 1.2 * Ku / Tu;
	         //Kd = 0.075 * Ku * Pu;
	      if (error != 0)
	      {
	    	  double Ku = 4.0 / abs(output - input);
	    	  double Pu = (last_time - dt) / 2.2;
	    	  double new_Kp = 0.6 * Ku;
	    	  double new_Ki = 1.2 * Kp / Pu;
	    	  double new_Kd = 0.075 * Kp * Pu;

	    	  Kp = new_Kp;
	    	  Ki = new_Ki;
	    	  Kd = new_Kd;
	    	  output = Kp * error + Ki * integral + Kd * derivative;

	    	  sprintf(bool, "Kp: %f, Ki: %f, Kd: %f\r\n", Kp, Ki, Kd);
	    	  HAL_UART_Transmit(&hlpuart1, (uint8_t *)bool, strlen(bool), 150);
	      }
	      else
	      {
	    	  output = Kp * error + Ki * integral + Kd * derivative;
	      }


	  /* Send the output value to the actuator */
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, output);

	  /* Save the error and feedback values for the next iteration */


	  /* Send the value of counter in angle */
	  sprintf(buff,"Angle: %.2f\r\n", feedback);
	  HAL_UART_Transmit(&hlpuart1,(uint8_t)buff, strlen(buff), 500);

	  if (output >= 0)
	          {
	              HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_SET);
	              HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, GPIO_PIN_RESET);
	          }
	  else
	          {
	              HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_RESET);
	              HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, GPIO_PIN_SET);
	          }

		  if(HAL_GPIO_ReadPin(STOP_BUTTON_GPIO_Port, STOP_BUTTON) == GPIO_PIN_SET)
		  	  {
		  	  	  HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_RESET);
		  	      HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, GPIO_PIN_SET);

		  	    lcd_send_cmd(0x80);
		  	    lcd_send_string("The valve is closing");
		  	  }


	  /*HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, RESET);
	     osDelay(1000);
	     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, SET);
	     osDelay(1000);*/
    //osDelay(1);
  }//semgive

  }
  /* USER CODE END PIDControllerTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
