/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fsm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_PERIOD 10/portTICK_RATE_MS		//10
#define BTN_PERIOD 25/portTICK_RATE_MS		//10
#define MSG_PERIOD 5/portTICK_RATE_MS		//5
#define MOV_PERIOD 100/portTICK_RATE_MS

#define ADC_PRIORITY 2
#define BTN_PRIORITY 0
#define MSG_PRIORITY 1
#define MOV_PRIORITY 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* Declaración de tareas */
osThreadId adcTaskHandle;
osThreadId btnTaskHandle;
osThreadId msgTaskHandle;
osThreadId movTaskHandle;

/* Declaración de semáforos */
//SemaphoreHandle_t xMutexBtn, xMutexPres, xMutexMov;

/* Declaración de mensajes */
//uint8_t send_note_on[3] = {0x90, 0x3C, 0x3C};
//uint8_t send_note_off[3] = {0x90, 0x3C, 0x00};
extern uint8_t midi_all_notes_off[3];

/* Declaración de flags */
int pres_flag;
int btn_flag;
int mov_flag;

/* Declaración de variables de pressure (externas) */
extern uint8_t adc_value_init;
extern uint8_t adc_value_now, adc_value_last;
extern uint8_t pres_val;

/* Declaración de variables de fingering (externas) */
extern int button_now[2];

/* Declaración de variables de movement (externas) */
extern uint8_t mod_value_now, mod_value_last;
extern LIS3DSH_DataScaled inc_value;

/* Declaración de otras variables */
LIS3DSH_InitTypeDef myAccConfigDef;
int i;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
void StartAdcTask(void const * argument);
void StartBtnTask(void const * argument);
void StartMsgTask(void const * argument);
void StartMovTask(void const * argument);

fsm_t* get_adc();
fsm_t* get_btn();
fsm_t* get_msg();
fsm_t* get_mov();
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
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
 // xMutexBtn = xSemaphoreCreateBinary();
 // xMutexPres = xSemaphoreCreateBinary();
 // xMutexMov = xSemaphoreCreateBinary();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  osThreadDef(adcTask, StartAdcTask, ADC_PRIORITY, 0, 128);
  adcTaskHandle = osThreadCreate(osThread(adcTask), NULL);

  osThreadDef(btnTask, StartBtnTask, BTN_PRIORITY, 0, 128);
  btnTaskHandle = osThreadCreate(osThread(btnTask), NULL);

  osThreadDef(msgTask, StartMsgTask, MSG_PRIORITY, 0, 128);
  msgTaskHandle = osThreadCreate(osThread(msgTask), NULL);

  osThreadDef(movTask, StartMovTask, MOV_PRIORITY, 0, 128);
  movTaskHandle = osThreadCreate(osThread(movTask), NULL);

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.BaudRate = 31250;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : btn_11_Pin btn_12_Pin btn_13_Pin btn_14_Pin 
                           btn_15_Pin btn_16_Pin btn_17_Pin btn_18_Pin 
                           btn_19_Pin btn_20_Pin */
  GPIO_InitStruct.Pin = btn_11_Pin|btn_12_Pin|btn_13_Pin|btn_14_Pin 
                          |btn_15_Pin|btn_16_Pin|btn_17_Pin|btn_18_Pin 
                          |btn_19_Pin|btn_20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : btn_7_Pin btn_8_Pin btn_9_Pin btn_10_Pin 
                           btn_1_Pin btn_2_Pin btn_3_Pin btn_4_Pin 
                           OTG_FS_OverCurrent_Pin btn5_Pin btn6_Pin */
  GPIO_InitStruct.Pin = btn_7_Pin|btn_8_Pin|btn_9_Pin|btn_10_Pin 
                          |btn_1_Pin|btn_2_Pin|btn_3_Pin|btn_4_Pin 
                          |OTG_FS_OverCurrent_Pin|btn5_Pin|btn6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


/*********************************************************************
*******************MÁQUINA DE ESTADOS ADC*****************************
**********************************************************************/

fsm_t* get_adc(){
	static fsm_trans_t adc_tt[] = {
			{IDLE_ADC, timeout_adc, IDLE_ADC, read_adc},
			{-1, NULL, -1, NULL},
	};
	return fsm_new(adc_tt);
}

void StartAdcTask(void const * argument){

	portTickType period = ADC_PERIOD;
	portTickType last;
	fsm_t* adc_fsm = get_adc();
	HAL_ADC_Start(&hadc1);

	/* Inicialización de la máquina */
	if(HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK){
		adc_value_init = 40;//HAL_ADC_GetValue(&hadc1);
	}
	adc_value_now = 0;
	adc_value_last = 0;
	pres_val = 0;

	/* Infinite loop */
	for(;;){
		//HAL_ADC_PollForConversion(&hadc1, 1000);
		last = xTaskGetTickCount();
		fsm_fire(adc_fsm);
		vTaskDelayUntil(&last, period);
	}
}


/*********************************************************************
*******************MÁQUINA DE ESTADOS BTN*****************************
**********************************************************************/

fsm_t* get_btn(){
	static fsm_trans_t btn_tt[] = {
			{IDLE_BTN, timeout_btn, IDLE_BTN, read_btn},
			{-1, NULL, -1, NULL},
	};
	return fsm_new(btn_tt);
}

void StartBtnTask(void const * argument){

	portTickType period = BTN_PERIOD;
	portTickType last;
	fsm_t* btn_fsm = get_btn();

	button_now[0] = 0;
	button_now[1] = 0;

	/* Infinite loop */
	for(;;){

		last = xTaskGetTickCount();
		fsm_fire(btn_fsm);
		vTaskDelayUntil(&last, period);
	}
}


/*********************************************************************
*******************MÁQUINA DE ESTADOS MSG*****************************
**********************************************************************/

fsm_t* get_msg(){
	static fsm_trans_t msg_tt[] = {
			{IDLE_MSG, check_btn_flag, IDLE_MSG, send_new_note},
			{IDLE_MSG, check_pres_flag, IDLE_MSG, send_new_pres},
			{IDLE_MSG, check_mov_flag, IDLE_MSG, send_new_mov},
			{-1, NULL, -1, NULL},
	};
	return fsm_new(msg_tt);
}

void StartMsgTask(void const * argument){

	portTickType period = MSG_PERIOD;
	portTickType last;
	fsm_t* msg_fsm = get_msg();
	HAL_UART_Transmit(&huart2, (uint8_t*)&midi_all_notes_off, 3, 100);

	/* Infinite loop */
	for(;;){

		last = xTaskGetTickCount();
		fsm_fire(msg_fsm);
		vTaskDelayUntil(&last, period);
	}
}



/*********************************************************************
*******************MÁQUINA DE ESTADOS MOV*****************************
**********************************************************************/

fsm_t* get_mov(){
	static fsm_trans_t mov_tt[] = {
			{IDLE_MOV, timeout_mov, IDLE_MOV, read_mov},
			{-1, NULL, -1, NULL},
	};
	return fsm_new(mov_tt);
}

void StartMovTask(void const * argument){

	portTickType period = MOV_PERIOD;
	portTickType last;
	fsm_t* mov_fsm = get_mov();
	inc_value.y = 0;
	mod_value_now = 0;
	mod_value_last = 0;
	mov_flag = 0;

	myAccConfigDef.dataRate = LIS3DSH_DATARATE_25;
	myAccConfigDef.fullScale = LIS3DSH_FULLSCALE_4;
	myAccConfigDef.antiAliasingBW = LIS3DSH_FILTER_BW_50;
	myAccConfigDef.enableAxes = LIS3DSH_Y_ENABLE;
	myAccConfigDef.interruptEnable = false;
	LIS3DSH_Init(&hspi1, &myAccConfigDef);


	/* Infinite loop */
	for(;;){

		last = xTaskGetTickCount();
		fsm_fire(mov_fsm);
		vTaskDelayUntil(&last, period);
	}
}

/* USER CODE END 4 */


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
