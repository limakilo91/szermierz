/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PIEZO_ADC_TRESH 1500
#define POWER_OFF_TIME_MS 1*60*1000 //1 minuta

#define STATE_INIT 1
#define STATE_WAITING_RAND 2
#define STATE_WAITING_JEBS 3
#define STATE_POWER_OFF 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */
uint32_t ADC_Data[16]; //tablica na próbki
uint32_t currentState;
uint32_t LEDState;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

uint32_t startup();
uint32_t wait_for_jebs_blocking();
uint32_t wait_for_jebs_cyclical();
uint32_t time_difference(uint32_t, uint32_t);
void wait_random_time(uint32_t, uint32_t);
void power_off_blocking();
void power_off_cyclical();
void LED_on();
void LED_off();
uint32_t read_piezo();
uint32_t read_battery();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t startup(){
	uint32_t start = HAL_GetTick();
	uint32_t stop = 0;
	uint32_t toRet = 0;
//	currentState = STATE_INIT;

//	HAL_ADC_Start(&hadc1); //start adc
	LEDState = 1;
	LED_off();
	wait_for_jebs_blocking(PIEZO_ADC_TRESH); //czekam  na uderzenie włączające
	stop = HAL_GetTick();

	toRet = time_difference(stop,start);
	currentState = STATE_WAITING_JEBS;
	srand(toRet);
	return toRet;
}

uint32_t wait_for_jebs_blocking(){
	volatile uint32_t adcVal = 0;
	LED_on();
	while ( (adcVal = read_piezo()) < PIEZO_ADC_TRESH ){
		HAL_Delay(10);
	}
	LED_off();
	return adcVal;
}

uint32_t wait_for_jebs_cyclical(){
	static uint32_t flag = 0;
	static uint32_t start = 0;
	volatile uint32_t adcVal = 0;

	if (!flag){ //tylko przy pierwszym starcie, resetowane po uderzeniu
		flag = 1;
		start = HAL_GetTick();
		LED_on();
	}

//	LED_on();

	adcVal = read_piezo();

	if(time_difference(HAL_GetTick(),start)>POWER_OFF_TIME_MS){ //jesli minął czas do power offa
		currentState = STATE_POWER_OFF;
		flag = 0;
		return 0;
	}
	else{ //jeśli nie minął czas do power offa
		if (adcVal > PIEZO_ADC_TRESH){ //jeśli jebnięte
			flag = 0;
			LED_off();
			currentState = STATE_WAITING_RAND;
		}
	}

	return adcVal;
}

void wait_random_time(uint32_t initial_delay_ms, uint32_t max_wait_ms){
	HAL_Delay( initial_delay_ms + ((rand()%1000)*max_wait_ms)/1000 );
}

uint32_t time_difference(uint32_t stop, uint32_t start){
	uint64_t temp = 0;
	uint32_t toRet = 0;
	if (stop<start){ //jeżeli się przekręcił licznik
		temp = start+stop-0xFFFFFFFF;
		toRet = (uint32_t)(temp);
	}
	else{ // a jesli nie to luz :)
		toRet = stop - start;
	}

	return toRet;
}

void power_off_blocking(){

}

void power_off_cyclical(){
	LED_off();
	HAL_Delay(10);
	volatile uint32_t adcVal = read_piezo();
	if (adcVal > PIEZO_ADC_TRESH){
		LED_on();
		currentState = STATE_WAITING_JEBS;
		HAL_Delay(500); //uspokojenie po jebs, bo inaczej od razu po wyjściu ze "sleepa" zalicza trzepnięcie ;)
	}
}

void LED_on(){
	if(!LEDState){ //jeśli LEDState == 0
		LEDState = 1;
		HAL_GPIO_WritePin(LED_SW_GPIO_Port, LED_SW_Pin, GPIO_PIN_SET); //wlacz ledy
	}
}

void LED_off(){
	if(LEDState){//jeśli LEDState == 1
		LEDState = 0;
		HAL_GPIO_WritePin(LED_SW_GPIO_Port, LED_SW_Pin, GPIO_PIN_RESET); //wylacz ledy
	}
}

uint32_t read_piezo(){
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000); //rozpocznij konwersję adc
	ADC_Data[0] = HAL_ADC_GetValue(&hadc1);
	return ADC_Data[0];
}

uint32_t read_battery(){
	return 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	// GIT TEST
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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

//  startup();
  currentState = STATE_INIT;

  //Enable Low Power Run (HAL)
  HAL_Delay(2000);
  HAL_SuspendTick();
  HAL_PWREx_EnableLowPowerRunMode();
  HAL_ResumeTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
	  switch(currentState){
	  	  case STATE_INIT:
	  		  startup();
	  		  currentState = STATE_WAITING_JEBS;
	  		  break;

	  	  case STATE_WAITING_JEBS:
	  		  wait_for_jebs_cyclical();
	  		  break;

	  	  case STATE_WAITING_RAND:
	  		  wait_random_time(500, 2000);
	  		  currentState = STATE_WAITING_JEBS;
	  		  break;

	  	  case STATE_POWER_OFF:
	  		  power_off_cyclical();
	  		  break;

	  	  default:
	  		  currentState = STATE_POWER_OFF;
	  		  break;
	  }

//	  wait_random_time(100, 10000);
//	  wait_for_jebs_blocking(PIEZO_ADC_TRESH);

//	  if (ADC_Data[0] > 2500){ //jesli uderzenie
//		  HAL_Delay(100+(rand()%100)*100); // i czekaj losowy czas
//	  }
//	  HAL_ADC_Stop(&hadc1); // w sumie to nie wyłączaj adc

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV64;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_SEQ_FIXED;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_3CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_SW_GPIO_Port, LED_SW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_SW_Pin */
  GPIO_InitStruct.Pin = LED_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_SW_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
