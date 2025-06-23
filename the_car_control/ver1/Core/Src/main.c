/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stdarg.h"
#include "stdio.h"
#include "string.h"
#include "kalman.h"
#include "math.h"
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define TRIGGER_PORT 	GPIOB
#define TRIGGER_PIN		GPIO_PIN_9

#define ECHO_PORT			GPIOB
#define ECHO_PIN			GPIO_PIN_8

char buff[40];
uint8_t data_rx[40];
uint8_t pos = 0;

void HAL_PRINTF(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    memset(buff, 0, sizeof(buff)); // X�a buffer
    vsnprintf(buff, sizeof(buff), format, args);
    va_end(args);
    HAL_UART_Transmit(&huart3, (uint8_t*)buff, strlen(buff), 1000);
}

void turn_right(){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 250);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 200);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);	
}

void turn_left(){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 250);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 200);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);	
}

void forward(){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 250);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 250);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);	
}

void go_back(){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 250);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 250);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);	
}

// Vừa tiến vừa rẽ phải
void forward_right() {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 250); // Động cơ trái nhanh
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 30);   // Động cơ phải dừng hoặc rất chậm
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);         // Trái tiến
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);         // Phải tiến
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
}

// Vừa tiến vừa rẽ trái
void forward_left() {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 30);  // Động cơ trái chậm
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 250); // Động cơ phải nhanh
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);         // Trái tiến
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);         // Phải tiến
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
}

// Vừa lùi vừa rẽ phải
void back_right() {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 250); // Động cơ trái nhanh
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 30);   // Động cơ phải dừng hoặc rất chậm
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);         // Trái lùi
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);         // Phải lùi
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
}

// Vừa lùi vừa rẽ trái
void back_left() {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 30);  // Động cơ trái chậm
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 250); // Động cơ phải nhanh
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);         // Trái lùi
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);         // Phải lùi
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
}

void xi_nhan_trai(uint8_t state){
	if(state==1){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
	}else{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
	}
}

void xi_nhan_phai(uint8_t state){
	if(state==1){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
	}else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
	}
}

void turn_led(uint8_t state){
	if(state==1){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
	}else{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);		
	}
}

void car_horn(uint8_t state){
	if(state==1){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1);
	}else{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
	}
}



void off(){
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);  // Động cơ phải tiến
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);  // Động cơ trái tiến
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
}

uint8_t data_rx1;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if(huart->Instance == huart3.Instance){
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				if(data_rx1 == 'S'){
            off();
        }else
        if(data_rx1 == 'F'){
            forward();
        }else
        if(data_rx1 == 'B'){
            go_back();
        }else
        if(data_rx1 == 'L'){
            turn_left();
        }else
        if(data_rx1 == 'R'){
            turn_right();
        }else
        if(data_rx1 == 'G'){  // Tien trai
            forward_left();
        }else
        if(data_rx1 == 'I'){  // Tien phai
            forward_right();
        }else
        if(data_rx1 == 'H'){  // Lui trai
            back_left();
        }else
        if(data_rx1 == 'J'){  // Lui phai
            back_right();
        }else
				if(data_rx1 == 'Z'){ // Xi nhan trai
						xi_nhan_trai(1);
				}else
				if(data_rx1 == 'X'){
						xi_nhan_trai(0);
				}
				if(data_rx1 == 'O'){ // Xi nhan phai
						xi_nhan_phai(1);
				}else
				if(data_rx1 == 'P'){ // Xi nhan phai
						xi_nhan_phai(0);
				}else
				if(data_rx1 == 'M'){ // Bat den
						turn_led(1);
				}else
				if(data_rx1 == 'N'){ // Tat den
						turn_led(0);
				}else
				if(data_rx1 == '1'){ // Bat coi
						car_horn(1);
				}else
				if(data_rx1 == '2'){ // Tat coi
						car_horn(0);
				}
        HAL_UART_Receive_IT(&huart3, &data_rx1, 1);
    }
}

float distance=0.0;

void SR05_Init(){
	HAL_TIM_Base_Start(&htim2);
}

void SR05_Trigger(){
	HAL_GPIO_WritePin(TRIGGER_PORT, TRIGGER_PIN, 1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(TRIGGER_PORT, TRIGGER_PIN, 0);
}

float SR05_Active(){
	uint16_t time;
	float dis;
	SR05_Trigger();
	while(!HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN));
	__HAL_TIM_SET_COUNTER(&htim2,0);
	while(HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN));
	time = __HAL_TIM_GET_COUNTER(&htim2);
	dis = (time*1.0)*(0.034/2);
	return dis;
}

void STOP_DISTANCE(){
	if(distance <= 3.0){
		off();
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
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart3, &data_rx1, 1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	SimpleKalmanFilter(1, 2, 0.001);
	SR05_Init();
	pos=0;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//read_joy();
//		distance=SR05_Active();
//		HAL_Delay(100);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim1.Init.Prescaler = 2811;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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

  /* USER CODE END TIM2_Init 2 */

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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 PA7
                           PA10 PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//    if(huart->Instance == huart3.Instance){
//        pos++;
//        if(data_rx[pos-1] == '\n'){
//            data_rx[pos] = '\0';
//            
//            if(strcmp((const char*)data_rx, "forward\r\n") == 0) {
//                forward();
//            }
//            else if(strcmp((const char*)data_rx, "go back\r\n") == 0) {
//                go_back();
//            }
//            else if(strcmp((const char*)data_rx, "turn right\r\n") == 0) {
//                turn_right();
//            }
//            else if(strcmp((const char*)data_rx, "turn left\r\n") == 0) {
//                turn_left();
//            }
//            else if(strcmp((const char*)data_rx, "led on\r\n") == 0) {
//                led(1);
//            }
//            else if(strcmp((const char*)data_rx, "led off\r\n") == 0) {
//                led(0);
//            }
//            else if(strcmp((const char*)data_rx, "horn on\r\n") == 0) {
//                car_horn(1);
//            }
//            else if(strcmp((const char*)data_rx, "horn off\r\n") == 0) {
//                car_horn(0);
//            }
//            
//            pos = 0;
//        }
//        HAL_UART_Receive_IT(&huart3, &data_rx[pos], 1);
//    }
//}
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
