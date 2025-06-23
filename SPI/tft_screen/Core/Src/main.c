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
#define 			RES_PORT 			GPIOC
#define 			RES_PIN 			GPIO_PIN_13
#define 			DC_PORT 			GPIOE
#define 			DC_PIN 				GPIO_PIN_5
#define 			CS_PORT 			GPIOC
#define 			CS_PIN 				GPIO_PIN_15
#define 			u8						uint8_t
#define 			u16						uint16_t
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
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void tft_write_cmd(u8 cmd){
	// Cho CS xuong 0 sau do len 1
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, 0);
	// De gui cmd can keo DC xuong 0
	HAL_GPIO_WritePin(DC_PORT, DC_PIN, 0);
	HAL_SPI_Transmit(&hspi1, &cmd, sizeof(cmd), 1000);
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, 1);
}

void tft_write_data(u8 data){
	// Cho CS xuong 0 sau do len 1
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, 0);
	// De gui data can keo DC len 1
	HAL_GPIO_WritePin(DC_PORT, DC_PIN, 1);
	HAL_SPI_Transmit(&hspi1, &data, sizeof(data), 1000);
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, 1);
}

void set_pos(u8 x1, u8 y1, u8 x2, u8 y2){
    tft_write_cmd(0x2A);
    tft_write_data(0x00);
    tft_write_data(x1);
    tft_write_data(0x00);
    tft_write_data(x2);
    
    tft_write_cmd(0x2B);
    tft_write_data(0x00);
    tft_write_data(y1);
    tft_write_data(0x00);
    tft_write_data(y2);
}

void tft_init(){
    HAL_GPIO_WritePin(RES_PORT, RES_PIN, 0);
    HAL_Delay(50);
    HAL_GPIO_WritePin(RES_PORT, RES_PIN, 1);
    HAL_Delay(50);
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, 0);

    // Software reset
    tft_write_cmd(0x01);
    HAL_Delay(150);

    // Sleep out
    tft_write_cmd(0x11);
    HAL_Delay(150);

    // Frame rate control
    tft_write_cmd(0xB1);
    tft_write_data(0x05);
    tft_write_data(0x3C);
    tft_write_data(0x3C);

    // Frame rate control (in idle mode)
    tft_write_cmd(0xB2);
    tft_write_data(0x05);
    tft_write_data(0x3C);
    tft_write_data(0x3C);

    // Frame rate control (in partial mode)
    tft_write_cmd(0xB3);
    tft_write_data(0x05);
    tft_write_data(0x3C);
    tft_write_data(0x3C);
    tft_write_data(0x05);
    tft_write_data(0x3C);
    tft_write_data(0x3C);

    // Display inversion control
    tft_write_cmd(0xB4);
    tft_write_data(0x03);

    // Power control
    tft_write_cmd(0xC0);
    tft_write_data(0x28);
    tft_write_data(0x08);
    tft_write_data(0x04);

    tft_write_cmd(0xC1);
    tft_write_data(0xC0);

    tft_write_cmd(0xC2);
    tft_write_data(0x0D);
    tft_write_data(0x00);

    tft_write_cmd(0xC3);
    tft_write_data(0x8D);
    tft_write_data(0x2A);

    tft_write_cmd(0xC4);
    tft_write_data(0x8D);
    tft_write_data(0xEE);

    // VCOM control
    tft_write_cmd(0xC5);
    tft_write_data(0x1A);

    // Gamma adjustment
    tft_write_cmd(0xE0);
    tft_write_data(0x04);
    tft_write_data(0x22);
    tft_write_data(0x07);
    tft_write_data(0x0A);
    tft_write_data(0x2E);
    tft_write_data(0x30);
    tft_write_data(0x25);
    tft_write_data(0x2A);
    tft_write_data(0x28);
    tft_write_data(0x26);
    tft_write_data(0x2E);
    tft_write_data(0x3A);
    tft_write_data(0x00);
    tft_write_data(0x01);
    tft_write_data(0x03);
    tft_write_data(0x13);

    tft_write_cmd(0xE1);
    tft_write_data(0x04);
    tft_write_data(0x16);
    tft_write_data(0x06);
    tft_write_data(0x0D);
    tft_write_data(0x2D);
    tft_write_data(0x26);
    tft_write_data(0x23);
    tft_write_data(0x27);
    tft_write_data(0x27);
    tft_write_data(0x25);
    tft_write_data(0x2D);
    tft_write_data(0x3B);
    tft_write_data(0x00);
    tft_write_data(0x01);
    tft_write_data(0x04);
    tft_write_data(0x13);

    // Memory Access Control (MADCTL) - Ð?m b?o RGB
    tft_write_cmd(0x36);
    tft_write_data(0x00); // RGB, không xoay

    // Pixel Format - RGB565
    tft_write_cmd(0x3A);
    tft_write_data(0x05);

    // Set display area
    set_pos(26, 1, 105, 160); // Offset cho ST7735S: X+26, Y+1

    // Display ON
    tft_write_cmd(0x29);
    HAL_Delay(150);

    HAL_GPIO_WritePin(CS_PORT, CS_PIN, 1);
}

void full_screen(u16 color){
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, 0);
    tft_write_cmd(0x2C);
    for(u16 i=0; i<80*160; i++){
				tft_write_data(color & 0xFF);  
        tft_write_data(color >> 8);    
				
        
    }
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, 1);
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	tft_init();
	full_screen(0xF800); // Ð? trong BGR
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
