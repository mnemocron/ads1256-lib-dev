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
#include <stdio.h>
#include <ads1255.h>
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
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
ADS125X_t adc1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/

struct __FILE{
  int handle;
  /* Whatever you require here. If the only file you are using is */
  /* standard output using printf() for debugging, no file handling */
  /* is required. */
};

FILE __stdout;

int fputc(int ch, FILE *f){
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

int ferror(FILE *f){
  /* Your implementation of ferror(). */
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
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	
	adc1.csPort = *SPI2_CS_GPIO_Port;
	adc1.csPin  = SPI2_CS_Pin;
	adc1.drdyPort = *SPI2_DRDY_GPIO_Port;
	adc1.drdyPin  = SPI2_DRDY_Pin;
	adc1.vref = 2.5f;
	adc1.oscFreq = ADS125X_OSC_FREQ;
	adc1.hspix = &hspi2;
	
	printf("\n");
	printf("config...\n");
	ADS125X_Init(&adc1, &hspi2, ADS125X_DRATE_2_5SPS, ADS125X_PGA1, 0);
	printf("...done\n");
	
	int32_t adsCode = 0;
	float volt = 0.0f;
	uint8_t spiDat[5];
	uint8_t spiRx[5];
	float pga = 1.0f;
	float vref = 2.5f;
	
	/** 
PROOF OF HARDCODE WORKING

reset...
Status: 0x30
ID: 0x03
ADCON: 0x20
DRATE: 0xf0
IO   : 0xe1
attempting config...
ADCON: 0x20 / 0x20
DRATE: 0x03 / 0x03
IO   : 0x01 / 00
MUX  : 0x01
config MUX...
MUX  : 0x01
calibration...
config MUX (0, 1)...
MUX  : 0x01
-4.9360775948 (0xff81a2ed)
config MUX (2, 3)...
MUX  : 0x23
-4.9361495972 (0xff81a274)
	*/
	
/**
config...
STATUS: 0x30
ADCON: 0x20
DRATE: 0x03
IO   : 0x01
...done
MUX  : 0x01
RDATA: 0057e9
MUX  : 0x23
RDATA: 0057e9
0.013414027169347, 0.013414027169347

*/
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		
/*HARDCODE MACHEN SO: */
		spiDat[0] = ADS125X_CMD_WREG | ADS125X_REG_MUX;	
		spiDat[1] = 1 -1;  // payload length = 3 bytes -1
		spiDat[2] = ADS125X_MUXP_AIN0 | ADS125X_MUXN_AIN1;
		while(HAL_GPIO_ReadPin(SPI2_DRDY_GPIO_Port, SPI2_DRDY_Pin) == GPIO_PIN_SET);  // wait for DRDY to go low
		HAL_SPI_Transmit(&hspi2, spiDat, 3, 10);
		HAL_Delay(1);
		// read back
		spiDat[0] = ADS125X_CMD_RREG | ADS125X_REG_MUX;
		spiDat[1] = 1 -1; // read 1 bytes
		while(HAL_GPIO_ReadPin(SPI2_DRDY_GPIO_Port, SPI2_DRDY_Pin) == GPIO_PIN_SET);  // wait for DRDY to go low
		HAL_SPI_Transmit(&hspi2, spiDat, 2, 10);
		HAL_Delay(1);
		HAL_SPI_Receive(&hspi2, spiRx, 1, 10);
		printf("MUX  : %#.2x\n", spiRx[0]);
		//ADS125X_ChannelDiff_Set(&adc1, ADS125X_MUXP_AIN0, ADS125X_MUXN_AIN1);
		// read Sensor
		volt = ADS125X_ADC_ReadVolt(&adc1);
		printf("%.10f (%#.6x)\n", volt, adsCode);
		HAL_Delay(250);
		
		
		spiDat[0] = ADS125X_CMD_WREG | ADS125X_REG_MUX;	
		spiDat[1] = 1 -1;  // payload length = 3 bytes -1
		spiDat[2] = ADS125X_MUXP_AIN2 | ADS125X_MUXN_AIN3;
		while(HAL_GPIO_ReadPin(SPI2_DRDY_GPIO_Port, SPI2_DRDY_Pin) == GPIO_PIN_SET);  // wait for DRDY to go low
		HAL_SPI_Transmit(&hspi2, spiDat, 3, 10);
		HAL_Delay(1);
		// read back
		spiDat[0] = ADS125X_CMD_RREG | ADS125X_REG_MUX;
		spiDat[1] = 1 -1; // read 1 bytes
		while(HAL_GPIO_ReadPin(SPI2_DRDY_GPIO_Port, SPI2_DRDY_Pin) == GPIO_PIN_SET);  // wait for DRDY to go low
		HAL_SPI_Transmit(&hspi2, spiDat, 2, 10);
		HAL_Delay(1);
		HAL_SPI_Receive(&hspi2, spiRx, 1, 10);
		printf("MUX  : %#.2x\n", spiRx[0]);
		// ADS125X_ChannelDiff_Set(&adc1, ADS125X_MUXP_AIN2, ADS125X_MUXN_AIN3);
		// read Sensor
		volt = ADS125X_ADC_ReadVolt(&adc1);
		printf("%.10f (%#.6x)\n", volt, adsCode);
		HAL_Delay(250);
		
/* LIBRARY MACHEN SO: */
/* ALI NIX SCHULD: */
		/*
		float volt[2] = {0.0f, 0.0f};
		ADS125X_ChannelDiff_Set(&adc1, ADS125X_MUXP_AIN0, ADS125X_MUXN_AIN1);
		volt[0] = ADS125X_ADC_ReadVolt(&adc1);
		printf("%.15f\n", volt[0]);
		HAL_Delay(250);
		
		
		ADS125X_ChannelDiff_Set(&adc1, ADS125X_MUXP_AIN2, ADS125X_MUXN_AIN3);
		ADS125X_CMD_Send(&adc1, ADS125X_CMD_SYNC);
		HAL_Delay(100);
		volt[1] = ADS125X_ADC_ReadVolt(&adc1);
		printf("%.15f\n", volt[1]);
		HAL_Delay(150);
		*/
		
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_DRDY_Pin */
  GPIO_InitStruct.Pin = SPI2_DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI2_DRDY_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
