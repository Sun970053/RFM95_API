/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "RFM95.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}

RFM95_t rfm95;
bool txFlag;
bool rxFlag;
uint8_t rxBuff[17];
//LoRa myLoRa;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool STM32_DIO0(void);
void STM32_NRST(bool val);
void STM32_NSEL(bool val);
void STM32_DelayUs(uint32_t delay);
uint8_t STM32_SPI_Write(uint8_t* pTxData, uint8_t dataLen, uint32_t timeout);
uint8_t STM32_SPI_Read(uint8_t* pRxData, uint8_t dataLen, uint32_t timeout);
uint8_t STM32_SPI_WriteRead(uint8_t* pTxData, uint8_t* pRxData, uint8_t dataLen, uint32_t timeout);
uint8_t STM32_SPI_CheckState(void);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == BTN_Pin)
	{
		txFlag = true;
	}
//	if(GPIO_Pin == DIO0_Pin)
//	{
//		rxFlag = true;
//	}
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
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim4);
  printf("Hi\r\n");
  rfm95.DIO.DIO0 = STM32_DIO0;
  rfm95.NRST = STM32_NRST;
  rfm95.NSEL = STM32_NSEL;
  rfm95.DelayUs = STM32_DelayUs;
  rfm95.SPI_Write = STM32_SPI_Write;
  rfm95.SPI_Read = STM32_SPI_Read;
  rfm95.SPI_WriteRead = STM32_SPI_WriteRead;
  rfm95.SPI_CheckState = STM32_SPI_CheckState;
  rfm95.spi_ok = (uint8_t)HAL_SPI_STATE_READY;

  uint8_t ret = RFM95_LoRa_Init(&rfm95);
  if(ret != RFM95_OK)
  {
	  printf("Init... fail!\r\n");
	  printf("Error code: %d\r\n", ret);
  }
  else
  {
	  printf("Init... success!\r\n");
  }
  RFM95_LoRa_setSyncWord(&rfm95, 0x34);

//  RFM95_LoRa_prepareReceive(&rfm95, true);
//  myLoRa = newLoRa();
//
//  	myLoRa.CS_port         = NSS_GPIO_Port;
//  	myLoRa.CS_pin          = NSS_Pin;
//  	myLoRa.reset_port      = RST_GPIO_Port;
//  	myLoRa.reset_pin       = RST_Pin;
//  	myLoRa.DIO0_port       = DIO0_GPIO_Port;
//  	myLoRa.DIO0_pin        = DIO0_Pin;
//  	myLoRa.hSPIx           = &hspi1;
//
//  	myLoRa.frequency             = 923400;             // default = 433 MHz
//  	myLoRa.spredingFactor        = SF_10;           // default = SF_7
//  	myLoRa.bandWidth             = BW_125KHz;       // default = BW_125KHz
//  	myLoRa.crcRate               = CR_4_5;          // default = CR_4_5
//  	myLoRa.power                 = POWER_20db;      // default = 20db
//  	myLoRa.overCurrentProtection = 150;             // default = 100 mA
//  //	myLoRa.preamble = 100;
//
//  	uint16_t LoRa_status = LoRa_init(&myLoRa);
//  	printf("LoRa initialization ..");
//  	if(LoRa_status == LORA_OK){
//  		printf(" success !\r\n");
//  	}
  uint8_t packet[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		  	  	  	  0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x78};
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //txFlag = true;
	  if(txFlag)
	  {
		  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);
		  ret = RFM95_LoRa_transmit(&rfm95, packet, 17);
		  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);
		  if(ret != RFM95_OK)
		  {
			  printf("Tx.. fail!\r\n");
			  printf("Error code: %d\r\n", ret);
		  }
		  else
		  {
			  printf("Tx... success!\r\n");
		  	  printf("Packet has been sent.\r\n");
		  	  printf("RSSI: %d\r\n", (int)RFM95_getCurrentRSSI(&rfm95));
		  	  printf("Random number: %d\r\n", (int)RFM95_LoRa_random(&rfm95));
		  }
		  RFM95_LoRa_prepareReceive(&rfm95, true);
		  txFlag = false;
	  }
//	  if(rxFlag)
//	  {
//		  ret = RFM95_LoRa_receive(&rfm95, rxBuff, 17);
//		  if(ret != RFM95_OK)
//		  {
//			  printf("Tx.. fail!\r\n");
//			  printf("Error code: %d\r\n", ret);
//		  }
//		  else
//		  {
//			  printf("Rx.. success!\r\n");
//			  for(int i = 0; i<17;i++)
//			  {
//				  printf("%02x ", rxBuff[i]);
//			  }
//			  printf("\r\n");
//		  }
//		  rxFlag = false;
//	  }
//	  HAL_Delay(5000);
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
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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
  htim4.Init.Prescaler = 100-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NSS_Pin|RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BLUE_Pin */
  GPIO_InitStruct.Pin = BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLUE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO0_Pin */
  GPIO_InitStruct.Pin = DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NSS_Pin RST_Pin */
  GPIO_InitStruct.Pin = NSS_Pin|RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
bool STM32_DIO0(void)
{
	return (bool)HAL_GPIO_ReadPin(DIO0_GPIO_Port, DIO0_Pin);
}

void STM32_NRST(bool val)
{
	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, (GPIO_PinState)val);
}

void STM32_NSEL(bool val)
{
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, (GPIO_PinState)val);
}

void STM32_DelayUs(uint32_t delay)
{
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	while((__HAL_TIM_GET_COUNTER(&htim4)) < delay);
}

uint8_t STM32_SPI_Write(uint8_t* pTxData, uint8_t dataLen, uint32_t timeout)
{
	return (uint8_t)HAL_SPI_Transmit(&hspi1, pTxData, dataLen, timeout);
}

uint8_t STM32_SPI_Read(uint8_t* pRxData, uint8_t dataLen, uint32_t timeout)
{
	return (uint8_t)HAL_SPI_Receive(&hspi1, pRxData, dataLen, timeout);
}

uint8_t STM32_SPI_WriteRead(uint8_t* pTxData, uint8_t* pRxData, uint8_t dataLen, uint32_t timeout)
{
	uint8_t* rxbuff = (uint8_t*)malloc((dataLen) * sizeof(uint8_t));
	uint8_t ret;
	ret = (uint8_t)HAL_SPI_TransmitReceive(&hspi1, pTxData, rxbuff, dataLen, timeout);
	for(int i = 0; i < dataLen - 1; i++)
	{
		pRxData[i] = rxbuff[i + 1];
	}
	free(rxbuff);
	return ret;
}

uint8_t STM32_SPI_CheckState(void)
{
	return (uint8_t)HAL_SPI_GetState(&hspi1);
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
