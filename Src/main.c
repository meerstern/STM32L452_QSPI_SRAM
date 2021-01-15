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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define WRITE_CMD			   0x02
#define READ_CMD               0x03
#define ESQI_CMD               0x38
#define RSTDQI_CMD			   0xFF
//#define ESDI_CMD			   0x3B
#define RDMR_CMD			   0x05
#define WRMR_CMD			   0x01
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
QSPI_HandleTypeDef hqspi;
DMA_HandleTypeDef hdma_quadspi;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define QSPIHandle hqspi
int _write (int fd, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 1000);
  return len;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int8_t SRAM_Write(uint32_t address, uint8_t *write_data, uint32_t write_length)
{
	QSPI_CommandTypeDef sCommand;
	HAL_StatusTypeDef res=0;
	sCommand.InstructionMode	 		= QSPI_INSTRUCTION_1_LINE;
	sCommand.Instruction			 	= WRITE_CMD;
	sCommand.AddressMode			 	= QSPI_ADDRESS_1_LINE;
	sCommand.Address		 			= address;
	sCommand.AddressSize			 	= QSPI_ADDRESS_24_BITS;
	sCommand.AlternateByteMode 			= QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DataMode					= QSPI_DATA_1_LINE;
	sCommand.DummyCycles			 	= 0;
	sCommand.DdrMode					= QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle			= QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode					= QSPI_SIOO_INST_EVERY_CMD;
	sCommand.NbData						= write_length;

	res+=HAL_QSPI_Command(&QSPIHandle, &sCommand, 0xFFF);
	res+=HAL_QSPI_Transmit(&QSPIHandle, write_data, 0xFFF);

	return res;
}

int8_t SRAM_QWrite(uint32_t address, uint8_t *write_data, uint32_t write_length)
{
	QSPI_CommandTypeDef sCommand;
	HAL_StatusTypeDef res=0;
	sCommand.InstructionMode	 		= QSPI_INSTRUCTION_4_LINES;
	sCommand.Instruction			 	= WRITE_CMD;
	sCommand.AddressMode			 	= QSPI_ADDRESS_4_LINES;
	sCommand.Address		 			= address;
	sCommand.AddressSize			 	= QSPI_ADDRESS_24_BITS;
	sCommand.AlternateByteMode 			= QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DataMode					= QSPI_DATA_4_LINES;
	sCommand.DummyCycles			 	= 0;
	sCommand.DdrMode					= QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle			= QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode					= QSPI_SIOO_INST_EVERY_CMD;
	sCommand.NbData						= write_length;

	res+=HAL_QSPI_Command(&QSPIHandle, &sCommand, 0xFFF);
	res+=HAL_QSPI_Transmit(&QSPIHandle, write_data, 0xFFF);

	return res;
}


int8_t SRAM_Read(uint32_t address, uint8_t *read_data, uint32_t read_length)
{
	QSPI_CommandTypeDef sCommand;
	HAL_StatusTypeDef res=0;
	sCommand.InstructionMode	 		= QSPI_INSTRUCTION_1_LINE;
	sCommand.Instruction			 	= READ_CMD;
	sCommand.AddressMode			 	= QSPI_ADDRESS_1_LINE;
	sCommand.Address		 			= address;
	sCommand.AddressSize			 	= QSPI_ADDRESS_24_BITS;
	sCommand.AlternateByteMode 			= QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DataMode					= QSPI_DATA_1_LINE;
	sCommand.DummyCycles			 	= 0;
	sCommand.DdrMode					= QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle			= QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode					= QSPI_SIOO_INST_EVERY_CMD;
	sCommand.NbData						= read_length;

	res+=HAL_QSPI_Command(&QSPIHandle, &sCommand, 0xFFF);
	res+=HAL_QSPI_Receive(&QSPIHandle, read_data, 0xFFF);

	return res;
}

int8_t SRAM_QRead(uint32_t address, uint8_t *read_data, uint32_t read_length)
{

	QSPI_CommandTypeDef sCommand;
	HAL_StatusTypeDef res=0;
	sCommand.InstructionMode	 		= QSPI_INSTRUCTION_4_LINES;
	sCommand.Instruction			 	= READ_CMD;
	sCommand.AddressMode			 	= QSPI_ADDRESS_4_LINES;
	sCommand.Address		 			= address;
	sCommand.AddressSize			 	= QSPI_ADDRESS_24_BITS;
	sCommand.AlternateByteMode 			= QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DataMode					= QSPI_DATA_4_LINES;
	sCommand.DummyCycles			 	= 2;
	sCommand.DdrMode					= QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle			= QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode					= QSPI_SIOO_INST_EVERY_CMD;
	sCommand.NbData						= read_length;

	res+=HAL_QSPI_Command(&QSPIHandle, &sCommand, 0xFFF);
	res+=HAL_QSPI_Receive(&QSPIHandle, read_data, 0xFFF);

	return res;
}

int8_t SRAM_SMode()
{

	QSPI_CommandTypeDef sCommand;
	HAL_StatusTypeDef res=0;
	sCommand.InstructionMode	 		= QSPI_INSTRUCTION_4_LINES;
	sCommand.Instruction			 	= RSTDQI_CMD;
	sCommand.AddressMode			 	= QSPI_ADDRESS_NONE;
	sCommand.Address		 			= 0;
	sCommand.AddressSize			 	= QSPI_ADDRESS_24_BITS;
	sCommand.AlternateByteMode 			= QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DataMode					= QSPI_DATA_NONE;
	sCommand.DummyCycles			 	= 0;
	sCommand.DdrMode					= QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle			= QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode					= QSPI_SIOO_INST_EVERY_CMD;
	sCommand.NbData						= 0;

	res=HAL_QSPI_Command(&QSPIHandle, &sCommand, 0xFFF);

	return res;
}

int8_t SRAM_QMode()
{

	QSPI_CommandTypeDef sCommand;
	HAL_StatusTypeDef res=0;

	sCommand.InstructionMode	 		= QSPI_INSTRUCTION_1_LINE;
	sCommand.Instruction			 	= ESQI_CMD;
	sCommand.AddressMode			 	= QSPI_ADDRESS_NONE;
	sCommand.Address		 			= 0;
	sCommand.AddressSize			 	= QSPI_ADDRESS_24_BITS;
	sCommand.AlternateByteMode 			= QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DataMode					= QSPI_DATA_NONE;
	sCommand.DummyCycles			 	= 0;
	sCommand.DdrMode					= QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle			= QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode					= QSPI_SIOO_INST_EVERY_CMD;
	sCommand.NbData						= 0;

	res=HAL_QSPI_Command(&QSPIHandle, &sCommand, 0xFFF);

	return res;
}

int8_t SRAM_ReadModeReg(uint8_t *val)
{
	QSPI_CommandTypeDef sCommand;
	HAL_StatusTypeDef res=0;
	sCommand.InstructionMode	 		= QSPI_INSTRUCTION_1_LINE;
	sCommand.Instruction			 	= RDMR_CMD;
	sCommand.AddressMode			 	= QSPI_ADDRESS_NONE;
	sCommand.Address		 			= 0;
	sCommand.AddressSize			 	= QSPI_ADDRESS_24_BITS;
	sCommand.AlternateByteMode 			= QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DataMode					= QSPI_DATA_1_LINE;
	sCommand.DummyCycles			 	= 0;
	sCommand.DdrMode					= QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle			= QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode					= QSPI_SIOO_INST_EVERY_CMD;
	sCommand.NbData						= 1;

	res+=HAL_QSPI_Command(&QSPIHandle, &sCommand, 0xFFF);
	res+=HAL_QSPI_Receive(&QSPIHandle, val, 0xFFF);
	val[0]=val[0]>>6;

	return res;
}

int8_t SRAM_WriteModeReg(uint8_t *val)
{
	QSPI_CommandTypeDef sCommand;
	HAL_StatusTypeDef res=0;
	sCommand.InstructionMode	 		= QSPI_INSTRUCTION_1_LINE;
	sCommand.Instruction			 	= WRMR_CMD;
	sCommand.AddressMode			 	= QSPI_ADDRESS_NONE;
	sCommand.Address		 			= 0;
	sCommand.AddressSize			 	= QSPI_ADDRESS_24_BITS;
	sCommand.AlternateByteMode 			= QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DataMode					= QSPI_DATA_1_LINE;
	sCommand.DummyCycles			 	= 0;
	sCommand.DdrMode					= QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle			= QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode					= QSPI_SIOO_INST_EVERY_CMD;
	sCommand.NbData						= 1;

	val[0]=val[0]<<6;
	res+=HAL_QSPI_Command(&QSPIHandle, &sCommand, 0xFFF);
	res+=HAL_QSPI_Transmit(&QSPIHandle, val, 0xFFF);

	return res;
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
  MX_QUADSPI_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("\n\rInitOK\n\r");
  printf("QSPI TEST FOR SRAM IS62WVS5128FB/IS65WVS5128FB\n\r");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

		uint8_t wData[0x100];
		uint8_t rData[0x100];
		uint32_t i;
		for(i =0;i<0x100;i ++)
		{
			wData[i] = i;
			rData[i] = 0;
		}

		uint8_t mode[1];

		SRAM_ReadModeReg(mode);
		//mode[0]=mode[0]>>6;
		printf("Mode Reg:%d\n\r",mode[0]);//0:Byte Mode, 1:Sequential Mode(default), 2:Page Mode, 3:Reserved

		printf("Start QSPI Single Mode\n");
		SRAM_Write(0x00, wData, 0x100);
		SRAM_Read(0x00, rData, 0x100);

		printf("QSPI Check Data : \r\n");
		for(i =0;i<0x100;i++)
		{
				if(rData[i] != wData[i])
				{
					printf("W:0x%02X R:0x%02X ",wData[i],rData[i]);
					while(1);
				}
		}
		printf("No Err\r\n");


		printf("QSPI Read Data : \r\n");
		for(i =0;i<0x10;i++)//Pick Up first 0x10 Data
		{
			printf("0x%02X  ",rData[i]);
		}
		printf("\r\n\r\n");

		SRAM_QMode();//QSPI Mode

		printf("Start QSPI Quad Mode\n");
		SRAM_QWrite(0x100, wData, 0x100);
		SRAM_QRead(0x100, rData, 0x100);

		printf("QSPI Check Data : \r\n");
		for(i =0;i<0x100;i++)
		{
				if(rData[i] != wData[i])
				{
					printf("W:0x%02X R:0x%02X ",wData[i],rData[i]);
					while(1);
				}
		}
		printf("No Err\r\n");

		printf("QSPI Read Data : \r\n");
		for(i =0;i<0x10;i++)//Pick Up first 0x10 Data
		{
			printf("0x%02X  ",rData[i]);
		}
		printf("\r\n\r\n");

		SRAM_SMode();//SPI Mode

		while(1);

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
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
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 24;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD4_Pin */
  GPIO_InitStruct.Pin = LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD4_GPIO_Port, &GPIO_InitStruct);

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
