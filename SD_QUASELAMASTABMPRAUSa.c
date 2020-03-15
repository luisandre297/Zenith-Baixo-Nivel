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

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

void SD_INI();
void SD_CMD(uint8_t n_cmd, uint8_t *resp, uint32_t arg);
void SD_ACMD(uint8_t n_cmd , uint8_t *resp , uint32_t arg);
void SD_WRITE(uint8_t *file, uint8_t adress, uint8_t size);
void SD_READ(uint8_t *file, uint8_t adress, uint8_t size);


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
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  uint32_t i;
  uint8_t file[513], file0[513];
  for(i=1; i<=512 ; i++) file[i] = 0xa4;
  for(i=1; i<=512 ; i++) file0[i] = 0xff;
  SD_INI();
  SD_WRITE(file, 0xaa, 0);
  SD_READ(file0, 0xaa, 0);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  i++;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  HAL_GPIO_WritePin(CSS_SD_GPIO_Port, CSS_SD_Pin, GPIO_PIN_SET);

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

  /*Configure GPIO pin : CSS_SD_Pin */
  GPIO_InitStruct.Pin = CSS_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CSS_SD_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void SD_INI()
{
	uint8_t vet_ini[20],resp[6];
	uint32_t i;
	HAL_GPIO_WritePin(GPIOB,CSS_SD_Pin, GPIO_PIN_SET);
	for(i=0; i<=20 ; i++) vet_ini[i] = 0b11111111;
	HAL_SPI_Transmit(&hspi2,vet_ini,20,10);
	for(i=0; i<=100 ; i++)
	 {
		 SD_CMD(0, resp,0);
		 if(((resp[0]&0b10000000) == 0)||((resp[1]&0b10000000) == 0)) i= 101;
	 }
	 HAL_SPI_Transmit(&hspi2,vet_ini,20,10);
	 SD_CMD(8,resp,0x1AA);
	 HAL_SPI_Transmit(&hspi2,vet_ini,20,10);
	 SD_CMD(58,resp,0);
	 HAL_SPI_Transmit(&hspi2,vet_ini,20,10);
	 for(i=0; i<=100 ; i++)
	 {
		SD_ACMD(41,resp, 0x400000);
		HAL_SPI_Transmit(&hspi2,vet_ini,20,10);
	 	if((resp[0] == 0)||(resp[1] == 0)) i= 101;
	 }
	 SD_CMD(7,resp,0);
	 SD_CMD(16,resp,512);
}

void SD_CMD(uint8_t n_cmd , uint8_t *resp , uint32_t arg)
{
	uint8_t vet_cmd[6],a=6, cons = 0xff;

	if(n_cmd==0 ||n_cmd==17 || n_cmd==24) a = 2;

	vet_cmd[0] = (n_cmd|0b01000000)&0b01111111;
	vet_cmd[1] = arg>>24;
	vet_cmd[2] = arg>>16;
	vet_cmd[3] = arg>>8;
	vet_cmd[4] = arg;
	vet_cmd[5] = 0x95;
	if(n_cmd == 8)  vet_cmd[5] = 0x86;
	if(n_cmd == 1)  vet_cmd[5] = 0xF9;
	if(n_cmd == 41) vet_cmd[5] = 0x77;

	 HAL_SPI_Transmit(&hspi2,&cons,1,10);
	HAL_GPIO_WritePin(GPIOB,CSS_SD_Pin, GPIO_PIN_RESET);
	 HAL_SPI_Transmit(&hspi2,&cons,1,10);

	HAL_SPI_Transmit(&hspi2,vet_cmd,6,10);
	HAL_SPI_Receive(&hspi2,resp,a,10);

	 HAL_SPI_Transmit(&hspi2,&cons,1,10);
	if(n_cmd !=0 )HAL_GPIO_WritePin(GPIOB,CSS_SD_Pin, GPIO_PIN_SET);
	 HAL_SPI_Transmit(&hspi2,&cons,1,10);
}

void SD_ACMD(uint8_t n_cmd , uint8_t *resp , uint32_t arg)
{
	uint8_t vet_cmd[6],vet_cmd1[6],cons = 0xff,i;

	vet_cmd[0] = (55|0b01000000)&0b01111111;
	vet_cmd[1] = 0;
	vet_cmd[2] = 0;
	vet_cmd[3] = 0;
	vet_cmd[4] = 0;
	vet_cmd[5] = 0b01;

	vet_cmd1[0] = (n_cmd|0b01000000)&0b01111111;
	vet_cmd1[1] = arg>>24;
	vet_cmd1[2] = arg>>16;
	vet_cmd1[3] = arg>>8;
	vet_cmd1[4] = arg;
	vet_cmd1[5] = 0x95;

	     HAL_SPI_Transmit(&hspi2,&cons,1,10);
		HAL_GPIO_WritePin(GPIOB,CSS_SD_Pin, GPIO_PIN_RESET);
		 HAL_SPI_Transmit(&hspi2,&cons,1,10);

		HAL_SPI_Transmit(&hspi2,vet_cmd,6,10);
		HAL_SPI_Receive(&hspi2,resp,6,10);

		 HAL_SPI_Transmit(&hspi2,&cons,1,10);
		HAL_GPIO_WritePin(GPIOB,CSS_SD_Pin, GPIO_PIN_SET);
		 HAL_SPI_Transmit(&hspi2,&cons,1,10);

		 for(i=0; i<=20 ; i++) HAL_SPI_Transmit(&hspi2,&cons,1,10);

		 HAL_SPI_Transmit(&hspi2,&cons,1,10);
		HAL_GPIO_WritePin(GPIOB,CSS_SD_Pin, GPIO_PIN_RESET);
		 HAL_SPI_Transmit(&hspi2,&cons,1,10);

		HAL_SPI_Transmit(&hspi2,vet_cmd1,6,10);
		HAL_SPI_Receive(&hspi2,resp,6,10);

		 HAL_SPI_Transmit(&hspi2,&cons,1,10);
		HAL_GPIO_WritePin(GPIOB,CSS_SD_Pin, GPIO_PIN_SET);
		 HAL_SPI_Transmit(&hspi2,&cons,1,10);

}


void SD_WRITE(uint8_t *file, uint8_t adress, uint8_t size)
{
	uint8_t vet_cmd[6],resp[2], cons = 0xff;
	uint32_t i;

        file[0] = 0xfe;
		vet_cmd[0] = (24|0b01000000)&0b01111111;
		vet_cmd[1] = adress>>24;
		vet_cmd[2] = adress>>16;
		vet_cmd[3] = adress>>8;
		vet_cmd[4] = adress;
		vet_cmd[5] = 0x95;

		 HAL_SPI_Transmit(&hspi2,&cons,1,10);
		HAL_GPIO_WritePin(GPIOB,CSS_SD_Pin, GPIO_PIN_RESET);
		 HAL_SPI_Transmit(&hspi2,&cons,1,10);

		HAL_SPI_Transmit(&hspi2,vet_cmd,6,10);
		HAL_SPI_Receive(&hspi2,resp,2,10);

		HAL_SPI_Transmit(&hspi2,file,513,10);
		for(i=0; i<=1000 ; i++)
		{
			HAL_SPI_Receive(&hspi2,resp,2,10);
			if((resp[0]!= 0xff)||(resp[1]!= 0xff)) i = 1002;
		}

		 HAL_SPI_Transmit(&hspi2,&cons,1,10);
		HAL_GPIO_WritePin(GPIOB,CSS_SD_Pin, GPIO_PIN_SET);
		 HAL_SPI_Transmit(&hspi2,&cons,1,10);



}

void SD_READ(uint8_t *file, uint8_t adress, uint8_t size)
{
	uint8_t vet_cmd[6],resp[2], cons = 0xff;
	uint32_t i;

			vet_cmd[0] = (17|0b01000000)&0b01111111;
			vet_cmd[1] = adress>>24;
			vet_cmd[2] = adress>>16;
			vet_cmd[3] = adress>>8;
			vet_cmd[4] = adress;
			vet_cmd[5] = 0x95;

			 HAL_SPI_Transmit(&hspi2,&cons,1,10);
			HAL_GPIO_WritePin(GPIOB,CSS_SD_Pin, GPIO_PIN_RESET);
			 HAL_SPI_Transmit(&hspi2,&cons,1,10);

			HAL_SPI_Transmit(&hspi2,vet_cmd,6,10);
			HAL_SPI_Receive(&hspi2,resp,2,10);

			for(i=0; i<=1000 ; i++)
			{
				HAL_SPI_Receive(&hspi2,resp,1,10);
				if(resp[0] == 0xfe) i = 1002;
			}

			HAL_SPI_Receive(&hspi2,file,513,10);

			 HAL_SPI_Transmit(&hspi2,&cons,1,10);
			HAL_GPIO_WritePin(GPIOB,CSS_SD_Pin, GPIO_PIN_SET);
			 HAL_SPI_Transmit(&hspi2,&cons,1,10);
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
