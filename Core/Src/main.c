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
#include "number.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define No_Op  0x00
#define Digit0 0x01
#define Digit1 0x02
#define Digit2 0x03
#define Digit3 0x04
#define Digit4 0x05
#define Digit5 0x06
#define Digit6 0x07
#define Digit7 0x08

#define DecodeMode  0x09
#define Intensity   0x0A
#define ScanLimit   0x0B
#define Shutdown    0x0C
#define DisplayTest 0x0F
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
void MAX7219_write(uint8_t address, uint8_t data);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void MAX7219_write(uint8_t address, uint8_t data)
{
	uint8_t buf[2]={0};
	buf[0]=address;
	buf[1]=data;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,buf, 2,1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
//	HAL_Delay(1);
}
uint8_t booltoByte(int (*ShowArray_ptr)[8])
{
	uint8_t data = 0x00;

  for (int i = 0; i < 8; i++)
    data = data << 1 | ShowArray_ptr[0][i];

  return data;
}
void SetPoint(uint8_t Seg,uint8_t value)
{
	MAX7219_write(Seg, value);
}
void draw(int (*ShowArray_ptr)[8])
{
  MAX7219_write(Digit0, booltoByte(&ShowArray_ptr[0]));
  MAX7219_write(Digit1, booltoByte(&ShowArray_ptr[1]));
  MAX7219_write(Digit2, booltoByte(&ShowArray_ptr[2]));
  MAX7219_write(Digit3, booltoByte(&ShowArray_ptr[3]));
  MAX7219_write(Digit4, booltoByte(&ShowArray_ptr[4]));
  MAX7219_write(Digit5, booltoByte(&ShowArray_ptr[5]));
  MAX7219_write(Digit6, booltoByte(&ShowArray_ptr[6]));
  MAX7219_write(Digit7, booltoByte(&ShowArray_ptr[7]));
}
void MAX7219clear()
{
  MAX7219_write(Digit0, 0x00);
  MAX7219_write(Digit1, 0x00);
  MAX7219_write(Digit2, 0x00);
  MAX7219_write(Digit3, 0x00);
  MAX7219_write(Digit4, 0x00);
  MAX7219_write(Digit5, 0x00);
  MAX7219_write(Digit6, 0x00);
  MAX7219_write(Digit7, 0x00);
}
void ShowNum()
{
	MAX7219clear();// clear show
	HAL_Delay(500);
	draw(&show_num0[0]);// show number 0
	HAL_Delay(500);
	draw(&show_num1[0]);// show number 1
	HAL_Delay(500);
	draw(&show_num2[0]);// show number 2
	HAL_Delay(500);
	draw(&show_num3[0]);// show number 3
	HAL_Delay(500);
	draw(&show_num4[0]);// show number 4
	HAL_Delay(500);
	draw(&show_num5[0]);// show number 5
	HAL_Delay(500);
	draw(&show_num6[0]);// show number 6
	HAL_Delay(500);
	draw(&show_num7[0]);// show number 7
	HAL_Delay(500);
	draw(&show_num8[0]);// show number 8
	HAL_Delay(500);
	draw(&show_num9[0]);// show number 9
	HAL_Delay(500);
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
  MAX7219_write(DecodeMode, 0x00);
  MAX7219_write(Intensity, 0x02);
  MAX7219_write(Shutdown, 0x01);
  MAX7219_write(DisplayTest, 0x00);
  MAX7219_write(ScanLimit, 0x07);
  MAX7219clear();//Clear MAX7219 LED Show

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int j =0x01;
  while (1)
  {
	  ShowNum();
	  j =0x01;
	  for(int i=0;i<7;i++)
	  {
		  SetPoint(Digit0,j);
		  SetPoint(Digit1,j);
		  SetPoint(Digit2,j);
		  SetPoint(Digit3,j);
		  SetPoint(Digit4,j);
		  SetPoint(Digit5,j);
		  SetPoint(Digit6,j);
		  SetPoint(Digit7,j);
		  j = j<<1;
		  HAL_Delay(300);
	  }
	  for(int i=0;i<8;i++)
	  {
		  SetPoint(Digit0,j);
		  SetPoint(Digit1,j);
		  SetPoint(Digit2,j);
		  SetPoint(Digit3,j);
		  SetPoint(Digit4,j);
		  SetPoint(Digit5,j);
		  SetPoint(Digit6,j);
		  SetPoint(Digit7,j);
		  j = j>>1;
		  HAL_Delay(300);
	  }
	  MAX7219clear();
	  for(int i=1;i<=8;i++)
	  {
		  SetPoint(i,0xff);
		  SetPoint(i-1,0x00);
		  HAL_Delay(300);
	  }
	  j = 0x08;
	  for(int i=8;i>=1;i--)
	  {
		  SetPoint(i,0x00);
		  SetPoint(i-1,0xff);
		  HAL_Delay(300);
	  }

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
