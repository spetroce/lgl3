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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/// bit 15 - Status Register Reset, 1 = Reset
#define NCV7719_SSR 0x8000
/// bit 14 - Channel Group Select, 1 = HB [8:7] | 0 = HB [6:1]
#define NCV7719_HBSEL 0x4000
/// bit 13 - Underload Shutdown, 1 = Enabled
#define NCV7719_ULDSC 0x2000
/// bit 0 - VSx Overvoltage Lockout, 1 = Enabled, Global Lockout
#define NCV7719_OVLO 0x0001

//   SPI Command Input Definitions, Word 0, Channels 8 – 7 (NCV7719_HBSEL = 1)
/// bit 8 - Enable Half−Bridge 8, 0 = Hi−Z, 1 = Enabled
#define NCV7719_HBEN8 0x0100
/// bit 7 - Enable Half−Bridge 7, 0 = Hi−Z, 1 = Enabled
#define NCV7719_HBEN7 0x0080
/// bit 2 - Configure Half−Bridge 8, 0 = LS On & HS Off, 1 = LS Off & HS On
#define NCV7719_HBCNF8 0x0004
/// bit 1 - Configure Half−Bridge 7, 0 = LS On & HS Off, 1 = LS Off & HS On
#define NCV7719_HBCNF7 0x0002

//   SPI Command Input Definitions, Word 1, Channels 6 – 1 (NCV7719_HBSEL = 0)
/// bit 12 - Enable Half−Bridge 6, 0 = Hi−Z, 1 = Enabled
#define NCV7719_HBEN6 0x1000
/// bit 11 - Enable Half−Bridge 5, 0 = Hi−Z, 1 = Enabled
#define NCV7719_HBEN5 0x0800
/// bit 10 - Enable Half−Bridge 4, 0 = Hi−Z, 1 = Enabled
#define NCV7719_HBEN4 0x0400
/// bit 9 - Enable Half−Bridge 3, 0 = Hi−Z, 1 = Enabled
#define NCV7719_HBEN3 0x0200
/// bit 8 - Enable Half−Bridge 2, 0 = Hi−Z, 1 = Enabled
#define NCV7719_HBEN2 0x0100
/// bit 7 - Enable Half−Bridge 1, 0 = Hi−Z, 1 = Enabled
#define NCV7719_HBEN1 0x0080
/// bit 6 - Configure Half−Bridge 6, 0 = LS On & HS Off, 1 = LS Off & HS On
#define NCV7719_HBCNF6 0x0040
/// bit 5 - Configure Half−Bridge 5, 0 = LS On & HS Off, 1 = LS Off & HS On
#define NCV7719_HBCNF5 0x0020
/// bit 4 - Configure Half−Bridge 4, 0 = LS On & HS Off, 1 = LS Off & HS On
#define NCV7719_HBCNF4 0x0010
/// bit 3 - Configure Half−Bridge 3, 0 = LS On & HS Off, 1 = LS Off & HS On
#define NCV7719_HBCNF3 0x0008
/// bit 2 - Configure Half−Bridge 2, 0 = LS On & HS Off, 1 = LS Off & HS On
#define NCV7719_HBCNF2 0x0004
/// bit 1 - Configure Half−Bridge 1, 0 = LS On & HS Off, 1 = LS Off & HS On
#define NCV7719_HBCNF1 0x0002

/*
   A
F     B
   G
E     C
   D

       GFEDCBA
0 - 0b00111111
1 - 0b00000110  4 ON, 0 OFF
2 - 0b01011011  4 ON, 1 OFF
0 - 0b00111111  2 ON, 1 OFF

       GFEDCBA
0 - 0b00111111
1 - 0b00000110  4 ON, 0 OFF
2 - 0b01011011  4 ON, 1 OFF
3 - 0b01001111  1 ON, 1 OFF
4 - 0b01100110  1 ON, 2 OFF
5 - 0b01101101  2 ON, 1 OFF
6 - 0b01111101  1 ON, 0 OFF
0 - 0b00111111  1 ON, 1 OFF

       GFEDCBA
0 - 0b00111111
1 - 0b00000110  4 ON, 0 OFF
2 - 0b01011011  4 ON, 1 OFF
3 - 0b01001111  1 ON, 1 OFF
4 - 0b01100110  1 ON, 2 OFF
5 - 0b01101101  2 ON, 1 OFF
6 - 0b01111101  1 ON, 0 OFF
7 - 0b00000111  1 ON, 4 OFF
8 - 0b01111111  4 ON, 0 OFF
9 - 0b01101111  0 ON, 1 OFF
0 - 0b00111111  1 ON, 1 OFF
*/

void Ncv7719_SetDigit(const uint8_t digit,) {
  uint16_t tx_data[2];
  uint16_t digit_xor = next_digit ^ prev_digit;
}

uint16_t Drv8323_ReadSpi(SPI_HandleTypeDef *hspi,
                         const Drv8323RegisterAddrMask_t reg_addr) {
  uint16_t tx_data = Drv8323_BuildCtrlWord(CTRL_MODE_READ, reg_addr, 0x0000);
  uint16_t rx_data = 0xbeef;
  uint16_t tx_rx_data_size = 1;  // spi configured with frame data size 16 bits
  uint32_t time_out_ms = 1000;

  HAL_GPIO_WritePin(DRV_SPI1_NSS_GPIO_Port, DRV_SPI1_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(hspi, (uint8_t*)(&tx_data), (uint8_t*)(&rx_data),
                          tx_rx_data_size, time_out_ms);
  HAL_GPIO_WritePin(DRV_SPI1_NSS_GPIO_Port, DRV_SPI1_NSS_Pin, GPIO_PIN_SET);
  return (rx_data & DRV8323_DATA_MASK);
}


void Drv8323_WriteSpi(SPI_HandleTypeDef *hspi,
                      const Drv8323RegisterAddrMask_t reg_addr,
                      uint16_t reg_data) {
  uint16_t tx_data = Drv8323_BuildCtrlWord(CTRL_MODE_WRITE, reg_addr, reg_data);
  uint16_t tx_data_size = 1;  // spi configured with frame data size 16 bits
  uint32_t time_out_ms = 1000;

  HAL_GPIO_WritePin(DRV_SPI1_NSS_GPIO_Port, DRV_SPI1_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hspi, (uint8_t*)(&tx_data), tx_data_size, time_out_ms);
  HAL_GPIO_WritePin(DRV_SPI1_NSS_GPIO_Port, DRV_SPI1_NSS_Pin, GPIO_PIN_SET);
  return;
}


void Drv3823_Enable(const bool enable) {
  HAL_GPIO_WritePin(DRV_EN_GPIO_Output_GPIO_Port,
                    DRV_EN_GPIO_Output_Pin, 
                    enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
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
