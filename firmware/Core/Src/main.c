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
#include "tim.h"
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

#define NUM_DISPLAY 4
#define NUM_PULSES 40
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/*** The following variables are for the Ncv7719_SetDigit() state machine. ***/

/// Controls the state Ncv7719_SetDigit() is in for trying to set the reflectors
/// which need to be ON or the ones that need to be off. The boot-up value is
/// true, which means reflcetors that need to be turned on are always set first,
/// then this is toggled to handle the reflectors that need to be turned off.
bool g_handle_on_bits = true,
     g_clock_is_init = false,
     g_disable_all_output = false;
uint8_t g_set_digit_toggle_cnt = 0,
        g_current_disp_idx = 0,
        g_current_digit[NUM_DISPLAY] = {10, 10, 10, 10},
        g_next_digit[NUM_DISPLAY] = {11, 11, 11, 11};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
   A
F     B
   G
E     C
   D

Some notes looking at the number of bits that have to be turned on and off to go
from the previous digit to the next. Don't want to exceed 4 ON or 4 OFF. 

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
1 - 0b00000110  0 ON, 4 OFF
2 - 0b01011011  4 ON, 1 OFF
3 - 0b01001111  1 ON, 1 OFF
4 - 0b01100110  1 ON, 2 OFF
5 - 0b01101101  2 ON, 1 OFF
6 - 0b01111101  1 ON, 0 OFF
7 - 0b00000111  1 ON, 4 OFF
8 - 0b01111111  4 ON, 0 OFF
9 - 0b01101111  0 ON, 1 OFF
0 - 0b00111111  1 ON, 1 OFF

Use these on startup. Assuming init_prev on power up and going to init_next will
ensure we don't flip more than 4 reflectors at once. The init_next pattern also
ensures that the first digit we go to does not flip more than 4 reflectors and
also statistically the fewest number of reflectors needed to be flipped.
init_prev: 0b01110000
init_next: 0b00001111

NCV7719 SPI Commands
UPPERCASE letter bits are to configure output as high or low
lowercase letter bits are to enable or disable output (Hi-Z)
X and x are for the common H-Bridge

      Channels 8-7     Channels 1-6
             xg    XG     fedcbaFEDCBA 
0 - 0b0000000000000000 0000000001111110
1 - 0b0000000000000000 0000000000001100
2 - 0b0000000000000010 0000000000110110
3 - 0b0000000000000010 0000000000011110
4 - 0b0000000000000010 0000000001001100
5 - 0b0000000000000010 0000000001011010
6 - 0b0000000000000010 0000000001111010
7 - 0b0000000000000000 0000000000001110
8 - 0b0000000000000010 0000000001111110
9 - 0b0000000000000010 0000000001011110
p - 0b0000000000000010 0000000001100000 (init_prev)
n = 0b0000000000000000 0000000000011110 (init_next)

To get the Enable half shift bits, take the bits requiring no change from the
Configure Half-Bridge dataset and bit shift them left by 6. This will give the
correct bits needed to place in Hi-Z or not.
*/

void Ncv7719_SetDigit() {
  const uint8_t disp_idx = g_current_disp_idx;
  if (disp_idx >= NUM_DISPLAY) {
    return;
  }
  const uint8_t curr_digit = g_current_digit[disp_idx],
                next_digit = g_next_digit[disp_idx];
  #define DIGIT_CFG_ARRAY_LEN 12
  if (curr_digit >= DIGIT_CFG_ARRAY_LEN ||
      next_digit >= DIGIT_CFG_ARRAY_LEN) {
    return;
  }
  if (curr_digit == next_digit) {
    if (++g_current_disp_idx >= NUM_DISPLAY) g_current_disp_idx = 0;
    return;
  }
  const uint32_t digit_cfg[DIGIT_CFG_ARRAY_LEN] =
  //          xg    XG    fedcbaFEDCBA
    {0b00000000000000000000000001111110,  // 0
     0b00000000000000000000000000001100,  // 1
     0b00000000000000100000000000110110,  // 2
     0b00000000000000100000000000011110,  // 3
     0b00000000000000100000000001001100,  // 4
     0b00000000000000100000000001011010,  // 5
     0b00000000000000100000000001111010,  // 6
     0b00000000000000000000000000001110,  // 7
     0b00000000000000100000000001111110,  // 8
     0b00000000000000100000000001011110,  // 9
     0b00000000000000100000000001100000,  // init_prev
     0b00000000000000000000000000011110}; // init_next
                                        //       xg    XG    fedcbaFEDCBA
  const uint32_t reset_pin_bit_cfg_on = 0b00000000000001000000000000000000;  // X
  const uint32_t reset_pin_bit_en =  reset_pin_bit_cfg_on << 6;  // x
  // Also use digit_xor to specify which half-bridges need to be enabled
  const uint32_t digit_xor = curr_digit ^ digit_cfg[next_digit];
  const uint32_t bits_to_turn_on = digit_cfg[next_digit] & digit_xor,
                 bits_to_turn_off = curr_digit & digit_cfg[digit_xor];
  const uint32_t bits_to_turn_on_en = bits_to_turn_on << 6,
                 bits_to_turn_off_en = bits_to_turn_off << 6;
  uint32_t tx_data = 0;
  if (!g_disable_all_output) {
    tx_data = g_handle_on_bits ?
              (bits_to_turn_on | bits_to_turn_on_en | reset_pin_bit_en) :  // first
              (bits_to_turn_off_en | reset_pin_bit_cfg_on | reset_pin_bit_en);  // second
  }
  uint16_t * tx_data_word = (uint16_t*)&tx_data;
  uint16_t rx_data = 0;
  uint16_t tx_rx_data_size = 1;  // spi configured with 16 bits frame data size
  uint32_t time_out_ms = 1000;
  GPIO_TypeDef * gpio_port[] = {SPI1_CS_0_GPIO_Port, SPI1_CS_1_GPIO_Port, SPI1_CS_2_GPIO_Port, SPI1_CS_3_GPIO_Port};
  uint16_t gpio_pin[] = {SPI1_CS_0_Pin, SPI1_CS_1_Pin, SPI1_CS_2_Pin, SPI1_CS_3_Pin};
  HAL_GPIO_WritePin(gpio_port[disp_idx], gpio_pin[disp_idx], GPIO_PIN_RESET);
  if (false) {
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)tx_data_word, (uint8_t*)(&rx_data), tx_rx_data_size, time_out_ms);
  } else {
    HAL_SPI_Transmit(&hspi1, (uint8_t*)tx_data_word, tx_rx_data_size, time_out_ms);
  }
  HAL_GPIO_WritePin(gpio_port[disp_idx], gpio_pin[disp_idx], GPIO_PIN_SET);
  // Need minimum 5 microsecond wait here between CSB toggle
  int i;
  for (i = 0; i < 25; ++i) { }
  HAL_GPIO_WritePin(gpio_port[disp_idx], gpio_pin[disp_idx], GPIO_PIN_RESET);
  if (false) {
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)(tx_data_word+1), (uint8_t*)(&rx_data), tx_rx_data_size, time_out_ms);
  } else {
    HAL_SPI_Transmit(&hspi1, (uint8_t*)(tx_data_word+1), tx_rx_data_size, time_out_ms);
  }
  HAL_GPIO_WritePin(gpio_port[disp_idx], gpio_pin[disp_idx], GPIO_PIN_SET);
  if (g_disable_all_output) {
    g_disable_all_output = false;
    // This update to g_current_digit will cause the next call to
    // Ncv7719_SetDigit() to skip to the next disp_idx
    g_current_digit[disp_idx] = next_digit;
    // On boot up, we set the clock to the pattern specified by a digit 11. Once
    // the last digit (NUM_DISPLAY - 1) is set, the clock is initialized and we
    // can start displaying numbers.
    if (!g_clock_is_init &&
        next_digit == 11 && disp_idx == (NUM_DISPLAY - 1)) {
      g_clock_is_init = true;
    }
    return;
  }
  // When g_handle_on_bits is false, we are on an EVEN iteration of
  // Ncv7719_SetDigit() meaning we just toggled the ON bits and then toggled the
  // OFF bits. We can now increment the g_set_digit_toggle_cnt counter and
  // repeat until we reach NUM_PULSES. When that occurs, we are still left with
  // the OFF segment outputs enabled and need to set g_disable_all_output to
  // true so on the next call of this function, all the outputs are turned off.
  if (!g_handle_on_bits) {
    ++g_set_digit_toggle_cnt;
    if (g_set_digit_toggle_cnt == NUM_PULSES) {
      g_set_digit_toggle_cnt = 0;
      g_disable_all_output = true;
    }
  }
  // Always leave g_handle_on_bits true for the next time g_current_digit != next_digit
  g_handle_on_bits = !g_handle_on_bits;
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
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);  // 2000 Hz interupt
  uint16_t beef_word = 0xBEEF;
  HAL_SPI_Transmit(&hspi1, (uint8_t*)&beef_word, 1, 1000);

  // g_tim_call_set_digit frequency is 2000 Hz
  const uint32_t LED_ON_DURATION = 100,  // 50 ms
                 LED_BLINK_PERIOD = 2000;  // 1 sec
  const uint32_t LED_ON_START_COUNT = LED_BLINK_PERIOD - LED_ON_DURATION;
  uint32_t led_blink_count = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (g_tim_call_set_digit) {
      g_tim_call_set_digit = false;
      ++led_blink_count;
      Ncv7719_SetDigit();
    }
    // Blink Status LED
    if (led_blink_count == LED_ON_START_COUNT) {
      HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    } else if (led_blink_count == LED_BLINK_PERIOD) {
      HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
      led_blink_count = 0;
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
