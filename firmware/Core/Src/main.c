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
#include "rtc.h"
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

/// SpiTransmit32() has a special interpretation for the value ENABLE_NCV_DRV,
/// which is to enable an NCV driver chip GPIO.
#define ENABLE_NCV_DRV  0xFFFFFFF
/// SpiTransmit32() has a special interpretation for the value DISABLE_NCV_DRV,
/// which is to disable an NCV driver chip GPIO.
#define DISABLE_NCV_DRV 0x7FFFFFF

#define SET_GPIO(port, pin) port->BSRR = pin;
#define RESET_GPIO(port, pin) port->BRR = pin;
/// Evaluates as true if a GPIO output is set
#define GPIO_IS_SET(port, pin) (port->ODR & pin)
/// Evaluates as true if a GPIO input is set
#define READ_GPIO(port, pin) (port->IDR & pin)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define TX_DATA_MAX_LEN 64
uint32_t g_tx_data[TX_DATA_MAX_LEN];
uint8_t g_disp_idx[TX_DATA_MAX_LEN];
uint32_t g_tx_data_write_idx = 0,
         g_tx_data_read_idx = 0,
         g_tx_data_len = 0;
/*** The following variables are for the Ncv7719_SetDigit() state machine. ***/
uint8_t g_ncv_current_digit[NUM_DISPLAY] = {11, 11, 11, 11},
        g_ncv_next_digit[NUM_DISPLAY] = {10, 10, 10, 10};
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

NCV7719 SPI Commands
UPPERCASE letter bits are to configure output as high or low
lowercase letter bits are to enable or disable output (Hi-Z)
X and x are for the common H-Bridge

0 - ABCDEF
1 -  BC
2 - AB DE G
3 - ABCD  G
4 -  BC  FG
5 - A CD FG
6 - A CDEFG
7 - ABC
8 - ABCDEFG
9 - ABCD FG

      Channels 8-7     Channels 1-6
             ae    AE     fdgxbcFDGXBC 
0 - 0b0000000000000110 0000000001100110
1 - 0b0000000000000000 0000000000000110
2 - 0b0000000000000110 0000000000110100
3 - 0b0000000000000100 0000000000110110
4 - 0b0000000000000000 0000000001010110
5 - 0b0000000000000100 0000000001110010
6 - 0b0000000000000110 0000000001110010
7 - 0b0000000000000100 0000000000000110
8 - 0b0000000000000110 0000000001110110
9 - 0b0000000000000100 0000000001110110

To get the Enable half shift bits, take the bits requiring no change from the
Configure Half-Bridge dataset and bit shift them left by 6. This will give the
correct bits needed to place in Hi-Z or not.

Channel 8 is a common tied to all the RESET pins of the 7-segment display.
+SET and -RESET will cause a segment to show.
-SET and +RESET will cause a segment to be hidden.
*/

uint32_t TxDataLen() {
  return g_tx_data_len;
}

void TxDataPush(const uint32_t data, const uint8_t disp_idx) {
  g_tx_data[g_tx_data_write_idx] = data;
  g_disp_idx[g_tx_data_write_idx] = disp_idx;
  if (++g_tx_data_write_idx >= TX_DATA_MAX_LEN) {
    g_tx_data_write_idx = 0;
  }
  ++g_tx_data_len;  // TODO: can overflow
}

bool TxDataPop(uint32_t * tx_data, uint8_t * disp_idx) {
  if (g_tx_data_len > 0) {
    *tx_data = g_tx_data[g_tx_data_read_idx];
    *disp_idx = g_disp_idx[g_tx_data_read_idx];
    if (++g_tx_data_read_idx >= TX_DATA_MAX_LEN) {
      g_tx_data_read_idx = 0;
    }
    --g_tx_data_len;
    return true;
  }
  return false;
}

bool TxDataNextDispIdx(uint8_t * disp_idx) {
  if (g_tx_data_len > 0) {
    *disp_idx = g_disp_idx[g_tx_data_read_idx];
    return true;
  }
  return false;
}

void SpiTransmit32() {
  uint32_t tx_data;
  uint8_t disp_idx;
  TxDataPop(&tx_data, &disp_idx);
  if (disp_idx >= NUM_DISPLAY) {
    return;
  }
  GPIO_TypeDef * ncv_en_gpio_port[] = {NCV_EN_1_GPIO_Port,
                                       NCV_EN_2_GPIO_Port,
                                       NCV_EN_3_GPIO_Port,
                                       NCV_EN_4_GPIO_Port};
  uint16_t ncv_en_gpio_pin[] = {NCV_EN_1_Pin,
                                NCV_EN_2_Pin,
                                NCV_EN_3_Pin,
                                NCV_EN_4_Pin};
  if (tx_data == ENABLE_NCV_DRV) {
    SET_GPIO(ncv_en_gpio_port[disp_idx], ncv_en_gpio_pin[disp_idx]);
    return;
  } else if (tx_data == DISABLE_NCV_DRV) {
    RESET_GPIO(ncv_en_gpio_port[disp_idx], ncv_en_gpio_pin[disp_idx]);
    return;
  }
  uint16_t tx_data_word[2];
  tx_data_word[0] = (uint16_t)(tx_data & 0xFFFFUL);
  tx_data_word[1] = (uint16_t)((tx_data >> 16) & 0xFFFFUL);
  // Set the NCV7719_HBSEL bit for the second word.
  tx_data_word[1] |= NCV7719_HBSEL;
  GPIO_TypeDef * spi_cs_gpio_port[] = {SPI1_CS_1_GPIO_Port,
                                       SPI1_CS_2_GPIO_Port,
                                       SPI1_CS_3_GPIO_Port,
                                       SPI1_CS_4_GPIO_Port};
  uint16_t spi_cs_gpio_pin[] = {SPI1_CS_1_Pin,
                                SPI1_CS_2_Pin,
                                SPI1_CS_3_Pin,
                                SPI1_CS_4_Pin};
  // Send first word over SPI (bits 0-15 of tx_data; NCV7719_HBSEL = 0)
  RESET_GPIO(spi_cs_gpio_port[disp_idx], spi_cs_gpio_pin[disp_idx])
  LL_SPI_TransmitData16(SPI1, tx_data_word[0]);
  RESET_GPIO(SPI1_MOSI_GPIO_Port, SPI1_MOSI_Pin)
  // Wait for transmit to complete
  while (SPI1->SR & SPI_SR_BSY);  // See LL_SPI_IsActiveFlag_BSY()
  SET_GPIO(spi_cs_gpio_port[disp_idx], spi_cs_gpio_pin[disp_idx])
  // Need minimum 5 microsecond wait here between CSB toggle. This empty loop
  // with 30 iterations makes a ~5.5 microsecond delay.
  int i;
  for (i = 0; i < 30; ++i) { }
  // Send second word over SPI (bits 16-31 of tx_data; NCV7719_HBSEL = 1)
  RESET_GPIO(spi_cs_gpio_port[disp_idx], spi_cs_gpio_pin[disp_idx])
  LL_SPI_TransmitData16(SPI1, tx_data_word[1]);
  RESET_GPIO(SPI1_MOSI_GPIO_Port, SPI1_MOSI_Pin)
  // Wait for SPI transmit to complete
  while (SPI1->SR & SPI_SR_BSY);
  SET_GPIO(spi_cs_gpio_port[disp_idx], spi_cs_gpio_pin[disp_idx])
}

void HideAllSegment(const uint8_t disp_idx) {
  if (disp_idx >= NUM_DISPLAY) {
    return;
  }
  TxDataPush(UINT32_MAX, disp_idx);
  //                  ae    AE0   fdgxbcFDGXBC0
  TxDataPush(0b00000001000001000000001100001000, disp_idx);
  TxDataPush(0b00000000000000000000101010001000, disp_idx);
  TxDataPush(0b00000000100000000001001000001000, disp_idx);
  TxDataPush(0b00000000000000000000011000001000, disp_idx);
  TxDataPush(0, disp_idx);
}

void ShowAllSegment(const uint8_t disp_idx) {
  if (disp_idx >= NUM_DISPLAY) {
    return;
  }
  TxDataPush(UINT32_MAX, disp_idx);
  //                  ae    AE0   fdgxbcFDGXBC0
  TxDataPush(0b00000001000001000000001100000100, disp_idx);
  TxDataPush(0b00000000000000000000101010100010, disp_idx);
  TxDataPush(0b00000000100000100001001001000000, disp_idx);
  TxDataPush(0b00000000000000000000011000010000, disp_idx);
  TxDataPush(0, disp_idx);
}

void SetDigit(const uint8_t disp_idx, uint8_t next_digit) {
  if (disp_idx >= NUM_DISPLAY) {
    return;
  }
  const uint8_t curr_digit = g_ncv_current_digit[disp_idx];
  #define DIGIT_CFG_ARRAY_LEN 12
  // If g_ncv_next_digit[disp_idx] is a valid value, this means we are currently
  // working on setting that value and will not change to a different value (ie.
  // a new next_digit) until g_ncv_next_digit[disp_idx] is not a valid value. On
  // boot up, all displays are initialized to digit 'n' (see above notes).
  if (g_ncv_next_digit[disp_idx] < DIGIT_CFG_ARRAY_LEN) {
    next_digit = g_ncv_next_digit[disp_idx];
  } else {
    if (next_digit < DIGIT_CFG_ARRAY_LEN) {
      g_ncv_next_digit[disp_idx] = next_digit;
    } else {
      return;
    }
  }
  if (curr_digit == next_digit) {
    return;
  }
  #define NUM_SEGMENT 7
  const uint32_t seg_mask[NUM_SEGMENT] =
  //          ae    AE0   fdgxbcFDGXBC0
    {0b00000000000001000000000000000000,  // A
     0b00000000000000000000000000000100,  // B
     0b00000000000000000000000000000010,  // C
     0b00000000000000000000000000100000,  // D
     0b00000000000000100000000000000000,  // E
     0b00000000000000000000000001000000,  // F
     0b00000000000000000000000000010000}; // G
  const uint32_t digit_cfg[DIGIT_CFG_ARRAY_LEN] =
  //          ae    AE0   fdgxbcFDGXBC0
    {0b00000000000001100000000001100110,  // 0
     0b00000000000000000000000000000110,  // 1
     0b00000000000001100000000000110100,  // 2
     0b00000000000001000000000000110110,  // 3
     0b00000000000000000000000001010110,  // 4
     0b00000000000001000000000001110010,  // 5
     0b00000000000001100000000001110010,  // 6
     0b00000000000001000000000000000110,  // 7
     0b00000000000001100000000001110110,  // 8
     0b00000000000001000000000001110110,  // 9
     0b00000000000001100000000001110110,  // all
     0b00000000000000000000000000000000}; // none
                                        //       ae    AE0   fdgxbcFDGXBC0
  const uint32_t reset_pin_bit_cfg_on = 0b00000000000000000000000000001000;  // X
  const uint32_t reset_pin_bit_en =  reset_pin_bit_cfg_on << 6;  // x
  // Also use digit_xor to specify which half-bridges need to be enabled
  const uint32_t digit_xor = digit_cfg[curr_digit] ^ digit_cfg[next_digit];
  const uint32_t all_bits_to_turn_on = digit_cfg[next_digit] & digit_xor,
                 all_bits_to_turn_off = digit_cfg[curr_digit] & digit_xor;
  // We can only turn on or off up to two segments at a time so create
  // incremental masks to go from the previous digit to the next one.
  const uint8_t MAX_SIMULTANEOUS_SEG = 2;
  uint32_t bits_to_turn_on[8] = {0},
           bits_to_turn_off[8] = {0};
  uint8_t bits_to_turn_on_len = 0,
          bits_to_turn_off_len = 0;
  uint8_t i, j, k;
  for (i = 0, j = 0, k = 0; i < NUM_SEGMENT; ++i) {
    if (all_bits_to_turn_on & seg_mask[i]) {
      bits_to_turn_on[bits_to_turn_on_len] |= seg_mask[i];
      ++j;
      if (j == MAX_SIMULTANEOUS_SEG) {
        j = 0;
        ++bits_to_turn_on_len;
      }
    } else if (all_bits_to_turn_off & seg_mask[i]) {
      bits_to_turn_off[bits_to_turn_off_len] |= seg_mask[i];
      ++k;
      if (k == MAX_SIMULTANEOUS_SEG) {
        ++bits_to_turn_off_len;
        k = 0;
      }
    }
  }
  if (j % MAX_SIMULTANEOUS_SEG != 0) {
    ++bits_to_turn_on_len;
  }
  if (k % MAX_SIMULTANEOUS_SEG != 0) {
    ++bits_to_turn_off_len;
  }
  TxDataPush(ENABLE_NCV_DRV, disp_idx);
  for (i = 0; i < bits_to_turn_on_len; ++i) {
    const uint32_t bits_to_turn_on_en = bits_to_turn_on[i] << 6;
    TxDataPush(bits_to_turn_on[i] | bits_to_turn_on_en | reset_pin_bit_en,
               disp_idx);
  }
  for (i = 0; i < bits_to_turn_off_len; ++i) {
    const uint32_t bits_to_turn_off_en = bits_to_turn_off[i] << 6;
    TxDataPush(bits_to_turn_off_en | reset_pin_bit_cfg_on | reset_pin_bit_en,
               disp_idx);
  }
  TxDataPush(0, disp_idx);  // Disable all the outputs 
  TxDataPush(DISABLE_NCV_DRV, disp_idx);
  g_ncv_current_digit[disp_idx] = next_digit;
  g_ncv_next_digit[disp_idx] = DIGIT_CFG_ARRAY_LEN;
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
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);  // 2000 Hz interupt
  if (!LL_SPI_IsEnabled(SPI1)) {
    LL_SPI_Enable(SPI1);
  }
  // g_tim_transmit_spi frequency is 2000 Hz
  const uint32_t LED_ON_DURATION = 100,  // 50 ms
                 LED_BLINK_PERIOD = 500;  // 0.250 sec
  const uint32_t LED_ON_START_COUNT = LED_BLINK_PERIOD - LED_ON_DURATION;
  uint32_t led_blink_count = 0;
  bool first_tim_call_set_digit = true;
  uint8_t disp_idx = 0,
          next_digit[NUM_DISPLAY] = {9, 9, 9, 9};
  bool is_init = false;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // Send test SPI message (this also makes the clock default state low, which
  // is good for the doing logic analyzer work).
  LL_SPI_TransmitData16(SPI1, 0xBEEF);
  RESET_GPIO(SPI1_MOSI_GPIO_Port, SPI1_MOSI_Pin)
  HAL_Delay(10);
  int i;
  for (i = 0; i < NUM_DISPLAY; ++i) {
    SetDigit(i, 10);
  }
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (g_tim_transmit_spi) {
      g_tim_transmit_spi = false;
      if (first_tim_call_set_digit) {
        first_tim_call_set_digit = false;
      } else {
        SpiTransmit32();
      }
      ++led_blink_count;
    }
    // Blink Status LED
    if (led_blink_count == LED_ON_START_COUNT) {
      SET_GPIO(LED_GREEN_GPIO_Port, LED_GREEN_Pin)
    } else if (led_blink_count == LED_BLINK_PERIOD) {
      RESET_GPIO(LED_GREEN_GPIO_Port, LED_GREEN_Pin)
      led_blink_count = 0;
      if (!is_init) {
        is_init = true;
        for (i = 0; i < NUM_DISPLAY; ++i) {
          SetDigit(i, 11);
        }
      } else {
        next_digit[disp_idx] += 1;
        if (next_digit[disp_idx] > 9) {
          next_digit[disp_idx] = 0;
        }
        SetDigit(disp_idx, next_digit[disp_idx]);
        ++disp_idx;
        if (disp_idx >= NUM_DISPLAY) {
          disp_idx = 0;
        }
      }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
