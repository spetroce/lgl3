/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "rtc.h"
#include "spi.h"
#include "tim.h"
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
#define NUM_BUTTON 4

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
uint8_t g_disp_idx[TX_DATA_MAX_LEN],
        g_button_sum[NUM_BUTTON] = {0},
        g_button_scan_start_count[NUM_BUTTON] = {0},
        g_button_scan_repeat_count[NUM_BUTTON] = {0};
uint32_t g_tx_data[TX_DATA_MAX_LEN],
         g_tx_data_write_idx = 0,
         g_tx_data_read_idx = 0,
         g_tx_data_len = 0;
bool g_button_last_state[NUM_BUTTON] = {false},
     g_button_state[NUM_BUTTON] = {false},
     g_button_scan[NUM_BUTTON] = {false};
// g_current_digit is used by SetDigit() to know what segments to turn on/off to
// get to the next digit. On boot we assume that all the segments are not
// showing by setting g_current_digit to have the index value 11 (hide all
// segments). The user needs to call SetDigit(x, 10) for all displays on startup
// to get the displays in a known configuration, where all segments are shown.
uint8_t g_current_digit[NUM_DISPLAY] = {11, 11, 11, 11};
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
  RESET_GPIO(GPIOA, LL_GPIO_PIN_7)  // TODO: Is this necessary?
  // Wait for transmit to complete
  while (SPI1->SR & SPI_SR_BSY);  // See LL_SPI_IsActiveFlag_BSY()
  SET_GPIO(spi_cs_gpio_port[disp_idx], spi_cs_gpio_pin[disp_idx])
  DelayUs(5);  // Need minimum 5 microsecond wait here between CSB toggle.
  // Send second word over SPI (bits 16-31 of tx_data; NCV7719_HBSEL = 1)
  RESET_GPIO(spi_cs_gpio_port[disp_idx], spi_cs_gpio_pin[disp_idx])
  LL_SPI_TransmitData16(SPI1, tx_data_word[1]);
  RESET_GPIO(GPIOA, LL_GPIO_PIN_7)  // TODO: Is this necessary?
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
  #define DIGIT_CFG_ARRAY_LEN 12
  if (next_digit >= DIGIT_CFG_ARRAY_LEN) {
    return;
  }
  const uint8_t curr_digit = g_current_digit[disp_idx];
  if (next_digit == curr_digit) {
    return;
  } else {
    // We basically assume that the user would not call SetDigit() so fast that
    // the following would not be true.
    g_current_digit[disp_idx] = next_digit;
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
}


// Call at 100 Hz
void UpdateButtonState() {
  const uint8_t MAX_SUM = 5,  // 50 milliseconds
                BUTTON_SCAN_START = 50,  // 500 milliseconds
                BUTTON_SCAN_REPEAT = 15;  // 150 milliseconds
  GPIO_TypeDef * button_gpio_port[NUM_BUTTON] = {HOUR_UP_GPIO_Port,
                                                 HOUR_DOWN_GPIO_Port,
                                                 MIN_UP_GPIO_Port,
                                                 MIN_DOWN_GPIO_Port};
  uint16_t button_gpio_pin[NUM_BUTTON] = {HOUR_UP_Pin,
                                          HOUR_DOWN_Pin,
                                          MIN_UP_Pin,
                                          MIN_DOWN_Pin};
  int i = 0;
  for (i = 0; i < NUM_BUTTON; ++i) {
    // High is button released, Low is button pressed.
    if (READ_GPIO(button_gpio_port[i], button_gpio_pin[i])) {
      if (g_button_sum[i] > 0) {
        g_button_sum[i] -= 1;
      }
      if (g_button_sum[i] == 0) {
        g_button_last_state[i] = false;
        g_button_scan[i] = false;
        g_button_scan_start_count[i] = 0;
        g_button_scan_repeat_count[i] = 0;
      }
    } else {
      if (g_button_sum[i] < MAX_SUM) {
        g_button_sum[i] += 1;
      }
      // Besides the first button press, g_button_sum[] must hit zero for the
      // button state to be true again.
      if (g_button_sum[i] == MAX_SUM) {
        if (g_button_scan[i]) {
          g_button_scan_repeat_count[i] += 1;
        }
        if (!g_button_last_state[i] ||
            (g_button_scan[i] &&
             g_button_scan_repeat_count[i] == BUTTON_SCAN_REPEAT)) {
          g_button_scan_repeat_count[i] = 0;
          g_button_last_state[i] = true;
          g_button_state[i] = true;
        }
        if (g_button_scan_start_count[i] < BUTTON_SCAN_START) {
          g_button_scan_start_count[i] += 1;
          if (g_button_scan_start_count[i] == BUTTON_SCAN_START) {
            g_button_scan[i] = true;
            g_button_state[i] = true;
          }
        }
      }
    }
  }
}


bool ButtonState(const uint8_t button_index) {
  if (button_index >= NUM_BUTTON) {
    return false;
  }
  if (g_button_state[button_index]) {
    g_button_state[button_index] = false;
    return true;
  }
  return false;
}


void SetRtcTime(const uint8_t hour,
                const uint8_t minute,
                const uint8_t second) {
  RTC_TimeTypeDef time;
  time.Hours = hour;
  time.Minutes = minute;
  time.Seconds = second;
  time.SubSeconds = 0;
  time.TimeFormat = RTC_HOURFORMAT_24;
  time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  time.StoreOperation = RTC_STOREOPERATION_RESET;
  HAL_RTC_SetTime(&hrtc, &time, FORMAT_BIN);
  RTC_DateTypeDef date;
  date.WeekDay = RTC_WEEKDAY_MONDAY;
  date.Month = RTC_MONTH_JANUARY;
  date.Date = 1;
  date.Year = 1;
  HAL_RTC_SetDate(&hrtc, &date, FORMAT_BCD);

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
  MX_RTC_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);  // DelayUs() timer
  HAL_TIM_Base_Start_IT(&htim6);  // 2000 Hz interupt
  if (!LL_SPI_IsEnabled(SPI1)) {
    LL_SPI_Enable(SPI1);
  }
  bool first_tim_call_set_digit = true;
  uint8_t disp_idx = 0,
          next_digit[NUM_DISPLAY] = {0, 0, 0, 0};
  uint32_t hour = 0,
           minute = 0,
           seconds = 0;
  // g_tim_transmit_spi frequency is 2000 Hz
  const uint32_t INIT_PERIOD = 125;  // 0.125 second
  bool is_init = false;
  uint32_t init_step = 0,
           init_period_count = 0,
           update_button_count = 0;
  const uint32_t LED_ON_DURATION = 100;  // 50 ms
  uint32_t led_blink_count = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // Send test SPI message (this also makes the clock default state low, which
  // is good for the doing logic analyzer work).
  LL_SPI_TransmitData16(SPI1, 0xBEEF);
  RESET_GPIO(GPIOA, LL_GPIO_PIN_7)  // TODO: Is this necessary?
  HAL_Delay(10);
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
      ++init_period_count;
      ++led_blink_count;
      ++update_button_count;
      if (update_button_count >= 20) {
        update_button_count = 0;
        UpdateButtonState();  // Runs at 100 Hz
      }
    }
    // Run Init Routine
    if (!is_init && init_period_count == INIT_PERIOD) {
      init_period_count = 0;
      int i;
      if (init_step == 0) {
        // Show all segments - Calling SetDigit(x, 10) on boot needs to be done
        // to properly initialize SetDigit() and the displays.
        for (i = 0; i < NUM_DISPLAY; ++i) {
          SetDigit(i, 10);
        }
      } else if (init_step == 16) {
        // Hide all segments
        for (i = 0; i < NUM_DISPLAY; ++i) {
          SetDigit(i, 11);
        }
      } else if (init_step >= 32) {
        // Cycle through all numbers and then stop with all digits on zero
        if (disp_idx == 0 && next_digit[disp_idx] == 10) {
          for (i = 0; i < NUM_DISPLAY; ++i) {
            next_digit[i] = 0;
            SetDigit(i, next_digit[i]);
          }
          is_init = true;
          SetRtcTime(0, 0, 0);
          continue;
        }
        SetDigit(disp_idx, next_digit[disp_idx]);
        next_digit[disp_idx] += 1;
        ++disp_idx;
        if (disp_idx >= NUM_DISPLAY) {
          disp_idx = 0;
        }
      }
      ++init_step;
    } else if (is_init) {
// #define BUTTON_TEST
#ifdef BUTTON_TEST
      int i;
      for (i = 0; i < NUM_DISPLAY; ++i) {
        if (ButtonState(i)) {
          next_digit[i] += 1;
          if (next_digit[i] > 9) {
            next_digit[i] = 0;
          }
          SetDigit(i, next_digit[i]);
        }
      }
#else
      bool update_rtc_time = false;
      if (ButtonState(0)) {
        if (hour >= 23) {
          hour = 0;
        } else {
          ++hour;
        }
        update_rtc_time = true;
      } else if (ButtonState(1)) {
        if (hour == 0) {
          hour = 23;
        } else {
          --hour;
        }
        update_rtc_time = true;
      }
      if (ButtonState(2)) {
        if (minute >= 59) {
          minute = 0;
        } else {
          ++minute;
        }
        update_rtc_time = true;
      } else if (ButtonState(3)) {
        if (minute == 0) {
          minute = 59;
        } else {
          --minute;
        }
        update_rtc_time = true;
      }
      if (update_rtc_time) {
        SetRtcTime(hour, minute, 0);
      }
      RTC_DateTypeDef rtc_date;
      RTC_TimeTypeDef rtc_time;
      HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
      HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);
      hour = rtc_time.Hours;
      minute = rtc_time.Minutes;
      if (seconds != rtc_time.Seconds) {
        seconds = rtc_time.Seconds;
        led_blink_count = 0;
      }
      SetDigit(0, rtc_time.Hours / 10 % 10);
      SetDigit(1, rtc_time.Hours % 10);
      SetDigit(2, rtc_time.Minutes /  10 % 10);
      SetDigit(3, rtc_time.Minutes % 10);
#endif
    }
    // Blink Status LED
    if (led_blink_count == 0) {
      SET_GPIO(LED_GREEN_GPIO_Port, LED_GREEN_Pin)
    } else if (led_blink_count == LED_ON_DURATION) {
      RESET_GPIO(LED_GREEN_GPIO_Port, LED_GREEN_Pin)
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCCEx_EnableLSCO(RCC_LSCOSOURCE_LSE);
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
