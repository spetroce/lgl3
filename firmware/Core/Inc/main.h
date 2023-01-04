/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"

#include "stm32g4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NCV_EN_4_Pin GPIO_PIN_0
#define NCV_EN_4_GPIO_Port GPIOA
#define NCV_EN_3_Pin GPIO_PIN_1
#define NCV_EN_3_GPIO_Port GPIOA
#define NCV_EN_2_Pin GPIO_PIN_2
#define NCV_EN_2_GPIO_Port GPIOA
#define HOUR_DOWN_Pin GPIO_PIN_3
#define HOUR_DOWN_GPIO_Port GPIOA
#define MIN_UP_Pin GPIO_PIN_4
#define MIN_UP_GPIO_Port GPIOA
#define HOUR_UP_Pin GPIO_PIN_0
#define HOUR_UP_GPIO_Port GPIOB
#define MIN_DOWN_Pin GPIO_PIN_1
#define MIN_DOWN_GPIO_Port GPIOB
#define NCV_EN_1_Pin GPIO_PIN_2
#define NCV_EN_1_GPIO_Port GPIOB
#define SPI1_CS_1_Pin GPIO_PIN_10
#define SPI1_CS_1_GPIO_Port GPIOB
#define SPI1_CS_2_Pin GPIO_PIN_11
#define SPI1_CS_2_GPIO_Port GPIOB
#define SPI1_CS_3_Pin GPIO_PIN_12
#define SPI1_CS_3_GPIO_Port GPIOB
#define SPI1_CS_4_Pin GPIO_PIN_13
#define SPI1_CS_4_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_5
#define LED_GREEN_GPIO_Port GPIOB
#define HW_REV_1_Pin GPIO_PIN_7
#define HW_REV_1_GPIO_Port GPIOB
#define HW_REV_2_Pin GPIO_PIN_9
#define HW_REV_2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
